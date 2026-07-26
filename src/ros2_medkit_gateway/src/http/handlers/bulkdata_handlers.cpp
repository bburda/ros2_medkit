// Copyright 2026 bburda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"

#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/http/entity_path_utils.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/bulk_data_store.hpp"
#include "ros2_medkit_gateway/dto/bulkdata.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Resolve the entity_id from the typed request. Bulk-data routes embed the
/// entity reference in the URL path; the registered route patterns capture
/// the entity id as group 1 (single-entity) or group 2 (nested subarea /
/// subcomponent). `parse_entity_path` walks the registered regex catalogue
/// and yields a normalised `EntityPathInfo` so both shapes resolve through
/// one helper. Returning an `ErrorInfo` keeps the failure surface aligned
/// with the legacy handler's "Invalid entity path" 400.
tl::expected<EntityPathInfo, ErrorInfo> parse_path(const http::TypedRequest & req) {
  auto info = parse_entity_path(req.path());
  if (!info) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid entity path"));
  }
  return *info;
}

}  // namespace

BulkDataHandlers::BulkDataHandlers(HandlerContext & ctx) : ctx_(ctx) {
}

std::string BulkDataHandlers::get_rosbag_mimetype(const std::string & format) {
  if (format == "mcap") {
    return "application/x-mcap";
  } else if (format == "sqlite3" || format == "db3") {
    return "application/x-sqlite3";
  }
  return "application/octet-stream";
}

std::string BulkDataHandlers::resolve_rosbag_file_path(const std::string & path) {
  // If it's a regular file, return as-is
  if (std::filesystem::is_regular_file(path)) {
    return path;
  }

  // If it's a directory (rosbag2 directory structure), find the db3/mcap file inside
  if (std::filesystem::is_directory(path)) {
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
      if (entry.is_regular_file()) {
        auto ext = entry.path().extension().string();
        // Look for db3 (sqlite3 format) or mcap files
        if (ext == ".db3" || ext == ".mcap") {
          return entry.path().string();
        }
      }
    }
  }

  return "";  // File not found
}

std::vector<std::string> BulkDataHandlers::get_source_filters(const EntityInfo & entity) const {
  return detail::compute_bulkdata_source_filters(ctx_.node()->get_thread_safe_cache(), entity);
}

namespace detail {

std::vector<std::string> compute_bulkdata_source_filters(const ThreadSafeEntityCache & cache,
                                                         const EntityInfo & entity) {
  if (entity.type == EntityType::FUNCTION) {
    // Functions are pure aggregated views over hosted apps - if no apps host the function,
    // there is nothing to query. No fall-through to fqn/namespace_path.
    return HandlerContext::resolve_app_host_fqns(cache, cache.get_apps_for_function(entity.id));
  }

  if (entity.type == EntityType::APP) {
    // A plugin-introspected app has no ROS binding, so fqn and namespace_path
    // are both empty and the bare-fqn path below would return no filter at all -
    // its rosbags and logs then belong to nobody and every download 404s. Its
    // reporting source is its bare id, exactly as fault scoping resolves it.
    std::string source = faults::resolve_app_source_fqn(cache, entity.id);
    if (!source.empty()) {
      std::vector<std::string> filters;
      filters.push_back(std::move(source));
      return filters;
    }
  }

  if (entity.type == EntityType::COMPONENT) {
    // Synthetic / runtime-discovered components have an empty fqn / namespace_path,
    // so the bare-fqn path used to silently return zero source filters and produce
    // empty descriptor lists plus failed ownership checks on download. Resolve hosted
    // apps first; manifest deployments where the component groups topics rather than
    // nodes still need the namespace prefix path, so fall through if no apps host it.
    auto filters = HandlerContext::resolve_app_host_fqns(cache, cache.get_apps_for_component(entity.id));
    // An external component reports faults under its own id too (a bridge raises
    // PLC_COMMS_LOST there), so it owns that source as well - the same rule
    // faults::collect_component_app_fqns already applies.
    auto comp = cache.get_component(entity.id);
    if (comp && comp->external.value_or(false)) {
      filters.push_back(entity.id);
    }
    if (!filters.empty()) {
      return filters;
    }
    // fall through to fqn/namespace_path
  }

  // For other entity types and manifest-only components, use FQN or namespace_path
  std::string filter = entity.fqn.empty() ? entity.namespace_path : entity.fqn;
  if (filter.empty()) {
    return {};
  }
  return {filter};
}

}  // namespace detail

// ---------------------------------------------------------------------------
// GET /{entity}/bulk-data - list categories
// ---------------------------------------------------------------------------

http::Result<dto::BulkDataCategoryList> BulkDataHandlers::list_categories(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  // Build categories list: "rosbags" always available + BulkDataStore categories.
  dto::BulkDataCategoryList response;
  response.items.push_back("rosbags");  // Always available via FaultManager

  auto * store = ctx_.bulk_data_store();
  if (store) {
    for (const auto & cat : store->list_categories()) {
      response.items.push_back(cat);
    }
  }
  return response;
}

// ---------------------------------------------------------------------------
// GET /{entity}/bulk-data/{category_id} - list descriptors
// ---------------------------------------------------------------------------

http::Result<dto::Collection<dto::BulkDataDescriptor>>
BulkDataHandlers::list_descriptors(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  auto category = extract_bulk_data_category(req.path());
  if (category.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing category"));
  }

  if (category == "rosbags") {
    // === Rosbags: served via FaultManager ===
    auto fault_mgr = ctx_.node()->get_fault_manager();

    // Get source filters for this entity (single for most, multiple for functions).
    // Functions aggregate rosbags from all hosting apps.
    auto source_filters = get_source_filters(entity);

    // Collect faults across all source filters for timestamp enrichment
    std::unordered_map<std::string, json> fault_map;
    for (const auto & source_filter : source_filters) {
      auto faults_result = fault_mgr->list_faults(source_filter);
      if (faults_result.success && faults_result.data.contains("faults")) {
        for (const auto & fault_json : faults_result.data["faults"]) {
          if (fault_json.contains("fault_code")) {
            std::string fc = fault_json["fault_code"].get<std::string>();
            fault_map[fc] = fault_json;
          }
        }
      }
    }

    // Collect rosbags across all source filters
    std::vector<json> all_rosbags;
    for (const auto & source_filter : source_filters) {
      auto rosbags_result = fault_mgr->list_rosbags(source_filter);
      if (rosbags_result.success && rosbags_result.data.contains("rosbags")) {
        for (const auto & rosbag : rosbags_result.data["rosbags"]) {
          all_rosbags.push_back(rosbag);
        }
      }
    }

    dto::Collection<dto::BulkDataDescriptor> response;
    for (const auto & rosbag : all_rosbags) {
      std::string fault_code = rosbag.value("fault_code", "");
      // Default to sqlite3 (the FaultManager default) when a bag predates the
      // persisted format field; the per-bag metadata normally carries the real one.
      std::string format = rosbag.value("format", "sqlite3");
      uint64_t size_bytes = rosbag.value("size_bytes", uint64_t{0});
      double duration_sec = rosbag.value("duration_sec", 0.0);

      // Use fault_code as bulk_data_id
      std::string bulk_data_id = fault_code;

      // Get timestamp from fault if available
      int64_t created_at_ns = 0;
      auto it = fault_map.find(fault_code);
      if (it != fault_map.end()) {
        double first_occurred = it->second.value("first_occurred", 0.0);
        created_at_ns = static_cast<int64_t>(first_occurred * 1'000'000'000);
      }

      dto::BulkDataDescriptor descriptor;
      descriptor.id = bulk_data_id;
      descriptor.name = fault_code + " recording " + format_timestamp_ns(created_at_ns);
      descriptor.mimetype = get_rosbag_mimetype(format);
      descriptor.size = size_bytes;
      descriptor.creation_date = format_timestamp_ns(created_at_ns);
      descriptor.x_medkit = json{{"fault_code", fault_code}, {"duration_sec", duration_sec}, {"format", format}};
      response.items.push_back(std::move(descriptor));
    }
    return response;
  }

  // === Non-rosbag categories: served via BulkDataStore ===
  auto * store = ctx_.bulk_data_store();
  if (!store || !store->is_known_category(category)) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Unknown category: " + category));
  }

  auto items_list = store->list_items(path_info->entity_id, category);
  dto::Collection<dto::BulkDataDescriptor> response;
  for (const auto & item : items_list) {
    dto::BulkDataDescriptor desc;
    desc.id = item.id;
    desc.name = item.name;
    desc.mimetype = item.mime_type;
    desc.size = item.size;
    desc.creation_date = item.created;
    if (!item.description.empty()) {
      desc.description = item.description;
    }
    if (!item.metadata.empty()) {
      desc.x_medkit = item.metadata;
    }
    response.items.push_back(std::move(desc));
  }
  return response;
}

// ---------------------------------------------------------------------------
// GET /{entity}/bulk-data/{category_id}/{file_id} - binary download
//
// Uses `reg.binary_download`: the framework wires the provider callback into
// cpp-httplib's range-aware content provider, sets `Content-Disposition`
// from `filename`, and propagates the typed `ErrorInfo` for failures. The
// handler stays free of `httplib::Response`.
// ---------------------------------------------------------------------------

http::Result<http::BinaryResponse> BulkDataHandlers::download(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  auto category = extract_bulk_data_category(req.path());
  auto bulk_data_id = extract_bulk_data_id(req.path());

  if (bulk_data_id.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing bulk-data ID"));
  }

  // Resolve the actual on-disk file path and the wire metadata (mimetype +
  // filename) once, branching on rosbag vs user-uploaded categories. The
  // downstream BinaryResponse assembly is identical for both branches.
  std::string actual_path;
  std::string mimetype;
  std::string filename;

  if (category == "rosbags") {
    // === Rosbags: served via FaultManager ===
    auto fault_mgr = ctx_.node()->get_fault_manager();
    std::string fault_code = bulk_data_id;

    auto rosbag_result = fault_mgr->get_rosbag(fault_code);
    if (!rosbag_result.success || !rosbag_result.data.contains("file_path")) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found", json{{"bulk_data_id", bulk_data_id}}));
    }

    // Security check: verify rosbag belongs to this entity. For functions,
    // check all hosting apps (aggregated view).
    auto source_filters = get_source_filters(entity);
    bool fault_verified = false;
    for (const auto & sf : source_filters) {
      auto fault_result = fault_mgr->get_fault(fault_code, sf);
      if (fault_result.success) {
        fault_verified = true;
        break;
      }
    }
    if (!fault_verified) {
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found for this entity",
                                       json{{"entity_id", path_info->entity_id}}));
    }

    std::string file_path = rosbag_result.data["file_path"].get<std::string>();
    // Default to sqlite3 (the FaultManager default) for bags predating the format
    // field; metadata normally carries the real one persisted at capture time.
    std::string format = rosbag_result.data.value("format", "sqlite3");
    mimetype = get_rosbag_mimetype(format);
    filename = fault_code + "." + format;

    // Rosbag2 emits a directory layout - resolve the inner db3/mcap file.
    actual_path = resolve_rosbag_file_path(file_path);
  } else {
    // === Non-rosbag categories: served via BulkDataStore ===
    auto * store = ctx_.bulk_data_store();
    if (!store || !store->is_known_category(category)) {
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Unknown category: " + category));
    }

    auto stored_path = store->get_file_path(path_info->entity_id, category, bulk_data_id);
    if (!stored_path) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found", json{{"bulk_data_id", bulk_data_id}}));
    }
    actual_path = *stored_path;

    auto item = store->get_item(path_info->entity_id, category, bulk_data_id);
    filename = item ? item->name : bulk_data_id;
    mimetype = item ? item->mime_type : "application/octet-stream";
  }

  if (actual_path.empty()) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to read bulk-data file"));
  }

  // Verify the resolved file is a readable regular file and grab its size for
  // Content-Length; if either check fails we surface the legacy 500.
  std::error_code ec;
  auto file_size = std::filesystem::file_size(actual_path, ec);
  if (ec) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to read bulk-data file"));
  }

  // Sanitise the filename for the Content-Disposition header. The framework
  // emits `attachment; filename="<sanitised>"`; embedded quotes are mapped to
  // underscores to preserve the legacy header-safety contract.
  std::string safe_name = filename;
  std::replace(safe_name.begin(), safe_name.end(), '"', '_');

  http::BinaryResponse resp;
  resp.content_type = mimetype;
  resp.filename = safe_name;
  resp.supports_ranges = true;
  resp.total_size = static_cast<uint64_t>(file_size);
  // 64 KB chunks, matching the legacy `stream_file_to_response` block size.
  static constexpr std::size_t kChunkSize = 64 * 1024;
  // Capture file path by value into the provider closure. cpp-httplib invokes
  // the provider on a worker thread for each range, so each call re-opens the
  // file rather than holding a long-lived ifstream that would race the next
  // request on the same handler instance.
  resp.provider = [path = actual_path](uint64_t offset, uint64_t length, httplib::DataSink & sink) -> bool {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
      return false;
    }
    file.seekg(static_cast<std::streamoff>(offset));
    if (!file.good()) {
      return false;
    }
    uint64_t remaining = length;
    std::vector<char> buf(std::min<uint64_t>(remaining, kChunkSize));
    while (remaining > 0 && file.good()) {
      auto to_read = static_cast<std::size_t>(std::min<uint64_t>(remaining, kChunkSize));
      file.read(buf.data(), static_cast<std::streamsize>(to_read));
      auto bytes_read = static_cast<std::size_t>(file.gcount());
      if (bytes_read == 0) {
        break;
      }
      sink.write(buf.data(), bytes_read);
      remaining -= bytes_read;
    }
    return remaining == 0;
  };
  return resp;
}

// ---------------------------------------------------------------------------
// POST /{entity}/bulk-data/{category_id} - multipart upload (201 + Location)
// ---------------------------------------------------------------------------

http::Result<std::pair<dto::BulkDataDescriptor, http::ResponseAttachments>>
BulkDataHandlers::upload(const http::TypedRequest & req, const http::MultipartBody & body) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  // Check lock access for bulk-data (typed validator returns ErrorInfo directly).
  if (auto lock_err = ctx_.validate_lock_access(req, entity, "bulk-data"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  auto category = extract_bulk_data_category(req.path());
  if (category.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing category"));
  }

  // Rosbags are managed by the fault system, not user-uploadable.
  if (category == "rosbags") {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Category 'rosbags' does not support upload. "
                                     "Rosbags are managed by the fault system."));
  }

  auto * store = ctx_.bulk_data_store();
  if (store == nullptr) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Bulk data storage not configured"));
  }

  if (!store->is_known_category(category)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Unknown bulk-data category: " + category));
  }

  // Locate the `file`, `description`, and `metadata` parts. cpp-httplib parses
  // every named part into MultipartBody.parts; walk the vector instead of
  // relying on `req.files` map ordering so the typed surface does not leak
  // through to the cpp-httplib shape.
  // FIXME(#409): httplib::MultipartFormData is unavailable in cpp-httplib
  // >= 0.20; migrate to the req.form API (tracked with the vendored-header pin).
  const httplib::MultipartFormData * file_part = nullptr;
  const httplib::MultipartFormData * description_part = nullptr;
  const httplib::MultipartFormData * metadata_part = nullptr;
  for (const auto & part : body.parts) {
    if (part.name == "file" && !file_part) {
      file_part = &part;
    } else if (part.name == "description" && !description_part) {
      description_part = &part;
    } else if (part.name == "metadata" && !metadata_part) {
      metadata_part = &part;
    }
  }

  if (!file_part) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing 'file' field in multipart/form-data request"));
  }

  std::string filename = file_part->filename.empty() ? "upload" : file_part->filename;
  std::string content_type = file_part->content_type.empty() ? "application/octet-stream" : file_part->content_type;

  // Enforce the configured maximum upload size (0 = unbounded).
  if (store->max_upload_bytes() > 0 && file_part->content.size() > store->max_upload_bytes()) {
    return tl::unexpected(make_error(413, ERR_PAYLOAD_TOO_LARGE, "File size exceeds maximum upload limit"));
  }

  std::string description;
  if (description_part) {
    description = description_part->content;
  }

  json metadata = json::object();
  if (metadata_part && !metadata_part->content.empty()) {
    auto parsed = json::parse(metadata_part->content, nullptr, false);
    if (parsed.is_discarded()) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid JSON in 'metadata' field"));
    }
    if (!parsed.is_object()) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "metadata must be a JSON object"));
    }
    metadata = std::move(parsed);
  }

  auto result =
      store->store(path_info->entity_id, category, filename, content_type, file_part->content, description, metadata);
  if (!result) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, result.error()));
  }

  const auto & stored = *result;
  dto::BulkDataDescriptor descriptor;
  descriptor.id = stored.id;
  descriptor.name = stored.name;
  descriptor.mimetype = stored.mime_type;
  descriptor.size = stored.size;
  descriptor.creation_date = stored.created;
  if (!stored.description.empty()) {
    descriptor.description = stored.description;
  }
  if (!stored.metadata.empty()) {
    descriptor.x_medkit = stored.metadata;
  }

  http::ResponseAttachments att;
  att.with_status(201).with_header("Location", req.path() + "/" + stored.id);
  return std::make_pair(std::move(descriptor), std::move(att));
}

// ---------------------------------------------------------------------------
// DELETE /{entity}/bulk-data/{category_id}/{file_id} - 204 No Content
// ---------------------------------------------------------------------------

http::Result<http::NoContent> BulkDataHandlers::remove(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "bulk-data"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  auto category = extract_bulk_data_category(req.path());
  if (category.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing category"));
  }

  // Rosbags are managed by the fault system, not user-deletable.
  if (category == "rosbags") {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Category 'rosbags' does not support deletion. "
                                     "Rosbags are managed by the fault system."));
  }

  auto item_id = extract_bulk_data_id(req.path());
  if (item_id.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing bulk-data ID"));
  }

  auto * store = ctx_.bulk_data_store();
  if (store == nullptr) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Bulk data storage not configured"));
  }

  if (!store->is_known_category(category)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Unknown bulk-data category: " + category));
  }

  auto result = store->remove(path_info->entity_id, category, item_id);
  if (!result) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data item not found"));
  }

  return http::NoContent{};
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
