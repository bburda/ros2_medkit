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
#include <set>
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

std::string rosbag_recording_id(const std::string & file_path) {
  std::filesystem::path p(file_path);
  if (!p.has_filename()) {
    p = p.parent_path();  // tolerate a trailing slash
  }
  return p.filename().string();
}

std::vector<std::string> rosbag_attached_fault_codes(const nlohmann::json & rosbag_data,
                                                     const std::string & requested_id) {
  if (rosbag_data.contains("fault_codes") && rosbag_data["fault_codes"].is_array()) {
    // Element by element rather than get<vector<string>>(): the array is built
    // from a ROS string vector today and cannot hold anything else, but this
    // function exists to tolerate a peer that predates the field, and a
    // whole-array conversion throws on the first non-string it meets.
    std::vector<std::string> codes;
    for (const auto & code : rosbag_data["fault_codes"]) {
      if (code.is_string()) {
        codes.push_back(code.get<std::string>());
      }
    }
    if (!codes.empty()) {
      return codes;
    }
  }
  // No list on the wire: a peer that predates the field, where the addressed id
  // was the fault code itself. Authorizing against it reproduces the old check.
  return {requested_id};
}

bool rosbag_resolved_by_fault_code(const nlohmann::json & rosbag_data, const std::string & requested_id) {
  const std::string resolved = rosbag_data.value("recording_id", "");
  // An absent id means a peer that predates the field; it answers by fault code
  // only, and the attached-codes fallback already reduces to the old check there.
  return !resolved.empty() && resolved != requested_id;
}

std::vector<dto::BulkDataDescriptor>
fold_rosbag_rows_into_descriptors(const std::vector<nlohmann::json> & rows,
                                  const std::unordered_map<std::string, nlohmann::json> & faults_by_code) {
  struct RecordingEntry {
    std::string recording_id;
    std::string format;
    uint64_t size_bytes{0};
    double duration_sec{0.0};
    int64_t created_at_ns{0};
    std::vector<std::string> fault_codes;
  };
  std::vector<RecordingEntry> recordings;
  std::unordered_map<std::string, size_t> index_by_recording;

  for (const auto & row : rows) {
    // The fault manager supplies the recording id; falling back to the path
    // basename covers a peer or a replay predating that field.
    std::string recording_id = row.value("recording_id", "");
    if (recording_id.empty()) {
      recording_id = rosbag_recording_id(row.value("file_path", ""));
    }
    if (recording_id.empty()) {
      continue;  // nothing addressable - a row with neither id nor path
    }

    const std::string fault_code = row.value("fault_code", "");
    // The recording's own timestamp. Taking it from the fault gave every recording
    // of one fault the same date - and this change is what lets a fault hold more
    // than one, so the date is exactly what tells the occurrences apart. An
    // acknowledged fault is missing from the default fault listing entirely, which
    // dated its recordings 1970 while the rows were still served. Fall back to the
    // fault only for a row that predates the field.
    int64_t created_at_ns = row.value("created_at_ns", int64_t{0});
    if (created_at_ns == 0) {
      if (auto it = faults_by_code.find(fault_code); it != faults_by_code.end()) {
        const double first_occurred = it->second.value("first_occurred", 0.0);
        created_at_ns = static_cast<int64_t>(first_occurred * 1'000'000'000);
      }
    }

    if (auto found = index_by_recording.find(recording_id); found != index_by_recording.end()) {
      auto & entry = recordings[found->second];
      entry.fault_codes.push_back(fault_code);
      // Rows of one burst carry the same recording timestamp, so this only decides
      // anything on the fallback path, where each row is dated by its own fault.
      if (created_at_ns != 0 && (entry.created_at_ns == 0 || created_at_ns < entry.created_at_ns)) {
        entry.created_at_ns = created_at_ns;
      }
      continue;
    }

    RecordingEntry entry;
    entry.recording_id = recording_id;
    // Default to sqlite3 (the historical FaultManager default) when a bag predates
    // the persisted format field; the per-bag metadata normally carries the real one.
    entry.format = row.value("format", "sqlite3");
    entry.size_bytes = row.value("size_bytes", uint64_t{0});
    entry.duration_sec = row.value("duration_sec", 0.0);
    entry.created_at_ns = created_at_ns;
    entry.fault_codes.push_back(fault_code);
    index_by_recording.emplace(recording_id, recordings.size());
    recordings.push_back(std::move(entry));
  }

  std::vector<dto::BulkDataDescriptor> descriptors;
  descriptors.reserve(recordings.size());
  for (auto & entry : recordings) {
    std::sort(entry.fault_codes.begin(), entry.fault_codes.end());
    entry.fault_codes.erase(std::unique(entry.fault_codes.begin(), entry.fault_codes.end()), entry.fault_codes.end());

    dto::BulkDataDescriptor descriptor;
    descriptor.id = entry.recording_id;
    descriptor.name = entry.recording_id + " recording " + format_timestamp_ns(entry.created_at_ns);
    descriptor.mimetype = BulkDataHandlers::get_rosbag_mimetype(entry.format);
    descriptor.size = entry.size_bytes;
    descriptor.creation_date = format_timestamp_ns(entry.created_at_ns);
    descriptor.x_medkit = nlohmann::json{{"fault_codes", entry.fault_codes},
                                         {"duration_sec", entry.duration_sec},
                                         {"format", entry.format},
                                         // Redundant with the descriptor id, kept because
                                         // clients already group on it.
                                         {"recording_id", entry.recording_id}};
    descriptors.push_back(std::move(descriptor));
  }
  return descriptors;
}

std::vector<std::string> compute_bulkdata_source_filters(const ThreadSafeEntityCache & cache,
                                                         const EntityInfo & entity) {
  // One rule for every entity type: the same fault-scope resolution that drives
  // GET /{entity}/faults (external app -> bare id, external component also owns
  // its own id, AREA recurses subareas, FUNCTION follows component hosts).
  // Rosbags are keyed by exact reporting source, so any private variant here
  // makes a fault the entity lists advertise a bag that 404s - e.g. an
  // area-scoped bulk_data_uri resolved through the area namespace never
  // matched a bag stored under a hosted app's bare id.
  auto sources = HandlerContext::resolve_entity_source_fqns(cache, entity);
  if (!sources.empty()) {
    return {sources.begin(), sources.end()};
  }

  if (entity.type == EntityType::FUNCTION) {
    // Functions are pure aggregated views over hosted apps - if no apps host
    // the function, there is nothing to query. No fall-through.
    return {};
  }

  // No resolvable hosted sources (manifest-only entities grouping topics rather
  // than nodes, or apps missing from the cache): fall back to FQN or namespace_path.
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

    dto::Collection<dto::BulkDataDescriptor> rosbag_response;
    rosbag_response.items = detail::fold_rosbag_rows_into_descriptors(all_rosbags, fault_map);
    return rosbag_response;
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
    //
    // The id is a recording id. A pre-#620 URL carrying a fault code still resolves:
    // the fault manager tries the recording first and falls back to "the newest
    // recording of this fault", which is what a fault-code URL has always returned.
    // Both routes end up holding a recording before anything is authorized, so there
    // is exactly one authorization semantic, not two.
    auto fault_mgr = ctx_.node()->get_fault_manager();

    auto rosbag_result = fault_mgr->get_rosbag(bulk_data_id);
    if (!rosbag_result.success || !rosbag_result.data.contains("file_path")) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found", json{{"bulk_data_id", bulk_data_id}}));
    }

    // Security check: the bag belongs to this entity when ANY fault it was captured
    // for is within the entity's source scope. Union rather than a single code
    // because a burst shares one recording, and each of those faults already had its
    // own downloadable copy of it before - so this grants nothing new, it only
    // renames the door. Tested with the shared boundary-aware matcher: the
    // transport's get_fault(code, source) check is a raw prefix match, so app id
    // "plc" would otherwise claim the assets of "plc_line1".
    auto source_filters = get_source_filters(entity);
    std::set<std::string> scope(source_filters.begin(), source_filters.end());

    // The compatibility path needs the REQUESTED code in scope, not just some code
    // the recording is attached to. A burst shares one bag, so authorizing on the
    // union alone answers 200 for a fault code this entity does not own - the bytes
    // are ones it could already fetch under its own code, but the 200 itself tells
    // the caller that another fault shares its recording. build_sovd_fault_response
    // refuses to mix sources for the same reason. When the id really was a recording
    // id there is no requested code and the union is the whole answer.
    if (detail::rosbag_resolved_by_fault_code(rosbag_result.data, bulk_data_id)) {
      auto requested = fault_mgr->get_fault(bulk_data_id, "");
      if (!requested.success || !faults::fault_in_source_scope(requested.data, scope)) {
        return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found for this entity",
                                         json{{"entity_id", path_info->entity_id}}));
      }
    }

    const auto attached_codes = detail::rosbag_attached_fault_codes(rosbag_result.data, bulk_data_id);

    const bool authorized = std::any_of(attached_codes.begin(), attached_codes.end(), [&](const std::string & code) {
      auto fault_result = fault_mgr->get_fault(code, "");
      return fault_result.success && faults::fault_in_source_scope(fault_result.data, scope);
    });
    if (!authorized) {
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found for this entity",
                                       json{{"entity_id", path_info->entity_id}}));
    }

    std::string file_path = rosbag_result.data["file_path"].get<std::string>();
    // Default to sqlite3 (the historical FaultManager default) for bags predating the
    // format field; metadata normally carries the real one persisted at capture time.
    std::string format = rosbag_result.data.value("format", "sqlite3");
    mimetype = get_rosbag_mimetype(format);
    // Named after the recording that was actually served, which for a compatibility
    // URL is not the segment the client sent.
    filename = rosbag_result.data.value("recording_id", bulk_data_id) + "." + format;

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
