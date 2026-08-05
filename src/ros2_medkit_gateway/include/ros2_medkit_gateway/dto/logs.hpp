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

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// LogContext - "context" sub-object inside LogEntry.
//
// Wire shape (from log_entry_schema() in schema_builder.cpp):
//   node     - ROS 2 node name (required)
//   function - optional calling function name
//   file     - optional source file name
//   line     - optional source line number (integer)
// =============================================================================
struct LogContext {
  std::string node;
  std::optional<std::string> function;
  std::optional<std::string> file;
  std::optional<int64_t> line;
};

template <>
inline constexpr auto dto_fields<LogContext> =
    std::make_tuple(field("node", &LogContext::node), field("function", &LogContext::function),
                    field("file", &LogContext::file), field("line", &LogContext::line));

template <>
inline constexpr std::string_view dto_name<LogContext> = "LogContext";

// =============================================================================
// LogEntry - single application log entry.
//
// Wire shape (from log_entry_schema() in schema_builder.cpp):
//   id        - log entry ID, e.g. "log_123" (required)
//   timestamp - ISO 8601 date-time string (required)
//   severity  - log level string (required)
//   message   - log message text (required)
//   context   - optional source context sub-object (LogContext)
//
// severity is NOT field_enum here: the log_mgr produces raw JSON items and the
// handler uses those items as-is (no bespoke severity validation on responses).
// =============================================================================
struct LogEntry {
  std::string id;
  std::string timestamp;
  std::string severity;
  std::string message;
  std::optional<LogContext> context;
};

template <>
inline constexpr auto dto_fields<LogEntry> =
    std::make_tuple(field("id", &LogEntry::id), field("timestamp", &LogEntry::timestamp),
                    field("severity", &LogEntry::severity), field("message", &LogEntry::message),
                    field("context", &LogEntry::context));

template <>
inline constexpr std::string_view dto_name<LogEntry> = "LogEntry";

// =============================================================================
// LogListXMedkit - typed x-medkit vendor extension on log list responses.
//
// Emitted by handle_get_logs on FUNCTION / AREA / COMPONENT / APP entities.
//
//   entity_id           - SOVD entity ID (all aggregating entity types)
//   aggregation_level   - "function"|"area"|"component" (aggregating entities)
//   aggregated          - true when log entries aggregated from multiple sources
//   host_count          - number of host apps resolved (FUNCTION only)
//   component_count     - number of components in the area (AREA only)
//   app_count           - number of apps resolved (AREA and COMPONENT)
//   aggregation_sources - list of node FQNs that contributed log entries
//   contributors        - aggregation peer provenance list (peer fan-out)
//   partial             - true when a peer fan-out request failed
//   failed_peers        - list of peer addresses that returned errors
//   peer_failures       - why each of those peers failed (timeout, unreachable, ...)
//   peer_dropped_items  - per-peer items dropped due to malformed JSON
//                         (observability for invisible drift)
//
// All fields are optional so the APP branch (which only emits x-medkit when
// there are peer contributors) and empty aggregation results are handled
// without special-casing.
// =============================================================================
struct LogListXMedkit {
  std::optional<std::string> entity_id;
  std::optional<std::string> aggregation_level;  // enum: "function"|"area"|"component"
  std::optional<bool> aggregated;
  std::optional<int64_t> host_count;
  std::optional<int64_t> component_count;
  std::optional<int64_t> app_count;
  std::optional<std::vector<std::string>> aggregation_sources;
  std::optional<std::vector<std::string>> contributors;
  std::optional<bool> partial;
  std::optional<std::vector<std::string>> failed_peers;
  /// Why each peer in `failed_peers` contributed nothing. Sibling rather than a
  /// widening of `failed_peers`, whose wire shape (a list of names) every
  /// existing client already reads.
  std::optional<std::vector<PeerFailure>> peer_failures;
  std::optional<std::vector<DroppedItem>> peer_dropped_items;
};

template <>
inline constexpr auto dto_fields<LogListXMedkit> = std::make_tuple(
    field("entity_id", &LogListXMedkit::entity_id),
    field_enum("aggregation_level", &LogListXMedkit::aggregation_level, kLogAggregationLevelValues),
    field("aggregated", &LogListXMedkit::aggregated), field("host_count", &LogListXMedkit::host_count),
    field("component_count", &LogListXMedkit::component_count), field("app_count", &LogListXMedkit::app_count),
    field("aggregation_sources", &LogListXMedkit::aggregation_sources),
    field("contributors", &LogListXMedkit::contributors), field("partial", &LogListXMedkit::partial),
    field("failed_peers", &LogListXMedkit::failed_peers), field("peer_failures", &LogListXMedkit::peer_failures),
    field("peer_dropped_items", &LogListXMedkit::peer_dropped_items));

template <>
inline constexpr std::string_view dto_name<LogListXMedkit> = "LogListXMedkit";

// =============================================================================
// Collection<LogEntry, LogListXMedkit> - typed list shape returned by the
// /{entity}/logs handler.
//
// Same wire shape as Collection<LogEntry> (`LogEntryList` schema), but the
// `x-medkit` payload carries the rich `LogListXMedkit` fields (entity_id,
// aggregation_level, aggregated, host_count, partial, failed_peers,
// peer_dropped_items, ...) instead of the generic `XMedkitCollection`. The
// schema name is kept identical to the legacy `LogEntryList` $ref so existing
// OpenAPI clients are not affected; the difference is purely server-side
// typing.
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<LogEntry, LogListXMedkit>> = "LogEntryList";

// =============================================================================
// LogConfiguration - GET response and PUT request body for /{entity}/logs/configuration.
//
// Wire shape (from log_configuration_schema() in schema_builder.cpp):
//   severity_filter - log level filter string (optional)
//   max_entries     - maximum number of entries ONE GET /logs answers with
//                     (optional, 1..10000)
//
// Both settings filter the answer, not the buffer: LogManager::on_log_entry
// consults neither, so nothing is dropped on account of them, and the ring
// buffer's own size is the separate max_buffer_size_ constructor argument.
// (Ingestion does drop entries for its own reasons - ring-buffer overflow, the
// distinct-node cap, a LogProvider observer claiming the entry - none of which
// these two settings influence.) See the note on LogConfig in
// core/log_types.hpp: SOVD reads max_entries as a storage limit, and this
// implementation deliberately does not.
//
// severity_filter uses plain field() (not field_enum) because handle_put_logs_configuration
// performs its own bespoke validation of severity via log_mgr->update_config(), which
// produces specific ERR_INVALID_PARAMETER errors. parse_body uses field() to allow
// any string value through; the handler's richer validation runs after parsing.
// =============================================================================
struct LogConfiguration {
  std::optional<std::string> severity_filter;
  std::optional<int64_t> max_entries;
};

template <>
inline constexpr auto dto_fields<LogConfiguration> = std::make_tuple(
    field("severity_filter", &LogConfiguration::severity_filter,
          "Lowest severity a `GET /{entity}/logs` answers with: `debug`, `info`, `warning`, `error` or `fatal`. "
          "Like `max_entries` this filters the answer, not the buffer - nothing is discarded on account of it, "
          "so raising it hides quieter entries from subsequent reads and lowering it brings them back. The "
          "effective floor is the stricter of this and the request's own `severity` parameter, so a lenient "
          "query cannot see past a strict configuration. A registered LogProvider plugin serves the query "
          "itself and this setting does not apply to it."),
    field("max_entries", &LogConfiguration::max_entries,
          "Ceiling on how many entries one `GET /{entity}/logs` answers with. A match longer than this is cut "
          "down to the most recent `max_entries` and the older ones are dropped from the answer - silently, with "
          "nothing on the response saying so and no way to page past it. Accepted range is 1 to 10000. It caps "
          "the answer, not the gateway's log buffer, whose size is a separate start-up setting. A registered "
          "LogProvider plugin serves the query itself and this setting does not apply to it.",
          FieldConstraints{/*minimum=*/1.0, /*maximum=*/10000.0, {}, {}, {}}));

template <>
inline constexpr std::string_view dto_name<LogConfiguration> = "LogConfiguration";

// =============================================================================
// LogQuery - query parameters for GET /{entity}/logs. Read by the handler via
// TypedRequest::query<LogQuery>() and declared in the OpenAPI spec via the same
// descriptor, so the two cannot drift.
// =============================================================================
struct LogQuery {
  std::optional<std::string> severity;
  std::optional<std::string> context;
};

template <>
inline constexpr auto dto_fields<LogQuery> =
    std::make_tuple(field_enum("severity", &LogQuery::severity, kLogSeverityFilterValues,
                               "Filter by minimum severity: debug, info, warning, error, or fatal"),
                    field("context", &LogQuery::context, "Filter by logger context substring (max 256 chars)"));

}  // namespace dto
}  // namespace ros2_medkit_gateway
