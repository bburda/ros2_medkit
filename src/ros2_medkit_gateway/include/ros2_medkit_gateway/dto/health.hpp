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
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/core/http/warning_codes.hpp"
#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// HealthDiscoveryLinking - "linking" sub-object inside HealthDiscovery.
//
// Wire shape (from health_handlers.cpp handle_health):
//   linked_count      - number of runtime nodes linked to manifest apps (integer)
//   orphan_count      - number of unlinked runtime nodes (integer)
//   binding_conflicts - number of binding conflict events (integer)
//                       NOTE: the old health_schema() declared this as
//                       array<string>, but the handler always emitted size_t.
//                       The DTO matches the actual wire format.
//   unmanifested_policy - configured ManifestConfig::unmanifested_nodes value
//                       ("ignore" | "warn" | "error" | "include_as_orphan"),
//                       so a monitor can evaluate orphan_count > 0 &&
//                       unmanifested_policy == "error" without parsing prose
//   warnings          - optional list of diagnostic warning strings
// =============================================================================
struct HealthDiscoveryLinking {
  int64_t linked_count{0};
  int64_t orphan_count{0};
  int64_t binding_conflicts{0};
  std::string unmanifested_policy;
  std::optional<std::vector<std::string>> warnings;
};

template <>
inline constexpr auto dto_fields<HealthDiscoveryLinking> = std::make_tuple(
    field("linked_count", &HealthDiscoveryLinking::linked_count),
    field("orphan_count", &HealthDiscoveryLinking::orphan_count),
    field("binding_conflicts", &HealthDiscoveryLinking::binding_conflicts),
    field_enum("unmanifested_policy", &HealthDiscoveryLinking::unmanifested_policy, kUnmanifestedPolicyValues),
    field("warnings", &HealthDiscoveryLinking::warnings));

template <>
inline constexpr std::string_view dto_name<HealthDiscoveryLinking> = "HealthDiscoveryLinking";

// =============================================================================
// HealthDiscovery - "discovery" sub-object inside Health.
//
// Wire shape (from health_handlers.cpp handle_health):
//   mode     - discovery mode string (required)
//   strategy - strategy name string (required)
//   pipeline - optional free-form JSON (merge report)
//   linking  - optional linking result sub-object
// =============================================================================
struct HealthDiscovery {
  std::string mode;
  std::string strategy;
  std::optional<nlohmann::json> pipeline;
  std::optional<HealthDiscoveryLinking> linking;
};

template <>
inline constexpr auto dto_fields<HealthDiscovery> =
    std::make_tuple(field("mode", &HealthDiscovery::mode), field("strategy", &HealthDiscovery::strategy),
                    field("pipeline", &HealthDiscovery::pipeline), field("linking", &HealthDiscovery::linking));

template <>
inline constexpr std::string_view dto_name<HealthDiscovery> = "HealthDiscovery";

// =============================================================================
// HealthWarning - single item inside Health::warnings array.
//
// Wire shape (from health_handlers.cpp get_health):
//   code          - stable machine-readable warning code string (required)
//   message       - human-readable description with remediation hints (required)
//   entity_ids    - SOVD entity ids this warning is about (required, array)
//   ros_node_fqns - ROS node fully-qualified names this warning is about
//                   (required, array)
//   peer_names    - aggregation peers involved in the anomaly (required, array)
//
// INVARIANT, true for every code present and future: every element of
// `entity_ids` is an addressable SOVD entity id - it satisfies
// validate_entity_id() and `GET /{collection}/{id}` can be built from it. That
// forbids ROS node FQNs there: validate_entity_id rejects '/' precisely
// because it "conflicts with URL routing" (entity_validation.cpp), so a FQN in
// `entity_ids` is a value no client can address. Node identifiers go in
// `ros_node_fqns` instead. A warning that is not about ROS nodes leaves that
// array empty, exactly as a warning that is not about aggregation leaves
// `peer_names` empty.
//
// The three id arrays are plain vectors rather than std::optional on purpose:
// SchemaWriter maps std::optional to anyOf:[T,null] (schema_writer.hpp), which
// would publish a null state the server can never emit and hand every
// generated client a tri-state to unpack. Empty array is the "none" value.
// =============================================================================
struct HealthWarning {
  std::string code;
  std::string message;
  std::vector<std::string> entity_ids;
  std::vector<std::string> ros_node_fqns;
  std::vector<std::string> peer_names;
};

template <>
inline constexpr auto dto_fields<HealthWarning> =
    std::make_tuple(field_enum("code", &HealthWarning::code, kHealthWarningCodeValues),
                    field("message", &HealthWarning::message), field("entity_ids", &HealthWarning::entity_ids),
                    field("ros_node_fqns", &HealthWarning::ros_node_fqns),
                    field("peer_names", &HealthWarning::peer_names));

template <>
inline constexpr std::string_view dto_name<HealthWarning> = "HealthWarning";

// =============================================================================
// Health - response for GET /health.
//
// Wire shape (from health_handlers.cpp handle_health):
//   status                        - always "healthy" (required)
//   timestamp                     - nanoseconds since epoch (integer, required)
//   discovery                     - optional discovery sub-object
//   x-medkit-data-provider        - optional free-form JSON stats object
//                                   (from Ros2TopicDataProvider::x_medkit_stats())
//   x-medkit-subscription-executor - optional free-form JSON stats object
//                                    (from Ros2TopicDataProvider::x_medkit_stats())
//   x-medkit-sse                  - optional free-form JSON object:
//                                   {connected_clients, max_clients}
//   peers                         - optional free-form JSON array (agg peer status)
//   warning_schema_version        - integer warnings-contract version; always
//                                   emitted, aggregation or not
//   warnings                      - array of HealthWarning; always emitted,
//                                   empty when nothing is flagged
//
// The x-medkit-* keys use hyphens (endpoint-level vendor extensions, NOT the
// nested "x-medkit" pattern). They are free-form nlohmann::json objects because
// x_medkit_stats() builds them dynamically. Both are optional (only present when
// a TopicDataProvider / executor is active).
// =============================================================================
struct Health {
  std::string status;
  int64_t timestamp{0};
  std::optional<HealthDiscovery> discovery;
  std::optional<nlohmann::json> x_medkit_data_provider;          // wire key: "x-medkit-data-provider"
  std::optional<nlohmann::json> x_medkit_subscription_executor;  // wire key: "x-medkit-subscription-executor"
  std::optional<nlohmann::json> x_medkit_entity_cache;           // wire key: "x-medkit-entity-cache"
  /// How much of `sse.max_clients` is in use, plus `events_received`: fault
  /// events the gateway has accepted from the fault manager since start. Wire
  /// key: "x-medkit-sse".
  ///
  /// The cap is small - two by default - and an aggregating gateway now
  /// consumes one slot on each of its peers for as long as somebody is
  /// watching its own fault stream. Without this an operator has no way to see
  /// why a peer refused their stream with 503.
  std::optional<nlohmann::json> x_medkit_sse;
  std::optional<nlohmann::json> peers;  // free-form array of peer status objects
  // NOT optional: these two are emitted in every mode, aggregation or not, so
  // the generated schema must list them in `required` and let a typed client
  // read them without a presence check. An empty `warnings` array is the
  // "nothing flagged" answer - absence is not a value this endpoint has.
  int64_t warning_schema_version{kWarningSchemaVersion};
  std::vector<HealthWarning> warnings;
};

template <>
inline constexpr auto dto_fields<Health> = std::make_tuple(
    field("status", &Health::status),
    field("timestamp", &Health::timestamp,
          "Gateway wall-clock reading taken while the response was being built, in nanoseconds since the Unix "
          "epoch. Not monotonic: it follows the host clock, so two readings can move backwards across an NTP "
          "step and the difference between them is not a reliable elapsed time."),
    field("discovery", &Health::discovery), field("x-medkit-data-provider", &Health::x_medkit_data_provider),
    field("x-medkit-subscription-executor", &Health::x_medkit_subscription_executor),
    field("x-medkit-entity-cache", &Health::x_medkit_entity_cache), field("x-medkit-sse", &Health::x_medkit_sse),
    field("peers", &Health::peers), field("warning_schema_version", &Health::warning_schema_version),
    field("warnings", &Health::warnings));

template <>
inline constexpr std::string_view dto_name<Health> = "HealthStatus";

// =============================================================================
// VersionInfoVendor - "vendor_info" sub-object inside VersionInfoEntry.
//
// Wire shape (from health_handlers.cpp handle_version_info):
//   version - gateway version string (required)
//   name    - gateway name string, always "ros2_medkit" (required)
// =============================================================================
struct VersionInfoVendor {
  std::string version;
  std::string name;
};

template <>
inline constexpr auto dto_fields<VersionInfoVendor> =
    std::make_tuple(field("version", &VersionInfoVendor::version), field("name", &VersionInfoVendor::name));

template <>
inline constexpr std::string_view dto_name<VersionInfoVendor> = "VersionInfoVendor";

// =============================================================================
// VersionInfoEntry - single item inside the VersionInfo::items array.
//
// Wire shape (from health_handlers.cpp handle_version_info):
//   version     - SOVD standard version string (required)
//   base_uri    - version-specific base URI string (required)
//   vendor_info - vendor-specific info sub-object (optional)
// =============================================================================
struct VersionInfoEntry {
  std::string version;
  std::string base_uri;
  std::optional<VersionInfoVendor> vendor_info;
};

template <>
inline constexpr auto dto_fields<VersionInfoEntry> =
    std::make_tuple(field("version", &VersionInfoEntry::version), field("base_uri", &VersionInfoEntry::base_uri),
                    field("vendor_info", &VersionInfoEntry::vendor_info));

template <>
inline constexpr std::string_view dto_name<VersionInfoEntry> = "VersionInfoEntry";

// =============================================================================
// XMedkitVersionInfo - typed x-medkit vendor extension on /version-info response.
//
// Emitted by handle_version_info when aggregation is active and a peer fan-out
// request is partial (some peers failed).
//
// Wire keys (from merge_peer_items in fan_out_helpers.hpp):
//   partial      - true when one or more peers failed during fan-out (optional)
//   failed_peers - list of peer addresses that returned errors (optional)
//   peer_failures - why each of those peers failed (timeout, unreachable, ...)
//
// Both fields are optional so the DTO is correctly empty (no "x-medkit" key
// in the response) when there are no aggregation peers or the fan-out succeeds
// completely.
// =============================================================================
struct XMedkitVersionInfo {
  std::optional<bool> partial;
  std::optional<std::vector<std::string>> failed_peers;
  /// Why each peer in `failed_peers` contributed nothing. Sibling rather than a
  /// widening of `failed_peers`, whose wire shape (a list of names) every
  /// existing client already reads.
  std::optional<std::vector<PeerFailure>> peer_failures;
};

template <>
inline constexpr auto dto_fields<XMedkitVersionInfo> =
    std::make_tuple(field("partial", &XMedkitVersionInfo::partial),
                    field("failed_peers", &XMedkitVersionInfo::failed_peers),
                    field("peer_failures", &XMedkitVersionInfo::peer_failures));

template <>
inline constexpr std::string_view dto_name<XMedkitVersionInfo> = "XMedkitVersionInfo";

// =============================================================================
// VersionInfo - response for GET /version-info (SOVD 7.4.1).
//
// Wire shape (from health_handlers.cpp handle_version_info):
//   items   - array of version entries (required)
//   x-medkit - optional typed vendor extension (aggregation fan-out metadata)
//
// The x-medkit field is only present when aggregation is active and a
// peer fan-out partially succeeds or fails.
// =============================================================================
struct VersionInfo {
  std::vector<VersionInfoEntry> items;
  std::optional<XMedkitVersionInfo> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<VersionInfo> =
    std::make_tuple(field("items", &VersionInfo::items), field("x-medkit", &VersionInfo::x_medkit));

template <>
inline constexpr std::string_view dto_name<VersionInfo> = "VersionInfo";

// =============================================================================
// RootCapabilities - "capabilities" sub-object inside RootOverview.
//
// Wire shape (from health_handlers.cpp handle_root capabilities object):
//   discovery, data_access, operations, async_actions, configurations,
//   faults, logs, bulk_data, cyclic_subscriptions, locking, triggers,
//   updates, authentication, tls, scripts, aggregation, vendor_extensions
//   (all boolean, all required)
// =============================================================================
struct RootCapabilities {
  bool discovery{false};
  bool data_access{false};
  bool operations{false};
  bool async_actions{false};
  bool configurations{false};
  bool faults{false};
  bool logs{false};
  bool bulk_data{false};
  bool cyclic_subscriptions{false};
  bool locking{false};
  bool triggers{false};
  bool updates{false};
  bool authentication{false};
  bool tls{false};
  bool scripts{false};
  bool aggregation{false};
  bool vendor_extensions{false};
};

template <>
inline constexpr auto dto_fields<RootCapabilities> = std::make_tuple(
    field("discovery", &RootCapabilities::discovery), field("data_access", &RootCapabilities::data_access),
    field("operations", &RootCapabilities::operations), field("async_actions", &RootCapabilities::async_actions),
    field("configurations", &RootCapabilities::configurations), field("faults", &RootCapabilities::faults),
    field("logs", &RootCapabilities::logs), field("bulk_data", &RootCapabilities::bulk_data),
    field("cyclic_subscriptions", &RootCapabilities::cyclic_subscriptions),
    field("locking", &RootCapabilities::locking), field("triggers", &RootCapabilities::triggers),
    field("updates", &RootCapabilities::updates), field("authentication", &RootCapabilities::authentication),
    field("tls", &RootCapabilities::tls), field("scripts", &RootCapabilities::scripts),
    field("aggregation", &RootCapabilities::aggregation),
    field("vendor_extensions", &RootCapabilities::vendor_extensions));

template <>
inline constexpr std::string_view dto_name<RootCapabilities> = "RootCapabilities";

// =============================================================================
// RootAuth - "auth" sub-object inside RootOverview.
//
// Wire shape (from health_handlers.cpp handle_root auth block):
//   enabled          - always true when this block is present (required)
//   algorithm        - JWT algorithm string (required)
//   require_auth_for - "none" | "write" | "all" (required)
// =============================================================================
struct RootAuth {
  bool enabled{false};
  std::string algorithm;
  std::string require_auth_for;
};

template <>
inline constexpr auto dto_fields<RootAuth> =
    std::make_tuple(field("enabled", &RootAuth::enabled), field("algorithm", &RootAuth::algorithm),
                    field("require_auth_for", &RootAuth::require_auth_for));

template <>
inline constexpr std::string_view dto_name<RootAuth> = "RootAuth";

// =============================================================================
// RootTls - "tls" sub-object inside RootOverview.
//
// Wire shape (from health_handlers.cpp handle_root TLS block):
//   enabled     - always true when this block is present (required)
//   min_version - TLS minimum version string (required)
// =============================================================================
struct RootTls {
  bool enabled{false};
  std::string min_version;
};

template <>
inline constexpr auto dto_fields<RootTls> =
    std::make_tuple(field("enabled", &RootTls::enabled), field("min_version", &RootTls::min_version));

template <>
inline constexpr std::string_view dto_name<RootTls> = "RootTls";

// =============================================================================
// RootOverview - response for GET / (API root).
//
// Wire shape (from health_handlers.cpp handle_root):
//   name         - "ROS 2 Medkit Gateway" (required)
//   version      - gateway version string (required)
//   api_base     - API base path string (required)
//   endpoints    - array of endpoint description strings (required)
//   capabilities - capabilities flags object (required)
//   auth         - optional auth info sub-object (present when auth enabled)
//   tls          - optional TLS info sub-object (present when TLS enabled)
// =============================================================================
struct RootOverview {
  std::string name;
  std::string version;
  std::string api_base;
  std::vector<std::string> endpoints;
  RootCapabilities capabilities;
  std::optional<RootAuth> auth;
  std::optional<RootTls> tls;
};

template <>
inline constexpr auto dto_fields<RootOverview> =
    std::make_tuple(field("name", &RootOverview::name), field("version", &RootOverview::version),
                    field("api_base", &RootOverview::api_base), field("endpoints", &RootOverview::endpoints),
                    field("capabilities", &RootOverview::capabilities), field("auth", &RootOverview::auth),
                    field("tls", &RootOverview::tls));

template <>
inline constexpr std::string_view dto_name<RootOverview> = "RootOverview";

}  // namespace dto
}  // namespace ros2_medkit_gateway
