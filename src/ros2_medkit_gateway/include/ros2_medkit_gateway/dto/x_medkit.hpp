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

#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entity_capability.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// ---------------------------------------------------------------------------
// XMedkitRos2 - the nested "ros2" sub-object inside every x-medkit payload.
//
// All members are optional: each entity type only populates the subset that
// applies to it.  Wire key for the namespace member is "namespace" even
// though the C++ member is named `ns` (reserved keyword avoidance).
// ---------------------------------------------------------------------------
struct XMedkitRos2 {
  std::optional<std::string> node;
  std::optional<std::string> ns;  // wire key: "namespace"
  std::optional<std::string> type;
  std::optional<std::string> topic;
  std::optional<std::string> service;
  std::optional<std::string> action;
  std::optional<std::string> kind;
  std::optional<std::string> direction;  // topic data direction: "publish"|"subscribe"|"both"
};

template <>
inline constexpr auto dto_fields<XMedkitRos2> =
    std::make_tuple(field("node", &XMedkitRos2::node), field("namespace", &XMedkitRos2::ns),
                    field("type", &XMedkitRos2::type), field("topic", &XMedkitRos2::topic),
                    field("service", &XMedkitRos2::service), field("action", &XMedkitRos2::action),
                    field("kind", &XMedkitRos2::kind), field("direction", &XMedkitRos2::direction));

template <>
inline constexpr std::string_view dto_name<XMedkitRos2> = "XMedkitRos2";

// ---------------------------------------------------------------------------
// XMedkitArea - x-medkit payload for Area entities.
//
// Populated by handle_get_area / handle_list_areas:
//   ros2.namespace       <- area.namespace_path
//   parent_area_id       <- area.parent_area_id  (detail only, via ext.add())
//   contributors         <- area.contributors    (detail only)
//
// Also used in sub-collection responses (handle_app_belongs_to):
//   missing              <- true when area reference cannot be resolved
//   unresolved_component <- component_id that could not be resolved (belongs-to only)
// ---------------------------------------------------------------------------
struct XMedkitArea {
  std::optional<XMedkitRos2> ros2;
  std::optional<std::string> parent_area_id;
  std::optional<std::vector<std::string>> contributors;
  std::optional<bool> missing;                      // broken reference sentinel
  std::optional<std::string> unresolved_component;  // belongs-to: unresolvable parent component id
};

template <>
inline constexpr auto dto_fields<XMedkitArea> =
    std::make_tuple(field("ros2", &XMedkitArea::ros2), field("parent_area_id", &XMedkitArea::parent_area_id),
                    field("contributors", &XMedkitArea::contributors), field("missing", &XMedkitArea::missing),
                    field("unresolved_component", &XMedkitArea::unresolved_component));

template <>
inline constexpr std::string_view dto_name<XMedkitArea> = "XMedkitArea";

// ---------------------------------------------------------------------------
// XMedkitComponent - x-medkit payload for Component entities.
//
// Populated by handle_get_component / handle_list_components:
//   source            <- comp.source
//   ros2.node         <- comp.fqn
//   ros2.namespace    <- comp.namespace_path
//   type              <- comp.type              (via ext.add())
//   parent_component_id <- comp.parent_component_id (via ext.add("parentComponentId",...))
//   depends_on        <- comp.depends_on        (via ext.add(), array of strings)
//   area              <- comp.area              (via ext.add())
//   variant           <- comp.variant           (via ext.add())
//   description       <- comp.description       (via ext.add())
//   contributors      <- comp.contributors
//   capabilities      <- the same EntityCapability array the response carries at root level
//   external          <- comp.external, non-ROS external asset classification; emitted true-only on every
//                        route that carries the component x-medkit, so "absence == not external" holds (#516)
//
// Also used in sub-collection responses (depends-on, subcomponents, hosts, contains, etc.):
//   missing           <- true when component reference cannot be resolved
//
// Note: "parentComponentId" uses camelCase on the wire per discovery_handlers.cpp.
// "dependsOn" uses camelCase on the wire per discovery_handlers.cpp.
// ---------------------------------------------------------------------------
struct XMedkitComponent {
  std::optional<XMedkitRos2> ros2;
  std::optional<std::string> source;
  std::optional<std::string> type;
  std::optional<std::string> parent_component_id;      // wire key: "parentComponentId"
  std::optional<std::vector<std::string>> depends_on;  // wire key: "dependsOn"
  std::optional<std::string> area;
  std::optional<std::string> variant;
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> contributors;
  std::optional<std::vector<EntityCapability>> capabilities;
  // Asset-identity nameplate (AssetIdentity::to_json shape: camelCase fields +
  // "_provenance"). Free-form JSON so the DTO layer reuses the exact
  // serialization emitted by Component::to_json and consumed by peer parsing.
  std::optional<nlohmann::json> identity;
  std::optional<bool> missing;   // broken reference sentinel
  std::optional<bool> external;  // non-ROS external asset classification (#516)
  // Emitted only when false; an absent field means reachable.
  std::optional<bool> available;
};

template <>
inline constexpr auto dto_fields<XMedkitComponent> = std::make_tuple(
    field("ros2", &XMedkitComponent::ros2), field("source", &XMedkitComponent::source),
    field("type", &XMedkitComponent::type), field("parentComponentId", &XMedkitComponent::parent_component_id),
    field("dependsOn", &XMedkitComponent::depends_on), field("area", &XMedkitComponent::area),
    field("variant", &XMedkitComponent::variant), field("description", &XMedkitComponent::description),
    field("contributors", &XMedkitComponent::contributors), field("capabilities", &XMedkitComponent::capabilities),
    field("identity", &XMedkitComponent::identity), field("missing", &XMedkitComponent::missing),
    field("external", &XMedkitComponent::external), field("available", &XMedkitComponent::available));

template <>
inline constexpr std::string_view dto_name<XMedkitComponent> = "XMedkitComponent";

// ---------------------------------------------------------------------------
// XMedkitApp - x-medkit payload for App entities.
//
// Populated by handle_get_app / handle_list_apps:
//   source        <- app.source
//   is_online     <- app.is_online
//   ros2.node     <- app.bound_fqn
//   component_id  <- app.component_id
//   contributors  <- app.contributors  (detail only)
//   external      <- app.external, non-ROS external asset classification; emitted true-only on every route
//                    that carries the app x-medkit, so "absence == not external" holds (#516/#517)
//
// Also used in sub-collection responses (depends-on, hosts, function-hosts):
//   missing       <- true when app reference cannot be resolved
// ---------------------------------------------------------------------------
struct XMedkitApp {
  std::optional<XMedkitRos2> ros2;
  std::optional<std::string> source;
  std::optional<bool> is_online;
  std::optional<std::string> component_id;
  std::optional<std::vector<std::string>> contributors;
  std::optional<bool> missing;   // broken reference sentinel
  std::optional<bool> external;  // non-ROS external asset classification (#516/#517)
  /// Absent while the entity is reachable. False marks an entity retained from
  /// a peer's last known declaration while that peer is not answering, so a
  /// client can tell "declared but unreachable" from "gone".
  std::optional<bool> available;
};

template <>
inline constexpr auto dto_fields<XMedkitApp> =
    std::make_tuple(field("ros2", &XMedkitApp::ros2), field("source", &XMedkitApp::source),
                    field("is_online", &XMedkitApp::is_online), field("component_id", &XMedkitApp::component_id),
                    field("contributors", &XMedkitApp::contributors), field("missing", &XMedkitApp::missing),
                    field("external", &XMedkitApp::external), field("available", &XMedkitApp::available));

template <>
inline constexpr std::string_view dto_name<XMedkitApp> = "XMedkitApp";

// ---------------------------------------------------------------------------
// XMedkitFunction - x-medkit payload for Function entities.
//
// Populated by handle_get_function / handle_list_functions:
//   source       <- func.source
//   hosts        <- func.hosts   (array of app IDs, via ext.add())
//   description  <- func.description (via ext.add())
//   contributors <- func.contributors (detail only)
// ---------------------------------------------------------------------------
struct XMedkitFunction {
  std::optional<XMedkitRos2> ros2;
  std::optional<std::string> source;
  std::optional<std::vector<std::string>> hosts;
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> contributors;
};

template <>
inline constexpr auto dto_fields<XMedkitFunction> =
    std::make_tuple(field("ros2", &XMedkitFunction::ros2), field("source", &XMedkitFunction::source),
                    field("hosts", &XMedkitFunction::hosts), field("description", &XMedkitFunction::description),
                    field("contributors", &XMedkitFunction::contributors));

template <>
inline constexpr std::string_view dto_name<XMedkitFunction> = "XMedkitFunction";

// ---------------------------------------------------------------------------
// XMedkitCollection - x-medkit payload on list (collection) responses.
//
// Emitted by every handle_list_* and sub-collection handler via resp_ext.add():
//   total_count        <- items.size()
//   contributors       <- (optional aggregation provenance, included for completeness)
//   partial            <- true when a fan-out peer request failed
//   failed_peers       <- list of peer URLs that returned errors
//   peer_failures      <- why each of those peers failed (timeout, unreachable, ...)
//   peer_dropped_items <- per-peer items dropped due to malformed JSON
//                         (observability for invisible drift)
// ---------------------------------------------------------------------------
struct XMedkitCollection {
  std::optional<std::size_t> total_count;
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
inline constexpr auto dto_fields<XMedkitCollection> = std::make_tuple(
    field("total_count", &XMedkitCollection::total_count), field("contributors", &XMedkitCollection::contributors),
    field("partial", &XMedkitCollection::partial), field("failed_peers", &XMedkitCollection::failed_peers),
    field("peer_failures", &XMedkitCollection::peer_failures),
    field("peer_dropped_items", &XMedkitCollection::peer_dropped_items));

template <>
inline constexpr std::string_view dto_name<XMedkitCollection> = "XMedkitCollection";

}  // namespace dto
}  // namespace ros2_medkit_gateway
