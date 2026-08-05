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

#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// FaultTriggerRule - one threshold rule on an app's discovered data points.
//
// Wire keys are exactly FaultTriggerEngine::rule_to_json():
//   id, data_name, operator, threshold, fault_code, severity, active
//
// Named FaultTriggerRule in the `dto` namespace; the engine's own
// `ros2_medkit_gateway::FaultTriggerRule` is a different type in a different
// namespace and carries two fields this one deliberately does not: `app_id`
// (the route path already says which app) and `crossed` (a runtime latch
// persisted to the store, never part of the REST response).
//
// The three routes are `raw()` registrations, so nothing parses a request or
// writes a response through these descriptors - the engine does both by hand.
// They exist so the published schema and the engine's field names are one edit
// apart. `field_enum` is therefore documentation here rather than enforcement,
// and both vocabularies are copied from the engine's own validators
// (`valid_operator`, `valid_severity`).
// =============================================================================
struct FaultTriggerRule {
  std::string id;
  std::string data_name;
  std::string comparison;  // wire key: "operator" (a C++ keyword)
  double threshold{0.0};
  std::string fault_code;
  std::string severity;
  bool active{true};
};

template <>
inline constexpr auto dto_fields<FaultTriggerRule> = std::make_tuple(
    field("id", &FaultTriggerRule::id, "Server-assigned rule id, as returned on create."),
    field("data_name", &FaultTriggerRule::data_name,
          "Data resource on the app whose value the rule watches. Must be one the app currently exposes."),
    field_enum("operator", &FaultTriggerRule::comparison, kFaultTriggerOperatorValues,
               "Comparison applied to the sampled value against `threshold`."),
    field("threshold", &FaultTriggerRule::threshold, "Value the comparison is made against."),
    field("fault_code", &FaultTriggerRule::fault_code,
          "Fault raised on a threshold cross. Unique across every rule on every app - fault codes are the "
          "fault store's primary key, so a duplicate is refused with 409."),
    field_enum("severity", &FaultTriggerRule::severity, kFaultTriggerSeverityValues,
               "Severity of the raised fault. Note `WARNING`, not the `WARN` a reported fault's "
               "`severity_label` uses."),
    // Required, unlike its counterpart on the create request: `rule_to_json`
    // always emits it, so a response never omits it. "Defaults to true" is a
    // property of the request, where the field really can be left out.
    field("active", &FaultTriggerRule::active, "Whether the rule is currently being evaluated."));

template <>
inline constexpr std::string_view dto_name<FaultTriggerRule> = "FaultTriggerRule";

// =============================================================================
// FaultTriggerRuleCreateRequest - POST /{entity}/fault-triggers body.
//
// Fields and their validation come from FaultTriggerEngine::create(): every
// member below except `active` is rejected with 400 when missing or malformed.
// `id` is server-assigned and is not read from the request.
// =============================================================================
struct FaultTriggerRuleCreateRequest {
  std::string data_name;
  std::string comparison;  // wire key: "operator"
  double threshold{0.0};
  std::string fault_code;
  std::string severity;
  std::optional<bool> active;
};

template <>
inline constexpr auto dto_fields<FaultTriggerRuleCreateRequest> = std::make_tuple(
    field("data_name", &FaultTriggerRuleCreateRequest::data_name,
          "Data resource on the app to watch. A name the app does not expose is refused with 400, so a rule "
          "cannot be created that could never fire."),
    field_enum("operator", &FaultTriggerRuleCreateRequest::comparison, kFaultTriggerOperatorValues,
               "Comparison applied to the sampled value against `threshold`."),
    field("threshold", &FaultTriggerRuleCreateRequest::threshold, "Value the comparison is made against."),
    field("fault_code", &FaultTriggerRuleCreateRequest::fault_code,
          "Fault to raise on a threshold cross. Must not already be used by another rule (409)."),
    field_enum("severity", &FaultTriggerRuleCreateRequest::severity, kFaultTriggerSeverityValues,
               "Severity of the raised fault."),
    field("active", &FaultTriggerRuleCreateRequest::active,
          "Whether to evaluate the rule immediately. Defaults to true."));

template <>
inline constexpr std::string_view dto_name<FaultTriggerRuleCreateRequest> = "FaultTriggerRuleCreateRequest";

// =============================================================================
// FaultTriggerRuleList - GET /{entity}/fault-triggers response.
//
// A struct of its own rather than `Collection<FaultTriggerRule>`: the list
// route emits a bare `{"items": [...]}` and nothing else, whereas the generic
// wrapper would publish optional `x-medkit` (typed `XMedkitCollection`) and
// `_links` members this route has never sent. That is the same defect
// `FaultList` carried - a documented vendor extension the gateway does not
// emit - and the fix is to describe only what is sent.
// =============================================================================
struct FaultTriggerRuleList {
  std::vector<FaultTriggerRule> items;
};

template <>
inline constexpr auto dto_fields<FaultTriggerRuleList> =
    std::make_tuple(field("items", &FaultTriggerRuleList::items, "Rules scoped to this app, in creation order."));

template <>
inline constexpr std::string_view dto_name<FaultTriggerRuleList> = "FaultTriggerRuleList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
