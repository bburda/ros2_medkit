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

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// CyclicSubscription - SOVD cyclic subscription CRUD response object.
//
// Emitted by handle_create (201), handle_list (items element),
// handle_get (200), handle_update (200).
//
// Wire keys (from CyclicSubscriptionHandlers::subscription_to_json):
//   id                - subscription UUID (required)
//   observed_resource - resource URI being observed (required)
//   event_source      - server-generated SSE stream URI (required)
//   protocol          - transport protocol, e.g. "sse" (required)
//   interval          - enum: "fast"|"normal"|"slow" (required)
// =============================================================================
struct CyclicSubscription {
  std::string id;
  std::string observed_resource;  // wire key: "observed_resource"
  std::string event_source;       // wire key: "event_source"
  std::string protocol;
  std::string interval;  // enum: "fast"|"normal"|"slow"
};

template <>
inline constexpr auto dto_fields<CyclicSubscription> = std::make_tuple(
    field("id", &CyclicSubscription::id), field("observed_resource", &CyclicSubscription::observed_resource),
    field("event_source", &CyclicSubscription::event_source), field("protocol", &CyclicSubscription::protocol),
    field_enum("interval", &CyclicSubscription::interval, kCyclicSubscriptionIntervalValues));

template <>
inline constexpr std::string_view dto_name<CyclicSubscription> = "CyclicSubscription";

// =============================================================================
// CyclicSubscriptionCreateRequest - POST /{entity}/cyclic-subscriptions body.
// Parsed by handle_create via parse_body<CyclicSubscriptionCreateRequest>.
//
// Wire keys (from handle_create body parsing +
//            cyclic_subscription_create_request_schema):
//   resource  - resource URI to subscribe to (required)
//   interval  - enum: "fast"|"normal"|"slow" (required)
//   duration  - subscription duration in seconds, must be > 0 (required)
//   protocol  - transport protocol, default "sse" (optional)
// =============================================================================
struct CyclicSubscriptionCreateRequest {
  std::string resource;
  std::string interval;  // enum: "fast"|"normal"|"slow"
  int duration{0};       // seconds; additional validation: must be > 0
  std::optional<std::string> protocol;
};

template <>
inline constexpr auto dto_fields<CyclicSubscriptionCreateRequest> =
    std::make_tuple(field("resource", &CyclicSubscriptionCreateRequest::resource),
                    field("interval", &CyclicSubscriptionCreateRequest::interval,
                          "Sampling rate, named rather than numeric: `fast`, `normal` or `slow`. Any other value "
                          "is rejected with 400."),
                    field("duration", &CyclicSubscriptionCreateRequest::duration,
                          "How long the subscription runs, in seconds. Must be at least 1 and no more than the "
                          "gateway's configured ceiling; only the lower bound is a schema constraint, because "
                          "the ceiling is deployment configuration and a request over it answers 400 naming the "
                          "limit.",
                          FieldConstraints{/*minimum=*/1.0, {}, {}, {}, {}}),
                    field("protocol", &CyclicSubscriptionCreateRequest::protocol));

template <>
inline constexpr std::string_view dto_name<CyclicSubscriptionCreateRequest> = "CyclicSubscriptionCreateRequest";

// =============================================================================
// CyclicSubscriptionUpdateRequest - PUT /{entity}/cyclic-subscriptions/{id}
// body. Parsed by handle_update via parse_body<CyclicSubscriptionUpdateRequest>.
//
// Wire keys (from handle_update body parsing):
//   interval  - enum: "fast"|"normal"|"slow" (optional)
//   duration  - new duration in seconds, must be > 0 (optional)
// =============================================================================
struct CyclicSubscriptionUpdateRequest {
  std::optional<std::string> interval;  // enum: "fast"|"normal"|"slow"
  std::optional<int> duration;          // seconds; additional validation: must be > 0
};

template <>
inline constexpr auto dto_fields<CyclicSubscriptionUpdateRequest> =
    std::make_tuple(field("interval", &CyclicSubscriptionUpdateRequest::interval,
                          "New sampling rate, validated exactly as on create: `fast`, `normal` or `slow`, "
                          "anything else 400. Omit it to leave the rate unchanged."),
                    field("duration", &CyclicSubscriptionUpdateRequest::duration,
                          "New run length in seconds, validated exactly as on create: at least 1 and no more "
                          "than the gateway's configured ceiling. Restarts the countdown from now rather than "
                          "adding to what is left. Omit it to leave the duration unchanged.",
                          FieldConstraints{/*minimum=*/1.0, {}, {}, {}, {}}));

template <>
inline constexpr std::string_view dto_name<CyclicSubscriptionUpdateRequest> = "CyclicSubscriptionUpdateRequest";

// =============================================================================
// Collection<CyclicSubscription> - named "CyclicSubscriptionList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<CyclicSubscription>> = "CyclicSubscriptionList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
