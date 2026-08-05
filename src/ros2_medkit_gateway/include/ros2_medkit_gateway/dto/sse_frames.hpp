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

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/errors.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// The JSON document carried in the `data:` field of an SSE frame.
//
// The eight SSE routes emit three different shapes, which is why they cannot
// share one schema and why the document used to declare none at all. Nothing
// serialises through these descriptors - each stream builds its frame by hand
// at the emission site - so they are the published schema, kept next to a
// comment naming the code that has to agree with them.
//
// A stream also sends non-JSON lines a schema cannot describe and a client
// must tolerate: `:keepalive` comments on an idle trigger stream, and the
// `id:` / `event:` fields the fault stream sets for reconnection.
// =============================================================================

// -----------------------------------------------------------------------------
// SubscriptionEventFrame - cyclic-subscription streams.
// Built by SubscriptionTransportProvider::make_sse_stream
// (http/handlers/sse_transport_provider.cpp).
//
// Exactly one of `payload` / `error` is present per frame: the sampler either
// returned a value or it did not.
// -----------------------------------------------------------------------------
struct SubscriptionEventFrame {
  std::string timestamp;
  std::optional<nlohmann::json> payload;
  std::optional<GenericError> error;
};

template <>
inline constexpr auto dto_fields<SubscriptionEventFrame> =
    std::make_tuple(field("timestamp", &SubscriptionEventFrame::timestamp,
                          "Sample time, ISO 8601 UTC with millisecond precision (e.g. `2026-07-29T10:15:30.123Z`)."),
                    field("payload", &SubscriptionEventFrame::payload,
                          "The sampled resource, in the shape that resource's own GET returns. Absent when `error` is "
                          "present."),
                    field("error", &SubscriptionEventFrame::error,
                          "Why this tick produced no sample. Absent when `payload` is present. A sampler that "
                          "returned a failure sets `error_code`, `vendor_code` and `message`; a sampler that threw "
                          "sets `error_code` (`internal-error`) and `message` only. `parameters` is never set on "
                          "either. The stream stays open and the next tick is retried."));

template <>
inline constexpr std::string_view dto_name<SubscriptionEventFrame> = "SubscriptionEventFrame";

// -----------------------------------------------------------------------------
// TriggerEventFrame - trigger event streams.
// Built by TriggerManager (core/managers/trigger_manager.cpp), drained by
// TriggerHandlers::sse_trigger_events.
//
// No `error` member, unlike the subscription frame: a trigger frame is only
// produced when the condition fired, so there is no failed-sample case to
// report. An idle stream sends `:keepalive` comment lines instead.
// -----------------------------------------------------------------------------
struct TriggerEventFrame {
  std::string timestamp;
  nlohmann::json payload;
};

template <>
inline constexpr auto dto_fields<TriggerEventFrame> =
    std::make_tuple(field("timestamp", &TriggerEventFrame::timestamp, "Fire time, ISO 8601 UTC."),
                    field("payload", &TriggerEventFrame::payload,
                          "The observed resource's value at the moment the condition fired - the whole value, not the "
                          "`path` sub-document the condition was evaluated against."));

template <>
inline constexpr std::string_view dto_name<TriggerEventFrame> = "TriggerEventFrame";

// -----------------------------------------------------------------------------
// FaultStreamXMedkit - the `x-medkit` object on a fault-stream frame.
//
// Present only when the fault's reporting source resolves to a known entity.
// It is a hint for addressing the fault's bulk-data, not an ownership claim:
// a debounced fault can have several co-reporters and this names the
// lexicographically first.
// -----------------------------------------------------------------------------
struct FaultStreamXMedkit {
  std::string entity_type;
  std::string entity_id;
};

template <>
inline constexpr auto dto_fields<FaultStreamXMedkit> = std::make_tuple(
    field("entity_type", &FaultStreamXMedkit::entity_type, "`areas`, `components`, `apps` or `functions`."),
    field("entity_id", &FaultStreamXMedkit::entity_id,
          "Entity to address for this fault's rosbag: "
          "`GET /{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}`."));

template <>
inline constexpr std::string_view dto_name<FaultStreamXMedkit> = "FaultStreamXMedkit";

// -----------------------------------------------------------------------------
// FaultStreamEvent - the global fault stream, GET /faults/stream.
// Built by SSEFaultHandler::format_sse_event
// (http/handlers/sse_fault_handler.cpp).
//
// A different shape from the two above, with no `payload` key at all. Its
// frames additionally carry `id:` and `event:` SSE fields, which is what makes
// the `Last-Event-ID` replay protocol on this route work.
// -----------------------------------------------------------------------------
struct FaultStreamEvent {
  std::string event_type;
  nlohmann::json fault;
  double timestamp{0.0};
  std::optional<FaultStreamXMedkit> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FaultStreamEvent> = std::make_tuple(
    field("event_type", &FaultStreamEvent::event_type,
          "What happened to the fault. Also sent as the frame's SSE `event:` field, so a client can "
          "subscribe per type."),
    field("fault", &FaultStreamEvent::fault,
          "The fault, in the flat `FaultListItem` shape the fault list uses (fault_code, severity, status, "
          "reporting_sources, ...)."),
    field("timestamp", &FaultStreamEvent::timestamp,
          "Event time as seconds since the Unix epoch, with nanosecond precision in the fraction. Note this "
          "is a number, where the subscription and trigger frames send an ISO 8601 string."),
    field("x-medkit", &FaultStreamEvent::x_medkit,
          "Where to fetch this fault's bulk-data. Absent when the reporting source resolves to no known "
          "entity."));

template <>
inline constexpr std::string_view dto_name<FaultStreamEvent> = "FaultStreamEvent";

}  // namespace dto
}  // namespace ros2_medkit_gateway
