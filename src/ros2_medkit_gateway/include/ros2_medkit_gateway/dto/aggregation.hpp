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

#include <string>
#include <string_view>
#include <tuple>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// ---------------------------------------------------------------------------
// DroppedItem - typed observability entry describing one peer-supplied item
// that was dropped during fan-out merge because it failed JsonReader<T>
// validation.
//
// Carried in the peer_dropped_items field on every collection-level XMedkit
// type. Without this, malformed peer items disappeared silently from
// aggregated responses ("invisible drift") with no observability for the
// caller.
//
// Wire keys:
//   peer       - peer URL the item came from
//   reason     - JsonReader<T> error message (e.g. "field missing: id")
//   source_id  - best-effort extraction of "id" from the malformed JSON; may
//                be empty when the raw item had no parseable id field
// ---------------------------------------------------------------------------
struct DroppedItem {
  std::string peer;
  std::string reason;
  std::string source_id;
};

template <>
inline constexpr auto dto_fields<DroppedItem> =
    std::make_tuple(field("peer", &DroppedItem::peer), field("reason", &DroppedItem::reason),
                    field("source_id", &DroppedItem::source_id));

template <>
inline constexpr std::string_view dto_name<DroppedItem> = "DroppedItem";

// ---------------------------------------------------------------------------
// PeerFailure - why one peer contributed nothing to a fanned-out collection.
//
// Carried in the peer_failures field beside failed_peers on every
// collection-level XMedkit type. failed_peers names WHICH peers failed;
// without this, a peer that ran out of time, a peer that refused the
// connection, a peer that answered 500, a body over the size limit and
// unparseable JSON are all the same entry in that list, and a client cannot
// tell a slow subsystem from a dead one.
//
// Wire keys:
//   peer   - peer name, matching the entry in failed_peers
//   reason - one of: timeout, unreachable, canceled, error-status, too-large,
//            invalid-response
// ---------------------------------------------------------------------------
struct PeerFailure {
  std::string peer;
  std::string reason;
};

template <>
inline constexpr auto dto_fields<PeerFailure> =
    std::make_tuple(field("peer", &PeerFailure::peer), field("reason", &PeerFailure::reason));

template <>
inline constexpr std::string_view dto_name<PeerFailure> = "PeerFailure";

}  // namespace dto
}  // namespace ros2_medkit_gateway
