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
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace http {

/**
 * @brief Addressing for an item whose wire id more than one member carries.
 *
 * Qualification follows AMBIGUITY, not aggregation. An entity that draws items
 * from members is the ordinary case, not the exception - in runtime discovery
 * every App hangs off the single host Component and namespace Functions are on
 * by default - so qualifying every id there would rename the items of the most
 * used entity in the product and refuse requests every current client sends.
 *
 * An id is ambiguous when more than one item in the merged collection carries
 * it: two members exposing the operation short name `calibrate` at different
 * ROS paths are two items with one id, and a caller holding that id cannot say
 * which one it meant. Those copies are addressed `<member>:<item>`. An id only
 * one item carries names one thing already and is left alone.
 *
 * A topic path is not ambiguous merely because several members publish and
 * subscribe to it: that is still one topic, merged into one item, and the bare
 * path addresses it exactly. It becomes ambiguous only if two gateways each
 * contribute an item under the same path.
 */
struct MemberQualifiedId {
  std::string member_id;   ///< Owning member; empty when the id carries no member half.
  std::string item_id;     ///< The item as its owning member names it.
  bool has_member{false};  ///< Whether the id carried a member half.
};

/**
 * @brief Split `id` into member and item halves at the FIRST colon.
 *
 * The first colon is the separator because an entity id is restricted to
 * alphanumerics, underscore and hyphen and so can never contain one, while an
 * item name can - a ROS 2 parameter name, for instance.
 *
 * Splitting only happens where a member half can mean something. On an entity
 * with no members a colon is an ordinary character of the item name, and
 * treating it as a separator would make the item addressable under a name
 * nothing exposes.
 */
inline MemberQualifiedId parse_member_qualified_id(const std::string & id, bool member_half_possible) {
  MemberQualifiedId parsed;
  parsed.item_id = id;

  const auto colon_pos = id.find(':');
  if (colon_pos != std::string::npos && member_half_possible) {
    parsed.member_id = id.substr(0, colon_pos);
    parsed.item_id = id.substr(colon_pos + 1);
    parsed.has_member = true;
  }
  return parsed;
}

/// The id that addresses `item_id` as the copy owned by `member_id`.
inline std::string make_member_qualified_id(const std::string & member_id, const std::string & item_id) {
  return member_id + ":" + item_id;
}

/**
 * @brief Rewrite the id of every item whose id another item in `items` shares.
 *
 * Runs on the merged collection - local items plus whatever the peer fan-out
 * contributed - because that is the first point at which a duplicate is
 * visible. Neither gateway can see the collision on its own: each holds one
 * `calibrate` and considers it unique.
 *
 * `member_ids_of` returns the item's contributing members, or nullptr when it
 * names none. An item that names no member, or names several, is left bare: a
 * qualifier picked from an ambiguous set would be a guess, and a guess here
 * reproduces the defect the qualified form exists to remove.
 */
template <class Item, class MemberIdsOf>
void qualify_ambiguous_ids(std::vector<Item> & items, MemberIdsOf member_ids_of) {
  std::unordered_map<std::string, size_t> id_counts;
  for (const auto & item : items) {
    ++id_counts[item.id];
  }

  for (auto & item : items) {
    auto count = id_counts.find(item.id);
    if (count == id_counts.end() || count->second < 2) {
      continue;
    }
    const std::vector<std::string> * members = member_ids_of(item);
    if (members == nullptr || members->size() != 1) {
      continue;
    }
    // An id already addressed to this member is left alone. Prefixing it again
    // yields a form whose first colon splits off the member twice, which names
    // nothing - and it happens whenever one member exposes the same short name
    // more than once, because both copies then carry the same qualified id.
    if (item.id.rfind(members->front() + ":", 0) == 0) {
      continue;
    }
    item.id = make_member_qualified_id(members->front(), item.id);
  }
}

}  // namespace http
}  // namespace ros2_medkit_gateway
