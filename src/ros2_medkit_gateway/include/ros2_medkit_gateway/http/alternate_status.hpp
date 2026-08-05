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

#include "ros2_medkit_gateway/http/handler_result.hpp"

namespace ros2_medkit_gateway {
namespace http {

/// Per-type HTTP status mapping used by `RouteRegistry::post_alternates` /
/// `RouteRegistry::del_alternates`. The framework picks the status from the
/// active alternative of the handler's returned `std::variant<TAlt...>`.
///
/// Default is `200`. Specialize for any DTO whose default should differ:
///
/// ```
/// template <> struct dto_alternate_status<MyDto> {
///   static constexpr int value = 202;
/// };
/// ```
///
/// `NoContent` is specialized below because it is a framework type. Other
/// specializations should live next to the DTO they describe (per-domain
/// header) so the wire-shape mapping stays close to the DTO definition.
template <class T>
struct dto_alternate_status {
  static constexpr int value = 200;
};

/// `NoContent` marker always maps to 204 No Content.
template <>
struct dto_alternate_status<NoContent> {
  static constexpr int value = 204;
};

/// Status wrappers. These are partial specializations over a different type
/// than the `NoContent` full specialization, so `Accepted<NoContent>` matches
/// only the partial one and resolves to 202, not 204.
template <class T>
struct dto_alternate_status<Created<T>> {
  static constexpr int value = 201;
};

template <class T>
struct dto_alternate_status<Accepted<T>> {
  static constexpr int value = 202;
};

/// The payload a status wrapper carries. An unwrapped type is its own payload,
/// so every existing route keeps its current schema and serialization.
///
/// The typed route helpers must ask this - never the wrapper itself - for the
/// schema `$ref`, the `has_dto_shape_v` assertion and the body writer. The
/// wrappers deliberately have no `dto_fields` / `dto_name` specialization.
template <class T>
struct status_payload {
  using type = T;
};
template <class T>
struct status_payload<Created<T>> {
  using type = T;
};
template <class T>
struct status_payload<Accepted<T>> {
  using type = T;
};
template <class T>
using status_payload_t = typename status_payload<T>::type;

/// Unwrap a status wrapper to the value the body writer should serialize.
/// Call qualified (`http::status_body(v)`): ADL finds these overloads for
/// `Created<T>` but not for a bare DTO in namespace `dto`.
///
/// Overload resolution picks the wrapper overloads over the generic one by
/// partial ordering of function templates - both are exact matches, and the
/// wrapper form is more specialized.
template <class T>
const T & status_body(const T & value) {
  return value;
}
template <class T>
const T & status_body(const Created<T> & wrapped) {
  return wrapped.value;
}
template <class T>
const T & status_body(const Accepted<T> & wrapped) {
  return wrapped.value;
}

}  // namespace http
}  // namespace ros2_medkit_gateway
