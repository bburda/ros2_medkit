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
#include <string_view>
#include <tuple>
#include <type_traits>
#include <variant>
#include <vector>

namespace ros2_medkit_gateway {
namespace dto {

/// Whether a DTO field is mandatory in the OpenAPI schema / request body.
enum class Presence { kRequired, kOptional };

// --- type traits -----------------------------------------------------------

template <class T>
struct is_optional : std::false_type {};
template <class U>
struct is_optional<std::optional<U>> : std::true_type {};
template <class T>
inline constexpr bool is_optional_v = is_optional<T>::value;

template <class T>
struct is_vector : std::false_type {};
template <class U, class A>
struct is_vector<std::vector<U, A>> : std::true_type {};
template <class T>
inline constexpr bool is_vector_v = is_vector<T>::value;

template <class T>
struct is_variant : std::false_type {};
template <class... U>
struct is_variant<std::variant<U...>> : std::true_type {};
template <class T>
inline constexpr bool is_variant_v = is_variant<T>::value;

/// Optional members default to kOptional, everything else to kRequired.
template <class M>
constexpr Presence default_presence() {
  return is_optional_v<M> ? Presence::kOptional : Presence::kRequired;
}

// --- Field -----------------------------------------------------------------
//
// Two field descriptor kinds:
//   * `Field<C, M>`           - typed scalar / nested DTO / container member.
//                               The three visitors introspect the C++ member
//                               type to derive wire shape, parsing, and
//                               schema.
//   * `OpaqueObjectField<C>`  - a `nlohmann::json` member whose runtime shape
//                               depends on context (live ROS message payload,
//                               action result, plugin-defined data). The
//                               visitors pass the value through without
//                               introspection; the schema is
//                               `{type:object, additionalProperties:true,
//                                x-medkit-opaque:true}`.
//
// Both descriptors are produced via factory functions (`field`, `field_enum`,
// `opaque_object`) and dispatched in the visitors via the
// `is_opaque_object_field_v` detection trait below.

/// JSON Schema constraints a field publishes beyond its C++ type.
///
/// Every member is unset by default, so a `field()` call that names no
/// constraints emits exactly what it emitted before this type existed. Only
/// write a constraint the handler enforces unconditionally: a bound the
/// gateway reads from a ROS parameter is configuration, and publishing it as
/// a schema keyword would make the document wrong on any deployment that
/// changed it. Say those in the `description` instead.
///
/// `format` is an override. Numeric width already derives from the member type
/// (`SchemaWriter` emits `format: int64` for a signed 64-bit integral), so set
/// this only for a string format the type cannot imply - `date-time`, `uri`.
struct FieldConstraints {
  std::optional<double> minimum{};
  std::optional<double> maximum{};
  std::optional<std::size_t> max_length{};
  std::string_view pattern{};
  std::string_view format{};
};

/// Binds a JSON key to a struct member plus OpenAPI metadata.
/// NEVER brace-initialize Field directly: aggregate CTAD is C++20-only.
/// Always construct via the field() / field_enum() factories below.
template <class Class, class Member>
struct Field {
  std::string_view key;
  Member Class::*ptr;
  Presence presence;
  std::string_view description;
  const std::string_view * enum_values;  // (ptr,count) into an inline constexpr array
  std::size_t enum_count;
  /// Written by every factory below rather than defaulted at the call sites:
  /// the three `Field{...}` aggregate initialisations would otherwise trip
  /// -Wmissing-field-initializers, which the build promotes to an error.
  FieldConstraints constraints;
};

template <class C, class M>
constexpr Field<C, M> field(std::string_view key, M C::*ptr, std::string_view desc = std::string_view{}) {
  return Field<C, M>{key, ptr, default_presence<M>(), desc, nullptr, 0, FieldConstraints{}};
}

/// Constrained field. C++17 has no designated initialisers, so a call site
/// spells the unset members out:
///   field("max_entries", &T::max_entries, "…",
///         FieldConstraints{/*minimum=*/1.0, /*maximum=*/10000.0, {}, {}, {}})
template <class C, class M>
constexpr Field<C, M> field(std::string_view key, M C::*ptr, std::string_view desc, FieldConstraints c) {
  return Field<C, M>{key, ptr, default_presence<M>(), desc, nullptr, 0, c};
}

template <class C, class M>
constexpr Field<C, M> field(std::string_view key, M C::*ptr, Presence p, std::string_view desc = std::string_view{}) {
  return Field<C, M>{key, ptr, p, desc, nullptr, 0, FieldConstraints{}};
}

/// Enum-constrained field: `values` must be an inline constexpr std::string_view array.
/// M must be std::string or std::optional<std::string>; JsonReader::check_enum only
/// fires for string members, so field_enum on any other type would silently skip
/// enforcement.
template <class C, class M, std::size_t N>
constexpr Field<C, M> field_enum(std::string_view key, M C::*ptr, const std::string_view (&values)[N],
                                 std::string_view desc = std::string_view{}) {
  static_assert(std::is_same_v<M, std::string> || std::is_same_v<M, std::optional<std::string>>,
                "field_enum requires a std::string or std::optional<std::string> member");
  return Field<C, M>{key, ptr, default_presence<M>(), desc, values, N, FieldConstraints{}};
}

// --- OpaqueObjectField ------------------------------------------------------

/// Binds a JSON key to a `nlohmann::json` struct member that carries an
/// "any-JSON-object whose shape is runtime-dependent" payload.
///
/// Visitor behaviour:
///   * JsonWriter  - writes the member value as-is (no introspection / no
///                   type tagging).
///   * JsonReader  - accepts any JSON object value; rejects scalars, arrays,
///                   and null with a FieldError; absent fields leave the
///                   member at its default (empty object).
///   * SchemaWriter - emits
///                   `{type:object, additionalProperties:true,
///                     x-medkit-opaque:true}` and marks the field required
///                   (opaque fields are not wrapped in std::optional).
///
/// Use for fields whose shape depends on runtime context: live ROS message
/// payloads, action results, plugin-provided data. NEVER brace-initialize
/// directly - construct via the opaque_object() factory.
template <class Class>
struct OpaqueObjectField {
  std::string_view key;
  nlohmann::json Class::*ptr;
  /// Detection tag picked up by `is_opaque_object_field` SFINAE.
  using opaque_object_tag = void;
};

/// Factory: declare a `nlohmann::json` member as an opaque any-object field.
/// See OpaqueObjectField above for visitor semantics.
template <class C>
constexpr OpaqueObjectField<C> opaque_object(std::string_view key, nlohmann::json C::*ptr) {
  return OpaqueObjectField<C>{key, ptr};
}

namespace detail {
template <class F, class = void>
struct is_opaque_object_field : std::false_type {};
template <class F>
struct is_opaque_object_field<F, std::void_t<typename F::opaque_object_tag>> : std::true_type {};
}  // namespace detail

/// True iff `F` is an OpaqueObjectField descriptor. Used by the three visitors
/// to dispatch between `Field` and `OpaqueObjectField` branches.
template <class F>
inline constexpr bool is_opaque_object_field_v = detail::is_opaque_object_field<std::decay_t<F>>::value;

// --- dto_fields / dto_name / is_dto -----------------------------------------

namespace detail {
/// Per-type marker used as the pointee of the primary (unspecialized)
/// `dto_fields<T>`. It exists purely so `is_dto_v<T>` can detect whether
/// a specialization was provided by comparing
/// `decltype(dto_fields<T>)` against `not_a_dto<T>*`.
///
/// The "missing dto_fields specialization" trap is NOT here - it lives in
/// `for_each_field` below, which is the only call site that consumes
/// `dto_fields<T>` as a value (i.e. where a static_assert can actually
/// fire). Keeping this struct empty means probe queries like
/// `is_dto_v<std::string>` (which never instantiate the body) stay
/// well-formed.
template <class T>
struct not_a_dto {};
}  // namespace detail

/// Primary template: a sentinel pointer. Specialize per DTO with
/// std::make_tuple(field(...), ...).
///
/// IMPORTANT: every dto_fields<X> / dto_name<X> specialization MUST be
/// co-located in the header that declares X. A TU that instantiates a visitor
/// before seeing the specialization would otherwise silently bind this
/// sentinel (latent ODR-adjacent bug); the static_assert inside
/// `for_each_field` surfaces that misuse as a compile error at the point
/// where a visitor actually tries to walk the (missing) field tuple.
///
/// IMPORTANT: a DTO must not transitively contain itself BY VALUE (infinite
/// template recursion) - use std::optional / std::vector / nlohmann::json for
/// any recursive shape.
///
/// IMPLEMENTATION NOTE: the primary holds a *pointer* to `not_a_dto<T>` rather
/// than a value. This keeps `decltype(dto_fields<T>)` well-formed for any T
/// without forcing instantiation of `detail::not_a_dto<T>`, and it gives
/// `is_dto_v<T>` a stable type identity to compare against.
template <class T>
inline constexpr detail::not_a_dto<T> * dto_fields = nullptr;

/// True iff dto_fields<T> was specialized (detected by the primary holding a
/// `not_a_dto<T>*` - any specialization replaces both the type and the value).
template <class T>
inline constexpr bool is_dto_v = !std::is_same_v<std::decay_t<decltype(dto_fields<T>)>, detail::not_a_dto<T> *>;

/// Opt-in marker for DTOs that bypass field-walking visitors and ship hand-
/// written `JsonWriter<T>` / `JsonReader<T>` / `SchemaWriter<T>` specializations
/// because their wire shape is runtime-dependent (e.g. plugin response
/// envelopes whose payload is opaque to the gateway). Specialize this to
/// `true` next to the type so framework templates that need a serializer
/// (the typed RouteRegistry, schema collector) can accept the type without
/// requiring a `dto_fields<T>` specialization.
template <class T>
inline constexpr bool is_opaque_dto_v = false;

/// True iff the type has a usable wire shape - either a regular field-walking
/// DTO (`is_dto_v<T>`) or an opaque DTO with hand-written visitor
/// specializations (`is_opaque_dto_v<T>`). The typed RouteRegistry uses this
/// to gate template instantiation on its response / body parameters.
template <class T>
inline constexpr bool has_dto_shape_v = is_dto_v<T> || is_opaque_dto_v<T>;

/// Schema name in components/schemas. Specialize per DTO with a string literal.
template <class T>
inline constexpr std::string_view dto_name = std::string_view{};

// --- field fold -------------------------------------------------------------

/// Invoke `v` once per field of T, in declaration order.
template <class T, class V>
void for_each_field(V && v) {
  static_assert(is_dto_v<T>,
                "DTO type used without including its dto_fields specialization header. "
                "Include the relevant dto/<domain>.hpp before instantiating "
                "JsonWriter/JsonReader/SchemaWriter for this type, or specialize "
                "dto_fields<T> if you are defining a new DTO.");
  std::apply(
      [&](auto &&... f) {
        (v(f), ...);
      },
      dto_fields<T>);
}

}  // namespace dto
}  // namespace ros2_medkit_gateway
