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
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

template <class U>
nlohmann::json schema_of();  // forward declaration

namespace detail {
template <class Variant, std::size_t... I>
nlohmann::json variant_schema(std::index_sequence<I...> /*seq*/) {
  return nlohmann::json{{"oneOf", nlohmann::json::array({schema_of<std::variant_alternative_t<I, Variant>>()...})}};
}
}  // namespace detail

/// Map a C++ value type to its OpenAPI JSON Schema fragment.
template <class U>
nlohmann::json schema_of() {
  if constexpr (is_dto_v<U>) {
    static_assert(!dto_name<U>.empty(),
                  "schema_of: DTO type is missing a dto_name<T> specialization "
                  "(would emit an empty \"$ref\" and a \"\" schema key)");
    return nlohmann::json{{"$ref", "#/components/schemas/" + std::string(dto_name<U>)}};
  } else if constexpr (is_optional_v<U>) {
    auto inner = schema_of<typename U::value_type>();
    return nlohmann::json{{"anyOf", nlohmann::json::array({inner, {{"type", "null"}}})}};
  } else if constexpr (is_vector_v<U>) {
    return nlohmann::json{{"type", "array"}, {"items", schema_of<typename U::value_type>()}};
  } else if constexpr (is_variant_v<U>) {
    return detail::variant_schema<U>(std::make_index_sequence<std::variant_size_v<U>>{});
  } else if constexpr (std::is_same_v<U, nlohmann::json>) {
    return nlohmann::json::object();  // {} = any JSON
  } else if constexpr (std::is_same_v<U, std::string>) {
    return nlohmann::json{{"type", "string"}};
  } else if constexpr (std::is_same_v<U, bool>) {
    return nlohmann::json{{"type", "boolean"}};
  } else if constexpr (std::is_integral_v<U>) {
    nlohmann::json integer{{"type", "integer"}};
    // `format: int64` asserts a signed 64-bit range, so it is derived from
    // signedness as well as width: `sizeof(U) == 8` alone also matches
    // `uint64_t` and `std::size_t`, whose upper half the format excludes.
    if constexpr (sizeof(U) == 8 && std::is_signed_v<U>) {
      integer["format"] = "int64";
    }
    return integer;
  } else if constexpr (std::is_floating_point_v<U>) {
    return nlohmann::json{{"type", "number"}};
  } else {
    static_assert(sizeof(U) == 0, "schema_of: unsupported field type");
    return {};
  }
}

/// The object schema derived from `dto_fields<T>`, with no type-specific
/// overrides applied.
///
/// Exposed separately from `SchemaWriter<T>` so a specialisation can start from
/// the derived schema and replace one property, instead of hand-writing the
/// whole thing and drifting from the descriptor the writer and reader use.
template <class T>
nlohmann::json derived_object_schema() {
  nlohmann::json props = nlohmann::json::object();
  nlohmann::json required = nlohmann::json::array();
  for_each_field<T>([&](const auto & f) {
    using FieldT = std::decay_t<decltype(f)>;
    if constexpr (is_opaque_object_field_v<FieldT>) {
      // Opaque any-object: fixed schema fragment, always required.
      props[std::string(f.key)] =
          nlohmann::json{{"type", "object"}, {"additionalProperties", true}, {"x-medkit-opaque", true}};
      required.push_back(std::string(f.key));
    } else {
      using MemberT = std::decay_t<decltype(std::declval<T>().*(f.ptr))>;
      nlohmann::json prop = schema_of<MemberT>();
      // The description stays at the property level even for a nullable
      // member: it describes the field, not one branch of its schema, and a
      // reader shows it either way. Everything below is a *validation*
      // keyword, which is why it goes somewhere else.
      if (!f.description.empty()) {
        prop["description"] = std::string(f.description);
      }
      // For optional members schema_of() yields {anyOf:[<inner>, {null}]}.
      // A validation keyword written at the property level would apply to the
      // null branch too and reject the null that anyOf advertises, so it goes
      // on the non-null branch ([0]). Required members have no anyOf and take
      // it at the top level. One lambda for the enum and every constraint, so
      // the two placements cannot drift apart.
      auto attach = [&prop](const char * keyword, nlohmann::json value) {
        if (prop.contains("anyOf") && prop["anyOf"].is_array() && !prop["anyOf"].empty()) {
          prop["anyOf"][0][keyword] = std::move(value);
        } else {
          prop[keyword] = std::move(value);
        }
      };
      if (f.enum_count > 0) {
        nlohmann::json values = nlohmann::json::array();
        for (std::size_t i = 0; i < f.enum_count; ++i) {
          values.push_back(std::string(f.enum_values[i]));
        }
        attach("enum", values);
      }
      if (f.constraints.minimum.has_value()) {
        attach("minimum", *f.constraints.minimum);
      }
      if (f.constraints.maximum.has_value()) {
        attach("maximum", *f.constraints.maximum);
      }
      if (f.constraints.max_length.has_value()) {
        attach("maxLength", *f.constraints.max_length);
      }
      if (!f.constraints.pattern.empty()) {
        attach("pattern", std::string(f.constraints.pattern));
      }
      // Last, so an explicit format overrides the one derived from the member
      // type (`int64` for a signed 64-bit integral).
      if (!f.constraints.format.empty()) {
        attach("format", std::string(f.constraints.format));
      }
      props[std::string(f.key)] = prop;
      if (f.presence == Presence::kRequired) {
        required.push_back(std::string(f.key));
      }
    }
  });
  nlohmann::json schema = {{"type", "object"}, {"properties", props}};
  if (!required.empty()) {
    schema["required"] = required;
  }
  return schema;
}

/// Generates the components/schemas object entry for a DTO type T.
template <class T>
struct SchemaWriter {
  static nlohmann::json schema() {
    return derived_object_schema<T>();
  }
};

}  // namespace dto
}  // namespace ros2_medkit_gateway
