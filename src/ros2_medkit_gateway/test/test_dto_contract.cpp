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

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <string_view>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/dto/config.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/data.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/errors.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/registry.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/scripts.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace dto = ros2_medkit_gateway::dto;

namespace {
struct Sample {
  std::string id;
  std::optional<std::string> note;
};
inline constexpr std::string_view kSampleColors[] = {"red", "green"};

struct ScalarSample {
  std::string id;
  bool active;
  int count;
};

// DTO with a required and an optional enum-constrained field, to exercise the
// JsonReader enum-rejection path and the optional-enum schema shape.
struct EnumSample {
  std::string color;                 // required enum
  std::optional<std::string> shade;  // optional enum
};
}  // namespace

template <>
inline constexpr auto dto::dto_fields<Sample> =
    std::make_tuple(dto::field("id", &Sample::id), dto::field("note", &Sample::note));

template <>
inline constexpr std::string_view dto::dto_name<Sample> = "Sample";

template <>
[[maybe_unused]] inline constexpr auto dto::dto_fields<ScalarSample> =
    std::make_tuple(dto::field("id", &ScalarSample::id), dto::field("active", &ScalarSample::active),
                    dto::field("count", &ScalarSample::count));

template <>
[[maybe_unused]] inline constexpr std::string_view dto::dto_name<ScalarSample> = "ScalarSample";

template <>
[[maybe_unused]] inline constexpr auto dto::dto_fields<EnumSample> =
    std::make_tuple(dto::field_enum("color", &EnumSample::color, kSampleColors),
                    dto::field_enum("shade", &EnumSample::shade, kSampleColors));

template <>
[[maybe_unused]] inline constexpr std::string_view dto::dto_name<EnumSample> = "EnumSample";

TEST(DtoContract, FieldFactoryDerivesPresenceFromOptional) {
  constexpr auto fields = dto::dto_fields<Sample>;
  EXPECT_EQ(std::get<0>(fields).presence, dto::Presence::kRequired);
  EXPECT_EQ(std::get<1>(fields).presence, dto::Presence::kOptional);
  EXPECT_EQ(std::get<0>(fields).key, "id");
}

TEST(DtoContract, IsDtoDetectsSpecialization) {
  EXPECT_TRUE(dto::is_dto_v<Sample>);
  EXPECT_FALSE(dto::is_dto_v<std::string>);
  EXPECT_FALSE(dto::is_dto_v<int>);
}

TEST(DtoContract, ForEachFieldVisitsEveryField) {
  int count = 0;
  dto::for_each_field<Sample>([&](const auto & /*f*/) {
    ++count;
  });
  EXPECT_EQ(count, 2);
}

TEST(DtoContract, FieldEnumPopulatesVocabularyAndDtoName) {
  constexpr auto f = dto::field_enum("color", &Sample::id, kSampleColors);
  EXPECT_EQ(f.enum_count, 2u);
  EXPECT_EQ(f.enum_values[0], "red");
  EXPECT_EQ(dto::dto_name<Sample>, "Sample");
}

TEST(JsonReader, RejectsValueOutsideEnum) {
  const nlohmann::json j = {{"color", "blue"}};  // "blue" is not in kSampleColors
  const auto result = dto::JsonReader<EnumSample>::read(j);
  ASSERT_FALSE(result.has_value());
  ASSERT_FALSE(result.error().empty());
  EXPECT_EQ(result.error()[0].field, "color");
  EXPECT_EQ(result.error()[0].message, "value not in allowed set");
}

TEST(JsonReader, AcceptsValueInsideEnum) {
  const nlohmann::json j = {{"color", "green"}};
  const auto result = dto::JsonReader<EnumSample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->color, "green");
}

TEST(SchemaWriter, OptionalEnumAttachesEnumToNonNullBranch) {
  const auto schema = dto::SchemaWriter<EnumSample>::schema();
  const auto & props = schema.at("properties");

  // Required enum field: enum sits at the top level of a {type:string} schema.
  const auto & color = props.at("color");
  ASSERT_TRUE(color.contains("enum"));
  EXPECT_EQ(color.at("enum"), nlohmann::json::array({"red", "green"}));

  // Optional enum field: schema is anyOf:[{string}, {null}]; the enum must live
  // on the non-null branch ([0]), NOT at the top level, so a null value (which
  // anyOf permits) is not rejected by a top-level enum that omits null.
  const auto & shade = props.at("shade");
  ASSERT_TRUE(shade.contains("anyOf"));
  EXPECT_FALSE(shade.contains("enum"));
  EXPECT_EQ(shade.at("anyOf")[0].at("enum"), nlohmann::json::array({"red", "green"}));
}

TEST(SchemaWriter, BuildsObjectSchemaWithProperties) {
  const auto schema = dto::SchemaWriter<Sample>::schema();
  EXPECT_EQ(schema.at("type"), "object");
  EXPECT_TRUE(schema.at("properties").contains("id"));
  EXPECT_TRUE(schema.at("properties").contains("note"));
  EXPECT_EQ(schema.at("properties").at("id").at("type"), "string");
}

TEST(SchemaWriter, RequiredListExcludesOptionalFields) {
  const auto schema = dto::SchemaWriter<Sample>::schema();
  const auto & req = schema.at("required");
  EXPECT_NE(std::find(req.begin(), req.end(), "id"), req.end());
  EXPECT_EQ(std::find(req.begin(), req.end(), "note"), req.end());
}

TEST(SchemaWriter, OptionalFieldEmitsAnyOfWithNull) {
  // OpenAPI 3.1 idiom for nullable scalar: {"anyOf": [<inner>, {"type": "null"}]}.
  // Generated clients can then express T | null instead of degrading to T | undefined.
  const auto schema = dto::SchemaWriter<Sample>::schema();
  const auto & note_prop = schema.at("properties").at("note");
  ASSERT_TRUE(note_prop.contains("anyOf")) << note_prop.dump();
  const auto & any_of = note_prop.at("anyOf");
  ASSERT_TRUE(any_of.is_array());
  ASSERT_EQ(any_of.size(), 2u);
  EXPECT_EQ(any_of[0].at("type"), "string");
  EXPECT_EQ(any_of[1].at("type"), "null");
}

TEST(JsonWriter, WritesRequiredFieldsAndSkipsEmptyOptional) {
  Sample s{"area_1", std::nullopt};
  const auto j = dto::JsonWriter<Sample>::write(s);
  EXPECT_EQ(j.at("id"), "area_1");
  EXPECT_FALSE(j.contains("note"));
}

TEST(JsonWriter, WritesPresentOptional) {
  Sample s{"area_1", std::string{"hello"}};
  const auto j = dto::JsonWriter<Sample>::write(s);
  EXPECT_EQ(j.at("note"), "hello");
}

TEST(JsonReader, DecodesValidObject) {
  const auto j = nlohmann::json{{"id", "x"}, {"note", "n"}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "x");
  ASSERT_TRUE(result->note.has_value());
  EXPECT_EQ(*result->note, "n");
}

TEST(JsonReader, ReportsMissingRequiredField) {
  const auto j = nlohmann::json{{"note", "n"}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1u);
  EXPECT_EQ(result.error()[0].field, "id");
}

TEST(JsonReader, ReportsWrongType) {
  const auto j = nlohmann::json{{"id", 123}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error()[0].field, "id");
}

TEST(JsonReader, IgnoresUnknownFields) {
  const auto j = nlohmann::json{{"id", "x"}, {"bogus", 1}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "x");
}

TEST(DtoSample, RoundTripsThroughWriterAndReader) {
  const Sample s = dto::make_sample<Sample>();
  const auto j = dto::JsonWriter<Sample>::write(s);
  const auto back = dto::JsonReader<Sample>::read(j);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->id, s.id);
}

TEST(DtoSample, SampleContainsEveryRequiredSchemaKey) {
  const Sample s = dto::make_sample<Sample>();
  const auto j = dto::JsonWriter<Sample>::write(s);
  const auto schema = dto::SchemaWriter<Sample>::schema();
  for (const auto & req : schema.at("required")) {
    EXPECT_TRUE(j.contains(req.get<std::string>())) << req;
  }
}

TEST(DtoSample, SynthesizesScalarMembers) {
  const ScalarSample s = dto::make_sample<ScalarSample>();
  const auto j = dto::JsonWriter<ScalarSample>::write(s);
  const auto back = dto::JsonReader<ScalarSample>::read(j);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->id, s.id);
  EXPECT_EQ(back->active, s.active);
  EXPECT_EQ(back->count, s.count);
}

// A present-but-null JSON value is a legitimate value for a free-form json
// member (setting a parameter/value to null), distinct from an absent field.
TEST(JsonReaderNull, ConfigWriteAcceptsExplicitNullData) {
  const auto j = nlohmann::json{{"data", nullptr}};
  const auto result = dto::JsonReader<dto::ConfigurationWriteRequest>::read(j);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->data.has_value());
  EXPECT_TRUE(result->data->is_null());
  EXPECT_FALSE(result->value.has_value());  // a genuinely absent optional stays absent
}

TEST(JsonReaderNull, DataWriteAcceptsExplicitNullData) {
  const auto j = nlohmann::json{{"type", "std_msgs/msg/Float32"}, {"data", nullptr}};
  const auto result = dto::JsonReader<dto::DataWriteRequest>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->data.is_null());
}

TEST(JsonReaderNull, NullOnNonJsonRequiredFieldIsTreatedAsMissing) {
  // For a non-json member null cannot be coerced, so it is treated like an
  // absent field - a required one therefore reports "missing required field".
  const auto j = nlohmann::json{{"id", nullptr}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1U);
  EXPECT_EQ(result.error()[0].field, "id");
}

// ScriptControlRequest.action uses plain field() so plugin backends may accept
// actions beyond the built-in stop/forced_termination; the value is validated
// by the provider, not rejected at parse time.
TEST(JsonReaderScripts, ControlActionAcceptsArbitraryValue) {
  const auto j = nlohmann::json{{"action", "pause"}};
  const auto result = dto::JsonReader<dto::ScriptControlRequest>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->action, "pause");
}

TEST(JsonReaderScripts, ControlActionStillRequiresPresence) {
  const auto result = dto::JsonReader<dto::ScriptControlRequest>::read(nlohmann::json::object());
  EXPECT_FALSE(result.has_value());
}

TEST(DtoRegistry, CollectsNamedSchemas) {
  const auto schemas = dto::collect_component_schemas();
  EXPECT_TRUE(schemas.is_object());
}

TEST(DtoErrors, GenericErrorIsDtoWithCorrectFields) {
  EXPECT_TRUE(dto::is_dto_v<dto::GenericError>);
  EXPECT_EQ(dto::dto_name<dto::GenericError>, "GenericError");

  const auto schema = dto::SchemaWriter<dto::GenericError>::schema();
  EXPECT_EQ(schema.at("type"), "object");
  EXPECT_TRUE(schema.at("properties").contains("error_code"));
  EXPECT_TRUE(schema.at("properties").contains("message"));
  EXPECT_TRUE(schema.at("properties").contains("parameters"));

  // error_code and message are required; parameters is optional
  const auto & req = schema.at("required");
  EXPECT_NE(std::find(req.begin(), req.end(), "error_code"), req.end());
  EXPECT_NE(std::find(req.begin(), req.end(), "message"), req.end());
  EXPECT_EQ(std::find(req.begin(), req.end(), "parameters"), req.end());
}

TEST(DtoErrors, GenericErrorRoundTrips) {
  dto::GenericError e{"x-medkit-entity-not-found", "Entity not found", std::nullopt};
  const auto j = dto::JsonWriter<dto::GenericError>::write(e);
  EXPECT_EQ(j.at("error_code"), "x-medkit-entity-not-found");
  EXPECT_EQ(j.at("message"), "Entity not found");
  EXPECT_FALSE(j.contains("parameters"));
  // Unset vendor_code stays off the wire, so a non-vendor error is unchanged
  // by the field's existence.
  EXPECT_FALSE(j.contains("vendor_code"));
}

TEST(DtoErrors, GenericErrorCarriesTheVendorCode) {
  // The shape `write_generic_error` puts on the wire for an x-medkit-* code:
  // the sentinel in error_code, the real code alongside it. The schema has to
  // describe both keys or a client sees every vendor failure as one error.
  dto::GenericError e{ros2_medkit_gateway::ERR_VENDOR_ERROR, "Entity not found", std::nullopt};
  e.vendor_code = "x-medkit-entity-not-found";
  const auto j = dto::JsonWriter<dto::GenericError>::write(e);
  EXPECT_EQ(j.at("error_code"), "vendor-error");
  EXPECT_EQ(j.at("vendor_code"), "x-medkit-entity-not-found");

  const auto schema = dto::SchemaWriter<dto::GenericError>::schema();
  EXPECT_TRUE(schema.at("properties").contains("vendor_code"));
  const auto & required = schema.at("required");
  EXPECT_EQ(std::find(required.begin(), required.end(), "vendor_code"), required.end());
}

TEST(DtoEnums, EntityTypeVocabularyHasFourValues) {
  EXPECT_EQ(std::size(dto::kEntityTypeValues), 4u);
  EXPECT_EQ(dto::kEntityTypeValues[0], "area");
  EXPECT_EQ(dto::kEntityTypeValues[3], "function");
}

TEST(DtoEnums, CyclicSubscriptionIntervalHasThreeValues) {
  EXPECT_EQ(std::size(dto::kCyclicSubscriptionIntervalValues), 3u);
}

// =============================================================================
// XMedkit DTOs
// =============================================================================

TEST(XMedkitDtos, XMedkitRos2IsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitRos2>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitRos2>, "XMedkitRos2");
}

TEST(XMedkitDtos, XMedkitRos2NamespaceKeyIsMappedCorrectly) {
  // The C++ member is `ns` but the wire key must be "namespace".
  dto::XMedkitRos2 r2;
  r2.ns = "/sensors";
  const auto j = dto::JsonWriter<dto::XMedkitRos2>::write(r2);
  EXPECT_TRUE(j.contains("namespace"));
  EXPECT_EQ(j.at("namespace"), "/sensors");
  EXPECT_FALSE(j.contains("ns"));
}

TEST(XMedkitDtos, XMedkitAreaNestedRos2IsSerializedCorrectly) {
  dto::XMedkitArea area;
  dto::XMedkitRos2 r2;
  r2.ns = "/perception";
  area.ros2 = r2;
  area.contributors = std::vector<std::string>{"local", "peer:robot2"};

  const auto j = dto::JsonWriter<dto::XMedkitArea>::write(area);

  // Nested "ros2" object must be present with the "namespace" sub-key.
  ASSERT_TRUE(j.contains("ros2"));
  EXPECT_TRUE(j.at("ros2").contains("namespace"));
  EXPECT_EQ(j.at("ros2").at("namespace"), "/perception");

  // optional parent_area_id is absent when not set
  EXPECT_FALSE(j.contains("parent_area_id"));

  // contributors are serialized
  ASSERT_TRUE(j.contains("contributors"));
  EXPECT_EQ(j.at("contributors").size(), 2u);
}

TEST(XMedkitDtos, XMedkitAreaSchemaHasRos2RefProperty) {
  const auto schema = dto::SchemaWriter<dto::XMedkitArea>::schema();
  ASSERT_EQ(schema.at("type"), "object");
  ASSERT_TRUE(schema.at("properties").contains("ros2"));
  // ros2 is std::optional<XMedkitRos2>: OpenAPI 3.1 anyOf+null idiom wraps the
  // $ref. Inner branch is the $ref to XMedkitRos2; null branch is {"type":"null"}.
  const auto & ros2_prop = schema.at("properties").at("ros2");
  ASSERT_TRUE(ros2_prop.contains("anyOf")) << ros2_prop.dump();
  const auto & any_of = ros2_prop.at("anyOf");
  ASSERT_EQ(any_of.size(), 2u);
  EXPECT_EQ(any_of[0].at("$ref"), "#/components/schemas/XMedkitRos2");
  EXPECT_EQ(any_of[1].at("type"), "null");
}

TEST(XMedkitDtos, XMedkitComponentIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitComponent>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitComponent>, "XMedkitComponent");
}

TEST(XMedkitDtos, XMedkitComponentCamelCaseWireKeys) {
  dto::XMedkitComponent comp;
  comp.parent_component_id = "parent_comp";
  comp.depends_on = std::vector<std::string>{"dep_a", "dep_b"};

  const auto j = dto::JsonWriter<dto::XMedkitComponent>::write(comp);

  EXPECT_TRUE(j.contains("parentComponentId"));
  EXPECT_EQ(j.at("parentComponentId"), "parent_comp");
  EXPECT_TRUE(j.contains("dependsOn"));
  EXPECT_EQ(j.at("dependsOn").size(), 2u);
}

TEST(XMedkitDtos, XMedkitAppIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitApp>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitApp>, "XMedkitApp");
}

TEST(XMedkitDtos, XMedkitAppRoundTrip) {
  dto::XMedkitApp app;
  dto::XMedkitRos2 r2;
  r2.node = "/sensors/camera";
  app.ros2 = r2;
  app.source = "runtime";
  app.is_online = true;
  app.component_id = "sensors_comp";

  const auto j = dto::JsonWriter<dto::XMedkitApp>::write(app);
  EXPECT_EQ(j.at("source"), "runtime");
  EXPECT_EQ(j.at("is_online"), true);
  EXPECT_EQ(j.at("component_id"), "sensors_comp");
  ASSERT_TRUE(j.contains("ros2"));
  EXPECT_EQ(j.at("ros2").at("node"), "/sensors/camera");
}

TEST(XMedkitDtos, XMedkitFunctionIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitFunction>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitFunction>, "XMedkitFunction");
}

TEST(XMedkitDtos, XMedkitFunctionHostsSerializedAsArray) {
  dto::XMedkitFunction func;
  func.source = "manifest";
  func.hosts = std::vector<std::string>{"app_a", "app_b"};
  func.description = "Navigation function";

  const auto j = dto::JsonWriter<dto::XMedkitFunction>::write(func);
  EXPECT_EQ(j.at("source"), "manifest");
  ASSERT_TRUE(j.contains("hosts"));
  EXPECT_EQ(j.at("hosts").size(), 2u);
  EXPECT_EQ(j.at("description"), "Navigation function");
}

TEST(XMedkitDtos, XMedkitCollectionIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitCollection>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitCollection>, "XMedkitCollection");
}

TEST(XMedkitDtos, XMedkitCollectionTotalCount) {
  dto::XMedkitCollection col;
  col.total_count = 42u;

  const auto j = dto::JsonWriter<dto::XMedkitCollection>::write(col);
  ASSERT_TRUE(j.contains("total_count"));
  EXPECT_EQ(j.at("total_count"), 42u);
  EXPECT_FALSE(j.contains("contributors"));
}

TEST(XMedkitDtos, AllXMedkitSchemasAreObjects) {
  const auto area_schema = dto::SchemaWriter<dto::XMedkitArea>::schema();
  const auto comp_schema = dto::SchemaWriter<dto::XMedkitComponent>::schema();
  const auto app_schema = dto::SchemaWriter<dto::XMedkitApp>::schema();
  const auto func_schema = dto::SchemaWriter<dto::XMedkitFunction>::schema();
  const auto coll_schema = dto::SchemaWriter<dto::XMedkitCollection>::schema();
  const auto ros2_schema = dto::SchemaWriter<dto::XMedkitRos2>::schema();

  EXPECT_EQ(area_schema.at("type"), "object");
  EXPECT_EQ(comp_schema.at("type"), "object");
  EXPECT_EQ(app_schema.at("type"), "object");
  EXPECT_EQ(func_schema.at("type"), "object");
  EXPECT_EQ(coll_schema.at("type"), "object");
  EXPECT_EQ(ros2_schema.at("type"), "object");
}

// =============================================================================
// All-DTO registry round-trip test
// =============================================================================

/// Every DTO schema describes a JSON object body, one of two ways: a plain
/// `type: object`, or an `anyOf` whose every branch is one. The union form is
/// what the pass-through envelopes publish - a route that answers with an
/// in-tree DTO for a ROS 2-backed entity and with a plugin's own JSON for a
/// plugin-owned one has two shapes, and collapsing them to `{type: object}`
/// (which is what those schemas used to say) told a client nothing about
/// either. The assertion still rejects a scalar or array body.
void expect_object_shaped(const nlohmann::json & schema, std::string_view name) {
  if (schema.contains("anyOf")) {
    ASSERT_TRUE(schema.at("anyOf").is_array()) << name;
    ASSERT_FALSE(schema.at("anyOf").empty()) << name;
    for (const auto & branch : schema.at("anyOf")) {
      // A $ref branch names a schema this same check covers on its own turn.
      const bool object_branch = branch.contains("$ref") || (branch.contains("type") && branch.at("type") == "object");
      EXPECT_TRUE(object_branch) << name << " anyOf branch is not an object: " << branch.dump();
    }
    return;
  }
  EXPECT_EQ(schema.at("type"), "object") << name;
}

template <class Tuple, std::size_t I>
void check_one() {
  using D = std::tuple_element_t<I, Tuple>;
  EXPECT_FALSE(dto::dto_name<D>.empty()) << "DTO at index " << I;
  const auto schema = dto::SchemaWriter<D>::schema();
  expect_object_shaped(schema, dto::dto_name<D>);

  // Value-equality round-trip via double-write compare.
  //
  // A weaker `has_value()` check would only catch reader exceptions; it would
  // silently accept the case where the reader drops a field that the writer
  // emitted (e.g. a typo in the field key, a missing visitor branch for a new
  // member). Writing -> reading -> writing again and comparing the two JSON
  // objects forces every emitted field to survive the round-trip.
  const D s = dto::make_sample<D>();
  const auto j1 = dto::JsonWriter<D>::write(s);
  auto parsed = dto::JsonReader<D>::read(j1);
  ASSERT_TRUE(parsed.has_value()) << dto::dto_name<D>;
  const auto j2 = dto::JsonWriter<D>::write(parsed.value());
  EXPECT_EQ(j1, j2) << dto::dto_name<D> << " round-trip lossy";
}

template <class Tuple, std::size_t... I>
void check_all(std::index_sequence<I...> /*seq*/) {
  (check_one<Tuple, I>(), ...);
}

TEST(DtoRegistry, EveryRegisteredDtoRoundTrips) {
  check_all<dto::AllDtos>(std::make_index_sequence<std::tuple_size_v<dto::AllDtos>>{});
}

// =============================================================================
// external field contract (#516)
// =============================================================================

TEST(XMedkitExternalContract, ComponentEmitsExternalWhenSet) {
  dto::XMedkitComponent c;
  c.external = true;
  auto j = dto::JsonWriter<dto::XMedkitComponent>::write(c);
  ASSERT_TRUE(j.contains("external"));
  EXPECT_EQ(j["external"], true);
}

TEST(XMedkitExternalContract, ComponentOmitsExternalWhenUnset) {
  dto::XMedkitComponent c;  // external nullopt
  auto j = dto::JsonWriter<dto::XMedkitComponent>::write(c);
  EXPECT_FALSE(j.contains("external"));
}

TEST(XMedkitExternalContract, AppEmitsExternalWhenSet) {
  dto::XMedkitApp a;
  a.external = true;
  auto j = dto::JsonWriter<dto::XMedkitApp>::write(a);
  ASSERT_TRUE(j.contains("external"));
  EXPECT_EQ(j["external"], true);
}

TEST(XMedkitExternalContract, AppOmitsExternalWhenUnset) {
  dto::XMedkitApp a;  // external nullopt
  auto j = dto::JsonWriter<dto::XMedkitApp>::write(a);
  EXPECT_FALSE(j.contains("external"));
}
