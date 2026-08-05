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

/// Unit tests for `openapi::unreachable_schemas`.
///
/// The rule it implements is asserted end-to-end against the served document by
/// `test_openapi_contract::test_no_unreachable_schemas`. These tests pin the
/// walk itself on documents small enough to reason about: multi-hop
/// reachability through `components/responses`, a cycle, and the cases where a
/// naive "does the name appear in the JSON text" check gives the wrong answer.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <set>
#include <string>

#include "ros2_medkit_gateway/core/openapi/document_checks.hpp"

using nlohmann::json;
using ros2_medkit_gateway::openapi::unreachable_schemas;

namespace {

json ref(const std::string & name) {
  return json{{"$ref", "#/components/schemas/" + name}};
}

/// A one-operation `paths` object whose 200 response body is `schema`.
json paths_returning(const json & schema) {
  json media = json::object();
  media["application/json"]["schema"] = schema;

  json response = json::object();
  response["description"] = "ok";
  response["content"] = media;

  json document = json::object();
  document["/things"]["get"]["responses"]["200"] = response;
  return document;
}

/// An object schema with one property pointing at `target`.
json object_with_property(const std::string & key, const json & target) {
  json schema = json::object();
  schema["type"] = "object";
  schema["properties"][key] = target;
  return schema;
}

const json kPlainObject = json{{"type", "object"}};

}  // namespace

TEST(SchemaReachability, ReportsASchemaNoOperationCanReach) {
  json document = json::object();
  document["paths"] = paths_returning(ref("Thing"));
  document["components"]["schemas"]["Thing"] = kPlainObject;
  document["components"]["schemas"]["Orphan"] = kPlainObject;

  EXPECT_EQ(unreachable_schemas(document), (std::set<std::string>{"Orphan"}));
}

TEST(SchemaReachability, FollowsRefsThroughComponentResponses) {
  // The blanket error responses are attached as a response-level $ref, so the
  // error body schema is two hops from any operation. A walk that only looked
  // at `paths` would call GenericError unreachable on every route in the real
  // document.
  json document = json::object();
  document["paths"]["/things"]["get"]["responses"]["400"] = json{{"$ref", "#/components/responses/GenericError"}};
  document["components"]["responses"]["GenericError"]["content"]["application/json"]["schema"] = ref("GenericError");
  document["components"]["schemas"]["GenericError"] = kPlainObject;

  EXPECT_TRUE(unreachable_schemas(document).empty());
}

TEST(SchemaReachability, FollowsRefsNestedInsideAReachedSchema) {
  json document = json::object();
  document["paths"] = paths_returning(ref("Outer"));
  document["components"]["schemas"]["Outer"] = object_with_property("inner", ref("Inner"));
  document["components"]["schemas"]["Inner"] = object_with_property("leaf", ref("Leaf"));
  document["components"]["schemas"]["Leaf"] = kPlainObject;

  EXPECT_TRUE(unreachable_schemas(document).empty());
}

TEST(SchemaReachability, ReachesEveryBranchOfAnAnyOf) {
  // The opaque envelopes publish `anyOf: [<in-tree shape>, <plugin shape>]`,
  // so a walk that stopped at the first `$ref` under a schema would report the
  // in-tree DTOs of every other branch as orphans.
  json envelope = json::object();
  envelope["anyOf"] = json::array({ref("First"), ref("Second"), kPlainObject});

  json document = json::object();
  document["paths"] = paths_returning(ref("Envelope"));
  document["components"]["schemas"]["Envelope"] = envelope;
  document["components"]["schemas"]["First"] = kPlainObject;
  document["components"]["schemas"]["Second"] = kPlainObject;

  EXPECT_TRUE(unreachable_schemas(document).empty());
}

TEST(SchemaReachability, TerminatesOnACycle) {
  json document = json::object();
  document["paths"] = paths_returning(ref("A"));
  document["components"]["schemas"]["A"] = object_with_property("b", ref("B"));
  document["components"]["schemas"]["B"] = object_with_property("a", ref("A"));

  EXPECT_TRUE(unreachable_schemas(document).empty());
}

TEST(SchemaReachability, DoesNotCountASchemaNamedOnlyInProse) {
  // A description that mentions `#/components/schemas/Orphan` is text, not a
  // reference. Matching the document's serialised text would wave this
  // through; walking the parsed tree for `$ref` keys does not.
  json document = json::object();
  document["paths"] = paths_returning(ref("Thing"));
  document["paths"]["/things"]["get"]["responses"]["200"]["description"] = "See #/components/schemas/Orphan";
  document["components"]["schemas"]["Thing"] = kPlainObject;
  document["components"]["schemas"]["Orphan"] = kPlainObject;

  EXPECT_EQ(unreachable_schemas(document), (std::set<std::string>{"Orphan"}));
}

TEST(SchemaReachability, ReportsASchemaReferencedOnlyByAnotherOrphan) {
  // Two dead schemas that reference each other are still dead. A "is this name
  // referenced anywhere" check would call both reachable.
  json document = json::object();
  document["paths"] = paths_returning(ref("Thing"));
  document["components"]["schemas"]["Thing"] = kPlainObject;
  document["components"]["schemas"]["OrphanA"] = object_with_property("b", ref("OrphanB"));
  document["components"]["schemas"]["OrphanB"] = kPlainObject;

  EXPECT_EQ(unreachable_schemas(document), (std::set<std::string>{"OrphanA", "OrphanB"}));
}

TEST(SchemaReachability, IsEmptyForADocumentWithoutComponents) {
  json document = json::object();
  document["paths"] = json::object();

  EXPECT_TRUE(unreachable_schemas(document).empty());
}

TEST(SchemaReachability, IgnoresADanglingRef) {
  // A $ref to a schema the document does not define is a different defect with
  // its own test (`test_every_ref_resolves`); this walk must not crash on it,
  // and must not let it mask a real orphan.
  json document = json::object();
  document["paths"] = paths_returning(ref("Missing"));
  document["components"]["schemas"]["Orphan"] = kPlainObject;

  EXPECT_EQ(unreachable_schemas(document), (std::set<std::string>{"Orphan"}));
}
