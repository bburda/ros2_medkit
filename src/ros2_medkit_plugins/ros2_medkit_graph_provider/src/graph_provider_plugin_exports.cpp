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

#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/openapi/route_descriptions.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_graph_provider/graph_provider_plugin.hpp"

using namespace ros2_medkit_gateway;

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new GraphProviderPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin) {
  return static_cast<GraphProviderPlugin *>(plugin);
}

namespace {

using openapi::OperationDesc;
using openapi::SchemaDesc;

// Every property below was read out of `build_graph_document_for_apps` and
// `build_edge_json` in graph_provider_plugin.cpp, not out of the tutorial:
// what is in `required` is what those two functions write unconditionally,
// and what is left out is what they write only on some branch. A key that is
// always present but sometimes JSON `null` is `or_null()`, not optional -
// the two say different things to a generated client.

SchemaDesc edge_metrics_schema() {
  return SchemaDesc::object()
      .property("frequency_hz", SchemaDesc::number().or_null().description(
                                    "Publish rate observed on the topic, in hertz. `null` while no rate has been "
                                    "observed, and also when several publishers make the measured rate ambiguous "
                                    "and the `multi_publisher_rate` policy is `suppress` - in which case "
                                    "`rate_ambiguous` is set."))
      .property("latency_ms", SchemaDesc::number().or_null().description(
                                  "End-to-end latency reported for the topic, in milliseconds. `null` until a "
                                  "producer reports one."))
      .property("drop_rate_percent",
                SchemaDesc::number().description(
                    "Fraction of messages reported dropped, in percent. 0 when nothing has been reported."))
      .property("metrics_status",
                SchemaDesc::string()
                    .enum_values({"pending", "active", "error"})
                    .description("`pending` before any metrics arrive for the topic, `active` while they are fresh, "
                                 "`error` once they have been stale for longer than the freshness window plus its "
                                 "grace period. `error` is what makes the edge count towards a `broken` pipeline."))
      .property("error_reason", SchemaDesc::string().description(
                                    "Why `metrics_status` is `error`. Present only on an edge in that state."))
      .property("publisher_count",
                SchemaDesc::integer().description(
                    "Live publishers on the data topic, from the ROS 2 graph. Omitted when the query did not "
                    "resolve - never reported as 0 to stand in for 'unknown'."))
      .property("rate_ambiguous",
                SchemaDesc::boolean().description(
                    "Present and true when more than one publisher makes `frequency_hz` untrustworthy. Not by "
                    "itself a failure: it does not change the edge's verdict."))
      .property("source", SchemaDesc::string().description(
                              "ROS 2 node that published the `/diagnostics` metrics for this topic. Omitted when "
                              "the publisher could not be resolved."))
      .required({"frequency_hz", "latency_ms", "drop_rate_percent", "metrics_status"});
}

SchemaDesc edge_schema() {
  return SchemaDesc::object()
      .property("edge_id", SchemaDesc::string().description(
                               "Identifier of this edge within this document. Stable only for the lifetime of one "
                               "response - `bottleneck_edge` refers to it."))
      .property("source", SchemaDesc::string().description("Entity id of the publishing App."))
      .property("target", SchemaDesc::string().description("Entity id of the subscribing App."))
      .property("topic_id", SchemaDesc::string().description("Identifier of the topic in this document's `topics`."))
      .property("transport_type",
                SchemaDesc::string().description(
                    "Always `unknown`: the plugin does not resolve the DDS transport behind an edge."))
      .property("metrics", edge_metrics_schema())
      .required({"edge_id", "source", "target", "topic_id", "transport_type", "metrics"});
}

SchemaDesc node_schema() {
  return SchemaDesc::object()
      .property("entity_id", SchemaDesc::string().description("Entity id of the App this node stands for."))
      .property("node_status",
                SchemaDesc::string()
                    .enum_values({"reachable", "unreachable"})
                    .description("Whether the App is currently in the ROS 2 graph. An unreachable node publishes and "
                                 "subscribes to nothing, so it contributes no edge - which is why it drags the "
                                 "pipeline to `degraded` on its own."))
      .property("last_seen", SchemaDesc::string().description(
                                 "When the App was last seen online, as `YYYY-MM-DDTHH:MM:SS.mmmZ`. Present only on "
                                 "an unreachable node the plugin has seen before."))
      .required({"entity_id", "node_status"});
}

SchemaDesc topic_schema() {
  return SchemaDesc::object()
      .property("topic_id", SchemaDesc::string().description("Identifier used by this document's edges."))
      .property("name", SchemaDesc::string().description("ROS 2 topic name."))
      .required({"topic_id", "name"});
}

SchemaDesc graph_schema() {
  auto scope = SchemaDesc::object()
                   .property("type", SchemaDesc::string().description(
                                         "Always `function`: a graph document is scoped to one SOVD Function."))
                   .property("entity_id", SchemaDesc::string().description("Id of that Function."))
                   .required({"type", "entity_id"});

  return SchemaDesc::object()
      .property("schema_version",
                SchemaDesc::string().description("Version of this document's shape, independent of the gateway's."))
      .property("graph_id", SchemaDesc::string().description("`<function_id>-graph`."))
      .property("timestamp",
                SchemaDesc::string().description("When the graph was built, as `YYYY-MM-DDTHH:MM:SS.mmmZ` in UTC."))
      .property("scope", scope)
      .property("pipeline_status",
                SchemaDesc::string()
                    .enum_values({"healthy", "degraded", "broken"})
                    .description("`broken` if any edge's metrics are stale, `degraded` if an edge is below its "
                                 "expected rate or over its drop-rate threshold or a scoped node is unreachable, "
                                 "`healthy` otherwise."))
      .property("bottleneck_edge", SchemaDesc::string().or_null().description(
                                       "`edge_id` of the edge furthest below its expected rate. Non-null only while "
                                       "`pipeline_status` is `degraded` and a rate ratio was computed."))
      .property("topics", SchemaDesc::array(topic_schema()).description("Topics connecting the Apps in scope."))
      .property("nodes", SchemaDesc::array(node_schema()).description("Apps in scope, one node each."))
      .property(
          "edges",
          SchemaDesc::array(edge_schema()).description("One edge per publisher/subscriber pair on a shared topic."))
      .required({"schema_version", "graph_id", "timestamp", "scope", "pipeline_status", "bottleneck_edge", "topics",
                 "nodes", "edges"});
}

}  // namespace

// The gateway resolves this symbol with dlsym and folds what it returns into
// the OpenAPI document it serves. Without it a route this plugin mounts is
// reachable but undiscoverable: the gateway's `RouteRegistry` knows nothing
// about plugin routes, so nothing else in the document mentions them.
//
// The `admin` role is not a preference. A plugin route is mounted straight onto
// the HTTP server, so the gateway's `RouteRegistry` never sees it and no
// `requires_role(...)` on a registration derives a permission entry for it. What
// covers it is `AuthConfig::residual_route_permissions()`, and that list is
// ADMIN's four `**` wildcards alone - a weaker role's entry would have to match
// `/api/v1/functions/{id}/x-medkit-graph` segment by segment, and none does.
// Declaring `viewer` here would publish a role the gateway answers 403 to.
extern "C" GATEWAY_PLUGIN_EXPORT openapi::RouteDescriptions describe_plugin_routes() {
  openapi::RouteDescriptionBuilder builder;

  OperationDesc get_graph;
  get_graph.tag("Graph")
      .operation_id("getFunctionGraph")
      .requires_role("admin")
      .description(
          "Returns the dataflow graph of one SOVD Function: its Apps as nodes, the topics between them as edges, "
          "and per-edge throughput, latency and drop-rate metrics sourced from `/diagnostics`. The document is "
          "built on demand from the live ROS 2 graph, so two calls a second apart can differ. Supports cyclic "
          "subscriptions: point a subscription at this resource to receive the same payload periodically.")
      .path_param("function_id", "The function identifier")
      .response(200,
                SchemaDesc::object()
                    .property("x-medkit-graph", graph_schema())
                    .required({"x-medkit-graph"})
                    .description("Vendor extension envelope, so the payload can be told apart from a SOVD "
                                 "standard resource by key."),
                "Dataflow graph for the function")
      // 400 malformed function id, or an id naming an entity that is not a
      // Function; 404 no such Function - both from
      // `PluginContext::validate_entity_for_route`. 409 when another client
      // holds a lock covering this collection. 503 when the plugin has no
      // context yet, or when no graph snapshot could be built for the
      // function. 500 when the handler throws - `PluginManager` catches it
      // and answers.
      .error_response(400, "GenericError")
      .error_response(404, "GenericError")
      .error_response(409, "GenericError")
      .error_response(500, "GenericError")
      .error_response(503, "GenericError");

  builder.add("/functions/{function_id}/x-medkit-graph")
      .summary("Get function dataflow graph")
      .get(std::move(get_graph));

  return builder.build();
}
