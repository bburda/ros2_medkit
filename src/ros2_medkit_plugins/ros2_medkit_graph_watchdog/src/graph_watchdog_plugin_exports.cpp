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
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/graph_watchdog_plugin.hpp"

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return ros2_medkit_gateway::PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT ros2_medkit_gateway::GatewayPlugin * create_plugin() {
  return new ros2_medkit_graph_watchdog::GraphWatchdogPlugin();
}

// Without this export the gateway never calls introspect(), the graph_watchdog App is
// never published, and every GRAPH_* fault goes back to being scoped to an entity that
// does not exist - reachable from no endpoint. The static_cast down to the concrete type
// is required: GatewayPlugin and IntrospectionProvider are unrelated bases, so the two
// subobject addresses differ.
extern "C" GATEWAY_PLUGIN_EXPORT ros2_medkit_gateway::IntrospectionProvider *
get_introspection_provider(ros2_medkit_gateway::GatewayPlugin * plugin) {
  return static_cast<ros2_medkit_graph_watchdog::GraphWatchdogPlugin *>(plugin);
}
