// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/address_space_browser.hpp"
#include "ros2_medkit_opcua/network_discovery.hpp"
#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"
#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <ros2_medkit_gateway/core/discovery/models/asset_identity.hpp>
#include <ros2_medkit_gateway/core/plugins/gateway_plugin.hpp>
#include <ros2_medkit_gateway/core/plugins/plugin_http_types.hpp>
#include <ros2_medkit_gateway/core/providers/data_provider.hpp>
#include <ros2_medkit_gateway/core/providers/fault_provider.hpp>
#include <ros2_medkit_gateway/core/providers/introspection_provider.hpp>
#include <ros2_medkit_gateway/core/providers/operation_provider.hpp>
#include <ros2_medkit_gateway/dto/faults.hpp>
#include <ros2_medkit_gateway/plugins/ros_plugin_context.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace ros2_medkit_gateway {

/// OPC-UA Gateway Plugin - bridges OPC-UA PLCs into the SOVD entity tree
///
/// Implements GatewayPlugin (lifecycle, routes), IntrospectionProvider
/// (entity discovery from OPC-UA address space), and the typed provider
/// interfaces (DataProvider, OperationProvider, FaultProvider) so that
/// standard SOVD endpoints (/data, /operations, /faults) work for PLC
/// entities alongside the vendor x-plc-* extensions.
///
/// Standard SOVD endpoints (via provider interfaces):
///   GET  /{type}/{id}/data                    - DataProvider::list_data
///   GET  /{type}/{id}/data/{name}             - DataProvider::read_data
///   PUT  /{type}/{id}/data/{name}             - DataProvider::write_data
///   GET  /{type}/{id}/operations              - OperationProvider::list_operations
///   POST /{type}/{id}/operations/{name}       - OperationProvider::execute_operation
///   GET  /{type}/{id}/faults                  - FaultProvider::list_faults
///   GET  /{type}/{id}/faults/{code}           - FaultProvider::get_fault
///   DELETE /{type}/{id}/faults/{code}         - FaultProvider::clear_fault
///
/// Vendor REST extensions (via get_routes):
///   GET  /apps/{id}/x-plc-data          - All OPC-UA values for entity
///   GET  /apps/{id}/x-plc-data/{node}   - Single node value
///   POST /apps/{id}/x-plc-operations/{op} - Write value to PLC
///   GET  /components/{id}/x-plc-status  - Connection state and stats
///
/// Whether a value can be written at all is a property of the build, not of a
/// setting. ``MEDKIT_OPCUA_READ_ONLY`` defaults to ON: the OPC-UA write path is
/// then absent from the object, no data point is ever marked writable, neither
/// the x-plc-operations capability nor its POST route is registered, and
/// write_data / the value-write half of execute_operation refuse with 501
/// before reaching the client. ``-DMEDKIT_OPCUA_READ_ONLY=OFF`` builds the
/// write-capable plugin.
class OpcuaPlugin : public ros2_medkit_gateway::GatewayPlugin,
                    public ros2_medkit_gateway::IntrospectionProvider,
                    public ros2_medkit_gateway::DataProvider,
                    public ros2_medkit_gateway::OperationProvider,
                    public ros2_medkit_gateway::FaultProvider {
 public:
  OpcuaPlugin();
  ~OpcuaPlugin() override;

  OpcuaPlugin(const OpcuaPlugin &) = delete;
  OpcuaPlugin & operator=(const OpcuaPlugin &) = delete;
  OpcuaPlugin(OpcuaPlugin &&) = delete;
  OpcuaPlugin & operator=(OpcuaPlugin &&) = delete;

  // -- GatewayPlugin interface --
  std::string name() const override {
    return "opcua";
  }
  void configure(const nlohmann::json & config) override;
  void set_context(ros2_medkit_gateway::PluginContext & context) override;
  std::vector<PluginRoute> get_routes() override;
  void shutdown() override;

  // -- IntrospectionProvider interface --
  ros2_medkit_gateway::IntrospectionResult introspect(const ros2_medkit_gateway::IntrospectionInput & input) override;

  // -- DataProvider interface --
  tl::expected<dto::DataListResult, DataProviderErrorInfo> list_data(const std::string & entity_id) override;
  tl::expected<dto::DataValue, DataProviderErrorInfo> read_data(const std::string & entity_id,
                                                                const std::string & resource_name) override;
  tl::expected<dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & entity_id, const std::string & resource_name, const nlohmann::json & value) override;
  // Component and alarms-fallback entities own no NodeMap entries; false here
  // keeps the gateway from advertising a `data` capability list_data would 404.
  bool has_data(const std::string & entity_id) const override;

  // -- OperationProvider interface --
  tl::expected<dto::Collection<dto::OperationItem>, OperationProviderErrorInfo>
  list_operations(const std::string & entity_id) override;
  tl::expected<dto::OperationExecutionResult, OperationProviderErrorInfo>
  execute_operation(const std::string & entity_id, const std::string & operation_name,
                    const nlohmann::json & parameters) override;
  // True for any entity_defs() entry (mirrors list_operations' own lookup),
  // false for the Component itself, which has no entity_defs entry.
  bool has_operations(const std::string & entity_id) const override;

  /// The address-space walk configuration after configure() has merged the
  /// node map and the ROS parameters. Read by tests that need to see which
  /// source supplied a setting, which no REST response exposes.
  const AutoBrowseConfig & auto_browse_config_for_test() const {
    return node_map_.auto_browse_config();
  }

  // -- FaultProvider interface --
  tl::expected<dto::FaultListResult, FaultProviderErrorInfo> list_faults(const std::string & entity_id) override;
  tl::expected<dto::FaultDetailResult, FaultProviderErrorInfo> get_fault(const std::string & entity_id,
                                                                         const std::string & fault_code) override;
  tl::expected<dto::FaultClearResult, FaultProviderErrorInfo> clear_fault(const std::string & entity_id,
                                                                          const std::string & fault_code) override;

  // Resolve the SOVD severity bucket for an event alarm. An explicit configured
  // override wins; with none configured the raw OPC-UA event Severity (1-1000)
  // is mapped to a bucket by band (>=801 CRITICAL, >=501 ERROR, >=201 WARNING,
  // else INFO). Pure + static so it is unit-testable without a live server.
  static std::string map_severity(uint16_t live_severity, const std::string & severity_override);

  // Overlay a ``plugins.opcua.auto_alarms`` JSON/ROS param onto ``cfg``. Accepts
  // the bare-boolean shorthand or the full map form (same fields as the node-map
  // YAML ``auto_alarms:`` loader: source_node_id, entity_id, auto_clear,
  // severity_bands, include/exclude). Only keys actually present overwrite
  // ``cfg``, so this composes on top of whatever the YAML block set - the JSON
  // param wins, mirroring how ``plugins.opcua.auto_browse`` overlays its config
  // and how env vars override the rest of the plugin config. Unknown keys warn.
  // Callers run ``NodeMap::finalize_auto_alarms_overlay`` afterwards to re-derive
  // the default entity and parsed source NodeId. Static + injected ``warn`` so
  // the parse is unit-testable without a plugin instance.
  static void apply_auto_alarms_param(const nlohmann::json & value, AutoAlarmsConfig & cfg,
                                      const std::function<void(const std::string &)> & warn);

 private:
  // Route handlers
  void handle_plc_data(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);
  void handle_plc_data_single(const ros2_medkit_gateway::PluginRequest & req,
                              ros2_medkit_gateway::PluginResponse & res);
#if !MEDKIT_OPCUA_READ_ONLY
  /// Route handler for the vendor write endpoint. Declared and registered only
  /// in a write-capable build; a read-only build serves no such route.
  void handle_plc_operations(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);
#endif
  void handle_plc_status(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);

  // Fault-detection signal (threshold / status-bit / enum) -> Fault bridge
  void on_alarm_change(const std::string & entity_id, const ros2_medkit::fault_detection::FaultSignal & signal);

  // Issue #386: native AlarmConditionType event lifecycle bridge.
  void on_event_alarm(const AlarmEventDelivery & delivery);

  // Report/clear fault via ROS 2 service (private helpers, not the FaultProvider overrides)
  void send_report_fault(const std::string & entity_id, const std::string & fault_code,
                         const std::string & severity_str, const std::string & message);
  void send_clear_fault(const std::string & fault_code);

  // Dispatch now if the fault_manager service is matched, else buffer the
  // dispatch (bounded, order-preserving) to be flushed once it appears.
  void send_or_buffer(std::function<void()> dispatch);
  // Flush buffered fault dispatches when the fault_manager service is ready.
  void flush_pending_reports();

  // Publish PLC values to ROS 2 topics (called after each poll)
  void publish_values(const PollSnapshot & snap);

  // Create the per-entry ROS 2 Float32 publishers for numeric node map
  // entries. Called from set_context() AFTER connect()+run_auto_browse() so
  // that auto-discovered entries also get a publisher (moved out of its
  // original inline position, which ran before the node map could contain
  // anything auto_browse added).
  void create_value_publishers();

  // Run the recursive OPC-UA address-space walk (auto_browse) against the
  // now-connected client and merge the discovered entries into node_map_, then
  // (re)create value publishers for any new entries. Requires a live session.
  // Called from set_context() right after the initial connect(), and again from
  // the poll thread on every reconnect via maybe_rebrowse_on_reconnect(). The
  // merge into node_map_ is serialized against the REST read paths by
  // node_map_mutex_. No-op (never called) when auto_browse is disabled.
  void run_auto_browse();

  // Poll-thread hook (from publish_values): re-run auto_browse when the client
  // has established a new session since the last walk. Covers the field case
  // where the gateway starts before the PLC is reachable (initial connect
  // fails, so the initial walk is skipped) and the PLC only comes up later, as
  // well as a PLC restart. Idempotent: merge drops already-mapped node ids and
  // create_value_publishers() skips existing topics.
  void maybe_rebrowse_on_reconnect();

  // Log the effective OPC-UA security profile (policy / mode / user auth) at
  // startup; warns when running unsecured.
  void log_security_profile() const;

  // Read-only active-scan discovery. When discovery is enabled and no endpoint
  // was explicitly configured, run one pass, log a summary, and (if a suitable
  // None/Anonymous OPC-UA data server is found) set client_config_.endpoint_url
  // to the discovered ip:port so the existing connect + introspect path adopts
  // it. Injected scan/identify default to the real POSIX + open62541pp
  // implementations; tests override them. No-op when discovery is disabled or
  // an endpoint is already configured.
  void run_startup_discovery();

  // Build JSON response for data endpoint
  nlohmann::json build_data_response(const std::string & entity_id) const;

  std::atomic<bool> shutdown_requested_{false};
  ros2_medkit_gateway::RosPluginContext * ctx_{nullptr};
  OpcuaClientConfig client_config_;
  PollerConfig poller_config_;
  std::string node_map_path_;

  // Read-only PLC/OPC-UA network discovery (opt-in, default disabled).
  OpcuaDiscoveryConfig discovery_config_;
  // True when the operator pinned endpoint_url via config or OPCUA_ENDPOINT_URL.
  // Discovery only auto-selects an endpoint when this is false, so it never
  // overrides an explicit target or opens a second session on a polled PLC.
  bool endpoint_configured_{false};
  // True when the operator supplied a ``plugins.opcua.auto_alarms`` block.
  // When false AND no node_map_path is set, config-less discovery enables
  // native A&C by itself (default source Server object i=2253) so discovered
  // Alarms & Conditions surface on /api/v1/faults with no per-alarm mapping.
  bool auto_alarms_configured_{false};
  // Injected discovery I/O; default to the real POSIX / open62541pp probes.
  // Tests substitute in-memory fakes to exercise the auto-endpoint path without
  // a network. Set in configure(), consumed in run_startup_discovery().
  PortScanFn discovery_scan_fn_;
  IdentifyFn discovery_identify_fn_;

  std::unique_ptr<OpcuaClient> client_;
  NodeMap node_map_;

  // Guards the mutable parts of node_map_ (entries / indices / entity_defs)
  // that auto_browse rewrites on reconnect against the REST read paths. The
  // re-walk runs on the poll thread and takes the unique lock only for the
  // merge; the poller's own node_map_ reads share that thread, so they need no
  // lock. Read handlers on the HTTP thread take a shared lock for as long as
  // they hold pointers/refs into node_map_.
  mutable std::shared_mutex node_map_mutex_;

  // OpcuaClient::connection_generation the last auto_browse walk ran against (0
  // = never walked). The poll thread re-walks when the live generation differs,
  // mirroring device_identity_generation_. Written on the set_context thread
  // (initial walk, happens-before the poller starts) then only on the poll
  // thread, so no atomic is needed.
  uint64_t auto_browse_generation_{0};

  // INV2: asset-identity nameplate read once per session from the server's
  // device-info (ServerStatus/BuildInfo + optional OPC-UA DI nameplate) on the
  // first connected introspect, then reused until the client reconnects.
  // device_identity_generation_ stores the OpcuaClient::connection_generation
  // the nameplate was read at (0 = never read), so a poller reconnect triggers
  // a fresh read on the next introspect without hammering the server.
  AssetIdentity device_identity_;
  uint64_t device_identity_generation_{0};

  // ROS 2 service clients for fault reporting
  struct FaultClients;
  std::unique_ptr<FaultClients> fault_clients_;

  // Ordered buffer of pending fault report/clear dispatches. ReportFault /
  // ClearFault are fire-and-forget, so a report sent before the fault_manager
  // service is DDS-matched is dropped. Instead of blocking startup for the sink,
  // buffer the dispatch (preserving report-then-clear order) and flush it on the
  // next poll once the service is ready.
  //
  // Accessed from TWO threads: the poll thread (on_alarm_change / on_event_alarm
  // -> send_*_fault -> send_or_buffer, and publish_values -> flush_pending_reports)
  // AND the REST thread (the FaultProvider::clear_fault route override ->
  // send_clear_fault -> send_or_buffer). A push_back that reallocates the vector
  // while the other thread iterates/reads it is a use-after-free that corrupts the
  // heap, so every access is serialised by pending_reports_mutex_. The mutex is
  // never held across the actual dispatch (async_send_request) to keep ROS I/O out
  // of the critical section.
  std::mutex pending_reports_mutex_;
  std::vector<std::function<void()>> pending_reports_;

  // Tracks which non-numeric nodes have already been warned about (avoids log spam).
  // Instance member instead of static to survive plugin reload (dlclose/dlopen).
  std::unordered_set<std::string> warned_non_numeric_;

  // ROS 2 publishers for PLC value bridging (node_id_str -> publisher).
  // Declared before poller_ so that C++ reverse-destruction-order guarantees
  // the poller thread is joined before publishers are destroyed.
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_;

  // Must be declared last among resources used by the poll thread.
  // ~OpcuaPoller() calls stop() which joins the thread.
  std::unique_ptr<OpcuaPoller> poller_;
};

}  // namespace ros2_medkit_gateway
