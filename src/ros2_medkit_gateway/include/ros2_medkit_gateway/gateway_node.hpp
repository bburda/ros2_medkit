// Copyright 2025 bburda, mfaferek93
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

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/aggregation/mdns_discovery.hpp"
#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/default_script_provider.hpp"
#include "ros2_medkit_gateway/core/fault_trigger_engine.hpp"
#include "ros2_medkit_gateway/core/http/rate_limiter.hpp"
#include "ros2_medkit_gateway/core/http/rest_server.hpp"
#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/core/managers/bulk_data_store.hpp"
#include "ros2_medkit_gateway/core/managers/configuration_manager.hpp"
#include "ros2_medkit_gateway/core/managers/data_access_manager.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/core/managers/script_manager.hpp"
#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/managers/update_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/entity_change_scope.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/core/subscription_transport.hpp"
#include "ros2_medkit_gateway/core/trigger_store.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_action_transport.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_fault_service_transport.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_log_source.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_parameter_transport.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_service_transport.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_topic_subscription_transport.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_topic_transport.hpp"
#include "ros2_medkit_gateway/ros2/trigger_topic_subscriber.hpp"
#include "ros2_medkit_gateway/ros2_common/callback_groups.hpp"
#include "ros2_medkit_gateway/trigger_fault_subscriber.hpp"

namespace ros2_medkit_gateway {

class GatewayNode : public rclcpp::Node {
 public:
  explicit GatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~GatewayNode() override;

  /// Count application (peer) nodes from (name, namespace) pairs: excludes hidden
  /// nodes and the gateway's own nodes (whose FQN starts with @p self_fqn). Static
  /// and public so the startup-summary counting logic is unit-testable.
  static size_t count_peer_nodes(const std::vector<std::pair<std::string, std::string>> & nodes_and_namespaces,
                                 const std::string & self_fqn);

  /// Translate a bind address into a connectable address for the sample curl
  /// (bind-all "0.0.0.0"/"::"/"" -> loopback). Static and public for testing.
  static std::string connectable_host(const std::string & bind_host);
  GatewayNode(const GatewayNode &) = delete;
  GatewayNode & operator=(const GatewayNode &) = delete;
  GatewayNode(GatewayNode &&) = delete;
  GatewayNode & operator=(GatewayNode &&) = delete;

  /**
   * @brief Get the thread-safe entity cache with O(1) lookups
   * @return Reference to ThreadSafeEntityCache
   */
  const ThreadSafeEntityCache & get_thread_safe_cache() const;

  /**
   * @brief Get the DataAccessManager instance
   * @return Raw pointer to DataAccessManager (valid for lifetime of GatewayNode)
   * @note The returned pointer is valid as long as the GatewayNode exists.
   *       REST server is stopped before GatewayNode destruction to ensure safe access.
   */
  DataAccessManager * get_data_access_manager() const;

  /**
   * @brief Attach a TopicDataProvider to route topic sampling through.
   *
   * Called by main() after the rclcpp::Executor is constructed and the gateway
   * node added to it, so the provider can build its subscription node. Stores
   * the shared_ptr to keep the provider alive for the gateway node's lifetime
   * and forwards the raw pointer into DataAccessManager. The pool-backed
   * provider is the single sampling path (issue #375 race fix).
   */
  void set_topic_data_provider(std::shared_ptr<TopicDataProvider> provider);

  /// @return Non-owning pointer to the currently attached TopicDataProvider,
  ///         or nullptr when no provider has been wired up yet.
  TopicDataProvider * get_topic_data_provider() const {
    return topic_data_provider_.get();
  }

  /**
   * @brief Get the OperationManager instance
   * @return Raw pointer to OperationManager (valid for lifetime of GatewayNode)
   */
  OperationManager * get_operation_manager() const;

  /**
   * @brief Get the DiscoveryManager instance
   * @return Raw pointer to DiscoveryManager (valid for lifetime of GatewayNode)
   */
  DiscoveryManager * get_discovery_manager() const;

  /**
   * @brief Get the ConfigurationManager instance
   * @return Raw pointer to ConfigurationManager (valid for lifetime of GatewayNode)
   */
  ConfigurationManager * get_configuration_manager() const;

  /**
   * @brief Get the FaultManager instance
   * @return Raw pointer to FaultManager (valid for lifetime of GatewayNode)
   */
  FaultManager * get_fault_manager() const;

  /**
   * @brief Get the BulkDataStore instance
   * @return Raw pointer to BulkDataStore (valid for lifetime of GatewayNode)
   */
  BulkDataStore * get_bulk_data_store() const;

  /**
   * @brief Get the LogManager instance
   * @return Raw pointer to LogManager (valid for lifetime of GatewayNode)
   */
  LogManager * get_log_manager() const;

  /**
   * @brief Get the SubscriptionManager instance
   * @return Raw pointer to SubscriptionManager (valid for lifetime of GatewayNode)
   */
  SubscriptionManager * get_subscription_manager() const;

  /**
   * @brief Get the UpdateManager instance
   * @return Raw pointer to UpdateManager (valid for lifetime of GatewayNode), or nullptr if disabled
   */
  UpdateManager * get_update_manager() const;

  /**
   * @brief Get the LockManager instance
   * @return Raw pointer to LockManager (valid for lifetime of GatewayNode), or nullptr if locking disabled
   */
  LockManager * get_lock_manager() const;

  /**
   * @brief Get the ScriptManager instance
   * @return Raw pointer to ScriptManager (valid for lifetime of GatewayNode), or nullptr if not initialized
   */
  ScriptManager * get_script_manager() const;

  /**
   * @brief Get the PluginManager instance
   * @return Raw pointer to PluginManager (valid for lifetime of GatewayNode)
   */
  PluginManager * get_plugin_manager() const;

  /**
   * @brief Wire the zero-config entity freeze-frame capture onto the shared
   *        subscription executor. Call once after the executor is built and
   *        plugins are loaded (main.cpp). No-op when disabled or no plugins.
   */
  void init_entity_freeze_frame_capture(ros2_common::Ros2SubscriptionExecutor & exec);

  /**
   * @brief Get the zero-config entity freeze-frame capture
   * @return Raw pointer, or nullptr when disabled / no plugins loaded
   */
  EntityFreezeFrameCapture * get_entity_freeze_frame_capture() const;

  /**
   * @brief Route the trigger topic subscriber through the shared subscription
   *        executor (issue #548). Its per-trigger subscriptions are then
   *        created, destroyed, and dispatched on the executor's single worker
   *        thread, so callbacks drain before teardown instead of racing the
   *        main multi-threaded executor. Call once after the executor is built
   *        (main.cpp). No-op when triggers are disabled.
   */
  void set_trigger_subscription_executor(ros2_common::Ros2SubscriptionExecutor & exec);

  /**
   * @brief Shut down the trigger topic subscriber, draining and destroying its
   *        subscriptions on the executor worker. Idempotent. Must be called
   *        BEFORE the subscription executor is destroyed (the trigger
   *        subscriptions live on the executor's node and are torn down via its
   *        worker). No-op when triggers are disabled. ~GatewayNode also calls
   *        it, which becomes a no-op once this has run.
   */
  void shutdown_trigger_subscriber();

  /**
   * @brief Stop and join the REST server.
   *
   * Callable from the main teardown before the subscription executor is
   * destroyed, so no httplib handler thread can be mid-subscribe()/unsubscribe()
   * (which dereference the subscription executor) when it is freed. Idempotent;
   * ~GatewayNode also calls it.
   */
  void stop_rest_server();

  /**
   * @brief Wire the config-less threshold-rule (fault-trigger) engine and its
   *        evaluation loop. Call once after plugins are loaded (main.cpp).
   *        No-op when disabled (``fault_triggers.enabled=false``) or no plugins.
   */
  void init_fault_trigger_engine();

  /**
   * @brief Get the config-less fault-trigger (threshold-rule) engine.
   * @return Raw pointer, or nullptr when disabled / no plugins loaded
   */
  FaultTriggerEngine * get_fault_trigger_engine() const;

  /**
   * @brief Get the ResourceSamplerRegistry instance
   * @return Raw pointer to ResourceSamplerRegistry (valid for lifetime of GatewayNode)
   */
  ResourceSamplerRegistry * get_sampler_registry() const;

  /**
   * @brief Get the TransportRegistry instance
   * @return Raw pointer to TransportRegistry (valid for lifetime of GatewayNode)
   */
  TransportRegistry * get_transport_registry() const;

  /**
   * @brief Get the SSEClientTracker instance
   * @return Shared pointer to SSEClientTracker
   */
  std::shared_ptr<SSEClientTracker> get_sse_client_tracker() const;

  /**
   * @brief Get the ResourceChangeNotifier instance
   * @return Raw pointer to ResourceChangeNotifier (valid for lifetime of GatewayNode)
   */
  ResourceChangeNotifier * get_resource_change_notifier() const;

  /**
   * @brief Get the TriggerManager instance
   * @return Raw pointer to TriggerManager (valid for lifetime of GatewayNode), or nullptr if disabled
   */
  TriggerManager * get_trigger_manager() const;

  /**
   * @brief Get the ConditionRegistry instance
   * @return Raw pointer to ConditionRegistry (valid for lifetime of GatewayNode)
   */
  ConditionRegistry * get_condition_registry() const;

  /**
   * @brief Get the AggregationManager instance
   * @return Raw pointer to AggregationManager (valid for lifetime of GatewayNode), or nullptr if disabled
   */
  AggregationManager * get_aggregation_manager() const;

  /**
   * @brief Handle a plugin's `PluginContext::notify_entities_changed` request.
   *
   * Runs a single `refresh_cache()` pass synchronously. The scope hint is
   * accepted and logged but the current implementation ignores it and always
   * does a full refresh - future work may limit the rediscovery to the
   * indicated area / component. The entry point is safe to call from any
   * thread: refresh passes triggered by plugin notifications, the periodic
   * refresh timer, and startup are serialized by an internal mutex inside
   * `refresh_cache()` because refresh touches discovery state (e.g., the
   * merge pipeline cached inside `DiscoveryManager`) that itself assumes
   * single-threaded execution - `ThreadSafeEntityCache`'s own mutex is not
   * sufficient.
   */
  void handle_entity_change_notification(const EntityChangeScope & scope);

  /**
   * @brief Test hook: simulate a plugin calling notify_entities_changed
   *        from within its own IntrospectionProvider::introspect() callback.
   *
   * Sets the per-thread in-refresh flag (the same flag the real refresh
   * path uses) and invokes `handle_entity_change_notification(scope)`.
   * The gateway is expected to short-circuit the call with a warning log
   * and return without reloading the manifest.
   *
   * Exists purely to let unit tests exercise the reentrancy guard without
   * spinning up a full plugin with a real IntrospectionProvider. Do NOT
   * call this from production code.
   */
  void trigger_reentrant_notification_for_testing(const EntityChangeScope & scope);

  /**
   * @brief Test hook: stop the gateway's own discovery refresh.
   *
   * Cancels the graph-event and backstop refresh timers and drops the graph
   * event so `refresh_cache()` never runs again after this call. Tests that
   * inject a known entity-cache state directly (via `get_thread_safe_cache()`)
   * must call this first, otherwise a graph-event-driven `refresh_cache()`
   * reconciles the cache back to the live ROS graph and silently wipes the
   * injected entities. Do NOT call this from production code.
   */
  void stop_discovery_refresh_for_testing();

 private:
  void refresh_cache();
  void start_rest_server();

  /// Log a one-time discovery summary shortly after startup: discovered node /
  /// topic / entity counts, the REST URL and a sample curl. When no application
  /// nodes are visible, also warn loudly with the active ROS environment
  /// (ROS_DOMAIN_ID / RMW_IMPLEMENTATION / ROS_LOCALHOST_ONLY) and likely causes,
  /// so a misconfigured bringup is not a silent empty tree.
  void log_startup_summary();

  // Configuration parameters
  std::string server_host_;
  int server_port_;
  // Safety-backstop refresh interval in milliseconds. The primary refresh
  // trigger is the rclcpp graph event polled by `graph_check_timer_`; this
  // value drives `backstop_timer_` which guarantees liveness if a graph
  // event is ever missed (lost wakeup, rclcpp anomaly, etc.).
  int refresh_interval_ms_;
  bool filter_internal_nodes_{true};
  CorsConfig cors_config_;
  AuthConfig auth_config_;
  RateLimitConfig rate_limit_config_;
  TlsConfig tls_config_;

  // ROS 2 subscription infrastructure (injected from main() after the executor
  // is constructed). Owned via shared_ptr because the provider is handed out
  // as a raw TopicDataProvider* to DataAccessManager.
  std::shared_ptr<TopicDataProvider> topic_data_provider_;

  // Topic transport adapter shared with DataAccessManager. Held here so the
  // provider attach/detach hooks can forward into the adapter alongside the
  // manager and discovery side updates.
  std::shared_ptr<ros2::Ros2TopicTransport> topic_transport_;

  // Shared callback groups for blocking-RPC response dispatch (Reentrant)
  // and per-action status subscriptions (MutuallyExclusive) - issue #575.
  // Declared BEFORE the transports so the groups destruct after every
  // entity registered into them; the node itself only holds weak
  // references to its callback groups.
  ros2_common::GatewayCallbackGroups callback_groups_;

  // Service / action transport adapters shared with OperationManager. Held
  // here so their lifetime matches the gateway's executor (transports own
  // rclcpp clients + subscriptions and must outlive the manager).
  std::shared_ptr<ros2::Ros2ServiceTransport> service_transport_;
  std::shared_ptr<ros2::Ros2ActionTransport> action_transport_;

  // Parameter transport adapter shared with ConfigurationManager. Owns the
  // parameter-client cache + defaults cache + spin_mutex; the manager forwards
  // shutdown() into it before rclcpp::shutdown() runs.
  std::shared_ptr<ros2::Ros2ParameterTransport> parameter_transport_;

  // Fault-service transport adapter shared with FaultManager. Owns the seven
  // rclcpp service clients, the seven per-client mutexes, and the
  // ros2_medkit_msgs <-> JSON conversion helpers that previously lived inside
  // FaultManager.
  std::shared_ptr<ros2::Ros2FaultServiceTransport> fault_service_transport_;

  // /rosout source adapter shared with LogManager. Owns the
  // rclcpp::Subscription<rcl_interfaces::msg::Log> + msg-to-LogEntry
  // conversion that previously lived inside LogManager.
  std::shared_ptr<ros2::Ros2LogSource> log_source_;

  // Managers
  std::unique_ptr<DiscoveryManager> discovery_mgr_;
  std::unique_ptr<DataAccessManager> data_access_mgr_;
  std::unique_ptr<OperationManager> operation_mgr_;
  std::unique_ptr<ConfigurationManager> config_mgr_;
  std::unique_ptr<FaultManager> fault_mgr_;
  std::unique_ptr<LogManager> log_mgr_;
  std::unique_ptr<BulkDataStore> bulk_data_store_;
  std::unique_ptr<SubscriptionManager> subscription_mgr_;
  std::unique_ptr<ResourceSamplerRegistry> sampler_registry_;
  std::unique_ptr<TransportRegistry> transport_registry_;
  std::shared_ptr<SSEClientTracker> sse_client_tracker_;
  // IMPORTANT: plugin_mgr_ BEFORE update_mgr_ - C++ destroys in reverse order,
  // so update_mgr_ waits for async tasks before plugin_mgr_ destroys the plugin.
  // plugin_ctx_ is owned here (outlives plugins); plugin_mgr_ holds a non-owning ref.
  std::unique_ptr<PluginContext> plugin_ctx_;
  std::unique_ptr<PluginManager> plugin_mgr_;
  std::unique_ptr<DefaultScriptProvider> default_script_provider_;
  std::unique_ptr<ScriptManager> script_mgr_;
  std::unique_ptr<UpdateManager> update_mgr_;
  std::unique_ptr<LockManager> lock_manager_;

  // Trigger infrastructure (destroyed after rest_server_)
  std::unique_ptr<ResourceChangeNotifier> resource_change_notifier_;
  std::unique_ptr<ConditionRegistry> condition_registry_;
  std::unique_ptr<TriggerStore> trigger_store_;
  std::unique_ptr<TriggerManager> trigger_mgr_;
  std::unique_ptr<TriggerFaultSubscriber> trigger_fault_subscriber_;
  // Zero-config freeze-frames for plugin-backed entities (nullptr when disabled)
  std::unique_ptr<EntityFreezeFrameCapture> entity_freeze_frame_capture_;
  // Config-less threshold-rule engine + its dedicated evaluation loop (issue
  // #235). The thread is joined in ~GatewayNode BEFORE plugin/fault shutdown.
  std::unique_ptr<FaultTriggerEngine> fault_trigger_engine_;
  std::thread fault_trigger_thread_;
  std::atomic<bool> fault_trigger_stop_{false};
  std::unique_ptr<TriggerTopicSubscriber> trigger_topic_subscriber_;
  // Adapter routing manager-side subscribe() calls onto trigger_topic_subscriber_.
  std::shared_ptr<ros2::Ros2TopicSubscriptionTransport> trigger_topic_transport_;

  // Aggregation infrastructure (destroyed in order: mdns -> rest_server -> aggregation)
  // mDNS threads must stop before rest_server to avoid callbacks during shutdown.
  // AggregationManager must outlive rest_server because handlers reference it.
  std::unique_ptr<AggregationManager> aggregation_mgr_;
  std::unique_ptr<MdnsDiscovery> mdns_discovery_;

  std::unique_ptr<RESTServer> rest_server_;

  // Cache with thread safety
  ThreadSafeEntityCache thread_safe_cache_;
  // One-shot WARN when entity_cache capacity is exceeded (grew on first refresh after reserve).
  // Cleared only at construction time; never reset so the WARN fires at most once per run.
  bool warned_cache_grow_{false};

  // Graph-change-driven discovery refresh.
  //
  // `graph_event_` is the rclcpp::Event signalled whenever the ROS 2 graph
  // changes (node up/down, topic/service/action add or remove). It is a
  // shared_ptr handed out by `rclcpp::Node::get_graph_event()`; the executor
  // owns the underlying notification and we just poll-and-clear it.
  //
  // `graph_check_timer_` polls `graph_event_->check_and_clear()` at a fast
  // cadence (100 ms) and runs `refresh_cache()` only when the event fires.
  // On a stable graph this keeps idle CPU near zero.
  //
  // `backstop_timer_` runs `refresh_cache()` unconditionally at the slower
  // `refresh_interval_ms_` cadence. Its sole purpose is liveness in the
  // unlikely case a graph event is missed.
  rclcpp::Event::SharedPtr graph_event_;
  rclcpp::TimerBase::SharedPtr graph_check_timer_;
  rclcpp::TimerBase::SharedPtr backstop_timer_;

  // Debounce graph-event-driven refreshes (issue #442). The graph event fires
  // many times per second under graph churn; coalesce so refresh_cache() runs
  // at most once per `refresh_debounce_ms_`. `graph_dirty_` marks a pending
  // (consumed-but-not-yet-serviced) graph change; `last_graph_refresh_` is the
  // last graph-driven refresh time. Set via `discovery.refresh_debounce_ms`.
  // `graph_dirty_`/`last_graph_refresh_` are atomic: under a MultiThreadedExecutor
  // successive ticks of the graph-check timer can run on different worker threads,
  // so this cross-tick state is accessed from more than one thread.
  int refresh_debounce_ms_{1000};
  std::atomic<bool> graph_dirty_{false};
  std::atomic<std::chrono::steady_clock::time_point> last_graph_refresh_{};

  // Serializes `refresh_cache()` across the refresh timer, plugin
  // `notify_entities_changed` calls and any other caller. Required because
  // the refresh pipeline touches discovery state that is not itself
  // thread-safe (the merge pipeline cached inside `DiscoveryManager`).
  // Recursive so that `handle_entity_change_notification` can also cover
  // the preceding `reload_manifest()` call without deadlocking when it
  // subsequently invokes `refresh_cache()`.
  std::recursive_mutex refresh_mutex_;

  // Timer for periodic cleanup of old action goals
  rclcpp::TimerBase::SharedPtr cleanup_timer_;

  // One-shot timer that fires the post-startup discovery summary (see
  // log_startup_summary). Reset after it runs.
  rclcpp::TimerBase::SharedPtr startup_summary_timer_;

  // Timer for periodic cleanup of expired cyclic subscriptions
  rclcpp::TimerBase::SharedPtr subscription_cleanup_timer_;

  // Timer for periodic cleanup of expired locks
  rclcpp::TimerBase::SharedPtr lock_cleanup_timer_;

  // REST server thread management
  std::unique_ptr<std::thread> server_thread_;
};

/**
 * @brief Filter ROS 2 internal nodes from an app list
 *
 * Removes apps whose base name begins with '_' (ROS 2 internal node convention).
 * For remote (peer-prefixed) apps, the peer prefix ("peer_name__") is stripped
 * before checking for the underscore prefix, using the routing table for precise
 * prefix detection.
 *
 * @param apps App vector to filter in place
 * @param peer_routing_table Maps entity_id -> peer_name for remote entities
 * @return Number of apps removed
 */
size_t filter_internal_node_apps(std::vector<App> & apps,
                                 const std::unordered_map<std::string, std::string> & peer_routing_table);

}  // namespace ros2_medkit_gateway
