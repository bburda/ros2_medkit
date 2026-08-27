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

#include <atomic>
#include <chrono>
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "ros2_medkit_gateway/core/configuration/parameter_types.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"
#include "ros2_medkit_gateway/core/util/bounded_lru_cache.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing ParameterTransport.
 *
 * Owns the rclcpp::AsyncParametersClient cache, the rclcpp::Parameter defaults
 * cache, the negative-cache for unreachable nodes, the spin_mutex serialising
 * parameter-client spins, and the JSON <-> rclcpp::ParameterValue conversion
 * helpers that previously lived inside ConfigurationManager.
 *
 * The client cache and the default-value caches are keyed by node name and
 * bounded by LRU eviction (BoundedLruCache). Each would otherwise gain one
 * permanent entry per distinct node name ever queried and leak memory on a
 * long-lived gateway facing a churning graph. The client cache uses a moderate
 * cap (kMaxParamClientCacheSize) because each entry is heavy; the default-value
 * caches use a much larger cap (kMaxDefaultsCacheSize) because their entries are
 * light AND because evicting a default can rebase what "default" means. To keep
 * reset-to-default correct, set_parameter records a parameter's pre-write value
 * in pre_write_values_, which get_default / list_defaults prefer over the
 * first-seen snapshot.
 *
 * Uses AsyncParametersClient + explicit spin_until_future_complete (not the
 * SyncParametersClient) so a service TIMEOUT is distinguished from a
 * successful-but-EMPTY response via rclcpp::FutureReturnCode. The sync client
 * collapses SUCCESS-with-empty-result and TIMEOUT into the same empty vector,
 * which caused a false-positive that negative-cached a HEALTHY node on a legit
 * empty get (e.g. a param undeclared between list and get) - see #531.
 *
 * The adapter exposes only the neutral ParameterTransport surface to the
 * manager; all rclcpp types stay confined to this translation unit.
 */
class Ros2ParameterTransport : public ParameterTransport {
 public:
  /**
   * @param node Non-owning ROS node used to declare parameters and to derive
   *             the gateway's own FQN for the self-node short circuit.
   * @param service_timeout_sec Timeout for AsyncParametersClient::wait_for_service
   *                            and each spin_until_future_complete round trip.
   * @param negative_cache_ttl_sec How long an unavailable node remains in the
   *                               negative cache before another IPC attempt.
   *                               Set to 0 to disable.
   * @param client_cache_size Capacity of the per-node AsyncParametersClient cache
   *                          before least-recently-used eviction. Defaults to
   *                          kMaxParamClientCacheSize; overridable mainly for
   *                          tests.
   * @param defaults_cache_size Capacity of each per-node default-value cache (the
   *                            first-seen snapshot and the pre-write record) before
   *                            least-recently-used eviction. Defaults to
   *                            kMaxDefaultsCacheSize; larger than the client cache
   *                            because a parameter map is far lighter than an
   *                            AsyncParametersClient, so evicting a default (which
   *                            can rebase "default" to a current value) stays rare.
   */
  Ros2ParameterTransport(rclcpp::Node * node, double service_timeout_sec, double negative_cache_ttl_sec,
                         std::size_t client_cache_size = kMaxParamClientCacheSize,
                         std::size_t defaults_cache_size = kMaxDefaultsCacheSize);

  ~Ros2ParameterTransport() override;

  Ros2ParameterTransport(const Ros2ParameterTransport &) = delete;
  Ros2ParameterTransport & operator=(const Ros2ParameterTransport &) = delete;
  Ros2ParameterTransport(Ros2ParameterTransport &&) = delete;
  Ros2ParameterTransport & operator=(Ros2ParameterTransport &&) = delete;

  // ParameterTransport implementation -----------------------------------------

  bool is_self_node(const std::string & node_name) const override;

  ParameterResult list_parameters(const std::string & node_name) override;
  ParameterResult get_parameter(const std::string & node_name, const std::string & param_name) override;
  ParameterResult set_parameter(const std::string & node_name, const std::string & param_name,
                                const json & value) override;

  ParameterResult list_own_parameters() override;
  ParameterResult get_own_parameter(const std::string & param_name) override;

  ParameterResult get_default(const std::string & node_name, const std::string & param_name) override;
  ParameterResult list_defaults(const std::string & node_name) override;

  bool is_node_available(const std::string & node_name) const override;
  void shutdown() override;

  /// Number of entries currently held in the per-node AsyncParametersClient
  /// cache. Thread-safe. Exposed for diagnostics and to let tests assert the
  /// cache stays bounded under many distinct node names.
  std::size_t param_client_cache_size() const;

  /// Number of entries currently held in the per-node first-seen default-value
  /// cache. Thread-safe. Exposed for diagnostics and eviction tests.
  std::size_t default_values_cache_size() const;

 private:
  /// Get or create a cached AsyncParametersClient for the given node.
  std::shared_ptr<rclcpp::AsyncParametersClient> get_param_client(const std::string & node_name);

  /// Spin param_node_ until `future` resolves or the service timeout elapses.
  /// The ONLY place a parameter round trip blocks; makes TIMEOUT explicit.
  /// @pre spin_mutex_ must be held by the caller (single spin on param_node_).
  /// @return TIMEOUT   - the node did not answer within service_timeout_sec;
  ///         SUCCESS   - future.get() is valid (its result may legitimately be
  ///                     an EMPTY vector: not-found / unset / raced);
  ///         INTERRUPTED - transport shutting down (or param_node_ gone).
  template <typename FutureT>
  rclcpp::FutureReturnCode spin_for(const FutureT & future);

  /// Cache default values for a node (called on first access).
  /// @pre spin_mutex_ must be held by the caller.
  /// @return true if the node's own round-trip failed/timed out (service not
  ///         discoverable, or list/get IPC threw) so the caller can short-circuit
  ///         WITHOUT depending on the negative-cache TTL (which is a no-op when
  ///         negative_cache_ttl_sec_ == 0). false if defaults were cached, were
  ///         already cached, or the transport is shutting down.
  bool cache_default_values(const std::string & node_name);

  /// Check if a node is in the negative cache (recently unavailable).
  bool is_node_unavailable(const std::string & node_name) const;

  /// Mark a node as unavailable in the negative cache.
  void mark_node_unavailable(const std::string & node_name);

  /// Convert ROS 2 parameter type to string.
  std::string parameter_type_to_string(rclcpp::ParameterType type) const;

  /// Convert ROS 2 ParameterValue to JSON.
  json parameter_value_to_json(const rclcpp::ParameterValue & value) const;

  /// The value as the configurations API may show it: the real one, or a
  /// redaction marker when the parameter's value is a credential.
  ///
  /// The configurations route is a read route, and a gateway that answers it
  /// with the secret that signs its own tokens hands any reader the means to
  /// mint one. Names are matched exactly, so a parameter that merely mentions
  /// one of these words keeps showing its value.
  json parameter_value_for_display(const std::string & name, const rclcpp::ParameterValue & value) const;

  /// Whether this parameter's value is a credential and must never be shown.
  static bool is_secret_parameter(const std::string & name);

  /// Convert JSON value to ROS 2 ParameterValue.
  rclcpp::ParameterValue json_to_parameter_value(const json & value, rclcpp::ParameterType hint_type) const;

  /// Get the service timeout as a chrono duration.
  std::chrono::duration<double> get_service_timeout() const;

  /// Try to acquire spin_mutex_ with timeout.
  /// Returns unique_lock on success, nullopt on timeout (result populated with error).
  std::optional<std::unique_lock<std::timed_mutex>> try_acquire_spin_lock(ParameterResult & result);

  /// Throttled WARN emitted when a default-value cache evicts an entry, so the
  /// rare case where a re-seed could rebase "default" is at least observable.
  void warn_defaults_evicted(const std::string & node_name) const;

  rclcpp::Node * node_;

  /// Timeout for waiting for parameter services.
  double service_timeout_sec_;

  /// Negative cache TTL in seconds (0 = disabled).
  double negative_cache_ttl_sec_;

  /// Gateway's own fully qualified node name (for self-query detection).
  std::string own_node_fqn_;

  /// Shutdown flag - prevents use-after-free on param_node_ after shutdown.
  std::atomic<bool> shutdown_requested_{false};

  /// Negative cache: nodes whose parameter service was recently unavailable.
  /// Avoids repeated blocking waits on nodes that don't have parameter services.
  mutable std::shared_mutex negative_cache_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> unavailable_nodes_;

  /// Default-value caches, both guarded by defaults_mutex_ and bounded by LRU
  /// eviction. Key: node_name, Value: map of param_name -> Parameter.
  ///
  /// default_values_ is the first-seen snapshot: the values a node reported the
  /// first time it was queried. pre_write_values_ records the value of a
  /// parameter as it was JUST BEFORE the gateway first overwrote it via
  /// set_parameter. get_default / list_defaults prefer pre_write_values_ so that
  /// "reset to default" restores what the value was before the gateway changed
  /// it, even if the first-seen snapshot was captured after that write or was
  /// evicted and re-seeded from the current (post-write) value.
  mutable std::mutex defaults_mutex_;
  BoundedLruCache<std::string, std::map<std::string, rclcpp::Parameter>> default_values_;
  BoundedLruCache<std::string, std::map<std::string, rclcpp::Parameter>> pre_write_values_;

  /// Mutex to serialize spin operations on param_node_.
  /// spin_for() drives spin_until_future_complete on param_node_, which is NOT
  /// thread-safe against a concurrent spin on the same node.
  /// Declared BEFORE param_node_ so it outlives the node during destruction.
  mutable std::timed_mutex spin_mutex_;

  /// Internal node for parameter client operations.
  /// Created once in constructor - must be in DDS graph early for fast service discovery.
  std::shared_ptr<rclcpp::Node> param_node_;

  /// Context of param_node_, plus the handle of the pre-shutdown callback registered on
  /// it. param_node_ must be released while the GraphListener is still running, and the
  /// callback is what guarantees that: Context::shutdown() runs pre-shutdown callbacks,
  /// then rcl_shutdown, then the on_shutdown callbacks - and the listener is stopped from
  /// one of the latter.
  ///
  /// The crash this prevents is a HALF-REGISTERED node, not a late destruction.
  /// NodeGraph::get_graph_event() consumes should_add_to_graph_listener_ before calling
  /// GraphListener::add_node(), and add_node() throws GraphListenerShutdownError once the
  /// listener is down. The flag is spent while the node was never listed, so ~NodeGraph
  /// takes its remove_node() branch, the node is absent from node_graph_interfaces_, and
  /// NodeNotFoundError escapes an implicitly-noexcept destructor: immediate std::terminate
  /// (SIGABRT), which no try/catch at the call site can intercept. Only a first-ever graph
  /// use racing the listener's shutdown can produce it, which is why the crash was
  /// intermittent rather than every run.
  ///
  /// A node that IS registered is fine either way: GraphListener::__shutdown() only joins
  /// its thread and finalizes the wait set, it never clears node_graph_interfaces_, and
  /// remove_node() erases directly once is_shutdown() is set. Do not "fix" a suspected
  /// late-destruction bug here - that path provably works.
  ///
  /// Every wait_for_service in this class runs under spin_mutex_ and the callback blocks
  /// on the same mutex, so no first graph use can straddle the listener's shutdown. The
  /// destructor deregisters the callback and calls the (idempotent) shutdown() for the
  /// ordinary case where the transport dies first.
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;

  /// Cached AsyncParametersClient per target node, bounded by LRU eviction.
  mutable std::mutex clients_mutex_;
  BoundedLruCache<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;

  /// Maximum entries in negative cache before hard eviction.
  static constexpr size_t kMaxNegativeCacheSize = 500;

  /// Maximum entries in the per-node AsyncParametersClient cache before
  /// least-recently-used eviction. Kept moderate because each entry is heavy: an
  /// AsyncParametersClient owns several DDS service clients on param_node_.
  /// Evicting one is transparent - the next query rebuilds the client.
  static constexpr std::size_t kMaxParamClientCacheSize = 256;

  /// Maximum entries in each per-node default-value cache before
  /// least-recently-used eviction. Much larger than the client cache because a
  /// parameter map is orders of magnitude lighter than an AsyncParametersClient,
  /// AND because evicting a default is NOT transparent: a re-seed can rebase what
  /// "default" means. A large cap keeps defaults eviction exceptional on any
  /// realistic graph while still bounding memory.
  static constexpr std::size_t kMaxDefaultsCacheSize = 8192;

  /// Margin added to service_timeout_sec_ for spin_mutex acquisition timeout.
  /// Must exceed service_timeout_sec_ to avoid spurious TIMEOUT errors.
  static constexpr double kSpinLockMarginSec{1.0};

  /// Timeout for shutdown to wait for in-flight IPC.
  static constexpr std::chrono::seconds kShutdownTimeout{10};
};

}  // namespace ros2_medkit_gateway::ros2
