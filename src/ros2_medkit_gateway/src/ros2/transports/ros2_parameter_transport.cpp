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

#include "ros2_medkit_gateway/ros2/transports/ros2_parameter_transport.hpp"

#include <algorithm>
#include <chrono>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace ros2_medkit_gateway::ros2 {

Ros2ParameterTransport::Ros2ParameterTransport(rclcpp::Node * node, double service_timeout_sec,
                                               double negative_cache_ttl_sec, std::size_t client_cache_size,
                                               std::size_t defaults_cache_size)
  : node_(node)
  , service_timeout_sec_(service_timeout_sec)
  , negative_cache_ttl_sec_(negative_cache_ttl_sec)
  , default_values_(defaults_cache_size,
                    [this](const std::string & node_name, const std::map<std::string, rclcpp::Parameter> &) {
                      warn_defaults_evicted(node_name);
                    })
  , pre_write_values_(defaults_cache_size,
                      [this](const std::string & node_name, const std::map<std::string, rclcpp::Parameter> &) {
                        warn_defaults_evicted(node_name);
                      })
  , param_clients_(client_cache_size) {
  // Create internal node for parameter client operations early.
  // Must be in DDS graph before any parameter queries for fast service discovery.
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.use_global_arguments(false);
  param_node_ = std::make_shared<rclcpp::Node>("_param_client_node", options);

  // Store own node FQN for self-query detection.
  own_node_fqn_ = node_->get_fully_qualified_name();

  RCLCPP_INFO(node_->get_logger(), "Ros2ParameterTransport initialized (timeout=%.1fs, negative_cache=%.0fs)",
              service_timeout_sec_, negative_cache_ttl_sec_);

  // Release param_node_ while the GraphListener is still running. The listener is stopped
  // from an on_shutdown callback, which Context::shutdown() runs AFTER rcl_shutdown and
  // therefore after every pre-shutdown callback - so registering here guarantees the node
  // is gone before the listener goes down.
  //
  // What that avoids is the half-registration race: NodeGraph::get_graph_event() consumes
  // should_add_to_graph_listener_ BEFORE calling add_node(), and add_node() throws
  // GraphListenerShutdownError once the listener is down. The flag is then spent while the
  // node was never listed, so ~NodeGraph takes its remove_node() branch, the node is absent
  // from node_graph_interfaces_, and NodeNotFoundError escapes a noexcept destructor ->
  // std::terminate, exit -6. Every wait_for_service in this class runs under spin_mutex_
  // and this callback blocks on the same mutex, so no first graph use can straddle the
  // listener's shutdown. (A node that IS registered tears down cleanly after shutdown:
  // GraphListener::__shutdown() never clears node_graph_interfaces_, and remove_node()
  // erases directly once is_shutdown() is set.)
  //
  // MUST stay the last statement of this constructor: if anything after it throws, the
  // destructor never runs, and pre_shutdown_handle_ is only a weak_ptr - destroying it
  // deregisters nothing, leaving a callback that later fires on freed memory.
  context_ = param_node_->get_node_base_interface()->get_context();
  pre_shutdown_handle_ = context_->add_pre_shutdown_callback([this] {
    shutdown();
  });
}

Ros2ParameterTransport::~Ros2ParameterTransport() {
  // Deregister first: the callback captures `this`, and once we return the object is gone.
  // This is real work even after the callback has already fired - Context::shutdown() runs
  // the callbacks off a copy and never erases them, so the entry is still registered. The
  // erase is load-bearing: the default context is a process-lifetime static that a later
  // rclcpp::init() re-initializes without clearing the callback lists, so an entry left
  // behind would fire on the NEXT shutdown against a freed transport. Context takes the
  // same pre_shutdown_callbacks_mutex_ it fires under, so this is serialized against a
  // callback in flight.
  if (context_) {
    context_->remove_pre_shutdown_callback(pre_shutdown_handle_);
  }
  shutdown();  // idempotent: a no-op when the pre-shutdown callback already ran
}

void Ros2ParameterTransport::shutdown() {
  if (shutdown_requested_.exchange(true)) {
    return;  // Already shut down
  }
  // Destroying the cached clients and param_node_ frees rcl entities on
  // param_node_. spin_for() reads that entity set under spin_mutex_, so the
  // teardown must also hold spin_mutex_ or it could free an entity mid-spin
  // (use-after-free). Each spin is bounded by the service timeout, so the lock
  // is always released shortly. If the initial timed wait lapses we BLOCK for it
  // rather than tear down under a live spinner: a spin cannot outlive its bounded
  // duration, so this cannot hang in practice, and a bounded hang would be safer
  // than freeing an entity a live spin still references.
  std::unique_lock<std::timed_mutex> spin_lock(spin_mutex_, std::defer_lock);
  if (!spin_lock.try_lock_for(kShutdownTimeout)) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Ros2ParameterTransport shutdown: spin_mutex_ still held after %llds; waiting for the "
                  "in-flight parameter operation to finish before tearing down clients",
                  static_cast<long long>(kShutdownTimeout.count()));
    }
    spin_lock.lock();  // block until the bounded spin releases; never tear down under a live spinner
  }
  std::lock_guard<std::mutex> lock(clients_mutex_);
  param_clients_.clear();
  param_node_.reset();
}

bool Ros2ParameterTransport::is_self_node(const std::string & node_name) const {
  return node_name == own_node_fqn_;
}

bool Ros2ParameterTransport::is_node_available(const std::string & node_name) const {
  return !is_node_unavailable(node_name);
}

std::optional<std::unique_lock<std::timed_mutex>>
Ros2ParameterTransport::try_acquire_spin_lock(ParameterResult & result) {
  auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_timeout_sec_ + kSpinLockMarginSec));
  std::unique_lock<std::timed_mutex> lock(spin_mutex_, std::defer_lock);
  if (!lock.try_lock_for(timeout)) {
    result.success = false;
    result.error_message = "Parameter service temporarily unavailable - timed out after " +
                           std::to_string(static_cast<int>(service_timeout_sec_ + kSpinLockMarginSec)) + "s";
    result.error_code = ParameterErrorCode::TIMEOUT;
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Parameter service spin lock timeout (%.1fs) - another operation may be blocking",
                  service_timeout_sec_ + kSpinLockMarginSec);
    }
    return std::nullopt;
  }
  return lock;
}

std::chrono::duration<double> Ros2ParameterTransport::get_service_timeout() const {
  return std::chrono::duration<double>(service_timeout_sec_);
}

bool Ros2ParameterTransport::is_node_unavailable(const std::string & node_name) const {
  std::shared_lock<std::shared_mutex> lock(negative_cache_mutex_);
  auto it = unavailable_nodes_.find(node_name);
  if (it == unavailable_nodes_.end()) {
    return false;
  }
  auto elapsed = std::chrono::steady_clock::now() - it->second;
  return elapsed < std::chrono::duration<double>(negative_cache_ttl_sec_);
}

void Ros2ParameterTransport::mark_node_unavailable(const std::string & node_name) {
  std::unique_lock<std::shared_mutex> lock(negative_cache_mutex_);
  unavailable_nodes_[node_name] = std::chrono::steady_clock::now();

  // Cleanup to prevent unbounded growth.
  if (unavailable_nodes_.size() > kMaxNegativeCacheSize) {
    auto now = std::chrono::steady_clock::now();
    auto ttl = std::chrono::duration<double>(negative_cache_ttl_sec_);
    for (auto it = unavailable_nodes_.begin(); it != unavailable_nodes_.end();) {
      if (now - it->second > ttl) {
        it = unavailable_nodes_.erase(it);
      } else {
        ++it;
      }
    }
    if (unavailable_nodes_.size() > kMaxNegativeCacheSize) {
      auto oldest =
          std::min_element(unavailable_nodes_.begin(), unavailable_nodes_.end(), [](const auto & a, const auto & b) {
            return a.second < b.second;
          });
      if (oldest != unavailable_nodes_.end()) {
        unavailable_nodes_.erase(oldest);
      }
    }
  }
}

std::shared_ptr<rclcpp::AsyncParametersClient> Ros2ParameterTransport::get_param_client(const std::string & node_name) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  if (auto * cached = param_clients_.find(node_name)) {
    return *cached;
  }
  if (!param_node_) {
    return nullptr;
  }
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(param_node_, node_name);
  return param_clients_.put(node_name, client);
}

std::size_t Ros2ParameterTransport::param_client_cache_size() const {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  return param_clients_.size();
}

std::size_t Ros2ParameterTransport::default_values_cache_size() const {
  std::lock_guard<std::mutex> lock(defaults_mutex_);
  return default_values_.size();
}

void Ros2ParameterTransport::warn_defaults_evicted(const std::string & node_name) const {
  if (!node_) {
    return;
  }
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 60000,
                       "Default-value cache over capacity; evicted cached defaults for node '%s'. A later "
                       "reset-to-default for a parameter on this node may restore its current value instead of "
                       "its original one. Increase the defaults cache size if this recurs.",
                       node_name.c_str());
}

template <typename FutureT>
rclcpp::FutureReturnCode Ros2ParameterTransport::spin_for(const FutureT & future) {
  // Copy the node shared_ptr ONCE under clients_mutex_ and spin the local copy.
  // spin_mutex_ (held by the caller) serialises spins in the normal case, but
  // shutdown() has a degraded path: if it cannot acquire spin_mutex_ within
  // kShutdownTimeout it resets param_node_ anyway. Reading the member directly here
  // would then race that reset (concurrent access to the same shared_ptr instance =
  // UB). shutdown() clears param_clients_ and resets param_node_ under clients_mutex_,
  // so taking clients_mutex_ synchronises the read; the local copy also keeps the node
  // alive for the whole spin even if shutdown resets the member mid-call (#531).
  std::shared_ptr<rclcpp::Node> node;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    node = param_node_;
  }
  // Guard against spinning a torn-down node (shutdown races with an in-flight op).
  if (shutdown_requested_.load() || !node) {
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }
  // Free-function overload spins a fresh SingleThreadedExecutor over the node
  // (collecting all current entities, including a just-created client) for up to
  // service_timeout_sec, then tears it down - no persistent executor to reconcile
  // with param_node_'s teardown. Serialised by spin_mutex_ (held by the caller),
  // so only one spin runs on the node at a time.
  return rclcpp::spin_until_future_complete(node, future, get_service_timeout());
}

ParameterResult Ros2ParameterTransport::list_own_parameters() {
  ParameterResult result;
  try {
    auto params = node_->get_parameters(node_->list_parameters({}, 0).names);

    // Cache defaults for reset operations (same as IPC path).
    {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      if (!default_values_.contains(own_node_fqn_)) {
        std::map<std::string, rclcpp::Parameter> node_defaults;
        for (const auto & param : params) {
          node_defaults[param.get_name()] = param;
        }
        default_values_.put(own_node_fqn_, std::move(node_defaults));
      }
    }

    json params_array = json::array();
    for (const auto & param : params) {
      json param_obj;
      param_obj["name"] = param.get_name();
      param_obj["value"] = parameter_value_for_display(param.get_name(), param.get_parameter_value());
      param_obj["type"] = parameter_type_to_string(param.get_type());
      params_array.push_back(param_obj);
    }
    result.success = true;
    result.data = params_array;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to list own parameters: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }
  return result;
}

ParameterResult Ros2ParameterTransport::get_own_parameter(const std::string & param_name) {
  ParameterResult result;
  try {
    if (!node_->has_parameter(param_name)) {
      result.success = false;
      result.error_message = "Parameter not found: " + param_name;
      result.error_code = ParameterErrorCode::NOT_FOUND;
      return result;
    }
    auto param = node_->get_parameter(param_name);
    auto descriptor = node_->describe_parameter(param_name);

    json param_obj;
    param_obj["name"] = param.get_name();
    param_obj["value"] = parameter_value_for_display(param.get_name(), param.get_parameter_value());
    param_obj["type"] = parameter_type_to_string(param.get_type());
    param_obj["description"] = descriptor.description;
    param_obj["read_only"] = descriptor.read_only;

    result.success = true;
    result.data = param_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to get own parameter: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }
  return result;
}

ParameterResult Ros2ParameterTransport::list_parameters(const std::string & node_name) {
  if (shutdown_requested_.load()) {
    return {false, {}, "Ros2ParameterTransport is shut down", ParameterErrorCode::SHUT_DOWN};
  }

  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  ParameterResult result;

  try {
    std::vector<rclcpp::Parameter> parameters;
    {
      auto spin_lock = try_acquire_spin_lock(result);
      if (!spin_lock) {
        return result;
      }

      // Get (or create) the client INSIDE the spin lock, exactly as get_parameter
      // and set_parameter do. Creating or LRU-evicting an AsyncParametersClient
      // mutates param_node_'s entity set, and spin_for() reads that set under
      // spin_mutex_. Doing it outside the lock would race a concurrent spin on
      // param_node_: an evicted client's rcl entities could be destroyed mid-spin
      // (use-after-free), reachable once the cache is at capacity and two parameter
      // requests run at once - the exact multi-node fan-out this transport serves.
      auto client = get_param_client(node_name);
      if (!client) {
        result.success = false;
        result.error_message = "Parameter client unavailable (transport shut down)";
        result.error_code = ParameterErrorCode::SHUT_DOWN;
        return result;
      }

      // Cache default values first (gives node extra time for DDS service discovery).
      // This is itself a bounded round-trip. If it just failed/timed out, short-circuit
      // rather than spending a SECOND up-to-service_timeout_sec_ round-trip below:
      // holding spin_mutex_ across both would exceed the acquisition margin granted by
      // try_acquire_spin_lock() and spuriously TIMEOUT a concurrent request to a
      // different, healthy node (#531).
      //
      // Gate on cache_default_values' own return value, NOT is_node_unavailable():
      // the negative cache is a no-op when negative_cache_ttl_sec_ == 0 (a documented,
      // in-range "disabled" value), so is_node_unavailable() would always be false there
      // and the cap would silently break, reintroducing the ~2x-timeout hold.
      if (cache_default_values(node_name)) {
        result.success = false;
        result.error_message = "Parameter service not available for node: " + node_name;
        result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
        return result;
      }

      // Bail out before the blocking wait once teardown has begun. shutdown() sets this
      // flag before it blocks on spin_mutex_, and our pre-shutdown callback runs BEFORE
      // rcl_shutdown - so wait_for_service's own rclcpp::ok early exit never trips and it
      // would burn its full service timeout while holding spin_mutex_, gating rcl_shutdown
      // for the whole process. Deliberately does NOT mark the negative cache: this says
      // nothing about whether the target node is reachable.
      if (shutdown_requested_.load()) {
        result.success = false;
        result.error_message = "Parameter client unavailable (transport shut down)";
        result.error_code = ParameterErrorCode::SHUT_DOWN;
        return result;
      }
      if (!client->wait_for_service(get_service_timeout())) {
        result.success = false;
        result.error_message = "Parameter service not available for node: " + node_name;
        result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
        mark_node_unavailable(node_name);
        if (node_) {
          RCLCPP_WARN(node_->get_logger(), "Parameter service not available for node: '%s' (cached for %.0fs)",
                      node_name.c_str(), negative_cache_ttl_sec_);
        }
        return result;
      }

      std::vector<std::string> param_names;
      {
        auto list_future = client->list_parameters({}, 0);
        auto rc = spin_for(list_future);
        if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
          // The node did not answer the list request in time (explicit TIMEOUT, not
          // an ambiguous empty vector). Negative-cache it and report 503 (#531).
          mark_node_unavailable(node_name);
          result.success = false;
          result.error_message = "Parameter service did not respond for node: " + node_name;
          result.error_code = ParameterErrorCode::TIMEOUT;
          return result;
        }
        if (rc != rclcpp::FutureReturnCode::SUCCESS) {
          // INTERRUPTED: transport is shutting down. Do NOT mark (node is fine).
          result.success = false;
          result.error_message = "Parameter query interrupted for node: " + node_name;
          result.error_code = ParameterErrorCode::SHUT_DOWN;
          return result;
        }
        param_names = list_future.get().names;
      }

      // Empty names on SUCCESS = a genuine zero-parameter node -> success=[], no mark.
      if (!param_names.empty()) {
        auto get_future = client->get_parameters(param_names);
        auto rc = spin_for(get_future);
        if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
          mark_node_unavailable(node_name);
          result.success = false;
          result.error_message = "Parameter service did not respond for node: " + node_name;
          result.error_code = ParameterErrorCode::TIMEOUT;
          return result;
        }
        if (rc != rclcpp::FutureReturnCode::SUCCESS) {
          result.success = false;
          result.error_message = "Parameter query interrupted for node: " + node_name;
          result.error_code = ParameterErrorCode::SHUT_DOWN;
          return result;
        }
        // SUCCESS: the batch result is authoritative. It may hold fewer entries than
        // param_names (params raced/undeclared between list and get) or even be empty
        // ("listed but not currently gettable") - that is a LEGIT response from a
        // responsive node, returned as-is, NOT a timeout. No mark. The Sync-era
        // per-name fallback (which existed only to disambiguate a timeout-induced
        // empty batch) is gone: TIMEOUT is now explicit above (#531).
        parameters = get_future.get();
      }
    }  // spin_mutex_ released - JSON building is lock-free.

    json params_array = json::array();
    for (const auto & param : parameters) {
      json param_obj;
      param_obj["name"] = param.get_name();
      param_obj["value"] = parameter_value_for_display(param.get_name(), param.get_parameter_value());
      param_obj["type"] = parameter_type_to_string(param.get_type());
      params_array.push_back(param_obj);
    }

    result.success = true;
    result.data = params_array;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to list parameters: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "Exception in list_parameters for node '%s': %s", node_name.c_str(), e.what());
    }
  }

  return result;
}

ParameterResult Ros2ParameterTransport::get_parameter(const std::string & node_name, const std::string & param_name) {
  if (shutdown_requested_.load()) {
    return {false, {}, "Ros2ParameterTransport is shut down", ParameterErrorCode::SHUT_DOWN};
  }

  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  ParameterResult result;

  try {
    rclcpp::Parameter param;
    std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;

    {
      auto spin_lock = try_acquire_spin_lock(result);
      if (!spin_lock) {
        return result;
      }
      auto client = get_param_client(node_name);
      if (!client) {
        result.success = false;
        result.error_message = "Parameter client unavailable (transport shut down)";
        result.error_code = ParameterErrorCode::SHUT_DOWN;
        return result;
      }

      // Teardown bail-out before the blocking wait - see list_parameters() for why.
      if (shutdown_requested_.load()) {
        result.success = false;
        result.error_message = "Parameter client unavailable (transport shut down)";
        result.error_code = ParameterErrorCode::SHUT_DOWN;
        return result;
      }
      if (!client->wait_for_service(get_service_timeout())) {
        result.success = false;
        result.error_message = "Parameter service not available for node: " + node_name;
        result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
        mark_node_unavailable(node_name);
        return result;
      }

      // Confirm the parameter exists via list. TIMEOUT -> node unreachable (503);
      // SUCCESS with empty names -> the parameter genuinely does not exist (NOT_FOUND,
      // node is responsive so NO mark).
      std::vector<std::string> found_names;
      {
        auto list_future = client->list_parameters({param_name}, 1);
        auto rc = spin_for(list_future);
        if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
          mark_node_unavailable(node_name);
          result.success = false;
          result.error_message = "Parameter service did not respond for node: " + node_name;
          result.error_code = ParameterErrorCode::TIMEOUT;
          return result;
        }
        if (rc != rclcpp::FutureReturnCode::SUCCESS) {
          result.success = false;
          result.error_message = "Parameter query interrupted for node: " + node_name;
          result.error_code = ParameterErrorCode::SHUT_DOWN;
          return result;
        }
        found_names = list_future.get().names;
      }

      if (found_names.empty()) {
        result.success = false;
        result.error_message = "Parameter not found: " + param_name;
        result.error_code = ParameterErrorCode::NOT_FOUND;
        return result;
      }

      // Read the value. TIMEOUT -> node unreachable (503) + mark. SUCCESS with an
      // EMPTY result -> the node answered but has no value for this name (unset /
      // raced / undeclared between list and get): that is NOT a timeout, so treat it
      // as NOT_FOUND and DO NOT mark the (responsive) node. Distinguishing these two
      // is the whole point of the async client - the sync client returned the same
      // empty vector for both and wrongly 503'd the healthy node (#531).
      std::vector<rclcpp::Parameter> parameters;
      {
        auto get_future = client->get_parameters({param_name});
        auto rc = spin_for(get_future);
        if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
          mark_node_unavailable(node_name);
          result.success = false;
          result.error_message = "Parameter service did not respond for node: " + node_name;
          result.error_code = ParameterErrorCode::TIMEOUT;
          return result;
        }
        if (rc != rclcpp::FutureReturnCode::SUCCESS) {
          result.success = false;
          result.error_message = "Parameter query interrupted for node: " + node_name;
          result.error_code = ParameterErrorCode::SHUT_DOWN;
          return result;
        }
        parameters = get_future.get();
      }

      if (parameters.empty()) {
        result.success = false;
        result.error_message = "Parameter not currently set: " + param_name;
        result.error_code = ParameterErrorCode::NOT_FOUND;
        return result;
      }

      param = parameters[0];

      // describe_parameters is NON-ESSENTIAL (only feeds the optional description /
      // read_only fields). Best-effort: only use it if the round trip SUCCEEDs;
      // a TIMEOUT or interruption here just skips the descriptor - never mark, never
      // fail, keep the value we already have (#531).
      {
        auto desc_future = client->describe_parameters({param_name});
        if (spin_for(desc_future) == rclcpp::FutureReturnCode::SUCCESS) {
          descriptors = desc_future.get();
        } else if (node_) {
          RCLCPP_DEBUG(node_->get_logger(),
                       "describe_parameters did not complete for '%s' on node '%s'; "
                       "returning value without descriptor",
                       param_name.c_str(), node_name.c_str());
        }
      }
    }  // spin_mutex_ released.

    json param_obj;
    param_obj["name"] = param.get_name();
    param_obj["value"] = parameter_value_for_display(param.get_name(), param.get_parameter_value());
    param_obj["type"] = parameter_type_to_string(param.get_type());

    if (!descriptors.empty()) {
      param_obj["description"] = descriptors[0].description;
      param_obj["read_only"] = descriptors[0].read_only;
    }

    result.success = true;
    result.data = param_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to get parameter: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }

  return result;
}

ParameterResult Ros2ParameterTransport::set_parameter(const std::string & node_name, const std::string & param_name,
                                                      const json & value) {
  if (shutdown_requested_.load()) {
    return {false, {}, "Ros2ParameterTransport is shut down", ParameterErrorCode::SHUT_DOWN};
  }

  // Self-query guard: set parameter directly on own node.
  if (is_self_node(node_name)) {
    ParameterResult result;
    try {
      if (!node_->has_parameter(param_name)) {
        result.success = false;
        result.error_message = "Parameter not found: " + param_name;
        result.error_code = ParameterErrorCode::NOT_FOUND;
        return result;
      }
      auto current_value = node_->get_parameter(param_name).get_parameter_value();
      rclcpp::ParameterValue param_value = json_to_parameter_value(value, current_value.get_type());
      auto set_result = node_->set_parameter(rclcpp::Parameter(param_name, param_value));
      if (!set_result.successful) {
        result.success = false;
        result.error_message = set_result.reason;
        result.error_code = ParameterErrorCode::INVALID_VALUE;
        return result;
      }
      json param_obj;
      param_obj["name"] = param_name;
      param_obj["value"] = parameter_value_for_display(param_name, param_value);
      param_obj["type"] = parameter_type_to_string(param_value.get_type());
      result.success = true;
      result.data = param_obj;
    } catch (const std::exception & e) {
      result.success = false;
      result.error_message = std::string("Failed to set own parameter: ") + e.what();
      result.error_code = ParameterErrorCode::INTERNAL_ERROR;
    }
    return result;
  }

  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  ParameterResult result;

  auto spin_lock = try_acquire_spin_lock(result);
  if (!spin_lock) {
    return result;
  }

  try {
    auto client = get_param_client(node_name);
    if (!client) {
      result.success = false;
      result.error_message = "Parameter client unavailable (transport shut down)";
      result.error_code = ParameterErrorCode::SHUT_DOWN;
      return result;
    }

    // Teardown bail-out before the blocking wait - see list_parameters() for why.
    if (shutdown_requested_.load()) {
      result.success = false;
      result.error_message = "Parameter client unavailable (transport shut down)";
      result.error_code = ParameterErrorCode::SHUT_DOWN;
      return result;
    }
    if (!client->wait_for_service(get_service_timeout())) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
      mark_node_unavailable(node_name);
      return result;
    }

    // Pre-read the current value purely to derive a type hint. Best-effort: a TIMEOUT
    // or empty result here just leaves hint_type NOT_SET (json_to_parameter_value then
    // infers from the JSON). Do NOT mark on the pre-read - it is not the authoritative
    // write-timeout signal (the set below is) (#531).
    std::vector<rclcpp::Parameter> current_params;
    {
      auto get_future = client->get_parameters({param_name});
      if (spin_for(get_future) == rclcpp::FutureReturnCode::SUCCESS) {
        current_params = get_future.get();
      } else if (node_) {
        RCLCPP_DEBUG(node_->get_logger(), "Pre-read of '%s' on node '%s' did not complete (continuing without hint)",
                     param_name.c_str(), node_name.c_str());
      }
    }
    rclcpp::ParameterType hint_type = rclcpp::ParameterType::PARAMETER_NOT_SET;
    if (!current_params.empty()) {
      hint_type = current_params[0].get_type();
    }

    // json_to_parameter_value can throw on bad CLIENT input (e.g. malformed value for
    // the parameter's type). It runs OUTSIDE any mark scope so a bad client value never
    // negative-caches a healthy node; the outer catch maps a throw to INTERNAL_ERROR.
    rclcpp::ParameterValue param_value = json_to_parameter_value(value, hint_type);
    rclcpp::Parameter param(param_name, param_value);

    std::vector<rcl_interfaces::msg::SetParametersResult> results;
    {
      auto set_future = client->set_parameters({param});
      auto rc = spin_for(set_future);
      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        // The node did not answer the write in time (explicit TIMEOUT). Negative-cache
        // and report 503, not 500 (#531).
        //
        // At-least-once ambiguity: a TIMEOUT/503 here does NOT guarantee the set did NOT
        // apply on the node - the request may have been delivered and executed with only
        // the response lost. Callers must re-read the parameter to confirm the effective
        // value rather than assuming the write was rejected.
        mark_node_unavailable(node_name);
        result.success = false;
        result.error_message = "Parameter service did not respond for node: " + node_name;
        result.error_code = ParameterErrorCode::TIMEOUT;
        return result;
      }
      if (rc != rclcpp::FutureReturnCode::SUCCESS) {
        result.success = false;
        result.error_message = "Parameter set interrupted for node: " + node_name;
        result.error_code = ParameterErrorCode::SHUT_DOWN;
        return result;
      }
      results = set_future.get();
    }

    if (results.empty()) {
      // SUCCESS but no result: a responsive node returns one SetParametersResult per
      // param, so an empty results vector on SUCCESS is a node-side anomaly, NOT a
      // timeout. Report INTERNAL_ERROR and do NOT mark the (responsive) node (#531).
      result.success = false;
      result.error_message = "Set parameter returned no result for: " + param_name;
      result.error_code = ParameterErrorCode::INTERNAL_ERROR;
      return result;
    }

    if (!results[0].successful) {
      // Legitimate rejection by a RESPONSIVE node (read-only / type / invalid value):
      // classify by reason, NOT a timeout.
      result.success = false;
      result.error_message = results[0].reason;
      const auto & reason = results[0].reason;
      if (reason.find("read-only") != std::string::npos || reason.find("read only") != std::string::npos ||
          reason.find("is read_only") != std::string::npos) {
        result.error_code = ParameterErrorCode::READ_ONLY;
      } else if (reason.find("type") != std::string::npos) {
        result.error_code = ParameterErrorCode::TYPE_MISMATCH;
      } else {
        result.error_code = ParameterErrorCode::INVALID_VALUE;
      }
      return result;
    }

    // The write succeeded. Record what the value was BEFORE this write (from the
    // pre-read above) so reset-to-default can later restore the pre-write value -
    // even if the first-seen default snapshot was captured after a write, or was
    // evicted and re-seeded from the current (post-write) value. emplace records
    // only the FIRST write per (node, param) - the original value - and only when
    // the pre-read actually returned a value, so this reuses that read and adds no
    // round trip.
    if (!current_params.empty()) {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      auto * node_pre_write = pre_write_values_.find(node_name);
      if (node_pre_write == nullptr) {
        pre_write_values_.put(node_name, {{param_name, current_params[0]}});
      } else {
        node_pre_write->emplace(param_name, current_params[0]);  // no-op if already recorded
      }
    }

    json param_obj;
    param_obj["name"] = param_name;
    param_obj["value"] = parameter_value_for_display(param_name, param_value);
    param_obj["type"] = parameter_type_to_string(param_value.get_type());

    result.success = true;
    result.data = param_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to set parameter: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }

  return result;
}

ParameterResult Ros2ParameterTransport::get_default(const std::string & node_name, const std::string & param_name) {
  if (shutdown_requested_.load()) {
    return {false, {}, "Ros2ParameterTransport is shut down", ParameterErrorCode::SHUT_DOWN};
  }

  // Prefer the pre-write record: if the gateway overwrote this parameter, the
  // authoritative default for reset is the value it had before that write, not
  // the first-seen snapshot (which could have been captured after the write, or
  // evicted and re-seeded from the current value). This also short-circuits the
  // defaults round trip below when a pre-write value is on record.
  {
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    if (auto * node_pre_write = pre_write_values_.find(node_name)) {
      auto pre_it = node_pre_write->find(param_name);
      if (pre_it != node_pre_write->end()) {
        ParameterResult pre_result;
        pre_result.success = true;
        pre_result.data = parameter_value_for_display(param_name, pre_it->second.get_parameter_value());
        return pre_result;
      }
    }
  }

  // For self-node, populate the defaults cache lazily from the live node
  // state (same path list_own_parameters() takes, but without the JSON
  // assembly side-effect) in case the manager calls reset before list.
  if (is_self_node(node_name)) {
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    if (!default_values_.contains(node_name)) {
      try {
        auto params = node_->get_parameters(node_->list_parameters({}, 0).names);
        std::map<std::string, rclcpp::Parameter> node_defaults;
        for (const auto & param : params) {
          node_defaults[param.get_name()] = param;
        }
        default_values_.put(node_name, std::move(node_defaults));
      } catch (const std::exception & e) {
        ParameterResult result;
        result.success = false;
        result.error_message = std::string("Failed to seed own defaults cache: ") + e.what();
        result.error_code = ParameterErrorCode::INTERNAL_ERROR;
        return result;
      }
    }
  } else {
    // Non-self: ensure defaults are cached. cache_default_values requires
    // spin_mutex_ to be held.
    bool needs_cache = false;
    {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      needs_cache = !default_values_.contains(node_name);
    }
    if (needs_cache) {
      ParameterResult tmp;
      auto spin_lock = try_acquire_spin_lock(tmp);
      if (!spin_lock) {
        return tmp;
      }
      cache_default_values(node_name);
    }
  }

  ParameterResult result;
  std::lock_guard<std::mutex> lock(defaults_mutex_);
  auto * node_defaults = default_values_.find(node_name);
  if (node_defaults == nullptr) {
    result.success = false;
    result.error_message = "No default values cached for node: " + node_name;
    result.error_code = ParameterErrorCode::NO_DEFAULTS_CACHED;
    return result;
  }
  auto param_it = node_defaults->find(param_name);
  if (param_it == node_defaults->end()) {
    result.success = false;
    result.error_message = "No default value for parameter: " + param_name;
    result.error_code = ParameterErrorCode::NOT_FOUND;
    return result;
  }
  result.success = true;
  result.data = parameter_value_for_display(param_name, param_it->second.get_parameter_value());
  return result;
}

ParameterResult Ros2ParameterTransport::list_defaults(const std::string & node_name) {
  if (shutdown_requested_.load()) {
    return {false, {}, "Ros2ParameterTransport is shut down", ParameterErrorCode::SHUT_DOWN};
  }

  // Mirror the get_default cache-population logic.
  if (is_self_node(node_name)) {
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    if (!default_values_.contains(node_name)) {
      try {
        auto params = node_->get_parameters(node_->list_parameters({}, 0).names);
        std::map<std::string, rclcpp::Parameter> node_defaults;
        for (const auto & param : params) {
          node_defaults[param.get_name()] = param;
        }
        default_values_.put(node_name, std::move(node_defaults));
      } catch (const std::exception & e) {
        ParameterResult result;
        result.success = false;
        result.error_message = std::string("Failed to seed own defaults cache: ") + e.what();
        result.error_code = ParameterErrorCode::INTERNAL_ERROR;
        return result;
      }
    }
  } else {
    bool needs_cache = false;
    {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      needs_cache = !default_values_.contains(node_name);
    }
    if (needs_cache) {
      ParameterResult tmp;
      auto spin_lock = try_acquire_spin_lock(tmp);
      if (!spin_lock) {
        return tmp;
      }
      cache_default_values(node_name);
    }
  }

  ParameterResult result;
  std::lock_guard<std::mutex> lock(defaults_mutex_);
  auto * node_defaults = default_values_.find(node_name);
  auto * node_pre_write = pre_write_values_.find(node_name);
  if (node_defaults == nullptr && node_pre_write == nullptr) {
    result.success = false;
    result.error_message = "No default values cached for node: " + node_name;
    result.error_code = ParameterErrorCode::NO_DEFAULTS_CACHED;
    return result;
  }

  // Merge the first-seen snapshot with the pre-write record, pre-write taking
  // precedence - the same preference get_default applies - so the listing reports
  // the same defaults reset would restore.
  std::map<std::string, rclcpp::Parameter> merged;
  if (node_defaults != nullptr) {
    merged = *node_defaults;
  }
  if (node_pre_write != nullptr) {
    for (const auto & [name, param] : *node_pre_write) {
      merged[name] = param;
    }
  }

  json defaults_array = json::array();
  for (const auto & [name, param] : merged) {
    json entry;
    entry["name"] = name;
    entry["value"] = parameter_value_for_display(name, param.get_parameter_value());
    entry["type"] = parameter_type_to_string(param.get_type());
    defaults_array.push_back(entry);
  }
  result.success = true;
  result.data = defaults_array;
  return result;
}

std::string Ros2ParameterTransport::parameter_type_to_string(rclcpp::ParameterType type) const {
  switch (type) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return "bool";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return "int";
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return "double";
    case rclcpp::ParameterType::PARAMETER_STRING:
      return "string";
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return "byte_array";
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return "bool_array";
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return "int_array";
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return "double_array";
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return "string_array";
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      return "not_set";
  }
}

json Ros2ParameterTransport::parameter_value_to_json(const rclcpp::ParameterValue & value) const {
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return value.get<bool>();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return value.get<int64_t>();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return value.get<double>();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return value.get<std::string>();
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return json(value.get<std::vector<uint8_t>>());
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return json(value.get<std::vector<bool>>());
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return json(value.get<std::vector<int64_t>>());
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return json(value.get<std::vector<double>>());
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return json(value.get<std::vector<std::string>>());
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      return nullptr;
  }
}

rclcpp::ParameterValue Ros2ParameterTransport::json_to_parameter_value(const json & value,
                                                                       rclcpp::ParameterType hint_type) const {
  if (hint_type != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    switch (hint_type) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        if (value.is_boolean()) {
          return rclcpp::ParameterValue(value.get<bool>());
        }
        if (value.is_string()) {
          std::string s = value.get<std::string>();
          return rclcpp::ParameterValue(s == "true" || s == "1");
        }
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (value.is_number_integer()) {
          return rclcpp::ParameterValue(value.get<int64_t>());
        }
        if (value.is_number_float()) {
          return rclcpp::ParameterValue(static_cast<int64_t>(value.get<double>()));
        }
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        if (value.is_number()) {
          return rclcpp::ParameterValue(value.get<double>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        if (value.is_string()) {
          return rclcpp::ParameterValue(value.get<std::string>());
        }
        return rclcpp::ParameterValue(value.dump());
      case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<bool>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<bool>>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<int64_t>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<int64_t>>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<double>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<double>>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<std::string>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<std::string>>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
      case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      default:
        break;
    }
  }

  if (value.is_boolean()) {
    return rclcpp::ParameterValue(value.get<bool>());
  }
  if (value.is_number_integer()) {
    return rclcpp::ParameterValue(value.get<int64_t>());
  }
  if (value.is_number_float()) {
    return rclcpp::ParameterValue(value.get<double>());
  }
  if (value.is_string()) {
    return rclcpp::ParameterValue(value.get<std::string>());
  }
  if (value.is_array()) {
    if (value.empty()) {
      return rclcpp::ParameterValue(std::vector<std::string>{});
    }
    if (value[0].is_boolean()) {
      return rclcpp::ParameterValue(value.get<std::vector<bool>>());
    }
    if (value[0].is_number_integer()) {
      return rclcpp::ParameterValue(value.get<std::vector<int64_t>>());
    }
    if (value[0].is_number_float()) {
      return rclcpp::ParameterValue(value.get<std::vector<double>>());
    }
    if (value[0].is_string()) {
      return rclcpp::ParameterValue(value.get<std::vector<std::string>>());
    }
  }

  return rclcpp::ParameterValue(value.dump());
}

bool Ros2ParameterTransport::cache_default_values(const std::string & node_name) {
  {
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    if (default_values_.contains(node_name)) {
      return false;  // already cached from a prior successful round-trip
    }
  }

  if (is_node_unavailable(node_name)) {
    return true;  // already negative-cached: caller should short-circuit
  }

  try {
    auto client = get_param_client(node_name);
    if (!client) {
      return false;  // transport shutting down, not a node-availability signal
    }

    // Teardown bail-out before the blocking wait - see list_parameters() for why. Returns
    // false so the caller treats it as "not a node-availability signal", matching the
    // shut-down client path above.
    if (shutdown_requested_.load()) {
      return false;
    }
    if (!client->wait_for_service(get_service_timeout())) {
      if (node_) {
        RCLCPP_WARN(node_->get_logger(), "Cannot cache defaults - service not available for node: '%s'",
                    node_name.c_str());
      }
      mark_node_unavailable(node_name);
      return true;  // service not discoverable: the round-trip cannot proceed
    }

    std::vector<std::string> param_names;
    {
      auto list_future = client->list_parameters({}, 0);
      auto rc = spin_for(list_future);
      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        // Node did not answer: negative-cache and short-circuit. Crucially, do NOT
        // cache anything - caching an empty map on a timeout would poison
        // default_values_ (entries are only evicted under LRU cache pressure, never
        // invalidated on recovery) so get_default/reset return NOT_FOUND until the
        // entry ages out even after the node recovers (#531).
        mark_node_unavailable(node_name);
        return true;
      }
      if (rc != rclcpp::FutureReturnCode::SUCCESS) {
        return true;  // INTERRUPTED (shutting down): short-circuit, no mark, no cache
      }
      param_names = list_future.get().names;
    }

    if (param_names.empty()) {
      // Genuine zero-parameter node (responsive, SUCCESS with empty names). This is a
      // STABLE, authoritative fact, so memoize an EMPTY defaults map: get_default then
      // returns NOT_FOUND (not NO_DEFAULTS_CACHED) and subsequent list/get/reset calls
      // do not re-round-trip under spin_mutex_ forever. Only a TIMEOUT must avoid caching
      // (poison-avoidance); a SUCCESS-empty is safe to cache (#531).
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      if (!default_values_.contains(node_name)) {
        default_values_.put(node_name, {});
      }
      return false;
    }

    std::vector<rclcpp::Parameter> parameters;
    {
      auto get_future = client->get_parameters(param_names);
      auto rc = spin_for(get_future);
      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        mark_node_unavailable(node_name);
        return true;  // timed out on the value read: do NOT cache (no empty-map poison)
      }
      if (rc != rclcpp::FutureReturnCode::SUCCESS) {
        return true;
      }
      parameters = get_future.get();
    }

    // SUCCESS with values (or a responsive node that returned an empty set - unset /
    // undeclared / raced): both are authoritative, so cache the resulting map. An empty
    // map here is a genuine SUCCESS-empty and is SAFE to memoize (unlike the TIMEOUT
    // branches above, which must not cache to avoid permanent poison) (#531).
    std::map<std::string, rclcpp::Parameter> node_defaults;
    for (const auto & param : parameters) {
      node_defaults[param.get_name()] = param;
    }

    {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      if (!default_values_.contains(node_name)) {
        default_values_.put(node_name, std::move(node_defaults));
      }
    }
  } catch (const std::exception & e) {
    // Defensive: no parameter round trip is expected to throw now (TIMEOUT/INTERRUPTED
    // are explicit FutureReturnCodes and future.get() on SUCCESS does not throw). Any
    // unexpected throw short-circuits without caching; do NOT mark (not proven a node
    // timeout).
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "Unexpected error caching defaults for node '%s': %s", node_name.c_str(),
                   e.what());
    }
    return true;
  }
  return false;  // defaults cached successfully
}

bool Ros2ParameterTransport::is_secret_parameter(const std::string & name) {
  // Exact names, deliberately not a substring match on "secret" or "token":
  // a substring rule hides ordinary parameters whose names happen to contain
  // the word, and the operator then cannot read a value that is not a secret.
  static const std::set<std::string> kSecretParameters{
      "auth.jwt_secret",              // signs this gateway's tokens
      "auth.clients",                 // holds client_id:client_secret:role
      "aggregation.peer_auth_header"  // what this gateway presents to peers
  };
  return kSecretParameters.find(name) != kSecretParameters.end();
}

json Ros2ParameterTransport::parameter_value_for_display(const std::string & name,
                                                         const rclcpp::ParameterValue & value) const {
  if (is_secret_parameter(name)) {
    // A fixed marker rather than the value's own shape. It reads as redaction
    // in any client, and the entry keeps its declared type so the parameter is
    // still discoverable - only its value is withheld.
    return json("***");
  }
  return parameter_value_to_json(value);
}

}  // namespace ros2_medkit_gateway::ros2
