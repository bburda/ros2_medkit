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

/// @file generic_client_compat.hpp
/// @brief Provides GenericServiceClient — either rclcpp::GenericClient (Iron+) or a shim for Humble.
///
/// rclcpp::GenericClient was introduced in Iron (rclcpp ≥ 21.x). Humble (rclcpp 16.x)
/// does not have it. This header defines HAS_GENERIC_CLIENT based on the rclcpp version
/// and provides a compatible GenericServiceClient type for OperationManager to use.
///
/// When HAS_GENERIC_CLIENT is true (Iron/Jazzy), GenericServiceClient is simply
/// rclcpp::GenericClient, and create_generic_service_client() delegates to
/// Node::create_generic_client().
///
/// When HAS_GENERIC_CLIENT is false (Humble), GenericServiceClient is a custom class
/// that replicates GenericClient's behavior using rcl C APIs and the same
/// rosidl_typesupport_introspection infrastructure available in all distros.
///
/// Both paths take the callback group the client's response callback should
/// be dispatched on (issue #575: the gateway registers its blocking-RPC
/// clients into a shared Reentrant group so responses are not serialized
/// behind the node's default MutuallyExclusive group). Passing nullptr falls
/// back to the node's default group on every distro.

#pragma once

#include <rclcpp/rclcpp.hpp>

// Detect whether rclcpp::GenericClient is available.
// rclcpp/version.h exists in all supported distros (Humble, Jazzy, Lyrical).
#include <rclcpp/version.h>

#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 21
#define HAS_GENERIC_CLIENT 1
#else
#define HAS_GENERIC_CLIENT 0
#endif

#if HAS_GENERIC_CLIENT

// ============================================================================
// Iron / Jazzy path — just alias to rclcpp::GenericClient
// ============================================================================

#include <rclcpp/generic_client.hpp>

#include <utility>

namespace ros2_medkit_gateway {
namespace compat {

using GenericServiceClient = rclcpp::GenericClient;

/// Create a GenericServiceClient (delegates to Node::create_generic_client).
/// @param group Callback group handling the reply callbacks; nullptr resolves
///              to the node's default group inside rclcpp.
inline GenericServiceClient::SharedPtr create_generic_service_client(rclcpp::Node * node,
                                                                     const std::string & service_name,
                                                                     const std::string & service_type,
                                                                     rclcpp::CallbackGroup::SharedPtr group) {
  return node->create_generic_client(service_name, service_type, rclcpp::ServicesQoS(), std::move(group));
}

}  // namespace compat
}  // namespace ros2_medkit_gateway

#else  // !HAS_GENERIC_CLIENT

// ============================================================================
// Humble compatibility shim — replicate GenericClient using rcl C APIs
// ============================================================================
//
// On Humble (rclcpp 16.x), the following APIs do NOT exist:
//   - rclcpp::GenericClient
//   - rclcpp::get_service_typesupport_handle()
//   - rclcpp::get_message_typesupport_handle()
//
// The following APIs ARE available:
//   - rclcpp::get_typesupport_library()  — loads the .so
//   - rclcpp::get_typesupport_handle()   — extracts message TS from .so
//   - rclcpp::ClientBase                 — base class for service clients
//   - rcl_client_init / rcl_send_request — C-level service client API
//   - rosidl_typesupport_introspection_cpp — message introspection
//
// This shim manually loads the service typesupport symbol from the library
// (the same way GenericClient does on Iron+, but with direct dlsym).

#include <rcl/client.h>
#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rmw/rmw.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_cpp/service_type_support_dispatch.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

namespace ros2_medkit_gateway {
namespace compat {

namespace detail {

/// Build the mangled symbol name for a service's rosidl_typesupport_cpp handle.
/// Example: "std_srvs/srv/Trigger" with ts_id "rosidl_typesupport_cpp"
///   => "rosidl_typesupport_cpp__get_service_type_support_handle__std_srvs__srv__Trigger"
inline std::string make_service_ts_symbol(const std::string & service_type,
                                          const std::string & typesupport_identifier) {
  // service_type format: "package/srv/ServiceName"
  std::string symbol = service_type;
  // Replace '/' with '__'
  std::string result;
  result.reserve(typesupport_identifier.size() + 40 + symbol.size());
  result += typesupport_identifier;
  result += "__get_service_type_support_handle__";
  for (char c : symbol) {
    if (c == '/') {
      result += "__";
    } else {
      result += c;
    }
  }
  return result;
}

/// Load the service typesupport handle from a shared library.
/// Equivalent to rclcpp::get_service_typesupport_handle() (available only in Iron+).
inline const rosidl_service_type_support_t * load_service_typesupport(const std::string & service_type,
                                                                      const std::string & typesupport_identifier,
                                                                      rcpputils::SharedLibrary & library) {
  std::string symbol_name = make_service_ts_symbol(service_type, typesupport_identifier);

  if (!library.has_symbol(symbol_name)) {
    throw std::runtime_error("Failed to find service typesupport symbol '" + symbol_name + "' in library");
  }

  // The symbol is a function: const rosidl_service_type_support_t* get_...()
  using GetServiceTSFunc = const rosidl_service_type_support_t * (*)();
  auto func = reinterpret_cast<GetServiceTSFunc>(library.get_symbol(symbol_name));
  if (!func) {
    throw std::runtime_error("Failed to load service typesupport function for '" + service_type + "'");
  }

  return func();
}

/// Get the introspection MessageMembers for the response message of a service.
/// The service_ts coming from "rosidl_typesupport_cpp" is a dispatch handle.
/// We use the dispatch function to obtain the introspection-flavor handle,
/// then cast its data to ServiceMembers to get response_members_.
/// Both rosidl_typesupport_cpp::get_service_typesupport_handle_function() and
/// rosidl_typesupport_introspection_cpp::ServiceMembers exist in Humble.
inline const rosidl_typesupport_introspection_cpp::MessageMembers *
get_response_introspection(const rosidl_service_type_support_t * service_ts) {
  // Dispatch from "rosidl_typesupport_cpp" → "rosidl_typesupport_introspection_cpp"
  const rosidl_service_type_support_t * intro_ts = rosidl_typesupport_cpp::get_service_typesupport_handle_function(
      service_ts, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (!intro_ts) {
    throw std::runtime_error("Failed to get introspection typesupport for service");
  }

  const auto * service_members =
      static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(intro_ts->data);
  return service_members->response_members_;
}

}  // namespace detail

/// Compatibility GenericClient for ROS 2 Humble.
/// Replicates the subset of rclcpp::GenericClient API used by OperationManager:
///   - async_send_request(void*)
///   - wait_for_service(timeout)
///   - remove_pending_request(request_id)
///
/// Implemented directly on top of rcl C APIs and dynamically loaded typesupport,
/// which are available in all ROS 2 distros from Humble onward.
class GenericServiceClient : public rclcpp::ClientBase {
 public:
  using Request = void *;
  using Response = void *;
  using SharedResponse = std::shared_ptr<void>;
  using Promise = std::promise<SharedResponse>;
  using Future = std::future<SharedResponse>;

  using SharedPtr = std::shared_ptr<GenericServiceClient>;

  /// FutureAndRequestId — wraps a std::future<SharedResponse> and associated request_id,
  /// mirroring rclcpp::GenericClient::FutureAndRequestId for API compatibility.
  struct FutureAndRequestId {
    Future future;
    int64_t request_id;

    FutureAndRequestId(Future f, int64_t id) : future(std::move(f)), request_id(id) {
    }

    ~FutureAndRequestId() = default;
    FutureAndRequestId(FutureAndRequestId && other) noexcept = default;
    FutureAndRequestId & operator=(FutureAndRequestId && other) noexcept = default;
    FutureAndRequestId(const FutureAndRequestId &) = delete;
    FutureAndRequestId & operator=(const FutureAndRequestId &) = delete;

    auto get() {
      return future.get();
    }
    bool valid() const noexcept {
      return future.valid();
    }
    void wait() const {
      future.wait();
    }

    template <class Rep, class Period>
    std::future_status wait_for(const std::chrono::duration<Rep, Period> & timeout) const {
      return future.wait_for(timeout);
    }
  };

  /// Construct a GenericServiceClient.
  /// @param node_base Node base interface (for rcl node handle)
  /// @param node_graph Node graph interface (for service availability checks)
  /// @param service_name Fully-qualified service name
  /// @param service_type Service type string, e.g. "std_srvs/srv/Trigger"
  /// @param client_options rcl client options
  GenericServiceClient(rclcpp::node_interfaces::NodeBaseInterface * node_base,
                       rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                       const std::string & service_name, const std::string & service_type,
                       rcl_client_options_t & client_options)
    : ClientBase(node_base, std::move(node_graph)) {
    // Step 1: Load the typesupport shared library for this service type
    ts_lib_ = rclcpp::get_typesupport_library(service_type, "rosidl_typesupport_cpp");

    // Step 2: Get the service type support handle from the loaded library
    // Note: rclcpp::get_service_typesupport_handle() doesn't exist in Humble,
    // so we manually resolve the symbol from the shared library.
    const rosidl_service_type_support_t * service_ts =
        detail::load_service_typesupport(service_type, "rosidl_typesupport_cpp", *ts_lib_);

    // Step 3: Get the introspection MessageMembers for the response type.
    // Used by create_response() to allocate and initialize response messages.
    response_members_ = detail::get_response_introspection(service_ts);

    // Step 4: Initialize the rcl client with the dynamically-loaded typesupport
    rcl_ret_t ret = rcl_client_init(this->get_client_handle().get(), this->get_rcl_node_handle(), service_ts,
                                    service_name.c_str(), &client_options);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to create generic service client");
    }
  }

  ~GenericServiceClient() override = default;

  /// Create a response message (allocated via introspection)
  std::shared_ptr<void> create_response() override {
    // Allocate buffer for the response message
    auto * response = new uint8_t[response_members_->size_of_];
    // Initialize with default values using the introspection init function
    response_members_->init_function(response, rosidl_runtime_cpp::MessageInitialization::ZERO);
    // Return with custom deleter that calls fini_function
    const auto * members = response_members_;
    return std::shared_ptr<void>(response, [members](void * ptr) {
      members->fini_function(ptr);
      delete[] static_cast<uint8_t *>(ptr);
    });
  }

  /// Create a request header for associating responses
  std::shared_ptr<rmw_request_id_t> create_request_header() override {
    return std::make_shared<rmw_request_id_t>();
  }

  /// Handle incoming response — dispatches to the pending promise
  void handle_response(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) override {
    std::unique_lock<std::mutex> lock(pending_requests_mutex_);
    auto it = pending_requests_.find(request_header->sequence_number);
    if (it != pending_requests_.end()) {
      auto promise = std::move(it->second);
      pending_requests_.erase(it);
      lock.unlock();
      promise.set_value(std::move(response));
    }
  }

  /// Send a request asynchronously. Returns a FutureAndRequestId.
  /// @param request Pointer to the deserialized request message (void*)
  FutureAndRequestId async_send_request(const Request request) {
    Promise promise;
    auto future = promise.get_future();

    // Send the request via rcl
    int64_t sequence_number;
    rcl_ret_t ret = rcl_send_request(get_client_handle().get(), request, &sequence_number);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send request");
    }

    // Store promise for later fulfillment when handle_response is called
    {
      std::lock_guard<std::mutex> lock(pending_requests_mutex_);
      pending_requests_.emplace(sequence_number, std::move(promise));
    }

    return FutureAndRequestId(std::move(future), sequence_number);
  }

  /// Remove a pending request (called on timeout to prevent resource leak)
  bool remove_pending_request(int64_t request_id) {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    return pending_requests_.erase(request_id) > 0;
  }

 private:
  /// Loaded typesupport library (kept alive for duration of client)
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;

  /// Response type introspection members (for allocating response messages)
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_;

  /// Map from sequence_number to pending promise
  std::mutex pending_requests_mutex_;
  std::map<int64_t, Promise> pending_requests_;
};

/// Create a GenericServiceClient for Humble.
/// @param group Callback group handling the reply callbacks; nullptr resolves
///              to the node's default group inside NodeServices::add_client.
inline GenericServiceClient::SharedPtr create_generic_service_client(rclcpp::Node * node,
                                                                     const std::string & service_name,
                                                                     const std::string & service_type,
                                                                     rclcpp::CallbackGroup::SharedPtr group) {
  rcl_client_options_t options = rcl_client_get_default_options();
  auto client = std::make_shared<GenericServiceClient>(
      node->get_node_base_interface().get(), node->get_node_graph_interface(), service_name, service_type, options);

  // Register the client with the given callback group so the executor polls
  // it for incoming responses. Without this, handle_response() is never
  // called and every future hangs until timeout.
  node->get_node_services_interface()->add_client(std::dynamic_pointer_cast<rclcpp::ClientBase>(client),
                                                  std::move(group));

  return client;
}

}  // namespace compat
}  // namespace ros2_medkit_gateway

#endif  // HAS_GENERIC_CLIENT
