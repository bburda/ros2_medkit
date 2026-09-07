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

#include "ros2_medkit_gateway/ros2/transports/ros2_fault_service_transport.hpp"

#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <string>

#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/msg/planned_stop.hpp"

namespace ros2_medkit_gateway::ros2 {

namespace {

// Synchronous fault-manager RPC under the transport's private executor.
//
// Mirrors the pattern in ros2_service_transport.cpp / ros2_action_transport.cpp
// (clamp negative timeouts, drop abandoned pending requests on timeout) and
// adapts it to the private-node / private-executor design from issue #399.
//
// wait_for_service runs outside `executor_mutex` because the graph listener
// that backs it is a per-context thread independent of any user executor; the
// local shared_ptr copies of `client` and `executor` keep the rclcpp state
// alive even if the transport destructor races the caller.
template <typename Service>
typename Service::Response::SharedPtr invoke_fault_service(
    // SharedPtrs are taken by value on purpose: they keep the rclcpp client
    // and executor alive for the duration of the call even if the transport
    // destructor concurrently resets the corresponding member shared_ptrs.
    typename rclcpp::Client<Service>::SharedPtr client,                   // NOLINT(performance-unnecessary-value-param)
    typename Service::Request::SharedPtr request,                         // NOLINT(performance-unnecessary-value-param)
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor,  // NOLINT(performance-unnecessary-value-param)
    std::mutex & executor_mutex, std::chrono::duration<double> timeout, const char * op_name, std::string & error_out) {
  const auto clamped = std::chrono::duration<double>(std::max(timeout.count(), 0.0));

  if (!client || !executor) {
    error_out = std::string(op_name) + " transport not initialised";
    return nullptr;
  }

  if (!client->wait_for_service(clamped)) {
    error_out = std::string(op_name) + " service not available";
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(executor_mutex);
  auto future = client->async_send_request(request);
  if (executor->spin_until_future_complete(future, clamped) != rclcpp::FutureReturnCode::SUCCESS) {
    // Drop the abandoned slot from the client's pending-request map; without
    // this, repeated timeouts leak entries until the client is destroyed.
    client->remove_pending_request(future.request_id);
    error_out = std::string(op_name) + " service call timed out";
    return nullptr;
  }
  return future.get();
}

}  // namespace

Ros2FaultServiceTransport::Ros2FaultServiceTransport(rclcpp::Node * node) : node_(node) {
  // Pick up configurable timeout. GatewayNode declares this parameter up front,
  // but unit tests may construct the transport with a plain rclcpp::Node.
  if (!node_->get_parameter("fault_manager.service_timeout_sec", service_timeout_sec_)) {
    service_timeout_sec_ = 5.0;
  }
  // isfinite first, and the range as a positive test. Do NOT rewrite this as
  // "not positive or too large": every comparison against NaN is false, so that
  // form lets NaN reach wait_for_service, where the duration_cast to
  // nanoseconds is undefined and the result is treated as "wait forever" - one
  // fault RPC then pins an HTTP worker for the life of the process. A
  // non-positive value is the quiet version of the same problem: every fault
  // RPC returns "service not available" with nothing in the log.
  const bool timeout_usable = std::isfinite(service_timeout_sec_) && service_timeout_sec_ > 0.0;
  if (!timeout_usable) {
    RCLCPP_WARN(node_->get_logger(),
                "fault_manager.service_timeout_sec %.2f must be finite and > 0. Using default 5.0.",
                service_timeout_sec_);
    service_timeout_sec_ = 5.0;
  }
  fault_manager_base_path_ = build_fault_manager_base_path(node_);

  // The fault service clients live on a private node driven by a private
  // executor. The host gateway node's MultiThreadedExecutor therefore never
  // processes these clients, so the client's pending-request cleanup cannot
  // race against the calling thread destroying the response shared_ptr - both
  // happen inline on the caller's thread inside spin_until_future_complete().
  client_node_ = std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_fault_clients");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);

  report_fault_client_ =
      client_node_->create_client<ros2_medkit_msgs::srv::ReportFault>(fault_manager_base_path_ + "/report_fault");
  get_fault_client_ =
      client_node_->create_client<ros2_medkit_msgs::srv::GetFault>(fault_manager_base_path_ + "/get_fault");
  list_faults_client_ =
      client_node_->create_client<ros2_medkit_msgs::srv::ListFaults>(fault_manager_base_path_ + "/list_faults");
  clear_fault_client_ =
      client_node_->create_client<ros2_medkit_msgs::srv::ClearFault>(fault_manager_base_path_ + "/clear_fault");
  get_snapshots_client_ =
      client_node_->create_client<ros2_medkit_msgs::srv::GetSnapshots>(fault_manager_base_path_ + "/get_snapshots");
  get_rosbag_client_ =
      client_node_->create_client<ros2_medkit_msgs::srv::GetRosbag>(fault_manager_base_path_ + "/get_rosbag");
  list_rosbags_client_ =
      client_node_->create_client<ros2_medkit_msgs::srv::ListRosbags>(fault_manager_base_path_ + "/list_rosbags");
  declare_planned_stop_client_ = client_node_->create_client<ros2_medkit_msgs::srv::DeclarePlannedStop>(
      fault_manager_base_path_ + "/declare_planned_stop");
  end_planned_stop_client_ = client_node_->create_client<ros2_medkit_msgs::srv::EndPlannedStop>(
      fault_manager_base_path_ + "/end_planned_stop");
  list_planned_stops_client_ = client_node_->create_client<ros2_medkit_msgs::srv::ListPlannedStops>(
      fault_manager_base_path_ + "/list_planned_stops");

  RCLCPP_INFO(node_->get_logger(), "Ros2FaultServiceTransport initialized (base_path=%s, timeout=%.1fs)",
              fault_manager_base_path_.c_str(), service_timeout_sec_);
}

Ros2FaultServiceTransport::~Ros2FaultServiceTransport() {
  // Tear down under the executor mutex so any in-flight RPC has finished. Drop
  // the clients first, then the executor, then the node, so the executor never
  // references a freed node.
  std::lock_guard<std::mutex> lock(executor_mutex_);

  report_fault_client_.reset();
  get_fault_client_.reset();
  list_faults_client_.reset();
  clear_fault_client_.reset();
  get_snapshots_client_.reset();
  get_rosbag_client_.reset();
  list_rosbags_client_.reset();
  declare_planned_stop_client_.reset();
  end_planned_stop_client_.reset();
  list_planned_stops_client_.reset();

  executor_.reset();
  client_node_.reset();
}

bool Ros2FaultServiceTransport::wait_for_services(std::chrono::duration<double> timeout) {
  // One deadline shared by all four waits: per-client timeouts would stack to
  // 4x on an absent fault manager. Clamped to 0 (= non-blocking check) because
  // a negative rclcpp timeout means wait forever.
  const auto clamped = std::chrono::duration<double>(std::max(timeout.count(), 0.0));
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(clamped);
  const auto remaining = [deadline] {
    return std::max(std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()),
                    std::chrono::duration<double>::zero());
  };
  return report_fault_client_->wait_for_service(remaining()) && get_fault_client_->wait_for_service(remaining()) &&
         list_faults_client_->wait_for_service(remaining()) && clear_fault_client_->wait_for_service(remaining());
}

bool Ros2FaultServiceTransport::is_available() const {
  return report_fault_client_->service_is_ready() && get_fault_client_->service_is_ready() &&
         list_faults_client_->service_is_ready() && clear_fault_client_->service_is_ready();
}

FaultResult Ros2FaultServiceTransport::report_fault(const std::string & fault_code, uint8_t severity,
                                                    const std::string & description, const std::string & source_id) {
  FaultResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
  request->fault_code = fault_code;
  request->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
  request->severity = severity;
  request->description = description;
  request->source_id = source_id;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::ReportFault>(
      report_fault_client_, request, executor_, executor_mutex_, std::chrono::duration<double>(service_timeout_sec_),
      "ReportFault", result.error_message);
  if (!response) {
    result.success = false;
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = response->accepted;
  result.data = {{"accepted", response->accepted}};
  if (!response->accepted) {
    result.error_message = "Fault report rejected";
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::list_faults(const std::string & source_id, bool include_prefailed,
                                                   bool include_confirmed, bool include_cleared, bool include_healed,
                                                   bool include_muted, bool include_clusters) {
  FaultResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListFaults::Request>();
  request->filter_by_severity = false;
  request->severity = 0;

  if (include_prefailed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
  }
  if (include_confirmed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  }
  if (include_cleared) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CLEARED);
  }
  if (include_healed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED);
  }

  request->include_muted = include_muted;
  request->include_clusters = include_clusters;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::ListFaults>(
      list_faults_client_, request, executor_, executor_mutex_, std::chrono::duration<double>(service_timeout_sec_),
      "ListFaults", result.error_message);
  if (!response) {
    result.success = false;
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  // Filter by source_id if provided (uses prefix matching)
  json faults_array = json::array();
  for (const auto & fault : response->faults) {
    if (!source_id.empty()) {
      const auto & sources = fault.reporting_sources;
      bool matches = false;
      for (const auto & src : sources) {
        if (src.rfind(source_id, 0) == 0) {
          matches = true;
          break;
        }
      }
      if (!matches) {
        continue;
      }
    }
    faults_array.push_back(conversions::fault_to_json(fault));
  }

  result.success = true;
  result.data = {{"faults", faults_array}, {"count", faults_array.size()}};
  result.data["muted_count"] = response->muted_count;
  result.data["cluster_count"] = response->cluster_count;

  if (include_muted && !response->muted_faults.empty()) {
    json muted_array = json::array();
    for (const auto & muted : response->muted_faults) {
      muted_array.push_back({{"fault_code", muted.fault_code},
                             {"root_cause_code", muted.root_cause_code},
                             {"rule_id", muted.rule_id},
                             {"delay_ms", muted.delay_ms}});
    }
    result.data["muted_faults"] = muted_array;
  }

  if (include_clusters && !response->clusters.empty()) {
    auto to_seconds = [](const builtin_interfaces::msg::Time & t) {
      return t.sec + static_cast<double>(t.nanosec) / 1e9;
    };

    json clusters_array = json::array();
    for (const auto & cluster : response->clusters) {
      clusters_array.push_back({{"cluster_id", cluster.cluster_id},
                                {"rule_id", cluster.rule_id},
                                {"rule_name", cluster.rule_name},
                                {"label", cluster.label},
                                {"representative_code", cluster.representative_code},
                                {"representative_severity", cluster.representative_severity},
                                {"fault_codes", cluster.fault_codes},
                                {"count", cluster.count},
                                {"first_at", to_seconds(cluster.first_at)},
                                {"last_at", to_seconds(cluster.last_at)}});
    }
    result.data["clusters"] = clusters_array;
  }

  return result;
}

FaultWithEnvJsonResult Ros2FaultServiceTransport::get_fault_with_env(const std::string & fault_code,
                                                                     const std::string & source_id) {
  FaultWithEnvJsonResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetFault::Request>();
  request->fault_code = fault_code;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::GetFault>(
      get_fault_client_, request, executor_, executor_mutex_, std::chrono::duration<double>(service_timeout_sec_),
      "GetFault", result.error_message);
  if (!response) {
    result.success = false;
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  if (!response->success) {
    result.success = false;
    result.error_message = response->error_message;
    return result;
  }

  // Verify source_id if provided (prefix match against any reporting source).
  if (!source_id.empty()) {
    bool matches = false;
    for (const auto & src : response->fault.reporting_sources) {
      if (src.rfind(source_id, 0) == 0) {
        matches = true;
        break;
      }
    }
    if (!matches) {
      result.success = false;
      result.error_message = "Fault not found for source: " + source_id;
      return result;
    }
  }

  result.success = true;
  result.data = {{"fault", conversions::fault_to_json(response->fault)},
                 {"environment_data", conversions::environment_data_to_json(response->environment_data)}};
  return result;
}

FaultResult Ros2FaultServiceTransport::get_fault(const std::string & fault_code, const std::string & source_id) {
  // Use get_fault_with_env and pull only the fault portion of the body.
  auto env_result = get_fault_with_env(fault_code, source_id);

  FaultResult result;
  result.success = env_result.success;
  result.error_message = env_result.error_message;
  result.failure = env_result.failure;

  if (env_result.success) {
    result.data = env_result.data["fault"];
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::clear_fault(const std::string & fault_code, bool skip_correlation_auto_clear) {
  FaultResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::ClearFault::Request>();
  request->fault_code = fault_code;
  request->skip_correlation_auto_clear = skip_correlation_auto_clear;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::ClearFault>(
      clear_fault_client_, request, executor_, executor_mutex_, std::chrono::duration<double>(service_timeout_sec_),
      "ClearFault", result.error_message);
  if (!response) {
    result.success = false;
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = response->success;
  result.data = {{"success", response->success}, {"message", response->message}};
  if (!response->success) {
    result.error_message = response->message;
  }

  if (!response->auto_cleared_codes.empty()) {
    result.data["auto_cleared_codes"] = response->auto_cleared_codes;
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::get_snapshots(const std::string & fault_code, const std::string & topic) {
  FaultResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetSnapshots::Request>();
  request->fault_code = fault_code;
  request->topic = topic;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::GetSnapshots>(
      get_snapshots_client_, request, executor_, executor_mutex_, std::chrono::duration<double>(service_timeout_sec_),
      "GetSnapshots", result.error_message);
  if (!response) {
    result.success = false;
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = response->success;

  if (response->success) {
    try {
      result.data = json::parse(response->data);
    } catch (const json::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
                  "Ros2FaultServiceTransport::get_snapshots: failed to parse JSON response for fault_code='%s' "
                  "topic='%s': %s; falling back to raw_data wrapper",
                  fault_code.c_str(), topic.c_str(), e.what());
      result.data = {{"raw_data", response->data}};
    }
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::get_rosbag(const std::string & fault_code) {
  FaultResult result;

  // The parameter is the bulk-data id, which is a recording id. It is sent in BOTH
  // fields: the fault manager prefers recording_id and falls back to fault_code,
  // which is what keeps a pre-#620 URL (and every existing .test.py that calls this
  // service with a fault code) working unchanged.
  auto request = std::make_shared<ros2_medkit_msgs::srv::GetRosbag::Request>();
  request->recording_id = fault_code;
  request->fault_code = fault_code;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::GetRosbag>(
      get_rosbag_client_, request, executor_, executor_mutex_, std::chrono::duration<double>(service_timeout_sec_),
      "GetRosbag", result.error_message);
  if (!response) {
    result.success = false;
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = response->success;

  if (response->success) {
    result.data = {{"file_path", response->file_path},       {"recording_id", response->recording_id},
                   {"fault_codes", response->fault_codes},   {"format", response->format},
                   {"duration_sec", response->duration_sec}, {"size_bytes", response->size_bytes}};
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::list_rosbags(const std::string & entity_fqn) {
  FaultResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListRosbags::Request>();
  request->entity_fqn = entity_fqn;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::ListRosbags>(
      list_rosbags_client_, request, executor_, executor_mutex_, std::chrono::duration<double>(service_timeout_sec_),
      "ListRosbags", result.error_message);
  if (!response) {
    result.success = false;
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = response->success;

  if (response->success) {
    // The response uses parallel arrays. Trust nothing about the remote
    // service: a server bug or schema drift could ship arrays of different
    // lengths, and indexing past end-of-vector is UB. Any array that does not
    // match fault_codes is reported as a mismatch rather than silently truncated.
    const size_t n = response->fault_codes.size();
    if (response->recording_ids.size() != n || response->file_paths.size() != n || response->formats.size() != n ||
        response->durations_sec.size() != n || response->sizes_bytes.size() != n ||
        response->created_at_ns.size() != n) {
      result.success = false;
      result.error_message = "ListRosbags response has mismatched array sizes (fault_codes=" + std::to_string(n) +
                             ", recording_ids=" + std::to_string(response->recording_ids.size()) +
                             ", file_paths=" + std::to_string(response->file_paths.size()) +
                             ", formats=" + std::to_string(response->formats.size()) +
                             ", durations_sec=" + std::to_string(response->durations_sec.size()) +
                             ", sizes_bytes=" + std::to_string(response->sizes_bytes.size()) +
                             ", created_at_ns=" + std::to_string(response->created_at_ns.size()) + ")";
      return result;
    }
    json rosbags = json::array();
    for (size_t i = 0; i < n; ++i) {
      rosbags.push_back({{"fault_code", response->fault_codes[i]},
                         {"recording_id", response->recording_ids[i]},
                         {"file_path", response->file_paths[i]},
                         {"format", response->formats[i]},
                         {"duration_sec", response->durations_sec[i]},
                         {"size_bytes", response->sizes_bytes[i]},
                         {"created_at_ns", response->created_at_ns[i]}});
    }
    result.data = {{"rosbags", rosbags}};
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

// ---------------------------------------------------------------------------
// Planned-stop windows
// ---------------------------------------------------------------------------

namespace {

// time_msg_to_ns / ns_to_time_msg come from fault_msg_conversions.hpp: the
// conversion has to be floor division for an instant before the epoch, and one
// copy of that rule is easier to keep right than two.
using conversions::ns_to_time_msg;
using conversions::time_msg_to_ns;

faults::PlannedStopWindow window_from_msg(const ros2_medkit_msgs::msg::PlannedStop & msg) {
  faults::PlannedStopWindow w;
  w.id = msg.id;
  w.from_ns = time_msg_to_ns(msg.starts_at);
  w.to_ns = time_msg_to_ns(msg.ends_at);
  w.reason = msg.reason;
  w.declared_by = msg.declared_by;
  w.declared_at_ns = time_msg_to_ns(msg.declared_at);
  w.ended_early = msg.ended_early;
  w.cancelled = msg.cancelled;
  return w;
}

}  // namespace

PlannedStopResult Ros2FaultServiceTransport::declare_planned_stop(const faults::PlannedStopWindow & window) {
  PlannedStopResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::DeclarePlannedStop::Request>();
  // Always an explicit instant: the service refuses zero, and the caller has
  // already filled in "now" when the body omitted a start.
  request->starts_at = ns_to_time_msg(window.from_ns);
  request->ends_at = ns_to_time_msg(window.to_ns);
  request->reason = window.reason;
  request->declared_by = window.declared_by;

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::DeclarePlannedStop>(
      declare_planned_stop_client_, request, executor_, executor_mutex_,
      std::chrono::duration<double>(service_timeout_sec_), "DeclarePlannedStop", result.error_message);
  if (!response) {
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = response->success;
  if (!response->success) {
    result.error_message = response->message;
    using Response = ros2_medkit_msgs::srv::DeclarePlannedStop::Response;
    switch (response->outcome) {
      case Response::OUTCOME_DUPLICATE_ID:
        result.refusal = PlannedStopRefusal::DuplicateId;
        break;
      case Response::OUTCOME_NOT_RETAINED:
        result.refusal = PlannedStopRefusal::NotRetained;
        break;
      default:
        result.refusal = PlannedStopRefusal::InvalidRequest;
        break;
    }
    return result;
  }
  result.stops.push_back(window_from_msg(response->stop));
  return result;
}

PlannedStopResult Ros2FaultServiceTransport::end_planned_stop(const std::string & id) {
  PlannedStopResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::EndPlannedStop::Request>();
  request->id = id;
  // A zero instant asks the fault manager for its own wall clock. Sending the
  // gateway's would put the two clocks' difference into the record of when the
  // stop finished, and the fault timestamps it will be compared against come
  // from the fault manager's clock.
  request->at = builtin_interfaces::msg::Time();

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::EndPlannedStop>(
      end_planned_stop_client_, request, executor_, executor_mutex_,
      std::chrono::duration<double>(service_timeout_sec_), "EndPlannedStop", result.error_message);
  if (!response) {
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = response->success;
  if (!response->success) {
    result.error_message = response->message;
    using Response = ros2_medkit_msgs::srv::EndPlannedStop::Response;
    result.refusal = response->outcome == Response::OUTCOME_ALREADY_ENDED ? PlannedStopRefusal::AlreadyEnded
                                                                          : PlannedStopRefusal::NotFound;
    return result;
  }
  result.stops.push_back(window_from_msg(response->stop));
  return result;
}

PlannedStopResult Ros2FaultServiceTransport::list_planned_stops(bool active_only) {
  PlannedStopResult result;

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListPlannedStops::Request>();
  request->active_only = active_only;
  request->now = builtin_interfaces::msg::Time();

  auto response = invoke_fault_service<ros2_medkit_msgs::srv::ListPlannedStops>(
      list_planned_stops_client_, request, executor_, executor_mutex_,
      std::chrono::duration<double>(service_timeout_sec_), "ListPlannedStops", result.error_message);
  if (!response) {
    result.failure = FaultFailure::Unavailable;
    return result;
  }

  result.success = true;
  result.stops.reserve(response->stops.size());
  for (const auto & stop : response->stops) {
    result.stops.push_back(window_from_msg(stop));
  }
  return result;
}

}  // namespace ros2_medkit_gateway::ros2
