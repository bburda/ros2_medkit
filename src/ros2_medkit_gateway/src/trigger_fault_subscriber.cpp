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

#include "ros2_medkit_gateway/trigger_fault_subscriber.hpp"

#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"

namespace ros2_medkit_gateway {

TriggerFaultSubscriber::TriggerFaultSubscriber(rclcpp::Node * node, ResourceChangeNotifier & notifier)
  : notifier_(notifier), logger_(node->get_logger()) {
  const auto fault_events_topic = build_fault_manager_events_topic(node);
  subscription_ = node->create_subscription<ros2_medkit_msgs::msg::FaultEvent>(
      fault_events_topic, rclcpp::QoS(100).reliable(),
      [this](const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
        on_fault_event(msg);
      });

  RCLCPP_INFO(logger_, "TriggerFaultSubscriber initialized, subscribed to %s", fault_events_topic.c_str());
}

TriggerFaultSubscriber::~TriggerFaultSubscriber() {
  subscription_.reset();
}

void TriggerFaultSubscriber::set_node_to_entity_resolver(NodeToEntityFn resolver) {
  node_to_entity_resolver_ = std::move(resolver);
}

void TriggerFaultSubscriber::on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
  // Convert fault to JSON using the same helper as SSEFaultHandler
  nlohmann::json fault_json = ros2::conversions::fault_to_json(msg->fault);
  fault_json["event_type"] = msg->event_type;

  // Derive entity_id from reporting_sources (first source, if available).
  // In hybrid/manifest mode, the resolver maps ROS FQNs to manifest entity IDs.
  // Falls back to last-segment extraction for runtime_only mode or when the
  // resolver cannot map the FQN.
  std::string entity_id;
  if (!msg->fault.reporting_sources.empty()) {
    auto raw_fqn = msg->fault.reporting_sources[0];

    // Try resolving ROS FQN to manifest entity ID
    if (node_to_entity_resolver_) {
      entity_id = node_to_entity_resolver_(raw_fqn);
    }

    // Fallback: extract last path segment (backward compat for runtime_only)
    if (entity_id.empty()) {
      auto last_slash = raw_fqn.rfind('/');
      entity_id = (last_slash != std::string::npos) ? raw_fqn.substr(last_slash + 1) : raw_fqn;
    }
  }

  // Map event_type to ChangeType. fault_cleared covers the manual clear AND
  // the auto-heal; both remove the fault from the active set, so both are
  // DELETED. Consumers needing the distinction read the payload's "status"
  // (CLEARED vs HEALED) - change_type itself has no reader today.
  ChangeType change_type = ChangeType::UPDATED;
  if (msg->event_type == "fault_confirmed") {
    change_type = ChangeType::CREATED;
  } else if (msg->event_type == "fault_cleared") {
    change_type = ChangeType::DELETED;
  }

  // Skip dispatch if entity_id could not be determined (no reporting sources
  // or FQN was just "/"). Prevents unintended catch-all matching.
  if (entity_id.empty()) {
    return;
  }

  // Prefix fault_code with '/' to match the resource_path format produced by
  // parse_resource_uri() (e.g. "/fault_001"), so fault triggers with a specific
  // resource_path can match.
  auto resource_path = "/" + msg->fault.fault_code;
  notifier_.notify("faults", entity_id, resource_path, fault_json, change_type);

  RCLCPP_DEBUG(logger_, "Forwarded fault event to notifier: %s for %s (entity=%s)", msg->event_type.c_str(),
               msg->fault.fault_code.c_str(), entity_id.c_str());
}

}  // namespace ros2_medkit_gateway
