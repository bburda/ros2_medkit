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

/**
 * @file unreadable_lifecycle_node.cpp
 * @brief Demo node that LOOKS like a managed lifecycle node but never answers GetState
 *
 * Proof fixture for GRAPH_NODE_UNREADABLE (ros2_medkit_graph_watchdog's
 * lifecycle_expectation detector): a required node whose lifecycle state can never be
 * read must be reported UNREADABLE, not silently folded into GRAPH_NODE_INACTIVE. No
 * existing demo node can produce that state - managed_lifecycle_node.cpp is a REAL
 * rclcpp_lifecycle::LifecycleNode, and a real one always answers GetState immediately.
 *
 * `find_lifecycle_get_state_path()`
 * (ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp) does not check that a
 * node IS an rclcpp_lifecycle::LifecycleNode; it matches purely on discovered SERVICE
 * TYPE: an App counts as "managed" once its services include one of type
 * `lifecycle_msgs/srv/GetState` and one of type `lifecycle_msgs/srv/ChangeState`
 * (see test_lifecycle_status_helpers.cpp's FindPathRequiresBothServices). So a plain
 * rclcpp::Node that advertises both service types under the conventional `~/get_state`
 * / `~/change_state` names is indistinguishable from a real LifecycleNode to the
 * gateway's discovery layer, while its GetState handler simply never responds.
 *
 * GetState is registered with rclcpp's DEFERRED-RESPONSE service callback signature -
 * `void(std::shared_ptr<rmw_request_id_t>, std::shared_ptr<Request>)`, no response
 * parameter - rather than the ordinary `(request, response)` or
 * `(header, request, response)` forms. Both of those always allocate a response and
 * `Service<T>::handle_request()` sends it unconditionally once the callback returns
 * (see rclcpp/any_service_callback.hpp's `dispatch()` and rclcpp/service.hpp's
 * `handle_request()`), so neither form could ever leave a request unanswered. The
 * deferred form's `dispatch()` returns nullptr instead, so `handle_request()` sends
 * nothing - the request stays open until something calls `Service<T>::send_response()`
 * on it directly, which this fixture does only from `answer_all_pending()` below.
 * Every request received while `answering_` is false is stored (never answered) in
 * `pending_`, deliberately LEAKED for the lifetime of this process: it is a demo-only
 * fixture that lives for one CTest run, so an unbounded queue of never-answered
 * `rmw_request_id_t`/Request pairs is memory this process's exit reclaims wholesale,
 * not a leak in a long-running system. In practice the queue never grows past a
 * handful of entries anyway - LifecycleWatcher (lifecycle_watcher.cpp) seeds a newly
 * discovered node once and re-seeds a non-active one at most `kReseedAttempts` (2)
 * further times, then stops calling GetState on it at all.
 *
 * ChangeState answers normally (SUCCESS always): only GetState needs to hang, and this
 * fixture is never driven through a lifecycle transition by anything that reads its
 * result.
 *
 * `start_answering` (bool parameter, default false) is how the e2e proves the CLEAR
 * half of the story: setting it true - a real `ros2 param set` /
 * `rcl_interfaces/srv/SetParameters` call against this node's own auto-started
 * parameter service - answers every currently-held GetState request with "active", and
 * every future one immediately. A parameter rather than a wall timer, so the e2e
 * controls exactly WHEN the answer arrives (after it has independently confirmed
 * GRAPH_NODE_UNREADABLE actually raised) instead of racing a fixed delay against the
 * 60-tick hold's own timing.
 *
 * Flipping `start_answering` ALSO publishes one `~/transition_event` - not optional
 * decoration, load-bearing. LifecycleWatcher (lifecycle_watcher.cpp) spends its GetState
 * re-seed budget (the initial seed plus `kReseedAttempts`, 2 more) within the first few
 * ticks after discovery, all while this fixture is still deliberately silent, and never
 * calls GetState on this node again afterwards - a sustained "" is treated as the
 * entry's terminal state. Past that point the ONLY channel that can still move the
 * cached label is `~/transition_event`, exactly as it is for a real rclcpp_lifecycle
 * node's ACTIVATE. Answering later GetState calls immediately (see `send_active` below)
 * is therefore necessary but not sufficient for the watcher to ever notice: nothing
 * calls GetState again to collect that answer unless this event arrives first.
 */

#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

class UnreadableLifecycleNode : public rclcpp::Node {
 public:
  UnreadableLifecycleNode() : Node("unreadable_lifecycle") {
    answering_ = this->declare_parameter<bool>("start_answering", false);

    // Reliable + volatile (the QoS default LifecycleWatcher's own subscription
    // requires - see its "stay volatile" comment), matching how rclcpp_lifecycle
    // itself publishes ~/transition_event.
    transition_event_pub_ = this->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
        "~/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    get_state_service_ = this->create_service<lifecycle_msgs::srv::GetState>(
        "~/get_state", [this](const std::shared_ptr<rmw_request_id_t> & header,
                              const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> & /*request*/) {
          // No lock needed: this node spins on a single-threaded executor (see main()
          // below), so this callback and the parameter callback that flips
          // `answering_` below can never run concurrently with each other.
          if (answering_) {
            send_active(header);
            return;
          }
          pending_.push_back(header);  // deliberately leaked for this process's lifetime - see the file doc
        });

    change_state_service_ = this->create_service<lifecycle_msgs::srv::ChangeState>(
        "~/change_state", [](const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> & /*request*/,
                             const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> & response) {
          // Not exercised by anything: only find_lifecycle_get_state_path()'s type
          // check needs this service to exist at all.
          response->success = true;
        });

    param_callback_handle_ =
        this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto & param : params) {
            if (param.get_name() == "start_answering" && param.as_bool()) {
              answer_all_pending();
            }
          }
          return result;
        });

    RCLCPP_INFO(get_logger(),
                "unreadable_lifecycle started: advertises get_state/change_state like a managed "
                "lifecycle node, but get_state never answers until start_answering:=true");
  }

  ~UnreadableLifecycleNode() override {
    get_state_service_.reset();
    change_state_service_.reset();
    param_callback_handle_.reset();
    transition_event_pub_.reset();
  }
  UnreadableLifecycleNode(const UnreadableLifecycleNode &) = delete;
  UnreadableLifecycleNode & operator=(const UnreadableLifecycleNode &) = delete;
  UnreadableLifecycleNode(UnreadableLifecycleNode &&) = delete;
  UnreadableLifecycleNode & operator=(UnreadableLifecycleNode &&) = delete;

 private:
  void send_active(const std::shared_ptr<rmw_request_id_t> & header) {
    lifecycle_msgs::srv::GetState::Response response;
    response.current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    response.current_state.label = "active";
    get_state_service_->send_response(*header, response);
  }

  void answer_all_pending() {
    if (answering_) {
      return;  // already flipped - a second start_answering:=true is a no-op, not a re-send
    }
    answering_ = true;
    for (const auto & header : pending_) {
      send_active(header);
    }
    pending_.clear();

    // The event LifecycleWatcher actually needs - see the file doc's "load-bearing"
    // note. start_state is filled in for a realistic message shape; only goal_state
    // (read as the new cached label) and start_state (checked only for the
    // "errorprocessing" edge, which this is not) are ever consulted by the watcher.
    lifecycle_msgs::msg::TransitionEvent event;
    event.start_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    event.start_state.label = "unconfigured";
    event.goal_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    event.goal_state.label = "active";
    transition_event_pub_->publish(event);
  }

  bool answering_ = false;
  std::vector<std::shared_ptr<rmw_request_id_t>> pending_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr get_state_service_;
  rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_service_;
  rclcpp::Publisher<lifecycle_msgs::msg::TransitionEvent>::SharedPtr transition_event_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv) {
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, [] {
    return std::make_shared<UnreadableLifecycleNode>();
  });
}
