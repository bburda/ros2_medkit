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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_fault_service_transport.hpp"
#include "ros2_medkit_gateway/trigger_fault_subscriber.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/get_rosbag.hpp"
#include "ros2_medkit_msgs/srv/get_snapshots.hpp"

using namespace std::chrono_literals;
using ros2_medkit_gateway::FaultFailure;
using ros2_medkit_gateway::FaultManager;
using ros2_medkit_gateway::ResourceChange;
using ros2_medkit_gateway::ResourceChangeNotifier;
using ros2_medkit_gateway::TriggerFaultSubscriber;
using ros2_medkit_msgs::srv::GetFault;
using ros2_medkit_msgs::srv::GetRosbag;
using ros2_medkit_msgs::srv::GetSnapshots;

class FaultManagerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    // ROS_DOMAIN_ID is injected per test by CMake (medkit_set_test_domain in
    // CMakeLists.txt), which keeps this suite's mock /fault_manager services
    // isolated from the real fault_manager_node that ros2_medkit_fault_manager
    // launches in its own tests. Do not override the injected domain here.
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Use unique node names to avoid DDS participant name collisions between tests.
    // On Humble (CycloneDDS), reusing the same node name across sequential tests
    // causes stale discovery state that can corrupt service responses.
    std::string node_name = "test_fault_manager_node_" + std::to_string(test_counter_++);
    node_ = std::make_shared<rclcpp::Node>(
        node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides({
                       {"fault_manager.service_timeout_sec", 3.0},
                   }));

    // Create executor for spinning
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override {
    if (spin_thread_.joinable()) {
      executor_->cancel();
      spin_thread_.join();
    }
    executor_.reset();
    node_.reset();
  }

  void start_spinning() {
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });
  }

  void stop_spinning() {
    if (spin_thread_.joinable()) {
      executor_->cancel();
      spin_thread_.join();
    }
  }

  bool wait_for_subscription_count(const std::string & topic_name, size_t expected_count,
                                   std::chrono::milliseconds timeout = 1s) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (node_->count_subscribers(topic_name) == expected_count) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return node_->count_subscribers(topic_name) == expected_count;
  }

  static inline int test_counter_ = 0;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST(FaultManagerPathsTest, BuildPathsFromNamespaceString) {
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_base_path(""), "/fault_manager");
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_base_path("/"), "/fault_manager");
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_base_path("///"), "/fault_manager");
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_base_path("robot1"), "/robot1/fault_manager");
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_base_path("/robot1/"), "/robot1/fault_manager");

  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_events_topic(""), "/fault_manager/events");
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_events_topic("///"), "/fault_manager/events");
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_events_topic("robot1"), "/robot1/fault_manager/events");
  EXPECT_EQ(ros2_medkit_gateway::build_fault_manager_events_topic("/robot1/"), "/robot1/fault_manager/events");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsServiceNotAvailable) {
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  // Don't create a service, so it will timeout
  auto result = fault_manager.get_snapshots("TEST_FAULT");

  EXPECT_FALSE(result.success);
  // On Humble, wait_for_service may report ready before DDS confirms absence,
  // so the call proceeds to async_send_request which then times out.
  EXPECT_TRUE(result.error_message == "GetSnapshots service not available" ||
              result.error_message == "GetSnapshots service call timed out");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsSuccessWithValidJson) {
  // Create mock service
  auto service = node_->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots", [](const std::shared_ptr<GetSnapshots::Request> & request,
                                         const std::shared_ptr<GetSnapshots::Response> & response) {
        response->success = true;
        nlohmann::json snapshot_data;
        snapshot_data["fault_code"] = request->fault_code;
        snapshot_data["captured_at"] = 1735830000.123;
        snapshot_data["topics"] = nlohmann::json::object();
        snapshot_data["topics"]["/joint_states"] = {{"message_type", "sensor_msgs/msg/JointState"},
                                                    {"data", {{"position", {1.0, 2.0}}}}};
        response->data = snapshot_data.dump();
      });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_snapshots("MOTOR_OVERHEAT");
  stop_spinning();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_TRUE(result.data.contains("fault_code"));
  EXPECT_EQ(result.data["fault_code"], "MOTOR_OVERHEAT");
  EXPECT_TRUE(result.data.contains("topics"));
  EXPECT_TRUE(result.data["topics"].contains("/joint_states"));
}

// Regression guard: Ros2FaultServiceTransport must drive its service clients
// on its own private executor. The transport's host node (node_) is left
// unspun here - the mock service runs on a separate node with its own spin
// thread - so the RPC can only complete if the transport spins its private
// client node itself.
TEST_F(FaultManagerTest, GetSnapshotsDrivesPrivateClientWithoutSpinningHostNode) {
  auto service_node = std::make_shared<rclcpp::Node>("mock_fault_service_node_" + std::to_string(test_counter_++));
  auto service = service_node->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots", [](const std::shared_ptr<GetSnapshots::Request> & request,
                                         const std::shared_ptr<GetSnapshots::Response> & response) {
        response->success = true;
        nlohmann::json snapshot_data;
        snapshot_data["fault_code"] = request->fault_code;
        response->data = snapshot_data.dump();
      });

  rclcpp::executors::SingleThreadedExecutor service_executor;
  service_executor.add_node(service_node);
  std::thread service_thread([&service_executor]() {
    service_executor.spin();
  });

  // node_ is deliberately never spun by this test.
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));
  auto result = fault_manager.get_snapshots("SELF_DRIVEN_FAULT");

  service_executor.cancel();
  service_thread.join();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_EQ(result.data["fault_code"], "SELF_DRIVEN_FAULT");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsUsesConfiguredFaultManagerNamespace) {
  node_ = std::make_shared<rclcpp::Node>(
      "test_fault_manager_node_namespaced_snapshots",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides({
          {"fault_manager.namespace", "robot1"},
      }));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  auto service = node_->create_service<GetSnapshots>(
      "/robot1/fault_manager/get_snapshots", [](const std::shared_ptr<GetSnapshots::Request> & request,
                                                const std::shared_ptr<GetSnapshots::Response> & response) {
        response->success = true;
        nlohmann::json snapshot_data;
        snapshot_data["fault_code"] = request->fault_code;
        snapshot_data["service_path"] = "/robot1/fault_manager/get_snapshots";
        response->data = snapshot_data.dump();
      });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_snapshots("NAMESPACED_FAULT");
  stop_spinning();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.data["fault_code"], "NAMESPACED_FAULT");
  EXPECT_EQ(result.data["service_path"], "/robot1/fault_manager/get_snapshots");
}

TEST_F(FaultManagerTest, InvalidFaultManagerNamespaceFallsBackToRootServicePath) {
  node_ = std::make_shared<rclcpp::Node>(
      "test_fault_manager_node_invalid_namespace",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides({
          {"fault_manager.namespace", "robot 1"},
      }));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  auto service = node_->create_service<GetSnapshots>("/fault_manager/get_snapshots",
                                                     [](const std::shared_ptr<GetSnapshots::Request> & request,
                                                        const std::shared_ptr<GetSnapshots::Response> & response) {
                                                       response->success = true;
                                                       nlohmann::json snapshot_data;
                                                       snapshot_data["fault_code"] = request->fault_code;
                                                       snapshot_data["service_path"] = "/fault_manager/get_snapshots";
                                                       response->data = snapshot_data.dump();
                                                     });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_snapshots("INVALID_NAMESPACE_FAULT");
  stop_spinning();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.data["service_path"], "/fault_manager/get_snapshots");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsSuccessWithTopicFilter) {
  // Create mock service that verifies topic filter is passed
  std::string received_topic;
  auto service = node_->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots", [&received_topic](const std::shared_ptr<GetSnapshots::Request> & request,
                                                        const std::shared_ptr<GetSnapshots::Response> & response) {
        received_topic = request->topic;
        response->success = true;
        response->data = "{}";
      });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  fault_manager.get_snapshots("TEST_FAULT", "/specific_topic");
  stop_spinning();

  EXPECT_EQ(received_topic, "/specific_topic");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsErrorResponse) {
  auto service = node_->create_service<GetSnapshots>("/fault_manager/get_snapshots",
                                                     [](const std::shared_ptr<GetSnapshots::Request> & /*request*/,
                                                        const std::shared_ptr<GetSnapshots::Response> & response) {
                                                       response->success = false;
                                                       response->error_message = "Fault not found";
                                                     });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_snapshots("NONEXISTENT_FAULT");
  stop_spinning();

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_message, "Fault not found");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsInvalidJsonFallback) {
  auto service = node_->create_service<GetSnapshots>("/fault_manager/get_snapshots",
                                                     [](const std::shared_ptr<GetSnapshots::Request> & /*request*/,
                                                        const std::shared_ptr<GetSnapshots::Response> & response) {
                                                       response->success = true;
                                                       response->data = "not valid json {{{";
                                                     });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_snapshots("TEST_FAULT");
  stop_spinning();

  EXPECT_TRUE(result.success);
  // When JSON parsing fails, raw_data should be returned
  EXPECT_TRUE(result.data.contains("raw_data"));
  EXPECT_EQ(result.data["raw_data"], "not valid json {{{");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsEmptyResponse) {
  auto service = node_->create_service<GetSnapshots>("/fault_manager/get_snapshots",
                                                     [](const std::shared_ptr<GetSnapshots::Request> & /*request*/,
                                                        const std::shared_ptr<GetSnapshots::Response> & response) {
                                                       response->success = true;
                                                       response->data = "{}";
                                                     });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_snapshots("TEST_FAULT");
  stop_spinning();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.is_object());
  EXPECT_TRUE(result.data.empty());
}

// GetRosbag service client tests

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetRosbagServiceNotAvailable) {
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  // Don't create a service, so it will timeout
  auto result = fault_manager.get_rosbag("TEST_FAULT");

  EXPECT_FALSE(result.success);
  // On Humble, wait_for_service may report ready before DDS confirms absence,
  // so the call proceeds to async_send_request which then times out.
  EXPECT_TRUE(result.error_message == "GetRosbag service not available" ||
              result.error_message == "GetRosbag service call timed out");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetRosbagSuccess) {
  auto service = node_->create_service<GetRosbag>(
      "/fault_manager/get_rosbag",
      [](const std::shared_ptr<GetRosbag::Request> & request, const std::shared_ptr<GetRosbag::Response> & response) {
        response->success = true;
        response->file_path = "/tmp/test_bag_" + request->fault_code;
        response->format = "sqlite3";
        response->duration_sec = 5.5;
        response->size_bytes = 12345;
      });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_rosbag("TEST_ROSBAG_FAULT");
  stop_spinning();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.contains("file_path"));
  EXPECT_EQ(result.data["file_path"], "/tmp/test_bag_TEST_ROSBAG_FAULT");
  EXPECT_EQ(result.data["format"], "sqlite3");
  EXPECT_EQ(result.data["duration_sec"], 5.5);
  EXPECT_EQ(result.data["size_bytes"], 12345);
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetRosbagUsesConfiguredFaultManagerNamespace) {
  node_ = std::make_shared<rclcpp::Node>(
      "test_fault_manager_node_namespaced_rosbag",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides({
          {"fault_manager.namespace", "/robot2"},
      }));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  auto service = node_->create_service<GetRosbag>(
      "/robot2/fault_manager/get_rosbag",
      [](const std::shared_ptr<GetRosbag::Request> & request, const std::shared_ptr<GetRosbag::Response> & response) {
        response->success = true;
        response->file_path = "/tmp/" + request->fault_code;
        response->format = "mcap";
        response->duration_sec = 2.0;
        response->size_bytes = 2048;
      });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_rosbag("NAMESPACED_ROSBAG");
  stop_spinning();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.data["file_path"], "/tmp/NAMESPACED_ROSBAG");
  EXPECT_EQ(result.data["format"], "mcap");
}

TEST_F(FaultManagerTest, TriggerFaultSubscriberUsesConfiguredFaultManagerNamespace) {
  node_ = std::make_shared<rclcpp::Node>(
      "test_trigger_fault_subscriber_namespaced",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides({
          {"fault_manager.namespace", "robot3"},
      }));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  ResourceChangeNotifier notifier;
  TriggerFaultSubscriber subscriber(node_.get(), notifier);
  auto publisher =
      node_->create_publisher<ros2_medkit_msgs::msg::FaultEvent>("/robot3/fault_manager/events", rclcpp::QoS(10));

  ASSERT_TRUE(wait_for_subscription_count("/robot3/fault_manager/events", 1u));
  EXPECT_EQ(node_->count_subscribers("/fault_manager/events"), 0u);
}

TEST_F(FaultManagerTest, TriggerFaultSubscriberForwardsNamespacedFaultEvents) {
  node_ = std::make_shared<rclcpp::Node>(
      "test_trigger_fault_subscriber_forwarding",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides({
          {"fault_manager.namespace", "/robot4"},
      }));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  ResourceChangeNotifier notifier;
  TriggerFaultSubscriber subscriber(node_.get(), notifier);
  auto publisher =
      node_->create_publisher<ros2_medkit_msgs::msg::FaultEvent>("/robot4/fault_manager/events", rclcpp::QoS(10));

  std::promise<ResourceChange> change_promise;
  auto change_future = change_promise.get_future();
  auto subscription_id =
      notifier.subscribe({"faults", "camera_node", "/FAULT_1"}, [&change_promise](const ResourceChange & change) {
        change_promise.set_value(change);
      });

  ASSERT_TRUE(wait_for_subscription_count("/robot4/fault_manager/events", 1u));
  start_spinning();

  ros2_medkit_msgs::msg::FaultEvent event;
  event.event_type = "fault_confirmed";
  event.fault.fault_code = "FAULT_1";
  event.fault.reporting_sources.push_back("/pipeline/camera_node");
  event.fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  publisher->publish(event);

  auto status = change_future.wait_for(2s);
  ASSERT_EQ(status, std::future_status::ready);

  auto change = change_future.get();
  EXPECT_EQ(change.collection, "faults");
  EXPECT_EQ(change.entity_id, "camera_node");
  EXPECT_EQ(change.resource_path, "/FAULT_1");
  EXPECT_EQ(change.change_type, ros2_medkit_gateway::ChangeType::CREATED);
  EXPECT_EQ(change.value["fault_code"], "FAULT_1");
  EXPECT_EQ(change.value["event_type"], "fault_confirmed");

  stop_spinning();
  notifier.unsubscribe(subscription_id);
}

TEST_F(FaultManagerTest, TriggerFaultSubscriberResolverCalledAndEntityIdUsed) {
  // Resolver is called with the raw FQN and the returned entity_id is used in the notification.
  node_ = std::make_shared<rclcpp::Node>("test_trigger_resolver_called",
                                         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  ResourceChangeNotifier notifier;
  TriggerFaultSubscriber subscriber(node_.get(), notifier);

  std::string received_fqn;
  subscriber.set_node_to_entity_resolver([&received_fqn](const std::string & fqn) -> std::string {
    received_fqn = fqn;
    return "manifest_entity_id";
  });

  auto publisher = node_->create_publisher<ros2_medkit_msgs::msg::FaultEvent>("/fault_manager/events", rclcpp::QoS(10));

  std::promise<ResourceChange> change_promise;
  auto change_future = change_promise.get_future();
  auto subscription_id = notifier.subscribe({"faults", "manifest_entity_id", "/RESOLVER_FAULT"},
                                            [&change_promise](const ResourceChange & change) {
                                              change_promise.set_value(change);
                                            });

  ASSERT_TRUE(wait_for_subscription_count("/fault_manager/events", 1u));
  start_spinning();

  ros2_medkit_msgs::msg::FaultEvent event;
  event.event_type = "fault_confirmed";
  event.fault.fault_code = "RESOLVER_FAULT";
  event.fault.reporting_sources.push_back("/pipeline/some_node");
  event.fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  publisher->publish(event);

  auto status = change_future.wait_for(2s);
  ASSERT_EQ(status, std::future_status::ready);

  auto change = change_future.get();
  EXPECT_EQ(change.entity_id, "manifest_entity_id");
  EXPECT_EQ(received_fqn, "/pipeline/some_node");

  stop_spinning();
  notifier.unsubscribe(subscription_id);
}

TEST_F(FaultManagerTest, TriggerFaultSubscriberResolverEmptyFallsBackToLastSegment) {
  // When resolver returns empty string, last-segment extraction is used.
  node_ = std::make_shared<rclcpp::Node>("test_trigger_resolver_empty_fallback",
                                         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  ResourceChangeNotifier notifier;
  TriggerFaultSubscriber subscriber(node_.get(), notifier);

  // Resolver always returns empty -> fallback to last segment
  subscriber.set_node_to_entity_resolver([](const std::string & /*fqn*/) -> std::string {
    return "";
  });

  auto publisher = node_->create_publisher<ros2_medkit_msgs::msg::FaultEvent>("/fault_manager/events", rclcpp::QoS(10));

  std::promise<ResourceChange> change_promise;
  auto change_future = change_promise.get_future();
  auto subscription_id = notifier.subscribe({"faults", "fallback_node", "/FALLBACK_FAULT"},
                                            [&change_promise](const ResourceChange & change) {
                                              change_promise.set_value(change);
                                            });

  ASSERT_TRUE(wait_for_subscription_count("/fault_manager/events", 1u));
  start_spinning();

  ros2_medkit_msgs::msg::FaultEvent event;
  event.event_type = "fault_confirmed";
  event.fault.fault_code = "FALLBACK_FAULT";
  event.fault.reporting_sources.push_back("/pipeline/fallback_node");
  event.fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  publisher->publish(event);

  auto status = change_future.wait_for(2s);
  ASSERT_EQ(status, std::future_status::ready);

  auto change = change_future.get();
  EXPECT_EQ(change.entity_id, "fallback_node");

  stop_spinning();
  notifier.unsubscribe(subscription_id);
}

TEST_F(FaultManagerTest, TriggerFaultSubscriberNoResolverUsesLastSegment) {
  // When no resolver is set, last-segment extraction works as before.
  node_ = std::make_shared<rclcpp::Node>("test_trigger_no_resolver",
                                         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  ResourceChangeNotifier notifier;
  TriggerFaultSubscriber subscriber(node_.get(), notifier);
  // No resolver set

  auto publisher = node_->create_publisher<ros2_medkit_msgs::msg::FaultEvent>("/fault_manager/events", rclcpp::QoS(10));

  std::promise<ResourceChange> change_promise;
  auto change_future = change_promise.get_future();
  auto subscription_id = notifier.subscribe({"faults", "camera_proc", "/NO_RESOLVER_FAULT"},
                                            [&change_promise](const ResourceChange & change) {
                                              change_promise.set_value(change);
                                            });

  ASSERT_TRUE(wait_for_subscription_count("/fault_manager/events", 1u));
  start_spinning();

  ros2_medkit_msgs::msg::FaultEvent event;
  event.event_type = "fault_confirmed";
  event.fault.fault_code = "NO_RESOLVER_FAULT";
  event.fault.reporting_sources.push_back("/sensors/camera_proc");
  event.fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  publisher->publish(event);

  auto status = change_future.wait_for(2s);
  ASSERT_EQ(status, std::future_status::ready);

  auto change = change_future.get();
  EXPECT_EQ(change.entity_id, "camera_proc");

  stop_spinning();
  notifier.unsubscribe(subscription_id);
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetRosbagNotFound) {
  auto service = node_->create_service<GetRosbag>("/fault_manager/get_rosbag",
                                                  [](const std::shared_ptr<GetRosbag::Request> & /*request*/,
                                                     const std::shared_ptr<GetRosbag::Response> & response) {
                                                    response->success = false;
                                                    response->error_message = "No rosbag file available for fault";
                                                  });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_rosbag("NONEXISTENT_FAULT");
  stop_spinning();

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_message, "No rosbag file available for fault");
}

// =============================================================================
// FaultFailure - which layer failed, as the transport reports it
// =============================================================================
//
// The handler's 404-vs-503 split reads this flag, so the transport is the one
// place that decides which of the two a caller sees. These pin both readings
// against the live transport, because a mapping that is only tested against a
// hand-set flag would not notice the transport ceasing to set it.

// No fault manager on the graph: no answer was obtained, so this is the one
// case that is genuinely the server's failure and must stay 503-worthy.
// @verifies REQ_INTEROP_013
TEST_F(FaultManagerTest, GetFaultWithNoServiceReportsUnavailable) {
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  // No service is created, so the call cannot obtain a response.
  auto result = fault_manager.get_fault_with_env("TEST_FAULT");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, FaultFailure::Unavailable)
      << "a fault manager that never answered must not be reported as a client error: " << result.error_message;
}

// The fault manager answered and declined. It is reachable and healthy, so
// this is a condition of the request however the refusal is worded - the
// wording used here is its own `fault_code` validation message, which no
// substring match for "not found" would have caught.
// @verifies REQ_INTEROP_013
TEST_F(FaultManagerTest, GetFaultRefusedByTheStoreReportsDeclined) {
  auto service = node_->create_service<GetFault>(
      "/fault_manager/get_fault",
      [](const std::shared_ptr<GetFault::Request> & /*request*/, const std::shared_ptr<GetFault::Response> & response) {
        response->success = false;
        response->error_message = "fault_code contains invalid character '~'";
      });

  start_spinning();
  FaultManager fault_manager(std::make_shared<ros2_medkit_gateway::ros2::Ros2FaultServiceTransport>(node_.get()));

  auto result = fault_manager.get_fault_with_env("F~F");
  stop_spinning();

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, FaultFailure::Declined)
      << "a refusal from a reachable fault manager must not be reported as a server failure";
  EXPECT_EQ(result.error_message, "fault_code contains invalid character '~'");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
