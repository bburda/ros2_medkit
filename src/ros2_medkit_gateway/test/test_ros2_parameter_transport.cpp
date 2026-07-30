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

// Regression coverage for #531: a discoverable-but-unresponsive node's
// parameter service must not be able to hang Ros2ParameterTransport forever.
// Before the fix, client->list_parameters(...)/get_parameters(...)/etc. were
// called with no timeout, so rclcpp defaulted to waiting forever once
// wait_for_service() had already succeeded (i.e. the service exists in the
// ROS graph but its callback never replies in time). That held spin_mutex_
// and an HTTP worker forever, eventually exhausting the whole gateway thread
// pool.

#include <gtest/gtest.h>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

#include <cstddef>

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/ros2/transports/ros2_parameter_transport.hpp"

using namespace ros2_medkit_gateway;

namespace {

constexpr const char * kUnresponsiveNodeName = "unresponsive_param_node";

// A fully responsive node with one declared integer parameter, used to exercise
// cache eviction against a node that actually answers parameter requests.
std::shared_ptr<rclcpp::Node> make_responsive_param_node(const std::string & node_name, const std::string & param_name,
                                                         int64_t value) {
  auto node = std::make_shared<rclcpp::Node>(node_name);
  node->declare_parameter(param_name, value);
  return node;
}

}  // namespace

class TestRos2ParameterTransportUnresponsiveNode : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // The node under test: parameter services are NOT auto-started by
    // rclcpp::Node (start_parameter_services(false)); instead we hand-roll a
    // ListParameters service whose callback blocks well past the transport's
    // service timeout, simulating a node that IS discoverable (the service
    // exists in the ROS graph, so wait_for_service() succeeds immediately)
    // but never actually answers a request in time.
    //
    // The callback blocks on release_blocking_callback_ rather than a fixed
    // sleep: it never completes on its own during the assertions below
    // (proving the client-side call is bounded purely by
    // Ros2ParameterTransport's own timeout, not by how long the server takes
    // to reply), while still letting TearDown release it immediately so the
    // test suite doesn't pay a long fixed delay every run.
    rclcpp::NodeOptions unresponsive_options;
    unresponsive_options.start_parameter_services(false);
    unresponsive_node_ = std::make_shared<rclcpp::Node>(kUnresponsiveNodeName, unresponsive_options);

    list_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::ListParameters>(
        "~/list_parameters", [this](const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> /*request*/,
                                    std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> /*response*/) {
          while (!release_blocking_callback_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
        });

    // The remaining parameter services just need to exist in the ROS graph
    // so that wait_for_service() (which only probes service discovery, not
    // responsiveness) succeeds regardless of which underlying client it
    // checks. They are not expected to be invoked by the scenarios below
    // since list_parameters is the first remote call in the round trip and
    // it never returns within the bounded timeout.
    get_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::GetParameters>(
        "~/get_parameters", [](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> /*request*/,
                               std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> /*response*/) {});
    set_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::SetParameters>(
        "~/set_parameters", [](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> /*request*/,
                               std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> /*response*/) {});
    describe_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::DescribeParameters>(
        "~/describe_parameters",
        [](const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> /*response*/) {});
    get_parameter_types_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::GetParameterTypes>(
        "~/get_parameter_types", [](const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> /*request*/,
                                    std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> /*response*/) {});
    set_parameters_atomically_service_ =
        unresponsive_node_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
            "~/set_parameters_atomically",
            [](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> /*request*/,
               std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> /*response*/) {});

    // Client-side node handed to the transport (used only for logging / the
    // self-node FQN short-circuit). Ros2ParameterTransport creates and owns
    // its own internal node for the actual AsyncParametersClient IPC, so this
    // node does not need to be spun for parameter round trips to proceed.
    client_node_ = std::make_shared<rclcpp::Node>("test_param_transport_client_node");

    // Small service_timeout_sec so the test proves boundedness quickly.
    // Generous negative_cache_ttl_sec so the negative-cache scenario below
    // is guaranteed to hit the cache rather than re-attempting IPC.
    transport_ = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

    // Spin the unresponsive node so its service callback can be dispatched
    // when a request arrives.
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(unresponsive_node_);
    spin_thread_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_thread_running_) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });

    // Give the service time to register in the ROS graph before the test
    // starts issuing requests (avoids a flaky wait_for_service failure that
    // would mask the scenario under test).
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  void TearDown() override {
    // Release any in-flight blocking service callback first so the spin loop
    // (spin_some, not a blocking spin()) can drain and the spin thread exit.
    release_blocking_callback_ = true;
    // Stop and JOIN the spin thread before touching the executor. The loop is a
    // spin_some(10ms) poll, so it returns on its own once spin_thread_running_
    // is cleared - cancel() is not needed to unblock it (that is only required
    // for a blocking spin()). Calling executor_->cancel() while the spin thread
    // was still inside spin_some() raced the executor's notify-waitable (tsan:
    // ExecutorNotifyWaitable::is_ready() on the spin thread vs
    // trigger_entity_recollect() from cancel() on the main thread). With the
    // thread joined first, nothing else touches the executor as we tear it down.
    spin_thread_running_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(unresponsive_node_);
    transport_.reset();
    executor_.reset();
    list_parameters_service_.reset();
    get_parameters_service_.reset();
    set_parameters_service_.reset();
    describe_parameters_service_.reset();
    get_parameter_types_service_.reset();
    set_parameters_atomically_service_.reset();
    unresponsive_node_.reset();
    client_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> unresponsive_node_;
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_atomically_service_;
  std::atomic<bool> release_blocking_callback_{false};

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<ros2::Ros2ParameterTransport> transport_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_thread_running_{false};
};

// GREEN (post-fix) behavior: a request against a discoverable-but-never-
// responding node's parameter service returns a bounded-time error instead
// of hanging forever.
//
// RED (pre-fix) would have been: this call blocks until the service replies,
// which in this fixture is only released by TearDown - i.e. it would not
// return during the test body at all. That cannot be safely exercised in a
// test that must terminate on its own (the point of #531 is precisely that
// nothing bounds it), so instead this asserts the call completes well within
// a small multiple of service_timeout_sec (0.5s) - which is only possible
// because get_service_timeout() bounds each spin_until_future_complete round
// trip.
//
// list_parameters() also short-circuits after its internal
// cache_default_values() round-trip marks the node unavailable, so the whole
// call is bounded by ~1x service_timeout_sec (one round-trip), not ~2x (#531).
// The tighter 2s assertion documents that single-round-trip cap; the 5s bound
// is the documented outer safety limit.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, ListParametersReturnsBoundedErrorInsteadOfHanging) {
  auto start = std::chrono::steady_clock::now();
  auto result = transport_->list_parameters(kUnresponsiveNodeName);
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.success);
  EXPECT_LT(elapsed, std::chrono::seconds(5))
      << "list_parameters() against an unresponsive-but-discoverable node must be bounded by "
         "service_timeout_sec, not block until the server replies (or forever, pre-fix)";
  EXPECT_LT(elapsed, std::chrono::seconds(2))
      << "list_parameters() must cap the spin_mutex_ hold at ~1x service_timeout_sec (single "
         "round-trip) by short-circuiting once cache_default_values marks the node unavailable";
}

// A second, immediate call against the same node must short-circuit via the
// negative cache (mark_node_unavailable() populated it on the first call's
// round-trip failure) rather than paying the timeout again.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, SecondCallShortCircuitsViaNegativeCache) {
  auto first_result = transport_->list_parameters(kUnresponsiveNodeName);
  ASSERT_FALSE(first_result.success);
  ASSERT_FALSE(transport_->is_node_available(kUnresponsiveNodeName))
      << "first round-trip failure must negative-cache the node (#531)";

  auto start = std::chrono::steady_clock::now();
  auto second_result = transport_->list_parameters(kUnresponsiveNodeName);
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(second_result.success);
  EXPECT_LT(elapsed, std::chrono::milliseconds(100))
      << "second call must hit the negative cache and fail fast, not re-attempt the 0.5s IPC timeout";
}

// A round-trip timeout against an unresponsive node must map to a
// node-unavailable error code (TIMEOUT / SERVICE_UNAVAILABLE -> HTTP 503), NOT
// INTERNAL_ERROR (-> HTTP 500). Before the fix the inner round-trip catch
// rethrew into the operation's outer catch, which set INTERNAL_ERROR and thus
// wrongly surfaced a 500 for what is really "the node isn't answering" (#531).
TEST_F(TestRos2ParameterTransportUnresponsiveNode, GetParameterTimeoutMapsTo503NotInternalError) {
  auto start = std::chrono::steady_clock::now();
  auto result = transport_->get_parameter(kUnresponsiveNodeName, "some_param");
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.success);
  EXPECT_LT(elapsed, std::chrono::seconds(5)) << "get_parameter() must be bounded by service_timeout_sec";
  EXPECT_NE(result.error_code, ParameterErrorCode::INTERNAL_ERROR)
      << "a node round-trip timeout must not be reported as INTERNAL_ERROR (would map to HTTP 500)";
  EXPECT_TRUE(result.error_code == ParameterErrorCode::TIMEOUT ||
              result.error_code == ParameterErrorCode::SERVICE_UNAVAILABLE)
      << "an unresponsive node must map to a 503 (node-unavailable) error code, got "
      << static_cast<int>(result.error_code);
}

// The list_parameters single-round-trip cap must hold even with the negative
// cache DISABLED (negative_cache_ttl_sec == 0, a documented in-range value).
// is_node_unavailable() is always false when the TTL is 0, so gating the cap on
// it would silently let list_parameters do a SECOND round-trip. The cap is
// instead driven by cache_default_values' own return value, which is
// TTL-independent (#531).
//
// Timing-INDEPENDENT assertion (does not flake under TSan/ASan, which triple
// ctest timeouts): the error CLASS is the proxy for which path ran.
// cache_default_values short-circuits with SERVICE_UNAVAILABLE, so seeing
// SERVICE_UNAVAILABLE proves the short-circuit fired (single round-trip). If the
// cap were defeated (second round-trip), list_parameters' own catch would set
// TIMEOUT instead. A generous outer time bound is kept only as a sanity net.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, ListParametersCapHoldsWithNegativeCacheDisabled) {
  constexpr double kServiceTimeoutSec = 1.0;
  constexpr double kNegativeCacheDisabled = 0.0;
  auto ttl0_transport =
      std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), kServiceTimeoutSec, kNegativeCacheDisabled);

  auto start = std::chrono::steady_clock::now();
  auto result = ttl0_transport->list_parameters(kUnresponsiveNodeName);
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ParameterErrorCode::SERVICE_UNAVAILABLE)
      << "with the negative cache disabled the cache_default_values short-circuit must still fire "
         "(SERVICE_UNAVAILABLE); TIMEOUT would mean the cap was defeated and a second round-trip ran";
  EXPECT_LT(elapsed, std::chrono::seconds(5)) << "sanity: still bounded";
}

// The per-node AsyncParametersClient cache is bounded by LRU eviction, so a
// gateway that queries an ever-growing set of distinct node names cannot leak
// one client per node for its whole lifetime (the pre-fix behaviour). A
// transport built with a small cache cap and pointed at many distinct,
// nonexistent node names must keep its client cache at the cap. get_param_client
// runs (and inserts) before the failing wait_for_service, so every distinct
// query would add an entry without the bound. A short service timeout is used
// because every query hits a nonexistent service; the assertion is on the cache
// size, not on timing.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, ClientCacheStaysBoundedAcrossManyNodes) {
  constexpr std::size_t kCacheCap = 4;
  constexpr double kShortTimeoutSec = 0.1;
  constexpr double kNegativeCacheDisabled = 0.0;
  auto bounded_transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), kShortTimeoutSec,
                                                                          kNegativeCacheDisabled, kCacheCap);

  constexpr std::size_t kDistinctNodes = kCacheCap * 3;
  for (std::size_t i = 0; i < kDistinctNodes; ++i) {
    const std::string node_name = "/nonexistent_param_node_" + std::to_string(i);
    auto result = bounded_transport->get_parameter(node_name, "some_param");
    EXPECT_FALSE(result.success) << "a nonexistent node must not resolve a parameter";
  }

  EXPECT_LE(bounded_transport->param_client_cache_size(), kCacheCap)
      << "the per-node client cache must never grow past its capacity";
  EXPECT_EQ(bounded_transport->param_client_cache_size(), kCacheCap)
      << "after " << kDistinctNodes << " distinct nodes the bounded cache should be full at the cap";
}

// Evicting a live node's AsyncParametersClient must be transparent: the next
// query rebuilds the client (on param_node_) and still succeeds. This is the
// contract the client-cache bound must preserve - eviction destroys rcl entities
// on param_node_, so a broken rebuild would otherwise be invisible.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, EvictedLiveClientIsTransparentlyRebuilt) {
  auto responsive_node = make_responsive_param_node("client_evict_responsive_node", "answer", 42);
  executor_->add_node(responsive_node);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const std::string responsive_fqn = responsive_node->get_fully_qualified_name();

  constexpr std::size_t kClientCap = 2;
  constexpr double kNegativeCacheDisabled = 0.0;
  auto transport =
      std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, kNegativeCacheDisabled, kClientCap);

  auto first = transport->get_parameter(responsive_fqn, "answer");
  ASSERT_TRUE(first.success) << first.error_message;
  EXPECT_EQ(first.data["value"].get<int64_t>(), 42);

  // Push the responsive node's client out of the LRU cache with distinct,
  // nonexistent nodes (each inserts a client before its wait_for_service fails).
  for (std::size_t i = 0; i <= kClientCap; ++i) {
    transport->get_parameter("/client_evict_filler_" + std::to_string(i), "x");
  }
  EXPECT_LE(transport->param_client_cache_size(), kClientCap);

  // Re-query the evicted live node: the client is rebuilt and the query succeeds.
  auto again = transport->get_parameter(responsive_fqn, "answer");
  ASSERT_TRUE(again.success) << "evicting a live node's client must be transparent; got: " << again.error_message;
  EXPECT_EQ(again.data["value"].get<int64_t>(), 42);

  executor_->remove_node(responsive_node);
}

// When a live node's cached defaults are evicted, get_default must RE-FETCH them
// rather than report NO_DEFAULTS_CACHED. Uses a defaults cap of 1 and two
// responsive nodes so caching the second evicts the first.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, EvictedDefaultsAreRefetchedNotReportedMissing) {
  auto node_a = make_responsive_param_node("defaults_evict_node_a", "p", 7);
  auto node_b = make_responsive_param_node("defaults_evict_node_b", "p", 8);
  executor_->add_node(node_a);
  executor_->add_node(node_b);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const std::string fqn_a = node_a->get_fully_qualified_name();
  const std::string fqn_b = node_b->get_fully_qualified_name();

  constexpr std::size_t kClientCap = 16;   // large: leave clients cached
  constexpr std::size_t kDefaultsCap = 1;  // tiny: force defaults eviction
  constexpr double kNegativeCacheDisabled = 0.0;
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, kNegativeCacheDisabled,
                                                                  kClientCap, kDefaultsCap);

  auto def_a1 = transport->get_default(fqn_a, "p");
  ASSERT_TRUE(def_a1.success) << def_a1.error_message;
  EXPECT_EQ(def_a1.data.get<int64_t>(), 7);
  EXPECT_EQ(transport->default_values_cache_size(), 1u);

  // Caching B's defaults evicts A's (defaults cap is 1).
  auto def_b = transport->get_default(fqn_b, "p");
  ASSERT_TRUE(def_b.success) << def_b.error_message;
  EXPECT_EQ(transport->default_values_cache_size(), 1u);

  // A's defaults were evicted; get_default must re-fetch, not report them missing.
  auto def_a2 = transport->get_default(fqn_a, "p");
  ASSERT_TRUE(def_a2.success) << "evicted defaults must be re-fetched, not NO_DEFAULTS_CACHED; got error_code "
                              << static_cast<int>(def_a2.error_code);
  EXPECT_EQ(def_a2.data.get<int64_t>(), 7);

  executor_->remove_node(node_a);
  executor_->remove_node(node_b);
}

// After the gateway overwrites a parameter, reset-to-default must restore the
// PRE-write value, not the current (post-write) one - even though the default
// caches only ever see current values. set_parameter records the pre-write value
// (from its existing pre-read) and get_default prefers it.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, GetDefaultReturnsPreWriteValueAfterSet) {
  auto responsive_node = make_responsive_param_node("prewrite_responsive_node", "p", 100);
  executor_->add_node(responsive_node);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const std::string fqn = responsive_node->get_fully_qualified_name();

  constexpr double kNegativeCacheDisabled = 0.0;
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, kNegativeCacheDisabled);

  auto set_res = transport->set_parameter(fqn, "p", nlohmann::json(200));
  ASSERT_TRUE(set_res.success) << set_res.error_message;

  auto def = transport->get_default(fqn, "p");
  ASSERT_TRUE(def.success) << def.error_message;
  EXPECT_EQ(def.data.get<int64_t>(), 100) << "get_default must return the pre-write value the gateway recorded";

  // Sanity: the live value really is 200 now (the write took effect).
  auto got = transport->get_parameter(fqn, "p");
  ASSERT_TRUE(got.success) << got.error_message;
  EXPECT_EQ(got.data["value"].get<int64_t>(), 200);

  executor_->remove_node(responsive_node);
}

// ---------------------------------------------------------------------------
// list_parameters RESPONDS, get_parameters / set_parameters BLOCK (or answer
// empty).
//
// This is the coverage the fully-unresponsive fixture above CANNOT provide.
// The transport uses AsyncParametersClient + spin_until_future_complete, so a
// get/set that times out yields FutureReturnCode::TIMEOUT while a get/set that
// answers with an empty result yields SUCCESS - two distinct outcomes the
// (removed) SyncParametersClient collapsed into the same empty vector. A node
// that answers list (so the transport gets past the first call) but blocks the
// value read/write exercises the TIMEOUT path; the same node answering the get
// with an EMPTY value (get_returns_empty_) exercises the SUCCESS-empty path -
// which must NOT be mistaken for a timeout (#531).
// ---------------------------------------------------------------------------
class TestRos2ParameterTransportListRespondsGetSetStuck : public ::testing::Test {
 protected:
  static constexpr const char * kNodeName = "list_responds_getset_stuck_node";
  static constexpr const char * kParamName = "sample_param";

  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node_ = std::make_shared<rclcpp::Node>(kNodeName, options);

    // list_parameters ALWAYS responds with one name, so the transport always
    // gets past the first round-trip call and reaches get/set.
    list_parameters_service_ = node_->create_service<rcl_interfaces::srv::ListParameters>(
        "~/list_parameters", [](const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> /*request*/,
                                std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response) {
          response->result.names.push_back(kParamName);
        });

    // get_parameters: blocks while block_get_ is set (client times out -> empty).
    // When get_returns_empty_ is set it RESPONDS immediately with an EMPTY values
    // vector (responsive, but no value) - the crucial discriminator between a genuine
    // "not currently gettable" and a timeout. Otherwise returns one INTEGER value per
    // requested name (responsive).
    get_parameters_service_ = node_->create_service<rcl_interfaces::srv::GetParameters>(
        "~/get_parameters", [this](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
                                   std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
          while (block_get_.load() && !release_all_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          if (release_all_.load() || get_returns_empty_.load()) {
            return;  // respond with an empty values vector (fast, responsive)
          }
          for (size_t i = 0; i < request->names.size(); ++i) {
            rcl_interfaces::msg::ParameterValue value;
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            value.integer_value = 42;
            response->values.push_back(value);
          }
        });

    // set_parameters: blocks while block_set_ is set (client times out -> empty),
    // otherwise returns one successful result per param (responsive).
    set_parameters_service_ = node_->create_service<rcl_interfaces::srv::SetParameters>(
        "~/set_parameters", [this](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
                                   std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
          while (block_set_.load() && !release_all_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          if (release_all_.load()) {
            return;
          }
          for (size_t i = 0; i < request->parameters.size(); ++i) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            response->results.push_back(result);
          }
        });

    // describe_parameters / get_parameter_types / set_parameters_atomically only
    // need to exist so wait_for_service() succeeds; they respond instantly empty.
    describe_parameters_service_ = node_->create_service<rcl_interfaces::srv::DescribeParameters>(
        "~/describe_parameters",
        [](const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> /*response*/) {});
    get_parameter_types_service_ = node_->create_service<rcl_interfaces::srv::GetParameterTypes>(
        "~/get_parameter_types", [](const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> /*request*/,
                                    std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> /*response*/) {});
    set_parameters_atomically_service_ = node_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
        "~/set_parameters_atomically",
        [](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> /*response*/) {});

    client_node_ = std::make_shared<rclcpp::Node>("test_listresponds_client_node");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_thread_running_) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  void TearDown() override {
    // Both halves of this order are load-bearing.
    // Join before cancel(): cancel() while the spin thread is inside
    // spin_some() races the executor's notify-waitable
    // (ExecutorNotifyWaitable::is_ready() against trigger_entity_recollect()).
    // Keep cancel(): it releases the executor's node references. Without it the
    // previous test's service outlives its fixture and answers the next test's
    // request, turning an expected timeout into a success.
    release_all_ = true;  // unstick any blocked callback first
    spin_thread_running_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->cancel();
    executor_->remove_node(node_);
    executor_.reset();
    list_parameters_service_.reset();
    get_parameters_service_.reset();
    set_parameters_service_.reset();
    describe_parameters_service_.reset();
    get_parameter_types_service_.reset();
    set_parameters_atomically_service_.reset();
    node_.reset();
    client_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_atomically_service_;

  std::atomic<bool> block_get_{false};
  std::atomic<bool> block_set_{false};
  std::atomic<bool> get_returns_empty_{false};
  std::atomic<bool> release_all_{false};

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_thread_running_{false};
};

// (a) get_parameter: list responds (param exists) but get_parameters never
// answers (blocked). spin_for returns FutureReturnCode::TIMEOUT, which must map
// to a 503-class code (TIMEOUT), NOT INTERNAL_ERROR (500), and negative-cache
// the node (#531).
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, GetParameterEmptyGetAfterFoundNameIsTimeout) {
  block_get_ = true;
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto result = transport->get_parameter(kNodeName, kParamName);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ParameterErrorCode::TIMEOUT)
      << "a blocked get_parameters after a found name is a value-read timeout, got "
      << static_cast<int>(result.error_code);
  EXPECT_NE(result.error_code, ParameterErrorCode::INTERNAL_ERROR);
  EXPECT_FALSE(transport->is_node_available(kNodeName)) << "the get timeout must negative-cache the node";
}

// (b) set_parameter: pre-read responds (fast type hint), but set_parameters
// never answers (blocked). spin_for TIMEOUT must map to TIMEOUT (503) +
// negative-cache, NOT INTERNAL_ERROR (500). This is the #531-fully-live-on-writes
// path: pre-fix a fully-unresponsive node's set returned 500 with no negative cache.
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, SetParameterEmptyResultsIsTimeoutNotInternalError) {
  block_get_ = false;  // pre-read (type hint) responds quickly
  block_set_ = true;   // the actual write stalls
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto result = transport->set_parameter(kNodeName, kParamName, nlohmann::json(7));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ParameterErrorCode::TIMEOUT)
      << "a blocked set_parameters is a write timeout, got " << static_cast<int>(result.error_code);
  EXPECT_NE(result.error_code, ParameterErrorCode::INTERNAL_ERROR);
  EXPECT_FALSE(transport->is_node_available(kNodeName)) << "the set timeout must negative-cache the node";
}

// (c) list_parameters op: defaults already cached (so cache_default_values does
// not short-circuit), then get_parameters stalls on the MAIN round-trip
// (spin_for TIMEOUT). Must be reported as TIMEOUT (503), not a misleading
// success=[] (#531). Asserting TIMEOUT (not SERVICE_UNAVAILABLE) pins the main
// round-trip path specifically, distinct from the cache_default_values gate.
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, ListParametersEmptyGetAfterNamesIsTimeoutNotEmptySuccess) {
  block_get_ = false;  // fully responsive first
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto first = transport->list_parameters(kNodeName);
  ASSERT_TRUE(first.success) << "responsive node must list successfully and cache defaults first";

  block_get_ = true;  // now stall value reads
  auto second = transport->list_parameters(kNodeName);

  EXPECT_FALSE(second.success);
  EXPECT_EQ(second.error_code, ParameterErrorCode::TIMEOUT)
      << "list returned names but the value read timed out: must be TIMEOUT (main round-trip), got "
      << static_cast<int>(second.error_code);
}

// Recovery: a node negative-cached on a get timeout must recover once it becomes
// responsive again and the TTL expires - it must NOT be 503'd forever. Also
// pins Fix 4 (no empty-defaults poison): cache_default_values must NOT have
// committed an empty defaults map on the timeout, or get_default would return
// NOT_FOUND forever; after recovery get_default returns the real value.
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, MarkedNodeRecoversAfterTtlExpires) {
  constexpr double kServiceTimeoutSec = 0.5;
  constexpr double kShortTtlSec = 0.4;
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), kServiceTimeoutSec, kShortTtlSec);

  block_get_ = true;
  auto stalled = transport->list_parameters(kNodeName);
  EXPECT_FALSE(stalled.success);
  EXPECT_FALSE(transport->is_node_available(kNodeName)) << "get timeout must negative-cache the node";

  // Node becomes responsive again; wait past the negative-cache TTL.
  block_get_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(700));

  auto recovered = transport->list_parameters(kNodeName);
  EXPECT_TRUE(recovered.success) << "a marked node must recover after the TTL expires, not stay 503 forever";

  // Fix 4: defaults were NOT poisoned with an empty map on the earlier timeout,
  // so after recovery the real default is retrievable (not NOT_FOUND forever).
  auto def = transport->get_default(kNodeName, kParamName);
  EXPECT_TRUE(def.success) << "get_default must return the real default after recovery, not a poisoned empty cache";
}

// ---------------------------------------------------------------------------
// THE discriminating test (the async refactor's reason to exist).
//
// A node whose get_parameters service RESPONDS but returns an EMPTY values
// vector (responsive, NOT blocked - e.g. the param was undeclared/raced between
// list and get) must be treated as "not found", NOT as a timeout. The node is
// HEALTHY and must stay available (never negative-cached / 503'd).
//
// The pre-async (SyncParametersClient) code CANNOT tell this responsive-empty
// apart from a timeout - both collapse to the same empty vector - so it wrongly
// marks the node and returns TIMEOUT/503. This test FAILS against that code
// (base=RED) and PASSES with the AsyncParametersClient + FutureReturnCode
// refactor, where SUCCESS-with-empty is distinct from TIMEOUT (#531).
// ---------------------------------------------------------------------------
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, GetParameterResponsiveEmptyIsNotFoundNotMarked) {
  block_get_ = false;         // do NOT block: the node answers promptly...
  get_returns_empty_ = true;  // ...but with an empty values vector (responsive)
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto result = transport->get_parameter(kNodeName, kParamName);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ParameterErrorCode::NOT_FOUND)
      << "a responsive node that answers get with an empty value is NOT_FOUND, not a timeout; got "
      << static_cast<int>(result.error_code);
  EXPECT_NE(result.error_code, ParameterErrorCode::TIMEOUT);
  EXPECT_TRUE(transport->is_node_available(kNodeName))
      << "a responsive-but-empty get must NOT negative-cache the healthy node (the pre-async "
         "code wrongly marks it here)";
}

// The list op form of the same discriminator: a responsive node that answers get
// with an empty values vector must yield success=[] with the node still available,
// NOT a 503 / negative-cache. Pre-async this responsive-empty was marked + 503'd.
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, ListParametersResponsiveEmptyGetIsEmptySuccessNotMarked) {
  block_get_ = false;
  get_returns_empty_ = true;
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto result = transport->list_parameters(kNodeName);

  EXPECT_TRUE(result.success) << "responsive node (empty get) must return success, not a 503; error_code="
                              << static_cast<int>(result.error_code);
  EXPECT_TRUE(result.data.is_array());
  EXPECT_TRUE(result.data.empty()) << "no gettable values -> empty parameter list";
  EXPECT_TRUE(transport->is_node_available(kNodeName)) << "a responsive-but-empty get must NOT negative-cache the node";
}

// A genuine SUCCESS-empty (responsive node, empty defaults) is a STABLE, authoritative
// fact and must be MEMOIZED, so:
//   - get_default returns NOT_FOUND (empty map cached), NOT NO_DEFAULTS_CACHED, and
//   - the second call answers from the cache WITHOUT another round trip.
// Only a TIMEOUT must avoid caching (poison-avoidance). This is the polish that restores
// memoization the async refactor had over-broadly dropped (#531).
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, ResponsiveEmptyMemoizesEmptyDefaults) {
  get_returns_empty_ = true;  // responsive node whose get returns an empty value set
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto listed = transport->list_parameters(kNodeName);
  ASSERT_TRUE(listed.success);
  ASSERT_TRUE(transport->is_node_available(kNodeName));

  // The empty defaults map was cached, so get_default is NOT_FOUND (not NO_DEFAULTS_CACHED).
  auto def = transport->get_default(kNodeName, kParamName);
  EXPECT_FALSE(def.success);
  EXPECT_EQ(def.error_code, ParameterErrorCode::NOT_FOUND)
      << "a genuine SUCCESS-empty must memoize an empty defaults map (NOT_FOUND), not leave it "
         "uncached (NO_DEFAULTS_CACHED); got "
      << static_cast<int>(def.error_code);

  // Block the node: a cached map means get_default must NOT re-round-trip (which would now
  // time out) - it answers from the cache, so still NOT_FOUND and the node stays available.
  block_get_ = true;
  auto def_cached = transport->get_default(kNodeName, kParamName);
  EXPECT_FALSE(def_cached.success);
  EXPECT_EQ(def_cached.error_code, ParameterErrorCode::NOT_FOUND)
      << "get_default must answer from the cached empty map, not re-round-trip; got "
      << static_cast<int>(def_cached.error_code);
  EXPECT_TRUE(transport->is_node_available(kNodeName)) << "no re-round-trip means the node is never marked";
}
