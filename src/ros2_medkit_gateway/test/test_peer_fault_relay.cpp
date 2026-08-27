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

/// Fast regression checks on PeerFaultRelay's lifecycle arithmetic. Whether the
/// relay actually carries a peer's faults is settled end to end in
/// test_aggregator_fault_stream.test.py, with two gateways and real HTTP;
/// nothing here can show that, and nothing here is offered as showing it.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/peer_fault_relay.hpp"

namespace ros2_medkit_gateway {
namespace {

/// A port nothing listens on, so a connect attempt is refused at once and the
/// reader thread spends its life in the interruptible backoff sleep.
constexpr const char * kDeadUrl = "http://127.0.0.1:1";

PeerFaultRelay::TargetSupplier none() {
  return []() {
    return std::vector<RelayTarget>{};
  };
}

PeerFaultRelay::TargetSupplier one(const std::string & name) {
  return [name]() {
    return std::vector<RelayTarget>{RelayTarget{name, kDeadUrl, ""}};
  };
}

PeerFaultRelay::EventCallback ignore() {
  return [](const StreamEvent &) {};
}

TEST(PeerFaultRelay, opens_nothing_without_a_client) {
  PeerFaultRelay relay(one("peer_a"), "/api/v1/faults/stream", ignore());

  relay.reconcile();

  // The whole point of the refcount: a gateway nobody is watching must not be
  // holding an SSE slot on any peer.
  EXPECT_EQ(relay.open_streams(), 0U);
}

TEST(PeerFaultRelay, opens_one_stream_per_listed_peer_while_a_client_is_attached) {
  PeerFaultRelay relay(one("peer_a"), "/api/v1/faults/stream", ignore());

  relay.acquire();
  EXPECT_EQ(relay.open_streams(), 1U);

  relay.release();
  EXPECT_EQ(relay.open_streams(), 0U);
}

TEST(PeerFaultRelay, the_last_client_closes_the_streams_not_the_first) {
  PeerFaultRelay relay(one("peer_a"), "/api/v1/faults/stream", ignore());

  relay.acquire();
  relay.acquire();
  relay.release();
  // One client still attached. Closing here would blind it.
  EXPECT_EQ(relay.open_streams(), 1U);

  relay.release();
  EXPECT_EQ(relay.open_streams(), 0U);
}

TEST(PeerFaultRelay, a_release_without_an_acquire_does_not_wrap_the_count) {
  PeerFaultRelay relay(one("peer_a"), "/api/v1/faults/stream", ignore());

  relay.release();
  relay.acquire();

  // An unsigned counter decremented past zero would leave this at SIZE_MAX and
  // the streams open for the life of the process.
  EXPECT_EQ(relay.open_streams(), 1U);
  relay.release();
  EXPECT_EQ(relay.open_streams(), 0U);
}

TEST(PeerFaultRelay, a_peer_that_goes_away_has_its_stream_closed) {
  std::atomic<bool> peer_listed{true};
  PeerFaultRelay relay(
      [&peer_listed]() {
        std::vector<RelayTarget> targets;
        if (peer_listed.load()) {
          targets.push_back(RelayTarget{"peer_a", kDeadUrl, ""});
        }
        return targets;
      },
      "/api/v1/faults/stream", ignore());

  relay.acquire();
  ASSERT_EQ(relay.open_streams(), 1U);

  peer_listed.store(false);
  // reconcile() is rate limited, so a second call inside the interval is a
  // no-op by design. Waiting past it is what this asserts against.
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  relay.reconcile();
  EXPECT_EQ(relay.open_streams(), 0U);

  relay.release();
}

TEST(PeerFaultRelay, shutdown_is_idempotent_and_refuses_later_clients) {
  PeerFaultRelay relay(one("peer_a"), "/api/v1/faults/stream", ignore());

  relay.acquire();
  relay.shutdown();
  relay.shutdown();
  EXPECT_EQ(relay.open_streams(), 0U);

  // A client arriving during teardown must not reopen what shutdown closed.
  relay.acquire();
  EXPECT_EQ(relay.open_streams(), 0U);
}

TEST(PeerFaultRelay, an_empty_peer_list_opens_nothing) {
  PeerFaultRelay relay(none(), "/api/v1/faults/stream", ignore());

  relay.acquire();

  EXPECT_EQ(relay.open_streams(), 0U);
  relay.release();
}

}  // namespace
}  // namespace ros2_medkit_gateway
