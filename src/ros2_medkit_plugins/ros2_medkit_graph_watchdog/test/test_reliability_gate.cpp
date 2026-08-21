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
#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"  // reliability_allows() free function
#include "ros2_medkit_graph_watchdog/presence_ownership.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_gateway::ServiceInfo;
using ros2_medkit_graph_watchdog::LifecycleWatcher;
using ros2_medkit_graph_watchdog::presence_ownership;
using ros2_medkit_graph_watchdog::PresenceOwnership;
using ros2_medkit_graph_watchdog::reliability_allows;
using ros2_medkit_graph_watchdog::ReliabilityGate;

class ReliabilityGateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("reliability_gate_test_node");
  }
  void TearDown() override {
    node_.reset();
  }
  static IntrospectionInput snap(const std::vector<std::string> & app_ids) {
    IntrospectionInput in;
    for (const auto & id : app_ids) {
      App a;
      a.id = id;
      in.apps.push_back(a);
    }
    return in;
  }
  /// A snapshot whose single App looks MANAGED to the gateway's discovery layer: it
  /// advertises services of the GetState and ChangeState types, which is the whole of the
  /// test find_lifecycle_get_state_path() applies. No such service exists in this process,
  /// so the watcher's seed times out and the tracked entry keeps its empty label - a
  /// genuinely unmeasured managed node over a real LifecycleWatcher and a real ROS graph,
  /// not an injected one.
  static IntrospectionInput managed_snap(const std::string & id) {
    ServiceInfo get_state;
    get_state.full_path = id + "/get_state";
    get_state.type = "lifecycle_msgs/srv/GetState";
    ServiceInfo change_state;
    change_state.full_path = id + "/change_state";
    change_state.type = "lifecycle_msgs/srv/ChangeState";

    App app;
    app.id = id;
    app.bound_fqn = id;
    app.services = {get_state, change_state};

    IntrospectionInput in;
    in.apps.push_back(app);
    return in;
  }
  /// One snapshot carrying a PLAIN app and a managed-looking one, so a single status_json()
  /// payload has to distinguish them. Built as one snapshot rather than two updates because
  /// the claim is about telling the two apart in the SAME payload.
  static IntrospectionInput mixed_snap(const std::string & plain_id, const std::string & managed_id) {
    auto in = managed_snap(managed_id);
    App plain;
    plain.id = plain_id;
    plain.bound_fqn = plain_id;
    in.apps.push_back(plain);
    return in;
  }
  /// One field of the status_json() entry for `id`, or null when the entity is absent.
  static nlohmann::json entity_field(const ReliabilityGate & gate, const std::string & id, const std::string & field) {
    const auto status = gate.status_json();
    for (const auto & entity : status["x-medkit-watchdog"]["entities"]) {
      if (entity["id"] == id) {
        return entity.value(field, nlohmann::json(nullptr));
      }
    }
    return nlohmann::json(nullptr);
  }
  /// The (armed, lifecycle) pair status_json() reports for `id`, or nullopt when the entity
  /// is absent from the payload. The pair is the instrument this file's ownership tests need:
  /// `armed` alone cannot tell a plain node from a managed one whose state was never read,
  /// which is exactly the distinction under test.
  static std::optional<std::pair<bool, nlohmann::json>> armed_and_lifecycle(const ReliabilityGate & gate,
                                                                            const std::string & id) {
    const auto status = gate.status_json();
    for (const auto & entity : status["x-medkit-watchdog"]["entities"]) {
      if (entity["id"] == id) {
        return std::make_pair(entity["armed"].get<bool>(), entity["lifecycle"]);
      }
    }
    return std::nullopt;
  }
  rclcpp::Node::SharedPtr node_;
  std::mutex mtx_;
};

TEST_F(ReliabilityGateTest, SuppressedUntilArmed) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 5);
  EXPECT_FALSE(g.allows_raise("/a"));  // tick 5, first_seen 5
  g.update(snap({"/a"}), 8);           // 3 elapsed
  EXPECT_TRUE(g.allows_raise("/a"));
}

TEST_F(ReliabilityGateTest, UnknownSourceFallsBackToGlobalWarmup) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 1);                    // graph first non-empty at tick 1
  EXPECT_FALSE(g.allows_raise("/some/topic"));  // unknown, 0 elapsed globally
  g.update(snap({"/a"}), 4);
  EXPECT_TRUE(g.allows_raise("/some/topic"));  // 3 elapsed globally
}

TEST_F(ReliabilityGateTest, StatusJsonShape) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 1);
  g.update(snap({"/a"}), 10);
  const auto j = g.status_json();
  ASSERT_TRUE(j.contains("x-medkit-watchdog"));
  const auto & body = j["x-medkit-watchdog"];
  EXPECT_TRUE(body.contains("entities"));
  EXPECT_EQ(body["warmup_cycles"], 3);
}

// The gate composes warmup AND lifecycle: an armed entity whose managed lifecycle is
// non-active must still be suppressed, and reported "warming_up" in status_json. Without
// this the `&& node_ok` conjunct in allows_raise() is unreachable from a gate-level test
// (every other test builds Apps with no services, so nothing is lifecycle-tracked and
// node_ok is always true - dropping the conjunct would leave the suite green).
TEST_F(ReliabilityGateTest, ArmedButLifecycleInactiveIsSuppressed) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 5);
  g.update(snap({"/a"}), 8);          // 3 elapsed -> armed
  EXPECT_TRUE(g.allows_raise("/a"));  // armed, nothing lifecycle-tracked -> allowed

  g.set_lifecycle_state_for_test("/a", "inactive");
  EXPECT_FALSE(g.allows_raise("/a"));  // armed but lifecycle non-active -> suppressed

  const auto j = g.status_json();
  bool found = false;
  for (const auto & e : j["x-medkit-watchdog"]["entities"]) {
    if (e["id"] == "/a") {
      found = true;
      EXPECT_EQ(e["state"], "warming_up");  // suppressed by lifecycle, though armed
      EXPECT_EQ(e["lifecycle"], "inactive");
      EXPECT_EQ(e["armed"], true);
    }
  }
  EXPECT_TRUE(found);

  // Once the node reports active, the same armed entity is allowed.
  g.set_lifecycle_state_for_test("/a", "active");
  EXPECT_TRUE(g.allows_raise("/a"));
}

// A null gate (not yet wired by the plugin) must never suppress a raise.
TEST(ReliabilityAllows, NullGateAlwaysAllows) {
  EXPECT_TRUE(reliability_allows(nullptr, "x"));
}

// The free function must mirror ReliabilityGate::allows_raise for a real gate:
// suppressed while warming up, allowed once armed.
TEST_F(ReliabilityGateTest, MirrorsGateArmedState) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/a"}), 5);
  EXPECT_FALSE(reliability_allows(&g, "/a"));  // not armed yet (0 elapsed)
  g.update(snap({"/a"}), 8);                   // 3 elapsed -> armed
  EXPECT_TRUE(reliability_allows(&g, "/a"));
}

// === presence ownership: two grounds, and only one of them is sticky ===

// The defect this predicate exists for, over a REAL watcher and a REAL graph: a managed node
// whose GetState has not answered yet keeps an empty label, the gate's permissive answer arms
// it anyway, and node_death used to admit it on that answer alone - taking ownership of a
// departure it could not yet attribute, and switching off the one detector that could.
TEST_F(ReliabilityGateTest, UnmeasuredManagedNodeIsArmedButNotOwnedByPresence) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(managed_snap("/unmeasured"), 5);
  g.update(managed_snap("/unmeasured"), 8);  // 3 elapsed -> armed
  // Freshly discovered and unanswered: an empty label with the watcher's attempts still to
  // spend. The settled form of the same label is a different answer - see the sweep below.
  g.set_lifecycle_state_for_test("/unmeasured", "", LifecycleWatcher::kReseedAttempts);

  ASSERT_TRUE(g.allows_raise("/unmeasured"))
      << "the gate's permissive answer for an unread label is deliberate and must not change: "
         "param_drift is app-keyed and depends on it";
  EXPECT_EQ(g.presence_ownership("/unmeasured"), PresenceOwnership::kUnclaimed)
      << "a managed node the watcher has not finished asking about cannot be owned by the "
         "presence detector - the measurement may be one tick away - and it has not been "
         "disowned either, because nothing has been measured at all";

  // The instrument, not a paraphrase of it: armed alone reads the same for a plain node and
  // for a managed one nobody has measured, so the claim is only visible in the PAIR.
  const auto pair = armed_and_lifecycle(g, "/unmeasured");
  ASSERT_TRUE(pair.has_value()) << "the gate never reported the entity at all";
  EXPECT_TRUE(pair->first);
  EXPECT_EQ(pair->second, nlohmann::json(""))
      << "the watcher must hold a TRACKED entry with an empty label here (asked, still waiting), "
         "not the null a non-managed node reports - otherwise this test proves nothing";
}

// Every state the watcher can report about an app, each one its own case, and each with the
// GROUND the answer rests on. A state the predicate treats specially and no test sets is a
// gap, so the sweep is exhaustive by construction: no managed record at all, an empty label
// in BOTH of its forms, and the four primary labels a lifecycle node can sit in.
//
// The empty label is two cases because the bound is what makes it two. While the watcher still
// has GetState attempts to spend the ignorance can still resolve, and taking ownership then
// would race a measurement. Once the budget is spent nothing will ASK again, and refusing
// there hands the departure to a detector that only looks at nodes an operator named in
// `require_active` - so the node is owned, but on a ground that a later label revokes.
TEST_F(ReliabilityGateTest, PresenceOwnershipSweepsEveryLifecycleStateTheWatcherCanReport) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/plain", "/m"}), 5);
  g.update(snap({"/plain", "/m"}), 8);  // 3 elapsed -> both armed

  // No managed record: a plain node, whose departure the presence detector owns outright, and
  // owns EARNED - there is no lifecycle here that could ever say otherwise.
  EXPECT_FALSE(g.lifecycle_state_of("/plain").has_value());
  EXPECT_EQ(g.presence_ownership("/plain"), PresenceOwnership::kEarned);

  struct Case {
    const char * label;
    int reseeds_remaining;
    PresenceOwnership owned;
  };
  const Case cases[] = {{"", LifecycleWatcher::kReseedAttempts, PresenceOwnership::kUnclaimed},
                        {"", 0, PresenceOwnership::kProvisional},
                        {"active", 0, PresenceOwnership::kEarned},
                        {"inactive", LifecycleWatcher::kReseedAttempts, PresenceOwnership::kDisowned},
                        {"unconfigured", LifecycleWatcher::kReseedAttempts, PresenceOwnership::kDisowned},
                        {"finalized", LifecycleWatcher::kReseedAttempts, PresenceOwnership::kDisowned}};
  for (const auto & c : cases) {
    g.set_lifecycle_state_for_test("/m", c.label, c.reseeds_remaining);
    EXPECT_EQ(g.presence_ownership("/m"), c.owned)
        << "lifecycle label '" << c.label << "' with " << c.reseeds_remaining << " re-seed attempt(s) left";
    const auto pair = armed_and_lifecycle(g, "/m");
    ASSERT_TRUE(pair.has_value());
    EXPECT_TRUE(pair->first) << "warmup is elapsed for the whole sweep, so `armed` must not move "
                                "with the label - only ownership does";
    EXPECT_EQ(pair->second, nlohmann::json(c.label));
  }
}

// The instrument the whole e2e suite reads has to tell the two apart on the wire, not only
// through lifecycle_state_of(): a status_json() that serialised "no managed record" and
// "asked, never answered" identically would satisfy every ownership case above while making
// every e2e assertion on the pair meaningless. One payload, both entities, opposite values.
// `measurement_pending` is asserted alongside for the same reason: an unread label means two
// different things and nothing else on the route separates them.
TEST_F(ReliabilityGateTest, StatusJsonSerialisesNoRecordAndAnUnreadLabelDifferently) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(mixed_snap("/plain", "/managed"), 5);
  g.update(mixed_snap("/plain", "/managed"), 8);

  const auto plain = armed_and_lifecycle(g, "/plain");
  ASSERT_TRUE(plain.has_value()) << "the gate never reported the plain entity at all";
  EXPECT_EQ(plain->second, nlohmann::json(nullptr))
      << "a node with no managed record must serialise as null, not as an empty string";
  EXPECT_EQ(entity_field(g, "/plain", "measurement_pending"), nlohmann::json(false))
      << "there is nothing to measure on a node with no lifecycle at all";

  const auto managed = armed_and_lifecycle(g, "/managed");
  ASSERT_TRUE(managed.has_value()) << "the gate never reported the managed entity at all";
  EXPECT_EQ(managed->second, nlohmann::json(""))
      << "a tracked node whose GetState never answered must serialise as \"\" - asked, and still "
         "waiting - which is a different fact from having no lifecycle at all";
}

// The bound, driven by the REAL watcher over a real graph rather than an injected budget: the
// same node is refused while the watcher still has attempts to spend and owned once it has
// spent them. Nothing here sets a label; the only thing that changes across the loop is how
// many GetState calls have actually been issued and failed.
//
// The budget is OBSERVED at both ends, never inferred from how many ticks have gone by: a
// watcher that handed out entries with no attempts at all, and issued no read ever, would
// otherwise satisfy every ownership assertion in this file while never asking a node anything.
TEST_F(ReliabilityGateTest, UnansweredManagedNodeIsOwnedOnlyOnceTheReseedBudgetIsSpent) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(managed_snap("/unanswered"), 5);  // discovered: entry created, seeded, unanswered
  ASSERT_EQ(g.lifecycle_reseeds_remaining_for_test("/unanswered"), LifecycleWatcher::kReseedAttempts)
      << "a newly tracked node must START with the full self-heal budget - if entries were "
         "handed out already spent, the flip below would prove nothing about asking";
  g.update(managed_snap("/unanswered"), 8);  // 3 elapsed -> armed
  ASSERT_TRUE(g.allows_raise("/unanswered"));
  ASSERT_EQ(g.lifecycle_state_of("/unanswered"), std::optional<std::string>(""))
      << "the fixture must be TRACKED with an unread label, or this test measures nothing";
  ASSERT_EQ(entity_field(g, "/unanswered", "measurement_pending"), nlohmann::json(true))
      << "the status route must say the watcher still intends to ask, or an operator cannot "
         "tell this node from one nothing will ever ask about again";

  // Bounded: kReseedAttempts issued reads, plus a margin for a tick whose blocking budget
  // cut the job before it ran (that tick is deliberately not charged - see update()).
  const int kMaxTicks = LifecycleWatcher::kReseedAttempts + 6;
  int left = g.lifecycle_reseeds_remaining_for_test("/unanswered");
  ASSERT_GT(left, 0) << "the arming tick spent the whole budget, so there is no transient half "
                        "of this claim left to observe";
  for (int i = 0; i < kMaxTicks && left > 0; ++i) {
    EXPECT_EQ(g.presence_ownership("/unanswered"), PresenceOwnership::kUnclaimed)
        << "still " << left << " re-seed attempt(s) left, so the ignorance can still resolve";
    g.update(managed_snap("/unanswered"), static_cast<uint64_t>(9 + i));
    const int now_left = g.lifecycle_reseeds_remaining_for_test("/unanswered");
    ASSERT_GE(now_left, 0) << "the entry stopped being tracked mid-test";
    ASSERT_LE(now_left, left) << "a re-seed budget must never grow while the node stays tracked";
    left = now_left;
  }
  ASSERT_EQ(left, 0) << "the re-seed budget never ran out in " << kMaxTicks
                     << " ticks, so the settled half of this claim was never reached";
  EXPECT_EQ(g.presence_ownership("/unanswered"), PresenceOwnership::kProvisional)
      << "the watcher has asked and failed every time it ever will; refusing ownership here "
         "leaves this node's death to a detector that only looks at `require_active` entries";
  EXPECT_EQ(entity_field(g, "/unanswered", "measurement_pending"), nlohmann::json(false))
      << "the route has to say the asking is over, since that is the whole of why the node is "
         "owned at all";
}

// Not a node that starts one way and stays there: a label ARRIVES on a node that had none,
// and later goes away again (a re-bind to a dead binding leaves the entry tracked with its
// label cleared). The answer has to follow the current knowledge on every tick, because the
// callers that latch it latch the GROUND, and a latch built on a stale ground is the defect
// one level up.
TEST_F(ReliabilityGateTest, PresenceOwnershipFollowsALabelThatArrivesAndVanishes) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(managed_snap("/flip"), 5);
  g.update(managed_snap("/flip"), 8);  // armed, still unmeasured
  ASSERT_TRUE(g.allows_raise("/flip"));
  g.set_lifecycle_state_for_test("/flip", "", LifecycleWatcher::kReseedAttempts);
  EXPECT_EQ(g.presence_ownership("/flip"), PresenceOwnership::kUnclaimed);

  g.set_lifecycle_state_for_test("/flip", "active");  // a transition_event finally arrives
  EXPECT_EQ(g.presence_ownership("/flip"), PresenceOwnership::kEarned);

  // Re-bound to a binding that answers nothing: the entry is re-seeded from scratch, so the
  // knowledge is gone AND the budget is back - transient ignorance again, not settled.
  g.set_lifecycle_state_for_test("/flip", "", LifecycleWatcher::kReseedAttempts);
  EXPECT_EQ(g.presence_ownership("/flip"), PresenceOwnership::kUnclaimed);
  EXPECT_TRUE(g.allows_raise("/flip")) << "the permissive answer is unchanged by any of this";

  // The same empty label with the budget spent is a different GROUND, not merely a different
  // answer: owned, but only until something says otherwise.
  g.set_lifecycle_state_for_test("/flip", "", 0);
  EXPECT_EQ(g.presence_ownership("/flip"), PresenceOwnership::kProvisional);

  // And that something is a real label. The subscription outlives the seed budget, so this is
  // the sequence the bound alone would have missed: provisional owner, then the graph says the
  // node was inactive all along.
  g.set_lifecycle_state_for_test("/flip", "inactive", 0);
  EXPECT_EQ(g.presence_ownership("/flip"), PresenceOwnership::kDisowned);

  g.set_lifecycle_state_for_test("/flip", "active");
  EXPECT_EQ(g.presence_ownership("/flip"), PresenceOwnership::kEarned);
}

// Ownership is a conjunction, and the warmup half is the other conjunct: an entity that is
// not armed yet is not owned, whatever its lifecycle says. Without this a predicate that only
// looked at the label would pass every other test in this file.
TEST_F(ReliabilityGateTest, PresenceOwnershipStillRequiresArming) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(snap({"/plain"}), 5);  // 0 elapsed -> not armed
  EXPECT_FALSE(g.allows_raise("/plain"));
  EXPECT_EQ(g.presence_ownership("/plain"), PresenceOwnership::kUnclaimed)
      << "not armed is UNCLAIMED, never disowned: nothing has been measured about this node, "
         "and a caller that already holds the key must not read this as a reason to drop it";
  g.update(snap({"/plain"}), 8);  // 3 elapsed -> armed
  EXPECT_EQ(g.presence_ownership("/plain"), PresenceOwnership::kEarned);
}

// The order the two negative answers are decided in, which is the whole of what separates
// "nothing is known yet" from "the graph says this node is someone else's". node_ok() already
// refuses a managed non-active node, so asking allows_raise() first would answer the same for
// a node measured `inactive` and for one that has simply not armed yet - and a caller that
// releases keys on the negative answer would then drop every node that restarts, because a
// returning node is un-armed for its whole re-warm.
TEST_F(ReliabilityGateTest, NotArmedAndMeasuredElsewhereAreDifferentAnswers) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  // The measured one is MANAGED, so its tracked entry (and the injected label with it) survives
  // the second update below - a service-less app is dropped from the watcher by every update.
  g.update(mixed_snap("/warming", "/measured"), 5);  // 0 elapsed: neither is armed
  g.set_lifecycle_state_for_test("/measured", "inactive");

  ASSERT_FALSE(g.allows_raise("/warming")) << "neither entity may be armed here, or this test "
                                              "cannot show which of the two facts decided it";
  ASSERT_FALSE(g.allows_raise("/measured"));
  EXPECT_EQ(g.presence_ownership("/warming"), PresenceOwnership::kUnclaimed)
      << "a node with no lifecycle record that has not armed yet has been measured as nothing";
  EXPECT_EQ(g.presence_ownership("/measured"), PresenceOwnership::kDisowned)
      << "a measured non-active label is knowledge about who this node belongs to, and it must "
         "outrank the warmup state rather than being hidden behind it";

  // And once armed, the un-measured one is owned outright while the measured one is not.
  g.update(mixed_snap("/warming", "/measured"), 8);
  EXPECT_EQ(g.presence_ownership("/warming"), PresenceOwnership::kEarned);
  EXPECT_EQ(g.presence_ownership("/measured"), PresenceOwnership::kDisowned)
      << "the label survives a re-seed that cannot answer, so the verdict must survive it too";
}

// A null gate (not yet wired by the plugin) must not suppress tracking either - same
// convention as reliability_allows(), and the detectors call both the same way. kEarned, not
// kProvisional: with no watcher behind it, nothing could ever withdraw a provisional grant.
TEST(PresenceOwnershipFreeFunction, NullGateAlwaysOwnsOutright) {
  EXPECT_EQ(presence_ownership(nullptr, "x"), PresenceOwnership::kEarned);
}

// The free function must mirror the member for a real gate, on every ground - a wrapper that
// dropped the gate and always returned kEarned would pass the null-gate test above on its own.
TEST_F(ReliabilityGateTest, PresenceOwnershipFreeFunctionMirrorsTheGate) {
  ReliabilityGate g(3, node_.get(), &mtx_);
  g.update(managed_snap("/mirror"), 5);
  EXPECT_EQ(presence_ownership(&g, "/mirror"), PresenceOwnership::kUnclaimed);  // not armed yet
  g.update(managed_snap("/mirror"), 8);                                         // armed, still unmeasured
  g.set_lifecycle_state_for_test("/mirror", "", LifecycleWatcher::kReseedAttempts);
  EXPECT_EQ(presence_ownership(&g, "/mirror"), PresenceOwnership::kUnclaimed);
  g.set_lifecycle_state_for_test("/mirror", "", 0);
  EXPECT_EQ(presence_ownership(&g, "/mirror"), PresenceOwnership::kProvisional);
  g.set_lifecycle_state_for_test("/mirror", "active");
  EXPECT_EQ(presence_ownership(&g, "/mirror"), PresenceOwnership::kEarned);
}
