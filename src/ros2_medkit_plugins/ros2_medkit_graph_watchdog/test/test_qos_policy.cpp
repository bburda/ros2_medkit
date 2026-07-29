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
#include "ros2_medkit_graph_watchdog/qos_policy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace {
using ros2_medkit_graph_watchdog::classify_subscriber_qos;
using ros2_medkit_graph_watchdog::qos_duration_ns;
using ros2_medkit_graph_watchdog::qos_incompatibility;
using ros2_medkit_graph_watchdog::subscriber_starvation;
using ros2_medkit_graph_watchdog::SubscriberQosVerdict;
rmw_qos_profile_t base() {
  return rmw_qos_profile_default;
}

TEST(QosPolicy, BestEffortPubReliableSubIsIncompatible) {
  auto pub = base();
  pub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, ReliablePubBestEffortSubIsCompatible) {
  auto pub = base();
  pub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, VolatilePubTransientLocalSubIsIncompatible) {
  auto pub = base();
  pub.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  auto sub = base();
  sub.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, AutomaticPubManualSubIsIncompatible) {
  auto pub = base();
  pub.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  auto sub = base();
  sub.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, IdenticalDefaultProfilesAreCompatible) {
  EXPECT_TRUE(qos_incompatibility(base(), base()).empty());
}
TEST(QosPolicy, UnresolvedProfilesNeverRaise) {
  // A live TopicEndpointInfo reports a RESOLVED profile, never SYSTEM_DEFAULT/UNKNOWN;
  // the strict RxO pairs below simply never match those enums, so nothing is raised.
  auto pub = base();
  pub.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}

TEST(SubscriberStarvation, StarvedWhenSolelyIncompatiblePublisher) {
  auto pub = base();
  pub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  EXPECT_FALSE(subscriber_starvation({pub}, sub).empty());  // receives from no publisher
}
TEST(SubscriberStarvation, NotStarvedWhenAnyPublisherCompatible) {
  // Mixed-QoS publisher set: one incompatible + one compatible -> the subscriber still gets
  // data from the compatible publisher, so this must NOT be flagged (the multi-publisher FP).
  auto bad = base();
  bad.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto good = base();
  good.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  EXPECT_TRUE(subscriber_starvation({bad, good}, sub).empty());
  EXPECT_TRUE(subscriber_starvation({good, bad}, sub).empty());  // order-independent
}
TEST(SubscriberStarvation, NoPublishersIsNotStarvation) {
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  EXPECT_TRUE(subscriber_starvation({}, sub).empty());  // an orphan, not a QoS fault
}

// classify_subscriber_qos separates "receives nothing" from "one producer is being
// discarded". The second case used to be indistinguishable from healthy.
TEST(ClassifySubscriberQos, StarvedWhenNoPublisherMatches) {
  auto pub = base();
  pub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  const auto result = classify_subscriber_qos({pub, pub}, sub);
  EXPECT_EQ(result.verdict, SubscriberQosVerdict::Starved);
  EXPECT_FALSE(result.reason.empty());
}

TEST(ClassifySubscriberQos, PartialWhenOnlySomePublishersMatch) {
  // /diagnostics shape: several publishers, one hand-rolled BEST_EFFORT, RELIABLE
  // aggregator. DDS never matches that pair, so those diagnostics are discarded forever
  // while the aggregator keeps receiving everyone else's - and nothing errors anywhere.
  auto bad = base();
  bad.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto good = base();
  good.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  EXPECT_EQ(classify_subscriber_qos({bad, good}, sub).verdict, SubscriberQosVerdict::Partial);
  EXPECT_EQ(classify_subscriber_qos({good, bad}, sub).verdict, SubscriberQosVerdict::Partial);  // order-independent
  EXPECT_FALSE(classify_subscriber_qos({bad, good}, sub).reason.empty());
}

TEST(ClassifySubscriberQos, OkWhenEveryPublisherMatches) {
  auto good = base();
  good.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  const auto result = classify_subscriber_qos({good, good}, sub);
  EXPECT_EQ(result.verdict, SubscriberQosVerdict::Ok);
  EXPECT_TRUE(result.reason.empty());
}

TEST(ClassifySubscriberQos, NoPublishersIsOkNotPartial) {
  auto sub = base();
  sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  EXPECT_EQ(classify_subscriber_qos({}, sub).verdict, SubscriberQosVerdict::Ok);  // orphan detector's concern
}

// --- deadline RxO ---

TEST(QosPolicy, DeadlineUnspecifiedPubFiniteSubIsIncompatible) {
  auto pub = base();
  pub.deadline = {0, 0};
  auto sub = base();
  sub.deadline = {0, 100000000};  // 100ms
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, DeadlineBothInfiniteIsCompatible) {
  auto pub = base();
  pub.deadline = RMW_DURATION_INFINITE;
  auto sub = base();
  sub.deadline = RMW_DURATION_INFINITE;
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, DeadlineUnspecifiedSubIsCompatibleRegardlessOfPub) {
  auto pub = base();
  pub.deadline = {0, 200000000};  // 200ms, finite
  auto sub = base();
  sub.deadline = {0, 0};  // unspecified
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, DeadlineInfiniteSubIsCompatible) {
  auto pub = base();
  pub.deadline = {0, 200000000};  // 200ms, finite
  auto sub = base();
  sub.deadline = RMW_DURATION_INFINITE;
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, DeadlineFinitePubLessEqualSubIsCompatible) {
  auto pub = base();
  pub.deadline = {0, 50000000};  // 50ms
  auto sub = base();
  sub.deadline = {0, 100000000};  // 100ms
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, DeadlineFinitePubExceedsSubIsIncompatible) {
  auto pub = base();
  pub.deadline = {0, 200000000};  // 200ms
  auto sub = base();
  sub.deadline = {0, 100000000};  // 100ms
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}

// --- liveliness lease duration RxO ---

TEST(QosPolicy, LeaseUnspecifiedPubFiniteSubIsIncompatible) {
  auto pub = base();
  pub.liveliness_lease_duration = {0, 0};
  auto sub = base();
  sub.liveliness_lease_duration = {0, 100000000};  // 100ms
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, LeaseBothInfiniteIsCompatible) {
  auto pub = base();
  pub.liveliness_lease_duration = RMW_DURATION_INFINITE;
  auto sub = base();
  sub.liveliness_lease_duration = RMW_DURATION_INFINITE;
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, LeaseUnspecifiedSubIsCompatibleRegardlessOfPub) {
  auto pub = base();
  pub.liveliness_lease_duration = {0, 200000000};  // 200ms, finite
  auto sub = base();
  sub.liveliness_lease_duration = {0, 0};  // unspecified
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, LeaseInfiniteSubIsCompatible) {
  auto pub = base();
  pub.liveliness_lease_duration = {0, 200000000};  // 200ms, finite
  auto sub = base();
  sub.liveliness_lease_duration = RMW_DURATION_INFINITE;
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, LeaseFinitePubLessEqualSubIsCompatible) {
  auto pub = base();
  pub.liveliness_lease_duration = {0, 50000000};  // 50ms
  auto sub = base();
  sub.liveliness_lease_duration = {0, 100000000};  // 100ms
  EXPECT_TRUE(qos_incompatibility(pub, sub).empty());
}
TEST(QosPolicy, LeaseFinitePubExceedsSubIsIncompatible) {
  auto pub = base();
  pub.liveliness_lease_duration = {0, 200000000};  // 200ms
  auto sub = base();
  sub.liveliness_lease_duration = {0, 100000000};  // 100ms
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}

// rmw_time_t::sec is uint64_t, so sec * 1e9 can overflow. Wrapping would turn an absurdly
// long duration into a SMALL nanosecond value - i.e. a tighter deadline than the operator
// asked for - and flip the RxO verdict. Saturation keeps it on the permissive side.
TEST(QosPolicy, HugeButFiniteDurationSaturatesInsteadOfWrapping) {
  constexpr uint64_t kMaxU64 = std::numeric_limits<uint64_t>::max();
  const rmw_time_t huge{kMaxU64 / 2, 0};  // finite, but sec * 1e9 overflows
  EXPECT_EQ(qos_duration_ns(huge), kMaxU64);

  const rmw_time_t nsec_carry{kMaxU64 / 1000000000ull, kMaxU64};  // overflows on the +nsec
  EXPECT_EQ(qos_duration_ns(nsec_carry), kMaxU64);

  // A sane value is untouched.
  EXPECT_EQ(qos_duration_ns(rmw_time_t{2, 500000000}), 2500000000ull);
}

// The saturated value must behave like the longest possible duration in a real comparison:
// a publisher offering it against a normal subscriber deadline is incompatible, never
// silently "compatible" via a wrapped small number.
TEST(QosPolicy, SaturatedPubDeadlineStaysIncompatibleAgainstFiniteSub) {
  auto pub = base();
  pub.deadline = {std::numeric_limits<uint64_t>::max() / 2, 0};
  auto sub = base();
  sub.deadline = {0, 100000000};  // 100ms
  EXPECT_FALSE(qos_incompatibility(pub, sub).empty());
}
}  // namespace
