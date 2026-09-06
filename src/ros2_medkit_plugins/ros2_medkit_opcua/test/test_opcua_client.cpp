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

#include "ros2_medkit_opcua/opcua_client.hpp"

#include <gtest/gtest.h>

namespace ros2_medkit_gateway {

// These tests verify OpcuaClient behavior without a running OPC-UA server.
// Integration tests with a real server are in the Docker demo.

TEST(OpcuaClientTest, DefaultState) {
  OpcuaClient client;
  EXPECT_FALSE(client.is_connected());
  // endpoint_url() reflects the config default ("opc.tcp://localhost:4840");
  // server_description() is only populated after a successful connect.
  EXPECT_EQ(client.endpoint_url(), "opc.tcp://localhost:4840");
  EXPECT_TRUE(client.server_description().empty());
}

TEST(OpcuaClientTest, ConnectFailsWithoutServer) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = "opc.tcp://localhost:49999";  // No server here
  config.connect_timeout = std::chrono::milliseconds(500);

  EXPECT_FALSE(client.connect(config));
  EXPECT_FALSE(client.is_connected());
}

TEST(OpcuaClientTest, DisconnectWhenNotConnected) {
  OpcuaClient client;
  // Should not crash
  client.disconnect();
  EXPECT_FALSE(client.is_connected());
}

TEST(OpcuaClientTest, BrowseWhenDisconnected) {
  OpcuaClient client;
  auto result = client.browse({0, UA_NS0ID_OBJECTSFOLDER});
  EXPECT_TRUE(result.empty());
}

TEST(OpcuaClientTest, ReadWhenDisconnected) {
  OpcuaClient client;
  auto result = client.read_value({1, "SomeNode"});
  EXPECT_FALSE(result.good);
}

// Issue #389 (read-based active-condition replay): browsing for conditions on a
// disconnected client returns nothing rather than throwing.
TEST(OpcuaClientTest, ReadSourceConditionsWhenDisconnected) {
  OpcuaClient client;
  auto conds = client.read_source_conditions(opcua::NodeId(0, UA_NS0ID_SERVER));
  EXPECT_TRUE(conds.empty());
}

// Issue #479: a disconnected scan must report scan_ok=false so the poller does
// NOT treat the empty result as "no active conditions" and falsely clear faults.
TEST(OpcuaClientTest, ReadSourceConditionsReportsScanFailureWhenDisconnected) {
  OpcuaClient client;
  bool scan_ok = true;  // must be flipped to false
  auto conds = client.read_source_conditions(opcua::NodeId(0, UA_NS0ID_SERVER), &scan_ok);
  EXPECT_TRUE(conds.empty());
  EXPECT_FALSE(scan_ok);
}

TEST(OpcuaClientTest, CreateSubscriptionWhenDisconnected) {
  OpcuaClient client;
  auto id = client.create_subscription(500.0, [](const std::string &, const OpcuaValue &) {});
  EXPECT_EQ(id, 0u);
}

TEST(OpcuaClientTest, RemoveSubscriptionsWhenEmpty) {
  OpcuaClient client;
  // Should not crash
  client.remove_subscriptions();
}

#if !MEDKIT_OPCUA_READ_ONLY
// OpcuaClient::write_value exists only in a write-capable build
// (-DMEDKIT_OPCUA_READ_ONLY=OFF). A read-only build has no declaration for
// these three to call, which is the source-level half of the property
// test_opcua_build_variant asserts on the built object.
TEST(OpcuaClientTest, WriteWhenDisconnected) {
  OpcuaClient client;
  EXPECT_FALSE(client.write_value({1, "SomeNode"}, 42.0));
}

TEST(OpcuaClientTest, WriteValueReturnsNotConnected) {
  OpcuaClient client;
  // Client never connected - write should return NotConnected error
  auto result = client.write_value(opcua::NodeId(0, 1), OpcuaValue{42.0});
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::WriteError::NotConnected);
}

TEST(OpcuaClientTest, WriteValueWithTypeHintDisconnected) {
  OpcuaClient client;
  // Even with a type hint, disconnected client returns NotConnected
  auto result = client.write_value(opcua::NodeId(2, 1), OpcuaValue{75.0}, "float");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::WriteError::NotConnected);
}
#endif  // !MEDKIT_OPCUA_READ_ONLY

// ---------------------------------------------------------------------------
// Issue #389: OPC-UA client security config parsing (pure helpers, no server).
// ---------------------------------------------------------------------------

TEST(OpcuaClientSecurity, ParseSecurityPolicy) {
  bool ok = false;
  EXPECT_EQ(OpcuaClient::parse_security_policy("None", &ok), SecurityPolicy::None);
  EXPECT_TRUE(ok);
  EXPECT_EQ(OpcuaClient::parse_security_policy("", &ok), SecurityPolicy::None);
  EXPECT_TRUE(ok);
  EXPECT_EQ(OpcuaClient::parse_security_policy("basic256sha256"), SecurityPolicy::Basic256Sha256);
  EXPECT_EQ(OpcuaClient::parse_security_policy("Basic256Sha256"), SecurityPolicy::Basic256Sha256);
  EXPECT_EQ(OpcuaClient::parse_security_policy("Aes128Sha256RsaOaep"), SecurityPolicy::Aes128Sha256RsaOaep);
  EXPECT_EQ(OpcuaClient::parse_security_policy("Aes256_Sha256_RsaPss"), SecurityPolicy::Aes256Sha256RsaPss);
  // Unknown -> safe default + ok=false.
  EXPECT_EQ(OpcuaClient::parse_security_policy("Bogus", &ok), SecurityPolicy::None);
  EXPECT_FALSE(ok);
}

TEST(OpcuaClientSecurity, SecurityPolicyUri) {
  EXPECT_EQ(OpcuaClient::security_policy_uri(SecurityPolicy::None), "http://opcfoundation.org/UA/SecurityPolicy#None");
  EXPECT_EQ(OpcuaClient::security_policy_uri(SecurityPolicy::Basic256Sha256),
            "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256");
  EXPECT_EQ(OpcuaClient::security_policy_uri(SecurityPolicy::Aes128Sha256RsaOaep),
            "http://opcfoundation.org/UA/SecurityPolicy#Aes128_Sha256_RsaOaep");
  EXPECT_EQ(OpcuaClient::security_policy_uri(SecurityPolicy::Aes256Sha256RsaPss),
            "http://opcfoundation.org/UA/SecurityPolicy#Aes256_Sha256_RsaPss");
}

TEST(OpcuaClientSecurity, ParseSecurityMode) {
  bool ok = false;
  EXPECT_EQ(OpcuaClient::parse_security_mode("none", &ok), SecurityMode::None);
  EXPECT_TRUE(ok);
  EXPECT_EQ(OpcuaClient::parse_security_mode("Sign"), SecurityMode::Sign);
  EXPECT_EQ(OpcuaClient::parse_security_mode("SignAndEncrypt"), SecurityMode::SignAndEncrypt);
  EXPECT_EQ(OpcuaClient::parse_security_mode("sign_and_encrypt"), SecurityMode::SignAndEncrypt);
  EXPECT_EQ(OpcuaClient::parse_security_mode("bogus", &ok), SecurityMode::None);
  EXPECT_FALSE(ok);
}

TEST(OpcuaClientSecurity, ParseUserAuthMode) {
  EXPECT_EQ(OpcuaClient::parse_user_auth_mode("anonymous"), UserAuthMode::Anonymous);
  EXPECT_EQ(OpcuaClient::parse_user_auth_mode(""), UserAuthMode::Anonymous);
  EXPECT_EQ(OpcuaClient::parse_user_auth_mode("username"), UserAuthMode::UsernamePassword);
  EXPECT_EQ(OpcuaClient::parse_user_auth_mode("UsernamePassword"), UserAuthMode::UsernamePassword);
  EXPECT_EQ(OpcuaClient::parse_user_auth_mode("x509"), UserAuthMode::X509);
  EXPECT_EQ(OpcuaClient::parse_user_auth_mode("certificate"), UserAuthMode::X509);
}

TEST(OpcuaClientSecurity, RequiresSecureChannel) {
  OpcuaClientConfig cfg;
  EXPECT_FALSE(OpcuaClient::requires_secure_channel(cfg));  // defaults: None/None
  cfg.user_auth_mode = UserAuthMode::UsernamePassword;
  // Username/password alone does NOT imply an encrypted channel.
  EXPECT_FALSE(OpcuaClient::requires_secure_channel(cfg));
  cfg.security_mode = SecurityMode::Sign;
  EXPECT_TRUE(OpcuaClient::requires_secure_channel(cfg));
  cfg.security_mode = SecurityMode::None;
  cfg.security_policy = SecurityPolicy::Basic256Sha256;
  EXPECT_TRUE(OpcuaClient::requires_secure_channel(cfg));
}

// Issue #478: non-anonymous credentials on an unencrypted channel must be
// flagged as cleartext so the plugin can warn loudly.
TEST(OpcuaClientSecurity, CredentialsSentInClear) {
  OpcuaClientConfig cfg;
  // Anonymous on a plain channel: nothing to leak.
  EXPECT_FALSE(OpcuaClient::credentials_sent_in_clear(cfg));

  // Username/password with no secured channel: credentials travel in clear.
  cfg.user_auth_mode = UserAuthMode::UsernamePassword;
  EXPECT_TRUE(OpcuaClient::credentials_sent_in_clear(cfg));

  // X.509 identity with no secured channel: still cleartext.
  cfg.user_auth_mode = UserAuthMode::X509;
  EXPECT_TRUE(OpcuaClient::credentials_sent_in_clear(cfg));

  // Once the channel is signed, credentials are protected.
  cfg.security_mode = SecurityMode::Sign;
  EXPECT_FALSE(OpcuaClient::credentials_sent_in_clear(cfg));

  // Encryption via the policy alone (mode None) also protects them.
  cfg.security_mode = SecurityMode::None;
  cfg.security_policy = SecurityPolicy::Basic256Sha256;
  EXPECT_FALSE(OpcuaClient::credentials_sent_in_clear(cfg));

  // Anonymous is never cleartext regardless of channel.
  cfg.user_auth_mode = UserAuthMode::Anonymous;
  cfg.security_policy = SecurityPolicy::None;
  EXPECT_FALSE(OpcuaClient::credentials_sent_in_clear(cfg));
}

TEST(OpcuaClientSecurity, SecurityConfigConflict) {
  OpcuaClientConfig cfg;
  // Both None (default): consistent, no conflict.
  EXPECT_FALSE(OpcuaClient::security_config_conflict(cfg));

  // Mode set but policy None: the policy URI would be left unpinned - conflict.
  cfg.security_mode = SecurityMode::Sign;
  EXPECT_TRUE(OpcuaClient::security_config_conflict(cfg));
  cfg.security_mode = SecurityMode::SignAndEncrypt;
  EXPECT_TRUE(OpcuaClient::security_config_conflict(cfg));

  // Both set: consistent.
  cfg.security_policy = SecurityPolicy::Basic256Sha256;
  EXPECT_FALSE(OpcuaClient::security_config_conflict(cfg));

  // Policy set but mode None: equally malformed - conflict.
  cfg.security_mode = SecurityMode::None;
  EXPECT_TRUE(OpcuaClient::security_config_conflict(cfg));
}

TEST(OpcuaClientSecurity, ConnectRejectsContradictorySecurityConfig) {
  // security_mode=Sign with security_policy=None must fail before touching the
  // network (would otherwise silently run an unpinned-policy channel).
  OpcuaClient client;
  OpcuaClientConfig cfg;
  cfg.endpoint_url = "opc.tcp://localhost:49997";
  cfg.connect_timeout = std::chrono::milliseconds(500);
  cfg.security_mode = SecurityMode::Sign;
  cfg.security_policy = SecurityPolicy::None;
  EXPECT_FALSE(client.connect(cfg));
  EXPECT_FALSE(client.is_connected());
}

TEST(OpcuaClientSecurity, ConnectSecureWithoutCertFailsFast) {
  // A secured profile with no client cert/key must fail config-time, before
  // touching the network, and leave the client disconnected.
  OpcuaClient client;
  OpcuaClientConfig cfg;
  cfg.endpoint_url = "opc.tcp://localhost:49998";
  cfg.connect_timeout = std::chrono::milliseconds(500);
  cfg.security_policy = SecurityPolicy::Basic256Sha256;
  cfg.security_mode = SecurityMode::SignAndEncrypt;
  EXPECT_FALSE(client.connect(cfg));
  EXPECT_FALSE(client.is_connected());
}

TEST(OpcuaClientTest, CurrentConfigPersistence) {
  OpcuaClient client;
  OpcuaClientConfig cfg;
  cfg.endpoint_url = "opc.tcp://test:4840";
  cfg.connect_timeout = std::chrono::milliseconds(1000);
  // connect will fail (no server) but config should be stored
  client.connect(cfg);
  auto stored = client.current_config();
  EXPECT_EQ(stored.endpoint_url, "opc.tcp://test:4840");
  EXPECT_EQ(stored.connect_timeout, std::chrono::milliseconds(1000));
}

// ---------------------------------------------------------------------------
// Issue #386: native OPC-UA AlarmCondition event subscription primitives.
// These tests cover the disconnected-state contract of the new public API.
// End-to-end event flow (real server emits AlarmConditionType, callback fires,
// state machine advances) is exercised in the docker integration test added
// alongside the test_alarm_server fixture in a follow-up commit on the same
// branch.
// ---------------------------------------------------------------------------

TEST(OpcuaClientTest, GenerationStartsAtZero) {
  OpcuaClient client;
  EXPECT_EQ(client.current_generation(), 0u);
}

TEST(OpcuaClientTest, AddEventMonitoredItemWhenDisconnected) {
  OpcuaClient client;
  auto mi = client.add_event_monitored_item(
      /*sub_id=*/1, opcua::NodeId(0, UA_NS0ID_SERVER), /*select=*/{},
      [](const auto &, const auto &, const auto &, const auto &) {});
  EXPECT_EQ(mi, 0u);
}

TEST(OpcuaClientTest, RemoveEventMonitoredItemUnknownIdReturnsFalse) {
  OpcuaClient client;
  EXPECT_FALSE(client.remove_event_monitored_item(/*sub_id=*/1, /*mi_id=*/9999));
}

TEST(OpcuaClientTest, RemoveEventMonitoredItemUnknownIdDoesNotBumpGeneration) {
  // remove_event_monitored_item is per-MI cleanup. It must NOT touch the
  // global generation counter - that would silently invalidate every peer
  // monitored item's trampoline check and drop their callbacks even though
  // the underlying server-side MIs are still live. Per-MI staleness is
  // tracked by EventCallbackContext::active instead. (Copilot review on
  // PR #387.)
  OpcuaClient client;
  uint64_t before = client.current_generation();
  client.remove_event_monitored_item(/*sub_id=*/1, /*mi_id=*/9999);
  EXPECT_EQ(client.current_generation(), before);
}

TEST(OpcuaClientTest, CallMethodWhenDisconnected) {
  OpcuaClient client;
  auto result = client.call_method(opcua::NodeId(0, UA_NS0ID_SERVER), opcua::NodeId(0, 11489), {});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::MethodError::NotConnected);
}

TEST(OpcuaClientTest, GenerationBumpsOnDisconnect) {
  OpcuaClient client;
  // The disconnect-without-connect path is a no-op - generation should not
  // change because there is no live subscription state to invalidate.
  client.disconnect();
  EXPECT_EQ(client.current_generation(), 0u);
}

TEST(OpcuaClientTest, RemoveSubscriptionsBumpsGenerationEvenWhenEmpty) {
  // remove_subscriptions() is a publicly exposed bulk-cleanup hook used by the
  // poller's reconnect path. It must increment the generation so any
  // captured-but-not-yet-fired callbacks from the now-defunct subscription set
  // are filtered out by the trampoline. This contract holds even when there
  // are no entries, because the poller does not synchronize fine-grained.
  OpcuaClient client;
  uint64_t before = client.current_generation();
  client.remove_subscriptions();
  EXPECT_GT(client.current_generation(), before);
}

// ---------------------------------------------------------------------------
// Issue #386 follow-up: cover the OPC-UA Part 4 §5.11.2 Call result
// classification paths (overall statusCode + per-argument inputArgumentResults)
// directly. The integration test in docker/scripts/run_alarm_tests.sh
// triggers the per-arg ``BadEventIdUnknown`` flow during real ack/confirm,
// but a unit-test anchor catches future regressions in seconds rather than
// the ~15 minutes a docker run takes.
// ---------------------------------------------------------------------------

TEST(OpcuaClientStatusToMethodError, MethodNotFoundFamily) {
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADMETHODINVALID, "x").code,
            OpcuaClient::MethodError::MethodNotFound);
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADNODEIDUNKNOWN, "x").code,
            OpcuaClient::MethodError::MethodNotFound);
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADNOTSUPPORTED, "x").code,
            OpcuaClient::MethodError::MethodNotFound);
}

TEST(OpcuaClientStatusToMethodError, InvalidArgumentFamily) {
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADARGUMENTSMISSING, "x").code,
            OpcuaClient::MethodError::InvalidArgument);
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADINVALIDARGUMENT, "x").code,
            OpcuaClient::MethodError::InvalidArgument);
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADTYPEMISMATCH, "x").code,
            OpcuaClient::MethodError::InvalidArgument);
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADTOOMANYARGUMENTS, "x").code,
            OpcuaClient::MethodError::InvalidArgument);
}

TEST(OpcuaClientStatusToMethodError, TimeoutAndDefault) {
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADTIMEOUT, "x").code,
            OpcuaClient::MethodError::MethodTimeout);
  // BadEventIdUnknown is not in any explicit case - falls through to
  // TransportError. The SOVD layer maps that to HTTP 502, prompting the
  // caller to retry rather than treat the request as fatally invalid.
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADEVENTIDUNKNOWN, "x").code,
            OpcuaClient::MethodError::TransportError);
  // Generic transport error.
  EXPECT_EQ(OpcuaClient::status_to_method_error(UA_STATUSCODE_BADCONNECTIONCLOSED, "x").code,
            OpcuaClient::MethodError::TransportError);
}

TEST(OpcuaClientClassifyCallResult, GoodOverallNoArgsSucceeds) {
  auto result = OpcuaClient::classify_call_result(UA_STATUSCODE_GOOD, {});
  EXPECT_TRUE(result.has_value());
}

TEST(OpcuaClientClassifyCallResult, GoodOverallAllArgsGoodSucceeds) {
  auto result = OpcuaClient::classify_call_result(UA_STATUSCODE_GOOD, {UA_STATUSCODE_GOOD, UA_STATUSCODE_GOOD});
  EXPECT_TRUE(result.has_value());
}

TEST(OpcuaClientClassifyCallResult, BadOverallStatusReturnsError) {
  auto result = OpcuaClient::classify_call_result(UA_STATUSCODE_BADMETHODINVALID, {UA_STATUSCODE_GOOD});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::MethodError::MethodNotFound);
}

TEST(OpcuaClientClassifyCallResult, GoodOverallBadInputArgReturnsError) {
  // The exact AlarmConditionType.Acknowledge case: server validates the
  // EventId argument, finds it's been superseded, sets statusCode=Good
  // (the call itself was well-formed) but inputArgumentResults[0]=
  // BadEventIdUnknown to signal the ack did NOT take effect.
  auto result = OpcuaClient::classify_call_result(UA_STATUSCODE_GOOD, {UA_STATUSCODE_BADEVENTIDUNKNOWN});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::MethodError::TransportError);
  EXPECT_NE(result.error().message.find("input arg 0"), std::string::npos);
}

TEST(OpcuaClientClassifyCallResult, FirstBadArgWinsOverLaterBadArgs) {
  // Multiple per-arg failures: classifier reports the earliest one so the
  // diagnostic message points at the first thing that broke (consistent
  // with how the spec lists arguments left-to-right).
  auto result = OpcuaClient::classify_call_result(UA_STATUSCODE_GOOD,
                                                  {UA_STATUSCODE_BADTYPEMISMATCH, UA_STATUSCODE_BADEVENTIDUNKNOWN});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::MethodError::InvalidArgument);
  EXPECT_NE(result.error().message.find("input arg 0"), std::string::npos);
}

TEST(OpcuaClientClassifyCallResult, OverallStatusBeatsArgResults) {
  // When both overall statusCode and arg results are bad, overall wins:
  // a transport-level rejection means the server never even validated args.
  auto result = OpcuaClient::classify_call_result(UA_STATUSCODE_BADTIMEOUT, {UA_STATUSCODE_BADEVENTIDUNKNOWN});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::MethodError::MethodTimeout);
}

}  // namespace ros2_medkit_gateway
