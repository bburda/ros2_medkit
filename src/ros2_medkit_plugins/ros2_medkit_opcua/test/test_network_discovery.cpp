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

#include "ros2_medkit_opcua/network_discovery.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {
namespace {

// --------------------------------------------------------------------------- //
// expand_cidr_hosts
// --------------------------------------------------------------------------- //
TEST(ExpandCidr, Slash24ExcludesNetworkAndBroadcast) {
  bool ok = false;
  const auto hosts = expand_cidr_hosts("192.168.1.0/24", &ok);
  EXPECT_TRUE(ok);
  ASSERT_EQ(hosts.size(), 254u);
  EXPECT_EQ(hosts.front(), "192.168.1.1");
  EXPECT_EQ(hosts.back(), "192.168.1.254");
  // Network and broadcast are excluded.
  EXPECT_EQ(std::count(hosts.begin(), hosts.end(), "192.168.1.0"), 0);
  EXPECT_EQ(std::count(hosts.begin(), hosts.end(), "192.168.1.255"), 0);
}

TEST(ExpandCidr, NonZeroBaseIsMaskedToNetwork) {
  bool ok = false;
  const auto hosts = expand_cidr_hosts("192.168.1.37/24", &ok);
  EXPECT_TRUE(ok);
  ASSERT_EQ(hosts.size(), 254u);
  EXPECT_EQ(hosts.front(), "192.168.1.1");
}

TEST(ExpandCidr, Slash30HasTwoUsableHosts) {
  bool ok = false;
  const auto hosts = expand_cidr_hosts("10.0.0.0/30", &ok);
  EXPECT_TRUE(ok);
  ASSERT_EQ(hosts.size(), 2u);
  EXPECT_EQ(hosts[0], "10.0.0.1");
  EXPECT_EQ(hosts[1], "10.0.0.2");
}

TEST(ExpandCidr, Slash32IsSingleHost) {
  bool ok = false;
  const auto hosts = expand_cidr_hosts("192.168.1.10/32", &ok);
  EXPECT_TRUE(ok);
  ASSERT_EQ(hosts.size(), 1u);
  EXPECT_EQ(hosts[0], "192.168.1.10");
}

TEST(ExpandCidr, Slash31HasTwoHostsNoReservation) {
  bool ok = false;
  const auto hosts = expand_cidr_hosts("192.168.1.0/31", &ok);
  EXPECT_TRUE(ok);
  ASSERT_EQ(hosts.size(), 2u);
  EXPECT_EQ(hosts[0], "192.168.1.0");
  EXPECT_EQ(hosts[1], "192.168.1.1");
}

TEST(ExpandCidr, RejectsTooWidePrefix) {
  bool ok = true;
  const auto hosts = expand_cidr_hosts("10.0.0.0/8", &ok);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(hosts.empty());
}

TEST(ExpandCidr, RejectsMalformed) {
  for (const char * bad : {"192.168.1.0", "192.168.1.0/", "192.168.1.0/33", "not.an.ip/24", "192.168.1.0/xx"}) {
    bool ok = true;
    const auto hosts = expand_cidr_hosts(bad, &ok);
    EXPECT_FALSE(ok) << bad;
    EXPECT_TRUE(hosts.empty()) << bad;
  }
}

// --------------------------------------------------------------------------- //
// parse_discovery_config
// --------------------------------------------------------------------------- //
TEST(ParseDiscoveryConfig, DefaultsDisabled) {
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(nlohmann::json::object(), [&](const std::string & m) {
    warnings.push_back(m);
  });
  EXPECT_FALSE(cfg.enabled);
  EXPECT_EQ(cfg.mode, "active");
  ASSERT_EQ(cfg.ports.size(), 1u);
  EXPECT_EQ(cfg.ports[0], 4840);
  EXPECT_TRUE(cfg.anonymous_none_only);
  // Unset, NOT 0: an absent key means "no cadence stated" (the caller takes its
  // built-in default), while an explicit 0 means "never re-scan".
  EXPECT_FALSE(cfg.interval_s.has_value());
}

TEST(ParseDiscoveryConfig, ExplicitZeroIntervalIsKeptAsAnExplicitZero) {
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(nlohmann::json{{"interval_s", 0}}, [&](const std::string & m) {
    warnings.push_back(m);
  });
  ASSERT_TRUE(cfg.interval_s.has_value());
  EXPECT_EQ(*cfg.interval_s, 0);
  EXPECT_TRUE(warnings.empty());
}

TEST(ParseDiscoveryConfig, NegativeIntervalWarnsAndLeavesTheCadenceUnset) {
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(nlohmann::json{{"interval_s", -5}}, [&](const std::string & m) {
    warnings.push_back(m);
  });
  EXPECT_FALSE(cfg.interval_s.has_value());
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("interval_s"), std::string::npos);
  // The warning must not tell the operator the kept default is one-shot: an
  // unset interval re-scans on the built-in cadence, only an explicit 0 stops.
  EXPECT_EQ(warnings[0].find("0 = one-shot"), std::string::npos) << warnings[0];
}

TEST(ParseDiscoveryConfig, ReadsAllKnownKeys) {
  const nlohmann::json j = {
      {"enabled", true},
      {"mode", "active"},
      {"subnets", {"192.168.1.0/24", "10.0.0.0/24"}},
      {"ports", {4840, 4841}},
      {"connect_timeout_ms", 300},
      {"scan_concurrency", 64},
      {"identify_timeout_ms", 2000},
      {"interval_s", 900},
      {"anonymous_none_only", false},
  };
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(j, [&](const std::string & m) {
    warnings.push_back(m);
  });
  EXPECT_TRUE(cfg.enabled);
  ASSERT_EQ(cfg.subnets.size(), 2u);
  EXPECT_EQ(cfg.subnets[0], "192.168.1.0/24");
  ASSERT_EQ(cfg.ports.size(), 2u);
  EXPECT_EQ(cfg.ports[1], 4841);
  EXPECT_EQ(cfg.connect_timeout_ms, 300);
  EXPECT_EQ(cfg.scan_concurrency, 64);
  EXPECT_EQ(cfg.identify_timeout_ms, 2000);
  ASSERT_TRUE(cfg.interval_s.has_value());
  EXPECT_EQ(*cfg.interval_s, 900);
  EXPECT_FALSE(cfg.anonymous_none_only);
  EXPECT_TRUE(warnings.empty());
}

TEST(ParseDiscoveryConfig, CidrKeyMergesWithSubnets) {
  const nlohmann::json j = {{"subnets", {"192.168.1.0/24"}}, {"cidr", "10.0.0.0/24"}};
  const auto cfg = parse_discovery_config(j, nullptr);
  ASSERT_EQ(cfg.subnets.size(), 2u);
  EXPECT_EQ(cfg.subnets[1], "10.0.0.0/24");
}

TEST(ParseDiscoveryConfig, WarnsOnUnknownKey) {
  const nlohmann::json j = {{"enabled", true}, {"typpo", 5}};
  std::vector<std::string> warnings;
  parse_discovery_config(j, [&](const std::string & m) {
    warnings.push_back(m);
  });
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("typpo"), std::string::npos);
}

TEST(ParseDiscoveryConfig, RejectsOutOfRangePortAndKeepsValidOnes) {
  const nlohmann::json j = {{"ports", {4840, 70000, 0, 502}}};
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(j, [&](const std::string & m) {
    warnings.push_back(m);
  });
  ASSERT_EQ(cfg.ports.size(), 2u);
  EXPECT_EQ(cfg.ports[0], 4840);
  EXPECT_EQ(cfg.ports[1], 502);
  EXPECT_EQ(warnings.size(), 2u);  // 70000 and 0
}

TEST(ParseDiscoveryConfig, WarnsOnPassiveMode) {
  const nlohmann::json j = {{"mode", "passive"}};
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(j, [&](const std::string & m) {
    warnings.push_back(m);
  });
  EXPECT_EQ(cfg.mode, "passive");
  ASSERT_FALSE(warnings.empty());
}

TEST(ParseDiscoveryConfig, NegativeConcurrencyKeepsDefault) {
  const nlohmann::json j = {{"scan_concurrency", -5}};
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(j, [&](const std::string & m) {
    warnings.push_back(m);
  });
  EXPECT_EQ(cfg.scan_concurrency, 100);  // default retained
  ASSERT_FALSE(warnings.empty());
}

TEST(ParseDiscoveryConfig, ClampsExcessiveConcurrency) {
  const nlohmann::json j = {{"scan_concurrency", 100000}};
  std::vector<std::string> warnings;
  const auto cfg = parse_discovery_config(j, [&](const std::string & m) {
    warnings.push_back(m);
  });
  EXPECT_EQ(cfg.scan_concurrency, 512);  // clamped to the sane maximum
  ASSERT_FALSE(warnings.empty());
  EXPECT_NE(warnings[0].find("clamping"), std::string::npos);
}

// --------------------------------------------------------------------------- //
// NetworkDiscovery::run - reproduces the validated live-network scenario
// (192.168.1.9 LDS, .10 Siemens S7-1500, .11 S7comm-only) with injected fakes.
// --------------------------------------------------------------------------- //
namespace {

// Fake port scanner backed by a set of open "ip:port" hosts.
PortScanFn make_scan(std::set<std::string> open) {
  return [open = std::move(open)](const std::string & ip, uint16_t port, int) {
    return open.count(ip + ":" + std::to_string(port)) > 0;
  };
}

// Fake identify keyed by connect URL.
IdentifyFn make_identify(std::map<std::string, IdentifyResult> table) {
  return [table = std::move(table)](const std::string & url, int) -> IdentifyResult {
    const auto it = table.find(url);
    if (it != table.end()) {
      return it->second;
    }
    IdentifyResult r;
    r.error = "unreachable";
    return r;
  };
}

IdentifyResult siemens_identity() {
  IdentifyResult r;
  r.ok = true;
  r.advertised_url = "opc.tcp://192.168.1.10:4840";
  r.application_uri = "urn:SIMATIC.S7-1500.OPC-UA.Application:Software PLC_1";
  r.product_uri = "https://www.siemens.com/s7-1500";
  r.application_name = "SIMATIC.S7-1500.OPC-UA.Application:Software PLC_1";
  r.application_type = 0;  // Server
  r.security_policies = {{"None", 1}, {"Basic256Sha256", 2}, {"Basic256Sha256", 3}};
  r.anonymous_none_available = true;
  return r;
}

IdentifyResult lds_identity() {
  IdentifyResult r;
  r.ok = true;
  // The LDS advertised a bare, non-resolvable NetBIOS hostname on our LAN.
  r.advertised_url = "opc.tcp://DESKTOP-TG98AEV:4840";
  r.application_uri = "urn:DESKTOP-TG98AEV:UALocalDiscoveryServer";
  r.product_uri = "http://opcfoundation.org/UA/LocalDiscoveryServer";
  r.application_name = "UA Local Discovery Server";
  r.application_type = 3;  // DiscoveryServer / LDS
  r.security_policies = {{"None", 1}, {"Basic256", 2}};
  r.anonymous_none_available = true;
  return r;
}

OpcuaDiscoveryConfig scenario_cfg() {
  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};  // explicit, so no local-interface derivation
  cfg.ports = {4840, 102};
  return cfg;
}

}  // namespace

TEST(NetworkDiscoveryRun, ClassifiesLiveNetworkScenario) {
  auto scan = make_scan({"192.168.1.9:4840", "192.168.1.10:4840", "192.168.1.10:102", "192.168.1.11:102"});
  auto identify = make_identify({
      {"opc.tcp://192.168.1.9:4840", lds_identity()},
      {"opc.tcp://192.168.1.10:4840", siemens_identity()},
  });
  NetworkDiscovery disc(scenario_cfg(), scan, identify);
  const auto eps = disc.run();

  // 4 endpoints: LDS(.9:4840), Siemens(.10:4840), s7comm lead(.10:102), s7comm lead(.11:102).
  ASSERT_EQ(eps.size(), 4u);

  const DiscoveredEndpoint * siemens = nullptr;
  const DiscoveredEndpoint * lds = nullptr;
  std::vector<const DiscoveredEndpoint *> leads;
  for (const auto & ep : eps) {
    if (ep.ip == "192.168.1.10" && ep.port == 4840) {
      siemens = &ep;
    } else if (ep.ip == "192.168.1.9" && ep.port == 4840) {
      lds = &ep;
    } else if (ep.port == 102) {
      leads.push_back(&ep);
    }
  }
  ASSERT_NE(siemens, nullptr);
  ASSERT_NE(lds, nullptr);
  EXPECT_EQ(leads.size(), 2u);

  // Siemens: OPC-UA data server, None/Anonymous, identified.
  EXPECT_TRUE(siemens->is_data_server());
  EXPECT_TRUE(siemens->anonymous_none_available);
  EXPECT_EQ(siemens->product_uri, "https://www.siemens.com/s7-1500");
  EXPECT_TRUE(siemens->identify_error.empty());

  // LDS: discovery server, not a data server.
  EXPECT_FALSE(lds->is_data_server());
  EXPECT_EQ(lds->application_type, 3);

  // URL preference: connect URL is the SCANNED ip:port even though the LDS
  // advertised a non-resolvable hostname; the advertised value is kept as meta.
  EXPECT_EQ(lds->endpoint_url, "opc.tcp://192.168.1.9:4840");
  EXPECT_EQ(lds->advertised_url, "opc.tcp://DESKTOP-TG98AEV:4840");

  // s7comm leads: non-OPC-UA, recorded but not identified.
  for (const auto * lead : leads) {
    EXPECT_EQ(lead->protocol, "unknown");
    EXPECT_FALSE(lead->is_data_server() && lead->anonymous_none_available);
  }
}

TEST(NetworkDiscoveryRun, DedupsByApplicationUri) {
  // Same PLC reachable on two IPs (dual-homed): one deduped entry by URI.
  auto scan = make_scan({"192.168.1.10:4840", "192.168.1.20:4840"});
  auto identify = make_identify({
      {"opc.tcp://192.168.1.10:4840", siemens_identity()},
      {"opc.tcp://192.168.1.20:4840", siemens_identity()},
  });
  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};
  cfg.ports = {4840};
  NetworkDiscovery disc(cfg, scan, identify);
  const auto eps = disc.run();
  ASSERT_EQ(eps.size(), 1u);
  // Lowest ip:port wins (deterministic).
  EXPECT_EQ(eps[0].ip, "192.168.1.10");
}

TEST(NetworkDiscoveryRun, ParallelIdentifyKeepsDeterministicOrder) {
  // Many concurrent hits, each with a distinct ApplicationUri. The identify
  // runs on the bounded pool, but results are written by index so the output
  // must stay strictly sorted by ip regardless of completion order.
  std::set<std::string> open;
  std::map<std::string, IdentifyResult> table;
  for (int host = 1; host <= 60; ++host) {
    const std::string ip = "192.168.1." + std::to_string(host);
    open.insert(ip + ":4840");
    IdentifyResult r = siemens_identity();
    r.application_uri = "urn:server-" + std::to_string(host);  // distinct => no dedup
    r.advertised_url = "opc.tcp://" + ip + ":4840";
    table.emplace("opc.tcp://" + ip + ":4840", r);
  }
  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};
  cfg.ports = {4840};
  cfg.scan_concurrency = 32;
  NetworkDiscovery disc(cfg, make_scan(open), make_identify(table));
  const auto eps = disc.run();

  ASSERT_EQ(eps.size(), 60u);
  for (size_t i = 0; i + 1 < eps.size(); ++i) {
    EXPECT_LT(std::stoi(eps[i].ip.substr(eps[i].ip.rfind('.') + 1)),
              std::stoi(eps[i + 1].ip.substr(eps[i + 1].ip.rfind('.') + 1)))
        << "endpoints must stay ip-ordered after the parallel identify";
  }
  EXPECT_EQ(eps.front().ip, "192.168.1.1");
  EXPECT_EQ(eps.back().ip, "192.168.1.60");
}

TEST(NetworkDiscoveryRun, IdentifyFailureRecordedAsLead) {
  auto scan = make_scan({"192.168.1.50:4840"});
  IdentifyResult bad;
  bad.error = "BadTimeout";
  auto identify = make_identify({{"opc.tcp://192.168.1.50:4840", bad}});
  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};
  cfg.ports = {4840};
  NetworkDiscovery disc(cfg, scan, identify);
  const auto eps = disc.run();
  ASSERT_EQ(eps.size(), 1u);
  EXPECT_EQ(eps[0].identify_error, "BadTimeout");
  EXPECT_EQ(NetworkDiscovery::select_auto_endpoint(eps, true), nullptr);
}

// --------------------------------------------------------------------------- //
// run(cancelled): a shutdown must not wait out a whole sweep
// --------------------------------------------------------------------------- //
TEST(NetworkDiscoveryRun, CancelStopsTheSweepInsteadOfProbingEveryHost) {
  // A /24 is 254 probes and a legal /16 is 65k; the caller runs them on the poll
  // thread a shutdown has to join. With scan_concurrency 1 the sweep is
  // sequential, so the probe count is exactly what the cancel predicate allowed.
  std::atomic<int> probes{0};
  std::atomic<bool> stop{false};
  auto scan = [&probes, &stop](const std::string &, uint16_t, int) {
    if (probes.fetch_add(1) + 1 >= 5) {
      stop.store(true);  // the shutdown flag flipping mid-sweep
    }
    return false;
  };
  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};
  cfg.ports = {4840};
  cfg.scan_concurrency = 1;

  NetworkDiscovery disc(cfg, scan, make_identify({}));
  const auto eps = disc.run([&stop]() {
    return stop.load();
  });

  EXPECT_TRUE(eps.empty());
  // One in-flight probe per worker may still complete after the flag flips.
  EXPECT_GE(probes.load(), 5);
  EXPECT_LE(probes.load(), 6) << "the sweep kept probing after it was cancelled";
}

TEST(NetworkDiscoveryRun, WithoutCancellationEveryHostIsStillProbed) {
  // Positive control for the test above on the same harness: the identical
  // sweep with no cancel predicate visits all 254 hosts, so a low probe count
  // there is the cancellation and not a broken fake.
  std::atomic<int> probes{0};
  auto scan = [&probes](const std::string &, uint16_t, int) {
    probes.fetch_add(1);
    return false;
  };
  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};
  cfg.ports = {4840};
  cfg.scan_concurrency = 1;

  NetworkDiscovery disc(cfg, scan, make_identify({}));
  const auto eps = disc.run();
  EXPECT_TRUE(eps.empty());
  EXPECT_EQ(probes.load(), 254);
}

TEST(NetworkDiscoveryRun, CancelBetweenSweepAndIdentifySkipsTheIdentifyRoundTrips) {
  // The identify phase is a separate batch, and each GetEndpoints blocks for up
  // to identify_timeout_ms. A cancel that arrives once the sweep is done must
  // not still pay for one round-trip per hit.
  std::atomic<bool> stop{false};
  auto scan = [&stop](const std::string & ip, uint16_t, int) {
    const bool hit = ip == "192.168.1.10" || ip == "192.168.1.11";
    if (ip == "192.168.1.254") {
      stop.store(true);  // sweep finished, shutdown requested
    }
    return hit;
  };
  std::atomic<int> identifies{0};
  auto identify = [&identifies](const std::string &, int) {
    identifies.fetch_add(1);
    return IdentifyResult{};
  };

  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};
  cfg.ports = {4840};
  cfg.scan_concurrency = 1;

  NetworkDiscovery disc(cfg, scan, identify);
  const auto eps = disc.run([&stop]() {
    return stop.load();
  });
  EXPECT_TRUE(eps.empty());
  EXPECT_EQ(identifies.load(), 0);
}

// --------------------------------------------------------------------------- //
// select_auto_endpoint
// --------------------------------------------------------------------------- //
TEST(SelectAutoEndpoint, PicksAnonymousNoneDataServerSkipsLds) {
  auto scan = make_scan({"192.168.1.9:4840", "192.168.1.10:4840"});
  auto identify = make_identify({
      {"opc.tcp://192.168.1.9:4840", lds_identity()},
      {"opc.tcp://192.168.1.10:4840", siemens_identity()},
  });
  OpcuaDiscoveryConfig cfg;
  cfg.enabled = true;
  cfg.subnets = {"192.168.1.0/24"};
  cfg.ports = {4840};
  NetworkDiscovery disc(cfg, scan, identify);
  const auto eps = disc.run();

  const auto * chosen = NetworkDiscovery::select_auto_endpoint(eps, true);
  ASSERT_NE(chosen, nullptr);
  EXPECT_EQ(chosen->ip, "192.168.1.10");  // the Siemens, not the LDS
}

TEST(SelectAutoEndpoint, SecuredOnlySkippedWhenAnonymousRequired) {
  DiscoveredEndpoint secured;
  secured.ip = "192.168.1.30";
  secured.port = 4840;
  secured.protocol = "opcua";
  secured.application_type = 0;
  secured.anonymous_none_available = false;  // secured-only

  std::vector<DiscoveredEndpoint> eps = {secured};
  EXPECT_EQ(NetworkDiscovery::select_auto_endpoint(eps, true), nullptr);
  // With anonymous_none_only relaxed, the secured server becomes selectable.
  EXPECT_EQ(NetworkDiscovery::select_auto_endpoint(eps, false), &eps[0]);
}

TEST(SelectAutoEndpoint, LdsNeverSelectedEvenWhenAnonymousRelaxed) {
  DiscoveredEndpoint lds;
  lds.ip = "192.168.1.9";
  lds.port = 4840;
  lds.protocol = "opcua";
  lds.application_type = 3;  // LDS
  lds.anonymous_none_available = true;

  std::vector<DiscoveredEndpoint> eps = {lds};
  EXPECT_EQ(NetworkDiscovery::select_auto_endpoint(eps, false), nullptr);
}

TEST(SelectAutoEndpoint, DeterministicLowestAddressWins) {
  DiscoveredEndpoint a;
  a.ip = "192.168.1.20";
  a.port = 4840;
  a.protocol = "opcua";
  a.application_type = 0;
  a.anonymous_none_available = true;
  DiscoveredEndpoint b = a;
  b.ip = "192.168.1.10";

  std::vector<DiscoveredEndpoint> eps = {a, b};
  const auto * chosen = NetworkDiscovery::select_auto_endpoint(eps, true);
  ASSERT_NE(chosen, nullptr);
  EXPECT_EQ(chosen->ip, "192.168.1.10");
}

}  // namespace
}  // namespace ros2_medkit_gateway
