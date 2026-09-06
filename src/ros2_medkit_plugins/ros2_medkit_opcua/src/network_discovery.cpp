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

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/socket.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

namespace {

// Parse "a.b.c.d" into a host-order uint32. Returns false on any malformed
// octet (out of range, non-numeric, wrong field count).
bool parse_ipv4(const std::string & s, uint32_t * out) {
  struct in_addr addr {};
  if (inet_pton(AF_INET, s.c_str(), &addr) != 1) {
    return false;
  }
  *out = ntohl(addr.s_addr);
  return true;
}

std::string ipv4_to_string(uint32_t host_order) {
  struct in_addr addr {};
  addr.s_addr = htonl(host_order);
  char buf[INET_ADDRSTRLEN] = {0};
  inet_ntop(AF_INET, &addr, buf, sizeof(buf));
  return std::string(buf);
}

// Numeric IPv4 compare (e.g. "192.168.1.2" < "192.168.1.10") - a plain string
// compare gets this wrong lexicographically. Falls back to string compare
// only when a side fails to parse (should not happen: ip always comes from
// expand_cidr_hosts / a scanned socket address) so a malformed value still
// sorts deterministically instead of silently mis-ranking.
bool ipv4_less(const std::string & a, const std::string & b) {
  uint32_t a_ip = 0;
  uint32_t b_ip = 0;
  if (parse_ipv4(a, &a_ip) && parse_ipv4(b, &b_ip)) {
    return a_ip < b_ip;
  }
  return a < b;
}

// Bounded fan-out: run ``body(i)`` for every i in [0, count) across at most
// ``max_workers`` threads (never more than ``count``) that pull indices off a
// shared atomic cursor. The single primitive backs both the connect sweep and
// the GetEndpoints identify so they share the same concurrency bound.
//
// ``cancelled`` is polled by every worker before it takes the next index, so an
// abort stops the fan-out after at most one more probe per worker instead of
// running the remaining tens of thousands. An empty predicate never cancels.
template <typename Body>
void parallel_for(size_t count, int max_workers, const std::function<bool()> & cancelled, Body && body) {
  if (count == 0) {
    return;
  }
  const int worker_count = std::max(1, std::min<int>(max_workers, static_cast<int>(count)));
  std::atomic<size_t> next{0};
  const auto worker = [&]() {
    for (;;) {
      if (cancelled && cancelled()) {
        return;
      }
      const size_t i = next.fetch_add(1);
      if (i >= count) {
        return;
      }
      body(i);
    }
  };
  std::vector<std::thread> pool;
  pool.reserve(static_cast<size_t>(worker_count));
  for (int w = 0; w < worker_count; ++w) {
    pool.emplace_back(worker);
  }
  for (auto & th : pool) {
    th.join();
  }
}

}  // namespace

std::vector<std::string> expand_cidr_hosts(const std::string & cidr, bool * ok) {
  const auto fail = [&]() -> std::vector<std::string> {
    if (ok) {
      *ok = false;
    }
    return {};
  };

  const auto slash = cidr.find('/');
  if (slash == std::string::npos) {
    return fail();
  }
  const std::string ip_part = cidr.substr(0, slash);
  const std::string prefix_part = cidr.substr(slash + 1);

  uint32_t base = 0;
  if (!parse_ipv4(ip_part, &base)) {
    return fail();
  }

  int prefix = -1;
  try {
    size_t consumed = 0;
    prefix = std::stoi(prefix_part, &consumed);
    if (consumed != prefix_part.size()) {
      return fail();
    }
  } catch (const std::exception &) {
    return fail();
  }
  // Reject a prefix wider than /16 so a typo cannot trigger a multi-million
  // host Internet sweep. /32 and /31 are handled as degenerate single/paired
  // ranges below.
  if (prefix < 16 || prefix > 32) {
    return fail();
  }

  if (ok) {
    *ok = true;
  }

  const uint32_t host_bits = 32 - static_cast<uint32_t>(prefix);
  const uint32_t mask = host_bits == 32 ? 0u : (0xFFFFFFFFu << host_bits);
  const uint32_t network = base & mask;

  std::vector<std::string> hosts;
  if (prefix >= 31) {
    // /31 and /32: no network/broadcast reservation - every address is usable.
    const uint32_t count = host_bits == 0 ? 1u : (1u << host_bits);
    for (uint32_t i = 0; i < count; ++i) {
      hosts.push_back(ipv4_to_string(network + i));
    }
    return hosts;
  }

  const uint32_t broadcast = network | ~mask;
  hosts.reserve(broadcast - network - 1);
  for (uint32_t addr = network + 1; addr < broadcast; ++addr) {
    hosts.push_back(ipv4_to_string(addr));
  }
  return hosts;
}

std::string derive_local_cidr() {
  struct ifaddrs * ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) {
    return {};
  }
  std::string result;
  for (struct ifaddrs * ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    if ((ifa->ifa_flags & IFF_LOOPBACK) != 0 || (ifa->ifa_flags & IFF_UP) == 0) {
      continue;
    }
    const std::string name = ifa->ifa_name ? ifa->ifa_name : "";
    // Skip container / virtual bridges so the box scans the plant LAN, not a
    // Docker network. Best-effort by common name prefixes.
    if (name.rfind("docker", 0) == 0 || name.rfind("br-", 0) == 0 || name.rfind("veth", 0) == 0 ||
        name.rfind("virbr", 0) == 0) {
      continue;
    }
    const auto * sin = reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_addr);
    const uint32_t host = ntohl(sin->sin_addr.s_addr);
    const uint32_t network = host & 0xFFFFFF00u;  // /24
    result = ipv4_to_string(network) + "/24";
    break;
  }
  freeifaddrs(ifaddr);
  return result;
}

OpcuaDiscoveryConfig parse_discovery_config(const nlohmann::json & j,
                                            const std::function<void(const std::string &)> & warn) {
  OpcuaDiscoveryConfig cfg;
  const auto warn_fn = [&](const std::string & m) {
    if (warn) {
      warn(m);
    }
  };
  if (!j.is_object()) {
    warn_fn("discovery: config is not an object - ignoring");
    return cfg;
  }

  static const std::vector<std::string> kKnown = {"enabled",
                                                  "mode",
                                                  "subnets",
                                                  "cidr",
                                                  "ports",
                                                  "connect_timeout_ms",
                                                  "scan_concurrency",
                                                  "identify_timeout_ms",
                                                  "interval_s",
                                                  "anonymous_none_only"};
  for (const auto & item : j.items()) {
    if (std::find(kKnown.begin(), kKnown.end(), item.key()) == kKnown.end()) {
      warn_fn("discovery: unknown key '" + item.key() + "' ignored");
    }
  }

  if (j.contains("enabled") && j["enabled"].is_boolean()) {
    cfg.enabled = j["enabled"].get<bool>();
  }
  if (j.contains("mode") && j["mode"].is_string()) {
    const std::string mode = j["mode"].get<std::string>();
    if (mode == "active" || mode == "passive" || mode == "both") {
      cfg.mode = mode;
      if (mode == "passive") {
        warn_fn(
            "discovery: mode 'passive' - passive sources (mDNS / LDS) are not implemented yet; "
            "discovery will not run");
      } else if (mode == "both") {
        warn_fn(
            "discovery: mode 'both' - passive sources (mDNS / LDS) are not implemented yet; "
            "only the active scan runs");
      }
    } else {
      warn_fn("discovery: unknown mode '" + mode + "' - defaulting to 'active'");
    }
  }

  // subnets (array) and/or cidr (string or array) both feed the target list.
  const auto add_subnets = [&](const nlohmann::json & node) {
    if (node.is_string()) {
      cfg.subnets.push_back(node.get<std::string>());
    } else if (node.is_array()) {
      for (const auto & el : node) {
        if (el.is_string()) {
          cfg.subnets.push_back(el.get<std::string>());
        }
      }
    }
  };
  if (j.contains("subnets")) {
    add_subnets(j["subnets"]);
  }
  if (j.contains("cidr")) {
    add_subnets(j["cidr"]);
  }

  if (j.contains("ports") && j["ports"].is_array()) {
    std::vector<uint16_t> ports;
    for (const auto & p : j["ports"]) {
      if (!p.is_number_integer()) {
        continue;
      }
      const int64_t v = p.get<int64_t>();
      if (v < 1 || v > 65535) {
        warn_fn("discovery: port " + std::to_string(v) + " out of range 1..65535 - skipped");
        continue;
      }
      ports.push_back(static_cast<uint16_t>(v));
    }
    if (!ports.empty()) {
      cfg.ports = std::move(ports);
    }
  }

  const auto read_positive_int = [&](const char * key, int & target) {
    if (j.contains(key) && j[key].is_number_integer()) {
      const int v = j[key].get<int>();
      if (v > 0) {
        target = v;
      } else {
        warn_fn(std::string("discovery: ") + key + " must be > 0 - keeping default");
      }
    }
  };
  read_positive_int("connect_timeout_ms", cfg.connect_timeout_ms);
  read_positive_int("scan_concurrency", cfg.scan_concurrency);
  read_positive_int("identify_timeout_ms", cfg.identify_timeout_ms);

  // Clamp scan_concurrency: every worker is a live thread holding one open
  // socket, so an unbounded value would exhaust the fd / thread limits on a
  // wide subnet. kMaxScanConcurrency stays well under a typical 1024 fd ulimit.
  constexpr int kMaxScanConcurrency = 512;
  if (cfg.scan_concurrency > kMaxScanConcurrency) {
    warn_fn("discovery: scan_concurrency " + std::to_string(cfg.scan_concurrency) + " exceeds maximum " +
            std::to_string(kMaxScanConcurrency) + " - clamping");
    cfg.scan_concurrency = kMaxScanConcurrency;
  }

  if (j.contains("interval_s") && j["interval_s"].is_number_integer()) {
    const int v = j["interval_s"].get<int>();
    if (v >= 0) {
      // An explicit 0 is kept as an explicit 0: it means "keep discovery on but
      // never re-scan", which an unset interval (the built-in cadence) cannot
      // express.
      cfg.interval_s = v;
    } else {
      warn_fn("discovery: interval_s must be >= 0 (0 disables re-scanning) - keeping the default cadence");
    }
  }
  if (j.contains("anonymous_none_only") && j["anonymous_none_only"].is_boolean()) {
    cfg.anonymous_none_only = j["anonymous_none_only"].get<bool>();
  }

  return cfg;
}

NetworkDiscovery::NetworkDiscovery(OpcuaDiscoveryConfig cfg, PortScanFn scan, IdentifyFn identify)
  : cfg_(std::move(cfg)), scan_(std::move(scan)), identify_(std::move(identify)) {
}

std::vector<std::string> NetworkDiscovery::resolve_subnets() const {
  if (!cfg_.subnets.empty()) {
    return cfg_.subnets;
  }
  const std::string local = derive_local_cidr();
  if (local.empty()) {
    return {};
  }
  return {local};
}

std::vector<DiscoveredEndpoint> NetworkDiscovery::run(const std::function<bool()> & cancelled) {
  // "passive" has no active scan implementation yet (mDNS / LDS FindServers
  // are a documented follow-up) - a no-op stub rather than silently running
  // the active scan mode wasn't asked for. parse_discovery_config() already
  // warns about this at parse time.
  if (cfg_.mode == "passive") {
    return {};
  }

  // 1. Build the (ip, port) task list from every resolved subnet.
  struct Target {
    std::string ip;
    uint16_t port;
  };
  std::vector<Target> targets;
  for (const auto & cidr : resolve_subnets()) {
    bool ok = false;
    const auto hosts = expand_cidr_hosts(cidr, &ok);
    if (!ok) {
      continue;
    }
    for (const auto & ip : hosts) {
      for (const auto port : cfg_.ports) {
        targets.push_back({ip, port});
      }
    }
  }

  // 2. Bounded-concurrency TCP connect sweep. The scan primitive is injected;
  // only the polite fan-out lives here.
  std::vector<Target> open_hits;
  std::mutex hits_mu;
  parallel_for(targets.size(), cfg_.scan_concurrency, cancelled, [&](size_t i) {
    const Target & t = targets[i];
    if (scan_(t.ip, t.port, cfg_.connect_timeout_ms)) {
      std::lock_guard<std::mutex> lk(hits_mu);
      open_hits.push_back(t);
    }
  });

  // Cancelled mid-sweep: the hit list is partial, so do not spend an identify
  // round-trip per hit on a pass whose caller is shutting down.
  if (cancelled && cancelled()) {
    return {};
  }

  // Deterministic identify order (numerically lowest ip:port first).
  std::sort(open_hits.begin(), open_hits.end(), [](const Target & a, const Target & b) {
    if (a.ip != b.ip) {
      return ipv4_less(a.ip, b.ip);
    }
    return a.port < b.port;
  });

  // 3. Identify OPC-UA hits. Each GetEndpoints call blocks up to
  // identify_timeout_ms, so a sequential loop would take N * timeout on a
  // subnet where several hosts accept TCP on 4840 but never complete the
  // handshake. Fan the identify out over the same bounded pool as the connect
  // sweep, writing each result by index so the deterministic ip:port order
  // (open_hits was sorted above) survives regardless of completion order.
  std::vector<DiscoveredEndpoint> built(open_hits.size());
  parallel_for(open_hits.size(), cfg_.scan_concurrency, cancelled, [&](size_t i) {
    const Target & t = open_hits[i];
    DiscoveredEndpoint ep;
    ep.ip = t.ip;
    ep.port = t.port;
    ep.endpoint_url = "opc.tcp://" + t.ip + ":" + std::to_string(t.port);

    if (t.port == 4840) {
      const IdentifyResult id = identify_(ep.endpoint_url, cfg_.identify_timeout_ms);
      if (id.ok) {
        ep.advertised_url = id.advertised_url;
        ep.application_uri = id.application_uri;
        ep.product_uri = id.product_uri;
        ep.application_name = id.application_name;
        ep.application_type = id.application_type;
        ep.security_policies = id.security_policies;
        ep.anonymous_none_available = id.anonymous_none_available;
      } else {
        ep.identify_error = id.error;
      }
    } else {
      // Non-OPC-UA OT port: recorded as a lead, no identify.
      ep.protocol = "unknown";
    }
    built[i] = std::move(ep);
  });

  // Cancelled during the identify phase: ``built`` holds default-constructed
  // (empty) entries for the hits no worker reached, which is not a result set.
  if (cancelled && cancelled()) {
    return {};
  }

  // 4. Dedup by ApplicationUri (fallback ip:port), sequentially over the
  // deterministic order so the lowest ip:port always wins.
  std::vector<DiscoveredEndpoint> ordered;
  std::unordered_map<std::string, size_t> by_key;  // dedup key -> index in ordered
  for (auto & ep : built) {
    // Dedup: prefer a stable ApplicationUri; a scan on two IPs of the same PLC
    // then collapses to one entry. Fall back to ip:port when the URI is empty
    // (identify failed or non-OPC-UA lead) so distinct hosts stay distinct.
    const std::string key = !ep.application_uri.empty() ? ("uri:" + ep.application_uri) : ("addr:" + ep.endpoint_url);
    if (by_key.find(key) == by_key.end()) {
      by_key.emplace(key, ordered.size());
      ordered.push_back(std::move(ep));
    }
    // On a duplicate ApplicationUri we keep the first (lowest ip:port by sort
    // order) and drop the rest - the primary connect URL should be stable.
  }

  return ordered;
}

const DiscoveredEndpoint * NetworkDiscovery::select_auto_endpoint(const std::vector<DiscoveredEndpoint> & eps,
                                                                  bool anonymous_none_only) {
  const DiscoveredEndpoint * best = nullptr;
  for (const auto & ep : eps) {
    if (ep.protocol != "opcua" || !ep.identify_error.empty()) {
      continue;
    }
    if (!ep.is_data_server()) {
      continue;  // skip a DiscoveryServer / LDS - nothing to browse
    }
    if (anonymous_none_only && !ep.anonymous_none_available) {
      continue;  // secured-only: surfaced as a lead, never auto-connected
    }
    if (best == nullptr || ipv4_less(ep.ip, best->ip) || (ep.ip == best->ip && ep.port < best->port)) {
      best = &ep;
    }
  }
  return best;
}

}  // namespace ros2_medkit_gateway
