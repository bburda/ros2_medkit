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

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {

/// One SecurityPolicy / MessageSecurityMode pair offered by a discovered
/// endpoint. ``mode`` mirrors the OPC-UA MessageSecurityMode enum
/// (1 = None, 2 = Sign, 3 = SignAndEncrypt).
struct DiscoverySecurityProfile {
  std::string policy;  ///< short policy name, e.g. "None", "Basic256Sha256"
  int mode{0};
};

/// Result of a read-only OPC-UA GetEndpoints identify on a single server. Plain
/// struct (no open62541 types) so the discovery orchestrator and its unit tests
/// are decoupled from the OPC-UA client library.
struct IdentifyResult {
  bool ok{false};
  std::string error;  ///< populated when ok == false

  std::string advertised_url;   ///< EndpointUrl the server advertised (may be unresolvable)
  std::string application_uri;  ///< server ApplicationUri (stable identity)
  std::string product_uri;
  std::string application_name;
  /// OPC-UA ApplicationType: 0 = Server, 1 = Client, 2 = ClientAndServer,
  /// 3 = DiscoveryServer (LDS).
  int application_type{0};
  std::vector<DiscoverySecurityProfile> security_policies;
  bool anonymous_none_available{false};  ///< a None + Anonymous endpoint is offered
};

/// A discovered OPC-UA endpoint, enriched with identify metadata.
struct DiscoveredEndpoint {
  std::string ip;
  uint16_t port{4840};
  std::string protocol = "opcua";

  /// Connect URL the plugin should use. Always the SCANNED ``ip:port`` because
  /// a server can advertise a non-resolvable hostname (an LDS on our LAN
  /// advertised ``opc.tcp://DESKTOP-...``); the advertised value is kept only
  /// as metadata in ``advertised_url``.
  std::string endpoint_url;
  std::string advertised_url;

  std::string application_uri;
  std::string product_uri;
  std::string application_name;
  int application_type{0};
  std::vector<DiscoverySecurityProfile> security_policies;
  bool anonymous_none_available{false};

  /// Non-empty when the GetEndpoints identify failed (port was open but the
  /// OPC-UA handshake did not complete). Such an endpoint is surfaced as a lead
  /// but is never auto-connected.
  std::string identify_error;

  /// True when the endpoint is an OPC-UA data server (ApplicationType Server or
  /// ClientAndServer) - a candidate for auto_browse. A DiscoveryServer (LDS,
  /// type 3) returns false and is never browsed.
  bool is_data_server() const {
    return application_type == 0 || application_type == 2;
  }
};

/// ``discovery:`` config block for the opcua plugin. Off by default; active
/// scan is opt-in and bounded.
struct OpcuaDiscoveryConfig {
  bool enabled{false};

  /// active | passive | both. Only "active" (bounded TCP scan + GetEndpoints)
  /// is implemented today; passive mDNS / LDS FindServers are a documented
  /// follow-up. A passive-only mode performs no scanning and finds nothing on a
  /// stock Siemens network.
  std::string mode = "active";

  /// Target CIDR subnets for the active scan. Empty => derive the plugin host's
  /// local /24 at runtime.
  std::vector<std::string> subnets;

  /// TCP ports probed per host. 4840 is the OPC-UA default; others are recorded
  /// as leads but not OPC-UA-identified.
  std::vector<uint16_t> ports{4840};

  int connect_timeout_ms{600};  ///< per-port TCP connect timeout
  int scan_concurrency{100};    ///< bounded, polite concurrent connect count
  int identify_timeout_ms{6000};

  /// Re-scan cadence, in seconds, while no OPC-UA session is established. 0
  /// selects the built-in default (see OpcuaPlugin::effective_rescan_interval_s).
  /// The startup scan always runs once. The cadence only governs how often the
  /// disconnected reconnect loop scans again, so a gateway that started before
  /// its PLC finished booting adopts the PLC when it appears instead of retrying
  /// the fallback endpoint forever. Never used once an endpoint is configured
  /// explicitly, and never while a session is up.
  int interval_s{0};

  /// Only auto-register endpoints that expose a None + Anonymous endpoint (what
  /// the plugin connects with today). Secured-only servers are surfaced as
  /// leads requiring operator credentials, never auto-connected.
  bool anonymous_none_only{true};
};

/// Probe whether a TCP port is open. Injected so the orchestrator is unit
/// testable without a network.
/// @return true if a connection was established within ``timeout_ms``.
using PortScanFn = std::function<bool(const std::string & ip, uint16_t port, int timeout_ms)>;

/// Read-only OPC-UA GetEndpoints identify against ``opc.tcp://ip:port``.
using IdentifyFn = std::function<IdentifyResult(const std::string & url, int timeout_ms)>;

/// Expand a CIDR (e.g. "192.168.1.0/24") into its usable host IP strings
/// (network + broadcast excluded for prefixes < 31). Returns empty and sets
/// ``*ok = false`` on a malformed CIDR or a prefix wider than /16 (guards
/// against an accidental full-Internet sweep).
std::vector<std::string> expand_cidr_hosts(const std::string & cidr, bool * ok = nullptr);

/// Best-effort local /24 derived from the host's first non-loopback,
/// non-Docker IPv4 interface. Empty when none is found.
std::string derive_local_cidr();

/// Parse + validate a ``discovery:`` JSON block. Unknown keys and out-of-range
/// values are reported through ``warn`` (never fatal); recognized values
/// override the defaults. Ports outside 1..65535, non-positive timeout /
/// concurrency, and an unknown ``mode`` are rejected back to their defaults
/// with a warning.
OpcuaDiscoveryConfig parse_discovery_config(const nlohmann::json & j,
                                            const std::function<void(const std::string &)> & warn);

/// Read-only active-scan discovery orchestrator. Sweeps the configured subnets
/// for open OPC-UA ports, runs a GetEndpoints identify on each :4840 hit, and
/// returns a deduplicated, classified endpoint list. All I/O is injected
/// (scan + identify) so the merge / dedup / URL-preference logic is unit
/// testable without touching the network.
class NetworkDiscovery {
 public:
  NetworkDiscovery(OpcuaDiscoveryConfig cfg, PortScanFn scan, IdentifyFn identify);

  /// Run one full discovery pass (blocking). Read-only: TCP connect +
  /// GetEndpoints only. Deduplicated by ApplicationUri (fallback ip:port),
  /// sorted deterministically by ip:port.
  std::vector<DiscoveredEndpoint> run();

  /// Resolve the subnets to scan: configured ``subnets`` if any, else the
  /// derived local /24. Exposed for logging / tests.
  std::vector<std::string> resolve_subnets() const;

  /// Pick the best endpoint for single-endpoint "auto endpoint" mode: an
  /// OPC-UA data server (not an LDS) that identified cleanly and, when
  /// ``anonymous_none_only``, offers a None + Anonymous endpoint. Deterministic
  /// (lowest ip:port). Returns nullptr when no candidate qualifies. Pure /
  /// static so the selection policy is unit tested without a network.
  static const DiscoveredEndpoint * select_auto_endpoint(const std::vector<DiscoveredEndpoint> & eps,
                                                         bool anonymous_none_only);

 private:
  OpcuaDiscoveryConfig cfg_;
  PortScanFn scan_;
  IdentifyFn identify_;
};

/// Real POSIX TCP connect probe (non-blocking connect + select timeout).
bool tcp_connect_probe(const std::string & ip, uint16_t port, int timeout_ms);

/// Real read-only OPC-UA GetEndpoints identify via open62541pp.
IdentifyResult opcua_getendpoints_identify(const std::string & url, int timeout_ms);

}  // namespace ros2_medkit_gateway
