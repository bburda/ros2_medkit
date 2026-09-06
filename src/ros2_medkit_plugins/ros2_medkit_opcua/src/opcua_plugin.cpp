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

#include "ros2_medkit_opcua/opcua_plugin.hpp"

#include "ros2_medkit_opcua/device_identity.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <ros2_medkit_msgs/msg/fault.hpp>
#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include <ros2_medkit_gateway/core/http/error_codes.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <stdexcept>

namespace ros2_medkit_gateway {

namespace {

// Named logger so per-operation traces respect ROS log level filtering
// (bburda review on PR #387). Quiet at INFO; ``--log-level
// opcua.plugin:=debug`` re-enables for diagnostics.
inline rclcpp::Logger opcua_plugin_logger() {
  static auto logger = rclcpp::get_logger("opcua.plugin");
  return logger;
}

inline bool plugin_debug_enabled() {
  // rcutils API for Humble compatibility; Jazzy+ adds rclcpp::Logger::
  // get_effective_level but Humble does not.
  return rcutils_logging_logger_is_enabled_for("opcua.plugin", RCUTILS_LOG_SEVERITY_DEBUG);
}

// Security-config parse wrappers. An unrecognized value must NOT silently fall
// back to the insecure default (SecurityPolicy::None / SecurityMode::None /
// anonymous auth): a typo would quietly downgrade a hardened link. Fail loud and
// refuse - the thrown exception disables the plugin in PluginManager::configure_plugins.
SecurityPolicy require_security_policy(const std::string & value) {
  bool ok = true;
  const auto parsed = OpcuaClient::parse_security_policy(value, &ok);
  if (!ok) {
    RCLCPP_ERROR(opcua_plugin_logger(),
                 "Unrecognized OPC-UA security_policy '%s'; refusing to start with an insecure fallback",
                 value.c_str());
    throw std::invalid_argument("opcua: unrecognized security_policy '" + value + "'");
  }
  return parsed;
}

SecurityMode require_security_mode(const std::string & value) {
  bool ok = true;
  const auto parsed = OpcuaClient::parse_security_mode(value, &ok);
  if (!ok) {
    RCLCPP_ERROR(opcua_plugin_logger(),
                 "Unrecognized OPC-UA security_mode '%s'; refusing to start with an insecure fallback", value.c_str());
    throw std::invalid_argument("opcua: unrecognized security_mode '" + value + "'");
  }
  return parsed;
}

UserAuthMode require_user_auth_mode(const std::string & value) {
  bool ok = true;
  const auto parsed = OpcuaClient::parse_user_auth_mode(value, &ok);
  if (!ok) {
    RCLCPP_ERROR(opcua_plugin_logger(),
                 "Unrecognized OPC-UA user_auth_mode '%s'; refusing to start with an insecure fallback", value.c_str());
    throw std::invalid_argument("opcua: unrecognized user_auth_mode '" + value + "'");
  }
  return parsed;
}

/// Parse a JSON "value" field, coerce to the node's declared data_type, and
/// validate against the optional min/max range. Shared by handle_plc_operations,
/// DataProvider::write_data, and OperationProvider::execute_operation to keep
/// the three write paths in sync.
tl::expected<OpcuaValue, std::string> parse_coerce_validate(const nlohmann::json & json_value,
                                                            const NodeMapEntry & entry) {
  OpcuaValue val;
  try {
    if (entry.data_type == "bool") {
      val = json_value.get<bool>();
    } else if (entry.data_type == "int") {
      val = json_value.get<int32_t>();
    } else if (entry.data_type == "string") {
      val = json_value.get<std::string>();
    } else {
      val = json_value.get<double>();
    }
  } catch (const nlohmann::json::type_error &) {
    return tl::make_unexpected("Value type mismatch for data_type: " + entry.data_type);
  }

  if (entry.has_range()) {
    double v = std::visit(
        [](auto && x) -> double {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_arithmetic_v<T>) {
            return static_cast<double>(x);
          }
          return 0.0;
        },
        val);
    if (v < *entry.min_value || v > *entry.max_value) {
      return tl::make_unexpected("Value " + std::to_string(v) + " out of range [" + std::to_string(*entry.min_value) +
                                 ", " + std::to_string(*entry.max_value) + "]");
    }
  }

  return val;
}

bool is_valid_path_segment(const std::string & s) {
  if (s.empty() || s.size() > 256) {
    return false;
  }
  return std::all_of(s.begin(), s.end(), [](unsigned char c) {
    return std::isalnum(c) || c == '_' || c == '-';
  });
}

// Parse a JSON array of namespace indices for auto_browse's
// namespace_allow/namespace_deny knobs. Out-of-range or non-integer entries
// are skipped with a warning rather than aborting the whole config.
std::vector<uint16_t> parse_json_ns_list(const nlohmann::json & arr, const char * field,
                                         const std::function<void(const std::string &)> & warn) {
  std::vector<uint16_t> out;
  if (!arr.is_array()) {
    warn(std::string("plugins.opcua.auto_browse.") + field + " must be an array - ignoring");
    return out;
  }
  for (const auto & v : arr) {
    if (!v.is_number_integer()) {
      warn(std::string("plugins.opcua.auto_browse.") + field + ": non-integer namespace index - skipping");
      continue;
    }
    const auto n = v.get<int64_t>();
    if (n < 0 || n > 65535) {
      warn(std::string("plugins.opcua.auto_browse.") + field + ": namespace index out of range [0,65535] - skipping");
      continue;
    }
    out.push_back(static_cast<uint16_t>(n));
  }
  return out;
}
}  // namespace

struct OpcuaPlugin::FaultClients {
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report;
  rclcpp::Client<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear;
};

OpcuaPlugin::OpcuaPlugin()
  : client_(std::make_unique<OpcuaClient>()), fault_clients_(std::make_unique<FaultClients>()) {
}

OpcuaPlugin::~OpcuaPlugin() {
  shutdown();
}

void OpcuaPlugin::configure(const nlohmann::json & config) {
  if (config.contains("endpoint_url")) {
    const std::string endpoint_url = config["endpoint_url"].get<std::string>();
    if (!endpoint_url.empty()) {
      client_config_.endpoint_url = endpoint_url;
      endpoint_configured_ = true;
    }
  }

  // OPC-UA SecureChannel security + user identity. All opt-in; defaults keep
  // the legacy anonymous + SecurityPolicy=None behaviour.
  if (config.contains("security_policy")) {
    client_config_.security_policy = require_security_policy(config["security_policy"].get<std::string>());
  }
  if (config.contains("security_mode") || config.contains("message_security_mode")) {
    const std::string mode_str = config.contains("security_mode") ? config["security_mode"].get<std::string>()
                                                                  : config["message_security_mode"].get<std::string>();
    client_config_.security_mode = require_security_mode(mode_str);
  }
  if (config.contains("client_cert_path")) {
    client_config_.client_cert_path = config["client_cert_path"].get<std::string>();
  }
  if (config.contains("client_key_path")) {
    client_config_.client_key_path = config["client_key_path"].get<std::string>();
  }
  if (config.contains("application_uri")) {
    client_config_.application_uri = config["application_uri"].get<std::string>();
  }
  if (config.contains("trust_list_paths") && config["trust_list_paths"].is_array()) {
    client_config_.trust_list_paths.clear();
    for (const auto & p : config["trust_list_paths"]) {
      if (p.is_string() && !p.get<std::string>().empty()) {
        client_config_.trust_list_paths.push_back(p.get<std::string>());
      }
    }
  }
  if (config.contains("reject_untrusted")) {
    client_config_.reject_untrusted = config["reject_untrusted"].get<bool>();
  }
  if (config.contains("user_auth_mode")) {
    client_config_.user_auth_mode = require_user_auth_mode(config["user_auth_mode"].get<std::string>());
  }
  if (config.contains("username")) {
    client_config_.username = config["username"].get<std::string>();
  }
  if (config.contains("password")) {
    client_config_.password = config["password"].get<std::string>();
  }
  if (config.contains("user_cert_path")) {
    client_config_.user_cert_path = config["user_cert_path"].get<std::string>();
  }

  if (config.contains("node_map_path")) {
    node_map_path_ = config["node_map_path"].get<std::string>();
  }

  if (config.contains("prefer_subscriptions")) {
    poller_config_.prefer_subscriptions = config["prefer_subscriptions"].get<bool>();
  }
  if (config.contains("subscription_interval_ms")) {
    poller_config_.subscription_interval_ms = config["subscription_interval_ms"].get<double>();
  }
  if (config.contains("poll_interval_ms")) {
    auto ms = std::clamp(config["poll_interval_ms"].get<int>(), 100, 60000);
    poller_config_.poll_interval = std::chrono::milliseconds(ms);
  }
  if (config.contains("condition_replay_strategy")) {
    poller_config_.condition_replay_strategy =
        OpcuaPoller::parse_replay_strategy(config["condition_replay_strategy"].get<std::string>());
  }
  if (auto * env = std::getenv("OPCUA_CONDITION_REPLAY")) {
    poller_config_.condition_replay_strategy = OpcuaPoller::parse_replay_strategy(env);
  }
  // Issue #478: drop the Confirm gate for servers that do not implement the
  // optional Confirm transition (e.g. Siemens S7-1500) so alarms clear on
  // Acknowledge alone. Default true keeps the spec-strict behaviour.
  if (config.contains("require_confirm_for_clear")) {
    poller_config_.require_confirm_for_clear = config["require_confirm_for_clear"].get<bool>();
  }
  if (auto * env = std::getenv("OPCUA_REQUIRE_CONFIRM_FOR_CLEAR")) {
    const std::string v = env;
    poller_config_.require_confirm_for_clear = !(v == "0" || v == "false" || v == "no" || v == "off");
  }

  // Issue #496: comms-lost fault knobs.
  if (config.contains("comms_lost_fault_enabled")) {
    poller_config_.comms_lost_fault_enabled = config["comms_lost_fault_enabled"].get<bool>();
  }
  if (auto * env = std::getenv("OPCUA_COMMS_LOST_ENABLED")) {
    const std::string v = env;
    poller_config_.comms_lost_fault_enabled = !(v == "0" || v == "false" || v == "no" || v == "off");
  }
  // Clamp the comms-lost debounce to [0, 1h], shared by the YAML and env paths.
  // Without an upper bound an overflowing or huge value pushes the gate so far
  // out it never fires, silently disabling the fault; 0 keeps the immediate
  // report behaviour.
  constexpr long kMaxCommsLostDebounceMs = 3600000;  // 1 hour
  const auto clamp_debounce_ms = [](long long ms) -> long {
    if (ms < 0) {
      return 0;
    }
    if (ms > kMaxCommsLostDebounceMs) {
      return kMaxCommsLostDebounceMs;
    }
    return static_cast<long>(ms);
  };
  if (config.contains("comms_lost_debounce_ms")) {
    const long long ms = config["comms_lost_debounce_ms"].get<long long>();
    poller_config_.comms_lost_debounce = std::chrono::milliseconds(clamp_debounce_ms(ms));
  }
  if (auto * env = std::getenv("OPCUA_COMMS_LOST_DEBOUNCE_MS")) {
    // Keep the existing value on a non-numeric / out-of-range / negative
    // override so a typo does not silently disable the debounce (atoi would
    // parse it as 0). errno + ERANGE rejects an overflowing run of digits that
    // strtol would otherwise saturate to LONG_MAX and slip past the >= 0 guard.
    errno = 0;
    char * end = nullptr;
    const long ms = std::strtol(env, &end, 10);
    if (end != env && *end == '\0' && errno != ERANGE && ms >= 0) {
      poller_config_.comms_lost_debounce = std::chrono::milliseconds(clamp_debounce_ms(ms));
    } else {
      log_warn(std::string("Ignoring invalid OPCUA_COMMS_LOST_DEBOUNCE_MS='") + env +
               "' (want a non-negative integer <= 3600000 ms)");
    }
  }
  if (config.contains("comms_lost_severity")) {
    poller_config_.comms_lost_severity = config["comms_lost_severity"].get<std::string>();
  }
  {
    // Validate against the SOVD severity buckets and default to ERROR on a typo,
    // so the comms-lost fault is not silently downgraded to INFO by the severity
    // map (same guard as the node-map severity validation).
    const std::string & s = poller_config_.comms_lost_severity;
    if (s != "INFO" && s != "WARNING" && s != "ERROR" && s != "CRITICAL") {
      log_warn("Unknown comms_lost_severity '" + s + "' - defaulting to ERROR");
      poller_config_.comms_lost_severity = "ERROR";
    }
  }

  // Environment variables override YAML config (for Docker)
  if (auto * env = std::getenv("OPCUA_ENDPOINT_URL"); env != nullptr && *env != '\0') {
    client_config_.endpoint_url = env;
    endpoint_configured_ = true;
  }
  if (auto * env = std::getenv("OPCUA_NODE_MAP_PATH")) {
    node_map_path_ = env;
  }
  // Security env overrides (for Docker / appliance deployments).
  if (auto * env = std::getenv("OPCUA_SECURITY_POLICY")) {
    client_config_.security_policy = require_security_policy(env);
  }
  if (auto * env = std::getenv("OPCUA_SECURITY_MODE")) {
    client_config_.security_mode = require_security_mode(env);
  }
  if (auto * env = std::getenv("OPCUA_CLIENT_CERT")) {
    client_config_.client_cert_path = env;
  }
  if (auto * env = std::getenv("OPCUA_CLIENT_KEY")) {
    client_config_.client_key_path = env;
  }
  if (auto * env = std::getenv("OPCUA_APPLICATION_URI")) {
    client_config_.application_uri = env;
  }
  if (auto * env = std::getenv("OPCUA_TRUST_LIST")) {
    // Colon-separated list of DER trust-store paths.
    client_config_.trust_list_paths.clear();
    std::string list = env;
    size_t start = 0;
    while (start <= list.size()) {
      size_t sep = list.find(':', start);
      std::string path = list.substr(start, sep == std::string::npos ? std::string::npos : sep - start);
      if (!path.empty()) {
        client_config_.trust_list_paths.push_back(path);
      }
      if (sep == std::string::npos) {
        break;
      }
      start = sep + 1;
    }
  }
  if (auto * env = std::getenv("OPCUA_REJECT_UNTRUSTED")) {
    std::string v = env;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    client_config_.reject_untrusted = !(v == "0" || v == "false" || v == "no");
  }
  if (auto * env = std::getenv("OPCUA_USER_AUTH")) {
    client_config_.user_auth_mode = require_user_auth_mode(env);
  }
  if (auto * env = std::getenv("OPCUA_USERNAME")) {
    client_config_.username = env;
  }
  if (auto * env = std::getenv("OPCUA_PASSWORD")) {
    client_config_.password = env;
  }
  if (auto * env = std::getenv("OPCUA_USER_CERT")) {
    client_config_.user_cert_path = env;
  }

  // Read-only PLC/OPC-UA network discovery (opt-in). Parsed + validated with
  // unknown-key warnings; env overrides follow for Docker/appliance deploys.
  if (config.contains("discovery")) {
    discovery_config_ = parse_discovery_config(config["discovery"], [this](const std::string & m) {
      log_warn(m);
    });
  }
  if (auto * env = std::getenv("OPCUA_DISCOVERY_ENABLED")) {
    const std::string v = env;
    discovery_config_.enabled = !(v == "0" || v == "false" || v == "no" || v == "off");
  }
  if (auto * env = std::getenv("OPCUA_DISCOVERY_SUBNETS")) {
    // Comma-separated CIDR list.
    discovery_config_.subnets.clear();
    std::string list = env;
    size_t start = 0;
    while (start <= list.size()) {
      const size_t sep = list.find(',', start);
      std::string cidr = list.substr(start, sep == std::string::npos ? std::string::npos : sep - start);
      if (!cidr.empty()) {
        discovery_config_.subnets.push_back(cidr);
      }
      if (sep == std::string::npos) {
        break;
      }
      start = sep + 1;
    }
  }
  if (auto * env = std::getenv("OPCUA_DISCOVERY_INTERVAL_S")) {
    errno = 0;
    char * end = nullptr;
    const long v = std::strtol(env, &end, 10);
    if (end != env && *end == '\0' && errno != ERANGE && v >= 0) {
      discovery_config_.interval_s = static_cast<int>(v);
    } else {
      log_warn(std::string("Ignoring invalid OPCUA_DISCOVERY_INTERVAL_S='") + env + "' (want a non-negative integer)");
    }
  }

  // Default the discovery I/O to the real probes; test builds override these
  // before set_context() to exercise the auto-endpoint path offline.
  if (!discovery_scan_fn_) {
    discovery_scan_fn_ = &tcp_connect_probe;
  }
  if (!discovery_identify_fn_) {
    discovery_identify_fn_ = &opcua_getendpoints_identify;
  }

  if (!node_map_path_.empty()) {
    if (!node_map_.load(node_map_path_)) {
      // FATAL: a node map that fails to load or validate (duplicate/colliding
      // fault codes, conflicting detection modes, oversized file, ...) must
      // NOT run. The global fault-code uniqueness that prevents raise/clear
      // flapping is only enforced at load, so starting the poller against a
      // rejected map would defeat it. Throw so PluginManager disables the
      // plugin instead of proceeding to set_context()/starting the poller.
      log_error("Failed to load node map from: " + node_map_path_);
      throw std::runtime_error("OPC-UA node map failed to load or validate: " + node_map_path_ +
                               " (see preceding log line for the specific rejection)");
    }
    log_info("Loaded node map with " + std::to_string(node_map_.entries().size()) + " entries from: " + node_map_path_);
  }

  // plugins.opcua.auto_browse (ROS param / JSON config). Overlays on top of
  // whatever the node-map YAML's `auto_browse:` block set (or the all-default
  // AutoBrowseConfig when there is no node map at all) - the same
  // "JSON/env wins over file" precedence the rest of this function uses.
  // This is what makes zero-config discovery possible: no node_map_path,
  // just `endpoint_url` (from discovery, PR #509) + `auto_browse: true`.
  if (config.contains("auto_browse")) {
    auto warn = [this](const std::string & msg) {
      log_warn(msg);
    };
    auto & ab_cfg = node_map_.mutable_auto_browse_config();
    const auto & ab = config["auto_browse"];
    if (ab.is_boolean()) {
      ab_cfg.enabled = ab.get<bool>();
    } else if (ab.is_object()) {
      ab_cfg.enabled = ab.value("enabled", true);
      if (ab.contains("root_nodes")) {
        if (ab["root_nodes"].is_array()) {
          ab_cfg.root_node_ids.clear();
          for (const auto & r : ab["root_nodes"]) {
            if (r.is_string()) {
              ab_cfg.root_node_ids.push_back(r.get<std::string>());
            } else {
              warn("plugins.opcua.auto_browse.root_nodes: non-string entry - skipping");
            }
          }
        } else {
          warn("plugins.opcua.auto_browse.root_nodes must be an array - ignoring");
        }
      }
      if (ab.contains("max_depth")) {
        if (!ab["max_depth"].is_number_integer()) {
          warn("plugins.opcua.auto_browse.max_depth must be an integer - keeping current value");
        } else {
          const auto d = ab["max_depth"].get<int64_t>();
          if (d >= 1 && d <= 64) {
            ab_cfg.max_depth = static_cast<int>(d);
          } else {
            warn("plugins.opcua.auto_browse.max_depth out of range [1,64] - keeping current value");
          }
        }
      }
      if (ab.contains("max_nodes")) {
        if (!ab["max_nodes"].is_number_integer()) {
          warn("plugins.opcua.auto_browse.max_nodes must be an integer - keeping current value");
        } else {
          const auto n = ab["max_nodes"].get<int64_t>();
          if (n >= 1 && n <= 200000) {
            ab_cfg.max_nodes = static_cast<size_t>(n);
          } else {
            warn("plugins.opcua.auto_browse.max_nodes out of range [1,200000] - keeping current value");
          }
        }
      }
      if (ab.contains("namespace_allow")) {
        ab_cfg.namespace_allow = parse_json_ns_list(ab["namespace_allow"], "namespace_allow", warn);
      }
      if (ab.contains("namespace_deny")) {
        ab_cfg.namespace_deny = parse_json_ns_list(ab["namespace_deny"], "namespace_deny", warn);
      }
      if (ab.contains("read_initial_values")) {
        if (ab["read_initial_values"].is_boolean()) {
          ab_cfg.read_initial_values = ab["read_initial_values"].get<bool>();
        } else {
          warn("plugins.opcua.auto_browse.read_initial_values must be a boolean - keeping current value");
        }
      }
      if (ab.contains("infer_writable")) {
        if (ab["infer_writable"].is_boolean()) {
          ab_cfg.infer_writable = ab["infer_writable"].get<bool>();
        } else {
          warn("plugins.opcua.auto_browse.infer_writable must be a boolean - keeping current value");
        }
      }
      static const std::array<const char *, 6> kKnownKeys{"enabled",   "root_nodes",      "max_depth",
                                                          "max_nodes", "namespace_allow", "namespace_deny"};
      for (const auto & [key, value] : ab.items()) {
        (void)value;
        if (key == "read_initial_values" || key == "infer_writable") {
          continue;
        }
        if (std::find_if(kKnownKeys.begin(), kKnownKeys.end(), [&key](const char * k) {
              return key == k;
            }) == kKnownKeys.end()) {
          warn("plugins.opcua.auto_browse: unknown key '" + key + "' - ignored");
        }
      }
    } else {
      log_warn("plugins.opcua.auto_browse must be a boolean or an object - ignoring");
    }
  }

  // plugins.opcua.auto_alarms (ROS param / JSON config). Overlays on top of
  // whatever the node-map YAML's ``auto_alarms:`` block set (or the default
  // AutoAlarmsConfig when there is no node map at all) - the same "JSON/env
  // wins over file" precedence the rest of this function uses. This is what
  // makes zero-config native A&C possible with no node_map_path: just an
  // ``endpoint_url`` (from discovery) + ``auto_alarms: true``. Explicit
  // ``event_alarms:`` mappings still take precedence over auto-derivation at
  // the poller (see OpcuaPoller::effective_alarm_sources) regardless.
  if (config.contains("auto_alarms")) {
    auto_alarms_configured_ = true;
    apply_auto_alarms_param(config["auto_alarms"], node_map_.mutable_auto_alarms(), [this](const std::string & msg) {
      log_warn(msg);
    });
    // Re-derive the fields load() fills in (default entity, parsed source
    // NodeId) and rebuild entity_defs so a param-only deployment still
    // surfaces the alarms App over SOVD. Non-fatal on an invalid overlay:
    // finalize_auto_alarms_overlay disables auto_alarms and logs rather than
    // taking the whole plugin down over one bad param.
    node_map_.finalize_auto_alarms_overlay();
    if (node_map_.auto_alarms().enabled) {
      log_info("auto_alarms enabled via plugins.opcua.auto_alarms param (source " +
               node_map_.auto_alarms().source_node_id_str + ", entity " + node_map_.auto_alarms().entity_id + ")");
    }
  }
}

void OpcuaPlugin::set_context(PluginContext & context) {
  ctx_ = as_ros_plugin_context(context);

  // NOTE: capabilities (x-plc-data, x-plc-operations, x-plc-status) are
  // registered per entity in introspect() rather than type-level here, so
  // that only PLC-backed entities advertise them. Type-level registration
  // would leak the capabilities onto every ROS 2 app/component in the SOVD
  // entity tree, misleading clients into calling endpoints that return 404.

  auto * node = ctx_->node();
  if (node) {
    fault_clients_->report = node->create_client<ros2_medkit_msgs::srv::ReportFault>("/fault_manager/report_fault");
    fault_clients_->clear = node->create_client<ros2_medkit_msgs::srv::ClearFault>("/fault_manager/clear_fault");
  }

  run_startup_discovery();

  log_security_profile();

  const bool connected = client_->connect(client_config_);
  if (connected) {
    log_info("Connected to OPC-UA server: " + client_config_.endpoint_url);
    clear_comms_lost_on_connect();
  } else {
    log_warn("Failed to connect to OPC-UA server: " + client_config_.endpoint_url);
  }

  // Config-less discovery: name the SOVD component from the device's OWN read
  // identity, never a hardcoded product string. DI nameplate (Siemens AG /
  // CPU 1505SP F) wins, else BuildInfo ApplicationName/ProductName, else the
  // neutral endpoint fallback opcua-<host>. Runs before the node map is
  // consumed (auto_alarms finalize + auto_browse below both rebuild
  // entity_defs off component_id_), so every derived reference stays
  // consistent. Only in config-less mode: an explicit node map owns the name.
  if (node_map_path_.empty()) {
    const OpcuaClient::DeviceInfo info = connected ? client_->read_device_info() : OpcuaClient::DeviceInfo{};
    const ComponentIdentity ci = derive_component_identity(info, client_config_.endpoint_url);
    if (!ci.id.empty()) {
      node_map_.set_component_identity(ci.id, ci.name);
      log_info("Component identity derived from device: id='" + ci.id + "', name='" + ci.name + "'");
    }

    // Zero-config native A&C: with no node map and no explicit auto_alarms
    // block, subscribe the Server EventNotifier by default so discovered
    // Alarms & Conditions surface on /api/v1/faults with a fault_code derived
    // from the ConditionName/source. Finalize picks up the component id set
    // above for the ``<component_id>_alarms`` fallback entity.
    if (!auto_alarms_configured_) {
      node_map_.mutable_auto_alarms().enabled = true;
      node_map_.finalize_auto_alarms_overlay();
      if (node_map_.auto_alarms().enabled) {
        log_info("auto_alarms auto-enabled (config-less native A&C, source " +
                 node_map_.auto_alarms().source_node_id_str + ", entity " + node_map_.auto_alarms().entity_id + ")");
      }
    }
  }

  if (connected && node_map_.auto_browse_config().enabled) {
    // auto_browse needs a live session to walk the address space, so it can
    // only run here - after connect(), before the node map is consumed to
    // build publishers/poller below.
    run_auto_browse();
  }

  // Publisher creation moved here (was originally before connect()) so that
  // any entries auto_browse just merged into node_map_ also get a publisher.
  create_value_publishers();

  poller_ = std::make_unique<OpcuaPoller>(*client_, node_map_);
  poller_->set_alarm_callback(
      [this](const std::string & entity_id, const ros2_medkit::fault_detection::FaultSignal & signal) {
        on_alarm_change(entity_id, signal);
      });
  poller_->set_event_alarm_callback([this](const AlarmEventDelivery & delivery) {
    on_event_alarm(delivery);
  });
  poller_->set_poll_callback([this](const PollSnapshot & snap) {
    publish_values(snap);
  });
  // Plumb operator-visible warnings out of the poll thread into the ROS log.
  // Without this, ``ConditionRefresh rejected`` etc. only land in the
  // container's stderr, invisible to anyone watching /rosout or rclpp logs.
  poller_config_.log_warn = [this](const std::string & msg) {
    log_warn(msg);
  };

  // Do not block startup on the fault sink. OPC-UA AlarmCondition notifications
  // are one-shot (reported once on the inactive->active transition) and
  // ReportFault is fire-and-forget, so a report raised before fault_manager is
  // DDS-matched would be lost. send_report_fault/send_clear_fault buffer the
  // dispatch and flush_pending_reports (run on every poll) drains it once the
  // service appears, so a late sink still receives the alarm - without stalling
  // startup or capping recovery at a fixed timeout.
  // Issue #496: on top of that, gate the comms-lost latch on the fault sink
  // being discovered, so the raised flag is not set before the report can land.
  poller_config_.report_sink_ready = [this]() {
    return fault_clients_->report && fault_clients_->report->service_is_ready();
  };
  // Config-less discovery with no configured endpoint: let the poller's
  // reconnect arm ask for a fresh scan while it is down. Without this the
  // startup scan is the only one that ever runs, so a gateway that scanned
  // while its PLC was still booting retries the fallback endpoint forever and
  // only a restart finds the PLC.
  if (effective_rescan_interval_s(discovery_config_, endpoint_configured_) > 0) {
    poller_config_.rediscover_endpoint = [this]() {
      return rescan_endpoint_for_reconnect();
    };
  }
  poller_->start(poller_config_);
  log_info("OPC-UA poller started (mode: " + std::string(poller_->using_subscriptions() ? "subscription" : "poll") +
           ")");
}

std::vector<GatewayPlugin::PluginRoute> OpcuaPlugin::get_routes() {
  return {
      {"GET", R"(apps/([^/]+)/x-plc-data)",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_data(req, res);
       }},
      {"GET", R"(apps/([^/]+)/x-plc-data/([^/]+))",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_data_single(req, res);
       }},
      {"POST", R"(apps/([^/]+)/x-plc-operations/([^/]+))",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_operations(req, res);
       }},
      {"GET", R"(components/([^/]+)/x-plc-status)",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_status(req, res);
       }},
  };
}

void OpcuaPlugin::shutdown() {
  if (shutdown_requested_.exchange(true)) {
    return;
  }
  log_info("Shutting down OPC-UA plugin...");
  if (poller_) {
    poller_->stop();
  }
  client_->disconnect();
  log_info("OPC-UA plugin shutdown complete");
}

// -- IntrospectionProvider --

IntrospectionResult OpcuaPlugin::introspect(const IntrospectionInput & /*input*/) {
  log_info("introspect() called - generating PLC entities");
  IntrospectionResult result;

  // Serialize against an auto_browse re-walk (poll thread) rebuilding node_map_.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);

  Area area;
  area.id = node_map_.area_id();
  area.name = node_map_.area_name();
  area.namespace_path = "/" + node_map_.area_id();
  area.source = "plugin";
  area.description = "PLC systems connected via OPC-UA";
  result.new_entities.areas.push_back(std::move(area));

  Component comp;
  comp.id = node_map_.component_id();
  comp.name = node_map_.component_name();
  comp.namespace_path = "/" + node_map_.area_id();
  comp.fqn = "/" + node_map_.area_id() + "/" + node_map_.component_id();
  comp.area = node_map_.area_id();
  // Identity authority is gated on channel trust: only an authenticated
  // session (secured channel + certificate validation) gets the protocol tag
  // "opcua", which outranks the operator manifest in the identity merge. An
  // unauthenticated session (None channel or accept-any cert) could be a rogue
  // server spoofing the nameplate, so it gets the generic "plugin" tag, which
  // ranks below "manifest" (fills gaps, never overrides). Both tags survive
  // the plugin layer (it only stamps "plugin" on empty sources); per-field
  // provenance stays "opcua" for transparency in both cases.
  comp.source = opcua_identity_trusted(client_config_) ? "opcua" : "plugin";
  // Fault scope grants bare-id ownership only to external entities; the poller
  // reports PLC_COMMS_LOST under this component's own id.
  comp.external = true;
  // Read the endpoint off the client, not off client_config_: a reconnect
  // rescan can adopt a different server after startup, and the client is the
  // one that holds the endpoint actually being connected to.
  const std::string live_endpoint = client_ ? client_->endpoint_url() : client_config_.endpoint_url;
  comp.description = "PLC runtime connected at " + live_endpoint;

  // INV2: fill the asset-identity nameplate from the live server's device-info
  // (ServerStatus/BuildInfo + optional OPC-UA DI nameplate). Read once per
  // session (it is stable for a connection) and refreshed after a reconnect:
  // the poller reestablishes the session in the background, and a server that
  // registers its DI namespace or fills BuildInfo lazily after boot would
  // otherwise latch whatever the first read got.
  if (client_ && client_->is_connected()) {
    const uint64_t session_generation = client_->connection_generation();
    if (session_generation != device_identity_generation_) {
      device_identity_ = opcua_device_info_to_identity(client_->read_device_info(), live_endpoint);
      device_identity_generation_ = session_generation;
      if (!device_identity_.empty()) {
        log_info("Populated asset identity from OPC-UA device-info (manufacturer='" + device_identity_.manufacturer +
                 "', model='" + device_identity_.model + "', orderCode='" + device_identity_.order_code + "')");
      }
    }
  }
  if (!device_identity_.empty()) {
    comp.identity = device_identity_;
  }

  result.new_entities.components.push_back(std::move(comp));

  // Register x-plc-status on the PLC runtime component only, so non-PLC
  // components in the shared SOVD entity tree do not advertise a
  // capability they cannot serve.
  if (ctx_) {
    ctx_->register_entity_capability(node_map_.component_id(), "x-plc-status");
  }

  for (const auto & def : node_map_.entity_defs()) {
    App app;
    app.id = def.id;
    app.name = def.name;
    app.component_id = def.component_id;
    app.external = true;
    app.is_online = client_->is_connected();
    app.source = "plugin";
    app.description = "PLC application with " + std::to_string(def.data_names.size()) + " data points";
    // Declare the value-bridge topics on the entity that owns the data points.
    // Graph discovery attributes them to the gateway node (their publisher),
    // so without this the entity-scoped topic lookup - what resolves a data
    // trigger's resource_path - comes up empty and the trigger never fires
    // (issue #584). entry_has_value_publisher is the same predicate
    // create_value_publishers() uses, so declaration and publication cannot
    // drift apart.
    for (const auto * entry : node_map_.entries_for_entity(def.id)) {
      if (entry_has_value_publisher(*entry)) {
        app.topics.publishes.push_back(entry->ros2_topic);
      }
    }
    result.new_entities.apps.push_back(std::move(app));

    // Register capabilities per entity (never type-level): only PLC-backed
    // apps advertise x-plc-data. The auto_alarms fallback App (and any
    // event_alarms-only App) has an entity_defs entry to host faults but no
    // NodeMap entries of its own, so def.data_names stays empty - skip it or
    // /apps/<id>/x-plc-data 404s despite being advertised. Apps with
    // writable nodes also get x-plc-operations.
    if (ctx_) {
      if (!def.data_names.empty()) {
        ctx_->register_entity_capability(def.id, "x-plc-data");
      }
      if (!def.writable_names.empty()) {
        ctx_->register_entity_capability(def.id, "x-plc-operations");
      }
    }
  }

  return result;
}

// -- Route handlers --

void OpcuaPlugin::handle_plc_data(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto entity_id = req.path_param(1);
  auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  // Held across build_data_response too (it reads node_map_ without locking).
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto entries = node_map_.entries_for_entity(entity_id);
  if (entries.empty()) {
    res.send_error(404, ERR_RESOURCE_NOT_FOUND, "No PLC data mapped for entity: " + entity_id);
    return;
  }

  res.send_json(build_data_response(entity_id));
}

void OpcuaPlugin::handle_plc_data_single(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto entity_id = req.path_param(1);
  auto data_name = req.path_param(2);

  if (!is_valid_path_segment(data_name)) {
    res.send_error(400, ERR_INVALID_PARAMETER, "Invalid data name");
    return;
  }

  auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  // Held for as long as ``entry`` (a pointer into node_map_) is dereferenced.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto * entry = node_map_.find_by_data_name(entity_id, data_name);
  if (!entry) {
    res.send_error(404, ERR_RESOURCE_NOT_FOUND, "Data point not found: " + data_name + " in entity: " + entity_id);
    return;
  }

  auto snap = poller_->snapshot();
  auto it = snap.values.find(entry->node_id_str);

  nlohmann::json j;
  j["name"] = entry->data_name;
  j["display_name"] = entry->display_name;
  j["node_id"] = entry->node_id_str;

  if (it != snap.values.end()) {
    std::visit(
        [&j](auto && v) {
          j["value"] = v;
        },
        it->second);
  } else {
    j["value"] = nullptr;
  }

  if (!entry->unit.empty()) {
    j["unit"] = entry->unit;
  }
  j["data_type"] = entry->data_type;
  j["writable"] = entry->writable;

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  j["timestamp"] = ts;

  res.send_json(j);
}

void OpcuaPlugin::handle_plc_operations(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto entity_id = req.path_param(1);
  auto op_name = req.path_param(2);

  if (!is_valid_path_segment(op_name)) {
    res.send_error(400, ERR_INVALID_PARAMETER, "Invalid operation name");
    return;
  }

  auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  std::string data_name;
  if (op_name.substr(0, 4) == "set_") {
    data_name = op_name.substr(4);
  } else {
    data_name = op_name;
  }

  // Held for as long as ``entry`` (a pointer into node_map_) is dereferenced,
  // including across the OPC-UA write below.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto * entry = node_map_.find_by_data_name(entity_id, data_name);
  if (!entry) {
    res.send_error(404, ERR_RESOURCE_NOT_FOUND, "Operation not found: " + op_name + " in entity: " + entity_id);
    return;
  }

  if (!entry->writable) {
    res.send_error(400, ERR_INVALID_REQUEST, "Data point is read-only: " + data_name);
    return;
  }

  nlohmann::json body;
  try {
    body = nlohmann::json::parse(req.body());
  } catch (const nlohmann::json::parse_error &) {
    res.send_error(400, ERR_INVALID_REQUEST, "Invalid JSON body");
    return;
  }

  if (!body.contains("value")) {
    res.send_error(400, ERR_INVALID_REQUEST, "Missing 'value' field");
    return;
  }

  auto parsed = parse_coerce_validate(body["value"], *entry);
  if (!parsed) {
    res.send_error(400, ERR_INVALID_REQUEST, parsed.error());
    return;
  }

  auto write_result = client_->write_value(entry->node_id, *parsed, entry->data_type);
  if (!write_result) {
    int status = (write_result.error().code == OpcuaClient::WriteError::NotConnected ||
                  write_result.error().code == OpcuaClient::WriteError::TransportError)
                     ? 502
                     : 400;
    res.send_error(status, ERR_SERVICE_UNAVAILABLE, write_result.error().message);
    return;
  }

  nlohmann::json response;
  response["status"] = "ok";
  response["operation"] = op_name;
  response["node_id"] = entry->node_id_str;
  std::visit(
      [&response](auto && v) {
        response["value_written"] = v;
      },
      *parsed);

  res.send_json(response);
}

void OpcuaPlugin::handle_plc_status(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto component_id = req.path_param(1);
  auto entity = ctx_->validate_entity_for_route(req, res, component_id);
  if (!entity) {
    return;
  }

  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto snap = poller_->snapshot();

  nlohmann::json j;
  j["component_id"] = component_id;
  j["connected"] = snap.connected;
  j["endpoint_url"] = client_->endpoint_url();
  j["server_description"] = client_->server_description();
  j["mode"] = poller_->using_subscriptions() ? "subscription" : "poll";
  j["poll_count"] = snap.poll_count;
  j["error_count"] = snap.error_count;
  j["node_count"] = node_map_.entries().size();
  j["entity_count"] = node_map_.entity_defs().size();

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  j["last_update"] = ts;

  nlohmann::json alarms = nlohmann::json::array();
  for (const auto & [code, active] : snap.alarms) {
    if (active) {
      alarms.push_back(code);
    }
  }
  j["active_alarms"] = alarms;

  res.send_json(j);
}

// -- Fault bridge --

void OpcuaPlugin::on_alarm_change(const std::string & entity_id,
                                  const ros2_medkit::fault_detection::FaultSignal & signal) {
  if (shutdown_requested_.load()) {
    return;
  }

  if (signal.active) {
    log_info("Alarm activated: " + signal.fault_code + " on " + entity_id);
    send_report_fault(entity_id, signal.fault_code, signal.severity, signal.message);
  } else {
    log_info("Alarm cleared: " + signal.fault_code + " on " + entity_id);
    send_clear_fault(signal.fault_code);
  }
}

std::string OpcuaPlugin::map_severity(uint16_t live_severity, const std::string & severity_override) {
  // Map raw OPC-UA Severity (1-1000) to SOVD severity bucket.
  // Selfpatch convention documented in design/index.rst; not from IEC 62682.
  // The resolved severity_override (mapping- or source-level, issue #389)
  // wins when set.
  if (!severity_override.empty()) {
    return severity_override;
  }
  if (live_severity >= 801) {
    return std::string("CRITICAL");
  }
  if (live_severity >= 501) {
    return std::string("ERROR");
  }
  if (live_severity >= 201) {
    return std::string("WARNING");
  }
  return std::string("INFO");
}

void OpcuaPlugin::apply_auto_alarms_param(const nlohmann::json & value, AutoAlarmsConfig & cfg,
                                          const std::function<void(const std::string &)> & warn) {
  // Field-for-field mirror of node_map.cpp's YAML ``auto_alarms:`` loader,
  // reading nlohmann::json instead of YAML::Node. Only keys actually present
  // overwrite ``cfg`` so a partial param overlays cleanly on top of whatever
  // the node-map YAML declared (the JSON param wins on collision).
  if (value.is_boolean()) {
    cfg.enabled = value.get<bool>();
    return;
  }
  if (!value.is_object()) {
    warn("plugins.opcua.auto_alarms must be a boolean or an object - ignoring");
    return;
  }

  // Presence of the map itself implies intent to enable (matches the bare
  // boolean and the YAML map form); ``enabled: false`` still turns it off.
  cfg.enabled = value.value("enabled", true);

  if (value.contains("source_node_id")) {
    if (value["source_node_id"].is_string() && !value["source_node_id"].get<std::string>().empty()) {
      cfg.source_node_id_str = value["source_node_id"].get<std::string>();
    } else {
      warn("plugins.opcua.auto_alarms.source_node_id must be a non-empty string - keeping current value");
    }
  }
  if (value.contains("entity_id")) {
    if (value["entity_id"].is_string() && !value["entity_id"].get<std::string>().empty()) {
      cfg.entity_id = value["entity_id"].get<std::string>();
    } else {
      warn("plugins.opcua.auto_alarms.entity_id must be a non-empty string - keeping current value");
    }
  }
  if (value.contains("auto_clear")) {
    if (value["auto_clear"].is_boolean()) {
      cfg.auto_clear = value["auto_clear"].get<bool>();
    } else {
      warn("plugins.opcua.auto_alarms.auto_clear must be a boolean - keeping current value");
    }
  }

  if (value.contains("severity_bands")) {
    const auto & bands = value["severity_bands"];
    if (!bands.is_object()) {
      warn("plugins.opcua.auto_alarms.severity_bands must be an object - ignoring");
    } else {
      auto read_band = [&](const char * key, uint16_t def) -> uint16_t {
        if (!bands.contains(key)) {
          return def;
        }
        if (!bands[key].is_number_integer()) {
          warn(std::string("plugins.opcua.auto_alarms.severity_bands.") + key + " must be an integer - using default");
          return def;
        }
        const auto raw = bands[key].get<int64_t>();
        if (raw < 0 || raw > 1000) {
          warn(std::string("plugins.opcua.auto_alarms.severity_bands.") + key +
               " out of range (0..1000) - using default");
          return def;
        }
        return static_cast<uint16_t>(raw);
      };
      cfg.severity_bands.critical_min = read_band("critical", cfg.severity_bands.critical_min);
      cfg.severity_bands.error_min = read_band("error", cfg.severity_bands.error_min);
      cfg.severity_bands.warning_min = read_band("warning", cfg.severity_bands.warning_min);
      for (const auto & [key, band_val] : bands.items()) {
        (void)band_val;
        if (key != "critical" && key != "error" && key != "warning") {
          warn("plugins.opcua.auto_alarms.severity_bands: unknown key '" + key + "' - ignored");
        }
      }
      if (!(cfg.severity_bands.critical_min >= cfg.severity_bands.error_min &&
            cfg.severity_bands.error_min >= cfg.severity_bands.warning_min)) {
        warn(
            "plugins.opcua.auto_alarms.severity_bands must satisfy critical >= error >= warning - "
            "resetting to the default bands (801/501/201)");
        cfg.severity_bands = AutoAlarmsSeverityBands{};
      }
    }
  }

  auto read_pattern_list = [&](const char * key) -> std::vector<std::string> {
    std::vector<std::string> out;
    if (!value.contains(key)) {
      return out;
    }
    if (!value[key].is_array()) {
      warn(std::string("plugins.opcua.auto_alarms.") + key + " must be an array of strings - ignoring");
      return out;
    }
    for (const auto & p : value[key]) {
      if (p.is_string()) {
        out.push_back(p.get<std::string>());
      } else {
        warn(std::string("plugins.opcua.auto_alarms.") + key + ": non-string entry - skipping");
      }
    }
    return out;
  };
  // Only replace the pattern lists when the param actually carries them, so a
  // param that omits include/exclude does not wipe a YAML-declared list.
  if (value.contains("include")) {
    cfg.include_patterns = read_pattern_list("include");
  }
  if (value.contains("exclude")) {
    cfg.exclude_patterns = read_pattern_list("exclude");
  }

  for (const auto & [key, item] : value.items()) {
    (void)item;
    if (key != "enabled" && key != "source_node_id" && key != "entity_id" && key != "auto_clear" &&
        key != "severity_bands" && key != "include" && key != "exclude") {
      warn("plugins.opcua.auto_alarms: unknown key '" + key + "' - ignored");
    }
  }
}

void OpcuaPlugin::on_event_alarm(const AlarmEventDelivery & delivery) {
  if (shutdown_requested_.load()) {
    return;
  }

  const std::string severity_str = map_severity(delivery.severity, delivery.severity_override);

  switch (delivery.action) {
    case AlarmAction::ReportConfirmed:
      log_info("AlarmCondition CONFIRMED: " + delivery.fault_code + " on " + delivery.entity_id +
               " (severity=" + std::to_string(delivery.severity) + ")");
      send_report_fault(delivery.entity_id, delivery.fault_code, severity_str, delivery.message);
      break;
    case AlarmAction::ReportHealed:
      // Intentional no-op (Copilot review on PR #387).
      //
      // OPC-UA AlarmConditionType HEALED means "alarm physically cleared
      // (ActiveState=false) but operator workflow incomplete (ack and/or
      // confirm pending)". Per Part 9 §5.7 the Cleared transition is
      // operator-driven, not statistical.
      //
      // ros2_medkit_msgs/srv/ReportFault has only FAILED/PASSED verbs and
      // fault_manager treats PASSED through a debounce engine. Sending
      // EVENT_PASSED on every latch would let fault_manager auto-clear
      // the fault via healing_threshold debounce, defeating the spec
      // contract that requires explicit operator Acknowledge + Confirm.
      // Conversely, healing_enabled=false would silently lose the HEALED
      // signal entirely.
      //
      // Until we add STATUS_LATCHED (or a similar lifecycle-distinguishing
      // status) to ros2_medkit_msgs/msg/Fault we keep status=CONFIRMED
      // until the next ClearFault fires. The operator-side gap (cannot
      // see "physically cleared, awaiting confirm" in the UI) is tracked
      // separately; see PR #387 review thread.
      log_info("AlarmCondition HEALED (latched, awaiting ack/confirm): " + delivery.fault_code);
      break;
    case AlarmAction::ClearFault:
      log_info("AlarmCondition CLEARED: " + delivery.fault_code);
      send_clear_fault(delivery.fault_code);
      break;
    case AlarmAction::NoOp:
      break;
  }
}

void OpcuaPlugin::send_report_fault(const std::string & entity_id, const std::string & fault_code,
                                    const std::string & severity_str, const std::string & message) {
  if (!fault_clients_->report) {
    log_warn("ReportFault service client not available");
    return;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
  request->fault_code = fault_code;
  request->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
  request->description = message;
  request->source_id = entity_id;

  // Map severity string to uint8 (case-insensitive)
  std::string sev_lower = severity_str;
  std::transform(sev_lower.begin(), sev_lower.end(), sev_lower.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (sev_lower == "critical") {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;
  } else if (sev_lower == "error") {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  } else if (sev_lower == "warning") {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;
  } else {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_INFO;
  }

  send_or_buffer([this, request]() {
    fault_clients_->report->async_send_request(request);
  });
}

void OpcuaPlugin::send_clear_fault(const std::string & fault_code) {
  if (!fault_clients_->clear) {
    log_warn("ClearFault service client not available");
    return;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ClearFault::Request>();
  request->fault_code = fault_code;

  send_or_buffer([this, request]() {
    fault_clients_->clear->async_send_request(request);
  });
}

void OpcuaPlugin::clear_comms_lost_on_connect() {
  if (!poller_config_.comms_lost_fault_enabled) {
    return;
  }
  // ClearFault is idempotent from this side: send_clear_fault is
  // fire-and-forget, so a "Fault not found" answer for a code that was never
  // raised costs nothing here and is the normal case on a healthy start.
  log_info(std::string("OPC-UA connection established; clearing any standing ") + kCommsLostFaultCode);
  send_clear_fault(kCommsLostFaultCode);
}

void OpcuaPlugin::send_or_buffer(std::function<void()> dispatch) {
  // Bound the buffer so a deployment with no fault_manager cannot grow it
  // without limit; drop the oldest (least relevant) pending dispatch.
  // Runs on both the poll thread and the REST clear_fault thread, so the vector
  // mutation is serialised by pending_reports_mutex_.
  constexpr size_t kMaxPendingReports = 256;
  bool dropped_oldest = false;
  {
    std::lock_guard<std::mutex> lock(pending_reports_mutex_);
    if (pending_reports_.size() >= kMaxPendingReports) {
      pending_reports_.erase(pending_reports_.begin());
      dropped_oldest = true;
    }
    pending_reports_.push_back(std::move(dispatch));
  }
  if (dropped_oldest) {
    log_warn("pending fault report buffer full (" + std::to_string(kMaxPendingReports) + "), dropping oldest");
  }
  // Drains immediately (in order) if the sink is already matched.
  flush_pending_reports();
}

void OpcuaPlugin::flush_pending_reports() {
  // Keep the reports buffered until the sink is discoverable (checked outside the
  // lock; rclcpp client state is thread-safe).
  if (!fault_clients_->report || !fault_clients_->report->service_is_ready()) {
    return;
  }
  // Move the ready batch out under the lock, then dispatch it OUTSIDE the lock so
  // the vector is never being reallocated by a concurrent send_or_buffer while it
  // is iterated here (the use-after-free that corrupted the heap), and so the ROS
  // service call never runs under the mutex.
  std::vector<std::function<void()>> batch;
  {
    std::lock_guard<std::mutex> lock(pending_reports_mutex_);
    if (pending_reports_.empty()) {
      return;
    }
    batch.swap(pending_reports_);
  }
  for (auto & dispatch : batch) {
    dispatch();
  }
}

void OpcuaPlugin::create_value_publishers() {
  auto * node = ctx_ ? ctx_->node() : nullptr;
  if (!node) {
    return;
  }
  // Create ROS 2 publishers for numeric PLC values only (skip string-typed entries)
  for (const auto & entry : node_map_.entries()) {
    if (publishers_.count(entry.node_id_str) > 0) {
      continue;  // already created (e.g. a previous call before a reconnect)
    }
    if (!entry_has_value_publisher(entry)) {
      log_info("PLC bridge: skipping non-numeric " + entry.node_id_str + " (data_type=" + entry.data_type + ")");
      continue;
    }
    publishers_[entry.node_id_str] =
        node->create_publisher<std_msgs::msg::Float32>(entry.ros2_topic, rclcpp::SensorDataQoS());
    log_info("PLC bridge: " + entry.node_id_str + " -> " + entry.ros2_topic + " (Float32)");
  }
}

void OpcuaPlugin::run_auto_browse() {
  OpcuaClientBrowseSource source(*client_);
  AutoBrowser browser(source, node_map_.auto_browse_config());
  // The address-space walk is a series of blocking OPC-UA round-trips - do it
  // without the lock so it never stalls the REST read handlers.
  AutoBrowseResult result = browser.browse();

  size_t writable_points = 0;
  for (const auto & e : result.entries) {
    if (e.writable) {
      ++writable_points;
    }
  }

  size_t added = 0;
  size_t entity_count = 0;
  {
    // Serialize the mutation (entries / indices / entity_defs rebuild) against
    // the HTTP read handlers, which take the shared lock.
    std::unique_lock<std::shared_mutex> lock(node_map_mutex_);
    added = node_map_.merge_auto_browsed_entries(std::move(result.entries));
    entity_count = node_map_.entity_defs().size();
  }
  // Publishers for any newly merged entries (idempotent via publishers_.count).
  // Safe without the node_map_ lock: the poll thread is the sole writer and we
  // are on it (or in single-threaded set_context startup).
  create_value_publishers();
  auto_browse_generation_ = client_->connection_generation();

  log_info("auto_browse: walked " + std::to_string(result.nodes_visited) + " address-space nodes, added " +
           std::to_string(added) + " data points (" + std::to_string(entity_count) + " entities total, " +
           std::to_string(writable_points) + " server-writable)" +
           (result.node_cap_hit ? " [node cap reached - tree may be incomplete]" : "") +
           (result.depth_cap_hit ? " [depth cap reached on at least one branch]" : ""));
}

void OpcuaPlugin::maybe_rebrowse_on_reconnect() {
  if (!node_map_.auto_browse_config().enabled || !client_ || !client_->is_connected()) {
    return;
  }
  const uint64_t generation = client_->connection_generation();
  if (generation == auto_browse_generation_) {
    return;  // already walked this session
  }
  run_auto_browse();
}

void OpcuaPlugin::publish_values(const PollSnapshot & snap) {
  if (shutdown_requested_.load()) {
    return;
  }
  // Poll-thread hook: drain any fault reports buffered before fault_manager was
  // discovered, so a late sink still receives them.
  flush_pending_reports();
  // Poll-thread hook: (re)run auto_browse after a fresh session so a PLC that
  // came up (or restarted) after the initial connect still gets walked.
  maybe_rebrowse_on_reconnect();
  for (const auto & [node_id, value] : snap.values) {
    auto pub_it = publishers_.find(node_id);
    if (pub_it == publishers_.end()) {
      continue;
    }

    std_msgs::msg::Float32 msg;
    bool is_numeric = std::visit(
        [&msg](auto && v) -> bool {
          using T = std::decay_t<decltype(v)>;
          if constexpr (std::is_arithmetic_v<T>) {
            msg.data = static_cast<float>(v);
            return true;
          }
          return false;
        },
        value);

    if (!is_numeric) {
      if (warned_non_numeric_.insert(node_id).second) {
        log_warn("Skipping non-numeric value for ROS 2 bridging: " + node_id);
      }
      continue;
    }

    if (!std::isfinite(msg.data)) {
      continue;
    }

    pub_it->second->publish(msg);
  }
}

void OpcuaPlugin::log_security_profile() const {
  auto policy_name = [](SecurityPolicy p) -> std::string {
    switch (p) {
      case SecurityPolicy::None:
        return "None";
      case SecurityPolicy::Basic256Sha256:
        return "Basic256Sha256";
      case SecurityPolicy::Aes128Sha256RsaOaep:
        return "Aes128Sha256RsaOaep";
      case SecurityPolicy::Aes256Sha256RsaPss:
        return "Aes256Sha256RsaPss";
    }
    return "None";
  };
  auto mode_name = [](SecurityMode m) -> std::string {
    switch (m) {
      case SecurityMode::None:
        return "None";
      case SecurityMode::Sign:
        return "Sign";
      case SecurityMode::SignAndEncrypt:
        return "SignAndEncrypt";
    }
    return "None";
  };
  auto auth_name = [](UserAuthMode a) -> std::string {
    switch (a) {
      case UserAuthMode::Anonymous:
        return "Anonymous";
      case UserAuthMode::UsernamePassword:
        return "Username/Password";
      case UserAuthMode::X509:
        return "X.509";
    }
    return "Anonymous";
  };

  const std::string profile = "OPC-UA security: SecurityPolicy=" + policy_name(client_config_.security_policy) +
                              ", MessageSecurityMode=" + mode_name(client_config_.security_mode) +
                              ", user=" + auth_name(client_config_.user_auth_mode);

  const bool secure_channel = OpcuaClient::requires_secure_channel(client_config_);

  if (OpcuaClient::credentials_sent_in_clear(client_config_)) {
    // Username/password or X.509 credentials sent without a Sign/SignAndEncrypt
    // channel travel in the clear and can be intercepted on the wire.
    log_warn(profile +
             ". WARNING: user credentials cross an unencrypted OPC-UA channel (MessageSecurityMode=None) - "
             "they can be intercepted. Use Sign or SignAndEncrypt on an untrusted network.");
  } else if (!secure_channel) {
    log_warn(profile + ". Unsecured - not suitable for untrusted networks.");
  } else if (!client_config_.reject_untrusted) {
    log_warn(profile + ". WARNING: reject_untrusted=false (accepts any server certificate).");
  } else {
    log_info(profile);
  }
}

int OpcuaPlugin::effective_rescan_interval_s(const OpcuaDiscoveryConfig & config, bool endpoint_configured) {
  if (!config.enabled || endpoint_configured) {
    return 0;
  }
  return config.interval_s > 0 ? config.interval_s : kDefaultRescanIntervalS;
}

std::optional<std::string> OpcuaPlugin::discover_endpoint(const OpcuaDiscoveryConfig & config, bool endpoint_configured,
                                                          const PortScanFn & scan, const IdentifyFn & identify,
                                                          const std::function<void(const std::string &)> & log_info,
                                                          const std::function<void(const std::string &)> & log_warn) {
  if (!config.enabled) {
    return std::nullopt;
  }
  // Never override an explicitly configured endpoint: discovery must not open a
  // second session on a PLC the operator already targets (and already polls).
  if (endpoint_configured) {
    return std::nullopt;
  }

  NetworkDiscovery discovery(config, scan, identify);

  const auto subnets = discovery.resolve_subnets();
  if (subnets.empty()) {
    log_warn("OPC-UA discovery: no subnet configured and could not derive a local /24; nothing to scan.");
    return std::nullopt;
  }
  std::string subnet_list;
  for (const auto & s : subnets) {
    subnet_list += (subnet_list.empty() ? "" : ", ") + s;
  }
  log_info("OPC-UA discovery: read-only active scan of [" + subnet_list + "] on " +
           std::to_string(config.ports.size()) + " port(s)...");

  const std::vector<DiscoveredEndpoint> found = discovery.run();

  // Summarize what was found and what was skipped (leads, LDS, secured-only).
  size_t data_servers = 0;
  size_t discovery_servers = 0;
  size_t secured_only = 0;
  size_t leads = 0;
  for (const auto & ep : found) {
    if (ep.protocol != "opcua") {
      ++leads;
      continue;
    }
    if (!ep.identify_error.empty()) {
      ++leads;
      continue;
    }
    if (!ep.is_data_server()) {
      ++discovery_servers;
      continue;
    }
    ++data_servers;
    if (!ep.anonymous_none_available) {
      ++secured_only;
    }
    log_info("OPC-UA discovery: found data server " + ep.endpoint_url + " (uri='" + ep.application_uri +
             "', product='" + ep.product_uri + "', None/Anonymous=" + (ep.anonymous_none_available ? "yes" : "no") +
             ")");
  }
  log_info("OPC-UA discovery summary: " + std::to_string(data_servers) + " data server(s), " +
           std::to_string(discovery_servers) + " discovery server(s)/LDS, " + std::to_string(secured_only) +
           " secured-only (need credentials), " + std::to_string(leads) + " non-OPC-UA/unidentified lead(s).");

  const DiscoveredEndpoint * chosen = NetworkDiscovery::select_auto_endpoint(found, config.anonymous_none_only);
  if (chosen == nullptr) {
    log_warn(
        "OPC-UA discovery: no auto-connectable None/Anonymous data server found; leaving the endpoint unchanged. "
        "Secured-only servers require operator credentials.");
    return std::nullopt;
  }

  log_info("OPC-UA discovery: selected endpoint " + chosen->endpoint_url + " (uri='" + chosen->application_uri + "')");
  return chosen->endpoint_url;
}

void OpcuaPlugin::run_startup_discovery() {
  if (!discovery_config_.enabled) {
    return;
  }
  if (endpoint_configured_) {
    log_info("OPC-UA discovery enabled but endpoint_url is explicitly configured (" + client_config_.endpoint_url +
             "); skipping auto-discovery to avoid a second session.");
    return;
  }

  // Stamp the scan before running it: the rescan cadence measures the gap
  // between the START of two sweeps, so a slow sweep does not immediately earn
  // another one.
  last_discovery_scan_ = std::chrono::steady_clock::now();
  const auto chosen = discover_endpoint(
      discovery_config_, endpoint_configured_, discovery_scan_fn_, discovery_identify_fn_,
      [this](const std::string & m) {
        log_info(m);
      },
      [this](const std::string & m) {
        log_warn(m);
      });

  if (!chosen) {
    // The startup scan can legitimately find nothing - a gateway that boots
    // alongside its PLC routinely scans while the PLC is still coming up. The
    // endpoint stays at its default and the poller's reconnect arm rescans on
    // the cadence below, so this is a delay rather than a dead end.
    log_info("OPC-UA discovery: startup scan selected no endpoint; the reconnect loop rescans every " +
             std::to_string(effective_rescan_interval_s(discovery_config_, endpoint_configured_)) + "s while down.");
    return;
  }

  client_config_.endpoint_url = *chosen;
  log_info("OPC-UA discovery: auto-selected endpoint " + *chosen + " - handing to the connect + introspect path.");
}

std::optional<std::string> OpcuaPlugin::rescan_endpoint_for_reconnect() {
  // A sweep is a bounded but multi-second blocking call on the poll thread, and
  // stop() has to wait for whatever it is in the middle of. Do not start one the
  // shutdown is going to throw away.
  if (shutdown_requested_.load()) {
    return std::nullopt;
  }
  const int interval_s = effective_rescan_interval_s(discovery_config_, endpoint_configured_);
  if (interval_s <= 0) {
    return std::nullopt;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_discovery_scan_ < std::chrono::seconds(interval_s)) {
    return std::nullopt;
  }
  last_discovery_scan_ = now;

  const auto chosen = discover_endpoint(
      discovery_config_, endpoint_configured_, discovery_scan_fn_, discovery_identify_fn_,
      [this](const std::string & m) {
        log_info(m);
      },
      [this](const std::string & m) {
        log_warn(m);
      });
  // The live client config, not client_config_: this runs on the poll thread
  // and client_config_ is read by the refresh thread in introspect(). The
  // client owns the endpoint once connect() has been called with it, and its
  // accessors are mutex-guarded.
  const std::string current = client_ ? client_->endpoint_url() : client_config_.endpoint_url;
  if (!chosen || *chosen == current) {
    return std::nullopt;
  }

  log_info("OPC-UA discovery: rescan while disconnected adopted endpoint " + *chosen + " (was " + current + ")");
  return chosen;
}

nlohmann::json OpcuaPlugin::build_data_response(const std::string & entity_id) const {
  auto entries = node_map_.entries_for_entity(entity_id);
  auto snap = poller_->snapshot();

  nlohmann::json items = nlohmann::json::array();

  for (const auto * entry : entries) {
    nlohmann::json item;
    item["name"] = entry->data_name;
    item["display_name"] = entry->display_name;
    item["node_id"] = entry->node_id_str;

    auto it = snap.values.find(entry->node_id_str);
    if (it != snap.values.end()) {
      std::visit(
          [&item](auto && v) {
            item["value"] = v;
          },
          it->second);
    } else {
      item["value"] = nullptr;
    }

    if (!entry->unit.empty()) {
      item["unit"] = entry->unit;
    }
    item["data_type"] = entry->data_type;
    item["writable"] = entry->writable;

    items.push_back(std::move(item));
  }

  nlohmann::json response;
  response["entity_id"] = entity_id;
  response["items"] = items;
  response["connected"] = snap.connected;

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  response["timestamp"] = ts;

  return response;
}

// -- DataProvider interface --

tl::expected<dto::DataListResult, DataProviderErrorInfo> OpcuaPlugin::list_data(const std::string & entity_id) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "plugin not initialized", 503});
  }

  // Held while the returned pointers into node_map_ are dereferenced below.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto entries = node_map_.entries_for_entity(entity_id);
  if (entries.empty()) {
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::EntityNotFound, "No PLC data for entity: " + entity_id, 404});
  }

  nlohmann::json items = nlohmann::json::array();
  auto snap = poller_->snapshot();

  for (const auto * entry : entries) {
    nlohmann::json item;
    item["id"] = entry->data_name;
    item["name"] = entry->display_name;
    item["category"] = "currentData";

    auto it = snap.values.find(entry->node_id_str);
    if (it != snap.values.end()) {
      std::visit(
          [&item](auto && v) {
            item["value"] = v;
          },
          it->second);
    } else {
      item["value"] = nullptr;
    }

    if (!entry->unit.empty()) {
      item["unit"] = entry->unit;
    }
    item["data_type"] = entry->data_type;
    item["writable"] = entry->writable;
    items.push_back(std::move(item));
  }

  return dto::DataListResult{nlohmann::json{{"items", std::move(items)}}};
}

tl::expected<dto::DataValue, DataProviderErrorInfo> OpcuaPlugin::read_data(const std::string & entity_id,
                                                                           const std::string & resource_name) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "plugin not initialized", 503});
  }

  // Held while ``entry`` (a pointer into node_map_) is dereferenced below.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto * entry = node_map_.find_by_data_name(entity_id, resource_name);
  if (!entry) {
    return tl::make_unexpected(DataProviderErrorInfo{
        DataProviderError::ResourceNotFound, "Data point not found: " + resource_name + " in " + entity_id, 404});
  }

  auto snap = poller_->snapshot();
  auto it = snap.values.find(entry->node_id_str);

  nlohmann::json result;
  result["id"] = entry->data_name;
  result["name"] = entry->display_name;
  result["node_id"] = entry->node_id_str;

  if (it != snap.values.end()) {
    std::visit(
        [&result](auto && v) {
          result["value"] = v;
        },
        it->second);
  } else {
    result["value"] = nullptr;
  }

  if (!entry->unit.empty()) {
    result["unit"] = entry->unit;
  }
  result["data_type"] = entry->data_type;
  result["writable"] = entry->writable;

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  result["timestamp"] = ts;

  return dto::DataValue{std::move(result)};
}

tl::expected<dto::DataWriteResult, DataProviderErrorInfo> OpcuaPlugin::write_data(const std::string & entity_id,
                                                                                  const std::string & resource_name,
                                                                                  const nlohmann::json & value) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "plugin not initialized", 503});
  }

  // Held for as long as ``entry`` (a pointer into node_map_) is dereferenced,
  // including across the OPC-UA write below.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto * entry = node_map_.find_by_data_name(entity_id, resource_name);
  if (!entry) {
    return tl::make_unexpected(DataProviderErrorInfo{
        DataProviderError::ResourceNotFound, "Data point not found: " + resource_name + " in " + entity_id, 404});
  }
  if (!entry->writable) {
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::ReadOnly, "Data point is read-only: " + resource_name, 400});
  }

  if (!value.contains("value")) {
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::InvalidValue, "Missing 'value' field in write body", 400});
  }

  auto parsed = parse_coerce_validate(value["value"], *entry);
  if (!parsed) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::InvalidValue, parsed.error(), 400});
  }

  auto write_result = client_->write_value(entry->node_id, *parsed, entry->data_type);
  if (!write_result) {
    auto wcode = write_result.error().code;
    if (wcode == OpcuaClient::WriteError::TypeMismatch) {
      return tl::make_unexpected(
          DataProviderErrorInfo{DataProviderError::InvalidValue, write_result.error().message, 400});
    }
    if (wcode == OpcuaClient::WriteError::AccessDenied) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, write_result.error().message, 403});
    }
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::TransportError, write_result.error().message, 502});
  }

  nlohmann::json result;
  result["id"] = resource_name;
  result["status"] = "ok";
  std::visit(
      [&result](auto && v) {
        result["value_written"] = v;
      },
      *parsed);
  return dto::DataWriteResult{std::move(result)};
}

bool OpcuaPlugin::has_data(const std::string & entity_id) const {
  // Same check list_data uses to 404 - keeps advertised capabilities honest
  // for entities with no NodeMap entries (the Component, the auto_alarms
  // fallback App).
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  return !node_map_.entries_for_entity(entity_id).empty();
}

// -- OperationProvider interface --

tl::expected<dto::Collection<dto::OperationItem>, OperationProviderErrorInfo>
OpcuaPlugin::list_operations(const std::string & entity_id) {
  dto::Collection<dto::OperationItem> collection;

  // Held while iterating node_map_ defs / entries (auto_browse may rebuild them).
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  for (const auto & def : node_map_.entity_defs()) {
    if (def.id != entity_id) {
      continue;
    }
    for (const auto & writable_name : def.writable_names) {
      auto * entry = node_map_.find_by_data_name(entity_id, writable_name);
      dto::OperationItem item;
      item.id = "set_" + writable_name;
      item.name = entry ? ("Set " + entry->display_name) : ("Set " + writable_name);
      item.proximity_proof_required = false;
      item.asynchronous_execution = false;
      collection.items.push_back(std::move(item));
    }

    // Issue #386: emit acknowledge_fault / confirm_fault when the entity
    // has at least one native AlarmConditionType event subscription. The
    // fault_code parameter (passed in operation execution body) discriminates
    // which condition the operator is acting on.
    bool has_event_alarms = std::any_of(node_map_.event_alarms().begin(), node_map_.event_alarms().end(),
                                        [&entity_id](const AlarmEventConfig & cfg) {
                                          return cfg.entity_id == entity_id;
                                        });
    if (has_event_alarms) {
      dto::OperationItem ack;
      ack.id = "acknowledge_fault";
      ack.name = "Acknowledge an alarm";
      ack.proximity_proof_required = false;
      ack.asynchronous_execution = false;
      collection.items.push_back(std::move(ack));

      dto::OperationItem confirm;
      confirm.id = "confirm_fault";
      confirm.name = "Confirm an alarm condition is addressed";
      confirm.proximity_proof_required = false;
      confirm.asynchronous_execution = false;
      collection.items.push_back(std::move(confirm));
    }

    return collection;
  }

  return tl::make_unexpected(
      OperationProviderErrorInfo{OperationProviderError::EntityNotFound, "Entity not found: " + entity_id, 404});
}

tl::expected<dto::OperationExecutionResult, OperationProviderErrorInfo>
OpcuaPlugin::execute_operation(const std::string & entity_id, const std::string & operation_name,
                               const nlohmann::json & parameters) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::Internal, "plugin not initialized", 503});
  }

  // Issue #386: acknowledge_fault and confirm_fault dispatch to OPC-UA
  // Method calls on the live ConditionId. AcknowledgeableConditionType
  // declares Acknowledge as method i=9111 and Confirm as method i=9113;
  // the objectId argument is the ConditionId NodeId of the live instance.
  if (operation_name == "acknowledge_fault" || operation_name == "confirm_fault") {
    if (!parameters.contains("fault_code") || !parameters["fault_code"].is_string()) {
      return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                                            "Missing required string parameter: fault_code", 400});
    }
    std::string fault_code = parameters["fault_code"].get<std::string>();
    std::string comment = parameters.value("comment", std::string{});

    // Input validation (bburda review on PR #387). ``fault_code`` lands in
    // log lines and HTTP error bodies via string concatenation; without
    // length / charset bounds an authenticated operator can inject
    // newlines, control chars, or multi-MB blobs into gateway logs.
    // ``comment`` is forwarded verbatim to the PLC as ``LocalizedText`` -
    // unbounded length lets the same role spam the PLC's condition log.
    // 256 chars matches the existing entity-id bound (``is_valid_path_segment``)
    // and is generous enough for typical ``PLC_OVERPRESSURE_SENSOR_3`` style
    // identifiers + a one-sentence operator comment.
    if (!is_valid_path_segment(fault_code)) {
      return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                                            "fault_code must be alphanumeric+_- and 1-256 chars (got " +
                                                                std::to_string(fault_code.size()) + " chars)",
                                                            400});
    }
    if (comment.size() > 256) {
      return tl::make_unexpected(
          OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                     "comment exceeds 256 chars (got " + std::to_string(comment.size()) + ")", 400});
    }

    auto runtime = poller_->lookup_condition(entity_id, fault_code);
    if (!runtime) {
      return tl::make_unexpected(OperationProviderErrorInfo{
          OperationProviderError::OperationNotFound,
          "No live condition for fault_code=" + fault_code + " on entity=" + entity_id, 404});
    }

    constexpr uint32_t kAcknowledgeMethodId = 9111;
    constexpr uint32_t kConfirmMethodId = 9113;
    opcua::NodeId method_id(0, operation_name == "acknowledge_fault" ? kAcknowledgeMethodId : kConfirmMethodId);

    std::vector<opcua::Variant> args;
    args.push_back(opcua::Variant::fromScalar(runtime->latest_event_id));
    args.push_back(opcua::Variant::fromScalar(opcua::LocalizedText("", comment)));

    if (plugin_debug_enabled()) {
      std::ostringstream hex_oss;
      const auto * bytes = runtime->latest_event_id.data();
      for (size_t i = 0; i < std::min<size_t>(runtime->latest_event_id.length(), 16); ++i) {
        char buf[3];
        std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned>(bytes[i]) & 0xffu);
        hex_oss << buf;
      }
      RCLCPP_DEBUG_STREAM(opcua_plugin_logger(), operation_name << " EventId len=" << runtime->latest_event_id.length()
                                                                << " hex=" << hex_oss.str()
                                                                << " conditionId=" << runtime->condition_id.toString());
    }

    auto result = client_->call_method(runtime->condition_id, method_id, args);
    if (!result.has_value()) {
      auto code = result.error().code;
      int http = 502;
      auto err = OperationProviderError::TransportError;
      if (code == OpcuaClient::MethodError::NotConnected) {
        http = 503;
        err = OperationProviderError::Internal;
      } else if (code == OpcuaClient::MethodError::MethodNotFound) {
        http = 404;
        err = OperationProviderError::OperationNotFound;
      } else if (code == OpcuaClient::MethodError::InvalidArgument) {
        http = 400;
        err = OperationProviderError::InvalidParameters;
      } else if (code == OpcuaClient::MethodError::MethodTimeout) {
        http = 504;
        err = OperationProviderError::TransportError;
      }
      return tl::make_unexpected(OperationProviderErrorInfo{err, result.error().message, http});
    }

    nlohmann::json out;
    out["status"] = "ok";
    out["operation"] = operation_name;
    out["fault_code"] = fault_code;
    out["entity_id"] = entity_id;
    out["condition_id"] = runtime->condition_id.toString();
    return dto::OperationExecutionResult{std::move(out)};
  }

  std::string data_name;
  if (operation_name.substr(0, 4) == "set_") {
    data_name = operation_name.substr(4);
  } else {
    data_name = operation_name;
  }

  // Held for as long as ``entry`` (a pointer into node_map_) is dereferenced,
  // including across the OPC-UA write below.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  auto * entry = node_map_.find_by_data_name(entity_id, data_name);
  if (!entry) {
    return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::OperationNotFound,
                                                          "Operation not found: " + operation_name, 404});
  }
  if (!entry->writable) {
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::Rejected, "Data point is read-only: " + data_name, 400});
  }
  if (!parameters.contains("value")) {
    return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                                          "Missing 'value' field in parameters", 400});
  }

  auto parsed = parse_coerce_validate(parameters["value"], *entry);
  if (!parsed) {
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::InvalidParameters, parsed.error(), 400});
  }

  auto write_result = client_->write_value(entry->node_id, *parsed, entry->data_type);
  if (!write_result) {
    auto wcode = write_result.error().code;
    if (wcode == OpcuaClient::WriteError::TypeMismatch) {
      return tl::make_unexpected(
          OperationProviderErrorInfo{OperationProviderError::InvalidParameters, write_result.error().message, 400});
    }
    if (wcode == OpcuaClient::WriteError::AccessDenied) {
      return tl::make_unexpected(
          OperationProviderErrorInfo{OperationProviderError::Rejected, write_result.error().message, 403});
    }
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::TransportError, write_result.error().message, 502});
  }

  nlohmann::json result;
  result["status"] = "ok";
  result["operation"] = operation_name;
  result["node_id"] = entry->node_id_str;
  std::visit(
      [&result](auto && v) {
        result["value_written"] = v;
      },
      *parsed);
  return dto::OperationExecutionResult{std::move(result)};
}

bool OpcuaPlugin::has_operations(const std::string & entity_id) const {
  // Mirrors list_operations' own lookup: true for any entity_defs() entry
  // (writable data points, or acknowledge_fault/confirm_fault on an
  // event-alarm entity), false for the Component, which has none.
  std::shared_lock<std::shared_mutex> node_map_lock(node_map_mutex_);
  const auto & defs = node_map_.entity_defs();
  return std::any_of(defs.begin(), defs.end(), [&entity_id](const PlcEntityDef & def) {
    return def.id == entity_id;
  });
}

// -- FaultProvider interface --

tl::expected<dto::FaultListResult, FaultProviderErrorInfo> OpcuaPlugin::list_faults(const std::string & entity_id) {
  if (!ctx_) {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::Internal, "plugin not initialized", 503});
  }

  // PluginContext::list_entity_faults returns a bare JSON array of fault
  // objects already scoped to this entity (see its contract). Project each into
  // the plugin fault-list item shape.
  auto faults = ctx_->list_entity_faults(entity_id);
  nlohmann::json items = nlohmann::json::array();
  if (faults.is_array()) {
    for (const auto & f : faults) {
      nlohmann::json item;
      item["code"] = f.value("fault_code", "");
      item["severity"] = f.value("severity", 0);
      item["description"] = f.value("description", "");
      item["status"] = f.value("status", "");
      // Fault records carry reporting_sources, not source_id. On this
      // entity-scoped list prefer the requested entity when it is among the
      // sources - reporting_sources[0] is ordering-dependent and can name a
      // different co-reporting entity; fall back to the first source only
      // when the entity itself never reported.
      std::string source_id = f.value("source_id", "");
      if (source_id.empty() && f.contains("reporting_sources") && f["reporting_sources"].is_array()) {
        for (const auto & src : f["reporting_sources"]) {
          if (src.is_string() && src.get<std::string>() == entity_id) {
            source_id = entity_id;
            break;
          }
        }
        if (source_id.empty() && !f["reporting_sources"].empty() && f["reporting_sources"][0].is_string()) {
          source_id = f["reporting_sources"][0].get<std::string>();
        }
      }
      item["source_id"] = source_id;
      // Pass occurrence data through: without it the entity-scoped route
      // serves faults with no timestamps ("time unknown" in clients) while the
      // instance-level route has them.
      for (const char * key : {"first_occurred", "last_occurred", "occurrence_count", "severity_label"}) {
        if (f.contains(key)) {
          item[key] = f[key];
        }
      }
      if (f.contains("reporting_sources")) {
        item["reporting_sources"] = f["reporting_sources"];
      }
      items.push_back(std::move(item));
    }
  }
  return dto::FaultListResult{nlohmann::json{{"items", std::move(items)}}};
}

tl::expected<dto::FaultDetailResult, FaultProviderErrorInfo> OpcuaPlugin::get_fault(const std::string & entity_id,
                                                                                    const std::string & fault_code) {
  if (!ctx_) {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::Internal, "plugin not initialized", 503});
  }

  // list_entity_faults returns a bare JSON array of fault objects scoped to
  // this entity (see PluginContext contract).
  auto faults = ctx_->list_entity_faults(entity_id);
  if (faults.is_array()) {
    for (const auto & f : faults) {
      if (f.value("fault_code", "") == fault_code) {
        return dto::FaultDetailResult{f};
      }
    }
  }

  return tl::make_unexpected(
      FaultProviderErrorInfo{FaultProviderError::FaultNotFound, "Fault not found: " + fault_code, 404});
}

tl::expected<dto::FaultClearResult, FaultProviderErrorInfo> OpcuaPlugin::clear_fault(const std::string & entity_id,
                                                                                     const std::string & fault_code) {
  if (!ctx_ || !fault_clients_) {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::Internal, "plugin not initialized", 503});
  }

  send_clear_fault(fault_code);
  return dto::FaultClearResult{
      nlohmann::json{{"status", "cleared"}, {"fault_code", fault_code}, {"entity_id", entity_id}}};
}

}  // namespace ros2_medkit_gateway
