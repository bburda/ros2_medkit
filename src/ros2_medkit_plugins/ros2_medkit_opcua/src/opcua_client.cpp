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

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <open62541/client_highlevel.h>
#include <open62541/client_subscriptions.h>
#include <open62541/types.h>
#include <open62541pp/config.hpp>
#ifdef UA_ENABLE_ENCRYPTION
#include <open62541/plugin/pki_default.h>
#endif
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>

namespace ros2_medkit_gateway {

// Per-event / per-method-call traces use a named logger so they integrate
// with the gateway's normal log level controls (e.g. RCUTILS_LOGGING_USE_STDOUT,
// ros2 launch --log-level opcua.client:=debug). Off at INFO by default; an
// operator turns them on without rebuilding (bburda review on PR #387).
namespace {
inline rclcpp::Logger opcua_client_logger() {
  static auto logger = rclcpp::get_logger("opcua.client");
  return logger;
}

// Pre-gate for traces whose stream-build cost is non-trivial (e.g. per-arg
// loops). RCLCPP_DEBUG_STREAM constructs the std::stringstream
// unconditionally, so for hot paths we check the effective level first.
// Uses the rcutils-level API (available in Humble through Lyrical) instead
// of rclcpp::Logger::get_effective_level (Jazzy+ only).
inline bool client_debug_enabled() {
  return rcutils_logging_logger_is_enabled_for("opcua.client", RCUTILS_LOG_SEVERITY_DEBUG);
}
}  // namespace

// Forward declaration - defined after Impl so the trampoline can call back into
// the client. Static linkage keeps the symbol private to this translation unit.
struct EventCallbackContext;
static void on_event_trampoline_c(UA_Client * client, UA_UInt32 sub_id, void * sub_ctx, UA_UInt32 mon_id,
                                  void * mon_ctx, size_t n_fields, UA_Variant * fields);

/// Heap-owned context passed to the open62541 C event callback. Lifetime is
/// owned by ``Impl::event_callbacks`` (unique_ptr); the raw pointer handed to C
/// is valid until ``remove_event_monitored_item`` or ``remove_subscriptions``
/// erases the entry.
///
/// Two staleness guards layered together:
/// - ``generation_snapshot`` filters callbacks fired from a defunct
///   subscription after the whole client reconnected (bumped on disconnect
///   and on ``remove_subscriptions``).
/// - ``active`` filters callbacks fired from a single monitored item that has
///   been individually removed via ``remove_event_monitored_item`` while
///   other items in the same subscription are still live. Per-MI flag,
///   not the global generation, so a single removal does not invalidate
///   peer callbacks.
struct EventCallbackContext {
  OpcuaClient * owner{nullptr};
  uint64_t generation_snapshot{0};
  std::atomic<bool> active{true};
  uint32_t subscription_id{0};
  uint32_t monitored_item_id{0};  // populated after createEvent returns
  OpcuaClient::EventCallback callback;
};

namespace {

/// Set the connected flag to false when the BadStatus code indicates a
/// terminal connection loss (as opposed to e.g. BadNodeIdUnknown which is a
/// per-node issue). Called from read_value, read_values and write_value so
/// that OpcuaPoller's reconnect logic (which keys off is_connected()) fires
/// regardless of which operation detected the drop first. Also bumps the
/// subscription generation so any in-flight event callbacks from the dying
/// subscription are filtered out by the trampoline.
void maybe_mark_disconnected(std::atomic<bool> & connected_flag, std::atomic<uint64_t> & generation,
                             const opcua::BadStatus & e) {
  const auto code = e.code();
  if (code == UA_STATUSCODE_BADCONNECTIONCLOSED || code == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
      code == UA_STATUSCODE_BADNOTCONNECTED) {
    if (connected_flag.exchange(false)) {
      generation.fetch_add(1, std::memory_order_release);
    }
  }
}

OpcuaValue variant_to_value(const opcua::Variant & var) {
  if (var.isEmpty()) {
    return std::string("<empty>");
  }
  if (var.isType<bool>()) {
    return var.getScalarCopy<bool>();
  }
  if (var.isType<int16_t>()) {
    return static_cast<int32_t>(var.getScalarCopy<int16_t>());
  }
  if (var.isType<uint16_t>()) {
    return static_cast<int32_t>(var.getScalarCopy<uint16_t>());
  }
  if (var.isType<int32_t>()) {
    return var.getScalarCopy<int32_t>();
  }
  if (var.isType<uint32_t>()) {
    return static_cast<int64_t>(var.getScalarCopy<uint32_t>());
  }
  if (var.isType<int64_t>()) {
    return var.getScalarCopy<int64_t>();
  }
  if (var.isType<uint64_t>()) {
    // OpcuaValue has no unsigned 64-bit slot; reinterpret the bit pattern as
    // int64_t. A plain static_cast of a value above INT64_MAX is only
    // implementation-defined under C++17, so copy the bits via memcpy to make
    // the reinterpretation well-defined. The raw bits are preserved for
    // status-word bit decode (which masks them) and for fault-enum codes that
    // fit in int64_t; codes above INT64_MAX read as negative in enum mode.
    const uint64_t raw = var.getScalarCopy<uint64_t>();
    int64_t signed_bits;
    std::memcpy(&signed_bits, &raw, sizeof(signed_bits));
    return signed_bits;
  }
  if (var.isType<float>()) {
    return var.getScalarCopy<float>();
  }
  if (var.isType<double>()) {
    return var.getScalarCopy<double>();
  }
  if (var.isType<opcua::String>()) {
    return std::string(var.getScalarCopy<opcua::String>());
  }
  if (var.isType<opcua::DateTime>()) {
    // Surfaced as an ISO-8601 UTC string (medkit has no native timestamp
    // OpcuaValue slot). Exercised by auto_browse's DateTime->"string"
    // mapping (e.g. a "TIMESTAMP" field on an alarm DB) and by any
    // hand-written node_map entry that points at a DateTime node.
    const auto tp = var.getScalarCopy<opcua::DateTime>().toTimePoint<std::chrono::system_clock>();
    const std::time_t tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm_utc{};
    if (gmtime_r(&tt, &tm_utc) == nullptr) {
      // Out-of-range timestamp (e.g. a corrupt/garbage DateTime value from
      // the server) - surface as unsupported rather than formatting an
      // uninitialized std::tm.
      return std::string("<invalid DateTime>");
    }
    char buf[32];
    const size_t n = std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    return std::string(buf, n);
  }
  return std::string("<unsupported type>");
}

// Lower-case copy for case-insensitive config parsing.
std::string to_lower(const std::string & s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

// Read a whole file into an opcua::ByteString (binary-safe). Returns an empty
// ByteString on failure; the caller treats empty cert/key as a config error.
opcua::ByteString read_file_bytes(const std::string & path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return opcua::ByteString{};
  }
  std::string data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  return opcua::ByteString(std::string_view(data));
}

opcua::Variant value_to_variant(const OpcuaValue & val) {
  return std::visit(
      [](auto && v) -> opcua::Variant {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::string>) {
          return opcua::Variant::fromScalar(opcua::String(v));
        } else if constexpr (std::is_same_v<T, float>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, double>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, bool>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, int32_t>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, int64_t>) {
          return opcua::Variant::fromScalar(v);
        } else {
          return opcua::Variant::fromScalar(v);
        }
      },
      val);
}

}  // namespace

struct OpcuaClient::Impl {
  opcua::Client client;
  OpcuaClientConfig config;
  std::atomic<bool> connected{false};
  mutable std::mutex client_mutex;
  std::string server_desc;

  // Subscription tracking - store actual Subscription objects
  struct SubInfo {
    opcua::Subscription<opcua::Client> sub;
    DataChangeCallback callback;
  };
  std::deque<SubInfo> subscriptions;  // deque for stable references (callbacks capture &cb)
  std::mutex sub_mutex;

  // Issue #386: native OPC-UA AlarmCondition event subscription support.
  //
  // generation increments whenever the connection drops or is closed. The
  // event trampoline captures a snapshot at createEvent time and drops
  // notifications when the snapshot diverges from the live counter, so
  // late-arriving events from a defunct subscription cannot reach user code.
  std::atomic<uint64_t> generation{0};

  // Successful-session counter: bumped once per fresh connect, never on the
  // already-connected fast path. Exposed via connection_generation() so the
  // plugin can detect a poller reconnect and refresh per-session state
  // (device-info nameplate) without re-reading on every introspect.
  std::atomic<uint64_t> connect_generation{0};

  // Heap-owned contexts for raw-C event monitored items. unique_ptr keeps
  // the ctx alive exactly as long as the entry sits in the map; we release
  // entries only after the server ACKs DeleteMonitoredItem (or when we tear
  // down the entire client in disconnect()). Keyed by monitored_item_id; we
  // store sub_id alongside in the ctx so cleanup can address the right
  // subscription.
  std::unordered_map<uint32_t, std::unique_ptr<EventCallbackContext>> event_callbacks;
  std::mutex event_callbacks_mutex;
};

OpcuaClient::OpcuaClient() : impl_(std::make_unique<Impl>()) {
}

OpcuaClient::~OpcuaClient() {
  disconnect();
}

// --- Security config helpers (pure / unit-testable) ---

SecurityPolicy OpcuaClient::parse_security_policy(const std::string & name, bool * ok) {
  if (ok) {
    *ok = true;
  }
  const std::string n = to_lower(name);
  if (n.empty() || n == "none") {
    return SecurityPolicy::None;
  }
  if (n == "basic256sha256") {
    return SecurityPolicy::Basic256Sha256;
  }
  if (n == "aes128sha256rsaoaep" || n == "aes128_sha256_rsaoaep") {
    return SecurityPolicy::Aes128Sha256RsaOaep;
  }
  if (n == "aes256sha256rsapss" || n == "aes256_sha256_rsapss") {
    return SecurityPolicy::Aes256Sha256RsaPss;
  }
  if (ok) {
    *ok = false;
  }
  return SecurityPolicy::None;
}

std::string OpcuaClient::security_policy_uri(SecurityPolicy policy) {
  switch (policy) {
    case SecurityPolicy::None:
      return "http://opcfoundation.org/UA/SecurityPolicy#None";
    case SecurityPolicy::Basic256Sha256:
      return "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256";
    case SecurityPolicy::Aes128Sha256RsaOaep:
      return "http://opcfoundation.org/UA/SecurityPolicy#Aes128_Sha256_RsaOaep";
    case SecurityPolicy::Aes256Sha256RsaPss:
      return "http://opcfoundation.org/UA/SecurityPolicy#Aes256_Sha256_RsaPss";
  }
  return "http://opcfoundation.org/UA/SecurityPolicy#None";
}

SecurityMode OpcuaClient::parse_security_mode(const std::string & name, bool * ok) {
  if (ok) {
    *ok = true;
  }
  const std::string n = to_lower(name);
  if (n.empty() || n == "none") {
    return SecurityMode::None;
  }
  if (n == "sign") {
    return SecurityMode::Sign;
  }
  if (n == "signandencrypt" || n == "sign_and_encrypt") {
    return SecurityMode::SignAndEncrypt;
  }
  if (ok) {
    *ok = false;
  }
  return SecurityMode::None;
}

UserAuthMode OpcuaClient::parse_user_auth_mode(const std::string & name, bool * ok) {
  if (ok) {
    *ok = true;
  }
  const std::string n = to_lower(name);
  if (n.empty() || n == "anonymous") {
    return UserAuthMode::Anonymous;
  }
  if (n == "username" || n == "usernamepassword" || n == "username_password" || n == "password") {
    return UserAuthMode::UsernamePassword;
  }
  if (n == "x509" || n == "certificate" || n == "cert") {
    return UserAuthMode::X509;
  }
  if (ok) {
    *ok = false;
  }
  return UserAuthMode::Anonymous;
}

bool OpcuaClient::requires_secure_channel(const OpcuaClientConfig & config) {
  return config.security_policy != SecurityPolicy::None || config.security_mode != SecurityMode::None;
}

bool OpcuaClient::credentials_sent_in_clear(const OpcuaClientConfig & config) {
  return config.user_auth_mode != UserAuthMode::Anonymous && !requires_secure_channel(config);
}

bool OpcuaClient::security_config_conflict(const OpcuaClientConfig & config) {
  const bool policy_none = config.security_policy == SecurityPolicy::None;
  const bool mode_none = config.security_mode == SecurityMode::None;
  return policy_none != mode_none;
}

namespace {

#ifdef UA_ENABLE_ENCRYPTION
opcua::MessageSecurityMode to_ua_security_mode(SecurityMode mode) {
  switch (mode) {
    case SecurityMode::None:
      return opcua::MessageSecurityMode::None;
    case SecurityMode::Sign:
      return opcua::MessageSecurityMode::Sign;
    case SecurityMode::SignAndEncrypt:
      return opcua::MessageSecurityMode::SignAndEncrypt;
  }
  return opcua::MessageSecurityMode::None;
}
#endif

// Rebuild ``client`` with the SecureChannel + user-identity profile requested
// by ``cfg``. A fresh ClientConfig is mandatory for secured connections
// because the application-instance certificate and trust list can only be
// supplied at construction (UA_ClientConfig_setDefaultEncryption). Returns
// false on a fatal configuration error (already logged); the caller then
// reports the connect as failed without contacting the server.
bool apply_security_config(opcua::Client & client, const OpcuaClientConfig & cfg) {
  // Reject a contradictory SecurityPolicy / MessageSecurityMode pairing before
  // building anything. Accepting e.g. security_mode=Sign with
  // security_policy=None would leave the policy URI unpinned and silently
  // defeat the explicit-selection guarantee below (OPC-UA Part 4 §7.37).
  if (OpcuaClient::security_config_conflict(cfg)) {
    const bool policy_none = cfg.security_policy == SecurityPolicy::None;
    RCLCPP_ERROR(opcua_client_logger(),
                 "OPC-UA security config is contradictory: %s. security_policy and "
                 "message_security_mode must both be None or both be set. Refusing to connect.",
                 policy_none ? "message_security_mode is Sign/SignAndEncrypt but security_policy is None"
                             : "security_policy is set but message_security_mode is None");
    return false;
  }

  const bool secure = OpcuaClient::requires_secure_channel(cfg);

  if (!secure) {
    // Unsecured channel (SecurityPolicy=None, MessageSecurityMode=None). A
    // username/password or X.509 identity may still be applied below.
    client = opcua::Client();
  } else {
#ifndef UA_ENABLE_ENCRYPTION
    RCLCPP_ERROR(opcua_client_logger(),
                 "OPC-UA SecurityPolicy/MessageSecurityMode requested but this build has no "
                 "encryption support (UA_ENABLE_ENCRYPTION is off)");
    return false;
#else
    if (cfg.client_cert_path.empty() || cfg.client_key_path.empty()) {
      RCLCPP_ERROR(opcua_client_logger(), "OPC-UA secured connection requires client_cert_path and client_key_path");
      return false;
    }
    auto cert = read_file_bytes(cfg.client_cert_path);
    auto key = read_file_bytes(cfg.client_key_path);
    if (cert.empty() || key.empty()) {
      RCLCPP_ERROR(opcua_client_logger(), "OPC-UA: failed to read client cert/key ('%s' / '%s')",
                   cfg.client_cert_path.c_str(), cfg.client_key_path.c_str());
      return false;
    }

    std::vector<opcua::ByteString> trust;
    trust.reserve(cfg.trust_list_paths.size());
    for (const auto & p : cfg.trust_list_paths) {
      auto b = read_file_bytes(p);
      if (b.empty()) {
        RCLCPP_WARN(opcua_client_logger(), "OPC-UA: skipping unreadable trust-list entry '%s'", p.c_str());
        continue;
      }
      trust.push_back(std::move(b));
    }

    opcua::ClientConfig cc(cert, key, trust, {});

    // Force the requested SecurityPolicy. An empty URI lets open62541 pick the
    // highest the server offers; we want explicit selection so a downgraded
    // server endpoint cannot silently weaken the channel.
    const std::string policy_uri = OpcuaClient::security_policy_uri(cfg.security_policy);
    if (cfg.security_policy != SecurityPolicy::None && !policy_uri.empty()) {
      UA_String_clear(&cc.handle()->securityPolicyUri);
      cc.handle()->securityPolicyUri = UA_STRING_ALLOC(policy_uri.c_str());
    }

    cc.setSecurityMode(to_ua_security_mode(cfg.security_mode));

    // The application URI must match the URI SAN in the client certificate or
    // the server rejects the channel with BadCertificateUriInvalid.
    if (!cfg.application_uri.empty()) {
      UA_String_clear(&cc.handle()->clientDescription.applicationUri);
      cc.handle()->clientDescription.applicationUri = UA_STRING_ALLOC(cfg.application_uri.c_str());
    }

    // Trust handling: by default validate the server certificate against the
    // trust list. When reject_untrusted is false, accept any server certificate
    // (accept-any: INSECURE, lab only). This is NOT trust-on-first-use - nothing
    // is pinned, so a substituted or expired server cert is accepted on every
    // connect with no chain / hostname / URI-SAN check.
    if (!cfg.reject_untrusted) {
      auto & cv = cc.handle()->certificateVerification;
      if (cv.clear) {
        cv.clear(&cv);
      }
      UA_CertificateVerification_AcceptAll(&cv);
    }

    client = opcua::Client(std::move(cc));
#endif
  }

  // open62541 1.3 security policies store &config->logger, and the configs
  // built above are moved into the client, leaving those pointers dangling
  // (segfault on the first policy log line, e.g. SecureChannel creation).
  // open62541pp re-points them only for <= 1.2; 1.4+ uses a heap logger.
#if defined(UA_OPEN62541_VER_MAJOR) && UA_OPEN62541_VER_MAJOR == 1 && UA_OPEN62541_VER_MINOR == 3
  {
    UA_ClientConfig * final_config = client.config().handle();
    for (size_t i = 0; i < final_config->securityPoliciesSize; ++i) {
      final_config->securityPolicies[i].logger = &final_config->logger;
    }
  }
#endif

  // User identity token (applied regardless of channel encryption).
  switch (cfg.user_auth_mode) {
    case UserAuthMode::Anonymous:
      client.config().setUserIdentityToken(opcua::AnonymousIdentityToken{});
      break;
    case UserAuthMode::UsernamePassword:
      client.config().setUserIdentityToken(opcua::UserNameIdentityToken(cfg.username, cfg.password));
      break;
    case UserAuthMode::X509: {
#ifdef UA_ENABLE_ENCRYPTION
      auto ucert = read_file_bytes(cfg.user_cert_path);
      if (ucert.empty()) {
        RCLCPP_ERROR(opcua_client_logger(), "OPC-UA X.509 user auth: failed to read user_cert_path '%s'",
                     cfg.user_cert_path.c_str());
        return false;
      }
      client.config().setUserIdentityToken(opcua::X509IdentityToken(std::move(ucert)));
#else
      RCLCPP_ERROR(opcua_client_logger(), "OPC-UA X.509 user auth requires an encryption-enabled build");
      return false;
#endif
      break;
    }
  }
  return true;
}

}  // namespace

bool OpcuaClient::connect(const OpcuaClientConfig & config) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  impl_->config = config;

  try {
    if (impl_->client.isConnected()) {
      impl_->connected = true;
      return true;
    }

    // (Re)build the client with the requested security profile before every
    // connect so reconnects re-apply the same SecureChannel settings.
    if (!apply_security_config(impl_->client, config)) {
      impl_->connected = false;
      return false;
    }

    impl_->client.config().setTimeout(static_cast<uint32_t>(config.connect_timeout.count()));
    impl_->client.connect(config.endpoint_url);
    impl_->connected = true;
    impl_->connect_generation.fetch_add(1, std::memory_order_release);

    try {
      opcua::Node node(impl_->client, opcua::NodeId(0, UA_NS0ID_SERVER_SERVERSTATUS_BUILDINFO_PRODUCTNAME));
      auto val = node.readValue();
      if (val.isType<opcua::String>()) {
        impl_->server_desc = std::string(val.getScalarCopy<opcua::String>());
      }
    } catch (...) {
      impl_->server_desc = "OPC-UA Server";
    }

    return true;
  } catch (const opcua::BadStatus & e) {
    RCLCPP_WARN(opcua_client_logger(), "OPC-UA connect to '%s' failed: %s", config.endpoint_url.c_str(), e.what());
    impl_->connected = false;
    return false;
  }
}

void OpcuaClient::disconnect() {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  if (impl_->connected) {
    // Bump generation FIRST so any in-flight event callbacks fired from the
    // dying subscription drop their work in the trampoline (they read
    // generation atomically) before we touch the storage they reference.
    // The ``if (impl_->connected)`` guard ensures we bump exactly once even
    // when ``maybe_mark_disconnected`` already fired earlier on a transport
    // error path - that helper uses ``exchange(false)`` and would have
    // already bumped, leaving impl_->connected = false here.
    impl_->generation.fetch_add(1, std::memory_order_release);
    try {
      // Issue #386: clear event monitored items BEFORE deleting subscriptions.
      // open62541's deleteSubscription cleans up server-side, but our
      // EventCallbackContext objects (held in unique_ptr) must outlive any
      // pending C callbacks; the generation bump above already filters them
      // out, so it is safe to drop the contexts here.
      {
        std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
        for (auto & [mi_id, ctx] : impl_->event_callbacks) {
          UA_Client_MonitoredItems_deleteSingle(impl_->client.handle(), ctx->subscription_id, mi_id);
        }
        impl_->event_callbacks.clear();
      }
      // Delete subscriptions
      {
        std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);
        for (auto & info : impl_->subscriptions) {
          try {
            info.sub.deleteSubscription();
          } catch (...) {
          }
        }
        impl_->subscriptions.clear();
      }
      impl_->client.disconnect();
    } catch (...) {
    }
    impl_->connected = false;
  }
}

bool OpcuaClient::is_connected() const {
  return impl_->connected.load();
}

uint64_t OpcuaClient::connection_generation() const {
  return impl_->connect_generation.load(std::memory_order_acquire);
}

OpcuaClientConfig OpcuaClient::current_config() const {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  return impl_->config;
}

std::string OpcuaClient::endpoint_url() const {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  return impl_->config.endpoint_url;
}

std::vector<std::string> OpcuaClient::browse(const opcua::NodeId & parent_node) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  std::vector<std::string> result;

  if (!impl_->connected) {
    return result;
  }

  try {
    opcua::Node node(impl_->client, parent_node);
    auto children = node.browseChildren();
    for (auto & child : children) {
      result.push_back(child.id().toString());
    }
  } catch (const opcua::BadStatus &) {
  }

  return result;
}

namespace {

// Resolve a ns=0 numeric DataType identifier to its OPC-UA builtin scalar
// type name. Only the types the plugin's four medkit data types
// (bool/int/float/string) can represent are named here; every other
// identifier (structured/enumerated/vendor types, or anything not in this
// table) returns empty so auto_browse skips the variable as unsupported.
//
// A lookup table (not a switch on opcua::DataTypeId) deliberately: that enum
// has hundreds of generated members (every OPC-UA Part 6 DataType, most
// irrelevant to a scalar PLC point) and this project's ROS2MedkitWarnings
// promotes -Wswitch-enum to an error, which would force listing all of them.
std::string builtin_data_type_name(uint32_t numeric_id) {
  static const std::unordered_map<uint32_t, std::string> kBuiltinNames = {
      {static_cast<uint32_t>(opcua::DataTypeId::Boolean), "Boolean"},
      {static_cast<uint32_t>(opcua::DataTypeId::SByte), "SByte"},
      {static_cast<uint32_t>(opcua::DataTypeId::Byte), "Byte"},
      {static_cast<uint32_t>(opcua::DataTypeId::Int16), "Int16"},
      {static_cast<uint32_t>(opcua::DataTypeId::UInt16), "UInt16"},
      {static_cast<uint32_t>(opcua::DataTypeId::Int32), "Int32"},
      {static_cast<uint32_t>(opcua::DataTypeId::UInt32), "UInt32"},
      {static_cast<uint32_t>(opcua::DataTypeId::Int64), "Int64"},
      {static_cast<uint32_t>(opcua::DataTypeId::UInt64), "UInt64"},
      {static_cast<uint32_t>(opcua::DataTypeId::Float), "Float"},
      {static_cast<uint32_t>(opcua::DataTypeId::Double), "Double"},
      {static_cast<uint32_t>(opcua::DataTypeId::String), "String"},
      {static_cast<uint32_t>(opcua::DataTypeId::DateTime), "DateTime"},
  };
  const auto it = kBuiltinNames.find(numeric_id);
  return it == kBuiltinNames.end() ? std::string{} : it->second;
}

}  // namespace

std::vector<OpcuaClient::BrowseChild> OpcuaClient::browse_detailed(const opcua::NodeId & parent_node) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  std::vector<BrowseChild> result;

  if (!impl_->connected) {
    return result;
  }

  try {
    opcua::Node node(impl_->client, parent_node);
    // Server-side node-class filter: only Object/Variable references come
    // back, so Methods (ConditionRefresh, Acknowledge, ...), ReferenceTypes,
    // DataTypes etc. never enter the walk. BrowseResultMask::All returns
    // BrowseName + DisplayName + NodeClass in the same round trip as the
    // NodeId, so the caller never needs a follow-up read per child.
    auto refs = node.browseReferences(opcua::BrowseDirection::Forward, opcua::ReferenceTypeId::HierarchicalReferences,
                                      true, opcua::NodeClass::Object | opcua::NodeClass::Variable);
    result.reserve(refs.size());
    for (auto & ref : refs) {
      const auto & expanded = ref.getNodeId();
      if (!expanded.isLocal()) {
        continue;  // reference to another server - nothing to read here
      }
      BrowseChild child;
      child.node_id = expanded.getNodeId();
      const auto bn = ref.getBrowseName();
      child.browse_name_ns = bn.getNamespaceIndex();
      child.browse_name = std::string(bn.getName());
      child.display_name = std::string(ref.getDisplayName().getText());
      child.node_class = ref.getNodeClass();
      result.push_back(std::move(child));
    }
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
  }

  return result;
}

std::string OpcuaClient::read_variable_type_name(const opcua::NodeId & variable_node) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return {};
  }

  try {
    opcua::Node node(impl_->client, variable_node);
    const auto data_type = node.readDataType();
    if (data_type.getNamespaceIndex() != 0 || data_type.getIdentifierType() != opcua::NodeIdType::Numeric) {
      return {};  // vendor/custom/enum type - not a scalar builtin
    }
    return builtin_data_type_name(data_type.getIdentifierAs<uint32_t>());
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
    return {};
  }
}

OpcuaClient::AccessLevelInfo OpcuaClient::read_access_level(const opcua::NodeId & variable_node) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  AccessLevelInfo info;

  if (!impl_->connected) {
    return info;
  }

  try {
    opcua::Node node(impl_->client, variable_node);
    const opcua::Bitmask<opcua::AccessLevel> access = node.readAccessLevel();
    info.access_level = access.get();
    bool have_user = false;
    opcua::Bitmask<opcua::AccessLevel> user = access;
    try {
      user = node.readUserAccessLevel();
      info.user_access_level = user.get();
      have_user = true;
    } catch (const opcua::BadStatus &) {
      // Not every server exposes UserAccessLevel; fall back to AccessLevel.
      info.user_access_level = info.access_level;
    }
    const auto & effective = have_user ? user : access;
    info.readable = effective.anyOf(opcua::AccessLevel::CurrentRead);
    info.writable = effective.anyOf(opcua::AccessLevel::CurrentWrite);
    info.ok = true;
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
  }

  return info;
}

ReadResult OpcuaClient::read_value(const opcua::NodeId & node_id) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  ReadResult result;
  result.node_id = node_id.toString();
  result.timestamp = std::chrono::system_clock::now();

  if (!impl_->connected) {
    result.good = false;
    return result;
  }

  try {
    opcua::Node node(impl_->client, node_id);
    auto val = node.readValue();
    result.value = variant_to_value(val);
    result.good = true;
  } catch (const opcua::BadStatus & e) {
    result.good = false;
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
  }

  return result;
}

std::vector<ReadResult> OpcuaClient::read_values(const std::vector<opcua::NodeId> & node_ids) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  std::vector<ReadResult> results;
  results.reserve(node_ids.size());
  auto now = std::chrono::system_clock::now();

  if (!impl_->connected) {
    for (const auto & nid : node_ids) {
      results.push_back({nid.toString(), {}, now, false});
    }
    return results;
  }

  for (const auto & nid : node_ids) {
    ReadResult r;
    r.node_id = nid.toString();
    r.timestamp = now;
    try {
      opcua::Node node(impl_->client, nid);
      r.value = variant_to_value(node.readValue());
      r.good = true;
    } catch (const opcua::BadStatus & e) {
      r.good = false;
      maybe_mark_disconnected(impl_->connected, impl_->generation, e);
    }
    results.push_back(std::move(r));
  }
  return results;
}

tl::expected<void, OpcuaClient::WriteErrorInfo>
OpcuaClient::write_value(const opcua::NodeId & node_id, const OpcuaValue & value, const std::string & data_type_hint) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return tl::make_unexpected(WriteErrorInfo{WriteError::NotConnected, "Not connected to OPC-UA server"});
  }

  // Helper: coerce OpcuaValue to double for numeric writes
  auto to_double = [](const OpcuaValue & v) -> double {
    return std::visit(
        [](auto && x) -> double {
          if constexpr (std::is_arithmetic_v<std::decay_t<decltype(x)>>) {
            return static_cast<double>(x);
          }
          return 0.0;
        },
        v);
  };

  // Helper: coerce OpcuaValue to int64 (widest integer) for width dispatch
  auto to_int64 = [](const OpcuaValue & v) -> int64_t {
    return std::visit(
        [](auto && x) -> int64_t {
          if constexpr (std::is_arithmetic_v<std::decay_t<decltype(x)>>) {
            return static_cast<int64_t>(x);
          }
          return 0;
        },
        v);
  };

  // Helper: coerce OpcuaValue to bool
  auto to_bool = [](const OpcuaValue & v) -> bool {
    return std::visit(
        [](auto && x) -> bool {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_integral_v<T>) {
            return x != 0;
          } else if constexpr (std::is_floating_point_v<T>) {
            return x > T{0} || x < T{0};
          }
          return false;
        },
        v);
  };

  // The coarse "int" NodeMap hint does not carry the OPC-UA integer width, so
  // a fixed Int32 write is rejected (BadTypeMismatch) by narrower nodes such as
  // the S7 "Int" (Int16). Probe the node's real DataType and write the matching
  // scalar width; fall back to Int32 when the type cannot be resolved.
  auto write_integer_typed = [&to_int64, &value](auto & node) -> tl::expected<void, WriteErrorInfo> {
    uint32_t dt = 0;
    try {
      const auto data_type = node.readDataType();
      if (data_type.getNamespaceIndex() == 0 && data_type.getIdentifierType() == opcua::NodeIdType::Numeric) {
        // node is a generic (auto) parameter, so data_type is a dependent type.
        dt = data_type.template getIdentifierAs<uint32_t>();
      }
    } catch (...) {
      // leave dt=0 -> Int32 default
    }
    const int64_t iv = to_int64(value);
    // if-else over numeric ids (not a switch: -Werror=switch-enum would demand
    // every DataTypeId enumerator be listed).
    auto id = [](opcua::DataTypeId d) {
      return static_cast<uint32_t>(d);
    };
    // Range-check before narrowing: an unchecked static_cast wraps (40000 ->
    // Int16 = -25536, any negative -> unsigned = a huge value), and write_data
    // would then echo the wrapped number back as a successful value_written.
    // Reject out-of-range as a TypeMismatch (write_data maps it to HTTP 400).
    auto put = [&node, iv](auto type_tag, const char * type_name) -> tl::expected<void, WriteErrorInfo> {
      using T = decltype(type_tag);
      bool out_of_range = false;
      if constexpr (std::is_signed_v<T>) {
        out_of_range = iv < static_cast<int64_t>(std::numeric_limits<T>::min()) ||
                       iv > static_cast<int64_t>(std::numeric_limits<T>::max());
      } else {
        out_of_range = iv < 0 || static_cast<uint64_t>(iv) > static_cast<uint64_t>(std::numeric_limits<T>::max());
      }
      if (out_of_range) {
        return tl::make_unexpected(
            WriteErrorInfo{WriteError::TypeMismatch, std::to_string(iv) + " out of range for " + type_name});
      }
      node.writeValueScalar(static_cast<T>(iv));
      return {};
    };
    if (dt == id(opcua::DataTypeId::SByte)) {
      return put(int8_t{}, "SByte");
    } else if (dt == id(opcua::DataTypeId::Byte)) {
      return put(uint8_t{}, "Byte");
    } else if (dt == id(opcua::DataTypeId::Int16)) {
      return put(int16_t{}, "Int16");
    } else if (dt == id(opcua::DataTypeId::UInt16)) {
      return put(uint16_t{}, "UInt16");
    } else if (dt == id(opcua::DataTypeId::UInt32)) {
      return put(uint32_t{}, "UInt32");
    } else if (dt == id(opcua::DataTypeId::Int64)) {
      return put(int64_t{}, "Int64");
    } else if (dt == id(opcua::DataTypeId::UInt64)) {
      return put(uint64_t{}, "UInt64");
    } else {
      return put(int32_t{}, "Int32");
    }
  };

  try {
    opcua::Node node(impl_->client, node_id);

    // Fast path: use the data_type_hint from NodeMap instead of probing
    // the server with a readValue round-trip. Halves mutex hold time.
    if (!data_type_hint.empty()) {
      if (data_type_hint == "float") {
        node.writeValueScalar(static_cast<float>(to_double(value)));
      } else if (data_type_hint == "int") {
        if (auto r = write_integer_typed(node); !r) {
          return tl::make_unexpected(r.error());
        }
      } else if (data_type_hint == "bool") {
        node.writeValueScalar(to_bool(value));
      } else if (data_type_hint == "string") {
        auto sval = std::visit(
            [](auto && x) -> std::string {
              using T = std::decay_t<decltype(x)>;
              if constexpr (std::is_same_v<T, std::string>) {
                return x;
              } else {
                return std::to_string(x);
              }
            },
            value);
        node.writeValueScalar(opcua::String(sval));
      } else {
        // Unknown hint - fall through to probe path
        node.writeValueScalar(static_cast<float>(to_double(value)));
      }
      return {};
    }

    // Slow path: read current value to discover expected type, then write.
    auto current = node.readValue();

    if (current.isType<float>()) {
      node.writeValueScalar(static_cast<float>(to_double(value)));
    } else if (current.isType<double>()) {
      node.writeValueScalar(to_double(value));
    } else if (current.isType<bool>()) {
      node.writeValueScalar(to_bool(value));
    } else if (current.isType<int8_t>() || current.isType<uint8_t>() || current.isType<int16_t>() ||
               current.isType<uint16_t>() || current.isType<int32_t>() || current.isType<uint32_t>() ||
               current.isType<int64_t>() || current.isType<uint64_t>()) {
      if (auto r = write_integer_typed(node); !r) {
        return tl::make_unexpected(r.error());
      }
    } else {
      auto var = value_to_variant(value);
      node.writeValue(var);
    }
    return {};
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
    auto code = e.code();
    if (code == UA_STATUSCODE_BADTYPEMISMATCH) {
      return tl::make_unexpected(WriteErrorInfo{WriteError::TypeMismatch, e.what()});
    }
    if (code == UA_STATUSCODE_BADUSERACCESSDENIED || code == UA_STATUSCODE_BADNOTWRITABLE) {
      return tl::make_unexpected(WriteErrorInfo{WriteError::AccessDenied, e.what()});
    }
    if (code == UA_STATUSCODE_BADNODEIDUNKNOWN) {
      return tl::make_unexpected(WriteErrorInfo{WriteError::NodeNotFound, e.what()});
    }
    return tl::make_unexpected(WriteErrorInfo{WriteError::TransportError, e.what()});
  }
}

uint32_t OpcuaClient::create_subscription(double publish_interval_ms, DataChangeCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return 0;
  }

  try {
    opcua::SubscriptionParameters params{};
    params.publishingInterval = publish_interval_ms;

    auto sub = impl_->client.createSubscription(params);
    uint32_t sub_id = sub.subscriptionId();

    std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);
    impl_->subscriptions.push_back({sub, std::move(callback)});

    return sub_id;
  } catch (const opcua::BadStatus &) {
    return 0;
  }
}

bool OpcuaClient::add_monitored_item(uint32_t subscription_id, const opcua::NodeId & node_id) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return false;
  }

  std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);
  // Find the subscription by ID
  for (auto & info : impl_->subscriptions) {
    if (info.sub.subscriptionId() == subscription_id) {
      try {
        auto nid_str = node_id.toString();
        auto cb_copy = info.callback;  // capture by value to avoid use-after-free

        info.sub.subscribeDataChange(
            node_id, opcua::AttributeId::Value,
            [nid_str, cb_copy](uint32_t /*sub_id*/, uint32_t /*mon_id*/, const opcua::DataValue & dv) {
              if (dv.hasValue()) {
                auto val = variant_to_value(dv.getValue());
                cb_copy(nid_str, val);
              }
            });
        return true;
      } catch (const opcua::BadStatus &) {
        return false;
      }
    }
  }
  return false;
}

void OpcuaClient::remove_subscriptions() {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  // Issue #386: bump generation so the trampoline drops any callback fired
  // from the now-defunct subscription before we erase its EventCallbackContext.
  impl_->generation.fetch_add(1, std::memory_order_release);

  // Clear event monitored items BEFORE the open62541pp Subscription destructor
  // runs (subscription deletion cascades server-side, but the C callback's
  // userdata must remain valid until its last possible invocation).
  {
    std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
    for (auto & [mi_id, ctx] : impl_->event_callbacks) {
      UA_Client_MonitoredItems_deleteSingle(impl_->client.handle(), ctx->subscription_id, mi_id);
    }
    impl_->event_callbacks.clear();
  }

  std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);
  for (auto & info : impl_->subscriptions) {
    try {
      info.sub.deleteSubscription();
    } catch (...) {
    }
  }
  impl_->subscriptions.clear();
}

std::string OpcuaClient::server_description() const {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  return impl_->server_desc;
}

std::vector<OpcuaClient::ConditionStateSnapshot> OpcuaClient::read_source_conditions(const opcua::NodeId & source_node,
                                                                                     bool * scan_ok) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  std::vector<ConditionStateSnapshot> out;
  // Default to "scan failed" so an early return (disconnected) or a thrown
  // browse never looks like a clean "no conditions" result to the caller.
  if (scan_ok) {
    *scan_ok = false;
  }
  if (!impl_->connected) {
    return out;
  }

  // A browse failure during state-variable resolution is "path absent" only
  // for the explicit not-found status codes. Anything else (timeout, channel
  // drop, server busy) is a transient failure that must NOT be read as "this
  // child is not a condition" - otherwise a momentary glitch silently drops a
  // live condition and lets reconcile clear its fault.
  auto is_path_absent_code = [](UA_StatusCode code) {
    return code == UA_STATUSCODE_BADNOMATCH || code == UA_STATUSCODE_BADNODEIDUNKNOWN ||
           code == UA_STATUSCODE_BADNOTFOUND || code == UA_STATUSCODE_BADBROWSENAMEINVALID;
  };

  // Read a Boolean two-step state variable (e.g. ActiveState/Id). Returns the
  // fallback when the path is absent or not Boolean. ``resolved`` reports
  // whether the ``state_name/Id`` path browsed successfully (used to decide if
  // a child is a condition); ``read_failed`` reports that the path resolved but
  // its value could not be read this scan (transient failure, distinct from the
  // path simply not existing). A transient BROWSE failure (as opposed to a
  // genuine path-absent code) is also treated as resolved+read_failed so the
  // condition is kept and flagged rather than dropped (issue #478).
  auto read_state_id = [&is_path_absent_code](opcua::Node<opcua::Client> & cond, const char * state_name, bool fallback,
                                              bool * resolved, bool * read_failed) -> bool {
    try {
      auto node = cond.browseChild({{0, state_name}, {0, "Id"}});
      if (resolved) {
        *resolved = true;
      }
      try {
        auto val = node.readValue();
        if (val.isType<bool>()) {
          return val.getScalarCopy<bool>();
        }
        return fallback;
      } catch (const opcua::BadStatus &) {
        // Path exists but the read failed transiently: keep the condition.
        if (read_failed) {
          *read_failed = true;
        }
        return fallback;
      }
    } catch (const opcua::BadStatus & e) {
      if (is_path_absent_code(e.code())) {
        // Path genuinely absent: this child is not an alarm condition.
        if (resolved) {
          *resolved = false;
        }
        return fallback;
      }
      // Transient browse failure: conservatively keep the condition and flag it
      // so the caller does not drop it (and let reconcile wrongly clear it).
      if (resolved) {
        *resolved = true;
      }
      if (read_failed) {
        *read_failed = true;
      }
      return fallback;
    }
  };

  // Collect candidate condition nodes from the source. AlarmCondition
  // instances may be linked as hierarchical Object children OR via the
  // non-hierarchical HasCondition reference (i=9006, Part 9 §5.5.3); we follow
  // both and recurse one level through HasCondition so conditions owned by an
  // intermediate Object are still found. De-duplicated by NodeId string.
  constexpr uint32_t kHasConditionRefId = 9006;
  auto collect_candidates = [&](opcua::Node<opcua::Client> & src, std::vector<opcua::Node<opcua::Client>> & out_nodes,
                                std::set<std::string> & seen_ids) {
    auto add = [&](opcua::Node<opcua::Client> n) {
      if (seen_ids.insert(n.id().toString()).second) {
        out_nodes.push_back(std::move(n));
      }
    };
    auto hier = src.browseChildren(opcua::ReferenceTypeId::HierarchicalReferences, opcua::NodeClass::Object);
    for (auto & c : hier) {
      add(c);
    }
    try {
      auto linked = src.browseChildren(opcua::NodeId(0, kHasConditionRefId), opcua::NodeClass::Object);
      for (auto & c : linked) {
        add(c);
      }
    } catch (const opcua::BadStatus &) {
      // HasCondition browse unsupported on this server: hierarchical children
      // already collected, continue.
    }
    // One-level recursion through HasCondition from each hierarchical child.
    for (auto & c : hier) {
      try {
        auto linked = c.browseChildren(opcua::NodeId(0, kHasConditionRefId), opcua::NodeClass::Object);
        for (auto & gc : linked) {
          add(gc);
        }
      } catch (const opcua::BadStatus &) {
      }
    }
  };

  try {
    opcua::Node<opcua::Client> src(impl_->client, source_node);
    std::vector<opcua::Node<opcua::Client>> children;
    std::set<std::string> candidate_ids;
    collect_candidates(src, children, candidate_ids);
    for (auto & child : children) {
      ConditionStateSnapshot snap;
      snap.condition_id = child.id();

      // A child is only treated as an alarm condition when ActiveState/Id
      // resolves; everything else under the source is ignored. A transient
      // read failure on a resolved ActiveState keeps the condition (flagged so
      // the caller does not drop/clear it) instead of silently discarding it.
      bool is_condition = false;
      bool active_read_failed = false;
      snap.active_state = read_state_id(child, "ActiveState", false, &is_condition, &active_read_failed);
      if (!is_condition) {
        continue;
      }
      snap.state_read_failed = active_read_failed;

      snap.enabled_state = read_state_id(child, "EnabledState", true, nullptr, nullptr);
      snap.acked_state = read_state_id(child, "AckedState", true, nullptr, nullptr);
      snap.confirmed_state = read_state_id(child, "ConfirmedState", true, nullptr, nullptr);

      try {
        auto retain_node = child.browseChild({{0, "Retain"}});
        try {
          auto retain = retain_node.readValue();
          if (retain.isType<bool>()) {
            snap.retain = retain.getScalarCopy<bool>();
          }
        } catch (const opcua::BadStatus &) {
          // Retain node resolved but its value could not be read this scan: a
          // transient failure. Do NOT let retain silently default to false
          // (which would Skip an inactive-but-retained condition and let
          // reconcile clear a still-tracked fault). Flag the snapshot unreliable
          // so it is kept (KeepOnly), not dropped (issue #478).
          snap.state_read_failed = true;
        }
      } catch (const opcua::BadStatus & e) {
        // A genuinely absent Retain path (optional on some servers) leaves
        // retain=false. A transient browse failure resolving it is unreliable,
        // so keep-and-flag rather than trust the default.
        if (!is_path_absent_code(e.code())) {
          snap.state_read_failed = true;
        }
      }
      try {
        auto sev = child.browseChild({{0, "Severity"}}).readValue();
        if (sev.isType<uint16_t>()) {
          snap.severity = sev.getScalarCopy<uint16_t>();
        }
      } catch (const opcua::BadStatus &) {
      }
      try {
        auto cname = child.browseChild({{0, "ConditionName"}}).readValue();
        if (cname.isType<opcua::String>()) {
          snap.condition_name = std::string(cname.getScalarCopy<opcua::String>());
        }
      } catch (const opcua::BadStatus &) {
      }
      try {
        auto msg = child.browseChild({{0, "Message"}}).readValue();
        if (msg.isType<opcua::LocalizedText>()) {
          snap.message = std::string(msg.getScalarCopy<opcua::LocalizedText>().getText());
        } else if (msg.isType<opcua::String>()) {
          snap.message = std::string(msg.getScalarCopy<opcua::String>());
        }
      } catch (const opcua::BadStatus &) {
      }

      out.push_back(std::move(snap));
    }
    // The source browse completed without throwing: this is a trustworthy
    // scan (possibly with zero conditions), safe to reconcile against.
    if (scan_ok) {
      *scan_ok = true;
    }
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
    // scan_ok stays false: a browse failure must not be read as "no conditions".
  }

  return out;
}

// ----------------------------------------------------------------------------
// INV2: device-info (nameplate) reads. ServerStatus/BuildInfo is universal;
// the OPC-UA DI companion nameplate (Manufacturer/Model/SerialNumber/...) is
// read best-effort when the server exposes the DI namespace. The raw C API is
// used for the browse + attribute reads so the code is independent of the
// open62541pp high-level browse surface (unavailable in v0.16).
// ----------------------------------------------------------------------------

namespace {

constexpr const char * kDiNamespaceUri = "http://opcfoundation.org/UA/DI/";

// Read a scalar String / LocalizedText node value. Empty on any error or on a
// non-string type. The caller holds ``client_mutex``.
std::string read_string_attribute(UA_Client * client, const UA_NodeId & node_id) {
  UA_Variant value;
  UA_Variant_init(&value);
  std::string out;
  if (UA_Client_readValueAttribute(client, node_id, &value) == UA_STATUSCODE_GOOD) {
    if (UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_STRING])) {
      const auto * s = static_cast<const UA_String *>(value.data);
      out.assign(reinterpret_cast<const char *>(s->data), s->length);
    } else if (UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_LOCALIZEDTEXT])) {
      const auto * lt = static_cast<const UA_LocalizedText *>(value.data);
      out.assign(reinterpret_cast<const char *>(lt->text.data), lt->text.length);
    }
  }
  UA_Variant_clear(&value);
  return out;
}

// Strip leading/trailing ASCII whitespace. Used for vendor identity fields
// exposed in a fixed-width, space-padded form (Siemens pads the MLFB
// OrderNumber to 20 chars) so the stored value is the bare order code.
std::string trim_ascii_ws(std::string s) {
  const char * ws = " \t\r\n";
  const auto begin = s.find_first_not_of(ws);
  if (begin == std::string::npos) {
    return {};
  }
  return s.substr(begin, s.find_last_not_of(ws) - begin + 1);
}

// One forward hierarchical child: its browse name and target NodeId (owned).
struct ChildRef {
  uint16_t namespace_index{0};
  std::string name;
  UA_NodeId node_id{};
};

void clear_child_refs(std::vector<ChildRef> & children) {
  for (auto & child : children) {
    UA_NodeId_clear(&child.node_id);
  }
}

// Append the references of one browse result to ``children`` and hand back its
// continuation point (owned copy; UA_BYTESTRING_NULL when the result is done).
UA_ByteString collect_browse_result(const UA_BrowseResult & result, std::vector<ChildRef> & children) {
  for (size_t i = 0; i < result.referencesSize; ++i) {
    const UA_ReferenceDescription & ref = result.references[i];
    ChildRef child;
    child.namespace_index = ref.browseName.namespaceIndex;
    child.name.assign(reinterpret_cast<const char *>(ref.browseName.name.data), ref.browseName.name.length);
    UA_NodeId_copy(&ref.nodeId.nodeId, &child.node_id);
    children.push_back(std::move(child));
  }
  UA_ByteString continuation = UA_BYTESTRING_NULL;
  if (result.continuationPoint.length > 0) {
    UA_ByteString_copy(&result.continuationPoint, &continuation);
  }
  return continuation;
}

// Browse forward hierarchical children of ``parent``, returning their browse
// names and NodeIds. Empty on failure. The caller must ``clear_child_refs``.
// Follows BrowseNext continuation points so a server that caps references per
// browse (OperationLimits.MaxReferencesPerNode) cannot silently truncate a
// large folder (e.g. a DeviceSet with many devices).
std::vector<ChildRef> browse_forward_children(UA_Client * client, const UA_NodeId & parent) {
  std::vector<ChildRef> children;
  UA_BrowseRequest request;
  UA_BrowseRequest_init(&request);
  request.nodesToBrowse = UA_BrowseDescription_new();
  if (request.nodesToBrowse == nullptr) {
    UA_BrowseRequest_clear(&request);
    return children;
  }
  request.nodesToBrowseSize = 1;
  UA_NodeId_copy(&parent, &request.nodesToBrowse[0].nodeId);
  request.nodesToBrowse[0].browseDirection = UA_BROWSEDIRECTION_FORWARD;
  request.nodesToBrowse[0].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HIERARCHICALREFERENCES);
  request.nodesToBrowse[0].includeSubtypes = true;
  request.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_BROWSENAME;

  UA_ByteString continuation = UA_BYTESTRING_NULL;
  UA_BrowseResponse response = UA_Client_Service_browse(client, request);
  if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD && response.resultsSize == 1) {
    continuation = collect_browse_result(response.results[0], children);
  }
  UA_BrowseResponse_clear(&response);
  UA_BrowseRequest_clear(&request);

  // Iteration cap guards against a misbehaving server that keeps returning a
  // continuation point; the server releases the point itself when it returns
  // the final (empty-continuation) batch.
  for (int iteration = 0; continuation.length > 0 && iteration < 1000; ++iteration) {
    UA_BrowseNextRequest next_request;
    UA_BrowseNextRequest_init(&next_request);
    next_request.releaseContinuationPoints = false;
    next_request.continuationPoints = &continuation;  // borrowed; freed manually below
    next_request.continuationPointsSize = 1;

    UA_BrowseNextResponse next_response = UA_Client_Service_browseNext(client, next_request);
    UA_ByteString next_continuation = UA_BYTESTRING_NULL;
    if (next_response.responseHeader.serviceResult == UA_STATUSCODE_GOOD && next_response.resultsSize == 1) {
      next_continuation = collect_browse_result(next_response.results[0], children);
    }
    UA_BrowseNextResponse_clear(&next_response);
    UA_ByteString_clear(&continuation);
    continuation = next_continuation;
  }
  if (continuation.length > 0) {
    // Early exit (iteration cap): release the server-side continuation point
    // instead of leaking it until session teardown. Best effort.
    UA_BrowseNextRequest release_request;
    UA_BrowseNextRequest_init(&release_request);
    release_request.releaseContinuationPoints = true;
    release_request.continuationPoints = &continuation;
    release_request.continuationPointsSize = 1;
    UA_BrowseNextResponse release_response = UA_Client_Service_browseNext(client, release_request);
    UA_BrowseNextResponse_clear(&release_response);
    UA_ByteString_clear(&continuation);
  }
  return children;
}

// Namespace index of a companion-spec URI in the server's NamespaceArray, or -1.
int find_namespace_index(UA_Client * client, const char * uri) {
  UA_Variant value;
  UA_Variant_init(&value);
  int index = -1;
  if (UA_Client_readValueAttribute(client, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER_NAMESPACEARRAY), &value) ==
          UA_STATUSCODE_GOOD &&
      UA_Variant_hasArrayType(&value, &UA_TYPES[UA_TYPES_STRING])) {
    const auto * entries = static_cast<const UA_String *>(value.data);
    for (size_t i = 0; i < value.arrayLength; ++i) {
      std::string ns(reinterpret_cast<const char *>(entries[i].data), entries[i].length);
      if (ns == uri) {
        index = static_cast<int>(i);
        break;
      }
    }
  }
  UA_Variant_clear(&value);
  return index;
}

// Fill the DI nameplate fields of ``info`` from the first device under
// Objects/DeviceSet that carries any identification property. Best-effort.
void read_di_nameplate(UA_Client * client, uint16_t di_ns, OpcuaClient::DeviceInfo & info) {
  auto objects_children = browse_forward_children(client, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER));
  UA_NodeId device_set = UA_NODEID_NULL;
  for (const auto & child : objects_children) {
    if (child.namespace_index == di_ns && child.name == "DeviceSet") {
      UA_NodeId_copy(&child.node_id, &device_set);
      break;
    }
  }
  clear_child_refs(objects_children);
  if (UA_NodeId_isNull(&device_set)) {
    return;
  }

  auto devices = browse_forward_children(client, device_set);
  UA_NodeId_clear(&device_set);

  for (const auto & device : devices) {
    auto props = browse_forward_children(client, device.node_id);
    auto read_prop = [&](const char * name) -> std::string {
      for (const auto & prop : props) {
        if (prop.namespace_index == di_ns && prop.name == name) {
          return read_string_attribute(client, prop.node_id);
        }
      }
      return {};
    };
    // OrderNumber is a vendor extension, NOT a standard DI property: its
    // BrowseName lives in the vendor namespace (Siemens exposes it as
    // ns=3;s=OrderNumber, BrowseName ns=3:OrderNumber), not the DI namespace,
    // so match by BrowseName only. Empty for vendors that do not expose it.
    auto read_prop_any_ns = [&](const char * name) -> std::string {
      for (const auto & prop : props) {
        if (prop.name == name) {
          return read_string_attribute(client, prop.node_id);
        }
      }
      return {};
    };
    std::string manufacturer = read_prop("Manufacturer");
    std::string model = read_prop("Model");
    std::string serial = read_prop("SerialNumber");
    std::string hardware_revision = read_prop("HardwareRevision");
    std::string software_revision = read_prop("SoftwareRevision");
    std::string order_number = trim_ascii_ws(read_prop_any_ns("OrderNumber"));
    clear_child_refs(props);

    if (!manufacturer.empty() || !model.empty() || !serial.empty() || !hardware_revision.empty() ||
        !software_revision.empty() || !order_number.empty()) {
      info.di_manufacturer = std::move(manufacturer);
      info.di_model = std::move(model);
      info.di_serial_number = std::move(serial);
      info.di_hardware_revision = std::move(hardware_revision);
      info.di_software_revision = std::move(software_revision);
      info.di_order_number = std::move(order_number);
      break;
    }
  }
  clear_child_refs(devices);
}

}  // namespace

OpcuaClient::DeviceInfo OpcuaClient::read_device_info() {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  DeviceInfo info;
  if (!impl_->connected) {
    return info;
  }
  UA_Client * client = impl_->client.handle();

  // Tier 1: ServerStatus/BuildInfo (ns=0 well-known nodes, always present).
  info.manufacturer_name =
      read_string_attribute(client, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER_SERVERSTATUS_BUILDINFO_MANUFACTURERNAME));
  info.product_name =
      read_string_attribute(client, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER_SERVERSTATUS_BUILDINFO_PRODUCTNAME));
  info.software_version =
      read_string_attribute(client, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER_SERVERSTATUS_BUILDINFO_SOFTWAREVERSION));
  info.build_number =
      read_string_attribute(client, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER_SERVERSTATUS_BUILDINFO_BUILDNUMBER));

  // Tier 2: OPC-UA DI nameplate (best-effort, only when the DI namespace exists).
  const int di_ns = find_namespace_index(client, kDiNamespaceUri);
  if (di_ns > 0) {
    read_di_nameplate(client, static_cast<uint16_t>(di_ns), info);
  }
  return info;
}

// ----------------------------------------------------------------------------
// Issue #386: native OPC-UA AlarmCondition event subscription primitives.
// open62541pp v0.16 has no native EventFilter / event subscription API, so the
// raw open62541 C API is used here. The free functions below build the
// EventFilter and trampoline; OpcuaClient owns the per-monitored-item context
// in unique_ptr storage so lifetime is explicit.
// ----------------------------------------------------------------------------

namespace {

/// Build an OPC-UA EventFilter from per-field SimpleAttributeOperand specs.
/// open62541 servers reject SAOs whose BrowsePath does not resolve directly
/// from the supplied ``typeDefinitionId`` (verified against open62541 1.4.6
/// with FULL ns0). Inheritance traversal is NOT performed during
/// validation, so ``AlarmConditionType+EventType`` returns
/// ``BadNodeIdUnknown`` even though EventType is inherited. The caller must
/// pass each field with the type that *directly* defines its first browse
/// segment.
///
/// Auto-prepends 3 fixed clauses so the trampoline can extract them
/// positionally:
///   [0] EventType  - BaseEventType property
///   [1] SourceNode - BaseEventType property
///   [2] ConditionId - ConditionType, empty BrowsePath, AttributeId=NodeId
///                    (Part 9 §5.5.2.13 special case)
UA_EventFilter make_event_filter(const std::vector<OpcuaClient::EventFieldSpec> & user_specs) {
  std::vector<OpcuaClient::EventFieldSpec> all_specs;
  all_specs.reserve(user_specs.size() + 3);
  all_specs.push_back({opcua::NodeId(0, UA_NS0ID_BASEEVENTTYPE), {{0, "EventType"}}, UA_ATTRIBUTEID_VALUE});
  all_specs.push_back({opcua::NodeId(0, UA_NS0ID_BASEEVENTTYPE), {{0, "SourceNode"}}, UA_ATTRIBUTEID_VALUE});
  all_specs.push_back({opcua::NodeId(0, UA_NS0ID_CONDITIONTYPE), {}, UA_ATTRIBUTEID_NODEID});
  for (const auto & s : user_specs) {
    all_specs.push_back(s);
  }

  UA_EventFilter filter;
  UA_EventFilter_init(&filter);
  filter.selectClausesSize = all_specs.size();
  filter.selectClauses = static_cast<UA_SimpleAttributeOperand *>(
      UA_Array_new(filter.selectClausesSize, &UA_TYPES[UA_TYPES_SIMPLEATTRIBUTEOPERAND]));

  for (size_t i = 0; i < all_specs.size(); ++i) {
    UA_SimpleAttributeOperand & sao = filter.selectClauses[i];
    UA_SimpleAttributeOperand_init(&sao);
    UA_NodeId_copy(all_specs[i].type_definition_id.handle(), &sao.typeDefinitionId);
    sao.attributeId = all_specs[i].attribute_id;
    const auto & path = all_specs[i].browse_path;
    sao.browsePathSize = path.size();
    if (sao.browsePathSize > 0) {
      sao.browsePath =
          static_cast<UA_QualifiedName *>(UA_Array_new(sao.browsePathSize, &UA_TYPES[UA_TYPES_QUALIFIEDNAME]));
      for (size_t j = 0; j < path.size(); ++j) {
        sao.browsePath[j] = UA_QUALIFIEDNAME_ALLOC(path[j].namespace_index, path[j].name.c_str());
      }
    }
  }
  return filter;
}

}  // namespace

// C-linkage trampoline matching ``UA_Client_EventNotificationCallback``.
// Defined at namespace scope so its address is a stable function pointer.
static void on_event_trampoline_c(UA_Client * /*client*/, UA_UInt32 sub_id, void * /*sub_ctx*/, UA_UInt32 mon_id,
                                  void * mon_ctx, size_t n_fields, UA_Variant * fields) {
  RCLCPP_DEBUG_STREAM(opcua_client_logger(),
                      "TRAMPOLINE FIRED sub=" << sub_id << " mon=" << mon_id << " n_fields=" << n_fields);
  auto * ctx = static_cast<EventCallbackContext *>(mon_ctx);
  if (ctx == nullptr || ctx->owner == nullptr) {
    RCLCPP_DEBUG(opcua_client_logger(), "TRAMPOLINE: ctx null");
    return;
  }
  // Stale callback from a defunct subscription - ctx is still valid (we only
  // free contexts after the generation has already moved past), but the
  // payload no longer reflects live state. Drop silently.
  if (ctx->generation_snapshot != ctx->owner->current_generation()) {
    return;
  }
  // Single MI removed via remove_event_monitored_item while peers remain
  // live - the global generation has not changed, so the peer trampolines
  // (in the same subscription) must keep firing. Drop only this MI's late
  // notifications via the per-context flag.
  if (!ctx->active.load(std::memory_order_acquire)) {
    return;
  }

  // Copy the UA_Variant fields into open62541pp wrappers. The Variant
  // lvalue constructor deep-copies the underlying buffer, which the
  // opcua::Variant destructor will free.
  std::vector<opcua::Variant> values;
  values.reserve(n_fields);
  for (size_t i = 0; i < n_fields; ++i) {
    values.emplace_back(opcua::Variant{fields[i]});
  }

  // Auto-prepended positions (matching make_event_filter):
  //   [0] EventType, [1] SourceNode, [2] ConditionId
  opcua::NodeId event_type;
  opcua::NodeId source_node;
  opcua::NodeId condition_id;
  if (n_fields >= 1 && values[0].isType<opcua::NodeId>()) {
    event_type = values[0].getScalarCopy<opcua::NodeId>();
  }
  if (n_fields >= 2 && values[1].isType<opcua::NodeId>()) {
    source_node = values[1].getScalarCopy<opcua::NodeId>();
  }
  if (n_fields >= 3 && values[2].isType<opcua::NodeId>()) {
    condition_id = values[2].getScalarCopy<opcua::NodeId>();
  }

  std::vector<opcua::Variant> user_values;
  if (n_fields > 3) {
    user_values.reserve(n_fields - 3);
    for (size_t i = 3; i < n_fields; ++i) {
      user_values.push_back(std::move(values[i]));
    }
  }

  if (ctx->callback) {
    // The callback chain (on_event -> apply_condition_state -> fault report /
    // clear) must never let an exception unwind through open62541's C frames:
    // they are not exception-safe (leaks / inconsistent internal state; formally
    // UB) and an escape past run_iterate would call std::terminate on the poll
    // thread. Contain everything at the C boundary.
    try {
      ctx->callback(user_values, source_node, event_type, condition_id);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(opcua_client_logger(), "OPC-UA event callback threw, dropping notification: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(opcua_client_logger(), "OPC-UA event callback threw a non-std exception, dropping notification");
    }
  }
}

uint64_t OpcuaClient::current_generation() const {
  return impl_->generation.load(std::memory_order_acquire);
}

void OpcuaClient::run_iterate(uint16_t timeout_ms) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  if (!impl_->connected) {
    return;
  }
  try {
    impl_->client.runIterate(timeout_ms);
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
  }
}

uint32_t OpcuaClient::add_event_monitored_item(uint32_t subscription_id, const opcua::NodeId & source_node,
                                               const std::vector<EventFieldSpec> & select_specs,
                                               EventCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return 0;
  }

  // Heap-allocate context. Ownership stays in event_callbacks; the C API gets
  // a non-owning raw pointer until remove_event_monitored_item or cleanup
  // erases the entry.
  auto ctx = std::make_unique<EventCallbackContext>();
  ctx->owner = this;
  ctx->generation_snapshot = impl_->generation.load(std::memory_order_acquire);
  ctx->subscription_id = subscription_id;
  ctx->callback = std::move(callback);
  EventCallbackContext * raw_ctx = ctx.get();

  UA_EventFilter filter = make_event_filter(select_specs);

  UA_MonitoredItemCreateRequest item;
  UA_MonitoredItemCreateRequest_init(&item);
  // Deep-copy the source NodeId so the request struct owns its string
  // buffer (if any). Cleared by UA_MonitoredItemCreateRequest_clear after
  // the call.
  UA_NodeId_copy(source_node.handle(), &item.itemToMonitor.nodeId);
  item.itemToMonitor.attributeId = UA_ATTRIBUTEID_EVENTNOTIFIER;
  item.monitoringMode = UA_MONITORINGMODE_REPORTING;
  item.requestedParameters.samplingInterval = 0.0;
  item.requestedParameters.discardOldest = true;
  item.requestedParameters.queueSize = 100;
  UA_ExtensionObject_setValueNoDelete(&item.requestedParameters.filter, &filter, &UA_TYPES[UA_TYPES_EVENTFILTER]);

  // Debug-level diagnostic so integration-test failures can surface the
  // exact NodeId / select-clause count we hand to the server. Quiet at
  // INFO; ``ros2 launch --log-level opcua.client:=debug`` (or env var
  // RCUTILS_CONSOLE_OUTPUT_FORMAT) re-enables it.
  RCLCPP_DEBUG_STREAM(opcua_client_logger(),
                      "add_event_monitored_item: subId=" << subscription_id << " nodeId=" << source_node.toString()
                                                         << " selectClauses=" << (select_specs.size() + 3));

  UA_MonitoredItemCreateResult result =
      UA_Client_MonitoredItems_createEvent(impl_->client.handle(), subscription_id, UA_TIMESTAMPSTORETURN_BOTH, item,
                                           raw_ctx, on_event_trampoline_c, /*deleteCallback=*/nullptr);

  // Free the locally-allocated request members (the ExtensionObject does
  // NOT own the filter because we used setValueNoDelete; clear it
  // separately below). UA_MonitoredItemCreateRequest_clear walks the
  // struct including itemToMonitor.nodeId.
  // Detach the filter from item.requestedParameters before clearing the
  // request, otherwise UA_*_clear would try to free our stack filter.
  // Re-init the ExtensionObject to a valid empty state.
  UA_ExtensionObject_init(&item.requestedParameters.filter);
  UA_MonitoredItemCreateRequest_clear(&item);
  UA_EventFilter_clear(&filter);

  RCLCPP_DEBUG_STREAM(opcua_client_logger(), "createEvent result: status=" << UA_StatusCode_name(result.statusCode)
                                                                           << " miId=" << result.monitoredItemId);

  if (result.statusCode != UA_STATUSCODE_GOOD) {
    UA_MonitoredItemCreateResult_clear(&result);
    return 0;
  }

  uint32_t mi_id = result.monitoredItemId;
  ctx->monitored_item_id = mi_id;
  UA_MonitoredItemCreateResult_clear(&result);

  {
    std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
    impl_->event_callbacks.emplace(mi_id, std::move(ctx));
  }
  return mi_id;
}

bool OpcuaClient::remove_event_monitored_item(uint32_t subscription_id, uint32_t mi_id) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  std::unique_ptr<EventCallbackContext> retired;
  {
    std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
    auto it = impl_->event_callbacks.find(mi_id);
    if (it == impl_->event_callbacks.end() || it->second->subscription_id != subscription_id) {
      return false;
    }
    // Flip the per-MI active flag BEFORE moving the unique_ptr so any
    // in-flight trampoline call observes the cleared flag and bails out
    // before touching the about-to-be-freed callback. Order:
    //   1. set active=false (release): trampoline reads it (acquire) and
    //      returns without invoking ``callback``.
    //   2. move unique_ptr out of map and erase: ctx still alive in
    //      ``retired`` until end of function.
    //   3. delete server-side MI synchronously (mutex serializes against
    //      the client's notification dispatch).
    //   4. ``retired`` falls out of scope -> ctx freed.
    // We deliberately do NOT bump the global ``generation`` counter here:
    // it would invalidate every peer monitored item registered against the
    // same subscription, silently dropping their callbacks even though
    // their MIs are still live on the server. Generation is reserved for
    // full disconnect / remove_subscriptions.
    it->second->active.store(false, std::memory_order_release);
    retired = std::move(it->second);
    impl_->event_callbacks.erase(it);
  }

  if (impl_->connected) {
    UA_StatusCode status = UA_Client_MonitoredItems_deleteSingle(impl_->client.handle(), subscription_id, mi_id);
    (void)status;  // best effort; on failure the server may already have dropped it
  }
  // ``retired`` falls out of scope here, freeing the EventCallbackContext.
  return true;
}

OpcuaClient::MethodErrorInfo OpcuaClient::status_to_method_error(uint32_t code, const std::string & message) {
  if (code == UA_STATUSCODE_BADMETHODINVALID || code == UA_STATUSCODE_BADNODEIDUNKNOWN ||
      code == UA_STATUSCODE_BADNOTSUPPORTED) {
    return {MethodError::MethodNotFound, message};
  }
  if (code == UA_STATUSCODE_BADARGUMENTSMISSING || code == UA_STATUSCODE_BADINVALIDARGUMENT ||
      code == UA_STATUSCODE_BADTYPEMISMATCH || code == UA_STATUSCODE_BADTOOMANYARGUMENTS) {
    return {MethodError::InvalidArgument, message};
  }
  if (code == UA_STATUSCODE_BADTIMEOUT) {
    return {MethodError::MethodTimeout, message};
  }
  return {MethodError::TransportError, message};
}

tl::expected<void, OpcuaClient::MethodErrorInfo>
OpcuaClient::classify_call_result(uint32_t overall_status_code, const std::vector<uint32_t> & arg_results) {
  if (overall_status_code != UA_STATUSCODE_GOOD) {
    return tl::make_unexpected(status_to_method_error(overall_status_code, UA_StatusCode_name(overall_status_code)));
  }
  // Per OPC-UA Part 4 §5.11.2, even when overall statusCode is Good the
  // server may flag per-argument validation failures via inputArgumentResults.
  // AlarmConditionType.Acknowledge surfaces BadEventIdUnknown here when our
  // cached EventId has been superseded - the call did NOT take effect.
  // Returning an error keeps the SOVD layer from falsely reporting success.
  for (size_t i = 0; i < arg_results.size(); ++i) {
    if (arg_results[i] != UA_STATUSCODE_GOOD) {
      std::string msg = std::string(UA_StatusCode_name(arg_results[i])) + " on input arg " + std::to_string(i);
      return tl::make_unexpected(status_to_method_error(arg_results[i], msg));
    }
  }
  return {};
}

tl::expected<std::vector<opcua::Variant>, OpcuaClient::MethodErrorInfo>
OpcuaClient::call_method(const opcua::NodeId & object_id, const opcua::NodeId & method_id,
                         const std::vector<opcua::Variant> & input_args) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return tl::make_unexpected(MethodErrorInfo{MethodError::NotConnected, "Not connected to OPC-UA server"});
  }

  try {
    opcua::CallMethodResult result =
        opcua::services::call(impl_->client, object_id, method_id, opcua::Span<const opcua::Variant>(input_args));
    UA_StatusCode code = result.getStatusCode().get();
    auto arg_results = result.getInputArgumentResults();
    std::vector<uint32_t> arg_codes;
    arg_codes.reserve(arg_results.size());
    for (const auto & arg_result : arg_results) {
      arg_codes.push_back(arg_result.get());
    }
    if (client_debug_enabled()) {
      // RCLCPP_DEBUG_STREAM constructs its std::stringstream unconditionally,
      // so build the per-arg suffix only when DEBUG is actually active.
      std::ostringstream args_oss;
      for (size_t i = 0; i < arg_codes.size(); ++i) {
        args_oss << " arg" << i << "=" << UA_StatusCode_name(arg_codes[i]);
      }
      RCLCPP_DEBUG_STREAM(opcua_client_logger(),
                          "call_method object=" << object_id.toString() << " method=" << method_id.toString()
                                                << " statusCode=" << UA_StatusCode_name(code) << args_oss.str());
    }

    auto classified = classify_call_result(code, arg_codes);
    if (!classified.has_value()) {
      return tl::make_unexpected(classified.error());
    }
    auto outputs_span = result.getOutputArguments();
    std::vector<opcua::Variant> outputs;
    outputs.reserve(outputs_span.size());
    for (const auto & v : outputs_span) {
      outputs.push_back(v);  // Variant is copyable
    }
    return outputs;
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
    return tl::make_unexpected(status_to_method_error(e.code(), e.what()));
  }
}

}  // namespace ros2_medkit_gateway
