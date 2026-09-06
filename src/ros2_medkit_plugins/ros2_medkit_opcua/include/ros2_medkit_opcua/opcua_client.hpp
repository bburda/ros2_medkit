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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <open62541pp/open62541pp.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Variant type for OPC-UA values exposed to the plugin
using OpcuaValue = std::variant<bool, int32_t, int64_t, float, double, std::string>;

/// Result of reading a single OPC-UA node
struct ReadResult {
  std::string node_id;
  OpcuaValue value;
  std::chrono::system_clock::time_point timestamp;
  bool good{true};
};

/// Callback for subscription data changes
using DataChangeCallback = std::function<void(const std::string & node_id, const OpcuaValue & value)>;

/// OPC-UA SecurityPolicy selector. Maps to the policy URI sent during the
/// SecureChannel handshake. ``None`` is the only value that works without a
/// client application-instance certificate; everything else requires
/// ``client_cert_path`` + ``client_key_path`` and an encryption-enabled build.
enum class SecurityPolicy { None, Basic256Sha256, Aes128Sha256RsaOaep, Aes256Sha256RsaPss };

/// OPC-UA MessageSecurityMode. ``None`` = cleartext; ``Sign`` = signed but not
/// encrypted; ``SignAndEncrypt`` = signed + encrypted. Independent of the
/// SecurityPolicy selector above (the server endpoint must offer the pair).
enum class SecurityMode { None, Sign, SignAndEncrypt };

/// Session user identity token type. ``Anonymous`` needs no credentials;
/// ``UsernamePassword`` uses ``username``/``password``; ``X509`` presents a
/// user certificate (``user_cert_path``).
enum class UserAuthMode { Anonymous, UsernamePassword, X509 };

/// Configuration for OPC-UA connection
struct OpcuaClientConfig {
  std::string endpoint_url = "opc.tcp://localhost:4840";
  std::chrono::milliseconds connect_timeout{5000};
  std::chrono::milliseconds reconnect_interval{3000};

  // --- SecureChannel security (opt-in; defaults reproduce the legacy
  // anonymous + SecurityPolicy=None behaviour) ---
  SecurityPolicy security_policy{SecurityPolicy::None};
  SecurityMode security_mode{SecurityMode::None};

  /// Client application-instance certificate (X.509 v3, DER-encoded) and its
  /// private key (PEM-encoded). Required for any SecurityPolicy other than
  /// None. Empty when running unsecured.
  std::string client_cert_path;
  std::string client_key_path;

  /// Application URI advertised by the client. MUST match the URI entry in
  /// the certificate's SubjectAltName, otherwise the server rejects the
  /// SecureChannel with BadCertificateUriInvalid. Empty leaves the
  /// open62541 default ("urn:open62541.client.application").
  std::string application_uri;

  /// Trusted server / CA certificates (DER-encoded) forming the trust store.
  /// Used to validate the server certificate when ``reject_untrusted`` is
  /// true.
  std::vector<std::string> trust_list_paths;

  /// When true (default) the server certificate must chain to an entry in
  /// ``trust_list_paths``; an untrusted server is rejected. When false the
  /// client accepts any server certificate (accept-any: INSECURE, lab only).
  /// This is NOT trust-on-first-use: nothing is pinned, so a later cert change
  /// is not detected.
  bool reject_untrusted{true};

  // --- Session user identity ---
  UserAuthMode user_auth_mode{UserAuthMode::Anonymous};
  std::string username;
  std::string password;
  /// User X.509 token certificate (DER-encoded), used when
  /// ``user_auth_mode == X509``.
  std::string user_cert_path;
};

/// RAII wrapper around open62541pp::Client with auto-reconnect
class OpcuaClient {
 public:
  OpcuaClient();
  ~OpcuaClient();

  OpcuaClient(const OpcuaClient &) = delete;
  OpcuaClient & operator=(const OpcuaClient &) = delete;
  OpcuaClient(OpcuaClient &&) = delete;
  OpcuaClient & operator=(OpcuaClient &&) = delete;

  /// Connect to OPC-UA server
  /// @return true if connected successfully
  bool connect(const OpcuaClientConfig & config);

  /// Disconnect and stop reconnect attempts
  void disconnect();

  /// Check if currently connected
  bool is_connected() const;

  /// Number of successfully established sessions so far (0 before the first
  /// connect). Increments once per fresh connect, NOT on a connect() call that
  /// finds the session already open. Lets callers detect a reconnect and
  /// refresh per-session state (e.g. re-read the device-info nameplate).
  uint64_t connection_generation() const;

  /// Get the endpoint URL (for status reporting)
  std::string endpoint_url() const;

  /// Get the current config (for reconnection)
  OpcuaClientConfig current_config() const;

  /// Browse child nodes of a given node
  /// @return Vector of child node ID strings (e.g., "ns=1;s=TankLevel")
  std::vector<std::string> browse(const opcua::NodeId & parent_node);

  /// One forward-hierarchical child returned by ``browse_detailed`` - the
  /// address-space walker (auto_browse, address_space_browser.hpp) needs the
  /// BrowseName, DisplayName and NodeClass to build the SOVD entity tree, not
  /// just the NodeId that plain ``browse()`` returns.
  struct BrowseChild {
    opcua::NodeId node_id;
    uint16_t browse_name_ns{0};
    std::string browse_name;
    std::string display_name;
    opcua::NodeClass node_class{opcua::NodeClass::Unspecified};
  };

  /// Browse forward hierarchical (Object/Variable only) children of
  /// ``parent_node``, server-side filtered to those two node classes and
  /// following BrowseNext continuation points automatically. Only local
  /// nodes are returned (cross-server references are dropped - there is
  /// nothing on this session to read their attributes from). Empty vector on
  /// failure or when not connected.
  std::vector<BrowseChild> browse_detailed(const opcua::NodeId & parent_node);

  /// Read the DataType attribute of a Variable node and resolve it to the
  /// OPC-UA builtin scalar type name ("Boolean", "SByte", "Byte", "Int16",
  /// "UInt16", "Int32", "UInt32", "Int64", "UInt64", "Float", "Double",
  /// "String", "DateTime") when the DataType is a ns=0 builtin identified
  /// numerically. Returns an empty string for structured / enumerated /
  /// vendor-defined types (auto_browse treats those as unsupported and skips
  /// the variable) and when not connected or the read fails.
  std::string read_variable_type_name(const opcua::NodeId & variable_node);

  /// Read a single value
  ReadResult read_value(const opcua::NodeId & node_id);

  /// Read multiple values
  std::vector<ReadResult> read_values(const std::vector<opcua::NodeId> & node_ids);

#if !MEDKIT_OPCUA_READ_ONLY
  /// The value-write surface, present only in a write-capable build
  /// (``-DMEDKIT_OPCUA_READ_ONLY=OFF``). A read-only build has no declaration
  /// and no definition, so nothing in the plugin can reach an OPC-UA Write
  /// service call - see the package README on why that is a build property and
  /// not a setting.

  /// OPC-UA write error classification
  enum class WriteError { NotConnected, TypeMismatch, AccessDenied, NodeNotFound, TransportError };

  /// Detailed write error info
  struct WriteErrorInfo {
    WriteError code;
    std::string message;
  };

  /// Write a value to a node
  /// @param data_type_hint If non-empty, skip readValue type-probe and use this hint
  ///        ("bool", "int", "float", "string") to select the write coercion directly.
  ///        Halves mutex hold time by eliminating a round-trip to the server.
  /// @return void on success, WriteErrorInfo on failure with specific error code
  tl::expected<void, WriteErrorInfo> write_value(const opcua::NodeId & node_id, const OpcuaValue & value,
                                                 const std::string & data_type_hint = "");
#endif

  /// The AccessLevel / UserAccessLevel bits of a Variable node, read straight
  /// from the server. ``ok`` is false when not connected or the attribute read
  /// failed (in which case the bit flags are all cleared and the caller must
  /// NOT assume a permission). ``writable`` is the effective, session-scoped
  /// CurrentWrite bit: UserAccessLevel when it read back, otherwise the node's
  /// inherent AccessLevel. The raw bitmasks are kept for diagnostics.
  struct AccessLevelInfo {
    bool ok{false};
    bool readable{false};
    bool writable{false};
    uint8_t access_level{0};       ///< raw AccessLevel bitmask
    uint8_t user_access_level{0};  ///< raw UserAccessLevel bitmask
  };

  /// Read a Variable node's AccessLevel and UserAccessLevel attributes and
  /// decode the CurrentRead / CurrentWrite bits (OPC-UA Part 3 §5.6.2). Used
  /// by auto_browse to mark a discovered data point writable exactly when the
  /// server says this session may write it - no hand-written node map needed.
  AccessLevelInfo read_access_level(const opcua::NodeId & variable_node);

  /// Create a subscription with data change notifications
  /// @return Subscription ID, or 0 on failure
  uint32_t create_subscription(double publish_interval_ms, DataChangeCallback callback);

  /// Add a monitored item to a subscription
  /// @return true if item was added
  bool add_monitored_item(uint32_t subscription_id, const opcua::NodeId & node_id);

  /// Remove all subscriptions
  void remove_subscriptions();

  /// One element of an OPC-UA SimpleAttributeOperand browse path.
  struct EventField {
    uint16_t namespace_index{0};
    std::string name;
  };

  /// Browse path for an event field, e.g. ``{{0, "EnabledState"}, {0, "Id"}}``.
  using EventBrowsePath = std::vector<EventField>;

  /// Full SimpleAttributeOperand spec - every clause in an EventFilter must
  /// have ``typeDefinitionId`` set to the type that *directly* defines the
  /// browse path's first segment (open62541 servers reject inherited
  /// lookups with BadNodeIdUnknown). ConditionId is the documented edge
  /// case (Part 9 §5.5.2.13): empty browse path + AttributeId=NodeId.
  struct EventFieldSpec {
    opcua::NodeId type_definition_id;
    EventBrowsePath browse_path;
    uint32_t attribute_id{13};  // UA_ATTRIBUTEID_VALUE
  };

  /// Callback invoked when an OPC-UA event arrives on a monitored item.
  /// @param select_values Values for caller-requested fields, in the order of
  ///        ``select_specs`` passed to ``add_event_monitored_item``.
  /// @param source_node Always-included SourceNode (extracted from the event
  ///        payload; null NodeId if the server omitted it).
  /// @param event_type Always-included EventType (null NodeId if absent).
  /// @param condition_id NodeId of the condition instance that emitted the
  ///        event (Part 9 §5.5.2.13). Null NodeId for non-condition events.
  using EventCallback =
      std::function<void(const std::vector<opcua::Variant> & select_values, const opcua::NodeId & source_node,
                         const opcua::NodeId & event_type, const opcua::NodeId & condition_id)>;

  /// Get the current subscription generation. Increments on every detected
  /// disconnect (clean ``disconnect()`` or transport-level drop). Used by the
  /// internal event trampoline to drop callbacks fired from defunct
  /// subscriptions.
  uint64_t current_generation() const;

  /// Run a single iteration of the open62541 client main loop. Required to
  /// dispatch incoming subscription notifications (events, data changes)
  /// to their callbacks. The poller calls this every iteration to keep
  /// AlarmCondition events flowing.
  void run_iterate(uint16_t timeout_ms = 100);

  /// Add an event-based monitored item to an existing subscription.
  ///
  /// Wraps ``UA_Client_MonitoredItems_createEvent`` from the open62541 C API
  /// because ``open62541pp`` v0.16 has no native EventFilter / event
  /// subscription support. ``EventType``, ``SourceNode`` and a ConditionId
  /// SAO (empty BrowsePath, AttributeId=NodeId) are always prepended; they
  /// are extracted from the event payload and delivered as separate callback
  /// parameters, not in ``select_values``.
  ///
  /// @return Server-assigned monitored item ID, or 0 on failure.
  uint32_t add_event_monitored_item(uint32_t subscription_id, const opcua::NodeId & source_node,
                                    const std::vector<EventFieldSpec> & select_specs, EventCallback callback);

  /// Remove a previously-added event monitored item. The server is asked to
  /// delete the item synchronously; the callback context is freed only after
  /// the server ACK so in-flight C callbacks cannot dangle.
  /// @return true if the item was found and removed cleanly.
  bool remove_event_monitored_item(uint32_t subscription_id, uint32_t mi_id);

  /// OPC-UA Method call error classification.
  enum class MethodError { NotConnected, MethodNotFound, InvalidArgument, MethodTimeout, TransportError };

  /// Detailed Method call error info.
  struct MethodErrorInfo {
    MethodError code;
    std::string message;
  };

  /// Synchronously call an OPC-UA Method on a target object.
  /// The read path uses it for ConditionRefresh, which asks the server to
  /// replay active conditions to this subscription and changes nothing on the
  /// controller (Part 9 5.5.7).
  /// @return Output arguments on success, MethodErrorInfo on failure.
  tl::expected<std::vector<opcua::Variant>, MethodErrorInfo>
  call_method(const opcua::NodeId & object_id, const opcua::NodeId & method_id,
              const std::vector<opcua::Variant> & input_args);

#if !MEDKIT_OPCUA_READ_ONLY
  /// Acknowledge or Confirm a live condition (Part 9 Acknowledge i=9111,
  /// Confirm i=9113), present only in a write-capable build.
  ///
  /// Separate from ``call_method`` because it is a different kind of call: it
  /// changes alarm state on the server, and a read-only build forbids every
  /// write path, not only value writes. ``call_method`` itself stays, because
  /// ConditionRefresh rides on it and asks the server for a replay rather than
  /// changing anything.
  ///
  /// @param condition_id ConditionId of the live instance to act on.
  /// @param method_id Acknowledge or Confirm on AcknowledgeableConditionType.
  /// @param event_id EventId of the event being acknowledged.
  /// @param comment Operator comment, forwarded as a LocalizedText.
  tl::expected<std::vector<opcua::Variant>, MethodErrorInfo> call_condition_method(const opcua::NodeId & condition_id,
                                                                                   const opcua::NodeId & method_id,
                                                                                   const opcua::ByteString & event_id,
                                                                                   const std::string & comment);
#endif

  /// Map an OPC-UA StatusCode (from an attempted method call or a
  /// per-argument validation result) to a ``MethodError`` category.
  /// Exposed as a public static helper so the classification table is
  /// covered by unit tests without needing a live OPC-UA connection.
  static MethodErrorInfo status_to_method_error(uint32_t code, const std::string & message);

  /// Classify the full result of a Call service exchange per OPC-UA
  /// Part 4 §5.11.2: overall ``statusCode`` covers transport / method
  /// resolution; ``inputArgumentResults`` covers per-argument validation.
  /// Returns success when both are Good. The first non-Good code wins:
  /// overall statusCode takes precedence, then arg_results in order.
  /// AlarmConditionType.Acknowledge surfaces ``BadEventIdUnknown`` in
  /// ``arg_results[0]`` when the EventId we cached has been superseded.
  /// Exposed as a static for unit-test coverage of the per-arg branch.
  static tl::expected<void, MethodErrorInfo> classify_call_result(uint32_t overall_status_code,
                                                                  const std::vector<uint32_t> & arg_results);

  /// Current state of one OPC-UA Condition instance, read directly from the
  /// address space (issue #389 reconnect replay). Used as the read-based
  /// fallback when ConditionRefresh is unavailable: instead of waiting for
  /// the server to replay buffered events, the client browses the alarm
  /// source for its condition instances and reads their state variables.
  struct ConditionStateSnapshot {
    opcua::NodeId condition_id;
    std::string condition_name;  // ConditionType.ConditionName (issue #389 mapping)
    bool enabled_state{true};
    bool active_state{false};
    bool acked_state{true};
    bool confirmed_state{true};
    bool retain{false};
    uint16_t severity{0};
    std::string message;
    /// True when the condition's ActiveState/Id node resolved but its value
    /// could not be read this scan (transient read failure). The caller must
    /// treat the condition as still present (do not clear it) even though its
    /// state fields fell back to their defaults.
    bool state_read_failed{false};
  };

  /// Browse ``source_node`` for AlarmCondition instances and read their
  /// current state (ActiveState/Id, AckedState/Id, ConfirmedState/Id,
  /// EnabledState/Id, Retain, Severity, Message). Only immediate children
  /// that expose an ``ActiveState`` node are treated as conditions; other
  /// children are skipped.
  ///
  /// ``scan_ok`` (when provided) distinguishes a successful empty scan from a
  /// failed one: it is set true only when the source browse completed, and
  /// false when disconnected or when the browse raised a Bad status. Callers
  /// must NOT reconcile (clear) active faults for a source whose scan failed,
  /// otherwise a transient disconnect would falsely clear live alarms.
  ///
  /// NOTE: this read-based fallback assumes each AlarmCondition instance is
  /// browseable as an immediate child of its alarm source via
  /// HierarchicalReferences. That holds for the open62541 reference server,
  /// but has NOT yet been validated against a real Siemens S7-1500 (whose
  /// address-space layout for condition instances must be confirmed before
  /// this path can be claimed to work there).
  std::vector<ConditionStateSnapshot> read_source_conditions(const opcua::NodeId & source_node,
                                                             bool * scan_ok = nullptr);

  /// Get server description string (for status endpoint)
  std::string server_description() const;

  // --- Security config parsing helpers (pure, unit-testable without a
  // server). Case-insensitive; unknown input falls back to the safe default
  // and sets ``*ok = false`` when provided. ---

  /// Parse a SecurityPolicy name ("None", "Basic256Sha256",
  /// "Aes128Sha256RsaOaep", "Aes256Sha256RsaPss"). Falls back to None.
  static SecurityPolicy parse_security_policy(const std::string & name, bool * ok = nullptr);

  /// Map a SecurityPolicy to its OPC-UA policy URI
  /// (e.g. "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256").
  static std::string security_policy_uri(SecurityPolicy policy);

  /// Parse a MessageSecurityMode name ("None", "Sign", "SignAndEncrypt").
  /// Falls back to None.
  static SecurityMode parse_security_mode(const std::string & name, bool * ok = nullptr);

  /// Parse a user-identity mode ("Anonymous", "Username"/"UsernamePassword",
  /// "X509"/"Certificate"). Falls back to Anonymous.
  static UserAuthMode parse_user_auth_mode(const std::string & name, bool * ok = nullptr);

  /// True when the config requests a secured SecureChannel (any
  /// SecurityPolicy other than None, or any MessageSecurityMode other than
  /// None). Username/password or X.509 identity alone does NOT imply an
  /// encrypted channel.
  static bool requires_secure_channel(const OpcuaClientConfig & config);

  /// True when user credentials (username/password or X.509) would be sent
  /// over an unencrypted SecureChannel: a non-anonymous identity combined with
  /// no secured channel. Such credentials can be intercepted on the wire and
  /// the caller is expected to warn loudly.
  static bool credentials_sent_in_clear(const OpcuaClientConfig & config);

  /// True when SecurityPolicy and MessageSecurityMode are set inconsistently:
  /// exactly one of them is None. OPC-UA (Part 4 §7.37) requires them to move
  /// together - a mode of Sign/SignAndEncrypt with SecurityPolicy=None would
  /// leave the policy URI unpinned (silently allowing a downgraded channel),
  /// and a policy without a mode is equally malformed. A conflicting config is
  /// rejected before contacting the server.
  static bool security_config_conflict(const OpcuaClientConfig & config);

  /// Device identity read from an OPC-UA server's information model.
  ///
  /// Two tiers, both best-effort (empty string when the server does not expose
  /// a field):
  ///   - ServerStatus/BuildInfo: present on every compliant server. Describes
  ///     the OPC-UA *server* (which for an embedded PLC IS the device).
  ///   - OPC-UA DI nameplate (companion spec ``http://opcfoundation.org/UA/DI/``):
  ///     the standard per-device identification properties, present only when the
  ///     server implements the DI model. More specific than BuildInfo.
  struct DeviceInfo {
    // ServerStatus/BuildInfo (ns=0 well-known nodes)
    std::string manufacturer_name;  ///< BuildInfo.ManufacturerName
    std::string product_name;       ///< BuildInfo.ProductName
    std::string software_version;   ///< BuildInfo.SoftwareVersion
    std::string build_number;       ///< BuildInfo.BuildNumber

    // OPC-UA DI DeviceType identification (only when the DI namespace is present)
    std::string di_manufacturer;       ///< DeviceType.Manufacturer
    std::string di_model;              ///< DeviceType.Model
    std::string di_serial_number;      ///< DeviceType.SerialNumber
    std::string di_hardware_revision;  ///< DeviceType.HardwareRevision
    std::string di_software_revision;  ///< DeviceType.SoftwareRevision
    /// Manufacturer order code (AAS ManufacturerOrderCode). Vendor extension,
    /// not a standard DI property: its BrowseName lives in the vendor namespace
    /// (Siemens exposes it as ns=3;s=OrderNumber), so it is matched by
    /// BrowseName only. Empty when the server does not expose it.
    std::string di_order_number;
  };

  /// Read device identity (nameplate) from the connected server.
  ///
  /// Reads ServerStatus/BuildInfo unconditionally and, when the server exposes
  /// the OPC-UA DI companion namespace, the DeviceSet nameplate. Best-effort:
  /// returns whatever was readable and never throws. Returns an all-empty
  /// struct when not connected.
  DeviceInfo read_device_info();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_medkit_gateway
