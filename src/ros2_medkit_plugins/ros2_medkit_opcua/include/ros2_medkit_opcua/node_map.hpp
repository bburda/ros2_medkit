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
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <open62541pp/open62541pp.hpp>

#include "ros2_medkit_fault_detection/fault_detection.hpp"

namespace ros2_medkit_gateway {

/// Threshold-mode alarm configuration for a node that maps to a SOVD fault.
/// Retained for backward compatibility with the original ``alarm:`` node-map
/// block; at load time it is also lowered to a shared
/// ``fault_detection::ThresholdRule`` (see ``NodeMapEntry::detection``) so the
/// runtime evaluator is the same one used by status-bit and enum modes.
struct AlarmConfig {
  std::string fault_code;
  std::string severity;  // "ERROR", "WARNING", "INFO"
  std::string message;
  double threshold{0.0};
  bool above_threshold{true};  // true = alarm when value > threshold
};

/// Reference to one extra OPC-UA event field to select and surface as an
/// associated value (issue #389). Used for vendor associated values such as
/// Siemens Program_Alarm ``SD_1``..``SD_n``. ``label`` is what appears in the
/// fault description; ``name``/``namespace_index`` address the event field via
/// a SimpleAttributeOperand on BaseEventType.
struct AssociatedValueRef {
  uint16_t namespace_index{0};
  std::string name;
  std::string label;  // defaults to ``name`` when unset
};

/// One condition-identity mapping inside an ``event_alarms`` source (issue
/// #389). A single OPC-UA source (e.g. the Server object as a catch-all, or a
/// real owning Object) can emit many distinct conditions; each mapping routes
/// a subset to its own SOVD fault. Match fields are AND-combined; an empty
/// match field is a wildcard. ``match_condition_name`` / ``match_source_node``
/// / ``match_event_type`` are equality matches; ``match_message`` is a
/// case-sensitive substring match on the event Message. The first mapping whose
/// non-empty match fields all match the observed event wins (declaration order
/// = precedence).
struct AlarmMapping {
  std::string match_condition_name;  // ConditionType.ConditionName (empty = any)
  std::string match_source_node;     // event SourceNode id string (empty = any)
  std::string match_event_type;      // event EventType id string (empty = any)
  std::string match_message;         // Message substring (empty = any); case-sensitive contains

  std::string fault_code;
  std::string severity_override;
  std::string message_override;
};

/// Configuration for a native OPC-UA AlarmConditionType event subscription
/// (issue #386). The plugin subscribes to events emitted from
/// ``alarm_source`` and bridges them through ``AlarmStateMachine`` into
/// SOVD faults. Mutually exclusive with the threshold-based ``AlarmConfig``
/// on a single ``NodeMapEntry``.
struct AlarmEventConfig {
  /// OPC-UA NodeId of the source node emitting the AlarmConditionType
  /// events (typically the parent Object that owns the condition, e.g.
  /// "ns=2;s=Tank.Pressure" or the Server object for system-wide alarms).
  std::string source_node_id_str;
  opcua::NodeId source_node_id;

  /// SOVD entity that should host the resulting fault.
  std::string entity_id;

  /// Source-level / fallback SOVD fault code (e.g. ``PLC_OVERPRESSURE``).
  /// Used when no ``mappings`` entry matches an observed event. May be empty
  /// when every alarm is routed through ``mappings``.
  std::string fault_code;

  /// Optional severity override. When empty, ``AlarmStateMachine`` derives
  /// the SOVD severity bucket from the event's ``Severity`` (1-1000) per
  /// the convention documented in design/index.rst.
  std::string severity_override;

  /// Optional friendly message override; falls back to the event's
  /// ``Message`` field when empty.
  std::string message_override;

  /// Issue #389: per-condition-identity mappings (multi-alarm). Resolved in
  /// declaration order; first match wins, falling back to the source-level
  /// ``fault_code`` above.
  std::vector<AlarmMapping> mappings;

  /// Issue #389: extra event fields to append to the fault description.
  std::vector<AssociatedValueRef> associated_values;
};

/// Result of resolving an observed event against an ``AlarmEventConfig``
/// (issue #389). ``matched`` is false when neither a mapping nor the
/// source-level fault_code applies (the event should be ignored).
struct ResolvedAlarm {
  std::string fault_code;
  std::string severity_override;
  std::string message_override;
  bool matched{false};
};

/// OPC-UA Severity (1-1000) -> SOVD severity bucket thresholds for
/// ``auto_alarms`` (zero-config native A&C). Mirrors the fixed bands
/// ``OpcuaPlugin::map_severity`` uses for explicit ``event_alarms``
/// (>=801 CRITICAL, >=501 ERROR, >=201 WARNING, else INFO) but is
/// per-deployment overridable via ``auto_alarms.severity_bands`` since a PLC
/// vendor's own severity convention need not match ours.
struct AutoAlarmsSeverityBands {
  uint16_t critical_min{801};
  uint16_t error_min{501};
  uint16_t warning_min{201};
};

/// Zero-config native OPC-UA Alarms & Conditions (completes the #509
/// discovery -> #510 auto_browse -> native faults story with no per-alarm
/// ``event_alarms`` mapping). When enabled, the plugin subscribes to
/// ``source_node_id`` (default the Server object, ``i=2253``) for
/// AlarmConditionType events and derives a fault for every condition it does
/// not already know from an explicit ``event_alarms`` mapping (issue #389
/// mappings always take precedence - see ``NodeMap::resolve_alarm``).
struct AutoAlarmsConfig {
  bool enabled{false};

  /// EventNotifier source to subscribe. Default is the Server object
  /// (``i=2253``, "system-wide" catch-all): the natural zero-config choice
  /// since Program_Alarm / ProDiag style conditions on Siemens, Beckhoff and
  /// CodeSys controllers surface there without the operator having to find
  /// the owning Object first.
  std::string source_node_id_str = "i=2253";
  opcua::NodeId source_node_id;

  /// SOVD entity that hosts an auto-derived fault when its SourceNode is not
  /// a known node-map entry. Defaults to ``"<component_id>_alarms"`` at load
  /// time (see ``NodeMap::load``) - deliberately not the bare
  /// ``component_id``, which the PLC runtime already registers as a
  /// Component entity (a second App entity with the same id collides in the
  /// SOVD merge pipeline). When the event's SourceNode matches an existing
  /// ``nodes:`` entry, the fault is hosted on THAT entry's entity instead (a
  /// more specific home than this fallback).
  std::string entity_id;

  /// When true (default), an auto-derived alarm clears as soon as
  /// ActiveState becomes false, bypassing BOTH halves of the
  /// Acknowledge/Confirm gate that otherwise applies to native alarms
  /// (``AlarmStateMachine``'s clear rule is ``acked && (confirmed ||
  /// !require_confirm_for_clear)`` - dropping only the Confirm half is not
  /// enough, since a zero-config alarm has no SOVD ``acknowledge_fault``
  /// operation registered to ever set Acked either). Zero-config is meant to
  /// "just work" without an operator workflow, and several PLCs (e.g.
  /// Siemens S7-1500) do not implement the optional Confirm transition at
  /// all - without this, such an alarm would latch forever. Set false to
  /// inherit the poller's normal ``require_confirm_for_clear`` gating
  /// (Acknowledge still required) for auto-derived alarms too.
  bool auto_clear{true};

  AutoAlarmsSeverityBands severity_bands;

  /// Case-sensitive substring filters against ConditionName / SourceName /
  /// Message. ``exclude`` is checked first: any match drops the event
  /// entirely (used to filter Siemens Server-object system messages such as
  /// "CPU not in RUN" that are not real alarms). ``include`` is then checked
  /// only if non-empty: the event is dropped unless at least one pattern
  /// matches. Both empty (the default) admits every event.
  std::vector<std::string> include_patterns;
  std::vector<std::string> exclude_patterns;
};

/// Mapping entry: OPC-UA NodeId -> SOVD entity data point
struct NodeMapEntry {
  std::string node_id_str;          // OPC-UA node ID string (e.g., "ns=1;s=TankLevel")
  opcua::NodeId node_id;            // Parsed NodeId
  std::string entity_id;            // SOVD entity this belongs to (e.g., "tank_process")
  std::string entity_display_name;  // Human-readable entity name (auto_browse path); "" for hand-written
  std::string data_name;            // Data point name (e.g., "tank_level")
  std::string display_name;         // Human-readable name
  std::string unit;                 // Unit of measurement (e.g., "mm", "C", "bar")
  std::string data_type;            // "float", "int", "bool", "string"
  bool writable{false};             // Can be written via x-plc-operations
  std::optional<double> min_value;  // Optional range validation for writes
  std::optional<double> max_value;
  std::optional<AlarmConfig> alarm;  // Optional threshold alarm -> fault mapping (back-compat)
  std::string ros2_topic;            // ROS 2 topic for value bridging (auto: /plc/{entity}/{name})

  // Shared fault-detection rule for this point: threshold, status-word bit
  // decode, or fault-code enum. Built at load time from the ``alarm:``,
  // ``status_bits:`` or ``fault_enum:`` node-map block. The poller evaluates
  // this through the shared ``fault_detection::evaluate`` regardless of mode.
  std::optional<ros2_medkit::fault_detection::DetectionRule> detection;

  bool has_range() const {
    return min_value.has_value() && max_value.has_value();
  }
};

/// Configuration for the recursive OPC-UA address-space walker (auto_browse).
/// Populated either from the node-map YAML's ``auto_browse:`` block (bool or
/// map) or overlaid from the plugin's ``plugins.opcua.auto_browse`` JSON/ROS
/// param - see ``OpcuaPlugin::configure``. Consumed by ``AutoBrowser``
/// (address_space_browser.hpp) once the client has a live session.
struct AutoBrowseConfig {
  bool enabled{false};

  /// Node IDs to start the walk from, in NodeMap::parse_node_id string form.
  /// Default is the standard Objects folder (``i=85``); every well-formed
  /// server organizes its instance tree underneath it, including the OPC-UA
  /// DI ``DeviceSet`` folder used to reach a PLC's nameplate + process data.
  std::vector<std::string> root_node_ids{"i=85"};

  /// Recursion depth cap measured from a root node (root's direct children
  /// are depth 1). Guards against a pathological / cyclic address space.
  int max_depth{8};

  /// Total node budget for one browse() call (root nodes + every child
  /// considered, whether or not it is namespace-filtered). Once reached the
  /// walk stops descending further and the caller is warned that the tree may
  /// be incomplete.
  std::size_t max_nodes{5000};

  /// Namespace allow-list. When non-empty, only these namespace indices are
  /// eligible to become entities/data points (and to be recursed into);
  /// ``namespace_deny`` is ignored. Empty (default) means "allow all except
  /// denied".
  std::vector<uint16_t> namespace_allow;

  /// Namespace deny-list, applied only when ``namespace_allow`` is empty.
  /// Defaults to ``{0}``: ns=0 is the standard/infrastructure namespace
  /// (Server object, ServerCapabilities, Types, Views, ...) which is pure
  /// noise for entity discovery. Root nodes passed explicitly in
  /// ``root_node_ids`` are never filtered, even if their namespace is denied.
  std::vector<uint16_t> namespace_deny{0};

  /// When true, each candidate Variable is read once during the walk and
  /// dropped from the result if the read fails (Bad status) - a
  /// reachability/access check that filters out nodes whose DataType
  /// resolves but whose Value is not actually gettable (write-only nodes,
  /// permission-denied, etc). When false, the extra round-trip is skipped and
  /// every type-matched Variable is kept, which is faster to start up on very
  /// large trees.
  bool read_initial_values{true};

  /// When true (default) each discovered Variable's AccessLevel/UserAccessLevel
  /// is read and its ``writable`` flag is set from the server's CurrentWrite
  /// bit, so a config-less walk exposes a write surface exactly where the PLC
  /// permits it. When false, every auto-browsed point stays read-only and only
  /// an explicit node_map ``nodes:`` entry can promote it (legacy behaviour).
  bool infer_writable{true};
};

/// SOVD entity definition derived from the node map
struct PlcEntityDef {
  std::string id;
  std::string name;
  std::string component_id;
  bool is_app{true};
  std::vector<std::string> data_names;
  std::vector<std::string> writable_names;
  bool has_faults{false};
};

/// Manages the OPC-UA NodeId to SOVD entity mapping, loaded from YAML
/// Single source of truth for "does this entry get a ROS 2 value publisher":
/// create_value_publishers() creates one exactly for these entries, and
/// introspect() declares exactly these topics on the owning entity - the two
/// must never drift apart or triggers bind to forever-idle topics (medkit
/// issue #584).
inline bool entry_has_value_publisher(const NodeMapEntry & entry) {
  return entry.data_type != "string" && !entry.ros2_topic.empty();
}

class NodeMap {
 public:
  NodeMap() = default;

  /// Load node map from YAML file
  /// @return true if loaded successfully
  bool load(const std::string & yaml_path);

  /// Get all entries
  const std::vector<NodeMapEntry> & entries() const {
    return entries_;
  }

  /// Get entries for a specific SOVD entity
  std::vector<const NodeMapEntry *> entries_for_entity(const std::string & entity_id) const;

  /// Get writable entries for a specific SOVD entity
  std::vector<const NodeMapEntry *> writable_entries_for_entity(const std::string & entity_id) const;

  /// Find entry by data_name within an entity
  const NodeMapEntry * find_by_data_name(const std::string & entity_id, const std::string & data_name) const;

  /// Find entry by OPC-UA node ID string
  const NodeMapEntry * find_by_node_id(const std::string & node_id_str) const;

  /// Get all entries that have threshold-based alarm configuration.
  /// Test / back-compat only - the live fault-evaluation path is
  /// detection_entries(); nothing reads this at runtime.
  std::vector<const NodeMapEntry *> alarm_entries() const;

  /// Get all entries carrying a shared fault-detection rule (threshold,
  /// status-word bit decode, or fault-code enum). This is the set the poller
  /// evaluates each cycle.
  std::vector<const NodeMapEntry *> detection_entries() const;

  /// Get all native OPC-UA AlarmConditionType event-mode entries (issue #386).
  const std::vector<AlarmEventConfig> & event_alarms() const {
    return event_alarms_;
  }

  /// Find an event-mode alarm by ``(entity_id, fault_code)`` (used by the
  /// SOVD ``acknowledge_fault`` / ``confirm_fault`` operations). Matches the
  /// source-level fault_code or any of the entry's mapping fault_codes.
  const AlarmEventConfig * find_event_alarm(const std::string & entity_id, const std::string & fault_code) const;

  /// Resolve an observed event against a config's mappings (issue #389).
  /// First matching mapping wins; falls back to the source-level fault_code.
  /// Pure / static so the precedence rules are unit-testable without a server.
  static ResolvedAlarm resolve_alarm(const AlarmEventConfig & cfg, const std::string & condition_name,
                                     const std::string & source_node, const std::string & event_type,
                                     const std::string & message);

  /// Get the zero-config native A&C configuration. ``enabled`` is false
  /// unless the node map YAML declares a truthy ``auto_alarms:`` or the
  /// plugin overlays a truthy ``plugins.opcua.auto_alarms`` param.
  const AutoAlarmsConfig & auto_alarms() const {
    return auto_alarms_;
  }

  /// Mutable access so ``OpcuaPlugin::configure`` can overlay the plugin's
  /// ``plugins.opcua.auto_alarms`` JSON/ROS param on top of whatever the
  /// node-map YAML declared (JSON wins - same precedence the env-var
  /// overrides use for the rest of the plugin config). Call
  /// ``finalize_auto_alarms_overlay`` afterwards to re-derive the fields
  /// ``load`` normally fills in and rebuild the entity defs.
  AutoAlarmsConfig & mutable_auto_alarms() {
    return auto_alarms_;
  }

  /// Re-derive the ``auto_alarms`` fields ``load`` normally computes after the
  /// plugin overlays a ``plugins.opcua.auto_alarms`` param (default
  /// ``entity_id`` from ``component_id``, parsed ``source_node_id``) and
  /// rebuild ``entity_defs_`` so a param-only zero-config deployment (no
  /// node-map file, so ``load`` never ran) still surfaces the alarms App in
  /// SOVD discovery. Idempotent - safe to call whether or not ``load`` ran.
  /// @return false when the overlaid config is invalid (enabled with an empty
  /// ``source_node_id``); the caller disables auto_alarms in that case.
  bool finalize_auto_alarms_overlay();

  /// Derive a stable SOVD fault_code for an auto-derived alarm with NO
  /// per-alarm mapping. Tiered, in order:
  ///   1. slug(ConditionName) + hash(SourceNode) - the OPC-UA condition
  ///      identity, when present.
  ///   2. slug(SourceName) + hash(SourceNode)    - the human-readable event
  ///      source, when ConditionName is empty.
  ///   3. a hash of SourceNode + EventType + Message, when neither name is
  ///      available.
  /// The SourceNode is folded into EVERY tier so two distinct conditions that
  /// share a ConditionName/SourceName but sit on different sources (e.g. two
  /// identical FB instances each raising "Overpressure") never collapse onto
  /// one fault_code - fault_manager keys/clears by code alone, so a collision
  /// would let one condition's clear wipe the other's still-active fault. The
  /// SourceNode is canonicalized first, so ``i=2253`` and ``ns=0;i=2253`` fold
  /// to one code. Tier 3 additionally folds in ``message`` deliberately: a
  /// real Siemens S7-1500 multiplexes every Program_Alarm of one FB through a
  /// single SourceNode (e.g. ``i=1845``) with no ConditionName/SourceName,
  /// distinguished only by Message text ("pa" vs "pa2"). Message is NOT in
  /// tiers 1/2, so a condition whose Message differs between its active and
  /// inactive notifications still maps to one code. Pure / static so it is
  /// unit-testable without a server.
  static std::string derive_auto_fault_code(const std::string & condition_name, const std::string & source_name,
                                            const std::string & source_node_str, const std::string & event_type_str,
                                            const std::string & message);

  /// Apply ``auto_alarms.include``/``exclude`` substring filters to an
  /// observed event's identity fields. ``exclude`` wins first (drops the
  /// event on any match); a non-empty ``include`` then requires at least one
  /// match. Pure / static so it is unit-testable without a server.
  static bool auto_alarm_passes_filters(const std::string & condition_name, const std::string & source_name,
                                        const std::string & message, const std::vector<std::string> & include_patterns,
                                        const std::vector<std::string> & exclude_patterns);

  /// Map a raw OPC-UA event Severity (1-1000) to a SOVD severity bucket using
  /// ``bands`` (``auto_alarms.severity_bands``, overridable per deployment).
  /// Pure / static so it is unit-testable without a server.
  static std::string map_auto_severity(uint16_t severity, const AutoAlarmsSeverityBands & bands);

  /// Get derived SOVD entity definitions
  const std::vector<PlcEntityDef> & entity_defs() const {
    return entity_defs_;
  }

  /// Get area ID configured for PLC systems
  const std::string & area_id() const {
    return area_id_;
  }
  const std::string & area_name() const {
    return area_name_;
  }

  /// Get component ID for the PLC runtime
  const std::string & component_id() const {
    return component_id_;
  }
  const std::string & component_name() const {
    return component_name_;
  }

  /// Override the SOVD component id/name at runtime. Config-less discovery
  /// calls this with an identity derived from the device itself (see
  /// ``derive_component_identity``) so the component never surfaces the
  /// neutral placeholder default. Must be called before ``build_entity_defs``
  /// runs (i.e. before ``finalize_auto_alarms_overlay`` /
  /// ``merge_auto_browsed_entries``) so every derived reference - entity_defs
  /// ``component_id``, the ``<component_id>_alarms`` auto-alarm entity, and the
  /// introspect component - picks up the same id. No-op on empty ``id``.
  void set_component_identity(const std::string & id, const std::string & name) {
    if (id.empty()) {
      return;
    }
    component_id_ = id;
    component_name_ = name.empty() ? id : name;
  }

  /// Whether auto-browse mode is enabled
  bool auto_browse() const {
    return auto_browse_config_.enabled;
  }

  /// Full auto_browse configuration (root nodes, depth/node caps, namespace
  /// filter, ...), loaded from the node-map YAML's ``auto_browse:`` block.
  const AutoBrowseConfig & auto_browse_config() const {
    return auto_browse_config_;
  }

  /// Mutable access so ``OpcuaPlugin::configure`` can overlay the plugin's
  /// ``plugins.opcua.auto_browse`` JSON/ROS param on top of whatever the
  /// node-map YAML declared (JSON wins - same precedence the env-var
  /// overrides use for the rest of the plugin config).
  AutoBrowseConfig & mutable_auto_browse_config() {
    return auto_browse_config_;
  }

  /// Merge entries discovered by a live ``AutoBrowser`` walk into the map.
  /// An auto-browsed entry whose ``node_id_str`` already has an explicit
  /// (YAML ``nodes:``) mapping is dropped - explicit config always wins.
  /// Rebuilds ``entity_defs_`` when at least one entry was added.
  /// @return number of entries actually added.
  std::size_t merge_auto_browsed_entries(std::vector<NodeMapEntry> entries);

  /// Parse an OPC-UA node ID string into an `opcua::NodeId`.
  ///
  /// Supports the full OPC 10000-6 section 5.3.1.10 format:
  ///
  ///     [ns=<ns-index>;]i=<uint32>     numeric
  ///     [ns=<ns-index>;]s=<string>     string (Siemens, Beckhoff, ...)
  ///     [ns=<ns-index>;]g=<guid>       GUID   (Microsoft-style UUID)
  ///     [ns=<ns-index>;]b=<base64>     opaque ByteString
  ///
  /// When the `ns=` prefix is omitted the namespace defaults to 0.
  /// Returns a default-constructed `opcua::NodeId` on parse failure; the
  /// plugin treats such an entry as unresolvable at runtime.
  ///
  /// Declared public so it can be unit-tested directly.
  static opcua::NodeId parse_node_id(const std::string & str);

 private:
  /// Build entity_defs_ from entries_
  void build_entity_defs();

  std::vector<NodeMapEntry> entries_;
  std::vector<PlcEntityDef> entity_defs_;
  std::unordered_map<std::string, std::vector<size_t>> entity_index_;  // entity_id -> entry indices
  std::unordered_map<std::string, size_t> node_id_index_;              // node_id_str -> entry index

  // Issue #386: native OPC-UA AlarmConditionType subscriptions, loaded from
  // top-level ``event_alarms:`` in the YAML. Stored separately from
  // ``entries_`` because event-mode alarms do not have a scalar node to
  // poll; their entity definitions are merged into ``entity_defs_`` via
  // build_entity_defs() so SOVD discovery is unaffected.
  std::vector<AlarmEventConfig> event_alarms_;

  // Zero-config native A&C (issue #(auto-alarms)), loaded from top-level
  // ``auto_alarms:``. Unlike ``event_alarms_`` this carries no per-condition
  // mappings; the poller derives a fault_code/entity/severity per observed
  // event at runtime (see NodeMap::derive_auto_fault_code / map_auto_severity).
  AutoAlarmsConfig auto_alarms_;

  std::string area_id_ = "plc_systems";
  std::string area_name_ = "PLC Systems";
  // Neutral placeholder only. In config-less discovery the plugin overrides
  // these from the device's own read identity (set_component_identity); an
  // explicit node-map YAML overrides them from ``component_id`` /
  // ``component_name``. No device-specific product string is ever baked in.
  std::string component_id_ = "opcua_device";
  std::string component_name_ = "OPC UA Device";
  AutoBrowseConfig auto_browse_config_;
};

}  // namespace ros2_medkit_gateway
