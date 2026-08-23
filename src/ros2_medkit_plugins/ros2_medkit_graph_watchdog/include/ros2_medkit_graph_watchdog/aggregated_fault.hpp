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
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"  // full IntrospectionInput
#include "ros2_medkit_graph_watchdog/detector.hpp"

namespace ros2_medkit_graph_watchdog {

/// Id of the synthetic App that owns every GRAPH_* fault. The plugin publishes it as an
/// external entity from its own IntrospectionProvider (graph_watchdog_plugin.cpp).
///
/// It must be an entity the plugin owns, not a borrowed one. Scoping these faults to the
/// host Component instead - the obvious-looking alternative - makes them reachable from NO
/// endpoint: `collect_component_app_fqns` (fault_scope.cpp) only puts a
/// component's bare id in the scope set when `external` is true, and a runtime host
/// Component built by HostInfoProvider never sets it, so `/components/<host>/faults` and
/// the scoped detail route both drop the fault. There is no server-level
/// `/faults/{code}` route either, so the flat `/faults` list was the only place these
/// faults existed.
///
/// Hanging each fault on the FQN of the node it is ABOUT does not work either:
/// fault identity in the store is `fault_code` alone (`fault_code TEXT PRIMARY KEY`),
/// and `reporting_sources` accumulates into a set on that one record while
/// `fault_in_source_scope` requires EVERY source to be in scope. Two dead nodes would
/// therefore hide GRAPH_NODE_DISAPPEARED from both of their `/apps/<fqn>/faults` pages.
/// One owned entity per code keeps exactly one source per record.
///
/// Same shape as the ADS plugin's device entity, for the same reason.
inline constexpr const char * kGraphWatchdogEntityId = "graph_watchdog";

/// The source_id every aggregated GRAPH_* fault is raised under. Constant by design:
/// see kGraphWatchdogEntityId. The snapshot is unused - it is kept in the signature
/// because every call site already has the context in hand.
inline std::string graph_source_id(const DetectorContext & /*ctx*/) {
  return kGraphWatchdogEntityId;
}

/// Level-triggered aggregate emitter for one fault code. Each tick a detector
/// hands emit() the currently-affected entities (ordered key -> detail phrase).
/// Non-empty -> raise ONE fault enumerating them; empty -> clear. Emitting every
/// tick is intentional: the fault_manager debounce advances toward CONFIRMED
/// while affected and, with healing_enabled, toward HEALED while clear.
class AggregatedFault {
 public:
  /// Cap on the generated description. One aggregated fault names every affected entity, so
  /// on a large graph the raw text grows without bound - an oversized ReportFault description
  /// then travels into every /faults payload and the fault_manager's store. Every detector
  /// aggregates through this helper, so the cap is applied in one place rather than copied per
  /// detector. A detector may additionally trim what a SINGLE entity contributes before handing
  /// it over (param_drift does), so that one entity cannot spend the whole budget by itself.
  static constexpr std::size_t kMaxDescriptionChars = 480;

  AggregatedFault(const char * code, std::uint8_t severity) : code_(code), severity_(severity) {
  }

  /// Returns whatever ctx.raise_fault()/ctx.clear_fault() returned - true only if the
  /// request actually reached async_send_request(), never merely because `affected` was
  /// non-empty. That proves local enqueue, not fault_manager receipt (the client is
  /// fire-and-forget) - see DetectorContext::raise_fault's own doc for exactly what this
  /// return value does and does not prove.
  bool emit(DetectorContext & ctx, const std::map<std::string, std::string> & affected) const {
    const std::string source = graph_source_id(ctx);
    if (affected.empty()) {
      return ctx.clear_fault(code_, source);
    }
    return ctx.raise_fault(code_, severity_, describe(affected), source);
  }

  /// emit() with a caller-chosen ORDER for the description. Return value: see emit()'s own
  /// doc.
  ///
  /// The map overload above lists entities lexicographically, which is fine when every
  /// entry is equally interesting. It is not fine when the set mixes long-standing entries
  /// with a brand-new one and the text is capped: the new entry can sort last and be cut.
  /// `order` names the keys of `affected` in the order they should appear; any key of
  /// `affected` missing from `order` is appended afterwards, so a caller can never silently
  /// drop an entity by getting the ordering wrong.
  bool emit_ordered(DetectorContext & ctx, const std::map<std::string, std::string> & affected,
                    const std::vector<std::string> & order) const {
    const std::string source = graph_source_id(ctx);
    if (affected.empty()) {
      return ctx.clear_fault(code_, source);
    }
    return ctx.raise_fault(code_, severity_, describe_ordered(affected, order), source);
  }

  /// Join the affected entities into one description, capped at kMaxDescriptionChars.
  /// Public so the cap is directly unit-testable; the truncation marker is inside the cap,
  /// so the returned string never exceeds it.
  static std::string describe(const std::map<std::string, std::string> & affected) {
    std::vector<std::string> keys;
    keys.reserve(affected.size());
    for (const auto & [key, detail] : affected) {
      (void)detail;
      keys.push_back(key);
    }
    return describe_ordered(affected, keys);
  }

  /// describe() over an explicit key order. Keys of `affected` not named in `order` are
  /// appended in map order, so a partial or stale `order` degrades to the old behaviour
  /// rather than dropping entities.
  static std::string describe_ordered(const std::map<std::string, std::string> & affected,
                                      const std::vector<std::string> & order) {
    std::string desc;
    std::set<std::string> emitted;
    const auto append = [&desc](const std::string & key, const std::string & detail) {
      if (!desc.empty()) {
        desc += "; ";
      }
      desc += detail.empty() ? key : detail;
    };
    for (const auto & key : order) {
      const auto it = affected.find(key);
      if (it == affected.end() || !emitted.insert(key).second) {
        continue;
      }
      append(key, it->second);
      if (desc.size() > kMaxDescriptionChars) {
        break;  // nothing appended later can shrink it - stop building and cap below
      }
    }
    if (desc.size() <= kMaxDescriptionChars) {
      for (const auto & [key, detail] : affected) {
        if (emitted.count(key) != 0) {
          continue;
        }
        append(key, detail);
        if (desc.size() > kMaxDescriptionChars) {
          break;
        }
      }
    }
    if (desc.size() > kMaxDescriptionChars) {
      desc = desc.substr(0, kMaxDescriptionChars - 3) + "...";
    }
    return desc;
  }

 private:
  const char * code_;
  std::uint8_t severity_;
};

}  // namespace ros2_medkit_graph_watchdog
