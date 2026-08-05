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

#include "ros2_medkit_gateway/core/fault_trigger_engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <utility>

#include "ros2_medkit_gateway/core/data_point_suggest.hpp"

namespace ros2_medkit_gateway {

namespace {

constexpr double kEqualEpsilon = 1e-9;

std::string json_string(const nlohmann::json & j, const char * key) {
  if (j.contains(key) && j[key].is_string()) {
    return j[key].get<std::string>();
  }
  return {};
}

}  // namespace

FaultTriggerEngine::FaultTriggerEngine(std::string storage_path, ValueFetcher fetcher, FaultReportFn report,
                                       FaultClearFn clear, LogFn log, DataPointNamesFn data_point_names,
                                       EntityExistsFn entity_exists)
  : storage_path_(std::move(storage_path))
  , fetcher_(std::move(fetcher))
  , report_(std::move(report))
  , clear_(std::move(clear))
  , log_(std::move(log))
  , data_point_names_(std::move(data_point_names))
  , entity_exists_(std::move(entity_exists)) {
  load();
}

bool FaultTriggerEngine::valid_operator(const std::string & op) {
  static const std::array<const char *, 5> kOps{">", "<", ">=", "<=", "=="};
  return std::any_of(kOps.begin(), kOps.end(), [&op](const char * o) {
    return op == o;
  });
}

bool FaultTriggerEngine::valid_severity(const std::string & sev) {
  static const std::array<const char *, 4> kSev{"INFO", "WARNING", "ERROR", "CRITICAL"};
  return std::any_of(kSev.begin(), kSev.end(), [&sev](const char * s) {
    return sev == s;
  });
}

bool FaultTriggerEngine::compare(double value, const std::string & op, double threshold) {
  if (op == ">") {
    return value > threshold;
  }
  if (op == "<") {
    return value < threshold;
  }
  if (op == ">=") {
    return value >= threshold;
  }
  if (op == "<=") {
    return value <= threshold;
  }
  if (op == "==") {
    return std::fabs(value - threshold) <= kEqualEpsilon;
  }
  return false;
}

nlohmann::json FaultTriggerEngine::rule_to_json(const FaultTriggerRule & r) {
  return nlohmann::json{{"id", r.id},
                        {"data_name", r.data_name},
                        {"operator", r.op},
                        {"threshold", r.threshold},
                        {"fault_code", r.fault_code},
                        {"severity", r.severity},
                        {"active", r.active}};
}

std::vector<FaultTriggerRule> FaultTriggerEngine::list(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<FaultTriggerRule> out;
  for (const auto & r : rules_) {
    if (r.app_id == app_id) {
      out.push_back(r);
    }
  }
  return out;
}

tl::expected<FaultTriggerRule, std::pair<int, std::string>> FaultTriggerEngine::create(const std::string & app_id,
                                                                                       const nlohmann::json & body) {
  if (!body.is_object()) {
    return tl::make_unexpected(std::make_pair(400, std::string("request body must be a JSON object")));
  }

  FaultTriggerRule rule;
  rule.app_id = app_id;
  rule.data_name = json_string(body, "data_name");
  rule.op = json_string(body, "operator");
  rule.fault_code = json_string(body, "fault_code");
  rule.severity = json_string(body, "severity");
  rule.active = body.contains("active") && body["active"].is_boolean() ? body["active"].get<bool>() : true;

  if (rule.data_name.empty()) {
    return tl::make_unexpected(std::make_pair(400, std::string("'data_name' is required")));
  }
  if (!valid_operator(rule.op)) {
    return tl::make_unexpected(std::make_pair(400, std::string("'operator' must be one of >, <, >=, <=, ==")));
  }
  if (!body.contains("threshold") || !body["threshold"].is_number()) {
    return tl::make_unexpected(std::make_pair(400, std::string("'threshold' must be a number")));
  }
  rule.threshold = body["threshold"].get<double>();
  if (rule.fault_code.empty()) {
    return tl::make_unexpected(std::make_pair(400, std::string("'fault_code' is required")));
  }
  if (!valid_severity(rule.severity)) {
    return tl::make_unexpected(
        std::make_pair(400, std::string("'severity' must be one of INFO, WARNING, ERROR, CRITICAL")));
  }

  // Reject a rule targeting an app the entity registry does not know (404): a
  // bogus app_id must not silently reserve a global fault_code. Checked before
  // the data-point probe so that a nullopt there means "known app, points
  // unreadable right now" (transient outage), never "unknown app".
  if (entity_exists_ && !entity_exists_(app_id)) {
    return tl::make_unexpected(std::make_pair(404, "app '" + app_id + "' does not exist"));
  }

  // A rule bound to a data point the app does not have can never fire, yet it
  // would list as active - a silently dead alarm. Reject it at creation when
  // the app's points are enumerable; nullopt (entity unreadable right now)
  // skips the check so a transient PLC outage does not block rule creation.
  if (data_point_names_) {
    const auto names = data_point_names_(rule.app_id);
    if (names.has_value() && std::find(names->begin(), names->end(), rule.data_name) == names->end()) {
      // ~200 PLC symbols make an alphabetical prefix useless: rank by edit
      // distance so the neighborhood of the typo is what gets listed, and
      // resolve the deterministic MAIN.counter -> counter mapping outright.
      constexpr size_t kMaxListed = 20;
      const auto listed = closest_data_points(rule.data_name, *names, kMaxListed);
      std::string available;
      for (size_t i = 0; i < listed.size(); ++i) {
        available += (i == 0 ? "" : ", ") + listed[i];
      }
      std::string msg = "data point '" + rule.data_name + "' does not exist on app '" + rule.app_id + "'";
      const std::string suggestion = suggest_data_point(rule.data_name, *names);
      if (!suggestion.empty()) {
        msg += " - did you mean '" + suggestion + "'?";
      }
      if (!available.empty()) {
        msg += names->size() > kMaxListed ? " (" + std::to_string(names->size()) + " available, closest " +
                                                std::to_string(listed.size()) + ": " + available + ")"
                                          : " (available: " + available + ")";
      }
      return tl::make_unexpected(std::make_pair(400, std::move(msg)));
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // fault_code is the PRIMARY KEY of the fault-manager store, so two rules
  // sharing one code would fight over the same stored fault (one rule's clear
  // erases the other's assertion) and make per-app delete/clear ambiguous.
  const auto dup = std::find_if(rules_.begin(), rules_.end(), [&](const FaultTriggerRule & r) {
    return r.fault_code == rule.fault_code;
  });
  if (dup != rules_.end()) {
    return tl::make_unexpected(
        std::make_pair(409, "fault_code '" + rule.fault_code + "' is already used by rule '" + dup->id + "' on app '" +
                                dup->app_id + "' - fault codes are global to the fault store, pick a distinct one"));
  }
  rule.id = "ftr_" + std::to_string(next_seq_++);
  rules_.push_back(rule);
  save_locked();
  if (log_) {
    log_("fault-trigger created: " + rule.id + " on " + app_id + "/" + rule.data_name + " " + rule.op + " " +
         std::to_string(rule.threshold) + " -> " + rule.fault_code + " (" + rule.severity + ")");
  }
  return rule;
}

bool FaultTriggerEngine::remove(const std::string & app_id, const std::string & id) {
  // Serialize the whole erase+clear against evaluate_once()'s report/clear (see
  // dispatch_mutex_): otherwise a DELETE landing between evaluate's snapshot and
  // its report_ would clear the fault here while evaluate re-asserts it, leaving
  // a stuck fault for a rule that no longer exists.
  std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
  std::string fault_code;
  bool was_crossed = false;
  bool found = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find_if(rules_.begin(), rules_.end(), [&](const FaultTriggerRule & r) {
      return r.app_id == app_id && r.id == id;
    });
    if (it == rules_.end()) {
      return false;
    }
    fault_code = it->fault_code;
    was_crossed = it->crossed;
    rules_.erase(it);
    save_locked();
    found = true;
  }
  // Clear outside mutex_ (the fault report path may re-enter unrelated code) but
  // still under dispatch_mutex_ so it is ordered against evaluate_once().
  if (found && was_crossed && clear_) {
    clear_(app_id, fault_code);
  }
  return found;
}

void FaultTriggerEngine::evaluate_once() {
  // Snapshot the active rules under the lock, run the (blocking) value fetch
  // outside every lock, then dispatch report/clear + fold the latch back under
  // dispatch_mutex_ (serialized against remove()).
  struct Pending {
    std::string id;
    std::string app_id;
    std::string data_name;
    std::string op;
    double threshold;
    std::string fault_code;
    std::string severity;
    bool crossed;
    std::optional<double> value;
  };
  std::vector<Pending> work;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & r : rules_) {
      if (r.active) {
        work.push_back(
            {r.id, r.app_id, r.data_name, r.op, r.threshold, r.fault_code, r.severity, r.crossed, std::nullopt});
      }
    }
  }
  if (work.empty()) {
    return;
  }

  // Blocking value fetch OUTSIDE any lock so neither mutex is held across I/O.
  for (auto & w : work) {
    w.value = fetcher_ ? fetcher_(w.app_id, w.data_name) : std::nullopt;
  }

  // Dispatch under dispatch_mutex_ so the whole re-check-still-exists -> report_
  // -> latch-fold section is atomic against remove()'s erase+clear. A rule
  // deleted since the snapshot is skipped, so evaluate never re-asserts a fault
  // that a concurrent DELETE just cleared.
  std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
  for (auto & w : work) {
    if (!w.value.has_value()) {
      continue;  // unreadable this poll: hold state, never false-clear a live fault
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool still_present = std::any_of(rules_.begin(), rules_.end(), [&](const FaultTriggerRule & r) {
        return r.id == w.id && r.app_id == w.app_id;
      });
      if (!still_present) {
        continue;  // removed under dispatch_mutex_: remove() already cleared it
      }
    }
    const bool now_crossed = compare(*w.value, w.op, w.threshold);
    if (now_crossed) {
      if (report_) {
        const std::string desc = w.data_name + " = " + std::to_string(*w.value) + " " + w.op + " " +
                                 std::to_string(w.threshold) + " (fault-trigger " + w.id + ")";
        // Re-report every poll while crossed: level-triggered, so the fault
        // confirms regardless of the manager's debounce threshold and stays
        // asserted until the value recovers.
        report_(w.app_id, w.fault_code, w.severity, desc);
      }
      w.crossed = true;
    } else {
      if (w.crossed && clear_) {
        clear_(w.app_id, w.fault_code);
      }
      w.crossed = false;
    }
  }

  // Fold latch state back into the live rules and persist any edge so a restart
  // mid-fault reloads the asserted latch and can still auto-clear on the falling
  // edge (a rule deleted mid-evaluation is simply skipped). Only writes on an
  // actual change, so a steadily-crossed rule does not re-save every poll.
  bool latch_changed = false;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & w : work) {
    auto it = std::find_if(rules_.begin(), rules_.end(), [&](FaultTriggerRule & r) {
      return r.id == w.id && r.app_id == w.app_id;
    });
    if (it != rules_.end() && it->crossed != w.crossed) {
      it->crossed = w.crossed;
      latch_changed = true;
    }
  }
  if (latch_changed) {
    save_locked();
  }
}

void FaultTriggerEngine::load() {
  if (storage_path_.empty()) {
    return;
  }
  std::ifstream in(storage_path_);
  if (!in.is_open()) {
    return;  // first run: no store yet
  }
  nlohmann::json j;
  try {
    in >> j;
  } catch (const nlohmann::json::exception & e) {
    if (log_) {
      log_("fault-trigger store '" + storage_path_ + "' is not valid JSON (" + e.what() + "); starting empty");
    }
    return;
  }
  if (!j.is_array()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & item : j) {
    if (!item.is_object()) {
      continue;
    }
    FaultTriggerRule r;
    r.id = json_string(item, "id");
    r.app_id = json_string(item, "app_id");
    r.data_name = json_string(item, "data_name");
    r.op = json_string(item, "operator");
    r.threshold = item.contains("threshold") && item["threshold"].is_number() ? item["threshold"].get<double>() : 0.0;
    r.fault_code = json_string(item, "fault_code");
    r.severity = json_string(item, "severity");
    r.active = item.contains("active") && item["active"].is_boolean() ? item["active"].get<bool>() : true;
    // Restore the latch (default false for pre-latch stores) so a fault asserted
    // before the restart still clears on the next falling edge instead of sticking.
    r.crossed = item.contains("crossed") && item["crossed"].is_boolean() ? item["crossed"].get<bool>() : false;
    if (r.id.empty() || r.app_id.empty() || r.data_name.empty() || !valid_operator(r.op) || r.fault_code.empty() ||
        !valid_severity(r.severity)) {
      continue;  // drop malformed persisted rows rather than crash on them
    }
    rules_.push_back(r);
    // Keep id generation past any persisted "ftr_<n>" so restarts don't collide.
    if (r.id.rfind("ftr_", 0) == 0) {
      try {
        const uint64_t n = std::stoull(r.id.substr(4));
        next_seq_ = std::max(next_seq_, n + 1);
      } catch (const std::exception &) {
        // non-numeric suffix: leave next_seq_ as is
      }
    }
  }
  if (log_) {
    log_("fault-trigger engine loaded " + std::to_string(rules_.size()) + " rule(s) from " + storage_path_);
  }
}

void FaultTriggerEngine::save_locked() const {
  if (storage_path_.empty()) {
    return;
  }
  nlohmann::json j = nlohmann::json::array();
  for (const auto & r : rules_) {
    nlohmann::json row = rule_to_json(r);
    row["app_id"] = r.app_id;    // store the owning entity too (not in the REST response)
    row["crossed"] = r.crossed;  // persist the latch (not in the REST DTO) so a restart
                                 // mid-fault can still auto-clear on the falling edge
    j.push_back(std::move(row));
  }
  const std::string tmp = storage_path_ + ".tmp";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out.is_open()) {
    if (log_) {
      log_("fault-trigger store '" + storage_path_ + "' not writable; rules will not persist");
    }
    return;
  }
  out << j.dump(2);
  out.close();
  std::rename(tmp.c_str(), storage_path_.c_str());
}

}  // namespace ros2_medkit_gateway
