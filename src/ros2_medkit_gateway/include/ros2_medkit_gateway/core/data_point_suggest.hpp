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

// Suggestion helpers for "data point does not exist" errors. PLC plugins
// register symbols under a sanitized leaf name (MAIN.counter -> counter), so
// an operator typing the source notation gets no hit; these helpers recover
// the intended name instead of dumping an alphabetical prefix of ~200 symbols.

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

inline size_t edit_distance(const std::string & a, const std::string & b) {
  std::vector<size_t> prev(b.size() + 1), cur(b.size() + 1);
  for (size_t j = 0; j <= b.size(); ++j) {
    prev[j] = j;
  }
  for (size_t i = 1; i <= a.size(); ++i) {
    cur[0] = i;
    for (size_t j = 1; j <= b.size(); ++j) {
      const size_t sub = prev[j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1);
      cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, sub});
    }
    std::swap(prev, cur);
  }
  return prev[b.size()];
}

/// MAIN.counter -> counter: leaf after the last '.' or '/', lowercased with
/// non-alphanumerics mapped to '_' - the same shape the PLC bridges produce
/// when they sanitize a symbol into a data point id.
inline std::string sanitized_leaf(const std::string & input) {
  const auto pos = input.find_last_of("./");
  std::string leaf = (pos == std::string::npos) ? input : input.substr(pos + 1);
  std::string out;
  out.reserve(leaf.size());
  for (char c : leaf) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    } else if (c == '_' || c == '-') {
      out += c;
    } else {
      out += '_';
    }
  }
  return out;
}

/// Best existing name for a miss, or empty when nothing is close. The
/// sanitized leaf wins exactly (the deterministic mapping); otherwise the
/// closest name within an input-length-scaled edit distance budget.
inline std::string suggest_data_point(const std::string & input, const std::vector<std::string> & names) {
  const std::string leaf = sanitized_leaf(input);
  if (!leaf.empty() && std::find(names.begin(), names.end(), leaf) != names.end()) {
    return leaf;
  }
  // Each candidate distance gets the budget of the string it was measured
  // against: a long namespace prefix must not buy a short leaf a huge budget
  // (MAIN.Very.Long.Prefix.rpm would otherwise "suggest" whatever is nearest
  // to a three-letter leaf).
  const size_t input_budget = std::max<size_t>(2, input.size() / 4);
  const size_t leaf_budget = std::max<size_t>(2, leaf.size() / 4);
  size_t best_score = SIZE_MAX;
  std::string best;
  for (const auto & name : names) {
    const size_t d_input = edit_distance(input, name);
    size_t score = d_input <= input_budget ? d_input : SIZE_MAX;
    if (!leaf.empty()) {
      const size_t d_leaf = edit_distance(leaf, name);
      if (d_leaf <= leaf_budget && d_leaf < score) {
        score = d_leaf;
      }
    }
    if (score < best_score || (score == best_score && score != SIZE_MAX && name < best)) {
      best_score = score;
      best = name;
    }
  }
  return best_score != SIZE_MAX ? best : std::string{};
}

/// Up to n names ordered by edit distance to the input (ties alphabetical),
/// so a truncated "available:" list shows the relevant neighborhood instead
/// of an alphabetical prefix.
inline std::vector<std::string> closest_data_points(const std::string & input, const std::vector<std::string> & names,
                                                    size_t n) {
  const std::string leaf = sanitized_leaf(input);
  std::vector<std::pair<size_t, std::string>> ranked;
  ranked.reserve(names.size());
  for (const auto & name : names) {
    size_t d = edit_distance(input, name);
    if (!leaf.empty()) {
      d = std::min(d, edit_distance(leaf, name));
    }
    ranked.emplace_back(d, name);
  }
  std::sort(ranked.begin(), ranked.end());
  std::vector<std::string> out;
  out.reserve(std::min(n, ranked.size()));
  for (size_t i = 0; i < ranked.size() && i < n; ++i) {
    out.push_back(ranked[i].second);
  }
  return out;
}

}  // namespace ros2_medkit_gateway
