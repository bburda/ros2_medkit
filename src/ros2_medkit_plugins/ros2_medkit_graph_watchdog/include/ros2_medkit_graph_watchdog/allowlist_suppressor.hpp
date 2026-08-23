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

#include <set>
#include <string>
#include <utility>

#include "ros2_medkit_graph_watchdog/suppressor.hpp"

namespace ros2_medkit_graph_watchdog {

/// Operator-declared veto. Suppresses a key iff it - or a name the SAME entity is also
/// known by - is present, verbatim, in the configured allow set: never a prefix or a
/// substring, so allowlisting `/r1/x` can never reach into `/r2/x` on a fleet where the two
/// share a suffix.
///
/// Deliberately mirrors lifecycle_expectation_detector.cpp's own require_active matching
/// (`id != app.id && id != fqn && id != leaf`) rather than inventing a second convention:
/// a bare name is the natural operator input, and `App::id` is unstable - it gains a
/// namespace prefix the moment a same-bare-name collision exists anywhere in the graph -
/// so the two config surfaces (require_active, allowlist) need to accept the same shapes or
/// an operator who has one working would reasonably expect the other to behave the same way
/// and be wrong.
///
/// The two surfaces reach that same three-way match through different mechanics, though:
/// lifecycle_expectation matches while the node is still PRESENT, so `app.id` and its fqn
/// are both in hand from the very same `ctx.snapshot->apps` entry it is iterating. This
/// class is asked about a key ONLY after the entity is already gone - present-tense
/// `ctx.snapshot` cannot answer "what was this dead key's id" at all. suppresses() itself
/// therefore still only ever sees a single string and covers the two forms derivable from
/// that string alone (the key verbatim, and its own bare leaf); the id form needs the
/// caller to have captured `App::id` while the entity was still alive and to re-offer it
/// through allows() below - see node_death_detector.cpp's tick() for where that capture
/// happens and why.
///
/// Shared by any detector whose candidate keys are plain strings (node_death's are
/// `App::effective_fqn()`; a future tf_stale conversion would use its own "parent->child"
/// pair strings) - the exact-match contract does not care which.
///
/// The empty string is never treated specially: a caller that means "match nothing" for
/// an empty entity_key has to actually insert "" into the allow set for that to happen.
/// Detector configure() code is expected to have already dropped empty entries when
/// building the set it hands here.
class AllowlistSuppressor : public Suppressor {
 public:
  explicit AllowlistSuppressor(std::set<std::string> allow) : allow_(std::move(allow)) {
  }

  /// True iff `candidate` is present, verbatim, in the configured allow set. Public (beyond
  /// what the Suppressor interface requires) so a caller holding a form of an entity's
  /// identity that suppresses() itself has no way to reach - `App::id`, captured while the
  /// entity was still present - can still check it against the same set. See the class doc.
  bool allows(const std::string & candidate) const {
    return allow_.count(candidate) > 0;
  }

  /// Matches `entity_key` verbatim, or the bare leaf of it (the substring after the last
  /// '/', or the whole key if it carries no '/') - the two forms answerable from the key
  /// alone. The id form is NOT checked here; see allows() and the class doc.
  bool suppresses(const std::string & entity_key, const DetectorContext & /*ctx*/) const override {
    if (allows(entity_key)) {
      return true;
    }
    const auto slash = entity_key.rfind('/');
    if (slash == std::string::npos) {
      return false;  // entity_key has no '/': it already IS its own bare leaf, checked above
    }
    return allows(entity_key.substr(slash + 1));
  }

  /// An operator-declared entry is a standing fact about that key for as long as this
  /// configuration is loaded - it does not start matching and then stop on its own, so a
  /// key it vetoes may safely have its tracker bookkeeping reclaimed.
  bool durable() const override {
    return true;
  }

 private:
  std::set<std::string> allow_;
};

}  // namespace ros2_medkit_graph_watchdog
