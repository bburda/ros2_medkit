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

// Test-only executable: the real ActionStatusBridgeNode with a deterministic
// stand-in for the one inherently racy primitive - resolving the action server's
// node FQN from DDS discovery. Everything else (reporter_for, the provisional
// bookkeeping, reattribute_provisional, report()+supersede, the timers) is the
// unmodified production code, exercised end to end against a real FaultManager.
//
// This reproduces issue #467's "abort before DDS discovery resolves the server
// FQN" race without depending on real DDS timing: the FIRST few resolutions for
// an action return "" (as rcl does while a participant's node name has not yet
// propagated), so the bridge falls back to the provisional action-name source
// and reports the fault under it. A later resolution returns the real FQN, so
// the rescan's reattribute_provisional() re-reports under the FQN with
// supersedes_source_id set and the FaultManager drops the provisional source.

#include <map>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_action_status_bridge/action_status_bridge_node.hpp"

namespace {

// server_fqn_for_action is a private virtual (a Non-Virtual-Interface
// customization point); a derived class may still override it, and the base's
// internal calls dispatch here through the vtable.
class SlowDiscoveryBridge : public ros2_medkit_action_status_bridge::ActionStatusBridgeNode {
 public:
  SlowDiscoveryBridge() : ActionStatusBridgeNode() {
  }
  ~SlowDiscoveryBridge() override = default;
  SlowDiscoveryBridge(const SlowDiscoveryBridge &) = delete;
  SlowDiscoveryBridge & operator=(const SlowDiscoveryBridge &) = delete;
  SlowDiscoveryBridge(SlowDiscoveryBridge &&) = delete;
  SlowDiscoveryBridge & operator=(SlowDiscoveryBridge &&) = delete;

 private:
  // Number of leading resolutions per action that are forced to "" (unresolved).
  // Call 1 is reporter_for's first report (-> provisional); the next few are
  // reattribute_provisional rescans that keep the provisional entry; after that
  // the real FQN is returned and the supersede fires. Keeping it > 1 makes the
  // provisional state observable across several rescans instead of a single tick.
  static constexpr int kForcedUnresolvedCalls = 3;

  std::string server_fqn_for_action(const std::string & action_name) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      int & n = calls_[action_name];
      if (n < kForcedUnresolvedCalls) {
        ++n;
        return "";  // simulate slow discovery: force the provisional fallback
      }
    }
    // Resolve for real via the public graph API, mirroring the production
    // server_fqn_for_action / server_fqn_from_endpoint logic.
    constexpr const char * kUnknownName = "_NODE_NAME_UNKNOWN_";
    constexpr const char * kUnknownNamespace = "_NODE_NAMESPACE_UNKNOWN_";
    for (const auto & p : get_publishers_info_by_topic(action_name + "/_action/status")) {
      const std::string & name = p.node_name();
      const std::string & ns = p.node_namespace();
      if (name.empty() || name == kUnknownName || ns == kUnknownNamespace) {
        continue;
      }
      std::string fqn;
      if (ns.empty() || ns == "/") {
        fqn = "/";
      } else {
        fqn = ns;
        fqn += "/";
      }
      fqn += name;
      return fqn;
    }
    return "";
  }

  std::mutex mutex_;
  std::map<std::string, int> calls_;
};

}  // namespace

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlowDiscoveryBridge>());
  rclcpp::shutdown();
  return 0;
}
