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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/stream_proxy.hpp"

namespace ros2_medkit_gateway {

/// Where one peer's stream can be reached. Supplied by the caller rather than
/// read from AggregationManager, which lives in the ROS adapter and cannot be
/// named from here.
struct RelayTarget {
  std::string name;
  std::string url;
  /// Value to send as Authorization, or empty for none. Supplied by the
  /// caller, because whether a client's credentials may be forwarded to a peer
  /// is `aggregation.forward_auth` and that lives in the ROS adapter. A peer
  /// that requires authentication refuses an unauthenticated stream, so
  /// without this the relay is silently dead against exactly the deployments
  /// that authenticate.
  std::string auth_header;
};

/**
 * @brief Relays the peers' event streams into an aggregating gateway's own.
 *
 * An aggregating gateway runs on its own ROS domain. Its fault_manager is the
 * one nothing reports to, so a stream fed only from the local graph is open,
 * valid and empty, which reads to a client exactly like a healthy system. The
 * collection route already fans out; this is the same answer for the stream.
 *
 * Streams are opened on the first attached client and closed with the last.
 * The relay costs one SSE client slot on every peer for as long as it is open,
 * and `sse.max_clients` defaults to 2 - so an aggregator nobody is watching
 * must not be holding those slots.
 */
class PeerFaultRelay {
 public:
  /// Returns the peers to relay from right now. Called on every reconcile, so
  /// a peer that appeared or went away is picked up without a timer of its own.
  using TargetSupplier = std::function<std::vector<RelayTarget>()>;
  /// Invoked from a proxy's reader thread for each event received.
  using EventCallback = std::function<void(const StreamEvent &)>;

  PeerFaultRelay(TargetSupplier targets, std::string path, EventCallback on_event);
  ~PeerFaultRelay();

  PeerFaultRelay(const PeerFaultRelay &) = delete;
  PeerFaultRelay & operator=(const PeerFaultRelay &) = delete;
  PeerFaultRelay(PeerFaultRelay &&) = delete;
  PeerFaultRelay & operator=(PeerFaultRelay &&) = delete;

  /// A client attached to the local stream. On the 0 -> 1 transition every
  /// peer's stream is opened.
  void acquire();

  /// A client left. On the 1 -> 0 transition every stream is closed.
  ///
  /// MUST NOT be called while holding a lock the relay's event callback also
  /// takes. Closing a stream joins its reader thread, and that thread may be
  /// inside the callback waiting for exactly that lock.
  void release();

  /// Open streams for peers that appeared, close those that went away. Does
  /// nothing while no client is attached.
  ///
  /// Called from the streaming loop, which wakes on every event, so the peer
  /// list is read at most once per kReconcileInterval however fast faults
  /// arrive. Reading it takes the aggregation manager's lock, and a burst of
  /// relayed events must not turn into a burst of lock acquisitions on the
  /// path that is delivering them.
  void reconcile();

  /// Number of peer streams currently open.
  std::size_t open_streams() const;

  /// Stop relaying and close every stream, whatever the client count. Called
  /// from the destructor and from the owner's shutdown, and is idempotent.
  void shutdown();

 private:
  /// Move every open proxy out of the map. Caller holds mutex_ and destroys
  /// the returned proxies after releasing it - destroying one joins its reader
  /// thread, which calls back into the owner and takes the owner's lock.
  std::vector<std::unique_ptr<SSEStreamProxy>> take_all_locked();

  TargetSupplier targets_;
  std::string path_;
  EventCallback on_event_;

  /// Reconcile at most this often. Peers appear and go away on the discovery
  /// cadence, so a second's delay in picking one up costs nothing.
  static constexpr std::chrono::milliseconds kReconcileInterval{1000};

  /// Open streams for the peers listed now. `force` skips the interval check;
  /// used on the first attached client, where waiting would leave the stream
  /// silent for a second with nothing to gain.
  void reconcile_now(bool force);

  mutable std::mutex mutex_;
  std::size_t clients_{0};
  std::chrono::steady_clock::time_point last_reconcile_{};
  bool shut_down_{false};
  /// Open proxies, keyed by peer name AND url. A peer keeps its name across an
  /// mDNS re-announce that moves it to a new address, so a name-only key would
  /// keep the dead connection and never open the new one.
  std::map<std::pair<std::string, std::string>, std::unique_ptr<SSEStreamProxy>> streams_;
  /// Last event id reached per peer, kept when its stream is closed so a later
  /// one resumes there instead of asking for the peer's whole buffer. Keyed
  /// like streams_, and it outlives them on purpose: the common case is the
  /// last client leaving and another arriving moments later.
  std::map<std::pair<std::string, std::string>, std::string> cursors_;
};

}  // namespace ros2_medkit_gateway
