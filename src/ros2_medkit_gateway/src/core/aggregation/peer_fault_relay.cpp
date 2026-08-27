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

#include "ros2_medkit_gateway/core/aggregation/peer_fault_relay.hpp"

#include <chrono>
#include <utility>

namespace ros2_medkit_gateway {

PeerFaultRelay::PeerFaultRelay(TargetSupplier targets, std::string path, EventCallback on_event)
  : targets_(std::move(targets)), path_(std::move(path)), on_event_(std::move(on_event)) {
}

PeerFaultRelay::~PeerFaultRelay() {
  shutdown();
}

void PeerFaultRelay::acquire() {
  bool opened_first = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shut_down_) {
      return;
    }
    opened_first = (clients_++ == 0);
  }
  if (opened_first) {
    reconcile_now(/*force=*/true);
  }
}

void PeerFaultRelay::release() {
  std::vector<std::pair<StreamKey, std::unique_ptr<SSEStreamProxy>>> retired;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (clients_ == 0) {
      return;
    }
    if (--clients_ == 0) {
      retired = take_all_locked();
    }
  }
  // Outside the lock on purpose: closing joins a reader thread, and a reader
  // thread delivering an event is inside the owner's callback.
  retire(std::move(retired));
}

void PeerFaultRelay::reconcile() {
  reconcile_now(/*force=*/false);
}

void PeerFaultRelay::reconcile_now(bool force) {
  // The target list is read outside the lock: it reaches into the aggregation
  // manager, which takes a lock of its own, and holding both in this order
  // would pin the two together for every wakeup of every streaming client.
  std::vector<RelayTarget> targets;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shut_down_ || clients_ == 0) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - last_reconcile_ < kReconcileInterval) {
      return;
    }
    last_reconcile_ = now;
  }
  targets = targets_ ? targets_() : std::vector<RelayTarget>{};

  // Closing a proxy joins its reader thread, so the ones being retired are
  // moved out under the lock and destroyed after it. A reader thread delivering
  // an event calls back into the owner, which locks its own queue; joining that
  // thread while holding this mutex puts the two lock orders against each other.
  std::vector<std::pair<StreamKey, std::unique_ptr<SSEStreamProxy>>> retired;
  std::vector<RelayTarget> to_open;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shut_down_ || clients_ == 0) {
      return;
    }
    for (auto it = streams_.begin(); it != streams_.end();) {
      const bool still_listed = std::any_of(targets.begin(), targets.end(), [&it](const RelayTarget & target) {
        return target.name == it->first.first && target.url == it->first.second;
      });
      if (still_listed) {
        ++it;
        continue;
      }
      retired.emplace_back(it->first, std::move(it->second));
      it = streams_.erase(it);
    }
    for (const auto & target : targets) {
      if (streams_.find(std::make_pair(target.name, target.url)) == streams_.end()) {
        to_open.push_back(target);
      }
    }
  }
  retire(std::move(retired));

  for (const auto & target : to_open) {
    // X-Medkit-No-Fan-Out is what the collection routes already use to stop a
    // request bouncing round a bidirectional or chained peering. A relayed
    // stream needs it for the same reason: without it, a peer that aggregates
    // in turn would relay this gateway's own relayed events back.
    httplib::Headers headers{{"X-Medkit-No-Fan-Out", "1"}};
    if (!target.auth_header.empty()) {
      headers.emplace("Authorization", target.auth_header);
    }
    auto proxy = std::make_unique<SSEStreamProxy>(target.url, path_, target.name, headers);
    proxy->on_event(on_event_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto cursor = cursors_.find(std::make_pair(target.name, target.url));
      if (cursor != cursors_.end()) {
        proxy->set_last_event_id(cursor->second);
      }
    }
    proxy->open();
    std::lock_guard<std::mutex> lock(mutex_);
    if (shut_down_ || clients_ == 0) {
      // The last client left while the stream was being opened. Unlock-free
      // teardown is not available here, but the proxy is not published, so
      // closing it after the lock is released is enough - it is the only
      // reference.
      break;
    }
    streams_.emplace(std::make_pair(target.name, target.url), std::move(proxy));
  }
}

std::size_t PeerFaultRelay::open_streams() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return streams_.size();
}

void PeerFaultRelay::shutdown() {
  std::vector<std::pair<StreamKey, std::unique_ptr<SSEStreamProxy>>> retired;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shut_down_) {
      return;
    }
    shut_down_ = true;
    clients_ = 0;
    retired = take_all_locked();
  }
  retire(std::move(retired));
}

std::vector<std::pair<PeerFaultRelay::StreamKey, std::unique_ptr<SSEStreamProxy>>> PeerFaultRelay::take_all_locked() {
  std::vector<std::pair<StreamKey, std::unique_ptr<SSEStreamProxy>>> retired;
  retired.reserve(streams_.size());
  for (auto & [key, proxy] : streams_) {
    // The key travels with the proxy so retire() can record where its reader
    // stopped. The cursor is NOT read here: this runs with the reader still
    // going, so a value taken now can be one event behind by the time the
    // reader is stopped, and that event would then be replayed.
    retired.emplace_back(key, std::move(proxy));
  }
  streams_.clear();
  return retired;
}

void PeerFaultRelay::retire(std::vector<std::pair<StreamKey, std::unique_ptr<SSEStreamProxy>>> retired) {
  for (auto & [key, proxy] : retired) {
    if (!proxy) {
      continue;
    }
    // Stopped BEFORE its cursor is read, so the value recorded is the last
    // event the reader actually delivered and not a snapshot it has since
    // moved past.
    proxy->close();
    auto cursor = proxy->last_event_id();
    proxy.reset();
    if (cursor.empty()) {
      continue;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    cursors_[key] = std::move(cursor);
    trim_cursors_locked();
  }
}

void PeerFaultRelay::trim_cursors_locked() {
  while (cursors_.size() > kMaxCursors) {
    // Prefer forgetting a peer that has no stream open right now: it is the
    // one least likely to be resumed, and a peer currently being relayed from
    // would otherwise lose its resume point while it was still in use.
    auto victim = std::find_if(cursors_.begin(), cursors_.end(), [this](const auto & entry) {
      return streams_.find(entry.first) == streams_.end();
    });
    cursors_.erase(victim == cursors_.end() ? cursors_.begin() : victim);
  }
}

}  // namespace ros2_medkit_gateway
