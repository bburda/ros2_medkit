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

#include "ros2_medkit_gateway/core/managers/update_manager.hpp"

#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"

namespace ros2_medkit_gateway {

UpdateManager::UpdateManager() = default;

UpdateManager::~UpdateManager() {
  shutdown();
}

void UpdateManager::shutdown() {
  // Collect all valid futures, then wait OUTSIDE the lock to avoid
  // deadlock (async tasks also acquire mutex_ during execution).
  std::vector<std::future<void>> futures;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Signal background tasks to stop accepting new work. Setting the flag
    // under mutex_ closes the window where a start_*() call has already passed
    // its stopped_ check but has not yet launched: those functions re-check the
    // flag under this same lock, so nothing can be launched past this point and
    // escape the drain below.
    stopped_ = true;
    for (auto & [id, state] : states_) {
      if (state && state->active_task.valid()) {
        futures.push_back(std::move(state->active_task));
      }
    }
  }
  for (auto & f : futures) {
    f.wait();
  }
  // Moving the futures out above leaves every active_task invalid, so a second
  // call collects nothing and returns immediately - shutdown() is idempotent
  // without needing a separate guard flag.
}

void UpdateManager::set_backend(UpdateProvider * backend) {
  backend_ = backend;
}

void UpdateManager::set_notifier(ResourceChangeNotifier * notifier) {
  notifier_ = notifier;
}

bool UpdateManager::has_backend() const {
  return backend_ != nullptr;
}

void UpdateManager::notify_status_change(const std::string & id, const UpdateStatusInfo & status) {
  if (!notifier_) {
    return;
  }
  notifier_->notify("updates", id, id, update_status_to_json(status));
}

tl::expected<std::vector<std::string>, UpdateError> UpdateManager::list_updates(const UpdateFilter & filter) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  auto result = backend_->list_updates(filter);
  if (!result) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, result.error().message});
  }
  return *result;
}

tl::expected<dto::UpdateDetail, UpdateError> UpdateManager::get_update(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  auto result = backend_->get_update(id);
  if (!result) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, result.error().message});
  }
  return std::move(*result);
}

tl::expected<void, UpdateError> UpdateManager::register_update(const nlohmann::json & metadata) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  auto result = backend_->register_update(metadata);
  if (!result) {
    const auto & err = result.error();
    switch (err.code) {
      case UpdateBackendError::AlreadyExists:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::AlreadyExists, err.message});
      case UpdateBackendError::InvalidInput:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::InvalidRequest, err.message});
      case UpdateBackendError::NotFound:
      case UpdateBackendError::Internal:
      default:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, err.message});
    }
  }
  // Seed an initial Pending state so GET /updates/<id>/status returns
  // {"status":"pending"} immediately after register, rather than 404. UI
  // consumers (UpdatesDashboard) gate action buttons on the status field
  // and won't render "Prepare" when the status query fails, so a newly-
  // registered update would otherwise be stuck from the frontend's view.
  // Callers (update_handlers.cpp) validate id presence + non-empty string
  // before invoking register_update, so metadata.at("id") is guaranteed to
  // hold a string at this point. The !state_ptr check matches the
  // idempotent-seed pattern used in start_prepare / run_prepare.
  const auto id = metadata.at("id").get<std::string>();
  std::lock_guard<std::mutex> lock(mutex_);
  auto & state_ptr = states_[id];
  if (!state_ptr) {
    state_ptr = std::make_unique<PackageState>();
    state_ptr->status =
        UpdateStatusInfo{UpdateStatus::Pending, UpdatePhase::None, std::nullopt, std::nullopt, std::nullopt};
  }
  return {};
}

tl::expected<void, UpdateError> UpdateManager::delete_update(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }

  bool had_state = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end()) {
      had_state = true;
      if (is_task_active(id)) {
        return tl::make_unexpected(
            UpdateError{UpdateErrorCode::InProgress, "Cannot delete update while operation is in progress"});
      }
      // Wait for completed task before proceeding (clean up future)
      if (it->second->active_task.valid()) {
        it->second->active_task.wait();
      }
      // Mark as deleting so no new operations can start on this package
      it->second->status.phase = UpdatePhase::Deleting;
    } else {
      // Create sentinel to prevent concurrent start_prepare
      states_[id] = std::make_unique<PackageState>();
      states_[id]->status.phase = UpdatePhase::Deleting;
    }
  }

  auto result = backend_->delete_update(id);
  if (result) {
    std::lock_guard<std::mutex> lock(mutex_);
    states_.erase(id);
  } else {
    // Rollback sentinel on failure
    const auto & err = result.error();
    std::lock_guard<std::mutex> lock(mutex_);
    if (had_state) {
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        // Mark the package as failed consistently across all surfaces
        // exposed via GET /updates/{id}/status: the SOVD `status` field,
        // the `x-medkit.phase` vendor extension, and `error_message`.
        // Without updating all three, clients see payloads like
        // {status:pending, "x-medkit":{phase:failed}} with no error context.
        it->second->status.status = UpdateStatus::Failed;
        it->second->status.phase = UpdatePhase::Failed;
        it->second->status.error_message = err.message;
      }
    } else {
      // Remove the sentinel we created - package never had state before
      states_.erase(id);
    }
    switch (err.code) {
      case UpdateBackendError::NotFound:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, err.message});
      case UpdateBackendError::AlreadyExists:
      case UpdateBackendError::InvalidInput:
      case UpdateBackendError::Internal:
      default:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, err.message});
    }
  }
  return {};
}

tl::expected<void, UpdateError> UpdateManager::start_prepare(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Checked under mutex_ so a task can never be launched after shutdown() has
  // taken its snapshot of the in-flight futures (see UpdateManager::shutdown).
  if (stopped_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, "UpdateManager is shutting down"});
  }

  // Verify package exists while holding lock to prevent concurrent deletion
  auto pkg = backend_->get_update(id);
  if (!pkg) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, pkg.error().message});
  }

  if (is_task_active(id)) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::InProgress, "An operation is already in progress for this package"});
  }

  auto & state_ptr = states_[id];
  if (!state_ptr) {
    state_ptr = std::make_unique<PackageState>();
  }

  if (state_ptr->status.phase == UpdatePhase::Deleting) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Deleting, "Package is being deleted"});
  }

  state_ptr->status =
      UpdateStatusInfo{UpdateStatus::Pending, UpdatePhase::Preparing, std::nullopt, std::nullopt, std::nullopt};
  state_ptr->active_task = std::async(std::launch::async, &UpdateManager::run_prepare, this, id);
  return {};
}

tl::expected<void, UpdateError> UpdateManager::start_execute(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Checked under mutex_ - see the note in start_prepare().
  if (stopped_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, "UpdateManager is shutting down"});
  }

  auto pkg = backend_->get_update(id);
  if (!pkg) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, pkg.error().message});
  }

  auto it = states_.find(id);
  if (it == states_.end() || !it->second || it->second->status.phase != UpdatePhase::Prepared) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotPrepared, "Package must be prepared before execution"});
  }
  if (is_task_active(id)) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::InProgress, "An operation is already in progress for this package"});
  }

  auto & state = *it->second;
  state.status =
      UpdateStatusInfo{UpdateStatus::Pending, UpdatePhase::Executing, std::nullopt, std::nullopt, std::nullopt};
  state.active_task = std::async(std::launch::async, &UpdateManager::run_execute, this, id);
  return {};
}

tl::expected<void, UpdateError> UpdateManager::start_automated(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Checked under mutex_ - see the note in start_prepare().
  if (stopped_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, "UpdateManager is shutting down"});
  }

  auto supported = backend_->supports_automated(id);
  if (!supported) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, supported.error().message});
  }
  if (!*supported) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::NotAutomated, "Package does not support automated updates"});
  }

  if (is_task_active(id)) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::InProgress, "An operation is already in progress for this package"});
  }

  auto & state_ptr = states_[id];
  if (!state_ptr) {
    state_ptr = std::make_unique<PackageState>();
  }

  if (state_ptr->status.phase == UpdatePhase::Deleting) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Deleting, "Package is being deleted"});
  }

  state_ptr->status =
      UpdateStatusInfo{UpdateStatus::Pending, UpdatePhase::Preparing, std::nullopt, std::nullopt, std::nullopt};
  state_ptr->active_task = std::async(std::launch::async, &UpdateManager::run_automated, this, id);
  return {};
}

tl::expected<UpdateStatusInfo, UpdateError> UpdateManager::get_status(const std::string & id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(id);
  if (it == states_.end() || !it->second) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, "No status available for package '" + id + "'"});
  }
  return it->second->status;
}

bool UpdateManager::is_task_active(const std::string & id) const {
  auto it = states_.find(id);
  if (it == states_.end() || !it->second) {
    return false;
  }
  if (!it->second->active_task.valid()) {
    return false;
  }
  return it->second->active_task.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
}

void UpdateManager::run_prepare(const std::string & id) {
  try {
    PackageState * state = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = states_[id].get();
      state->status.status = UpdateStatus::InProgress;
    }
    notify_status_change(id, state->status);

    UpdateProgressReporter reporter(state->status, mutex_);
    auto result = backend_->prepare(id, reporter);

    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (result) {
        state->status.status = UpdateStatus::Completed;
        state->status.phase = UpdatePhase::Prepared;
      } else {
        state->status.status = UpdateStatus::Failed;
        state->status.error_message = result.error().message;
        state->status.phase = UpdatePhase::Failed;
      }
      snapshot = state->status;
    }
    notify_status_change(id, snapshot);
  } catch (const std::exception & e) {
    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        it->second->status.status = UpdateStatus::Failed;
        it->second->status.error_message = std::string("Exception: ") + e.what();
        it->second->status.phase = UpdatePhase::Failed;
        snapshot = it->second->status;
      }
    }
    notify_status_change(id, snapshot);
  } catch (...) {
    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        it->second->status.status = UpdateStatus::Failed;
        it->second->status.error_message = "Unknown exception during prepare";
        it->second->status.phase = UpdatePhase::Failed;
        snapshot = it->second->status;
      }
    }
    notify_status_change(id, snapshot);
  }
}

void UpdateManager::run_execute(const std::string & id) {
  try {
    PackageState * state = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = states_[id].get();
      state->status.status = UpdateStatus::InProgress;
    }
    notify_status_change(id, state->status);

    UpdateProgressReporter reporter(state->status, mutex_);
    auto result = backend_->execute(id, reporter);

    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (result) {
        state->status.status = UpdateStatus::Completed;
        state->status.phase = UpdatePhase::Executed;
      } else {
        state->status.status = UpdateStatus::Failed;
        state->status.error_message = result.error().message;
        state->status.phase = UpdatePhase::Failed;
      }
      snapshot = state->status;
    }
    notify_status_change(id, snapshot);
  } catch (const std::exception & e) {
    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        it->second->status.status = UpdateStatus::Failed;
        it->second->status.error_message = std::string("Exception: ") + e.what();
        it->second->status.phase = UpdatePhase::Failed;
        snapshot = it->second->status;
      }
    }
    notify_status_change(id, snapshot);
  } catch (...) {
    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        it->second->status.status = UpdateStatus::Failed;
        it->second->status.error_message = "Unknown exception during execute";
        it->second->status.phase = UpdatePhase::Failed;
        snapshot = it->second->status;
      }
    }
    notify_status_change(id, snapshot);
  }
}

void UpdateManager::run_automated(const std::string & id) {
  try {
    PackageState * state = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = states_[id].get();
      state->status.status = UpdateStatus::InProgress;
    }
    notify_status_change(id, state->status);

    // Phase 1: Prepare
    UpdateProgressReporter reporter(state->status, mutex_);
    auto prep_result = backend_->prepare(id, reporter);

    if (!prep_result) {
      UpdateStatusInfo snapshot;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        state->status.status = UpdateStatus::Failed;
        state->status.error_message = prep_result.error().message;
        state->status.phase = UpdatePhase::Failed;
        snapshot = state->status;
      }
      notify_status_change(id, snapshot);
      return;
    }

    // Phase 2: Execute (reset progress for execute phase)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state->status.phase = UpdatePhase::Executing;
      state->status.progress = std::nullopt;
      state->status.sub_progress = std::nullopt;
    }

    auto exec_result = backend_->execute(id, reporter);

    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (exec_result) {
        state->status.status = UpdateStatus::Completed;
        state->status.phase = UpdatePhase::Executed;
      } else {
        state->status.status = UpdateStatus::Failed;
        state->status.error_message = exec_result.error().message;
        state->status.phase = UpdatePhase::Failed;
      }
      snapshot = state->status;
    }
    notify_status_change(id, snapshot);
  } catch (const std::exception & e) {
    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        it->second->status.status = UpdateStatus::Failed;
        it->second->status.error_message = std::string("Exception: ") + e.what();
        it->second->status.phase = UpdatePhase::Failed;
        snapshot = it->second->status;
      }
    }
    notify_status_change(id, snapshot);
  } catch (...) {
    UpdateStatusInfo snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        it->second->status.status = UpdateStatus::Failed;
        it->second->status.error_message = "Unknown exception during automated update";
        it->second->status.phase = UpdatePhase::Failed;
        snapshot = it->second->status;
      }
    }
    notify_status_change(id, snapshot);
  }
}

}  // namespace ros2_medkit_gateway
