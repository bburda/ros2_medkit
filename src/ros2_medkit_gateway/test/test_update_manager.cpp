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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/core/managers/update_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

/// Mock backend for unit testing
class MockUpdateBackend : public UpdateProvider {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> ids;
    ids.reserve(packages_.size());
    for (const auto & [id, _] : packages_) {
      ids.push_back(id);
    }
    return ids;
  }

  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found"});
    }
    return dto::UpdateDetail{it->second};
  }

  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & metadata) override {
    auto id = metadata.value("id", std::string{});
    if (id.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing id"});
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (packages_.count(id)) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::AlreadyExists, "already exists"});
    }
    packages_[id] = metadata;
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (packages_.erase(id) == 0) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found"});
    }
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & reporter) override {
    reporter.set_progress(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reporter.set_progress(100);
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & reporter) override {
    reporter.set_progress(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return {};
  }

  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found"});
    }
    return it->second.value("automated", false);
  }

 private:
  std::mutex mutex_;
  std::unordered_map<std::string, json> packages_;
};

class UpdateManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    backend_ = std::make_unique<MockUpdateBackend>();
    backend_ptr_ = backend_.get();
    manager_ = std::make_unique<UpdateManager>();
    manager_->set_backend(backend_ptr_);
  }

  void TearDown() override {
    manager_.reset();
    backend_.reset();
  }

  std::unique_ptr<MockUpdateBackend> backend_;
  MockUpdateBackend * backend_ptr_ = nullptr;
  std::unique_ptr<UpdateManager> manager_;
};

// @verifies REQ_INTEROP_082
TEST_F(UpdateManagerTest, HasBackend) {
  EXPECT_TRUE(manager_->has_backend());
}

// @verifies REQ_INTEROP_082
TEST_F(UpdateManagerTest, NoBackendMode) {
  UpdateManager no_backend;
  EXPECT_FALSE(no_backend.has_backend());
  auto result = no_backend.list_updates({});
  EXPECT_FALSE(result.has_value());
}

// @verifies REQ_INTEROP_082
TEST_F(UpdateManagerTest, RegisterAndList) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  auto reg = manager_->register_update(pkg);
  EXPECT_TRUE(reg.has_value());

  auto list = manager_->list_updates({});
  ASSERT_TRUE(list.has_value());
  EXPECT_EQ(list->size(), 1u);
  EXPECT_EQ((*list)[0], "test-pkg");
}

// @verifies REQ_INTEROP_085
TEST_F(UpdateManagerTest, GetUpdate) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto result = manager_->get_update("test-pkg");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->content["id"], "test-pkg");
  // Wire-byte round-trip: JsonWriter<UpdateDetail> must reproduce the bare
  // metadata object the plugin stored, so vendor extension keys survive the
  // typed envelope.
  EXPECT_EQ(dto::JsonWriter<dto::UpdateDetail>::write(*result), pkg);
}

// @verifies REQ_INTEROP_085
TEST_F(UpdateManagerTest, GetUpdateNotFound) {
  auto result = manager_->get_update("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, UpdateErrorCode::NotFound);
}

// @verifies REQ_INTEROP_084
TEST_F(UpdateManagerTest, DeleteUpdate) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto del = manager_->delete_update("test-pkg");
  EXPECT_TRUE(del.has_value());

  auto get = manager_->get_update("test-pkg");
  EXPECT_FALSE(get.has_value());
}

// @verifies REQ_INTEROP_091
TEST_F(UpdateManagerTest, PrepareAndPollStatus) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto prep = manager_->start_prepare("test-pkg");
  ASSERT_TRUE(prep.has_value());

  // Poll until completed
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager_->get_status("test-pkg");
    if (s) {
      status = *s;
      if (status.status == UpdateStatus::Completed) {
        found = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for status Completed";
}

// @verifies REQ_INTEROP_092
TEST_F(UpdateManagerTest, ExecuteRequiresPrepare) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto exec = manager_->start_execute("test-pkg");
  EXPECT_FALSE(exec.has_value());
  EXPECT_EQ(exec.error().code, UpdateErrorCode::NotPrepared);
}

// @verifies REQ_INTEROP_092
TEST_F(UpdateManagerTest, ExecuteAfterPrepare) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  (void)manager_->start_prepare("test-pkg");
  // Wait for prepare to complete
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool found = false;
    while (std::chrono::steady_clock::now() < deadline) {
      auto s = manager_->get_status("test-pkg");
      if (s && s->status == UpdateStatus::Completed) {
        found = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(found) << "Timed out waiting for prepare to complete";
  }

  auto exec = manager_->start_execute("test-pkg");
  ASSERT_TRUE(exec.has_value());

  // Wait for execute to complete
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    UpdateStatusInfo status;
    bool found = false;
    while (std::chrono::steady_clock::now() < deadline) {
      auto s = manager_->get_status("test-pkg");
      ASSERT_TRUE(s.has_value());
      status = *s;
      if (status.status == UpdateStatus::Completed) {
        found = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(found) << "Timed out waiting for execute to complete";
    EXPECT_EQ(status.status, UpdateStatus::Completed);
  }
}

// @verifies REQ_INTEROP_093
TEST_F(UpdateManagerTest, AutomatedCompletes) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", true}};
  (void)manager_->register_update(pkg);

  auto result = manager_->start_automated("test-pkg");
  ASSERT_TRUE(result.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager_->get_status("test-pkg");
    ASSERT_TRUE(s.has_value());
    status = *s;
    if (status.status == UpdateStatus::Completed) {
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for automated update to complete";
  EXPECT_EQ(status.status, UpdateStatus::Completed);
}

// @verifies REQ_INTEROP_093
TEST_F(UpdateManagerTest, AutomatedRejectsNonAutomated) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto result = manager_->start_automated("test-pkg");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, UpdateErrorCode::NotAutomated);
}

// @verifies REQ_INTEROP_084
TEST_F(UpdateManagerTest, DeleteDuringOperationFails) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  (void)manager_->start_prepare("test-pkg");

  auto del = manager_->delete_update("test-pkg");
  EXPECT_FALSE(del.has_value());
  EXPECT_EQ(del.error().code, UpdateErrorCode::InProgress);
}

// @verifies REQ_INTEROP_094
TEST_F(UpdateManagerTest, StatusNotFoundForUnknown) {
  auto result = manager_->get_status("unknown");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, UpdateErrorCode::NotFound);
}

// @verifies REQ_INTEROP_094
TEST_F(UpdateManagerTest, StatusPendingRightAfterRegister) {
  // Registering an update must immediately yield a Pending status so the
  // UpdatesDashboard (which gates action buttons on a non-null status
  // response) can render Prepare / Execute / Delete without waiting for a
  // separate prepare call.
  json pkg = {{"id", "fresh-pkg"}, {"update_name", "Fresh"}, {"automated", false}};
  ASSERT_TRUE(manager_->register_update(pkg).has_value());

  auto status = manager_->get_status("fresh-pkg");
  ASSERT_TRUE(status.has_value()) << status.error().message;
  EXPECT_EQ(status->status, UpdateStatus::Pending);
  EXPECT_FALSE(status->progress.has_value());
  EXPECT_FALSE(status->error_message.has_value());
}

// @verifies REQ_INTEROP_083
TEST_F(UpdateManagerTest, DuplicateRegistration) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  auto first = manager_->register_update(pkg);
  EXPECT_TRUE(first.has_value());

  auto second = manager_->register_update(pkg);
  EXPECT_FALSE(second.has_value());
  EXPECT_EQ(second.error().code, UpdateErrorCode::AlreadyExists);
}

// @verifies REQ_INTEROP_091
TEST_F(UpdateManagerTest, ConcurrentPrepareOnSamePackageRejected) {
  json pkg = {{"id", "test-pkg"}};
  (void)manager_->register_update(pkg);

  auto result1 = manager_->start_prepare("test-pkg");
  EXPECT_TRUE(result1.has_value());

  auto result2 = manager_->start_prepare("test-pkg");
  EXPECT_FALSE(result2.has_value());
  EXPECT_EQ(result2.error().code, UpdateErrorCode::InProgress);
}

/// Mock backend that returns errors from prepare/execute
class MockFailingBackend : public UpdateProvider {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "backend error"});
  }
  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return dto::UpdateDetail{json{{"id", "pkg"}}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & metadata) override {
    auto id = metadata.value("id", std::string{});
    if (id.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing id"});
    }
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "download failed"});
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "install failed"});
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

/// Mock backend that throws exceptions from prepare/execute
class MockThrowingBackend : public UpdateProvider {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return dto::UpdateDetail{json{{"id", "pkg"}}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    throw std::runtime_error("plugin crashed");
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    throw std::runtime_error("plugin crashed");
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

// @verifies REQ_INTEROP_091
TEST(UpdateManagerFailureTest, PrepareFailureSetsFailedStatus) {
  auto backend = std::make_unique<MockFailingBackend>();
  auto manager = std::make_unique<UpdateManager>();
  manager->set_backend(backend.get());
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  auto result = manager->start_prepare("test-pkg");
  ASSERT_TRUE(result.has_value());

  // Poll until status is no longer InProgress
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("download failed"), std::string::npos);
  manager.reset();
  backend.reset();
}

/// Mock backend with working prepare but failing execute
class MockExecuteFailingBackend : public UpdateProvider {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return dto::UpdateDetail{json{{"id", "pkg"}}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & reporter) override {
    reporter.set_progress(100);
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "install failed"});
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

/// Mock backend with working prepare but throwing execute
class MockExecuteThrowingBackend : public UpdateProvider {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return dto::UpdateDetail{json{{"id", "pkg"}}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & reporter) override {
    reporter.set_progress(100);
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    throw std::runtime_error("plugin crashed during install");
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

// @verifies REQ_INTEROP_091
TEST(UpdateManagerFailureTest, PrepareExceptionSetsFailedStatus) {
  auto backend = std::make_unique<MockThrowingBackend>();
  auto manager = std::make_unique<UpdateManager>();
  manager->set_backend(backend.get());
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  auto result = manager->start_prepare("test-pkg");
  ASSERT_TRUE(result.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("Exception"), std::string::npos);
  manager.reset();
  backend.reset();
}

// Helper: prepare a package and wait for completion
static bool prepare_and_wait(UpdateManager & manager, const std::string & id) {
  auto prep = manager.start_prepare(id);
  if (!prep) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager.get_status(id);
    if (s && s->status == UpdateStatus::Completed) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// @verifies REQ_INTEROP_092
TEST(UpdateManagerFailureTest, ExecuteFailureSetsFailedStatus) {
  auto backend = std::make_unique<MockExecuteFailingBackend>();
  auto manager = std::make_unique<UpdateManager>();
  manager->set_backend(backend.get());
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  ASSERT_TRUE(prepare_and_wait(*manager, "test-pkg")) << "Timed out waiting for prepare to complete";

  auto exec = manager->start_execute("test-pkg");
  ASSERT_TRUE(exec.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for execute to fail";
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("install failed"), std::string::npos);
  manager.reset();
  backend.reset();
}

// @verifies REQ_INTEROP_092
TEST(UpdateManagerFailureTest, ExecuteExceptionSetsFailedStatus) {
  auto backend = std::make_unique<MockExecuteThrowingBackend>();
  auto manager = std::make_unique<UpdateManager>();
  manager->set_backend(backend.get());
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  ASSERT_TRUE(prepare_and_wait(*manager, "test-pkg")) << "Timed out waiting for prepare to complete";

  auto exec = manager->start_execute("test-pkg");
  ASSERT_TRUE(exec.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for execute exception to be caught";
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("Exception"), std::string::npos);
  manager.reset();
  backend.reset();
}

/// Mock backend whose register succeeds but delete_update fails. Lets us
/// verify the rollback path updates status, phase, and error_message so the
/// /updates/{id}/status payload stays internally consistent.
class MockDeleteFailingBackend : public UpdateProvider {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<dto::UpdateDetail, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return dto::UpdateDetail{json{{"id", "pkg"}}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "backend delete exploded"});
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return {};
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

TEST(UpdateManagerFailureTest, DeleteRollbackUpdatesStatusPhaseAndErrorMessage) {
  // When delete_update fails for a known package, the rollback must leave
  // status=Failed, phase=Failed, and error_message populated so the status
  // endpoint does not emit an inconsistent payload like
  // {status:pending, "x-medkit":{phase:failed}} without any error details.
  auto backend = std::make_unique<MockDeleteFailingBackend>();
  auto manager = std::make_unique<UpdateManager>();
  manager->set_backend(backend.get());

  json pkg = {{"id", "test-pkg"}};
  ASSERT_TRUE(manager->register_update(pkg).has_value());

  auto del = manager->delete_update("test-pkg");
  ASSERT_FALSE(del.has_value());

  auto status = manager->get_status("test-pkg");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->status, UpdateStatus::Failed);
  EXPECT_EQ(status->phase, UpdatePhase::Failed);
  ASSERT_TRUE(status->error_message.has_value());
  EXPECT_NE(status->error_message->find("backend delete exploded"), std::string::npos);

  // Serialized payload reflects the rollback end-to-end. Use at() so a
  // missing x-medkit/phase fails the test deterministically instead of being
  // inserted as null by operator[].
  auto j = update_status_to_json(*status);
  EXPECT_EQ(j.at("status"), "failed");
  ASSERT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j.at("x-medkit").at("phase"), "failed");
  EXPECT_EQ(j.at("error"), "backend delete exploded");

  manager.reset();
  backend.reset();
}

TEST(UpdateStatusToJson, SerializesPhaseAsVendorExtension) {
  UpdateStatusInfo status{UpdateStatus::Completed, UpdatePhase::Prepared, std::nullopt, std::nullopt, std::nullopt};
  auto j = update_status_to_json(status);
  EXPECT_EQ(j.at("status"), "completed");
  ASSERT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j.at("x-medkit").at("phase"), "prepared");
}

TEST(UpdateStatusToJson, ExposesExecutedPhaseForTerminalCompletion) {
  UpdateStatusInfo status{UpdateStatus::Completed, UpdatePhase::Executed, 100, std::nullopt, std::nullopt};
  auto j = update_status_to_json(status);
  EXPECT_EQ(j.at("status"), "completed");
  ASSERT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j.at("x-medkit").at("phase"), "executed");
  EXPECT_EQ(j.at("progress"), 100);
}

TEST(UpdateStatusToJson, EmitsNonePhaseForFreshlyRegistered) {
  UpdateStatusInfo status;
  auto j = update_status_to_json(status);
  EXPECT_EQ(j.at("status"), "pending");
  ASSERT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j.at("x-medkit").at("phase"), "none");
}

/// Backend that parks inside prepare() until the test releases it, so a task
/// can be held demonstrably in flight across a shutdown() call.
class GatedUpdateBackend : public MockUpdateBackend {
 public:
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    entered_.set_value();
    released_.get_future().wait();
    return {};
  }

  /// Blocks until prepare() is actually running on the async task thread.
  void wait_until_in_prepare() {
    entered_.get_future().wait();
  }

  /// Lets prepare() return.
  void release() {
    released_.set_value();
  }

 private:
  std::promise<void> entered_;
  std::promise<void> released_;
};

// Update tasks run on their own std::async threads and call
// ResourceChangeNotifier::notify() after the backend returns, so they can
// outlive the notifier. shutdown() is the drain owners rely on: GatewayNode
// calls it before the notifier is shut down and freed. If shutdown() stops
// waiting for in-flight tasks, the notify below lands on a destroyed notifier
// - which is exactly the heap-use-after-free ASan reported from run_prepare().
TEST(UpdateManagerLifetime, ShutdownDrainsInFlightTasksBeforeNotifierIsDestroyed) {
  GatedUpdateBackend backend;
  auto notifier = std::make_unique<ResourceChangeNotifier>();

  std::atomic<int> terminal_notifications{0};
  notifier->subscribe(NotifierFilter{"updates", "", ""}, [&terminal_notifications](const ResourceChange & change) {
    if (change.value.value("status", std::string{}) == "completed") {
      terminal_notifications.fetch_add(1);
    }
  });

  UpdateManager manager;
  manager.set_backend(&backend);
  manager.set_notifier(notifier.get());

  ASSERT_TRUE(manager.register_update(json{{"id", "gated-pkg"}, {"update_name", "Gated"}}).has_value());
  ASSERT_TRUE(manager.start_prepare("gated-pkg").has_value());
  backend.wait_until_in_prepare();

  // The task is now parked inside prepare(), so it has NOT yet notified.
  // shutdown() must block until it has. Releasing from another thread after a
  // delay makes that the measured quantity: if shutdown() fails to drain it
  // returns while the task is still parked, and the check below fires. Without
  // the delay the task wins the race on its own and the test proves nothing.
  std::atomic<bool> shutdown_returned{false};
  std::thread releaser([&backend, &shutdown_returned]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_FALSE(shutdown_returned.load()) << "shutdown() returned while a task was still inside prepare()";
    backend.release();
  });

  manager.shutdown();
  shutdown_returned.store(true);
  releaser.join();

  // Draining the notifier delivers everything notify() enqueued. worker_loop()
  // only exits once the queue is empty, so after this the count is final.
  notifier->shutdown();
  EXPECT_EQ(terminal_notifications.load(), 1) << "shutdown() returned before the in-flight task notified";

  // With the task drained, destroying the notifier while the manager is still
  // alive is safe - the ordering GatewayNode relies on.
  notifier.reset();
}

// shutdown() must be safe to call more than once: GatewayNode calls it during
// teardown and ~UpdateManager calls it again as the standalone backstop.
TEST(UpdateManagerLifetime, ShutdownIsIdempotentAndRejectsLaterStarts) {
  MockUpdateBackend backend;
  UpdateManager manager;
  manager.set_backend(&backend);

  ASSERT_TRUE(manager.register_update(json{{"id", "pkg"}, {"update_name", "Test"}}).has_value());

  manager.shutdown();
  manager.shutdown();

  auto prep = manager.start_prepare("pkg");
  ASSERT_FALSE(prep.has_value());
  EXPECT_EQ(prep.error().code, UpdateErrorCode::Internal);
}
