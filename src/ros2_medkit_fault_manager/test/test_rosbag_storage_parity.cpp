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

/// Evidence retention parity between the two storage backends.
///
/// A fault holding several recordings was enforced by the SQLite schema before, and
/// separately by a std::map in the in-memory backend. Two enforcement points means two
/// chances to diverge, and `storage_type: memory` silently keeping the old behaviour
/// would be invisible to a suite that only exercises SQLite. Every assertion here
/// therefore runs against both backends from one body.
///
/// Snapshots have the same shape and were left out of the first version of this file,
/// which is how two backends ended up disagreeing about whether a capture larger than
/// the cap survives at all, and about which capture `get_snapshots` returns first.
/// Both suites live here so a new evidence table cannot be added to one backend only.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace {

using ros2_medkit_fault_manager::FaultStorage;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::rosbag_recording_id;
using ros2_medkit_fault_manager::RosbagFileInfo;
using ros2_medkit_fault_manager::SnapshotData;
using ros2_medkit_fault_manager::SqliteFaultStorage;

/// Backends differ only in how they are constructed, so the fixture reaches them
/// through this rather than through #ifdef-style branching inside the test bodies.
template <typename Backend>
struct BackendFactory;

template <>
struct BackendFactory<InMemoryFaultStorage> {
  static std::unique_ptr<FaultStorage> make(const std::filesystem::path & /*dir*/) {
    return std::make_unique<InMemoryFaultStorage>();
  }
};

template <>
struct BackendFactory<SqliteFaultStorage> {
  static std::unique_ptr<FaultStorage> make(const std::filesystem::path & dir) {
    return std::make_unique<SqliteFaultStorage>((dir / "parity.db").string());
  }
};

template <typename Backend>
class RosbagRetentionParityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ =
        std::filesystem::temp_directory_path() / ("medkit_parity_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    storage_ = BackendFactory<Backend>::make(dir_);
  }

  void TearDown() override {
    storage_.reset();
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  /// Create a real bag directory so unlink behaviour is observable, not just row state.
  /// Bags are directories on disk, which is what both backends remove.
  std::string make_bag(const std::string & name) {
    const auto path = dir_ / name;
    std::filesystem::create_directories(path);
    std::ofstream(path / "metadata.yaml") << "version: 9\n";
    return path.string();
  }

  RosbagFileInfo row(const std::string & code, const std::string & bag_name, int64_t created_ns) {
    RosbagFileInfo info;
    info.fault_code = code;
    info.file_path = make_bag(bag_name);
    info.recording_id = rosbag_recording_id(info.file_path);
    info.format = "mcap";
    info.duration_sec = 5.0;
    info.size_bytes = 1024;
    info.created_at_ns = created_ns;
    return info;
  }

  std::filesystem::path dir_;
  std::unique_ptr<FaultStorage> storage_;
};

using Backends = ::testing::Types<InMemoryFaultStorage, SqliteFaultStorage>;
TYPED_TEST_SUITE(RosbagRetentionParityTest, Backends);

TYPED_TEST(RosbagRetentionParityTest, RecordingIdIsTheBasenameOfEveryStoredRow) {
  // The gateway addresses a recording by this string and the fault_manager resolves it
  // back to rows, so a backend that stored anything else would break the URL silently.
  this->storage_->set_max_rosbags_per_fault(0);
  this->storage_->store_rosbag_file(this->row("BASENAME", "fault_BASENAME_1", 1000));
  this->storage_->store_rosbag_file(this->row("BASENAME", "fault_BASENAME_2", 2000));

  const auto rows = this->storage_->get_rosbag_files("BASENAME");
  ASSERT_EQ(rows.size(), 2u);
  for (const auto & r : rows) {
    EXPECT_EQ(r.recording_id, std::filesystem::path(r.file_path).filename().string());
  }
}

TYPED_TEST(RosbagRetentionParityTest, RecordingIdIsDerivedWhenTheCallerLeftItEmpty) {
  // rosbag_capture fills it, but the interface is public and embedders re-store rows.
  // A row with no recording_id is unaddressable, so neither backend may store one.
  this->storage_->set_max_rosbags_per_fault(0);
  auto info = this->row("DERIVED", "fault_DERIVED_1", 1000);
  info.recording_id.clear();
  this->storage_->store_rosbag_file(info);

  const auto rows = this->storage_->get_rosbag_files("DERIVED");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].recording_id, "fault_DERIVED_1");
}

TYPED_TEST(RosbagRetentionParityTest, SeveralRecordingsForOneFaultSurviveUnderACap) {
  // The feature itself: this is exactly what the schema made impossible before.
  this->storage_->set_max_rosbags_per_fault(3);
  this->storage_->store_rosbag_file(this->row("FLAP", "fault_FLAP_1", 1000));
  this->storage_->store_rosbag_file(this->row("FLAP", "fault_FLAP_2", 2000));
  this->storage_->store_rosbag_file(this->row("FLAP", "fault_FLAP_3", 3000));

  const auto rows = this->storage_->get_rosbag_files("FLAP");
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_EQ(rows[0].recording_id, "fault_FLAP_3") << "newest first";
  EXPECT_EQ(rows[2].recording_id, "fault_FLAP_1");
  for (const auto & r : rows) {
    EXPECT_TRUE(std::filesystem::exists(r.file_path));
  }
}

TYPED_TEST(RosbagRetentionParityTest, CapKeepsTheNewestAndUnlinksTheEvicted) {
  this->storage_->set_max_rosbags_per_fault(2);
  const auto oldest = this->row("CAP", "fault_CAP_1", 1000);
  this->storage_->store_rosbag_file(oldest);
  this->storage_->store_rosbag_file(this->row("CAP", "fault_CAP_2", 2000));
  this->storage_->store_rosbag_file(this->row("CAP", "fault_CAP_3", 3000));

  const auto rows = this->storage_->get_rosbag_files("CAP");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].recording_id, "fault_CAP_3");
  EXPECT_EQ(rows[1].recording_id, "fault_CAP_2");
  EXPECT_FALSE(std::filesystem::exists(oldest.file_path)) << "the evicted bag must not outlive its last row";
}

TYPED_TEST(RosbagRetentionParityTest, ACapOfOneReproducesThePreviousBehaviourExactly) {
  // The shipped default. Landing at parity is what makes any regression report about
  // the plumbing rather than about the retention policy.
  this->storage_->set_max_rosbags_per_fault(1);
  const auto first = this->row("ONE", "fault_ONE_1", 1000);
  this->storage_->store_rosbag_file(first);
  this->storage_->store_rosbag_file(this->row("ONE", "fault_ONE_2", 2000));

  const auto rows = this->storage_->get_rosbag_files("ONE");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].recording_id, "fault_ONE_2");
  EXPECT_FALSE(std::filesystem::exists(first.file_path));

  const auto newest = this->storage_->get_rosbag_file("ONE");
  ASSERT_TRUE(newest.has_value());
  EXPECT_EQ(newest->recording_id, "fault_ONE_2");
}

TYPED_TEST(RosbagRetentionParityTest, ACapOfOneIsTheDefaultWithoutConfiguringAnything) {
  // A backend constructed directly - tests, embedders - must not silently start
  // accumulating recordings just because the cap became configurable.
  const auto first = this->row("DEFAULT", "fault_DEFAULT_1", 1000);
  this->storage_->store_rosbag_file(first);
  this->storage_->store_rosbag_file(this->row("DEFAULT", "fault_DEFAULT_2", 2000));

  EXPECT_EQ(this->storage_->get_rosbag_files("DEFAULT").size(), 1u);
  EXPECT_FALSE(std::filesystem::exists(first.file_path));
}

TYPED_TEST(RosbagRetentionParityTest, ZeroMeansUnlimited) {
  this->storage_->set_max_rosbags_per_fault(0);
  for (int i = 1; i <= 6; ++i) {
    this->storage_->store_rosbag_file(this->row("UNBOUNDED", "fault_UNBOUNDED_" + std::to_string(i), i * 1000));
  }
  EXPECT_EQ(this->storage_->get_rosbag_files("UNBOUNDED").size(), 6u);
}

TYPED_TEST(RosbagRetentionParityTest, ReStoringTheSamePathIsAnUpsertNotASecondRecording) {
  // Uniqueness is on (fault_code, file_path). Re-storing the same bag - a restart
  // replaying its rows, say - must update in place, not consume a cap slot.
  this->storage_->set_max_rosbags_per_fault(3);
  auto info = this->row("UPSERT", "fault_UPSERT_1", 1000);
  this->storage_->store_rosbag_file(info);
  info.size_bytes = 4096;
  info.duration_sec = 9.0;
  this->storage_->store_rosbag_file(info);

  const auto rows = this->storage_->get_rosbag_files("UPSERT");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].size_bytes, 4096u);
  EXPECT_DOUBLE_EQ(rows[0].duration_sec, 9.0);
  EXPECT_TRUE(std::filesystem::exists(rows[0].file_path)) << "an upsert must not unlink the bag it just updated";
}

TYPED_TEST(RosbagRetentionParityTest, RefreshingALinkDoesNotMoveItWithinATieGroup) {
  // The tiebreak is positional - SQLite's id, the in-memory sequence number - so a
  // refresh must not renumber the row. SQLite's INSERT OR REPLACE would: it deletes
  // and re-inserts, handing the row a fresh id and silently promoting it past rows
  // it was stored before.
  this->storage_->set_max_rosbags_per_fault(0);
  auto first = this->row("REFRESH", "fault_REFRESH_1", 5000);
  this->storage_->store_rosbag_file(first);
  this->storage_->store_rosbag_file(this->row("REFRESH", "fault_REFRESH_2", 5000));

  first.size_bytes = 8192;
  this->storage_->store_rosbag_file(first);

  const auto rows = this->storage_->get_rosbag_files("REFRESH");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].recording_id, "fault_REFRESH_2") << "the refresh must not reorder the tie group";
  EXPECT_EQ(rows[1].recording_id, "fault_REFRESH_1");
  EXPECT_EQ(rows[1].size_bytes, 8192u) << "but it must still update the row";
}

TYPED_TEST(RosbagRetentionParityTest, NewestFirstIsStableWhenAWholeBurstSharesATimestamp) {
  // Every row of one burst carries the same created_at_ns, so ties are guaranteed
  // rather than exotic. SQLite breaks them by id; the in-memory backend needs its own
  // sequence counter to agree. Without it the two orders diverge only sometimes,
  // which is a flaky test rather than an honest failure.
  this->storage_->set_max_rosbags_per_fault(0);
  for (int i = 1; i <= 4; ++i) {
    this->storage_->store_rosbag_file(this->row("TIE", "fault_TIE_" + std::to_string(i), 7000));
  }

  const auto rows = this->storage_->get_rosbag_files("TIE");
  ASSERT_EQ(rows.size(), 4u);
  EXPECT_EQ(rows[0].recording_id, "fault_TIE_4") << "insertion order breaks the tie, newest first";
  EXPECT_EQ(rows[1].recording_id, "fault_TIE_3");
  EXPECT_EQ(rows[2].recording_id, "fault_TIE_2");
  EXPECT_EQ(rows[3].recording_id, "fault_TIE_1");
}

TYPED_TEST(RosbagRetentionParityTest, ACapTieIsBrokenByInsertionOrderNotArbitrarily) {
  // The same tie, now deciding which bag gets deleted. Whichever row wins, both
  // backends must agree, and the survivor's bag must be the one still on disk.
  this->storage_->set_max_rosbags_per_fault(1);
  const auto first = this->row("TIECAP", "fault_TIECAP_1", 7000);
  this->storage_->store_rosbag_file(first);
  const auto second = this->row("TIECAP", "fault_TIECAP_2", 7000);
  this->storage_->store_rosbag_file(second);

  const auto rows = this->storage_->get_rosbag_files("TIECAP");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].recording_id, "fault_TIECAP_2");
  EXPECT_TRUE(std::filesystem::exists(second.file_path));
  EXPECT_FALSE(std::filesystem::exists(first.file_path));
}

TYPED_TEST(RosbagRetentionParityTest, ABurstRecordingSurvivesWhileASiblingRowReferencesIt) {
  // One bag, several faults. The cap evicts A's link but B still holds the bytes.
  this->storage_->set_max_rosbags_per_fault(1);
  auto shared = this->row("BURST_A", "fault_BURST_1", 1000);
  this->storage_->store_rosbag_file(shared);
  shared.fault_code = "BURST_B";
  this->storage_->store_rosbag_file(shared);

  const auto shared_path = shared.file_path;
  this->storage_->store_rosbag_file(this->row("BURST_A", "fault_BURST_2", 2000));

  EXPECT_TRUE(std::filesystem::exists(shared_path)) << "the sibling fault still owns the shared bag";
  const auto sibling = this->storage_->get_rosbag_files("BURST_B");
  ASSERT_EQ(sibling.size(), 1u);
  EXPECT_EQ(sibling[0].file_path, shared_path);
}

TYPED_TEST(RosbagRetentionParityTest, RecordingLookupReturnsEveryFaultOfTheBurst) {
  // This is what the gateway authorizes against: the union of faults attached to a
  // recording. Missing one would 404 a download the entity is entitled to.
  this->storage_->set_max_rosbags_per_fault(0);
  auto shared = this->row("LOOKUP_A", "fault_LOOKUP_1", 1000);
  this->storage_->store_rosbag_file(shared);
  shared.fault_code = "LOOKUP_B";
  this->storage_->store_rosbag_file(shared);
  shared.fault_code = "LOOKUP_C";
  this->storage_->store_rosbag_file(shared);

  const auto rows = this->storage_->get_rosbag_files_by_recording("fault_LOOKUP_1");
  ASSERT_EQ(rows.size(), 3u);
  std::vector<std::string> codes;
  for (const auto & r : rows) {
    codes.push_back(r.fault_code);
    EXPECT_EQ(r.file_path, shared.file_path);
  }
  std::sort(codes.begin(), codes.end());
  EXPECT_EQ(codes, (std::vector<std::string>{"LOOKUP_A", "LOOKUP_B", "LOOKUP_C"}));
}

TYPED_TEST(RosbagRetentionParityTest, AnUnknownRecordingLooksUpEmptyRatherThanThrowing) {
  // The gateway's compatibility path depends on this: empty means "not a recording,
  // try it as a fault code", so it must be a normal answer.
  EXPECT_TRUE(this->storage_->get_rosbag_files_by_recording("fault_NOPE_1").empty());
  EXPECT_TRUE(this->storage_->get_rosbag_files("NOPE").empty());
  EXPECT_FALSE(this->storage_->get_rosbag_file("NOPE").has_value());
}

TYPED_TEST(RosbagRetentionParityTest, DeletingARecordingRemovesEveryLinkAndTheBag) {
  // What the quota sweep calls. Deleting by fault code here would wipe the whole
  // history of every fault the bag touched.
  this->storage_->set_max_rosbags_per_fault(0);
  auto shared = this->row("DELREC_A", "fault_DELREC_1", 1000);
  this->storage_->store_rosbag_file(shared);
  shared.fault_code = "DELREC_B";
  this->storage_->store_rosbag_file(shared);
  const auto kept = this->row("DELREC_A", "fault_DELREC_2", 2000);
  this->storage_->store_rosbag_file(kept);

  EXPECT_EQ(this->storage_->delete_rosbag_recording("fault_DELREC_1"), 2u);
  EXPECT_FALSE(std::filesystem::exists(shared.file_path));
  EXPECT_TRUE(this->storage_->get_rosbag_files("DELREC_B").empty());

  const auto survivors = this->storage_->get_rosbag_files("DELREC_A");
  ASSERT_EQ(survivors.size(), 1u) << "the other fault's other recording is untouched";
  EXPECT_EQ(survivors[0].recording_id, "fault_DELREC_2");
  EXPECT_TRUE(std::filesystem::exists(kept.file_path));

  EXPECT_EQ(this->storage_->delete_rosbag_recording("fault_DELREC_1"), 0u);
}

TYPED_TEST(RosbagRetentionParityTest, DeletingAFaultDropsAllItsRecordings) {
  this->storage_->set_max_rosbags_per_fault(0);
  const auto a = this->row("DELALL", "fault_DELALL_1", 1000);
  const auto b = this->row("DELALL", "fault_DELALL_2", 2000);
  this->storage_->store_rosbag_file(a);
  this->storage_->store_rosbag_file(b);

  EXPECT_TRUE(this->storage_->delete_rosbag_file("DELALL"));
  EXPECT_TRUE(this->storage_->get_rosbag_files("DELALL").empty());
  EXPECT_FALSE(std::filesystem::exists(a.file_path));
  EXPECT_FALSE(std::filesystem::exists(b.file_path));
}

TYPED_TEST(RosbagRetentionParityTest, ASharedBagCountsOnceTowardsStorageAcrossSeveralRecordings) {
  // The quota sums bytes per path, not per row. N recordings per fault multiplies the
  // rows, so a per-row sum would over-report and evict healthy bags under no pressure.
  this->storage_->set_max_rosbags_per_fault(0);
  auto shared = this->row("QUOTA_A", "fault_QUOTA_1", 1000);
  this->storage_->store_rosbag_file(shared);
  shared.fault_code = "QUOTA_B";
  this->storage_->store_rosbag_file(shared);
  shared.fault_code = "QUOTA_C";
  this->storage_->store_rosbag_file(shared);
  this->storage_->store_rosbag_file(this->row("QUOTA_A", "fault_QUOTA_2", 2000));

  EXPECT_EQ(this->storage_->get_total_rosbag_storage_bytes(), 2048u) << "two distinct bags of 1024, counted once each";
}

TYPED_TEST(RosbagRetentionParityTest, GetAllRosbagFilesListsEveryRecordingOldestFirst) {
  // The quota sweep walks this list and evicts from the front, so the extra rows this
  // change introduces have to appear here in a defined order.
  this->storage_->set_max_rosbags_per_fault(0);
  this->storage_->store_rosbag_file(this->row("ALL_A", "fault_ALL_1", 3000));
  this->storage_->store_rosbag_file(this->row("ALL_A", "fault_ALL_2", 1000));
  this->storage_->store_rosbag_file(this->row("ALL_B", "fault_ALL_3", 2000));

  const auto rows = this->storage_->get_all_rosbag_files();
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_EQ(rows[0].recording_id, "fault_ALL_2");
  EXPECT_EQ(rows[1].recording_id, "fault_ALL_3");
  EXPECT_EQ(rows[2].recording_id, "fault_ALL_1");
}

/// Snapshot retention parity. Same two backends, same reason.
template <typename Backend>
class SnapshotRetentionParityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("medkit_snap_parity_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    storage_ = BackendFactory<Backend>::make(dir_);
  }

  void TearDown() override {
    storage_.reset();
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  /// One capture: `topics` rows sharing a capture_id, as SnapshotCapture writes them.
  std::vector<SnapshotData> capture(const std::string & code, int64_t capture_id, size_t topics) {
    std::vector<SnapshotData> rows;
    for (size_t i = 0; i < topics; ++i) {
      SnapshotData row;
      row.fault_code = code;
      row.topic = "/topic_" + std::to_string(i);
      row.message_type = "std_msgs/msg/Float64";
      row.data = R"({"data": )" + std::to_string(capture_id) + "}";
      row.captured_at_ns = capture_id * 1000 + static_cast<int64_t>(i);
      row.capture_id = capture_id;
      rows.push_back(row);
    }
    return rows;
  }

  void confirm(const std::string & code) {
    rclcpp::Clock clock;
    storage_->report_fault_event(code, ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED,
                                 ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR, "parity", "/node", clock.now(),
                                 ros2_medkit_fault_manager::DebounceConfig{});
  }

  std::filesystem::path dir_;
  std::unique_ptr<FaultStorage> storage_;
};

TYPED_TEST_SUITE(SnapshotRetentionParityTest, Backends);

TYPED_TEST(SnapshotRetentionParityTest, ACaptureLargerThanTheCapIsKeptWholeRatherThanTorn) {
  // The in-memory trim used to evict the newest set too, leaving the fault with no
  // readings at all, while SQLite exempted it. `storage_type: memory` with the
  // default cap of 10 and eleven configured topics lost every freeze frame.
  this->storage_->set_max_snapshots_per_fault(3);
  this->confirm("BIG");

  this->storage_->store_snapshots(this->capture("BIG", 1, 5));

  const auto rows = this->storage_->get_snapshots("BIG");
  EXPECT_EQ(rows.size(), 5u) << "a capture over the cap on its own is kept entire";
  for (const auto & row : rows) {
    EXPECT_EQ(row.capture_id, 1);
  }
}

TYPED_TEST(SnapshotRetentionParityTest, TheOldestWholeCaptureGoesFirstPastTheCap) {
  this->storage_->set_max_snapshots_per_fault(4);
  this->confirm("EVICT");

  this->storage_->store_snapshots(this->capture("EVICT", 1, 2));
  this->storage_->store_snapshots(this->capture("EVICT", 2, 2));
  this->storage_->store_snapshots(this->capture("EVICT", 3, 2));

  const auto rows = this->storage_->get_snapshots("EVICT");
  ASSERT_EQ(rows.size(), 4u);
  for (const auto & row : rows) {
    EXPECT_NE(row.capture_id, 1) << "the oldest set goes whole, not row by row";
  }
}

TYPED_TEST(SnapshotRetentionParityTest, GetSnapshotsReturnsTheNewestCaptureFirst) {
  // The reader folds rows into a topic map and keeps the leading capture, so the
  // order decides which values a client sees. In-memory returned insertion order,
  // which is the oldest set first - the exact opposite of SQLite.
  this->storage_->set_max_snapshots_per_fault(0);
  this->confirm("ORDER");

  this->storage_->store_snapshots(this->capture("ORDER", 1, 2));
  this->storage_->store_snapshots(this->capture("ORDER", 2, 2));

  const auto rows = this->storage_->get_snapshots("ORDER");
  ASSERT_EQ(rows.size(), 4u);
  EXPECT_EQ(rows.front().capture_id, 2) << "newest capture leads";
  EXPECT_EQ(rows.back().capture_id, 1);
}

TYPED_TEST(SnapshotRetentionParityTest, MaxCaptureIdIsWhatARestartMustContinueFrom) {
  // SnapshotCapture seeds its counter from this. A backend answering 0 with rows
  // present would hand the next capture an id below the stored ones, and eviction
  // protects the highest id - so the capture just written would be the one dropped.
  this->storage_->set_max_snapshots_per_fault(0);
  EXPECT_EQ(this->storage_->get_max_capture_id(), 0) << "nothing stored yet";

  this->confirm("SEED_A");
  this->confirm("SEED_B");
  this->storage_->store_snapshots(this->capture("SEED_A", 4, 1));
  this->storage_->store_snapshots(this->capture("SEED_B", 7, 1));

  EXPECT_EQ(this->storage_->get_max_capture_id(), 7) << "global, not per fault";
}

TYPED_TEST(SnapshotRetentionParityTest, ClearFaultKeepsSnapshotsWhenEvidenceIsRetained) {
  this->storage_->set_max_snapshots_per_fault(0);
  this->confirm("KEEP");
  this->storage_->set_retain_snapshots_on_clear(true);
  this->storage_->store_snapshots(this->capture("KEEP", 1, 2));

  ASSERT_TRUE(this->storage_->clear_fault("KEEP"));

  EXPECT_EQ(this->storage_->get_snapshots("KEEP").size(), 2u);
  EXPECT_TRUE(this->storage_->retains_snapshots_on_clear());
}

TYPED_TEST(SnapshotRetentionParityTest, ClearFaultDropsSnapshotsByDefault) {
  this->storage_->set_max_snapshots_per_fault(0);
  this->confirm("DROP");
  this->storage_->store_snapshots(this->capture("DROP", 1, 2));

  ASSERT_TRUE(this->storage_->clear_fault("DROP"));

  EXPECT_TRUE(this->storage_->get_snapshots("DROP").empty());
  EXPECT_FALSE(this->storage_->retains_snapshots_on_clear());
}

}  // namespace
