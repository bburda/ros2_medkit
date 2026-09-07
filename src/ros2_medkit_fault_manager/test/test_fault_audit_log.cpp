// Copyright 2026 mfaferek93, bburda
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
#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <random>
#include <string>

#include "ros2_medkit_fault_manager/fault_audit_log.hpp"

using ros2_medkit_fault_manager::AuditEvent;
using ros2_medkit_fault_manager::FaultAuditLog;
using ros2_medkit_fault_manager::kTransitionCleared;
using ros2_medkit_fault_manager::kTransitionConfirmed;
using ros2_medkit_fault_manager::kTransitionHealed;
using ros2_medkit_fault_manager::kTransitionOccurred;

namespace {

AuditEvent make_event(const std::string & code, const char * transition, int64_t ts) {
  AuditEvent e;
  e.fault_code = code;
  e.transition = transition;
  e.severity = 2;
  e.status = "CONFIRMED";
  e.source_id = "/robot/source";
  e.description = "pump pressure low";
  e.occurred_at_ns = ts;
  return e;
}

/// Run SQL directly against the audit DB file (used to simulate tampering an
/// immutable row). Asserts success.
void raw_exec(const std::string & db_path, const std::string & sql) {
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(db_path.c_str(), &db), SQLITE_OK);
  char * err = nullptr;
  int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &err);
  std::string err_str = err ? err : "";
  sqlite3_free(err);
  sqlite3_close(db);
  ASSERT_EQ(rc, SQLITE_OK) << err_str;
}

/// Run SQL directly and return the raw result code (does not assert), so a test
/// can confirm the append-only triggers reject a write.
int raw_exec_rc(const std::string & db_path, const std::string & sql) {
  sqlite3 * db = nullptr;
  if (sqlite3_open(db_path.c_str(), &db) != SQLITE_OK) {
    ADD_FAILURE() << "sqlite3_open failed: " << (db ? sqlite3_errmsg(db) : "out of memory");
    sqlite3_close(db);  // sqlite3 allows close on a failed-open handle (incl. nullptr).
    return SQLITE_ERROR;
  }
  int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, nullptr);
  sqlite3_close(db);
  return rc;
}

/// Drop the DB-level append-only triggers so a raw tamper write can land. This
/// mimics an attacker who bypassed the (defense-in-depth, not security boundary)
/// triggers; verify() must still detect the recompute-free edit afterwards.
const char * const kDropTriggers =
    "DROP TRIGGER IF EXISTS audit_log_no_update; DROP TRIGGER IF EXISTS audit_log_no_delete; ";

/// Drop the guard-gated audit_anchors triggers so a raw anchor write can land
/// (same attacker model as kDropTriggers, for the anchor table).
const char * const kDropAnchorTriggers =
    "DROP TRIGGER IF EXISTS audit_anchors_no_update; DROP TRIGGER IF EXISTS audit_anchors_no_insert; "
    "DROP TRIGGER IF EXISTS audit_anchors_no_delete; ";

/// Count rows in a table via a fresh connection (used to assert audit_anchors
/// stays bounded across many rotations).
int64_t count_table_rows(const std::string & db_path, const std::string & table) {
  sqlite3 * db = nullptr;
  EXPECT_EQ(sqlite3_open(db_path.c_str(), &db), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  const std::string sql = "SELECT COUNT(*) FROM " + table;
  int64_t rows = -1;
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK && sqlite3_step(stmt) == SQLITE_ROW) {
    rows = sqlite3_column_int64(stmt, 0);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return rows;
}

}  // namespace

class FaultAuditLogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    path_ = (std::filesystem::temp_directory_path() / ("test_audit_" + std::to_string(dist(gen)) + ".db")).string();
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
    std::filesystem::remove(path_ + "-wal", ec);
    std::filesystem::remove(path_ + "-shm", ec);
  }

  std::string path_;
};

// Each transition appends a chained row with a monotonic seq and linked hashes.
TEST_F(FaultAuditLogTest, AppendsChainedRowPerTransition) {
  FaultAuditLog log(path_);

  EXPECT_EQ(log.append(make_event("F1", kTransitionOccurred, 100)), 1);
  EXPECT_EQ(log.append(make_event("F1", kTransitionConfirmed, 200)), 2);
  EXPECT_EQ(log.append(make_event("F1", kTransitionCleared, 300)), 3);

  auto records = log.read();
  ASSERT_EQ(records.size(), 3u);

  // First row links to genesis; each subsequent prev_hash equals the prior hash.
  EXPECT_EQ(records[0].seq, 1);
  EXPECT_EQ(records[0].prev_hash, FaultAuditLog::genesis_hash());
  EXPECT_EQ(records[1].prev_hash, records[0].record_hash);
  EXPECT_EQ(records[2].prev_hash, records[1].record_hash);

  // record_hash is the sha256 of prev_hash + canonical(event).
  const std::string expected =
      FaultAuditLog::sha256_hex(records[0].prev_hash + FaultAuditLog::canonicalize(1, records[0].event));
  EXPECT_EQ(records[0].record_hash, expected);

  EXPECT_EQ(log.record_count(), 3);
  EXPECT_EQ(log.head().seq, 3);
  EXPECT_EQ(log.head().record_hash, records[2].record_hash);
}

// Verify confirms an untampered chain.
TEST_F(FaultAuditLogTest, VerifyUntamperedChain) {
  FaultAuditLog log(path_);
  for (int i = 0; i < 10; ++i) {
    log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
  }
  auto result = log.verify();
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.checked, 10);
}

// Known SHA-256 vector proves the EVP wiring (sha256("") == e3b0c442...).
TEST_F(FaultAuditLogTest, Sha256KnownVector) {
  EXPECT_EQ(FaultAuditLog::sha256_hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

// Editing a past row makes verify fail.
TEST_F(FaultAuditLogTest, EditingPastRowFailsVerify) {
  {
    FaultAuditLog log(path_);
    log.append(make_event("F1", kTransitionOccurred, 100));
    log.append(make_event("F2", kTransitionOccurred, 200));
    log.append(make_event("F3", kTransitionOccurred, 300));
    ASSERT_TRUE(log.verify().ok);
  }

  // Tamper: change a stored field without recomputing the hash.
  raw_exec(path_, std::string(kDropTriggers) + "UPDATE audit_log SET description = 'forged' WHERE seq = 2");

  FaultAuditLog reopened(path_);
  auto result = reopened.verify();
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.bad_seq, 2);
}

// Deleting a middle row makes verify fail (chain gap).
TEST_F(FaultAuditLogTest, DeletingMiddleRowFailsVerify) {
  {
    FaultAuditLog log(path_);
    log.append(make_event("F1", kTransitionOccurred, 100));
    log.append(make_event("F2", kTransitionOccurred, 200));
    log.append(make_event("F3", kTransitionOccurred, 300));
  }

  raw_exec(path_, std::string(kDropTriggers) + "DELETE FROM audit_log WHERE seq = 2");

  FaultAuditLog reopened(path_);
  EXPECT_FALSE(reopened.verify().ok);
}

// Deleting the newest row is caught by the persisted head check.
TEST_F(FaultAuditLogTest, DeletingNewestRowFailsVerify) {
  {
    FaultAuditLog log(path_);
    log.append(make_event("F1", kTransitionOccurred, 100));
    log.append(make_event("F2", kTransitionOccurred, 200));
    log.append(make_event("F3", kTransitionOccurred, 300));
  }

  // Drop the last row but leave the head pointing at seq 3.
  raw_exec(path_, std::string(kDropTriggers) + "DELETE FROM audit_log WHERE seq = 3");

  FaultAuditLog reopened(path_);
  EXPECT_FALSE(reopened.verify().ok);
}

// Truncation bypass: deleting the newest row AND the head row must still FAIL.
// The head row is read directly from the DB, so a missing head on a non-empty
// log is treated as tampering rather than silently rebuilt from MAX(seq).
TEST_F(FaultAuditLogTest, DeletingNewestRowAndHeadFailsVerify) {
  {
    FaultAuditLog log(path_);
    log.append(make_event("F1", kTransitionOccurred, 100));
    log.append(make_event("F2", kTransitionOccurred, 200));
    log.append(make_event("F3", kTransitionOccurred, 300));
    ASSERT_TRUE(log.verify().ok);
  }

  // Drop the newest row and the persisted head row together.
  raw_exec(path_, std::string(kDropTriggers) +
                      "DELETE FROM audit_log WHERE seq = 3; DELETE FROM audit_chain_head WHERE id = 1;");

  FaultAuditLog reopened(path_);
  auto result = reopened.verify();
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.bad_seq, 2);  // last surviving row
}

// The chain head persists across a reopen and the chain resumes from it.
TEST_F(FaultAuditLogTest, HeadPersistsAcrossReopen) {
  std::string head_hash;
  {
    FaultAuditLog log(path_);
    log.append(make_event("F1", kTransitionOccurred, 100));
    log.append(make_event("F2", kTransitionConfirmed, 200));
    head_hash = log.head().record_hash;
    EXPECT_EQ(log.head().seq, 2);
  }

  FaultAuditLog reopened(path_);
  EXPECT_EQ(reopened.head().seq, 2);
  EXPECT_EQ(reopened.head().record_hash, head_hash);

  // The next append continues the same chain.
  EXPECT_EQ(reopened.append(make_event("F3", kTransitionCleared, 300)), 3);
  auto records = reopened.read();
  ASSERT_EQ(records.size(), 3u);
  EXPECT_EQ(records[2].prev_hash, head_hash);
  EXPECT_TRUE(reopened.verify().ok);
}

// Rotation seals a segment and prunes it, leaving the tail verifiable.
TEST_F(FaultAuditLogTest, RotationSealsAndPrunesButStaysVerifiable) {
  FaultAuditLog log(path_, /*retention_max_records=*/5);
  for (int i = 1; i <= 12; ++i) {
    log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
  }

  // Only the most recent 5 rows are retained.
  EXPECT_EQ(log.record_count(), 5);
  auto records = log.read();
  ASSERT_EQ(records.size(), 5u);
  EXPECT_EQ(records.front().seq, 8);
  EXPECT_EQ(records.back().seq, 12);
  EXPECT_EQ(log.head().seq, 12);

  // The surviving tail still verifies via the sealed anchor.
  auto result = log.verify();
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.checked, 5);
}

// Tampering a row inside a rotated tail is still detected.
TEST_F(FaultAuditLogTest, RotationThenTamperFails) {
  {
    FaultAuditLog log(path_, /*retention_max_records=*/5);
    for (int i = 1; i <= 12; ++i) {
      log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
    }
  }
  raw_exec(path_, std::string(kDropTriggers) + "UPDATE audit_log SET source_id = 'forged' WHERE seq = 10");

  FaultAuditLog reopened(path_, 5);
  auto result = reopened.verify();
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.bad_seq, 10);
}

// Removing the sealed anchor breaks the link for the oldest retained row.
TEST_F(FaultAuditLogTest, MissingAnchorFailsVerify) {
  {
    FaultAuditLog log(path_, /*retention_max_records=*/5);
    for (int i = 1; i <= 12; ++i) {
      log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
    }
  }
  raw_exec(path_, std::string(kDropAnchorTriggers) + "DELETE FROM audit_anchors");

  FaultAuditLog reopened(path_, 5);
  EXPECT_FALSE(reopened.verify().ok);
}

// canonicalize is deterministic and order-stable.
TEST_F(FaultAuditLogTest, CanonicalizeDeterministic) {
  auto e = make_event("F1", kTransitionConfirmed, 12345);
  EXPECT_EQ(FaultAuditLog::canonicalize(7, e), FaultAuditLog::canonicalize(7, e));
  // seq is part of the canonical form, so a different seq changes the bytes.
  EXPECT_NE(FaultAuditLog::canonicalize(7, e), FaultAuditLog::canonicalize(8, e));
}

// read(after_seq) returns only newer records, oldest-first.
TEST_F(FaultAuditLogTest, ReadAfterSeq) {
  FaultAuditLog log(path_);
  for (int i = 1; i <= 5; ++i) {
    log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
  }
  auto tail = log.read(/*limit=*/0, /*after_seq=*/3);
  ASSERT_EQ(tail.size(), 2u);
  EXPECT_EQ(tail[0].seq, 4);
  EXPECT_EQ(tail[1].seq, 5);
}

// Defense-in-depth: the append-only triggers reject an out-of-band UPDATE or
// DELETE on audit_log while the in-process rotation prune still works.
TEST_F(FaultAuditLogTest, AppendOnlyTriggersRejectOutOfBandWrites) {
  {
    FaultAuditLog log(path_);
    log.append(make_event("F1", kTransitionOccurred, 100));
    log.append(make_event("F2", kTransitionOccurred, 200));
  }

  // Both a raw UPDATE and a raw DELETE are aborted by the triggers.
  EXPECT_NE(raw_exec_rc(path_, "UPDATE audit_log SET description = 'forged' WHERE seq = 1"), SQLITE_OK);
  EXPECT_NE(raw_exec_rc(path_, "DELETE FROM audit_log WHERE seq = 1"), SQLITE_OK);

  // Rows are intact and the chain still verifies.
  FaultAuditLog reopened(path_);
  EXPECT_EQ(reopened.record_count(), 2);
  EXPECT_TRUE(reopened.verify().ok);
}

// The guarded rotation prune is exempt from the append-only delete trigger.
TEST_F(FaultAuditLogTest, RotationPrunePassesAppendOnlyTrigger) {
  FaultAuditLog log(path_, /*retention_max_records=*/3);
  for (int i = 1; i <= 8; ++i) {
    log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
  }
  EXPECT_EQ(log.record_count(), 3);
  EXPECT_TRUE(log.verify().ok);
}

// A "healed" auto-recovery row chains and verifies like any other transition, and
// stays distinct from a "cleared" so the timeline can tell them apart.
TEST_F(FaultAuditLogTest, HealedTransitionChainVerifies) {
  FaultAuditLog log(path_);
  log.append(make_event("F1", kTransitionOccurred, 100));
  log.append(make_event("F1", kTransitionConfirmed, 200));
  log.append(make_event("F1", kTransitionHealed, 300));

  auto records = log.read();
  ASSERT_EQ(records.size(), 3u);
  EXPECT_EQ(records[2].event.transition, kTransitionHealed);
  EXPECT_NE(records[2].event.transition, std::string(kTransitionCleared));
  EXPECT_TRUE(log.verify().ok);
}

// Defense-in-depth (item 4): the prune guard itself is protected. An out-of-band
// connection cannot flip audit_prune_guard open, so it cannot then delete a
// prefix past the append-only delete trigger. The in-process prune still works.
TEST_F(FaultAuditLogTest, GuardProtectTriggerBlocksExternalGuardFlip) {
  {
    FaultAuditLog log(path_, /*retention_max_records=*/3);
    for (int i = 1; i <= 8; ++i) {
      log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
    }
    EXPECT_EQ(log.record_count(), 3);  // in-process prune succeeded despite the protect trigger
    EXPECT_TRUE(log.verify().ok);
  }

  // An external writer (no in-process temp unlock marker) cannot open the guard.
  EXPECT_NE(raw_exec_rc(path_, "UPDATE audit_prune_guard SET enabled = 1 WHERE id = 1"), SQLITE_OK);
  // With the guard still closed, a raw DELETE remains blocked by the delete trigger.
  EXPECT_NE(raw_exec_rc(path_, "DELETE FROM audit_log"), SQLITE_OK);

  FaultAuditLog reopened(path_, 3);
  EXPECT_EQ(reopened.record_count(), 3);
  EXPECT_TRUE(reopened.verify().ok);
}

// Non-UTF-8 bytes in unvalidated content (description / source_id come straight
// from the ReportFault request) must NOT make canonicalize -> append throw and
// leave a silent gap. Canonicalization is total: any byte sequence hashes
// reproducibly and the chain still verifies.
TEST_F(FaultAuditLogTest, NonUtf8ContentAppendsAndVerifies) {
  FaultAuditLog log(path_);

  AuditEvent e = make_event("F1", kTransitionOccurred, 100);
  e.description = "pressure \xff\xfe low";  // invalid UTF-8 bytes
  e.source_id = "src\x80\x81";              // invalid UTF-8 continuation bytes

  int64_t seq = 0;
  ASSERT_NO_THROW(seq = log.append(e));
  EXPECT_EQ(seq, 1);

  // A plain append still chains on top of the non-UTF-8 record.
  ASSERT_NO_THROW(log.append(make_event("F2", kTransitionConfirmed, 200)));

  // canonicalize is deterministic on the same arbitrary bytes.
  EXPECT_EQ(FaultAuditLog::canonicalize(1, e), FaultAuditLog::canonicalize(1, e));

  auto result = log.verify();
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.checked, 2);
}

// Embedded NUL bytes in unvalidated content (description / source_id come from
// the ReportFault request) must survive the write/read round-trip byte-exact.
// bind_text stores the full length, so the read path must too - reading a
// NUL-terminated const char* would truncate at the first NUL, change the
// canonical form on re-read, and make verify() falsely report tampering on
// legitimate content.
TEST_F(FaultAuditLogTest, EmbeddedNulContentRoundTripsAndVerifies) {
  FaultAuditLog log(path_);

  AuditEvent e = make_event("F1", kTransitionOccurred, 100);
  e.description = std::string("pressure\0low", 12);  // embedded NUL, bytes after it
  e.source_id = std::string("src\0id", 6);           // embedded NUL, bytes after it
  e.status = std::string("CON\0FIRMED", 10);         // embedded NUL, bytes after it

  int64_t seq = 0;
  ASSERT_NO_THROW(seq = log.append(e));
  EXPECT_EQ(seq, 1);

  // A plain append still chains on top of the embedded-NUL record.
  ASSERT_NO_THROW(log.append(make_event("F2", kTransitionConfirmed, 200)));

  auto result = log.verify();
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.checked, 2);

  // The stored bytes read back whole, not truncated at the first NUL.
  auto records = log.read();
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].event.description, e.description);
  EXPECT_EQ(records[0].event.source_id, e.source_id);
  EXPECT_EQ(records[0].event.status, e.status);
}

// Many rotations must not grow audit_anchors without bound: verify() only ever
// needs the anchor at the current prune boundary, so older anchors are pruned in
// the same rotation. audit_log stays at the cap and audit_anchors stays bounded.
TEST_F(FaultAuditLogTest, ManyRotationsDoNotGrowAnchorsUnbounded) {
  FaultAuditLog log(path_, /*retention_max_records=*/3);
  for (int i = 1; i <= 100; ++i) {
    log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
  }
  EXPECT_EQ(log.record_count(), 3);
  // Without pruning old anchors this would be ~97 rows; it must stay bounded.
  EXPECT_LE(count_table_rows(path_, "audit_anchors"), 1);

  // Pruning old anchors must not break verify: the surviving tail still links to
  // the current boundary anchor.
  auto result = log.verify();
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.checked, 3);
}

// A rotation failure AFTER a durable append must not fail the append: COMMIT
// already made the record durable, so a broken rotation is counted (retried next
// time), never surfaced to the caller as a failed/dropped write.
TEST_F(FaultAuditLogTest, RotationFailureDoesNotFailDurableAppend) {
  FaultAuditLog log(path_, /*retention_max_records=*/3);
  for (int i = 1; i <= 5; ++i) {
    log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
  }
  ASSERT_EQ(log.record_count(), 3);
  ASSERT_EQ(log.rotation_failures(), 0);

  // Sabotage rotation out-of-band: drop audit_anchors so the next rotation's
  // anchor INSERT fails INSIDE rotate_if_needed_locked, after the append COMMIT.
  raw_exec(path_, std::string(kDropAnchorTriggers) + "DROP TABLE audit_anchors;");

  int64_t seq = 0;
  EXPECT_NO_THROW(seq = log.append(make_event("F6", kTransitionOccurred, 600)));
  EXPECT_EQ(seq, 6);
  EXPECT_EQ(log.rotation_failures(), 1);

  // The append is durable: the record is present and is the newest.
  auto tail = log.read(/*limit=*/0, /*after_seq=*/5);
  ASSERT_EQ(tail.size(), 1u);
  EXPECT_EQ(tail.front().seq, 6);
  EXPECT_EQ(log.head().seq, 6);
}

// Defense-in-depth (item 7): audit_anchors carries the same guard-gated triggers
// as audit_log, so an out-of-band INSERT/UPDATE/DELETE of an anchor is rejected
// while the guard is closed. The in-process rotation (guard open) still seals and
// prunes anchors normally.
TEST_F(FaultAuditLogTest, AnchorTriggersRejectOutOfBandWrites) {
  {
    FaultAuditLog log(path_, /*retention_max_records=*/3);
    for (int i = 1; i <= 8; ++i) {
      log.append(make_event("F" + std::to_string(i), kTransitionOccurred, 100 + i));
    }
    EXPECT_EQ(log.record_count(), 3);
    EXPECT_TRUE(log.verify().ok);
  }

  // A forged anchor INSERT, an UPDATE of the real anchor, and a DELETE are all
  // rejected out-of-band (guard closed), so a forged prefix-truncation cannot be
  // planted without first dropping the triggers.
  EXPECT_NE(raw_exec_rc(path_, "INSERT INTO audit_anchors (last_seq, sealed_at_ns, last_hash) VALUES (999, 0, 'x')"),
            SQLITE_OK);
  EXPECT_NE(raw_exec_rc(path_, "UPDATE audit_anchors SET last_hash = 'forged'"), SQLITE_OK);
  EXPECT_NE(raw_exec_rc(path_, "DELETE FROM audit_anchors"), SQLITE_OK);

  // The real anchor survived and the tail still verifies.
  FaultAuditLog reopened(path_, 3);
  EXPECT_TRUE(reopened.verify().ok);
}

// A planned-stop transition belongs to the installation, not to a fault: it
// carries no fault code, and the chain has to survive that.
TEST_F(FaultAuditLogTest, PlannedStopTransitionsChainWithNoFaultCode) {
  FaultAuditLog log(path_);

  AuditEvent started;
  started.fault_code = "";
  started.transition = ros2_medkit_fault_manager::kTransitionPlannedStopStarted;
  started.status = "ACTIVE";
  started.source_id = "shift_lead";
  started.description = "line 3 quarterly maintenance";
  started.occurred_at_ns = 1000;
  EXPECT_EQ(log.append(started), 1);

  AuditEvent ended;
  ended.fault_code = "";
  ended.transition = ros2_medkit_fault_manager::kTransitionPlannedStopEnded;
  ended.status = "INACTIVE";
  ended.source_id = "shift_lead";
  ended.description = "line 3 back up";
  ended.occurred_at_ns = 2000;
  EXPECT_EQ(log.append(ended), 2);

  EXPECT_TRUE(log.verify().ok);

  const auto records = log.read();
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].event.transition, "planned_stop_started");
  EXPECT_EQ(records[0].event.fault_code, "");
  EXPECT_EQ(records[0].event.source_id, "shift_lead");
  EXPECT_EQ(records[0].event.description, "line 3 quarterly maintenance");
  EXPECT_EQ(records[1].event.transition, "planned_stop_ended");
  EXPECT_EQ(records[1].event.status, "INACTIVE");
  EXPECT_EQ(records[1].prev_hash, records[0].record_hash);

  // The declaration is part of the hashed record, not a side note: rewriting the
  // reason without recomputing the chain has to be detectable.
  raw_exec(path_, std::string(kDropTriggers) + "UPDATE audit_log SET description = 'unplanned outage' WHERE seq = 1;");
  FaultAuditLog reopened(path_);
  EXPECT_FALSE(reopened.verify().ok);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
