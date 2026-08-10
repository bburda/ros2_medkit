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

#include "ros2_medkit_fault_manager/fault_audit_log.hpp"

#include <openssl/evp.h>

#include <array>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace ros2_medkit_fault_manager {

namespace {

/// 64 hex zeros: prev_hash of the first record. A fixed, well-known anchor so a
/// verifier can confirm the chain starts where it claims to.
constexpr const char * kGenesisHash = "0000000000000000000000000000000000000000000000000000000000000000";

/// JSON-encode a string value for the canonical form. Audit content
/// (description / source_id / fault_code / ...) is UNVALIDATED request data and
/// may contain arbitrary bytes. nlohmann's default (strict) UTF-8 handler THROWS
/// on any non-UTF-8 byte; that would make canonicalize() -> append() throw and
/// leave a silent, un-verifiable gap in the chain. The `replace` handler
/// substitutes U+FFFD for invalid bytes deterministically, so any byte sequence
/// canonicalizes reproducibly and append never throws on content. Output for
/// valid UTF-8 is byte-for-byte identical to the old strict dump, so chains
/// written before this change still verify.
std::string json_encode(const std::string & value) {
  return nlohmann::json(value).dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
}

/// RAII wrapper for a prepared SQLite statement (audit-log local copy).
class Stmt {
 public:
  Stmt(sqlite3 * db, const char * sql) : db_(db) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
      throw std::runtime_error(std::string("audit: failed to prepare: ") + sqlite3_errmsg(db));
    }
  }

  ~Stmt() {
    if (stmt_) {
      sqlite3_finalize(stmt_);
    }
  }

  Stmt(const Stmt &) = delete;
  Stmt & operator=(const Stmt &) = delete;
  Stmt(Stmt &&) = delete;
  Stmt & operator=(Stmt &&) = delete;

  void bind_text(int index, const std::string & value) {
    if (value.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("audit: text too large to bind");
    }
    if (sqlite3_bind_text(stmt_, index, value.c_str(), static_cast<int>(value.size()), SQLITE_TRANSIENT) != SQLITE_OK) {
      throw std::runtime_error(std::string("audit: failed to bind text: ") + sqlite3_errmsg(db_));
    }
  }

  void bind_int64(int index, int64_t value) {
    if (sqlite3_bind_int64(stmt_, index, value) != SQLITE_OK) {
      throw std::runtime_error(std::string("audit: failed to bind int64: ") + sqlite3_errmsg(db_));
    }
  }

  int step() {
    return sqlite3_step(stmt_);
  }

  std::string column_text(int index) {
    // sqlite3_column_text yields a NUL-terminated pointer, so std::string(text)
    // would truncate content at the first embedded NUL. bind_text stores the full
    // bytes via value.size(), so reading with the real length keeps the write/read
    // round-trip byte-exact and the recomputed hash matching. sqlite3_column_bytes
    // must be read AFTER sqlite3_column_text so it reports the UTF-8 byte count.
    const auto * text = reinterpret_cast<const char *>(sqlite3_column_text(stmt_, index));
    if (text == nullptr) {
      return std::string();
    }
    return std::string(text, static_cast<std::size_t>(sqlite3_column_bytes(stmt_, index)));
  }

  int64_t column_int64(int index) {
    return sqlite3_column_int64(stmt_, index);
  }

 private:
  sqlite3 * db_;
  sqlite3_stmt * stmt_{nullptr};
};

void exec_or_throw(sqlite3 * db, const char * sql, const char * what) {
  char * err_msg = nullptr;
  if (sqlite3_exec(db, sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error(std::string("audit: ") + what + ": " + error);
  }
}

/// SQL function registered ONLY on this in-process connection. The prune-guard
/// protection trigger calls it, so an out-of-band connection (which never
/// registered it) cannot flip the guard: preparing the UPDATE fails on the
/// unknown function. Connection-scoped, hence not a security boundary - a
/// write-capable adversary can register the same function or drop the trigger.
void audit_prune_authorized(sqlite3_context * ctx, int /*argc*/, sqlite3_value ** /*argv*/) {
  sqlite3_result_int(ctx, 1);
}

}  // namespace

FaultAuditLog::FaultAuditLog(const std::string & db_path, int64_t retention_max_records)
  : db_path_(db_path), retention_max_records_(retention_max_records < 0 ? 0 : retention_max_records) {
  int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;
  sqlite3 * raw = nullptr;
  if (sqlite3_open_v2(db_path.c_str(), &raw, flags, nullptr) != SQLITE_OK) {
    std::string error = raw ? sqlite3_errmsg(raw) : "unknown error";
    sqlite3_close(raw);  // no-op on nullptr; releases a failed-open handle.
    throw std::runtime_error("audit: failed to open '" + db_path + "': " + error);
  }
  // Take ownership immediately. From here on ANY throw (including out of the
  // PRAGMA / schema / head setup below) runs db_'s destructor and closes the
  // handle: a member unique_ptr is destroyed even when the constructor throws,
  // whereas a raw pointer whose only cleanup lived in ~FaultAuditLog() would leak.
  db_.reset(raw);

  exec_or_throw(db_.get(), "PRAGMA journal_mode=WAL;", "enable WAL");
  sqlite3_busy_timeout(db_.get(), 5000);

  // Authorize this connection for the prune-guard protection trigger (see below).
  if (sqlite3_create_function_v2(db_.get(), "audit_prune_authorized", 0, SQLITE_UTF8, nullptr, &audit_prune_authorized,
                                 nullptr, nullptr, nullptr) != SQLITE_OK) {
    throw std::runtime_error(std::string("audit: failed to register prune-authorization function: ") +
                             sqlite3_errmsg(db_.get()));
  }

  initialize_schema();
  head_ = load_head_locked();
}

FaultAuditLog::~FaultAuditLog() = default;

std::string FaultAuditLog::genesis_hash() {
  return kGenesisHash;
}

std::string FaultAuditLog::sha256_hex(const std::string & data) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> md{};
  unsigned int md_len = 0;

  EVP_MD_CTX * ctx = EVP_MD_CTX_new();
  if (ctx == nullptr) {
    throw std::runtime_error("audit: EVP_MD_CTX_new failed");
  }
  const bool ok = EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) == 1 &&
                  EVP_DigestUpdate(ctx, data.data(), data.size()) == 1 &&
                  EVP_DigestFinal_ex(ctx, md.data(), &md_len) == 1;
  EVP_MD_CTX_free(ctx);
  if (!ok) {
    throw std::runtime_error("audit: SHA-256 digest failed");
  }

  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(static_cast<std::size_t>(md_len) * 2);
  for (unsigned int i = 0; i < md_len; ++i) {
    out.push_back(kHex[md[i] >> 4]);
    out.push_back(kHex[md[i] & 0x0F]);
  }
  return out;
}

std::string FaultAuditLog::canonicalize(int64_t seq, const AuditEvent & event) {
  // Deterministic field order with JSON-escaped string values. Numeric fields
  // render in fixed decimal form. This is the exact byte sequence the hash is
  // taken over, so it must be stable across processes and re-reads (and total
  // for arbitrary bytes - see json_encode).
  std::string out;
  out.reserve(192);
  out += "{\"seq\":";
  out += std::to_string(seq);
  out += ",\"ts\":";
  out += std::to_string(event.occurred_at_ns);
  out += ",\"code\":";
  out += json_encode(event.fault_code);
  out += ",\"transition\":";
  out += json_encode(event.transition);
  out += ",\"severity\":";
  out += std::to_string(static_cast<int>(event.severity));
  out += ",\"status\":";
  out += json_encode(event.status);
  out += ",\"source\":";
  out += json_encode(event.source_id);
  out += ",\"description\":";
  out += json_encode(event.description);
  out += "}";
  return out;
}

void FaultAuditLog::initialize_schema() {
  // Immutable transition rows. seq is the monotonic chain index.
  exec_or_throw(db_.get(),
                R"(
    CREATE TABLE IF NOT EXISTS audit_log (
      seq INTEGER PRIMARY KEY,
      occurred_at_ns INTEGER NOT NULL,
      fault_code TEXT NOT NULL,
      transition TEXT NOT NULL,
      severity INTEGER NOT NULL,
      status TEXT NOT NULL,
      source_id TEXT NOT NULL,
      description TEXT NOT NULL,
      prev_hash TEXT NOT NULL,
      record_hash TEXT NOT NULL
    );
    CREATE INDEX IF NOT EXISTS idx_audit_log_fault_code ON audit_log(fault_code);
  )",
                "create audit_log table");

  // Single-row persisted chain head, so the chain resumes across restarts.
  exec_or_throw(db_.get(),
                R"(
    CREATE TABLE IF NOT EXISTS audit_chain_head (
      id INTEGER PRIMARY KEY CHECK (id = 1),
      seq INTEGER NOT NULL,
      record_hash TEXT NOT NULL
    );
  )",
                "create audit_chain_head table");

  // Sealed-segment anchors written before pruning. Each captures the final
  // (seq, hash) of a pruned prefix so the surviving tail stays verifiable.
  exec_or_throw(db_.get(),
                R"(
    CREATE TABLE IF NOT EXISTS audit_anchors (
      last_seq INTEGER PRIMARY KEY,
      sealed_at_ns INTEGER NOT NULL,
      last_hash TEXT NOT NULL
    );
  )",
                "create audit_anchors table");

  // Append-only enforcement at the DB layer (defense-in-depth, NOT a security
  // boundary: anyone able to write the file can also DROP these triggers). The
  // single-row guard lets the in-process rotation prune delete a sealed prefix
  // while every other UPDATE/DELETE on audit_log is rejected.
  exec_or_throw(db_.get(),
                R"(
    CREATE TABLE IF NOT EXISTS audit_prune_guard (
      id INTEGER PRIMARY KEY CHECK (id = 1),
      enabled INTEGER NOT NULL DEFAULT 0
    );
    INSERT INTO audit_prune_guard (id, enabled) VALUES (1, 0)
      ON CONFLICT(id) DO NOTHING;
    CREATE TRIGGER IF NOT EXISTS audit_log_no_update
    BEFORE UPDATE ON audit_log
    BEGIN
      SELECT RAISE(ABORT, 'audit_log is append-only');
    END;
    CREATE TRIGGER IF NOT EXISTS audit_log_no_delete
    BEFORE DELETE ON audit_log
    WHEN (SELECT enabled FROM audit_prune_guard WHERE id = 1) IS NOT 1
    BEGIN
      SELECT RAISE(ABORT, 'audit_log is append-only');
    END;
  )",
                "create append-only triggers");

  // Protect the prune guard itself. Without this, an external writer could simply
  // `UPDATE audit_prune_guard SET enabled = 1` and then DELETE rows past the
  // append-only delete trigger. The guard may only be flipped by a connection on
  // which audit_prune_authorized() is registered (this in-process connection); an
  // out-of-band UPDATE fails because that function is unknown to it. This is still
  // defense-in-depth, not a security boundary: a write-capable adversary can
  // register the same function or DROP this trigger. verify() remains the backstop.
  exec_or_throw(db_.get(),
                R"(
    CREATE TRIGGER IF NOT EXISTS audit_prune_guard_protect
    BEFORE UPDATE ON audit_prune_guard
    WHEN audit_prune_authorized() IS NOT 1
    BEGIN
      SELECT RAISE(ABORT, 'audit_prune_guard is protected (in-process prune only)');
    END;
  )",
                "create prune-guard protection trigger");

  // Guard audit_anchors with the SAME in-process prune guard as audit_log.
  // verify() links the surviving tail back to a sealed anchor, so an out-of-band
  // writer that could freely INSERT a fake anchor (or DELETE a real one) could
  // forge a prefix-truncation that is indistinguishable from legitimate pruning.
  // Anchors are never UPDATEd by this class, and the in-process rotation INSERT +
  // prune both run with the guard open, so those pass. Defense-in-depth only, NOT
  // a security boundary: a write-capable adversary can flip the guard or DROP
  // these triggers; verify() remains the backstop and cannot see a forged
  // truncation performed this way.
  exec_or_throw(db_.get(),
                R"(
    CREATE TRIGGER IF NOT EXISTS audit_anchors_no_update
    BEFORE UPDATE ON audit_anchors
    BEGIN
      SELECT RAISE(ABORT, 'audit_anchors is not updatable');
    END;
    CREATE TRIGGER IF NOT EXISTS audit_anchors_no_insert
    BEFORE INSERT ON audit_anchors
    WHEN (SELECT enabled FROM audit_prune_guard WHERE id = 1) IS NOT 1
    BEGIN
      SELECT RAISE(ABORT, 'audit_anchors sealing is in-process only');
    END;
    CREATE TRIGGER IF NOT EXISTS audit_anchors_no_delete
    BEFORE DELETE ON audit_anchors
    WHEN (SELECT enabled FROM audit_prune_guard WHERE id = 1) IS NOT 1
    BEGIN
      SELECT RAISE(ABORT, 'audit_anchors pruning is in-process only');
    END;
  )",
                "create audit_anchors protection triggers");
}

std::optional<ChainHead> FaultAuditLog::read_head_row_locked() const {
  Stmt stmt(db_.get(), "SELECT seq, record_hash FROM audit_chain_head WHERE id = 1");
  if (stmt.step() == SQLITE_ROW) {
    ChainHead head_record;
    head_record.seq = stmt.column_int64(0);
    head_record.record_hash = stmt.column_text(1);
    return head_record;
  }
  return std::nullopt;
}

ChainHead FaultAuditLog::load_head_locked() const {
  // Resume strictly from the persisted head row. There is deliberately no
  // MAX(seq) fallback: append+head-update are written in one transaction, so a
  // missing head row while rows exist means tampering, not a recoverable crash.
  // verify() reports that case as a failure rather than silently fabricating a
  // head from the surviving rows.
  if (auto head_record = read_head_row_locked()) {
    return *head_record;
  }
  return ChainHead{0, genesis_hash()};
}

void FaultAuditLog::store_head_locked(const ChainHead & head_record) {
  Stmt stmt(db_.get(),
            "INSERT INTO audit_chain_head (id, seq, record_hash) VALUES (1, ?, ?) "
            "ON CONFLICT(id) DO UPDATE SET seq = excluded.seq, record_hash = excluded.record_hash");
  stmt.bind_int64(1, head_record.seq);
  stmt.bind_text(2, head_record.record_hash);
  if (stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("audit: failed to store head: ") + sqlite3_errmsg(db_.get()));
  }
}

int64_t FaultAuditLog::append(const AuditEvent & event) {
  std::lock_guard<std::mutex> lock(mutex_);

  const int64_t new_seq = head_.seq + 1;
  const std::string prev_hash = head_.record_hash;
  const std::string canonical = canonicalize(new_seq, event);
  const std::string record_hash = sha256_hex(prev_hash + canonical);

  exec_or_throw(db_.get(), "BEGIN IMMEDIATE", "begin append");
  try {
    Stmt insert(db_.get(),
                "INSERT INTO audit_log (seq, occurred_at_ns, fault_code, transition, severity, status, "
                "source_id, description, prev_hash, record_hash) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
    insert.bind_int64(1, new_seq);
    insert.bind_int64(2, event.occurred_at_ns);
    insert.bind_text(3, event.fault_code);
    insert.bind_text(4, event.transition);
    insert.bind_int64(5, static_cast<int64_t>(event.severity));
    insert.bind_text(6, event.status);
    insert.bind_text(7, event.source_id);
    insert.bind_text(8, event.description);
    insert.bind_text(9, prev_hash);
    insert.bind_text(10, record_hash);
    if (insert.step() != SQLITE_DONE) {
      throw std::runtime_error(std::string("audit: failed to insert record: ") + sqlite3_errmsg(db_.get()));
    }

    store_head_locked(ChainHead{new_seq, record_hash});
    exec_or_throw(db_.get(), "COMMIT", "commit append");
  } catch (...) {
    exec_or_throw(db_.get(), "ROLLBACK", "rollback append");
    throw;
  }

  head_ = ChainHead{new_seq, record_hash};

  // The append is durable now that COMMIT succeeded. Retention rotation is
  // best-effort maintenance (seal + prune of the oldest segment); a rotation
  // failure (SQLITE_BUSY on a WAL checkpoint, disk-full on the prune DELETE)
  // must NOT turn a committed append into a reported dropped write. Swallow it,
  // count it so the maintenance failure stays observable, and let the next
  // append retry the rotation.
  try {
    rotate_if_needed_locked();
  } catch (const std::exception &) {
    ++rotation_failures_;
  }
  return new_seq;
}

void FaultAuditLog::rotate_if_needed_locked() {
  if (retention_max_records_ <= 0) {
    return;
  }

  // Count retained rows and find the oldest surviving seq.
  int64_t count = 0;
  int64_t min_seq = 0;
  {
    Stmt stmt(db_.get(), "SELECT COUNT(*), COALESCE(MIN(seq), 0) FROM audit_log");
    if (stmt.step() == SQLITE_ROW) {
      count = stmt.column_int64(0);
      min_seq = stmt.column_int64(1);
    }
  }
  if (count <= retention_max_records_) {
    return;
  }

  // Prune the oldest (count - retention_max_records_) rows. The boundary row is
  // the highest seq being pruned; its hash becomes the sealed anchor that the
  // first surviving row's prev_hash links back to.
  const int64_t prune_count = count - retention_max_records_;
  const int64_t boundary_seq = min_seq + prune_count - 1;

  std::string boundary_hash;
  int64_t sealed_at_ns = 0;
  {
    Stmt stmt(db_.get(), "SELECT record_hash, occurred_at_ns FROM audit_log WHERE seq = ?");
    stmt.bind_int64(1, boundary_seq);
    if (stmt.step() != SQLITE_ROW) {
      // Should not happen given the count; leave the log intact rather than
      // prune without a valid anchor.
      return;
    }
    boundary_hash = stmt.column_text(0);
    sealed_at_ns = stmt.column_int64(1);
  }

  exec_or_throw(db_.get(), "BEGIN IMMEDIATE", "begin rotate");
  try {
    // Open the prune guard FIRST so every guarded write below - the anchor
    // INSERT, the sealed-prefix audit_log DELETE, and the old-anchor DELETE -
    // passes its guard-gated trigger. Both the flip and the writes are in this
    // transaction, so a ROLLBACK restores the guard to its closed state.
    exec_or_throw(db_.get(), "UPDATE audit_prune_guard SET enabled = 1 WHERE id = 1", "open prune guard");

    Stmt anchor(db_.get(),
                "INSERT INTO audit_anchors (last_seq, sealed_at_ns, last_hash) VALUES (?, ?, ?) "
                "ON CONFLICT(last_seq) DO NOTHING");
    anchor.bind_int64(1, boundary_seq);
    anchor.bind_int64(2, sealed_at_ns);
    anchor.bind_text(3, boundary_hash);
    if (anchor.step() != SQLITE_DONE) {
      throw std::runtime_error(std::string("audit: failed to write anchor: ") + sqlite3_errmsg(db_.get()));
    }

    Stmt del(db_.get(), "DELETE FROM audit_log WHERE seq <= ?");
    del.bind_int64(1, boundary_seq);
    if (del.step() != SQLITE_DONE) {
      throw std::runtime_error(std::string("audit: failed to prune records: ") + sqlite3_errmsg(db_.get()));
    }

    // Bound audit_anchors: verify() only ever needs the anchor at the current
    // prune boundary (the oldest retained row links to it, and an all-pruned log
    // links its head to it). Older anchors describe already-forgotten prefixes
    // and are never read again, so drop them - otherwise audit_anchors grows one
    // row per rotation without bound while audit_log stays at the cap.
    Stmt prune_anchors(db_.get(), "DELETE FROM audit_anchors WHERE last_seq < ?");
    prune_anchors.bind_int64(1, boundary_seq);
    if (prune_anchors.step() != SQLITE_DONE) {
      throw std::runtime_error(std::string("audit: failed to prune anchors: ") + sqlite3_errmsg(db_.get()));
    }

    exec_or_throw(db_.get(), "UPDATE audit_prune_guard SET enabled = 0 WHERE id = 1", "close prune guard");
    exec_or_throw(db_.get(), "COMMIT", "commit rotate");
  } catch (...) {
    exec_or_throw(db_.get(), "ROLLBACK", "rollback rotate");
    throw;
  }
}

std::vector<AuditRecord> FaultAuditLog::read(int64_t limit, int64_t after_seq) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::string sql =
      "SELECT seq, occurred_at_ns, fault_code, transition, severity, status, source_id, description, "
      "prev_hash, record_hash FROM audit_log WHERE seq > ? ORDER BY seq ASC";
  if (limit > 0) {
    sql += " LIMIT ?";
  }

  Stmt stmt(db_.get(), sql.c_str());
  stmt.bind_int64(1, after_seq);
  if (limit > 0) {
    stmt.bind_int64(2, limit);
  }

  std::vector<AuditRecord> result;
  while (stmt.step() == SQLITE_ROW) {
    AuditRecord rec;
    rec.seq = stmt.column_int64(0);
    rec.event.occurred_at_ns = stmt.column_int64(1);
    rec.event.fault_code = stmt.column_text(2);
    rec.event.transition = stmt.column_text(3);
    rec.event.severity = static_cast<uint8_t>(stmt.column_int64(4));
    rec.event.status = stmt.column_text(5);
    rec.event.source_id = stmt.column_text(6);
    rec.event.description = stmt.column_text(7);
    rec.prev_hash = stmt.column_text(8);
    rec.record_hash = stmt.column_text(9);
    result.push_back(std::move(rec));
  }
  return result;
}

ChainHead FaultAuditLog::head() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return head_;
}

int64_t FaultAuditLog::record_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  Stmt stmt(db_.get(), "SELECT COUNT(*) FROM audit_log");
  if (stmt.step() != SQLITE_ROW) {
    return 0;
  }
  return stmt.column_int64(0);
}

int64_t FaultAuditLog::rotation_failures() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rotation_failures_;
}

AuditVerifyResult FaultAuditLog::verify() const {
  std::lock_guard<std::mutex> lock(mutex_);

  AuditVerifyResult result;

  // Walk every retained row oldest-first, recomputing each link.
  Stmt stmt(db_.get(),
            "SELECT seq, occurred_at_ns, fault_code, transition, severity, status, source_id, description, "
            "prev_hash, record_hash FROM audit_log ORDER BY seq ASC");

  bool first = true;
  int64_t expected_seq = 0;
  std::string expected_prev;

  while (stmt.step() == SQLITE_ROW) {
    AuditRecord rec;
    rec.seq = stmt.column_int64(0);
    rec.event.occurred_at_ns = stmt.column_int64(1);
    rec.event.fault_code = stmt.column_text(2);
    rec.event.transition = stmt.column_text(3);
    rec.event.severity = static_cast<uint8_t>(stmt.column_int64(4));
    rec.event.status = stmt.column_text(5);
    rec.event.source_id = stmt.column_text(6);
    rec.event.description = stmt.column_text(7);
    rec.prev_hash = stmt.column_text(8);
    rec.record_hash = stmt.column_text(9);

    if (first) {
      first = false;
      // The first retained row must link to genesis (seq 1) or to a sealed
      // anchor whose last_seq == rec.seq - 1.
      if (rec.seq == 1) {
        if (rec.prev_hash != genesis_hash()) {
          result.ok = false;
          result.bad_seq = rec.seq;
          result.error = "first record does not link to genesis";
          return result;
        }
      } else {
        Stmt anchor(db_.get(), "SELECT last_hash FROM audit_anchors WHERE last_seq = ?");
        anchor.bind_int64(1, rec.seq - 1);
        if (anchor.step() != SQLITE_ROW) {
          result.ok = false;
          result.bad_seq = rec.seq;
          result.error = "no sealed anchor for the oldest retained record (history truncated)";
          return result;
        }
        const std::string anchor_hash = anchor.column_text(0);
        if (rec.prev_hash != anchor_hash) {
          result.ok = false;
          result.bad_seq = rec.seq;
          result.error = "oldest retained record does not link to its sealed anchor";
          return result;
        }
      }
      expected_seq = rec.seq;
      expected_prev = rec.prev_hash;
    } else {
      // Subsequent rows must be contiguous and chain to the previous record.
      if (rec.seq != expected_seq) {
        result.ok = false;
        result.bad_seq = rec.seq;
        result.error = "non-contiguous seq (record deleted or reordered)";
        return result;
      }
      if (rec.prev_hash != expected_prev) {
        result.ok = false;
        result.bad_seq = rec.seq;
        result.error = "prev_hash does not match previous record_hash (chain broken)";
        return result;
      }
    }

    const std::string recomputed = sha256_hex(rec.prev_hash + canonicalize(rec.seq, rec.event));
    if (recomputed != rec.record_hash) {
      result.ok = false;
      result.bad_seq = rec.seq;
      result.error = "record_hash mismatch (record tampered)";
      return result;
    }

    ++result.checked;
    expected_prev = rec.record_hash;
    expected_seq = rec.seq + 1;
  }

  // Read the persisted head row DIRECTLY from the DB (not the cached head_). A
  // missing head row cannot be silently recovered from MAX(seq), so deleting the
  // newest record together with the head row is reported as tampering instead of
  // verifying clean.
  const std::optional<ChainHead> persisted = read_head_row_locked();

  if (result.checked > 0) {
    // A non-empty log must carry its head row, and it must match the last record.
    if (!persisted) {
      result.ok = false;
      result.bad_seq = expected_seq - 1;
      result.error = "audit_chain_head row missing while audit_log is non-empty (head deleted / truncated)";
      return result;
    }
    if (persisted->seq != expected_seq - 1 || persisted->record_hash != expected_prev) {
      result.ok = false;
      result.bad_seq = persisted->seq;
      result.error = "persisted head does not match the last retained record";
      return result;
    }
  } else if (persisted && persisted->seq != 0) {
    // Empty retained log with a head past genesis: everything was pruned, so the
    // head must point at a sealed anchor. (No head row, or a genesis head, on an
    // empty log is the never-written case and is fine.)
    Stmt anchor(db_.get(), "SELECT last_hash FROM audit_anchors WHERE last_seq = ?");
    anchor.bind_int64(1, persisted->seq);
    if (anchor.step() != SQLITE_ROW || anchor.column_text(0) != persisted->record_hash) {
      result.ok = false;
      result.bad_seq = persisted->seq;
      result.error = "head references a record that is neither retained nor sealed";
      return result;
    }
  }

  return result;
}

}  // namespace ros2_medkit_fault_manager
