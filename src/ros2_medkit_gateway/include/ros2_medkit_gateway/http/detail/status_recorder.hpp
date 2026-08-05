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

/**
 * @file
 * @brief Emitted-status recorder: observes what the gateway actually puts on
 *        the wire so an integration test can assert the OpenAPI document
 *        declares it.
 *
 * Every other check on the document compares it against a list somebody typed
 * - and every such list can rot silently. This is the one mechanism that
 * notices `declared` falling behind `observed` without anyone maintaining a
 * list: the recorder reports pairs, the test asserts the superset.
 *
 * **Test builds only.** The whole file is behind `MEDKIT_STATUS_RECORDER`,
 * which `ros2_medkit_gateway/CMakeLists.txt` defines exactly when
 * `BUILD_TESTING` is on. A shipped gateway - the Docker image builds with
 * `-DBUILD_TESTING=OFF` - compiles none of it, and `make_error()` is
 * byte-identical to what it was.
 *
 * ### What it observes
 *
 * - The status that reached `httplib::Response` for every route the
 *   `RouteRegistry` mounts, attributed to that route's OpenAPI *templated*
 *   path (`/apps/{app_id}/data/{data_id}`), not the concrete request path.
 *   This is the authoritative observation: it is the status the client
 *   receives, whether or not a `make_error()` produced it.
 * - The `file:line` of every `make_error()` call that ran, which is how a
 *   reader can tell how much of the ~281-site error surface a given run
 *   exercised rather than assuming a recorder that saw something saw
 *   everything.
 *
 * ### Why there is no ambient thread-local here
 *
 * The route identity is a member of `StatusRecordingScope`, the same object
 * that reads the final status, so nothing has to travel out-of-band.
 * `make_error()`'s hook is deliberately route-agnostic: it contributes to a
 * site set, not to the `(route, status)` set the assertion reads. That leaves
 * `make_error()` - an `inline` header function whose out-of-line copy lands in
 * `gateway_ros2`, which is linked into six MODULE targets - with no
 * thread-local access at all.
 *
 * Route-attributing the sites *would* need an ambient carrier, and that
 * carrier would have to be a namespace-scope `extern thread_local` (the
 * `tl_forward_response` pattern in `forward_response_scope.hpp`), never a
 * function-local `static thread_local`, which compiles to initial-exec TLS a
 * shared object cannot relocate. Not needing the carrier is the stronger
 * position, and the wire-status set is strictly more accurate than
 * route-attributed construction sites would be (it sees the status the client
 * receives, including one no `make_error()` built), so it is not planned.
 *
 * ### What it structurally cannot see
 *
 * - Anything answered **before** routing: the rate limiter's 429
 *   (`rate_limiter.cpp` writes `res.status` directly), the auth middleware's
 *   401/403, the CORS reject's 403, the OPTIONS pre-flight 204.
 * - Anything cpp-httplib answers by itself: 404/405 for an unrouted request,
 *   413 for a payload over `set_payload_max_length`, 400 for a malformed
 *   request line, 416 for an **unparseable** `Range` (an unsatisfiable but
 *   parseable one yields 206, not 416).
 * - Routes registered straight onto the server rather than through the
 *   registry: the Swagger UI subtree, this endpoint, and every route a
 *   plugin mounts through `PluginManager::register_routes`. The last of
 *   those *is* in the served document - the gateway folds each plugin's
 *   `describe_plugin_routes()` output into it - so it marks them
 *   `x-medkit-plugin-served`, which is how
 *   `test_openapi_error_coverage` tells "unreachable by this recorder" from
 *   "unreached, and that is a defect". The two `/docs` routes used to be in
 *   this list and no longer are: they go through the registry.
 * - Any status on a code path the test run never drives.
 *
 * Those are declared by hand; the recorder's own output is what says which.
 */

#ifdef MEDKIT_STATUS_RECORDER

#include <httplib.h>

#include <string>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {
namespace http {
namespace detail {

/// Record that a `make_error()` at *file*:*line* built an error carrying
/// *status*. Called from `make_error()` itself, so it runs on request threads
/// and on background threads alike; the site set is deliberately not
/// route-attributed (see the file comment).
void record_error_site(int status, const char * file, int line);

/// Record that *status* reached the wire for the route mounted at *method*
/// *path*, where *path* is the OpenAPI templated form.
void record_emitted_status(const std::string & method, const std::string & path, int status);

/// Snapshot of everything recorded so far:
/// `{"emitted": [{"method","path","status"}...], "error_sites": ["f:12 -> 404"...]}`.
nlohmann::json emitted_status_report();

/**
 * @brief RAII scope installed around every registry-mounted handler.
 *
 * On destruction it records the status the handler left on the response. The
 * `-1` normalisation mirrors cpp-httplib's own default (`httplib.h`: a handler
 * that sets no status yields 200, or 206 when the request carried a `Range`),
 * so a streaming or download handler that never touches `res.status` is
 * recorded as what the client will actually receive.
 *
 * Non-copyable, non-movable: it holds references for the duration of one
 * handler call and must not outlive or escape it.
 */
class StatusRecordingScope {
 public:
  StatusRecordingScope(const std::string & method, const std::string & path, const httplib::Request & req,
                       const httplib::Response & res)
    : method_(method), path_(path), req_(req), res_(res) {
  }

  ~StatusRecordingScope() {
    const int status = res_.status > 0 ? res_.status : (req_.ranges.empty() ? 200 : 206);
    record_emitted_status(method_, path_, status);
  }

  StatusRecordingScope(const StatusRecordingScope &) = delete;
  StatusRecordingScope & operator=(const StatusRecordingScope &) = delete;
  StatusRecordingScope(StatusRecordingScope &&) = delete;
  StatusRecordingScope & operator=(StatusRecordingScope &&) = delete;

 private:
  const std::string & method_;
  const std::string & path_;
  const httplib::Request & req_;
  const httplib::Response & res_;
};

}  // namespace detail
}  // namespace http
}  // namespace ros2_medkit_gateway

#endif  // MEDKIT_STATUS_RECORDER
