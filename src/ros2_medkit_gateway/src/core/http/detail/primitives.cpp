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

#include "ros2_medkit_gateway/http/detail/primitives.hpp"

#include <algorithm>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"

namespace ros2_medkit_gateway {
namespace http {
namespace detail {

namespace {

// SOVD GenericError responses must carry an HTTP status in the 400-599 range.
// We clamp out-of-range values defensively so misconfigured providers cannot
// surface 200-class success codes or sub-400 informational codes as errors.
int clamp_error_status(int status) {
  return std::clamp(status, 400, 599);
}

constexpr const char * kContentTypeJson = "application/json";

}  // namespace

void write_json_body(FrameworkOrPluginAccess /*token*/, httplib::Response & res, const nlohmann::json & body,
                     int status) {
  // Sentinel: status == 0 means "leave res.status untouched". The remaining
  // raw-route callers (PluginResponse, DocsHandlers, SSEFaultHandler) rely
  // on this so they can pre-set res.status (e.g. 201 Created) before
  // calling the writer.
  if (status != 0) {
    res.status = status;
  }
  res.set_content(body.dump(2), kContentTypeJson);
}

void write_json_text(FrameworkOrPluginAccess /*token*/, httplib::Response & res, const std::string & body, int status) {
  // Same sentinel contract as write_json_body above.
  if (status != 0) {
    res.status = status;
  }
  res.set_content(body, kContentTypeJson);
}

void write_generic_error(FrameworkOrPluginAccess /*token*/, httplib::Response & res, const ErrorInfo & err) {
  res.status = clamp_error_status(err.http_status);

  nlohmann::json error_json;
  // Vendor-specific x-medkit-* codes are remapped to the SOVD vendor-error
  // envelope so generic clients see a known top-level code while still
  // getting the precise vendor code in a side field.
  if (is_vendor_error_code(err.code)) {
    error_json["error_code"] = ERR_VENDOR_ERROR;
    error_json["vendor_code"] = err.code;
  } else {
    error_json["error_code"] = err.code;
  }
  error_json["message"] = err.message;

  // SOVD GenericError schema (7.4.2) requires additional info in 'parameters'
  // field. Skip the key entirely when nothing was supplied to keep the wire
  // shape minimal.
  if (!err.params.is_null() && !err.params.empty()) {
    error_json["parameters"] = err.params;
  }

  res.set_content(error_json.dump(2), kContentTypeJson);
}

void write_oauth2_error(FrameworkOrPluginAccess /*token*/, httplib::Response & res, const ErrorInfo & err) {
  res.status = clamp_error_status(err.http_status);

  // RFC 6749 §5.2: OAuth 2.0 error responses use top-level `error` (snake_case
  // code) and `error_description` fields. There is no `parameters` wrapper and
  // no SOVD-style remapping of vendor codes - the auth endpoints speak OAuth2
  // wire format, period.
  nlohmann::json error_json;
  error_json["error"] = err.code;
  error_json["error_description"] = err.message;

  res.set_content(error_json.dump(2), kContentTypeJson);
}

}  // namespace detail
}  // namespace http
}  // namespace ros2_medkit_gateway
