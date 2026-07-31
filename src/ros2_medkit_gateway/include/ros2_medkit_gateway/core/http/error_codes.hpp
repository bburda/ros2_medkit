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

#include <string>

namespace ros2_medkit_gateway {

/**
 * @brief SOVD Standard Error Codes
 *
 * These error codes follow the SOVD GenericError schema defined in
 * sovd-openapi-spec/commons/errors.yaml. They should be used in all
 * error responses to ensure compliance with the SOVD specification.
 */

/// Invalid request format or missing required parameters
constexpr const char * ERR_INVALID_REQUEST = "invalid-request";

/// Entity (component, app, area, function) not found
constexpr const char * ERR_ENTITY_NOT_FOUND = "entity-not-found";

/// Resource (operation, configuration, data, fault) not found
constexpr const char * ERR_RESOURCE_NOT_FOUND = "resource-not-found";

/// Operation not found on entity
constexpr const char * ERR_OPERATION_NOT_FOUND = "operation-not-found";

/// Invalid parameter value or type
constexpr const char * ERR_INVALID_PARAMETER = "invalid-parameter";

/// Service temporarily unavailable
constexpr const char * ERR_SERVICE_UNAVAILABLE = "service-unavailable";

/// Internal server error
constexpr const char * ERR_INTERNAL_ERROR = "internal-error";

/// Collection not supported on entity type
constexpr const char * ERR_COLLECTION_NOT_SUPPORTED = "collection-not-supported";

/// Payload too large (used for file upload size limits)
constexpr const char * ERR_PAYLOAD_TOO_LARGE = "payload-too-large";

/// Feature not implemented (SOVD 501 Not Implemented)
constexpr const char * ERR_NOT_IMPLEMENTED = "not-implemented";

/// Authentication required
constexpr const char * ERR_UNAUTHORIZED = "unauthorized";

/// Access denied / insufficient permissions
constexpr const char * ERR_FORBIDDEN = "forbidden";

/// Rate limit exceeded (429 Too Many Requests)
constexpr const char * ERR_RATE_LIMIT_EXCEEDED = "rate-limit-exceeded";

/// Generic vendor-specific error (used with vendor_code field)
constexpr const char * ERR_VENDOR_ERROR = "vendor-error";

/**
 * @brief ros2_medkit Vendor Error Codes (x-medkit-*)
 *
 * These are ros2_medkit-specific error codes that provide detailed
 * information about ROS 2-specific failures. When used, the error
 * response should include:
 * - error_code: "vendor-error"
 * - vendor_code: one of these x-medkit-* codes
 */

/// ROS 2 service call timed out or service unavailable
constexpr const char * ERR_X_MEDKIT_ROS2_SERVICE_UNAVAILABLE = "x-medkit-ros2-service-unavailable";

/// ROS 2 action goal was rejected
constexpr const char * ERR_X_MEDKIT_ROS2_ACTION_REJECTED = "x-medkit-ros2-action-rejected";

/// ROS 2 parameter is read-only and cannot be modified
constexpr const char * ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY = "x-medkit-ros2-parameter-read-only";

/// Dynamic type introspection failed for ROS 2 message
constexpr const char * ERR_X_MEDKIT_ROS2_TYPE_INTROSPECTION_FAILED = "x-medkit-ros2-type-introspection-failed";

/// ROS 2 node not available or not responding
constexpr const char * ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE = "x-medkit-ros2-node-unavailable";

/// ROS 2 topic not available
constexpr const char * ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE = "x-medkit-ros2-topic-unavailable";

/// ROS 2 action server not available
constexpr const char * ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE = "x-medkit-ros2-action-unavailable";

/// Gateway is shutting down; request cannot be served
constexpr const char * ERR_X_MEDKIT_GATEWAY_SHUTDOWN = "x-medkit-gateway-shutdown";

/// Subscription creation in Ros2SubscriptionExecutor failed (rcl-level error)
constexpr const char * ERR_X_MEDKIT_SUBSCRIBE_FAILED = "x-medkit-subscribe-failed";

/// Concurrent cold-wait pool saturation; retry with backoff
constexpr const char * ERR_X_MEDKIT_COLD_WAIT_CAP_EXCEEDED = "x-medkit-cold-wait-cap-exceeded";

/// Software update package not found
constexpr const char * ERR_X_MEDKIT_UPDATE_NOT_FOUND = "x-medkit-update-not-found";

/// Duplicate update package ID on registration
constexpr const char * ERR_X_MEDKIT_UPDATE_ALREADY_EXISTS = "x-medkit-update-already-exists";

/// Cannot modify/delete update while operation is in progress
constexpr const char * ERR_X_MEDKIT_UPDATE_IN_PROGRESS = "x-medkit-update-in-progress";

/// Execute called before prepare completed
constexpr const char * ERR_X_MEDKIT_UPDATE_NOT_PREPARED = "x-medkit-update-not-prepared";

/// Automated mode not supported for this package
constexpr const char * ERR_X_MEDKIT_UPDATE_NOT_AUTOMATED = "x-medkit-update-not-automated";

/// Vendor-specific: invalid resource URI format for subscriptions
constexpr const char * ERR_X_MEDKIT_INVALID_RESOURCE_URI = "x-medkit-invalid-resource-uri";

/// Vendor-specific: collection not supported for entity type
constexpr const char * ERR_X_MEDKIT_COLLECTION_NOT_SUPPORTED = "x-medkit-collection-not-supported";

/// Vendor-specific: no data provider registered for collection
constexpr const char * ERR_X_MEDKIT_COLLECTION_NOT_AVAILABLE = "x-medkit-collection-not-available";

/// Vendor-specific: resource URI entity doesn't match route entity
constexpr const char * ERR_X_MEDKIT_ENTITY_MISMATCH = "x-medkit-entity-mismatch";

/// Vendor-specific: unsupported subscription protocol
constexpr const char * ERR_X_MEDKIT_UNSUPPORTED_PROTOCOL = "x-medkit-unsupported-protocol";

/// SOVD standard: client's lock was broken by another client (409)
constexpr const char * ERR_LOCK_BROKEN = "lock-broken";

/// SOVD standard: caller lacks the required access rights for this operation (403)
constexpr const char * ERR_INSUFFICIENT_ACCESS_RIGHTS = "insufficient-access-rights";

/// SOVD standard: a required precondition was not fulfilled (409)
constexpr const char * ERR_PRECONDITION_NOT_FULFILLED = "precondition-not-fulfilled";

/// SOVD standard: entity queried but did not respond (504). SOVD reserves
/// 504 Gateway Timeout for "no response from the underlying entity in time"
/// - used when a ROS 2 round-trip times out and the outcome is therefore
/// UNKNOWN, as opposed to a definitive failure reported by the entity.
constexpr const char * ERR_NOT_RESPONDING = "not-responding";

// Script error codes (vendor-specific only; generic cases use ERR_RESOURCE_NOT_FOUND / ERR_INVALID_PARAMETER)
constexpr const char * ERR_SCRIPT_ALREADY_EXISTS = "x-medkit-script-already-exists";
constexpr const char * ERR_SCRIPT_MANAGED = "x-medkit-managed-script";
constexpr const char * ERR_SCRIPT_RUNNING = "x-medkit-script-running";
constexpr const char * ERR_SCRIPT_NOT_RUNNING = "x-medkit-script-not-running";
constexpr const char * ERR_SCRIPT_CONCURRENCY_LIMIT = "x-medkit-concurrency-limit";
constexpr const char * ERR_SCRIPT_FILE_TOO_LARGE = "x-medkit-script-too-large";

/// Plugin provider returned an error (used for DataProvider/OperationProvider/FaultProvider errors)
constexpr const char * ERR_PLUGIN_ERROR = "x-medkit-plugin-error";

/// Framework-internal sentinel signalling that a typed handler propagated a
/// validator's Forwarded outcome - the response has already been committed by
/// the peer-forwarding path and the typed wrapper must NOT render any body or
/// status. Never appears on the wire. Always paired with `http_status == 0`
/// so accidental rendering still produces a recognizable shape.
constexpr const char * ERR_X_INTERNAL_FORWARDED = "x-medkit-internal-forwarded";

/**
 * @brief Check if an error code is a vendor-specific code
 * @param error_code Error code to check
 * @return true if code starts with "x-medkit-"
 */
inline bool is_vendor_error_code(const std::string & error_code) {
  return error_code.rfind("x-medkit-", 0) == 0;
}

}  // namespace ros2_medkit_gateway
