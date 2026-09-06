// Copyright 2025 bburda
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
 * @file handlers.hpp
 * @brief Convenience header that includes all HTTP handler classes.
 *
 * Include this file to get access to all REST API handlers.
 */

// Discovery handlers (unified for all entity types)
#include "ros2_medkit_gateway/core/http/handlers/discovery_handlers.hpp"

// Resource handlers (unified across entity types)
#include "ros2_medkit_gateway/core/http/handlers/data_handlers.hpp"

// Other handlers
#include "ros2_medkit_gateway/core/http/handlers/auth_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/config_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/docs_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/health_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/lifecycle_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/lock_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/log_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/operation_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/script_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/trigger_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/update_handlers.hpp"
#include "ros2_medkit_gateway/http/handlers/cyclic_subscription_handlers.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/handlers/planned_stop_handlers.hpp"
#include "ros2_medkit_gateway/http/handlers/sse_fault_handler.hpp"
