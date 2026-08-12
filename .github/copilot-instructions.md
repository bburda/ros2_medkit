# Copilot Instructions for ros2_medkit

## Project Overview

**ros2_medkit** is a ROS 2 diagnostics gateway that exposes ROS 2 system information via a RESTful HTTP API aligned with the **SOVD (Service-Oriented Vehicle Diagnostics)** specification. It models robots as a diagnostic entity tree: **Area -> Component -> App**, with optional **Function** groupings.

**Tech Stack**: C++17, ROS 2 Jazzy/Humble/Lyrical, Ubuntu 26.04/24.04/22.04

## Package Structure

Multi-package colcon workspace under `src/`:

| Package | Purpose |
|---------|---------|
| `ros2_medkit_gateway` | HTTP gateway - REST server, discovery, entity management, handlers, plugin framework |
| `ros2_medkit_fault_manager` | Fault aggregation with SQLite, AUTOSAR DEM-style debounce, rosbag/snapshot capture |
| `ros2_medkit_fault_reporter` | Client library for nodes to report faults |
| `ros2_medkit_diagnostic_bridge` | Bridges `/diagnostics` topic to fault manager |
| `ros2_medkit_serialization` | Runtime JSON <-> ROS 2 message serialization via dynmsg |
| `ros2_medkit_msgs` | Custom service/message definitions |
| `ros2_medkit_integration_tests` | Python integration test framework + launch tests |
| `ros2_medkit_beacon_common` | Shared beacon validation, hint store, and entity mapper |
| `ros2_medkit_topic_beacon` | Push-based topic beacon discovery plugin |
| `ros2_medkit_param_beacon` | Pull-based parameter beacon discovery plugin |

## Architecture

The package is organised in two named layers under `src/ros2_medkit_gateway/`:

- **`include/.../core/`** + **`src/core/`** - middleware-neutral business logic. Compiles into the `gateway_core` STATIC library with no ROS dependencies. Hosts the HTTP routing, request handlers, JWT auth, fault model, peer aggregation, manifest parser, entity cache, and the neutral managers (lock, bulk-data, subscription, script, update, plugin).
- The remainder of `src/` - ROS adapter layer. Compiles into `gateway_ros2` (links `gateway_core` publicly). Hosts `gateway_node` (rclcpp::Node), the ROS-coupled managers (data access, operation, configuration, fault facade, log, trigger), `runtime_discovery`, `native_topic_sampler`, and ROS-specific provider implementations.

```
GatewayNode (src/main.cpp + src/gateway_node.cpp)             [gateway_ros2]
├── DiscoveryManager      - Entity discovery (runtime / manifest / hybrid)
│   └── EntityCache       - Thread-safe in-memory entity store         [core]
├── DataAccessManager     - Topic sampling and publishing
├── OperationManager      - Service calls and action goals
├── ConfigurationManager  - ROS 2 parameter CRUD
├── FaultManager          - Fault query/clear, SSE streams (delegates to fault_manager package)
├── LogManager            - /rosout ring buffer + plugin delegation
├── PluginManager         - Dynamic plugin loading (.so), provider interfaces  [core]
└── RESTServer            - cpp-httplib HTTP server (separate thread)          [core]
    └── HandlerContext    - Shared validation, entity lookup, error/JSON helpers
```

The `gateway_core_purity` linter (greps for ROS includes under `core/`) and the `test_gateway_core_smoke` link-time test (compiles a TU against `gateway_core` alone) enforce the contract that the neutral layer carries no ROS coupling.

### Entity Model (SOVD-aligned)

Defined in `include/ros2_medkit_gateway/models/`:

- **Area** - namespace grouping (`/powertrain`, `/chassis`, `root`)
- **Component** - groups Apps by namespace (synthetic in runtime mode, explicit in manifest)
- **App** - individual ROS 2 node
- **Function** - capability grouping (functional view, aggregates Apps)

Entity types: `SovdEntityType` enum in `entity_types.hpp`. Capability matrix in `entity_capabilities.hpp` (SOVD Table 8).

### SOVD Resource Collections

Each entity type supports specific collections:

| Collection | Server | Area | Component | App | Function |
|-----------|--------|------|-----------|-----|----------|
| configurations | x | | x | x | |
| data | x | | x | x | x (aggregated) |
| faults | x | | x | x | |
| operations | x | | x | x | x (aggregated) |
| logs | x | | x | x | |

## Code Style & Conventions

- **C++17** with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`
- **Formatting**: Google-based clang-format, 120 cols, 2-space indent, middle pointer alignment (`Type * ptr`)
- **Headers**: `#pragma once`, Apache 2.0 copyright (year 2026 for new files)
- **Namespace**: `medkit::` for code under `core/`, `medkit::` (via the `ros2_medkit_gateway = medkit` alias in `include/ros2_medkit_gateway/compat.hpp`) for everything else. Existing code referencing `ros2_medkit_gateway::` keeps compiling via the alias during the migration.
- **Error handling**: `tl::expected<T, std::string>` for fallible operations - NOT exceptions in handlers
- **Thread safety**: Always get fresh `EntityCache` via `ctx_.node()->get_thread_safe_cache()` (in handlers)
- **Header-only libs**: nlohmann::json, tl::expected, jwt-cpp, cpp-httplib

## Handler Pattern (follow for ALL new handlers)

Handlers live under `src/core/http/handlers/` (most groups - they are middleware-neutral) and `src/http/handlers/` (handlers that pull rclcpp transitively, currently `handler_context`, `fault_handlers`, `sse_fault_handler`, `cyclic_subscription_handlers`). One file per group. All handlers take `HandlerContext&` in constructor.

```cpp
void ExampleHandlers::handle_request(const httplib::Request & req, httplib::Response & res) {
  // 1. Extract entity ID from regex match
  auto entity_id = req.matches[1].str();

  // 2. Validate entity (sends error response automatically if invalid)
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) return;  // Response already sent (error or forwarded to peer)

  // 3. Check collection support (static method)
  if (auto err = HandlerContext::validate_collection_access(*entity, ResourceCollection::DATA)) {
    HandlerContext::send_error(res, 400, ERR_COLLECTION_NOT_SUPPORTED, *err);
    return;
  }

  // 4. Get fresh thread-safe cache
  const auto & cache = ctx_.node()->get_thread_safe_cache();

  // 5. Business logic (use tl::expected for errors)
  auto result = some_operation();
  if (!result) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, result.error());
    return;
  }

  // 6. Return JSON response
  HandlerContext::send_json(res, json{{"items", *result}});
}
```

### HandlerContext Key Methods

| Method | Returns | Purpose |
|--------|---------|---------|
| `validate_entity_for_route(req, res, id)` | `ValidateResult` (`tl::expected<EntityInfo, ValidationOutcome>`) | Unified entity validation; returns error or forward outcome on failure |
| `validate_collection_access(entity, collection)` | `optional<string>` | Check SOVD capability support |
| `send_error(res, status, code, msg, params)` | void | SOVD GenericError response |
| `send_json(res, data)` | void | JSON success response |
| `node()` | `GatewayNode*` | Access gateway subsystems |

### Error Codes

Defined in `include/ros2_medkit_gateway/http/error_codes.hpp`:

**SOVD standard**: `ERR_ENTITY_NOT_FOUND`, `ERR_INVALID_REQUEST`, `ERR_INVALID_PARAMETER`, `ERR_COLLECTION_NOT_SUPPORTED`, `ERR_RESOURCE_NOT_FOUND`, `ERR_OPERATION_NOT_FOUND`, `ERR_SERVICE_UNAVAILABLE`, `ERR_INTERNAL_ERROR`, `ERR_NOT_IMPLEMENTED`, `ERR_UNAUTHORIZED`, `ERR_FORBIDDEN`

**Vendor-specific** (`x-medkit-` prefix): `ERR_X_MEDKIT_ROS2_SERVICE_UNAVAILABLE`, `ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE`, `ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE`, etc.

Error response schema:
```json
{
  "error_code": "entity-not-found",
  "message": "Component not found",
  "parameters": {"entity_id": "my_component"}
}
```

### Route Registration

Routes in `rest_server.cpp::setup_routes()`. Dual-path pattern - apps and components share handlers:

```cpp
srv->Get((api_path("/apps") + R"(/([^/]+)/data$)"),
         [this](auto & req, auto & res) { data_handlers_->handle_list_data(req, res); });
srv->Get((api_path("/components") + R"(/([^/]+)/data$)"),
         [this](auto & req, auto & res) { data_handlers_->handle_list_data(req, res); });
```

Handler classes: `HealthHandlers`, `DiscoveryHandlers`, `DataHandlers`, `OperationHandlers`, `ConfigHandlers`, `FaultHandlers`, `LogHandlers`, `AuthHandlers`, `BulkDataHandlers`, `CyclicSubscriptionHandlers`, `UpdateHandlers`

## Build System

**Static library pattern**: two layered targets - `gateway_core` (sources under `src/core/`, no ROS dependencies, links cpp-httplib + nlohmann/json + yaml-cpp + tl::expected + jwt-cpp + OpenSSL + SQLite + dl) and `gateway_ros2` (the remaining ROS-adapter sources, links `gateway_core` publicly via `medkit_target_dependencies()`). The `gateway_node` executable and existing test targets link `gateway_ros2` so they transitively get both layers. The link-only smoke test `test_gateway_core_smoke` deliberately links `gateway_core` plus GTest with no `ament_target_dependencies` to enforce the neutral contract.

**CMake modules** in `cmake/`:
- `ROS2MedkitCcache.cmake` - auto-detect ccache
- `ROS2MedkitCompat.cmake` - multi-distro compatibility (`medkit_find_yaml_cpp()`, `medkit_find_cpp_httplib()`, `medkit_target_dependencies()`)
- `ROS2MedkitLinting.cmake` - centralized clang-tidy config

New packages must include all three and use `medkit_*` helpers for dependencies.

```bash
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
```

## Testing

### Unit Tests (C++ GTest)

Location: `src/ros2_medkit_gateway/test/test_*.cpp`

```cpp
// @verifies REQ_INTEROP_061
TEST_F(LogHandlersTest, GetLogsReturnsBadRequestWhenMatchesMissing) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_logs(req, res);
  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ERR_INVALID_REQUEST);
}
```

**ROS_DOMAIN_ID isolation**: GTest, gmock, pytest and launch tests that create `rclcpp::Node` get a domain automatically when registered through `medkit_add_gtest()` / `medkit_add_gmock()` / `medkit_add_pytest_test()` / `medkit_add_launch_test()` in `ROS2MedkitTestDomain.cmake` - a wrapper takes a free domain from a shared band at test start and holds it for exactly as long as the test runs. There is no hand-maintained ID table to update. A test that creates no ROS node calls `medkit_test_needs_no_domain()` instead.

### Integration Tests (Python launch_testing)

Location: `src/ros2_medkit_integration_tests/test/features/*.test.py`

Base class: `GatewayTestCase` in `ros2_medkit_test_utils/gateway_test_case.py`

```python
class TestLoggingApi(GatewayTestCase):
    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    # @verifies REQ_INTEROP_061
    def test_app_get_logs_returns_200(self):
        data = self.get_json('/apps/temp_sensor/logs')
        self.assertIn('items', data)
```

Demo nodes in `test/demo_nodes/`. Registry in `launch_helpers.py` maps short keys to executables.

### Requirements Traceability

Tag tests with `// @verifies REQ_XXX` (C++) or `# @verifies REQ_XXX` (Python). Place BEFORE the test definition or inside body.

## Plugin Framework

Plugins provide custom backends via provider interfaces:

| Provider | Interface | Purpose |
|----------|-----------|---------|
| `DataProvider` | `core/providers/data_provider.hpp` | Per-entity data resource backend |
| `OperationProvider` | `core/providers/operation_provider.hpp` | Per-entity operation backend |
| `FaultProvider` | `core/providers/fault_provider.hpp` | Per-entity fault backend |
| `LogProvider` | `core/providers/log_provider.hpp` | Custom log backends |
| `UpdateProvider` | `core/providers/update_provider.hpp` | Software update management |
| `ScriptProvider` | `core/providers/script_provider.hpp` | Custom script execution backends |
| `IntrospectionProvider` | `core/providers/introspection_provider.hpp` | Custom entity introspection |

Plugins loaded as `.so` files with `extern "C"` factory. Provider interfaces return typed C++ structs (not JSON) - manager layer handles serialization. Plugin failures are caught and isolated.

## Key Dependencies

- **cpp-httplib**: HTTP server (header-only, found via pkg-config or cmake)
- **nlohmann_json**: JSON serialization (header-only)
- **yaml-cpp**: Configuration parsing
- **tl::expected**: Error handling (header-only, vendored)
- **jwt-cpp**: JWT authentication (header-only, vendored)
- **OpenSSL**: TLS support

## Code Review Rules

When reviewing pull requests, apply these rules to the diff. Flag violations as review comments with a brief explanation of the correct pattern.

### Thread Safety

- **Flag** any code that stores the result of `get_thread_safe_cache()` in a member variable or passes it across function boundaries. The cache must be obtained fresh before each use within the current scope.
- **Flag** mutable shared state accessed without a mutex or atomic.
- **Flag** blocking operations (sleep, long loops, synchronous I/O) inside ROS 2 callbacks.

### Error Handling

- **Flag** `throw` statements in handler or manager code. Use `tl::expected<T, std::string>` and return `tl::make_unexpected(msg)` instead.
- **Flag** dereferencing a `tl::expected` or `std::optional` result without checking `if (!result)` first.
- **Flag** error responses that don't use `HandlerContext::send_error()` or use string literals instead of constants from `error_codes.hpp`.
- **Flag** custom vendor error codes that don't start with `x-medkit-` prefix.
- **Flag** any code after `validate_entity_for_route()` that doesn't check for failure and return early - the response is already committed by the validator (either error sent or request forwarded to peer).

### Handler Pattern

- **Flag** handlers that manually look up entities (e.g., calling `cache.get_component()` directly) instead of using `ctx_.validate_entity_for_route(req, res, entity_id)`.
- **Flag** handlers that access entity collections without calling `HandlerContext::validate_collection_access()` first.
- **Flag** handlers that call `ctx_.node()->get_thread_safe_cache()` at class construction or store the cache - it must be called per request.
- **Flag** new routes in `rest_server.cpp` that register only for `/apps/` or only for `/components/` when the endpoint applies to both entity types (dual-path pattern).
- **Flag** calling `res.set_header("Content-Type", ...)` after `set_chunked_content_provider()` - the content provider already sets the header, duplicating it breaks responses.

### SOVD Compliance

- **Flag** capability lists that don't match the entity type per Table 8. Use `EntityCapabilities::for_type()` to verify.
- **Flag** entity responses missing HATEOAS `_links` (use `LinksBuilder`) or capability arrays (use `CapabilityBuilder`).
- **Flag** error responses that don't follow the GenericError schema (`error_code`, `message`, optional `parameters`).

### Testing

Every code change must include corresponding tests. A feature without tests is not complete.

**Unit tests (C++ GTest)** - required for all new logic:
- **Flag** new or modified files in `src/core/http/handlers/` or `src/http/handlers/` without a corresponding `test/test_*_handlers.cpp` in the diff. Handler unit tests verify request validation, error responses, and edge cases without ROS 2 runtime.
- **Flag** new or modified manager classes (`src/core/managers/*.cpp` or `src/*.cpp`) without corresponding `test/test_*.cpp`. Manager tests verify business logic in isolation.
- **Flag** new utility functions or static methods without unit tests covering their behavior.

**Integration tests (Python launch_testing)** - required for all endpoint changes:
- **Flag** new or modified routes in `rest_server.cpp` without a corresponding integration test in `src/ros2_medkit_integration_tests/test/features/`. Integration tests verify end-to-end REST API behavior through the running gateway with demo nodes.
- **Flag** integration test classes that don't extend `GatewayTestCase` or don't set `REQUIRED_APPS`/`MIN_EXPECTED_APPS` for discovery waiting.
- **Flag** new endpoint tests that only check the happy path (200 OK). Tests should also verify error cases (404 for missing entity, 400 for invalid parameters).

**General test rules:**
- **Flag** any use of `GTEST_SKIP()`, `@pytest.mark.skip`, `unittest.skip`, or equivalent - tests must not be skipped, fix the code or the test instead.
- **Flag** GTest/launch test files that create `rclcpp::Node` without registering through `medkit_add_gtest()` / `medkit_add_gmock()` / `medkit_add_launch_test()` in `ROS2MedkitTestDomain.cmake`, which allocate a `ROS_DOMAIN_ID` automatically - there is no hand-maintained ID table to update, and for a launch test a manual `set_tests_properties()` afterward fails the configure outright when the guard in that macro has not registered the test.
- **Flag** integration tests that select `/parameter_events` or `/rosout` topics - these are ROS 2 system topics without continuous data flow.
- **Flag** test files for SOVD-spec endpoints that are missing `// @verifies REQ_XXX` traceability tags.

### Build & CMake

- **Flag** new `.cpp` source files under `src/core/` that aren't picked up by `gateway_core`'s `GLOB_RECURSE` source list, or new ROS-adapter `.cpp` files that aren't added to the `gateway_ros2` STATIC library target in CMakeLists.txt.
- **Flag** any `#include <rclcpp/...>` or message-package include in a file under `core/` - the layer must stay middleware-neutral. The `gateway_core_purity` linter and `test_gateway_core_smoke` link test enforce this; do not paper over their failures.
- **Flag** new test targets that don't link to `gateway_ros2` (the default), unless the test is intentionally a core-only smoke test linking `gateway_core` directly.
- **Flag** use of `ament_target_dependencies()` - use `medkit_target_dependencies()` instead (deprecated in Kilted, removed in ament_cmake 2.8.5+ which Lyrical ships).
- **Flag** direct `find_package(yaml-cpp)` or `find_package(httplib)` - use `medkit_find_yaml_cpp()` / `medkit_find_cpp_httplib()` for multi-distro compatibility.

### Documentation

- **Flag** PRs that add or modify REST endpoints but don't include changes to `docs/api/`.
- **Flag** PRs that change architecture or public interfaces without updating design docs in `docs/design/` or `src/ros2_medkit_gateway/design/`.
- **Flag** new files missing Apache 2.0 copyright header (year 2026) or `#pragma once` (headers).

### Anti-Patterns Quick Reference

| If you see this... | Flag it and suggest... |
|---|---|
| `auto & cache = node_->get_thread_safe_cache();` stored as member | Call `get_thread_safe_cache()` fresh per request |
| `throw std::runtime_error(...)` in handler/manager | Return `tl::make_unexpected("message")` |
| `cache.get_component(id)` in handler without validation | Use `ctx_.validate_entity_for_route(req, res, id)` |
| `ament_target_dependencies(target ...)` | Use `medkit_target_dependencies(target ...)` |
| `find_package(yaml-cpp REQUIRED)` | Use `medkit_find_yaml_cpp()` |
| `res.set_header("Content-Type", ...)` + `set_chunked_content_provider(...)` | Remove `set_header` - provider sets Content-Type |
| `GTEST_SKIP()` or `@pytest.mark.skip` | Fix the test or the code, never skip |
| `set_tests_properties(... ENVIRONMENT "ROS_DOMAIN_ID=99")` by hand | Use `medkit_add_gtest()` / `medkit_add_gmock()` / `medkit_add_launch_test()`, which allocate the domain |
| New endpoint route for `/apps/` only | Add matching route for `/components/` (dual-path) |
| Error string literal like `"not-found"` | Use constant `ERR_ENTITY_NOT_FOUND` from `error_codes.hpp` |
| `res.status = 404; res.set_content(...)` manually | Use `HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, msg)` |

## Documentation

Sphinx-based docs in `docs/`. Published to GitHub Pages.

- `docs/design/` - Architecture and design decisions (PlantUML, RST)
- `docs/requirements/` - SOVD specs and project requirements (sphinx-needs)
- `docs/api/` - REST API endpoint documentation
- `src/ros2_medkit_gateway/design/` - Package-level design docs
- `src/ros2_medkit_gateway/README.md` - Package overview and usage
- `postman/` - API collections for manual testing

**Documentation is part of every change, not a follow-up task.** User-facing docs (API reference, design docs, README) must be updated in the same PR as the code change. A feature is not complete until its documentation is shipped.

When adding or modifying endpoints, features, or architectural components, update:
1. REST API docs in `docs/api/` (endpoint behavior, request/response schemas)
2. Design docs in `docs/design/` or `src/ros2_medkit_gateway/design/` if architecture changes
3. Package README if public interface changes
4. Postman collections if API surface changes
5. Requirements traceability - link tests to specs via `// @verifies REQ_XXX`

## Discovery Modes

Configured in `gateway_params.yaml` under `discovery.mode`:

- **runtime_only** (default) - ROS 2 graph introspection, synthetic components by namespace
- **manifest_only** - YAML manifest entities only
- **hybrid** - Manifest as source of truth + runtime linking
