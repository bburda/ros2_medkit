# Quality Declaration for ros2_medkit

This document is a declaration of software quality for the `ros2_medkit` packages,
based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

## Quality Level 3

This repository (`ros2_medkit`, 6 packages) claims to be in the **Quality Level 3** category.

The packages covered by this declaration:

| Package | Description |
|---|---|
| `ros2_medkit_fault_manager` | Central fault management node with storage, correlation, and rosbag capture |
| `ros2_medkit_gateway` | REST API gateway (SOVD-compatible) with SSE, auth, and discovery |
| `ros2_medkit_msgs` | Custom message and service definitions |
| `ros2_medkit_fault_reporter` | Client library for fault reporting |
| `ros2_medkit_serialization` | Runtime JSON serialization for ROS 2 messages |
| `ros2_medkit_diagnostic_bridge` | Bridge node converting `/diagnostics` to medkit faults |

Below is the rationale, organized by each requirement listed in REP-2004.

---

## Version Policy [1]

### Version Scheme [1.i]

`ros2_medkit` uses [Semantic Versioning 2.0.0](https://semver.org/) as recommended by the
[ROS 2 Developer Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide-Build-Debs.html).

All packages in the repository share the same version number.

### Version Stability [1.ii]

The current version is **0.7.0**.
The package follows semver; the pre-1.0 version reflects that the public API may still evolve
based on early adopter feedback, not a lack of quality infrastructure.
The 1.0.0 release is planned after the API has been validated through pilot deployments.

### Public API Declaration [1.iii]

Public API headers are organized in `include/` directories for each package:

- `ros2_medkit_fault_reporter/include/` — `fault_reporter.hpp`, `local_filter.hpp`
- `ros2_medkit_serialization/include/` — `json_serializer.hpp`, `type_cache.hpp`
- `ros2_medkit_fault_manager/include/` — `fault_manager_node.hpp`, `fault_storage.hpp`
- `ros2_medkit_gateway/include/` — `gateway_node.hpp`, `data_access_manager.hpp`, `operation_manager.hpp`

The REST API is documented at [docs/api/rest.rst](docs/api/rest.rst) and follows the
[SOVD standard](https://www.asam.net/standards/detail/sovd/) for endpoint structure.

ROS 2 message and service definitions in `ros2_medkit_msgs` constitute a stable public interface.

### API Stability Within a Released ROS Distribution [1.iv]

API stability is a goal but not yet formally guaranteed (pre-1.0.0).
Breaking changes, if any, are documented in package-level `CHANGELOG.rst` files.
The REST API is versioned (`/api/v1/`) and follows the SOVD standard, providing de facto stability.

### ABI Stability Within a Released ROS Distribution [1.vi]

ABI stability is not guaranteed at this time. This is not required for Quality Level 3.

---

## Change Control Process [2]

### All Changes via Change Request [2.i]

All changes to `ros2_medkit` occur through pull requests on
[GitHub](https://github.com/selfpatch/ros2_medkit).
The `main` branch is protected; direct pushes are not allowed.

### Contributor Origin [2.ii]

Contributor origin is tracked via Git commit history and GitHub accounts.
Contributions are accepted under the repository's Apache-2.0 license.

### Peer Review Policy [2.iii]

All pull requests require at least one approving review before merge.
GitHub Copilot code review is used in addition to human review.

### Continuous Integration [2.iv]

Every pull request runs the following, and a failure in any of them blocks
review:

- **Build & Test:** build + unit/integration tests on Ubuntu Noble / ROS 2 Jazzy, Ubuntu Jammy / ROS 2 Humble, and Ubuntu Resolute / ROS 2 Lyrical. `ros2_medkit_opcua` is excluded here and covered by its own workflow instead
- **Quality:** `clang-format` and the other ament linters, the static coverage-scope check, which fails if a package compiles production C++ without opting into coverage instrumentation, and two source gates (`check_no_naked_subscriptions.sh`, `check_handlers_typed_query.sh`). Jazzy only
- **clang-tidy:** on a pull request this is incremental - it analyses the `.cpp` and `.hpp` files the branch changed, and does not run at all when the branch changes none. The full sweep over the compilation database runs on pushes to `main`
- **Sanitizers:** the unit and integration suites again under ASan + UBSan and under TSan, on Jazzy. Only packages that include the shared sanitizer module are instrumented; `ros2_medkit_fault_detection`, `ros2_medkit_sovd_service_interface` and the integration-test fixture executables are not
- **Documentation:** Sphinx build with `-W` plus linkcheck, on any pull request touching `docs/`, `src/` or `scripts/`
- **OPC-UA plugin:** the OpenPLC-based integration suite, on any pull request touching the plugin, the gateway plugin/provider headers or `ros2_medkit_msgs`

Reported on pushes to `main`, not run on pull requests:

- **Coverage:** Debug build with coverage instrumentation. Reports are archived as artifacts and uploaded to [Codecov](https://codecov.io/gh/selfpatch/ros2_medkit)

Merging additionally requires one approving review. Branch rules enforce the
review requirement and a linear history; the checks above are not configured as
required status checks, so passing them is a review expectation rather than a
mechanical block.

CI configuration: [`.github/workflows/ci.yml`](.github/workflows/ci.yml) and [`.github/workflows/quality.yml`](.github/workflows/quality.yml)

### Documentation Policy [2.v]

Documentation is built and deployed via a separate
[docs workflow](.github/workflows/docs.yml) using Sphinx + Doxygen (Breathe).

---

## Documentation [3]

### Feature Documentation [3.i]

Features are documented in the [online documentation](https://selfpatch.github.io/ros2_medkit/)
and the [README.md](README.md).

### Public API Documentation [3.ii]

- REST API: [docs/api/rest.rst](docs/api/rest.rst) (700+ lines, all endpoints documented)
- ROS 2 messages/services: [docs/api/messages.rst](docs/api/messages.rst)
- C++ API: [docs/api/cpp.rst](docs/api/cpp.rst) (Doxygen-generated via Breathe)

### License [3.iii]

The license for `ros2_medkit` is **Apache License 2.0**.
See the [LICENSE](LICENSE) file. Each source file contains the license header.

### Copyright Statements [3.iv]

Copyright is held by selfpatch.ai (Bartosz Burda, Michal Faferek).
Copyright statements are included in all source files.

### Quality Declaration [3.v]

This is the quality declaration document (you are reading it).

---

## Testing [4]

### Feature Testing [4.i]

The test suite comprises:

| Category | Count | Lines of Test Code |
|---|---|---|
| C++ unit tests (GTest) | 52 | ~14,000 |
| Python integration tests (launch_testing) | 13 | ~3,000 |

Tests cover: fault management, storage backends (SQLite + in-memory), snapshot capture,
rosbag capture, correlation engine, REST API endpoints, authentication, discovery,
configuration management, JSON serialization, and diagnostic bridge.

### Public API Testing [4.ii]

All REST API endpoints are covered by integration tests.
All ROS 2 service interfaces are covered by unit tests.

### Coverage [4.iii]

Current line coverage: **75%** (as reported by [Codecov](https://codecov.io/gh/selfpatch/ros2_medkit)).

Coverage is generated in CI and uploaded to Codecov for `main` branch builds, with a 1% regression threshold configured in `codecov.yml`.
Test directories, demo nodes, and build artifacts are excluded from coverage measurement.

### Performance Testing [4.iv]

No dedicated performance regression tests at this time.

### Linters and Static Analysis [4.v]

All packages use:

- `ament_cmake_clang_format` — code formatting
- `ament_cmake_clang_tidy` — static analysis
- `ament_lint_auto` + `ament_lint_common` — standard ROS 2 linting

Linter tests are enforced in CI on every pull request.

---

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

| Dependency | Quality Level |
|---|---|
| `rclcpp` | Level 1 |
| `builtin_interfaces` | Level 1 |
| `diagnostic_msgs` | Level 1 |
| `rosbag2_cpp` | Level 3 |
| `rosbag2_storage` | Level 3 |
| `rosbag2_storage_mcap` | Level 3 |
| `rosbag2_storage_default_plugins` | Level 3 |

### Optional Direct Runtime Non-ROS Dependencies [5.ii]

| Dependency | Description |
|---|---|
| `sqlite3` | Fault storage backend |
| `nlohmann-json` | JSON parsing |
| `cpp-httplib` | HTTP server (header-only) |
| `libssl` | TLS support |
| `jwt-cpp` | JWT authentication (vendored, header-only) |

---

## Platform Support [6]

### Target Platforms [6.i]

`ros2_medkit` is tested and supported on:

- **Ubuntu 24.04 (Noble)** with **ROS 2 Jazzy** (primary)
- **Ubuntu 22.04 (Jammy)** with **ROS 2 Humble**
- **Ubuntu 26.04 (Resolute)** with **ROS 2 Lyrical** (newest LTS, released May 2026)

Jazzy and Humble are Tier 1 LTS platforms per [REP-2000](https://www.ros.org/reps/rep-2000.html); Lyrical is the newest LTS (May 2026) and is also tested in CI.

---

## Security [7]

### Vulnerability Disclosure Policy [7.i]

The project follows the ROS 2 vulnerability disclosure policy as described in
[REP-2006](https://www.ros.org/reps/rep-2006.html).

Security issues can be reported via GitHub Security Advisories on the
[ros2_medkit repository](https://github.com/selfpatch/ros2_medkit/security).

---

## Current Status Summary

| Requirement | Status | Notes |
|---|---|---|
| Version policy | Met | Semver, all packages at 0.7.0 |
| Stable version (>=1.0.0) | Caveat | Pre-1.0; API versioned, 1.0.0 planned post-pilot |
| Change requests | Met | All changes via PR |
| CI | Met | Build + test + coverage on every PR |
| License | Met | Apache-2.0 |
| Copyright | Met | All source files have headers |
| Feature tests | Met | 65 tests across unit + integration |
| Coverage | Met | 75% line coverage |
| Linting | Met | clang-format, clang-tidy, ament_lint |
| Platform support | Met | Ubuntu Noble / ROS 2 Jazzy + Ubuntu Jammy / ROS 2 Humble + Ubuntu Resolute / ROS 2 Lyrical |
| Security policy | Met | REP-2006 compliant |

**Caveat:** Version is 0.7.0 (pre-1.0.0, requirement 1.ii). The REST API is versioned (`/api/v1/`)
and the package meets all other Level 3 requirements. The 1.0.0 release is planned after
API validation through pilot deployments.
