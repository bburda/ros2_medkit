# ros2_medkit_cmake

Shared CMake modules for the ros2_medkit workspace. Provides multi-distro compatibility,
build acceleration, and centralized linting configuration across all packages.

## Modules

| Module | Description |
|--------|-------------|
| `ROS2MedkitCcache.cmake` | Auto-detect and configure ccache with PCH-aware sloppiness settings |
| `ROS2MedkitCompat.cmake` | Multi-distro compatibility shims for ROS 2 Humble, Jazzy, and Lyrical |
| `ROS2MedkitCoverage.cmake` | gcov/lcov instrumentation behind `-DENABLE_COVERAGE=ON` |
| `ROS2MedkitLinting.cmake` | Centralized clang-tidy configuration (opt-in locally, mandatory in CI) |
| `ROS2MedkitSanitizers.cmake` | ASan / TSan / UBSan behind `-DSANITIZER=asan,ubsan` |
| `ROS2MedkitTestDomain.cmake` | Per-package `ROS_DOMAIN_ID` allocation for test isolation |
| `ROS2MedkitWarnings.cmake` | Shared warning flags and vendored-code relaxations |

### ROS2MedkitCompat

Resolves dependency differences across ROS 2 distributions:

- `medkit_find_yaml_cpp()` - Finds yaml-cpp (namespaced targets on Jazzy, manual fallback on Humble)
- `medkit_find_cpp_httplib()` - Finds cpp-httplib >= 0.14 via pkg-config, CMake config, or vendored fallback (`VENDORED_DIR` param)
- `medkit_target_dependencies()` - Drop-in replacement for `ament_target_dependencies` (removed on Lyrical)
- `medkit_detect_compat_defs()` / `medkit_apply_compat_defs()` - Compile definitions for version-specific APIs

### ROS2MedkitCoverage

Adds `--coverage -O0 -g` when built with `-DENABLE_COVERAGE=ON`, and is a no-op
otherwise. The flags are applied at directory scope, so every target declared
after the `include()` is instrumented - libraries, executables and test binaries
alike.

Every package that compiles production C++ must include this module, and must
include it **before its first target**. Packages that compile only test
scaffolding are exempt; they are listed in `EXCLUDED_PACKAGES` in the gate
script, each with a reason.

Skipping it does not break the build: the package emits no `.gcda`, lcov never
sees it, and it silently leaves both the numerator and the denominator of the
reported coverage percentage. `scripts/check_coverage_packages.sh` guards
against that, deriving the packages that must appear from the source tree rather
than from this include. It runs `--static-only` in the Quality workflow and
against the generated report in the CI coverage job.

The gate checks that the include is present, not where it sits, because no
reliable line-based check survives legal CMake (targets inside `function()`
bodies, `if(FALSE)` blocks, `INTERFACE` libraries that compile nothing). An
include placed after a target still shows up whenever it costs the package all
of its records; a partial slip does not. Put it above the first target.

## Usage

In your package's `CMakeLists.txt`, before the first target:

```cmake
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)
```

Add to `package.xml`:

```xml
<buildtool_depend>ros2_medkit_cmake</buildtool_depend>
```

The cmake modules are automatically available via ament's extras hook after `find_package`.

## License

Apache License 2.0
