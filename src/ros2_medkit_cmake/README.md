# ros2_medkit_cmake

Shared CMake modules for the ros2_medkit workspace. Provides multi-distro compatibility,
build acceleration, and centralized linting configuration across all packages.

## Modules

| Module | Description |
|--------|-------------|
| `ROS2MedkitCcache.cmake` | Auto-detect and configure ccache with PCH-aware sloppiness settings |
| `ROS2MedkitCompat.cmake` | Multi-distro compatibility shims for ROS 2 Humble, Jazzy, and Lyrical |
| `ROS2MedkitCoverage.cmake` | gcov/lcov instrumentation behind `-DENABLE_COVERAGE=ON` |
| `ROS2MedkitLinting.cmake` | Centralized clang-tidy configuration (opt-in local gate; CI runs `run-clang-tidy` instead) |
| `ROS2MedkitSanitizers.cmake` | ASan / TSan / UBSan behind `-DSANITIZER=asan,ubsan` |
| `ROS2MedkitTestDomain.cmake` | Per-package `ROS_DOMAIN_ID` allocation for test isolation |
| `ROS2MedkitWarnings.cmake` | Shared warning flags and vendored-code relaxations |

### ROS2MedkitCompat

Resolves dependency differences across ROS 2 distributions:

- `medkit_find_yaml_cpp()` - Finds yaml-cpp (namespaced targets on Jazzy, manual fallback on Humble)
- `medkit_find_cpp_httplib()` - Finds cpp-httplib >= 0.14 via pkg-config, CMake config, or vendored fallback (`VENDORED_DIR` param)
- `medkit_target_dependencies()` - Drop-in replacement for `ament_target_dependencies` (removed on Lyrical)
- `medkit_detect_compat_defs()` / `medkit_apply_compat_defs()` - Compile definitions for version-specific APIs

### ROS2MedkitLinting

Registers the package's `clang_tidy` CTest test behind `-DENABLE_CLANG_TIDY=ON`,
off by default. This is a local gate: CI does not use it. The `clang-tidy` job
in `.github/workflows/quality.yml` configures with `-DENABLE_CLANG_TIDY=OFF` and
runs `run-clang-tidy` over the compilation database instead.

A participating package registers exactly one such test - packages that exclude
`ament_cmake_clang_tidy` and never call `ros2_medkit_clang_tidy()` register none.
colcon runs a separate `ctest` per package, so `--ctest-args -j` has nothing to
parallelise: CTest only ever sees one matching test. The analysis is therefore
parallelised **inside** the test:

- `-DROS2_MEDKIT_CLANG_TIDY_JOBS=<n>` at configure time sets how many
  `clang-tidy` processes one package runs. It defaults to `min(host cores, 2)`,
  falling back to 1 where CMake cannot determine the core count.
- `ros2_medkit_clang_tidy(JOBS <n>)` overrides it for a single package.
- `./scripts/test.sh tidy --jobs <n>` is the everyday switch. The count is baked
  into the CTest command at configure time, so the script reconfigures the
  clang-tidy packages first - a couple of seconds, no recompilation. The value
  then sticks in the CMake cache, so every `tidy` run prints the count in
  effect.

How many packages analyse at once is colcon's `--parallel-workers`, not
CTest's `-j`. Because the parallelism now lives inside each test, the package
tests must run one at a time: `./scripts/test.sh tidy` pins
`--parallel-workers 1` and ignores a caller's override, because running both
levels at once multiplies peak memory by the number of packages.

The footprint is large enough that the default is capped rather than set to the
core count. Measured on `ros2_medkit_gateway`, a 16-core host:

| `JOBS`        | wall clock | peak resident |
|---------------|-----------:|--------------:|
| 2 *(default)* |   17min58s |       2.6 GiB |
| 4             |    9min55s |       4.7 GiB |
| 8             |    6min00s |       9.4 GiB |
| 16            |    4min32s |      17.4 GiB |

Memory scales linearly at roughly 1.2 GiB per job - a single `clang-tidy`
process holds 1.4 GiB at its peak - while wall clock does not. The default is
sized so that one package fits an 8 GB machine, which puts it at 2. That is
deliberately the slow end of the curve: the alternative is a default that only
works on the largest machine anyone here has.

If you have the memory, take it - `./scripts/test.sh tidy --jobs 8` roughly
triples the speed for 9.4 GiB.

Over-subscribing is guarded, but only through the script. `ament_clang_tidy`
derives its exit status from the warnings it managed to parse, so a `clang-tidy`
process killed by the OOM reaper contributes nothing and the test passes as if
the package were clean. `./scripts/test.sh tidy` scans the per-test logs for the
`failed with error code` marker and fails the run instead, naming the packages
that lost coverage. A bare `colcon test -R clang_tidy` has no such guard.

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
