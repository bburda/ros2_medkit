# ros2_medkit_cmake

Shared CMake modules for the ros2_medkit workspace. Provides multi-distro compatibility,
build acceleration, and centralized linting configuration across all packages.

## Modules

| Module | Description |
|--------|-------------|
| `ROS2MedkitCcache.cmake` | Auto-detect and configure ccache with PCH-aware sloppiness settings |
| `ROS2MedkitCompat.cmake` | Multi-distro compatibility shims for ROS 2 Humble, Jazzy, and Lyrical |
| `ROS2MedkitCoverage.cmake` | gcov/lcov instrumentation behind `-DENABLE_COVERAGE=ON` |
| `ROS2MedkitLinting.cmake` | Shared lint configs via `medkit_lint_config()`, plus the opt-in clang-tidy gate (CI runs `run-clang-tidy` instead) |
| `ROS2MedkitSanitizers.cmake` | ASan / TSan / UBSan behind `-DSANITIZER=asan,ubsan` |
| `ROS2MedkitTestDomain.cmake` | Per-test `ROS_DOMAIN_ID` allocation for test isolation - an OS-level socket lock taken at test start and released when the process ends, not a per-package table |
| `ROS2MedkitWarnings.cmake` | Shared warning flags and vendored-code relaxations |

### ROS2MedkitCompat

Resolves dependency differences across ROS 2 distributions:

- `medkit_find_yaml_cpp()` - Finds yaml-cpp (namespaced targets on Jazzy, manual fallback on Humble)
- `medkit_find_cpp_httplib()` - Finds cpp-httplib >= 0.14 via pkg-config, CMake config, or vendored fallback (`VENDORED_DIR` param)
- `medkit_target_dependencies()` - Drop-in replacement for `ament_target_dependencies` (removed on Lyrical)
- `medkit_detect_compat_defs()` / `medkit_apply_compat_defs()` - Compile definitions for version-specific APIs

### ROS2MedkitLinting

`medkit_lint_config(<file name> <output variable>)` resolves one of the three
shared configs - `.clang-format`, `.clang-tidy`, `.flake8`:

```cmake
include(ROS2MedkitLinting)
medkit_lint_config(.clang-format _config)
ament_clang_format(${_format_files} CONFIG_FILE "${_config}")
```

The configs are files of this package, in `cmake/` next to the modules and
installed next to them, so the same lookup works from a source tree, a plain
install and a `--symlink-install`. The repository root paths that editors,
`pre-commit` and `scripts/clang-tidy-diff.sh` use are symlinks to them, so there
is a single copy of each.

Address them any other way and it breaks quietly. `${CMAKE_CURRENT_SOURCE_DIR}/../..`
reaches the repository root, which exists in a workspace checkout and nowhere
else: a binary package is built from an export of one package directory, so the
linter gets a path that is not there. Nobody sees it, because bloom's
`debian/rules` runs the test step as `dh_auto_test || true`. A missing config
therefore aborts the configure step - a linter that runs unconfigured, or does
not run, is the worse outcome.

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

### ROS2MedkitSanitizers

A no-op unless `-DSANITIZER=` names at least one of `asan`, `tsan`, `ubsan`
(comma-separated; `asan` and `tsan` cannot be combined). When it is active it
overrides three things the build type would otherwise decide, and it does so
through `add_compile_options`, which CMake places after
`CMAKE_CXX_FLAGS_<CONFIG>` on the command line:

- `-O1` instead of the build type's level. Sanitizers report fewer false
  positives and run faster here than at `-O0`.
- `-g1`: line tables plus descriptions of functions and external variables, but
  no locals and no types. Before this was set, full DWARF took the instrumented
  ASan tree to about 27 GB, most of it debug info, which is written by the
  compiler, read by the linker and stored in ccache. A sanitizer report still
  names `file:line`, and still names the overflowed variable, because that name
  comes from the frame descriptor ASan embeds rather than from DWARF. What is
  lost is inspecting locals in a debugger on a core file.
- `-UNDEBUG`, so `assert()` stays live. `Release` and `RelWithDebInfo` both
  carry `-DNDEBUG`, and the CI sanitizer jobs build `RelWithDebInfo`, so
  without this the one build meant to abort on a broken invariant was the one
  compiling every assert away. Note how wide this reaches: the flag is
  directory-scoped like the others, so it enables assertions in everything
  compiled into a participating package, not only the handful the repository
  writes itself. `nlohmann/json` routes its `JSON_ASSERT` to `assert`, and the
  vendored `cpp-httplib` and `dynmsg` carry their own. Those checks firing is
  the intended behaviour of a sanitizer build, but it does mean a latent bug in
  a header shows up as an abort in the sanitizer jobs and nowhere else.

Do not move these into `CMAKE_CXX_FLAGS`, where the build type's flags would
come last and win instead.

### ROS2MedkitTestDomain

Gives every test a `ROS_DOMAIN_ID` of its own, allocated when the test starts and held for
exactly as long as the test runs.

```cmake
include(ROS2MedkitTestDomain)

medkit_add_gtest(test_foo test/test_foo.cpp)
medkit_add_gmock(test_baz test/test_baz.cpp)
medkit_add_pytest_test(test_py test/test_py.py)
medkit_add_launch_test(test_bar test/test_bar.test.py TIMEOUT 90)
```

There is no table, no per-package pool and no size to keep an eye on. Adding a test to a
package is registering it, and nothing else.

Each of those macros wraps the test's command in `medkit_domain_runner.py`, which replaces
ament's `run_test.py`: it takes a domain, exports it, runs the test, and releases the domain
when the test process ends - including when it is killed, because the release is the kernel
closing a socket. The lock itself is `domain_coordinator.domain_id` from `ament_cmake_ros`,
which binds a TCP socket on `32768 + domain`. Because it is an OS-level lock rather than a
CTest property, it reaches across the separate `ctest` runs colcon starts per package, which
is what a `RESOURCE_LOCK` never could.

A test that needs several domains at once - a multi-gateway test running a second and a third
gateway - asks for them with `DOMAINS <n>`. The first arrives as `ROS_DOMAIN_ID` and the rest
as `MEDKIT_SECONDARY_DOMAINS`, which `ros2_medkit_test_utils.constants.get_test_domain_id`
reads. All of them are held by that test alone.

```cmake
medkit_add_launch_test(test_peer_aggregation test/test_peer_aggregation.test.py DOMAINS 4)
```

For a test that builds its own command line and so cannot take an ament test runner, there is
`medkit_add_wrapped_test(<name> [DOMAINS <n>] COMMAND <cmd...>)`, which puts
`medkit_run_with_domain.py` in front of the command instead.

#### A launch test's properties belong to the call that registers it

`medkit_add_launch_test(<name> <file> [TIMEOUT n] [DOMAINS n] [LABELS "a;b"] [ENV "K=V" ...]
[ARGS "foo:=bar" ...])` is the only supported way to give a launch test a label or an
environment variable:

```cmake
medkit_add_launch_test(test_multi test/test_multi.test.py TIMEOUT 300 DOMAINS 4
  LABELS "integration" ENV "FOO=bar" "BAZ=qux" ARGS "foo:=bar")
```

Setting them afterwards - `set_tests_properties(test_multi PROPERTIES LABELS
...)` or `set_property(TEST test_multi APPEND PROPERTY ENVIRONMENT ...)` - is
not supported, and not merely discouraged: see the next section for why the
build can register no launch test at all, and a property call that names a
test CMake never registered is a hard configure error, not a no-op. `ENV` is
applied after the domain the macro already set for the test, by appending, so
a caller's own environment can never overwrite `MEDKIT_TEST_DOMAINS` - an
`ENV` entry that sets the literal key `MEDKIT_TEST_DOMAINS=` fails the
configure with `FATAL_ERROR` naming `DOMAINS` as the way to set it, rather
than silently landing behind the macro's own entry and winning at run time
the way a plain `set_tests_properties(... PROPERTIES ENVIRONMENT ...)` used
to. This is a literal string match, not a generator-expression evaluation:
it does not chase down a value spelled to avoid the literal key (e.g.
`ENV "$<1:MEDKIT_TEST_DOMAINS=99>"`), which nobody writes by accident and
which a blanket rejection of `$<` would also catch legitimate uses of.
`LABELS` overrides `add_launch_test()`'s own default of `"launch_test"`, the same
override callers used to apply by hand. `ARGS` forwards extra launch
arguments straight to `add_launch_test()`, same as it always did - it has its
own keyword rather than falling through as an unrecognised argument, because
`LABELS` being multi-value (to survive `ENV`'s/`LABELS`'s own semicolon-split
values, see the `.cmake` comment) would otherwise absorb an `ARGS` that had
no keyword of its own instead of erroring on it.

This does not reach `medkit_add_gtest`, `medkit_add_gmock`,
`medkit_add_pytest_test` or `medkit_add_wrapped_test`: those are always
registered, so `MEDKIT_TEST_DOMAINS` is appended to their `ENVIRONMENT` and a
caller adding entries of its own still uses
`set_property(TEST ... APPEND PROPERTY ENVIRONMENT ...)` afterwards, same as
before. A plain `set_tests_properties(... PROPERTIES ENVIRONMENT ...)` drops
the domain there too, and the check below says so.

#### Launch tests are not registered in a system package build

`medkit_add_launch_test()` registers nothing when two conditions both hold -
the install prefix is the ROS distribution root, AND
`CMAKE_INSTALL_LOCALSTATEDIR` is the absolute path `/var` - and prints why
once per directory instead:

```
ros2_medkit: launch tests are not registered in this build. Installing into
/opt/ros/humble is a system package build, whose test step runs before the
install step, so a launch test cannot resolve this package through the ament
index. gtests and pytest tests still run.
```

A launch test resolves the executables and the Python helper package of the
package under test through the ament index, which only an installed prefix
carries. Installing straight into the ROS distribution root is what a system
package build does - the ROS build farm's binarydeb job - and that job runs
its test step *before* the install step, so the package it is testing is not
yet on `AMENT_PREFIX_PATH`. Every launch test then fails on resolution rather
than on anything the test set out to check. Humble is the only distro whose
binarydeb job still runs package tests at all; every distro from Iron onward
disabled that step (`ros2/ros_buildfarm_config` PR 298). Because bloom runs it
as `dh_auto_test || true`, the failures cannot stop the build, so what used to
reach the build farm's log was every launch test failing for a reason that
said nothing about the package.

The install prefix alone is not a safe signal: `colcon build --install-base
/opt/ros/<distro>` also installs straight into the distribution root, but its
test step runs *after* its own install step, so its launch tests work fine.
`CMAKE_INSTALL_LOCALSTATEDIR` is the second signal that tells the two apart -
debhelper's `cmake` buildsystem always passes it
(`-DCMAKE_INSTALL_LOCALSTATEDIR=/var`), and no `colcon build` ever does. The
check is on the *value*, not merely `DEFINED`: a package that
`include(GNUInstallDirs)`s defines `CMAKE_INSTALL_LOCALSTATEDIR` itself, to
the relative `var` (no leading slash), which is a different value from what
debhelper passes - `DEFINED` alone cannot tell those apart, so the guard
requires the absolute `/var`. No in-tree package includes `GNUInstallDirs`
today, so this is hardening against a real default rather than a fix for an
observed failure. Both comparisons (the prefix and `CMAKE_INSTALL_LOCALSTATEDIR`)
are exact (`STREQUAL`), so a trailing slash is stripped from each before
comparing rather than trusted to match bloom's spelling.

The "once per directory" of the printed message is a macro-scoped CMake
variable, not a `CACHE` entry: a package that calls `medkit_add_launch_test`
many times in one `CMakeLists.txt` sees the explanation once, and a different
`CMakeLists.txt` in the same build - a different package, or a second
`add_subdirectory()` - prints its own.

gtests and pytest tests are untouched on purpose. That chroot is the only
place our packages are exercised against a minimal dependency closure, and it
is where the missing rosbag2 sqlite3 storage plugin was found - taking that
coverage away along with the launch tests would trade one blind spot for
another.

Only domains **1-100 and 215-231** are drawn from. RTPS gives a domain the UDP slice
`[7400 + 250 * d, 7400 + 250 * d + 249]`, and the kernel hands out ephemeral ports from
`net.ipv4.ip_local_port_range` (32768-60999 by default), which covers domains 101-214. If an
unrelated process is given one of those ports first, the node dies at startup with
`failed to bind to ANY:<port>: address in use` and every case in the file fails at once.
Domain 0 stays free because it is the ROS 2 default a developer shell uses, and 232 is
dropped because its slice runs past 65535. The full derivation, per DDS implementation, is in
`scripts/medkit_domain.py`.

That band is only correct for a machine whose `net.ipv4.ip_local_port_range` is the default
one, and that is a sysctl: `1024 65535` and `10240 65535` are common on CI images and
Kubernetes nodes, and under the first of them 45% of ephemeral ports land in a slice the band
calls safe. So the range is read at run time, on the machine that *tests* - in CI the machine
that builds is not the machine that tests - and a band that does not hold there is refused
rather than used. Nothing is downgraded silently: the refusal names the sysctl, and names the
part of the band that is still safe so it can be taken deliberately.

```
MEDKIT_TEST_EPHEMERAL_PORT_RANGE=<low>-<high>   declare the range on a machine where
                                                /proc/sys is not visible at all
MEDKIT_TEST_DOMAIN_BAND=1-10                    narrow the band to a subset (never widen)
```

The declaration can only widen what the kernel reports, never narrow it, so it cannot be used
to silence the check. A machine where the range can be read neither way is refused too, and
deliberately not assumed to be a default one: a wrong guess there shows up much later, as a
node in some other test failing to bind.

That is 117 domains. `scripts/test.sh` runs `colcon test` with `ctest -j $(nproc)` inside
each package, so more tests than that can be in flight at once on a large machine; a test
that finds the band full waits for a domain rather than failing, up to
`MEDKIT_TEST_DOMAIN_WAIT` seconds (180 by default). What it never does is fall back to a
literal or to domain 0.

A test that asks for several domains takes all of them or none. While it waits it holds
nothing, which is what keeps two multi-domain tests from wedging each other: each holding
part of what the other needs is a deadlock that only ends when both wait budgets run out.

`MEDKIT_TEST_DOMAINS` is appended to the test's `ENVIRONMENT`, so a caller adding its own
entries must use `set_property(TEST ... APPEND PROPERTY ENVIRONMENT ...)`. A plain
`set_tests_properties(... PROPERTIES ENVIRONMENT ...)` afterwards drops it, and the check
below says so. This is the general rule for `medkit_add_gtest`, `medkit_add_gmock`,
`medkit_add_pytest_test` and `medkit_add_wrapped_test` - all four are always registered, so a
call naming the test afterwards is safe, just easy to get wrong in the APPEND direction.
**A launch test is the one exception**: it is not always registered (see "Launch tests are not
registered in a system package build" above), so a `set_tests_properties` or `set_property`
call naming it afterwards is not a subtler bug to get right, it is unsupported - use
`medkit_add_launch_test`'s own `LABELS` and `ENV` parameters instead.

The failure this scheme has to guard against is silent: a test registered with plain
`ament_add_gtest` or `add_launch_test` does not fail, it runs on domain 0 and sees every node
on the machine. Two gates catch it, and neither has to be asked for:

- `test_dds_domain_allocation`, registered in every package automatically by the extras hook
  behind `find_package(ros2_medkit_cmake)`. It runs on the machine that *tests*, reads back
  the generated CTest properties, and fails on any test whose command does not go through
  the wrapper. Nothing in a package's `CMakeLists.txt` registers it, which is the point: a
  gate a package has to opt into is a gate the next package will not have.
- `test_dds_domain_coverage`, registered once by this package, which sweeps every package
  build directory in the workspace and applies the same rule - including to packages that
  never found `ros2_medkit_cmake` and so carry no gate of their own. A package with tests
  and no gate is reported by name.

A test that genuinely creates no ROS entities says so at its own call site:

```cmake
add_test(NAME core_only COMMAND $<TARGET_FILE:test_core_only>)
set_tests_properties(core_only PROPERTIES LABELS "unit")
medkit_test_needs_no_domain(core_only)   # after the LABELS assignment, which replaces
```

The package's own suite proves the parts that matter: that the band is what it says it is,
that a port held by an unrelated process is stepped over rather than fatal, that an exhausted
band is reported rather than downgraded, that excess holders wait and are served as domains
free up, that a wrapper killed with `SIGKILL` frees its domain, and - with the control that
makes the zero mean something - that two independently allocated ROS nodes hear nothing from
each other while the same two on one domain hear each other fine.

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
