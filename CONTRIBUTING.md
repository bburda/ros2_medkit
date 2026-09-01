# Contributing to ros2_medkit

Thanks for your interest in contributing to ros2_medkit! This guide explains how to report issues, suggest features, and contribute code.

## How to Report Issues

### Did you find a bug?

- **Ensure the bug was not already reported** by searching [Issues](https://github.com/selfpatch/ros2_medkit/issues)
- If you can't find an existing issue, [open a new one](https://github.com/selfpatch/ros2_medkit/issues/new/choose) and select the **Bug report** template
- Fill in all sections of the template:
  - **Steps to reproduce** - numbered steps to recreate the issue
  - **Expected behavior** - what you expected to happen
  - **Actual behavior** - what actually happened, including error messages or stack traces
  - **Environment** - ros2_medkit version, ROS 2 distro, OS
  - **Additional information** - logs, snippets, or screenshots if helpful

### Do you want to suggest a feature or improvement?

- Check if the feature has already been suggested in [Issues](https://github.com/selfpatch/ros2_medkit/issues)
- If not, [open a new issue](https://github.com/selfpatch/ros2_medkit/issues/new/choose) and select the **Feature request / General issue** template
- Fill in all sections:
  - **Proposal** - describe the change or feature you'd like to see
  - **Motivation** - why is this important? Who does it benefit?
  - **Alternatives considered** - other options or implementations you considered
  - **Additional context** - any other context or screenshots

## How to Contribute Code

### Development Workflow

1. **Fork the repository** and clone your fork locally
2. **Install pre-commit hooks** (one-time setup):
   ```bash
   pip install pre-commit
   pre-commit install
   ```
3. **Create a branch** from `main` with a descriptive name:
   - `feature/short-description` for new features
   - `fix/short-description` for bug fixes
   - `docs/short-description` for documentation changes
4. **Make your changes** following the project's coding standards
5. **Test your changes** locally (see Build and Test section below)
6. **Commit your changes** with clear, descriptive commit messages
   - Pre-commit hooks will automatically check formatting
7. **Push your branch** to your fork
8. **Open a Pull Request** against the `main` branch of this repository

### Commit Messages

- Use clear and descriptive commit messages
- Start with a verb in imperative mood (e.g., "Add", "Fix", "Update", "Remove")
- Keep the first line under 72 characters
- Add a blank line followed by a more detailed explanation if needed

Examples:
```
Add support for SOVD entity mapping

Fix memory leak in diagnostic tree traversal

Update documentation for colcon build process
```

### Build and Test

Before opening or updating a Pull Request, you **must** build and test locally:

```bash
source /opt/ros/jazzy/setup.bash   # or humble - adjust for your distro
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

Use `scripts/test.sh` for testing (preferred over raw colcon commands):

```bash
./scripts/test.sh              # Unit tests only (default)
./scripts/test.sh integ        # Integration tests only
./scripts/test.sh lint         # Fast linters (no clang-tidy)
./scripts/test.sh all          # Everything
./scripts/test.sh <test_name>  # Single test by CTest name regex
```

Every run starts by dropping the result files of the previous one, so the summary it prints
is the tally of the run you just started and not of everything tested since the last clean.
`colcon test-result` counts any XML under `build/` whose root element is `<testsuite>` or
`<testsuites>`, whatever it is called, so the sweep asks colcon itself which files those are
rather than matching a name. Run it on its own with `./scripts/drop_stale_results.sh` if you
invoke `colcon test` directly.

Every run also reclaims shared memory left behind by DDS participants that were killed
rather than shut down. The aggregation suites kill peers on purpose, and nothing gives
those segments back: about 0.65 MB each, and a container's `/dev/shm` is 64 MB by default,
so they accumulate across runs until something unrelated fails for lack of space. The
reclaiming is `fastdds shm clean`, the vendor's own tool, which removes a segment only when
no process holds its lock. Run it on its own with `./scripts/sweep_shm.sh`; it refuses
while ROS processes are alive, and `--force` overrides that.

#### Pre-commit and Pre-push Hooks

```bash
pipx install pre-commit
pre-commit install
pre-commit install --hook-type pre-push
```

On commit: clang-format, cmake-lint, shellcheck, flake8, ament-copyright, trailing whitespace.
On push: incremental clang-tidy on changed `.cpp` files.

#### Reproducing a sanitizer failure locally

The ASan/TSan jobs multiply every declared CTest `TIMEOUT` by three, but a
budget a test asserts on *itself* is invisible to that rewrite - an
instrumented gateway can blow a "must answer within N seconds" assertion long
before ctest's clock runs out, and the failure then reads as a product
regression rather than as instrumentation overhead. Those budgets read
`MEDKIT_TEST_TIME_SCALE`, which the sanitizer jobs export with the same factor.

Set it when reproducing a sanitizer failure locally, or the run you get is not
the run CI got:

```bash
MEDKIT_TEST_TIME_SCALE=3 colcon test --ctest-args -LE linter
```

It is honoured by both suites - Python integration tests via
`ros2_medkit_test_utils.constants.get_time_scale()`, and C++ fixtures that wait
on wall-clock budgets via their own local `test_time_scale()` helper. Unset,
unparseable or below `1` means no scaling, so ordinary runs keep the tight
budgets that give the assertions their falsifying power.

#### An executor's node set is fixed while its thread runs

A test that needs a node on the graph for part of its run must give that node
its own executor, not hand it to one a thread is already spinning:

```cpp
#include "ros2_medkit_gateway/test_support/spinning_executor.hpp"

auto helper = std::make_shared<rclcpp::Node>("helper");
ros2_medkit_gateway::test_support::SpinningExecutor helper_spin({helper});
// ... the node is served for as long as helper_spin is in scope
```

`add_node()` and `remove_node()` make the executor rebuild its entity
collection, and a thread inside `spin()`/`spin_some()` reads that collection
through the executor's notify waitable. There is no lock a caller can take
across the two, and ThreadSanitizer reports the overlap as a write in
`operator delete` against `ExecutorNotifyWaitable::is_ready()`. The report
names whichever test happened to be running, so it reads as a defect in that
test rather than as the shared pattern it is.

`SpinningExecutor` takes its nodes in the constructor, starts the thread after
them, and cancels and joins before any node leaves, so the two never overlap.
A fixture may still add its nodes in `SetUp()` as long as it does so before the
spin thread starts and removes them after joining it.

#### Code Coverage

Run from the workspace root. This mirrors the measurement pipeline of the CI
coverage job (same lcov capture, filters and flags), so the local percentage is
comparable to the published one. The job additionally builds with
`--symlink-install`, installs `ros-jazzy-test-msgs` via rosdep, and prints
`lcov --list`; see `.github/workflows/ci.yml` for the authoritative version.

```bash
WS="${PWD}"   # gcov records the path the compiler was given, not the resolved one

colcon build --packages-skip ros2_medkit_opcua \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
colcon test --packages-skip ros2_medkit_opcua --ctest-args -LE linter

lcov --capture --directory build --output-file coverage.raw.info \
  --ignore-errors mismatch,negative,empty,gcov
lcov --extract coverage.raw.info "${WS}/src/*/src/*" "${WS}/src/*/include/*" \
  --output-file coverage.extracted.info --ignore-errors unused,empty
lcov --remove coverage.extracted.info '*/vendored/*' \
  --output-file coverage.info --ignore-errors unused,empty
genhtml coverage.info --output-directory coverage_html --ignore-errors source

./scripts/check_coverage_packages.sh coverage.info --skip ros2_medkit_opcua
```

In CI this gate runs in the `coverage (merge)` job rather than alongside the tests: the
coverage run is sharded by package, so each shard's tracefile is legitimately partial and
the merged report is the first place the question "did every package reach the report"
can be asked.

Open `coverage_html/index.html` in your browser.

`ros2_medkit_opcua` is skipped because it pulls `open62541pp` over the network
via `FetchContent`, which costs every matrix run time for a plugin unrelated to
it; CI skips it in the same jobs and builds it separately in
`.github/workflows/opcua-plugin.yml`. Dropping the `--remove` step leaves
vendored third-party code in the report and your number will not match CI's.

#### Coverage scope

`ENABLE_COVERAGE` is handled by the shared `ROS2MedkitCoverage` module, not by
per-package CMake code. **Every package that compiles production C++ must
`include(ROS2MedkitCoverage)` before it declares its first target** - the flags
apply at directory scope, so a target created before the include is not
instrumented. Packages that compile only test scaffolding are exempt and are
listed in `EXCLUDED_PACKAGES` in the gate script, each with a reason;
`ros2_medkit_integration_tests` is the current entry.

A package that misses this does not fail the build. It emits no `.gcda`, lcov
never sees it, and it drops out of both the numerator and the denominator - the
reported percentage then silently describes a subset of the workspace.

`scripts/check_coverage_packages.sh` guards against that. It derives the set of
packages that *must* appear from the source tree (anything holding hand-written
C++ under `src/` or `include/`), never from the presence of the include - a set
derived from the include could not detect a package that never added it, or one
where the line was deleted. It runs twice in CI: `--static-only` in the Quality
workflow, so a missing include fails in seconds, and against the real report in
the coverage job. Pass `--skip <package>` for a package the current job does not
build.

#### CI/CD

All PRs are tested on Ubuntu 24.04 (Jazzy), where build and tests run in a single job, plus Humble and Lyrical. Linters and clang-tidy run in the Quality workflow. Coverage is uploaded to Codecov on push to main. ccache is configured in the CI and Quality workflows and in the OPC-UA workflow's colcon jobs. The Pixi workflow and the Docker image builds compile the workspace without it, so a slow build there is expected rather than a cache problem.

Every ccache-backed build step ends with `scripts/ccache_report.sh`, which writes the hit rate, cache size and cleanup count to the job summary. It raises a workflow warning in the two states that make a cache useless: cleanups above zero, meaning the cache evicted objects the same build was still producing and `CCACHE_MAXSIZE` is too small for it, and a hit rate under 50% with no cleanups, meaning the cache was never restored and the key or its `restore-keys` prefix is wrong. It never fails the step - a cold cache is a legitimate state.

### Pull Request Checklist

Before submitting your PR, ensure:

- [ ] Code follows the repository's style and conventions
- [ ] Build succeeds without warnings
- [ ] All tests pass locally
- [ ] New tests are added for new functionality
- [ ] Existing tests are updated if behavior changes
- [ ] Documentation is updated where applicable
- [ ] PR description clearly explains what changed and why
- [ ] Related issue is referenced (e.g., "Fixes #123")

### Code Review Process

- The project maintainers will review pull requests as time permits
- Address review feedback promptly
- Keep discussions professional and constructive
- Be patient - maintainers may need time to review

## What NOT to Contribute

- **Purely cosmetic changes** that don't add functionality (whitespace, formatting without other changes)
- **Large refactors** without prior discussion - open an issue first
- **Breaking changes** without coordination with maintainers
- **Code that doesn't pass tests** or breaks existing functionality

## Questions and Help

- For questions about **using ros2_medkit**, open a [Discussion](https://github.com/selfpatch/ros2_medkit/discussions) or an Issue with the question label
- For questions about **contributing**, feel free to ask in your PR or Issue
- For **security vulnerabilities**, see [`SECURITY.md`](SECURITY.md)

## Code of Conduct

By contributing to ros2_medkit, you agree to abide by our [Code of Conduct](CODE_OF_CONDUCT.md). Please be respectful and considerate in all interactions.

## License

By contributing to ros2_medkit, you agree that your contributions will be licensed under the Apache License 2.0.

---

Thank you for improving ros2_medkit! We value contributions of all sizes - from typo fixes to major features. 🚀
