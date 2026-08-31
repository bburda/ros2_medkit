#!/bin/bash
# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

# Quick-test presets for ros2_medkit development.
# Usage: ./scripts/test.sh [preset] [extra colcon test args...]
#
# Presets:
#   unit        - Unit tests only, no linters, no integration (DEFAULT)
#
# ctest -L takes a regex, so the label-selecting presets ask for their own
# label or "gate". The domain gates carry the "gate" label and no other: they
# must not be labelled "linter", because CI runs with -LE linter and a gate
# excluded from CI is not a gate.
#   integ       - Integration tests only
#   lint        - Linters only (clang-format, copyright, cmake-lint; NO clang-tidy)
#   tidy        - clang-tidy only (slow; accepts --jobs N, see below)
#   all         - Everything (equivalent to bare colcon test)
#   <test_name> - Run a single test by CTest name regex
#
# Examples:
#   ./scripts/test.sh                        # unit tests only
#   ./scripts/test.sh integ                  # integration tests only
#   ./scripts/test.sh lint                   # fast linters only
#   ./scripts/test.sh test_health_handler    # single test
#   ./scripts/test.sh unit --packages-select ros2_medkit_gateway
#   ./scripts/test.sh tidy --jobs 8          # trade memory for speed
#
# The tidy preset defaults to a small number of clang-tidy processes per package
# so one package fits an 8 GB machine. Each process peaks around 1.4 GiB, so
# --jobs N costs roughly N x 1.4 GiB. The value sticks in the CMake cache and is
# printed on every run.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PRESET="${1:-unit}"
shift 2>/dev/null || true

COMMON_ARGS=(--event-handlers console_direct+ --parallel-workers "$(nproc)" --return-code-on-test-failure)

# Drop every result file from earlier runs before starting, so the summary at the
# end is the tally of THIS run. A separate script because it has a test of its own,
# scripts/test_drop_stale_results.sh, and because "drop the stale results" is worth
# being able to ask for on its own. The reasoning lives there.
"$HERE/drop_stale_results.sh" build

# Reclaim shared memory stranded by participants that were killed rather than shut
# down - the aggregation suites kill peers on purpose, and nothing gives those
# segments back. Tolerated when it declines: it refuses to sweep while ROS
# processes are alive, and that refusal must not stop a test run.
"$HERE/sweep_shm.sh" || true

# Run tests, capture exit code so we always show results even on failure.
set +e
case "$PRESET" in
  unit)
    echo "==> Running unit tests (no linters, no integration)"
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -j "$(nproc)" -LE "linter|integration" \
      "$@"
    ;;
  integ)
    echo "==> Running integration tests only"
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -j "$(nproc)" -L "integration|gate" \
      "$@"
    ;;
  lint)
    echo "==> Running linters (excluding clang-tidy)"
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -j "$(nproc)" -L "linter|gate" -E "clang_tidy" \
      "$@"
    ;;
  tidy)
    # One package at a time: each clang_tidy test already analyses its own
    # translation units in parallel (ROS2_MEDKIT_CLANG_TIDY_JOBS, capped by
    # default so one package fits an 8 GB machine). Running the package tests
    # concurrently on top of that would multiply peak memory by the number of
    # packages.
    #
    # colcon is the knob that does it, not CTest. Every participating package
    # registers exactly one clang_tidy test and colcon runs a separate ctest
    # per package, so --ctest-args -j has nothing to parallelise here.
    TIDY_JOBS=""
    TIDY_ARGS=()
    while [ $# -gt 0 ]; do
      case "$1" in
        --jobs)
          # Guard the arity: this branch runs with errexit off, so a bare
          # "shift 2" on a single remaining argument fails without shifting
          # and the loop spins forever.
          if [ $# -lt 2 ]; then
            echo "error: --jobs requires a value" >&2
            exit 2
          fi
          TIDY_JOBS="$2"
          shift 2
          ;;
        --jobs=*) TIDY_JOBS="${1#--jobs=}"; shift ;;
        *) TIDY_ARGS+=("$1"); shift ;;
      esac
    done

    # The process count is baked into the CTest command at configure time, so
    # changing it means reconfiguring. That costs a couple of seconds and no
    # recompilation, which is what makes a flag here practical.
    if [ -n "$TIDY_JOBS" ]; then
      if ! [[ "$TIDY_JOBS" =~ ^[1-9][0-9]*$ ]]; then
        echo "error: --jobs takes a positive integer, got '$TIDY_JOBS'" >&2
        exit 2
      fi
      echo "==> Reconfiguring clang-tidy packages for $TIDY_JOBS job(s) per package"
      for testfile in build/*/CTestTestfile.cmake; do
        grep -q "add_test(clang_tidy" "$testfile" 2>/dev/null || continue
        if ! cmake "$(dirname "$testfile")" \
             -DROS2_MEDKIT_CLANG_TIDY_JOBS="$TIDY_JOBS" >/dev/null; then
          echo "error: reconfigure failed for $(dirname "$testfile")" >&2
          exit 2
        fi
      done
    fi

    # Report what is actually baked into the tests. The value persists in the
    # CMake cache until changed again, so printing it keeps that from becoming
    # invisible state.
    TIDY_EFFECTIVE=$(grep -ho '"--jobs" "[0-9]*"' build/*/CTestTestfile.cmake 2>/dev/null |
      grep -o '[0-9]*' | sort -un | paste -sd, -)
    echo "==> Running clang-tidy (${TIDY_EFFECTIVE:-unconfigured} job(s) per package)"

    # --parallel-workers 1 goes LAST, after the caller's arguments. colcon keeps
    # recognising its own options after --ctest-args, and the option is a plain
    # argparse store, so the final occurrence wins. Placed any earlier, a
    # caller's own --parallel-workers would silently undo the memory guard.
    if [[ " ${TIDY_ARGS[*]} " == *" --parallel-workers "* ||
          " ${TIDY_ARGS[*]} " == *" --parallel-workers="* ]]; then
      echo "    note: the tidy preset pins --parallel-workers 1, so that override is ignored"
    fi
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -R "clang_tidy" \
      "${TIDY_ARGS[@]}" --parallel-workers 1
    ;;
  all)
    echo "==> Running all tests"
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -j "$(nproc)" \
      "$@"
    ;;
  *)
    echo "==> Running tests matching: $PRESET"
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -j "$(nproc)" -R "$PRESET" \
      "$@"
    ;;
esac
TEST_EXIT=$?
set -e

# ament_clang_tidy exits 0 even when a clang-tidy process dies. It catches
# CalledProcessError, prints the failure to stderr and returns whatever output
# it recovered - nothing, for a process killed by a signal - and then derives
# the exit status purely from the warnings it managed to parse. A translation
# unit whose clang-tidy was OOM-killed therefore contributes zero findings and
# the test passes, which is the worst way to lose coverage: silently.
#
# run_test.py merges the child's stderr into the per-test log, so the marker is
# recoverable. Fail the run on it rather than let a half-analysed package read
# as clean.
if [ "$PRESET" = "tidy" ]; then
  TIDY_LOGS=(build/*/ament_clang_tidy/clang_tidy.txt)
  if grep -qs "failed with error code" "${TIDY_LOGS[@]}"; then
    echo ""
    echo "==> clang-tidy processes died during this run:"
    grep -hs "failed with error code" "${TIDY_LOGS[@]}" | sort -u | sed 's/^/    /'
    echo "    Those translation units were NOT analysed, so this run proves nothing"
    echo "    about them. Error code -9 means the OOM reaper took them: lower --jobs."
    TEST_EXIT=1
  fi
fi

echo ""
echo "==> Results:"
colcon test-result --verbose || true
exit "$TEST_EXIT"
