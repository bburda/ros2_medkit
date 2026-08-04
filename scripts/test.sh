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
#   integ       - Integration tests only
#   lint        - Linters only (clang-format, copyright, cmake-lint; NO clang-tidy)
#   tidy        - clang-tidy only (slow, ~8-10 min)
#   all         - Everything (equivalent to bare colcon test)
#   <test_name> - Run a single test by CTest name regex
#
# Examples:
#   ./scripts/test.sh                        # unit tests only
#   ./scripts/test.sh integ                  # integration tests only
#   ./scripts/test.sh lint                   # fast linters only
#   ./scripts/test.sh test_health_handler    # single test
#   ./scripts/test.sh unit --packages-select ros2_medkit_gateway

PRESET="${1:-unit}"
shift 2>/dev/null || true

COMMON_ARGS=(--event-handlers console_direct+ --parallel-workers "$(nproc)" --return-code-on-test-failure)

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
      --ctest-args -j "$(nproc)" -L integration \
      "$@"
    ;;
  lint)
    echo "==> Running linters (excluding clang-tidy)"
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -j "$(nproc)" -L linter -E "clang_tidy" \
      "$@"
    ;;
  tidy)
    echo "==> Running clang-tidy"
    # One package at a time: each clang_tidy test already analyses its own
    # translation units in parallel (ROS2_MEDKIT_CLANG_TIDY_JOBS, default one
    # process per core). Running the package tests concurrently on top of that
    # would multiply peak memory by the number of packages.
    colcon test "${COMMON_ARGS[@]}" \
      --ctest-args -j 1 -R "clang_tidy" \
      "$@"
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

echo ""
echo "==> Results:"
colcon test-result --verbose || true
exit "$TEST_EXIT"
