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
#
# Drop every test result file an earlier run left behind.
#
# Usage: ./scripts/drop_stale_results.sh [build base, default "build"]
#
# `colcon test-result` reports on whatever it finds under the build base, and it
# does not look for a naming convention. Two extensions do the finding:
#
#   colcon_test_result.xunit  walks the whole tree, parses EVERY .xml file and
#                             counts the ones whose root element is <testsuite>
#                             or <testsuites>
#   colcon_cmake.ctest        reads each Testing/TAG file and counts the Test.xml
#                             it names
#
# So a run that tested one package still reports the tallies of every package
# tested since the last clean, and a preset that selects a handful of tests
# prints a total in the thousands. That is not cosmetic: the summary is what a
# reader takes as the outcome of the run they just started.
#
# Sweeping by name is what let it survive an earlier attempt at this. Only
# launch tests and ament's own runner write <name>.xunit.xml; `ament_add_gtest`
# writes <target>.gtest.xml, and in this workspace the gtest files outnumber the
# xunit ones roughly thirty to one, so a `-name '*.xunit.xml'` sweep left almost
# everything in place.
#
# Ask colcon itself instead. `--delete-yes` removes exactly the files its own
# crawler found, which is the same crawler that produces the tally, so the sweep
# cannot drift from the tally by construction, and it cannot touch an .xml that
# is not a result file - the open62541 node sets under build/ros2_medkit_opcua
# are ~230 of those.
#
# The Testing directories go with it. The ctest extension only ever offers TAG
# for deletion, so the Test.xml it points at would survive; and what neither
# extension reads (LastTest.log, the older timestamped directories) is stale
# CTest state of no use to the next run either.

set -euo pipefail

BUILD_BASE="${1:-build}"

if [ ! -d "$BUILD_BASE" ]; then
  exit 0
fi

colcon test-result --test-result-base "$BUILD_BASE" --delete-yes >/dev/null
find "$BUILD_BASE" -type d -name 'Testing' -prune -exec rm -rf {} +
