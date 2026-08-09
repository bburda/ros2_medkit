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
# Pin what drop_stale_results.sh has to remove.
#
# The property is not "it deletes some files". It is that NOTHING colcon would
# count survives the sweep, whatever the file is named, and that a file colcon
# would not count is left alone. So the fixture carries one of every shape this
# workspace actually produces:
#
#   *.xunit.xml   ament's test runner and launch_testing
#   *.gtest.xml   ament_add_gtest, which is 171 of the 177 result files in a
#                 full build here and is exactly what a sweep by name missed
#   Testing/      a REAL ctest run, so TAG and Test.xml are whatever the
#                 installed ctest writes rather than what this test guesses
#   a plain .xml  an OPC UA node set, one of ~230 build inputs under build/
#                 that a blunter sweep would have destroyed
#
# The fixture is measured before the sweep as well as after. A fixture colcon
# does not count would make the "0 tests" assertion pass while proving nothing.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/drop_stale_results.sh"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

BASE="$WORK/build"
mkdir -p "$BASE/pkg_a/test_results/pkg_a" "$BASE/pkg_b"

# Two cases, passing. What matters is that colcon parses and counts them.
cat > "$BASE/pkg_a/test_results/pkg_a/from_ament.xunit.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<testsuite name="pkg_a" tests="2" failures="0" errors="0" time="0.1">
  <testcase classname="pkg_a" name="one" time="0.05"/>
  <testcase classname="pkg_a" name="two" time="0.05"/>
</testsuite>
XML

# Three cases, under a <testsuites> root, which is the shape googletest writes.
cat > "$BASE/pkg_a/test_results/pkg_a/from_gtest.gtest.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="3" failures="0" disabled="0" errors="0" time="0.3" name="AllTests">
  <testsuite name="Suite" tests="3" failures="0" disabled="0" errors="0" time="0.3">
    <testcase name="a" status="run" time="0.1" classname="Suite"/>
    <testcase name="b" status="run" time="0.1" classname="Suite"/>
    <testcase name="c" status="run" time="0.1" classname="Suite"/>
  </testsuite>
</testsuites>
XML

# A build input that is not a result file. It must survive: deleting every .xml
# under the build base would force a rebuild of the packages that vendor these.
cat > "$BASE/pkg_b/Opc.Ua.Example.NodeSet2.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<UANodeSet><Aliases/></UANodeSet>
XML

# A real ctest run, so Testing/TAG and Testing/<stamp>/Test.xml are the genuine
# article. One test, so the arithmetic below stays readable. `-T Test` is the
# mode colcon runs, and the only one that writes TAG; a plain `ctest` leaves
# Testing/Temporary and nothing colcon can find.
cat > "$BASE/pkg_a/CTestTestfile.cmake" <<'CMAKE'
add_test(from_ctest "/bin/true")
CMAKE
# ctest names this file twice on stdout when a build directory does not have it.
# Empty is enough; it only exists to keep the fixture's output readable.
: > "$BASE/pkg_a/DartConfiguration.tcl"
(cd "$BASE/pkg_a" && ctest -T Test >/dev/null)

fail() {
  echo "FAIL: $1" >&2
  exit 1
}

summary() {
  # `colcon test-result` exits non-zero when it finds failures; there are none
  # here, but keep the pipeline honest anyway.
  colcon test-result --test-result-base "$BASE" 2>/dev/null | tail -1
}

BEFORE="$(summary)"
echo "before: $BEFORE"
# 2 xunit + 3 gtest + 1 ctest. An exact number, not "more than zero": if a shape
# stopped being counted the fixture would quietly stop testing that shape.
case "$BEFORE" in
  *"6 tests"*) ;;
  *) fail "the fixture is not what this test thinks it is, colcon reports: $BEFORE" ;;
esac

"$SCRIPT" "$BASE"

AFTER="$(summary)"
echo "after : $AFTER"
case "$AFTER" in
  *"0 tests"*) ;;
  *) fail "stale results survived the sweep, colcon still reports: $AFTER" ;;
esac

[ -f "$BASE/pkg_b/Opc.Ua.Example.NodeSet2.xml" ] ||
  fail "the sweep deleted a build input that is not a test result"

# A missing build base is the state of a fresh clone, and must not be an error.
"$SCRIPT" "$WORK/never_built" ||
  fail "the sweep failed on a build base that does not exist"

echo "OK: nothing colcon would count survives the sweep, and nothing else was touched"
