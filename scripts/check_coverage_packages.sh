#!/usr/bin/env bash
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
# Fail-loud gate against silently shrinking coverage scope.
#
# A package that never applies --coverage produces no .gcda, so lcov never sees
# it and it disappears from the report entirely - out of the numerator AND the
# denominator. The workspace percentage then describes only the instrumented
# subset while looking like it describes everything. That is how nine C++
# packages sat outside the reported number without any job turning red.
#
# The expected package set is derived from the SOURCE TREE - any package holding
# hand-written C++ outside test/ and vendored/ - and never from the presence of
# include(ROS2MedkitCoverage). Deriving it from the include would be circular:
# deleting that one line would remove the package from the report and from the
# assertion about it in the same edit, and a newly added package that never
# opts in would never be expected in the first place. Both are exactly the
# regression this gate exists to catch.
#
# Two checks:
#   static  - every expected package includes ROS2MedkitCoverage
#   report  - every expected package reached the lcov report, contributing a
#             record for one of its own implementation files (not merely a
#             header, which a consumer's translation unit would supply anyway)
#
# WHAT THIS DOES NOT CATCH, stated so nobody mistakes it for more:
#   * per-file loss. The report check is per package: a package that loses
#     instrumentation on all but one of its .cpp files still passes. Closing
#     that needs a committed per-package file-count baseline, which is churn
#     on every legitimate source addition.
#   * include placed after a target. The flags apply at directory scope, so a
#     target declared before the include is not instrumented. Asserting the
#     include's line number against the first add_library/add_executable is not
#     workable: such a grep fails legal CMake (targets inside function() bodies,
#     if(FALSE) blocks, INTERFACE libraries that compile nothing) while missing
#     real ones (ament_auto_add_library, uppercase commands, add_subdirectory).
#     A misplaced include still shows up here whenever it costs the package
#     every .cpp record; a partial slip does not.
#
# Usage:
#   ./scripts/check_coverage_packages.sh <coverage.info> [--skip <package>]...
#   ./scripts/check_coverage_packages.sh --static-only [--skip <package>]...
#
# --static-only runs the source-tree half without a coverage build, for the
# lint job, where it costs a fraction of a second.
#
# --skip marks a package deliberately not built in the current job (e.g.
# ros2_medkit_opcua, excluded from the coverage build). Skips are echoed, and a
# --skip naming a package that is not expected is an error rather than a silent
# no-op, so the flag cannot rot after a rename.
#
# Exit codes:
#   0  every expected package passes every applicable check
#   1  at least one expected package failed a check
#   2  gate misconfiguration (bad arguments, missing files, empty expected set)

set -euo pipefail

# Packages that hold C++ but are deliberately outside the coverage report.
# Every entry needs a reason. Entries are validated against the source tree, so
# one that no longer names a real package is an error rather than dead weight.
EXCLUDED_PACKAGES=(
  # Everything it compiles is test scaffolding - demo_nodes/ fixtures driven by
  # the launch_testing suites - not production code under measurement.
  "ros2_medkit_integration_tests"
)

COVERAGE_FILE=""
STATIC_ONLY=""
SKIPPED=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --static-only)
      STATIC_ONLY="yes"
      shift
      ;;
    --skip)
      if [[ $# -lt 2 ]]; then
        echo "FAIL: --skip requires a package name." >&2
        exit 2
      fi
      SKIPPED+=("$2")
      shift 2
      ;;
    -*)
      echo "FAIL: unknown argument '$1'." >&2
      exit 2
      ;;
    *)
      if [[ -n "${COVERAGE_FILE}" ]]; then
        echo "FAIL: unexpected extra argument '$1'." >&2
        exit 2
      fi
      COVERAGE_FILE="$1"
      shift
      ;;
  esac
done

if [[ -z "${STATIC_ONLY}" && -z "${COVERAGE_FILE}" ]]; then
  echo "FAIL: usage: $(basename "$0") <coverage.info> [--skip <package>]..." >&2
  echo "              $(basename "$0") --static-only [--skip <package>]..." >&2
  exit 2
fi

if [[ -n "${COVERAGE_FILE}" && ! -s "${COVERAGE_FILE}" ]]; then
  echo "FAIL: coverage file '${COVERAGE_FILE}' is missing or empty." >&2
  exit 2
fi

if [[ ! -d src ]]; then
  echo "FAIL: 'src' not found. Run this script from the colcon workspace root." >&2
  exit 2
fi

# Expected set: package directories holding hand-written C++ under src/ or
# include/. -L follows a symlinked src (a plain `find src` refuses to descend
# into one and would silently yield an empty set). The manifest list is
# materialised first so a find failure is fatal instead of being swallowed by a
# process substitution, which `set -o pipefail` does not cover.
manifests=""
if ! manifests=$(find -L src -name package.xml -print 2>/dev/null | sort); then
  echo "FAIL: could not enumerate packages under 'src'." >&2
  exit 2
fi
if [[ -z "${manifests}" ]]; then
  echo "FAIL: no package.xml found under 'src' - the gate would pass vacuously." >&2
  exit 2
fi

# Every extension GCC will compile as C++ and gcov will therefore record. A
# narrower list would make a package invisible purely by naming its files .cc.
CXX_FIND_EXPR=(
  -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.c++'
  -o -name '*.hpp' -o -name '*.hh' -o -name '*.hxx' -o -name '*.h'
)
CXX_IMPL_RE='\.(cpp|cc|cxx|c\+\+)$'

expected=()
seen_names=""
declare -A pkg_dir_of=()
declare -A has_impl_sources=()
declare -A package_exists=()
while IFS= read -r manifest; do
  [[ -n "${manifest}" ]] || continue
  pkg_dir="$(dirname "${manifest}")"
  pkg="$(basename "${pkg_dir}")"
  package_exists["${pkg}"]="${pkg_dir}"

  # Two package directories sharing a basename would collapse into one entry in
  # the associative arrays below, leaving the first silently unchecked. Refuse
  # rather than half-check.
  if [[ "${seen_names}" == *"|${pkg}|"* ]]; then
    echo "FAIL: two packages share the directory name '${pkg}'." >&2
    echo "This gate keys packages by name; rename one or teach it full paths." >&2
    exit 2
  fi
  seen_names="${seen_names}|${pkg}|"

  excluded=""
  for e in "${EXCLUDED_PACKAGES[@]}"; do
    if [[ "${pkg}" == "${e}" ]]; then
      excluded="yes"
      break
    fi
  done
  if [[ -n "${excluded}" ]]; then
    echo "EXCLUDED: ${pkg} (see EXCLUDED_PACKAGES in $(basename "$0"))"
    continue
  fi

  # Search the whole package, not just src/ and include/: a package keeping its
  # sources in nodes/ or lib/ is just as much production code. test/ and
  # vendored/ are the only things measurement legitimately ignores.
  # No `head` in this pipeline: it would close the pipe early, SIGPIPE the
  # producer, and `set -o pipefail` would then kill the gate with no output.
  sources=$(find -L "${pkg_dir}" -type f \( "${CXX_FIND_EXPR[@]}" \) 2>/dev/null \
            | grep -vE "^${pkg_dir}/(test|vendored)/|/vendored/" || true)
  [[ -n "${sources}" ]] || continue

  expected+=("${pkg}")
  pkg_dir_of["${pkg}"]="${pkg_dir}"
  # Header-only packages (INTERFACE libraries) legitimately reach the report
  # only through a consumer's translation unit; packages shipping an
  # implementation file must show one of their own.
  if printf '%s\n' "${sources}" | grep -qE "${CXX_IMPL_RE}"; then
    has_impl_sources["${pkg}"]="yes"
  fi
done <<< "${manifests}"

if [[ ${#expected[@]} -eq 0 ]]; then
  echo "FAIL: no package under 'src' holds C++ sources - the gate would pass vacuously." >&2
  echo "This is not the ros2_medkit workspace, or the layout changed." >&2
  exit 2
fi

# An EXCLUDED_PACKAGES entry naming nothing is a rotted exemption. Unlike a
# rotted --skip it would never surface, because exclusion happens before any
# check runs - so validate it explicitly.
for e in "${EXCLUDED_PACKAGES[@]}"; do
  if [[ -z "${package_exists[${e}]:-}" ]]; then
    echo "FAIL: EXCLUDED_PACKAGES names '${e}', which is not a package under 'src'." >&2
    echo "It was renamed or removed. Drop the entry or correct it." >&2
    exit 2
  fi
done

# A --skip that matches nothing is a rotted flag, not a no-op.
for s in ${SKIPPED[@]+"${SKIPPED[@]}"}; do
  found=""
  for pkg in "${expected[@]}"; do
    if [[ "${pkg}" == "${s}" ]]; then
      found="yes"
      break
    fi
  done
  if [[ -z "${found}" ]]; then
    echo "FAIL: --skip '${s}' matches no expected package." >&2
    echo "It was renamed, moved, or no longer holds C++. Update the caller." >&2
    exit 2
  fi
done

failed=()

# --- static check: the include is present -------------------------------------
# Case-insensitive and tolerant of whitespace before the paren, because CMake
# commands are case-insensitive and `include (X)` is legal. A stricter matcher
# would fail correct code.
for pkg in "${expected[@]}"; do
  cmakelists="${pkg_dir_of[${pkg}]}/CMakeLists.txt"
  if [[ ! -f "${cmakelists}" ]]; then
    failed+=("${pkg} (has C++ sources but no CMakeLists.txt)")
    continue
  fi

  if ! grep -qiE '^[[:space:]]*include[[:space:]]*\([[:space:]]*ROS2MedkitCoverage[[:space:]]*\)' \
         "${cmakelists}"; then
    failed+=("${pkg} (missing include(ROS2MedkitCoverage))")
  fi
done

# --- report check: the package actually reached the lcov output ---------------
checked=0
if [[ -z "${STATIC_ONLY}" ]]; then
  for pkg in "${expected[@]}"; do
    skip=""
    for s in ${SKIPPED[@]+"${SKIPPED[@]}"}; do
      if [[ "${pkg}" == "${s}" ]]; then
        skip="yes"
        break
      fi
    done
    if [[ -n "${skip}" ]]; then
      echo "SKIP: ${pkg} (not built in this job)"
      continue
    fi

    checked=$((checked + 1))
    # Anchor on the package directory as it sits under the workspace, so build/
    # and install/ copies cannot stand in as proof that a package's production
    # code was measured. Vendored records are excluded here too: CI strips them
    # with lcov --remove, but this gate must not depend on the caller doing so.
    # shellcheck disable=SC2016
    # Single quotes are deliberate: `&` is sed's "the whole match" backreference
    # and must reach sed literally, not be expanded by the shell.
    esc='s/[][\.*^$(){}?+|/]/\\&/g'
    dir_re=$(printf '%s' "${pkg_dir_of[${pkg}]}" | sed "${esc}")
    files=$(grep -E "^SF:.*/${dir_re}/" "${COVERAGE_FILE}" \
            | grep -vc '/vendored/' || true)
    if [[ "${files}" -eq 0 ]]; then
      failed+=("${pkg} (no files in ${COVERAGE_FILE})")
      continue
    fi

    # Headers alone do not prove the package was instrumented. A consumer's
    # instrumented translation unit attributes coverage to every header it
    # includes, so a package could lose instrumentation on all of its own
    # implementation files and still appear "present". Demand a record for one
    # of its own .cpp/.cc/.cxx whenever it ships any.
    if [[ -n "${has_impl_sources[${pkg}]:-}" ]]; then
      impl=$(grep -E "^SF:.*/${dir_re}/" "${COVERAGE_FILE}" \
             | grep -v '/vendored/' | grep -cE "${CXX_IMPL_RE}" || true)
      if [[ "${impl}" -eq 0 ]]; then
        failed+=("${pkg} (only headers in ${COVERAGE_FILE}, no implementation file of its own)")
        continue
      fi
    fi

    printf 'OK:   %-42s %s file(s)\n' "${pkg}" "${files}"
  done
fi

# Reported before the "everything was skipped" bail-out below, so a real static
# failure is never swallowed by a misconfigured caller.
if [[ ${#failed[@]} -gt 0 ]]; then
  echo >&2
  echo "FAIL: coverage scope is smaller than the source tree:" >&2
  for f in "${failed[@]}"; do
    echo "  - ${f}" >&2
  done
  echo >&2
  echo "Every package holding production C++ must include the shared" >&2
  echo "ROS2MedkitCoverage module, before its first target, and must appear in" >&2
  echo "the report. A package that does neither still builds and still runs its" >&2
  echo "tests; it just leaves the reported percentage describing a subset. Fix by:" >&2
  echo "  * adding include(ROS2MedkitCoverage) above the first target, or" >&2
  echo "  * passing --skip <package> if this job deliberately does not build it, or" >&2
  echo "  * adding it to EXCLUDED_PACKAGES with a reason if it is not production code" >&2
  exit 1
fi

if [[ -z "${STATIC_ONLY}" && ${checked} -eq 0 ]]; then
  echo "FAIL: every expected package was skipped - nothing was verified." >&2
  exit 2
fi

if [[ -n "${STATIC_ONLY}" ]]; then
  echo "OK: all ${#expected[@]} C++ package(s) include ROS2MedkitCoverage."
else
  echo
  echo "OK: all ${checked} checked C++ package(s) include ROS2MedkitCoverage and reached ${COVERAGE_FILE}."
fi
