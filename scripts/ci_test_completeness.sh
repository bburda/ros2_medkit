#!/usr/bin/env bash
# Say plainly when a test step ran out of time, instead of letting it read as a
# test failure.
#
# A step killed by its timeout is marked FAILED by the runner, not cancelled, and
# the in-flight test leaves no result file. `colcon test-result` then reports that
# as one erroring test named `<test>.xunit.missing_result`: a test that never
# failed, among thousands that never ran. On its own that is indistinguishable
# from a single flake, which is how a budget overrun gets filed as a test problem
# while every package queued behind it disappears without trace.
#
# The step's own log is not readable from inside the job, so the overrun is
# detected by the clock instead: the caller records a start timestamp before the
# test step and passes the step's cap here. Elapsed at or above the cap means the
# runner killed it.
#
# What this deliberately does NOT do is count registered tests against result
# files. Some tests legitimately write none - `test_dds_domain_allocation` is
# registered by ros2_medkit_cmake in every package and produces no xunit - so
# that comparison reports missing results on a run where everything passed.
#
# Usage: ci_test_completeness.sh <start-epoch-seconds> <cap-minutes>
set -euo pipefail

START_EPOCH="${1:-}"
CAP_MINUTES="${2:-}"

usage() {
  echo "usage: ci_test_completeness.sh <start-epoch-seconds> <cap-minutes>" >&2
  echo "  both are positive integers; the start is seconds since the epoch, as" >&2
  echo "  written by date +%s before the step being measured" >&2
}

# Validated rather than trusted: a start value from a different clock, or a cap
# of zero, otherwise turns this from a diagnostic into a source of false
# verdicts - a monotonic start reads as an enormous overrun, and a zero cap
# divides by zero while still printing an overrun.
if [[ ! "${START_EPOCH}" =~ ^[0-9]+$ || ! "${CAP_MINUTES}" =~ ^[0-9]+$ ]]; then
  usage
  exit 2
fi
if [[ "${START_EPOCH}" -eq 0 || "${CAP_MINUTES}" -eq 0 ]]; then
  usage
  exit 2
fi

now=$(date +%s)
if [[ "${START_EPOCH}" -gt "${now}" ]]; then
  echo "::error::the recorded start (${START_EPOCH}) is in the future - the clock moved, so no margin can be computed"
  exit 2
fi

elapsed=$(( now - START_EPOCH ))
cap_seconds=$(( CAP_MINUTES * 60 ))

empty=0
found=0
while IFS= read -r xml; do
  found=$((found + 1))
  if [[ ! -s "${xml}" ]]; then
    echo "::error::empty result file: ${xml}"
    empty=$((empty + 1))
  fi
done < <(find build -path '*/test_results/*' -name '*.xml' 2>/dev/null)

missing=$(find build -path '*/test_results/*' -name '*.missing_result' 2>/dev/null | wc -l)

printf 'test step: %ds elapsed of a %ds cap (%d%%); results: %d written, %d empty, %d missing_result\n' \
  "${elapsed}" "${cap_seconds}" "$(( elapsed * 100 / cap_seconds ))" "${found}" "${empty}" "${missing}"

status=0

if [[ "${found}" -eq 0 ]]; then
  echo "::error::no test result files at all - the run never reached the tests"
  status=1
fi

if [[ "${empty}" -gt 0 ]]; then
  status=1
fi

# The runner kills the step AT its cap and this check runs afterwards, so an
# overrun is simply elapsed at or past the cap. An earlier version subtracted a
# fixed minute, which made every cap below five minutes report an overrun on a
# run that had barely started.
if [[ "${elapsed}" -ge "${cap_seconds}" ]]; then
  echo "::error::the test step reached its ${CAP_MINUTES} minute cap. Any missing_result above belongs to the test that was interrupted, and the packages queued behind it did not run at all. This is a budget overrun, not a test verdict."
  status=1
elif [[ "${elapsed}" -ge $(( cap_seconds * 8 / 10 )) ]]; then
  echo "::warning::the test step used $(( elapsed * 100 / cap_seconds ))% of its ${CAP_MINUTES} minute cap - it will start being killed before anyone decides to raise it"
fi

if [[ "${missing}" -gt 0 && "${elapsed}" -lt "${cap_seconds}" ]]; then
  echo "::warning::${missing} test(s) produced no result file without the step running out of time - investigate the test, not the budget"
fi

exit "${status}"
