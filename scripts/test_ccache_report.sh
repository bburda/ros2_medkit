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
# Pin what ccache_report.sh warns about.
#
# A warning that fires on every run carries no information, and one that never
# fires carries none either. The rows below are what CI jobs actually reported,
# so the thresholds are checked against states the jobs reach rather than
# invented ones. Three rows carry most of the weight:
#
#   humble       four cleanups in a cache at a fifth of its ceiling is ccache
#                trimming a subdirectory, not a ceiling that is too small. An
#                earlier threshold warned here.
#   humble-old   the Jammy image ships ccache 4.5, which has no
#                max_cache_size_kibibyte. Without the unit fallback the fill
#                reads 0 and the eviction warning can never fire on that job.
#   foreign      full and useless at once. This is what a restore-keys prefix
#                matching another job's cache looks like from inside.
#
# ccache is stubbed on PATH, so the script under test runs unmodified.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/ccache_report.sh"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/bin"
cat > "$WORK/bin/ccache" <<'STUB'
#!/bin/bash
case "$1" in
  --print-stats)
    printf 'cache_miss\t%s\n'              "$STUB_MISS"
    printf 'cleanups_performed\t%s\n'      "$STUB_CLEANUPS"
    printf 'direct_cache_hit\t%s\n'        "$STUB_HITS"
    printf 'preprocessed_cache_hit\t0\n'
    printf 'cache_size_kibibyte\t%s\n'     "$STUB_SIZE_KIB"
    # An empty STUB_MAX_KIB stands for the older ccache that omits the field.
    if [ -n "$STUB_MAX_KIB" ]; then
      printf 'max_cache_size_kibibyte\t%s\n' "$STUB_MAX_KIB"
    fi
    ;;
  --get-config) echo "$STUB_MAX_HUMAN" ;;
  *) exit 1 ;;
esac
STUB
chmod +x "$WORK/bin/ccache"
export PATH="$WORK/bin:$PATH"

# label|hits|miss|cleanups|size_kib|max_kib|max_human|expected|expected_fill
# expected is one of: silent, evicted, hitrate, foreign
# expected_fill of "-" skips the fill assertion.
CASES=(
  # Measured per build. 500M resolves to 488281 KiB, 1.5G to 1464843, 2G to 1953125.
  "jazzy-asan|553|1|0|954204|1953125|2.0 GB|silent|48"
  "jazzy-tsan|553|1|0|692061|1464843|1.5 GB|silent|47"
  "jazzy-lint|553|1|0|104858|488281|500.0 MB|silent|21"
  "lyrical|384|0|0|136314|488281|500.0 MB|silent|27"
  "humble|272|139|2|91845|488281|500.0 MB|silent|18"
  "foreign|42|369|219|482344|488281|500.0 MB|foreign|98"
  # Same job, same numbers, on the ccache that omits the machine-readable max.
  "humble-old|272|139|2|91845||500.0M|silent|18"
  # Constructed. A full cache that is not evicting is doing its job; a step that
  # compiled nothing must not be reported as a 0% hit rate; a cache that evicts
  # while serving most calls simply wants a bigger ceiling.
  "full-but-clean|900|100|0|480000|488281|500.0 MB|silent|98"
  "nothing-compiled|0|0|0|1000|488281|500.0 MB|silent|-"
  "pinned-but-useful|700|300|400|487000|488281|500.0 MB|evicted|99"
  "cold|0|400|0|1000|488281|500.0 MB|hitrate|0"
)

failures=0
for row in "${CASES[@]}"; do
  IFS='|' read -r label hits miss cleanups size_kib max_kib max_human expected exp_fill <<<"$row"

  out=$(
    STUB_HITS="$hits" STUB_MISS="$miss" STUB_CLEANUPS="$cleanups" \
    STUB_SIZE_KIB="$size_kib" STUB_MAX_KIB="$max_kib" STUB_MAX_HUMAN="$max_human" \
    "$SCRIPT" "$label"
  )

  if grep -q "title=ccache is full of another build" <<<"$out"; then
    actual=foreign
  elif grep -q "title=ccache evicted during the build" <<<"$out"; then
    actual=evicted
  elif grep -q "title=ccache hit rate below" <<<"$out"; then
    actual=hitrate
  else
    actual=silent
  fi

  count=$(grep -c '::warning' <<<"$out" || true)
  expected_count=0
  [ "$expected" != silent ] && expected_count=1
  if [ "$count" -ne "$expected_count" ]; then
    echo "FAIL $label: expected $expected_count warning(s), got $count"
    echo "     $out"
    failures=$((failures + 1))
    continue
  fi

  if [ "$actual" != "$expected" ]; then
    echo "FAIL $label: expected $expected, got $actual"
    echo "     $out"
    failures=$((failures + 1))
    continue
  fi

  if [ "$exp_fill" != "-" ]; then
    fill=$(sed -n 's/.*(\([0-9]*\)% full).*/\1/p' <<<"$out")
    if [ "$fill" != "$exp_fill" ]; then
      echo "FAIL $label: expected ${exp_fill}% full, reported ${fill:-none}%"
      failures=$((failures + 1))
      continue
    fi
  fi

  echo "ok   $label -> $actual"
done

if [ "$failures" -gt 0 ]; then
  echo "$failures case(s) failed"
  exit 1
fi
echo "all ${#CASES[@]} cases passed"
