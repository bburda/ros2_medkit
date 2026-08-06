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
# fires carries none either. The rows below are the real numbers six CI jobs
# reported, so the thresholds are checked against the states the jobs actually
# reach rather than against invented ones. `humble` is the row that matters
# most: four cleanups in a cache at 15% of its ceiling is ccache trimming a
# subdirectory, not a ceiling that is too small, and an earlier version of the
# script warned about it.
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
    printf 'cache_miss\t%s\n'                 "$STUB_MISS"
    printf 'cleanups_performed\t%s\n'         "$STUB_CLEANUPS"
    printf 'direct_cache_hit\t%s\n'           "$STUB_HITS"
    printf 'preprocessed_cache_hit\t0\n'
    printf 'cache_size_kibibyte\t%s\n'        "$STUB_SIZE_KIB"
    printf 'max_cache_size_kibibyte\t%s\n'    "$STUB_MAX_KIB"
    ;;
  --get-config) echo "$STUB_MAX_HUMAN" ;;
  *) exit 1 ;;
esac
STUB
chmod +x "$WORK/bin/ccache"
export PATH="$WORK/bin:$PATH"

# label|hits|miss|cleanups|size_kib|max_kib|max_human|expected
# expected is one of: silent, evicted, hitrate
CASES=(
  # Measured in CI. 500M resolves to 488281 KiB, 1.5G to 1464844 KiB.
  "lyrical|788|364|0|136314|488281|500.0 MB|silent"
  "jazzy-tidy|1166|496|0|104858|488281|500.0 MB|silent"
  "humble|672|555|4|73400|488281|500.0 MB|silent"
  "jazzy-tsan|501|1161|186|692061|1464844|1.5 GB|hitrate"
  "jazzy-test|430|2185|1672|482344|488281|500.0 MB|evicted"
  # Constructed. A full cache that is not evicting is doing its job, and a step
  # that compiled nothing must not be reported as a 0% hit rate.
  "full-but-clean|900|100|0|480000|488281|500.0 MB|silent"
  "nothing-compiled|0|0|0|1000|488281|500.0 MB|silent"
  # The ceiling wins when both shapes are true at once.
  "pinned-and-cold|10|990|400|487000|488281|500.0 MB|evicted"
)

failures=0
for row in "${CASES[@]}"; do
  IFS='|' read -r label hits miss cleanups size_kib max_kib max_human expected <<<"$row"

  out=$(
    STUB_HITS="$hits" STUB_MISS="$miss" STUB_CLEANUPS="$cleanups" \
    STUB_SIZE_KIB="$size_kib" STUB_MAX_KIB="$max_kib" STUB_MAX_HUMAN="$max_human" \
    "$SCRIPT" "$label"
  )

  if grep -q "title=ccache evicted during the build" <<<"$out"; then
    actual=evicted
  elif grep -q "title=ccache hit rate below" <<<"$out"; then
    actual=hitrate
  else
    actual=silent
  fi

  # Exactly one warning, never two.
  count=$(grep -c '::warning' <<<"$out" || true)
  if [ "$actual" = silent ] && [ "$count" -ne 0 ]; then
    echo "FAIL $label: expected no warning, got $count"
    failures=$((failures + 1))
    continue
  fi
  if [ "$actual" != silent ] && [ "$count" -ne 1 ]; then
    echo "FAIL $label: expected exactly one warning, got $count"
    failures=$((failures + 1))
    continue
  fi

  if [ "$actual" != "$expected" ]; then
    echo "FAIL $label: expected $expected, got $actual"
    echo "     $out"
    failures=$((failures + 1))
  else
    echo "ok   $label -> $actual"
  fi
done

if [ "$failures" -gt 0 ]; then
  echo "$failures case(s) failed"
  exit 1
fi
echo "all ${#CASES[@]} cases passed"
