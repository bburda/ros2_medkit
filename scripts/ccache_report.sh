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
# Report what ccache actually did in a CI build step, and say so loudly when it
# thrashed.
#
# The build-time argument for the cache sizes in the sanitizer jobs is that a
# cache which evicts during its own build is worse than no cache: it throws away
# objects the same compilation is still producing. That state is visible in
# `ccache -s` and nothing ever read it, so a regression back into it would have
# been silent - the job just gets slow again.
#
# Usage: scripts/ccache_report.sh <label>
#
# Emits a one-line table row to $GITHUB_STEP_SUMMARY when running under Actions,
# always prints to stdout, and raises a workflow warning when the cache filled
# up or the hit rate collapsed. It never fails the step: a cold cache is a
# legitimate state (a new key, a first run after a flag change) and blocking a
# merge on a cache condition would be worse than the slow build it reports.

set -euo pipefail

LABEL="${1:-ccache}"

# Hit-rate floor. Below this the cache is not paying for itself. The issue that
# motivated the current sizes measured 0-49% while thrashing and 81% once the
# cache fit, so the gap is wide and the exact threshold is not delicate.
HIT_FLOOR=50

if ! command -v ccache >/dev/null 2>&1; then
  echo "ccache not installed, nothing to report"
  exit 0
fi

stats=$(ccache --print-stats)
get() { awk -v k="$1" '$1 == k { print $2; found = 1 } END { if (!found) print 0 }' <<<"$stats"; }

direct=$(get direct_cache_hit)
preprocessed=$(get preprocessed_cache_hit)
miss=$(get cache_miss)
cleanups=$(get cleanups_performed)
size_kib=$(get cache_size_kibibyte)

hits=$((direct + preprocessed))
total=$((hits + miss))

if [ "$total" -gt 0 ]; then
  hit_rate=$((hits * 100 / total))
else
  hit_rate=0
fi

max_size=$(ccache --get-config max_size)
size_gib=$(awk -v k="$size_kib" 'BEGIN { printf "%.2f", k / 1048576 }')

echo "==> ccache ($LABEL): ${hit_rate}% hits (${hits}/${total}), ${size_gib} GiB of ${max_size}, ${cleanups} cleanups"

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "| ccache | hit rate | size | max | cleanups |"
    echo "|---|---:|---:|---:|---:|"
    echo "| \`$LABEL\` | ${hit_rate}% | ${size_gib} GiB | ${max_size} | ${cleanups} |"
  } >> "$GITHUB_STEP_SUMMARY"
fi

# Two distinct failures, reported separately because the fixes differ. Cleanups
# mean the ceiling is too low for this build's object set: raise CCACHE_MAXSIZE,
# or free room in the repository's Actions cache quota. A low hit rate with no
# cleanups means the cache was not restored at all: look at the cache key and
# its restore-keys prefix.
if [ "$cleanups" -gt 0 ] && [ "$total" -gt 0 ]; then
  echo "::warning title=ccache evicted during the build::$LABEL ran $cleanups cleanup(s) at ${size_gib} GiB of ${max_size}. The cache is discarding objects this same build produced, so CCACHE_MAXSIZE is too small for it or the Actions cache quota is evicting the entry."
fi

if [ "$total" -gt 0 ] && [ "$hit_rate" -lt "$HIT_FLOOR" ] && [ "$cleanups" -eq 0 ]; then
  echo "::warning title=ccache hit rate below ${HIT_FLOOR}%::$LABEL served ${hit_rate}% of ${total} cacheable calls from cache without evicting anything, which points at a cache that was never restored rather than one that is too small. Check the cache key and its restore-keys prefix."
fi
