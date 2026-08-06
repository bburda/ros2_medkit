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
# ccache keeps its counters inside the cache directory, and actions/cache
# restores that directory whole, counters included. Read without preparation
# they are therefore the running total over every job that has fed this cache
# entry, not what the build in front of you did. Every build step that calls
# this runs `ccache -z` first so the numbers below describe that build alone.
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

# Fill level at which cleanups mean the ceiling is the problem. A cleanup on its
# own proves nothing: ccache trims a subdirectory when that subdirectory passes
# max_size/16, so a cache at 14% of its ceiling can still report a handful. The
# state worth a warning is the one that motivated the current sizes - a cache
# pinned at its ceiling, evicting objects the same build is still producing.
FULL_PCT=90

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
max_kib=$(get max_cache_size_kibibyte)

hits=$((direct + preprocessed))
total=$((hits + miss))

if [ "$total" -gt 0 ]; then
  hit_rate=$((hits * 100 / total))
else
  hit_rate=0
fi

if [ "$max_kib" -gt 0 ]; then
  fill=$((size_kib * 100 / max_kib))
else
  fill=0
fi

max_size=$(ccache --get-config max_size)
size_gib=$(awk -v k="$size_kib" 'BEGIN { printf "%.2f", k / 1048576 }')

echo "==> ccache ($LABEL): ${hit_rate}% hits (${hits}/${total}), ${size_gib} GiB of ${max_size} (${fill}% full), ${cleanups} cleanups"

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "| ccache | hit rate | size | max | full | cleanups |"
    echo "|---|---:|---:|---:|---:|---:|"
    echo "| \`$LABEL\` | ${hit_rate}% | ${size_gib} GiB | ${max_size} | ${fill}% | ${cleanups} |"
  } >> "$GITHUB_STEP_SUMMARY"
fi

# Two distinct failures, reported separately because the fixes differ, and only
# one of them can be true at a time.
#
# A cache pinned at its ceiling that is still running cleanups is discarding
# objects the same build produced. Raise CCACHE_MAXSIZE, or free room in the
# repository's Actions cache quota so the entry stops being evicted. Cleanups
# below the fill threshold are not this: ccache trims per subdirectory, so a
# nearly empty cache can report a few and it means nothing.
#
# A low hit rate with the ceiling nowhere in sight is the other shape. Either
# the cache was never restored - check the key and its restore-keys prefix - or
# the branch changed something that invalidated the objects, such as a compiler
# flag, in which case the next run recovers on its own.
if [ "$total" -gt 0 ] && [ "$cleanups" -gt 0 ] && [ "$fill" -ge "$FULL_PCT" ]; then
  echo "::warning title=ccache evicted during the build::$LABEL ran $cleanups cleanup(s) while ${fill}% full (${size_gib} GiB of ${max_size}). The cache is discarding objects this same build produced, so CCACHE_MAXSIZE is too small for it or the Actions cache quota is evicting the entry."
elif [ "$total" -gt 0 ] && [ "$hit_rate" -lt "$HIT_FLOOR" ]; then
  echo "::warning title=ccache hit rate below ${HIT_FLOOR}%::$LABEL served ${hit_rate}% of ${total} cacheable calls from cache while only ${fill}% full, so the ceiling is not the constraint. Either the cache was not restored - check the key and its restore-keys prefix - or this branch changed a compiler flag and invalidated the objects."
fi
