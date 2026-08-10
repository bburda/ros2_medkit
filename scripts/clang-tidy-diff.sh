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

# Incremental clang-tidy: only analyses C++ source files passed as arguments.
# Designed for pre-commit which passes staged filenames.
# Requires: merged compile_commands.json (run ./scripts/merge-compile-commands.sh after build).
#
# On the pre-push stage pre-commit passes everything that differs between the local
# branch and the REMOTE branch. After a rebase the remote tip is the pre-rebase one,
# so every file the base branch moved in the meantime arrives here looking changed -
# on a branch rebased over a few dozen commits that is most of the repository, in
# packages the author never touched and may not even have built. The list is
# therefore narrowed to what this branch really changes, which is what the hook is
# for. Files given explicitly on the command line are always honoured.
#
# Usage (standalone):
#   ./scripts/clang-tidy-diff.sh src/ros2_medkit_gateway/src/config.cpp
#
# Usage (via pre-commit): automatic - pre-commit passes changed files.

set -euo pipefail

COMPILE_DB="build/compile_commands.json"

if [ ! -f "$COMPILE_DB" ]; then
  echo "Warning: No merged compile_commands.json found."
  echo "Run: colcon build && ./scripts/merge-compile-commands.sh"
  exit 0
fi

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CLANG_TIDY_CONFIG=".clang-tidy"
ERRORS=0

# Files this branch changes, against the point it forked from the base branch. Empty
# when the base ref is unavailable (a fresh clone, a detached checkout, a fork with a
# different remote name), and the filter below is then skipped rather than silencing
# the hook.
BRANCH_FILES=""
if BASE_REF="$(git merge-base origin/main HEAD 2>/dev/null)"; then
  BRANCH_FILES="$(git diff --name-only "$BASE_REF" HEAD 2>/dev/null || true)"
fi

for file in "$@"; do
  # Not ours: a file the base branch changed, handed over by the pre-push stage
  # because the remote tip predates a rebase. Its package may not even be built here,
  # and clang-tidy without a compile database reports the whole file as errors.
  if [ -n "$BRANCH_FILES" ] && ! printf '%s\n' "$BRANCH_FILES" | grep -Fxq "$file"; then
    continue
  fi

  # Only process C++ source files (.cpp). Headers are checked transitively
  # through their including .cpp files. CI runs full clang-tidy on all files.
  case "$file" in
    *.cpp) ;;
    *) continue ;;
  esac

  if [[ "$file" == *"/vendored/"* ]]; then
    continue
  fi

  # Resolve to absolute path for matching against compile_commands.json
  ABS_FILE="$REPO_ROOT/$file"
  if [ ! -f "$ABS_FILE" ]; then
    ABS_FILE="$(realpath "$file" 2>/dev/null || echo "$file")"
  fi

  # Capture output: clang-tidy emits a "X warnings generated. Suppressed Y"
  # summary plus source snippets on stderr even when every diagnostic is
  # suppressed. Replaying that for every clean file overflows pre-commit's
  # output buffer (BlockingIOError on sys.stdout.buffer.write). Keep stdout
  # silent for clean runs and only print the captured output on failure.
  output=$(clang-tidy -p "$(dirname "$COMPILE_DB")" --quiet --config-file="$CLANG_TIDY_CONFIG" "$ABS_FILE" 2>&1) || rc=$?
  rc=${rc:-0}
  if [ "$rc" -ne 0 ]; then
    echo "clang-tidy: $file"
    echo "$output"
    ERRORS=$((ERRORS + 1))
  fi
  unset rc
done

if [ "$ERRORS" -gt 0 ]; then
  echo "clang-tidy found issues in $ERRORS file(s)"
  exit 1
fi
