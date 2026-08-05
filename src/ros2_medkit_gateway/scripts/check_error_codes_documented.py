#!/usr/bin/env python3
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

"""
Every error code the gateway can emit must appear in the REST guide.

Why this exists rather than a review habit: the error-code table is prose, and
prose carries completeness claims ("the full set", "every refusal") that
nothing checks. Four review rounds on this branch produced zero logic defects
and a run of statement defects, every one of them a universal claim written
where no test could reach it. This is the reach.

The rule is one-directional on purpose. A code with a non-test emitter must be
documented, because a client can receive it. A documented code with no emitter
is not an error here - a plugin backend can raise codes this repository never
mentions, and the guide is allowed to describe them.

Exclusions are declared in ``UNREACHABLE`` below, each with the reason it
cannot reach a client. Adding a code there is a claim about the code path, so
it needs the same evidence any other claim does.
"""

import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[3]
HEADER = (
    REPO / 'src/ros2_medkit_gateway/include/ros2_medkit_gateway/core/http/error_codes.hpp'
)
GUIDE = REPO / 'docs/api/rest.rst'

# Heading whose list-table this check reads. Only the table's first column
# counts as "documented": a passing mention of a code elsewhere in the guide
# does not put it in the table, and an earlier version of this script matched
# any inline literal anywhere in the file - which let six emitted codes pass
# while absent from the table, and made the table's completeness sentence true
# by accident rather than by check.
TABLE_HEADING = 'Common Error Codes'

# Every tree that can put an error code on the wire. Headers included because
# `typed_router.hpp`, `handler_context.hpp` and several dto headers name codes
# directly; in-tree plugins included because they serve our own routes - the
# guide's "a plugin backend may raise codes this list does not mention" caveat
# is about third-party plugins, not ours.
SOURCE_ROOTS = (
    REPO / 'src/ros2_medkit_gateway/src',
    REPO / 'src/ros2_medkit_gateway/include',
    REPO / 'src/ros2_medkit_plugins',
)

# Codes with an emitter that a client can nonetheless never receive. Each entry
# states why, and the reason has to be a property of the code path - not
# "nobody got round to documenting it".
# Only ``ERR_*`` values belong here. ``LockManager``'s internal strings
# ("lock-conflict", "lock-disabled", ...) are not declared in the header at
# all - they are mapped to SOVD codes by ``to_sovd_error_code`` before they
# reach a response - so this check never sees them and must not name them.
UNREACHABLE = {
    # write_typed_error returns before rendering when it sees this sentinel:
    # the peer-forwarding path has already committed the wire response. The
    # declaration says "Never appears on the wire" and the framework enforces
    # it (route_registry.cpp, the ERR_X_INTERNAL_FORWARDED early return).
    'x-medkit-internal-forwarded',
}


def declared_codes():
    """Return {constant name: wire value} for every ERR_* in the header."""
    text = HEADER.read_text(encoding='utf-8')
    return dict(
        re.findall(r'constexpr\s+const\s+char\s*\*\s*(ERR_\w+)\s*=\s*"([^"]+)"', text)
    )


def source_files():
    """Yield every non-test C++ file that could name an error code."""
    for root in SOURCE_ROOTS:
        for pattern in ('*.cpp', '*.hpp'):
            for path in sorted(root.rglob(pattern)):
                if 'test' in path.parts or path.name.startswith('test_'):
                    continue
                if path == HEADER:
                    continue  # the declarations themselves, not an emitter
                yield path


def emitted_codes(codes):
    """Return {wire value: first source file} for codes named outside tests."""
    found = {}
    for path in source_files():
        text = path.read_text(encoding='utf-8')
        for name, value in codes.items():
            if value in found:
                continue
            if re.search(r'\b' + re.escape(name) + r'\b', text):
                found[value] = path.relative_to(REPO)
    return found


def documented_codes():
    """Return the first-column values of the Common Error Codes table."""
    lines = GUIDE.read_text(encoding='utf-8').splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.strip() == TABLE_HEADING)
    except StopIteration:
        return set()

    # Scan from the `.. list-table::` that follows the heading, not from the
    # heading itself - the line directly under it is the RST underline, which
    # is unindented and would end the scan immediately.
    try:
        table = next(
            i for i, ln in enumerate(lines[start:], start) if ln.startswith('.. list-table::')
        )
    except StopIteration:
        return set()

    # The table ends at the first line that is neither blank nor indented -
    # the next top-level block. Only `* - ``code``` lines count: those are
    # list-table row keys, i.e. the table's first column.
    documented = set()
    for line in lines[table + 1:]:
        if line and not line.startswith(' '):
            break
        match = re.match(r'\s*\* - ``([a-z0-9][a-z0-9-]*)``\s*$', line)
        if match:
            documented.add(match.group(1))
    return documented


def main():
    codes = declared_codes()
    if not codes:
        print('FAIL: parsed no ERR_* constants - has error_codes.hpp moved?', file=sys.stderr)
        return 1

    emitted = emitted_codes(codes)
    documented = documented_codes()

    # Positive control. A parser that silently matches nothing reports every
    # code as missing, which reads like a real failure and is just as useless
    # as one that matches everything. Both of those bugs shipped in earlier
    # drafts of this script, so the parse is checked before its result is used.
    if len(documented) < 10:
        print(
            f'FAIL: parsed only {len(documented)} row(s) from the '
            f'"{TABLE_HEADING}" table - the parser, not the table, is broken.',
            file=sys.stderr,
        )
        return 1

    stale = sorted(UNREACHABLE - set(emitted))
    missing = sorted(
        (value, path)
        for value, path in emitted.items()
        if value not in documented and value not in UNREACHABLE
    )

    if missing:
        print(
            f'FAIL: {len(missing)} error code(s) can reach a client but are absent '
            f'from {GUIDE.relative_to(REPO)}:',
            file=sys.stderr,
        )
        for value, path in missing:
            print(f'  {value:38s} emitted by {path}', file=sys.stderr)
        print(
            '\nAdd a row to the "Common Error Codes" table, or - if the code '
            'genuinely cannot reach a client - add it to UNREACHABLE in this '
            'script with the reason.',
            file=sys.stderr,
        )
        return 1

    if stale:
        print(
            'FAIL: UNREACHABLE names code(s) nothing emits any more, so the '
            'stated reason can no longer be checked: ' + ', '.join(stale),
            file=sys.stderr,
        )
        return 1

    checked = len(emitted) - len(UNREACHABLE & set(emitted))
    print(
        f'OK: {checked} of {len(codes)} declared error code(s) reach a client '
        f'and are documented ({len(codes) - len(emitted)} have no emitter, '
        f'{len(UNREACHABLE & set(emitted))} excluded as unreachable)'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
