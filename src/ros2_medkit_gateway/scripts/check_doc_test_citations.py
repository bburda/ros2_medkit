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
Every test a design document names by hand has to exist.

The gateway's design docs say which mechanism is enforced by which check, and
they say it by naming the check. That naming is the load-bearing part: "pinned
by ``RouteRegistryTest.RouteDeclaredStatusWinsOverTheMiddlewareComponent``" is
the difference between a documented guarantee and an assertion about nothing.
A rename in the test tree falsifies every such sentence at once, silently,
because nothing links the prose to the test.

This is that link. It reads the citations out of the documents and resolves
each against the test tree.

Three citation forms are recognised, which is exactly what the documents use:

* ``<file>::<test_method>`` or ``<file>.test.py::<test_method>`` - a Python
  launch_testing case. Resolved by finding the file and the ``def``.
* ``<Suite>Test.<CaseName>`` - a C++ GTest case. Resolved against
  ``TEST``/``TEST_F``/``TEST_P`` in the C++ test sources.
* ``test_<name>`` on its own - resolved against any of the three things that
  name could be: a test source file, a Python case, or a GTest case.

The direction is one-way on purpose. A cited test must exist; a test nothing
cites is not a defect. Only the first direction can make a document false.
"""

import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[3]

# Documents whose test citations are checked. The gateway design docs make the
# claims; ``docs/api`` repeats two of them for readers who never open a design
# doc, and a citation is just as load-bearing there.
DOC_ROOTS = (
    REPO / 'src/ros2_medkit_gateway/design',
    REPO / 'docs/api',
)

# Where a test can live. Any ``test`` directory under a package, at any depth -
# the gateway keeps unit tests in ``test/`` and the integration package keeps
# feature tests in ``test/features/``.
SOURCE_ROOT = REPO / 'src'

# Below this many resolvable citations the parser, not the documents, is what
# broke. An earlier check on this branch passed by matching nothing at all;
# this is the guard against repeating that.
MIN_CITATIONS = 15

QUALIFIED = re.compile(r'``([A-Za-z_][A-Za-z0-9_]*?)(?:\.test\.py)?::(test_[A-Za-z0-9_]+)``')
GTEST = re.compile(r'``([A-Za-z][A-Za-z0-9_]*Test)\.([A-Za-z_][A-Za-z0-9_]*)``')
BARE = re.compile(r'``(test_[A-Za-z0-9_]+)``')


def documents():
    """Yield every RST file whose citations this check resolves."""
    for root in DOC_ROOTS:
        for path in sorted(root.rglob('*.rst')):
            yield path


def python_tests():
    """Return {file stem: set of test method names} for the Python tests."""
    cases = {}
    for path in sorted(SOURCE_ROOT.rglob('*.test.py')):
        if 'test' not in path.parts:
            continue
        stem = path.name[: -len('.test.py')]
        text = path.read_text(encoding='utf-8')
        cases[stem] = set(re.findall(r'^\s*def (test_[A-Za-z0-9_]+)\s*\(', text, re.M))
    return cases


GTEST_CASE = re.compile(
    r'\bTEST(?:_F|_P)?\(\s*([A-Za-z_][A-Za-z0-9_]*)\s*,\s*([A-Za-z_][A-Za-z0-9_]*)\s*\)'
)


def gtest_cases():
    """Return the set of (suite, case) pairs declared in the C++ tests."""
    found = set()
    for path in sorted(SOURCE_ROOT.rglob('*.cpp')):
        if 'test' not in path.parts:
            continue
        for suite, case in GTEST_CASE.findall(path.read_text(encoding='utf-8')):
            found.add((suite, case))
    return found


def test_file_stems():
    """Return every basename a test source file is known by."""
    stems = set()
    for path in sorted(SOURCE_ROOT.rglob('*')):
        if 'test' not in path.parts or not path.is_file():
            continue
        if path.name.endswith('.test.py'):
            stems.add(path.name[: -len('.test.py')])
        elif path.suffix in ('.cpp', '.py'):
            stems.add(path.stem)
    return stems


def main():
    py = python_tests()
    gtests = gtest_cases()
    stems = test_file_stems()

    # Positive controls on each corpus, before any of them is used to judge a
    # document. A resolver that silently found nothing reports every citation
    # as broken, which reads like a real failure and proves as little as one
    # that matches everything.
    if not py:
        print('FAIL: found no Python test files - has the test layout moved?', file=sys.stderr)
        return 1
    if not gtests:
        print('FAIL: found no GTest cases - has the test layout moved?', file=sys.stderr)
        return 1

    checked = 0
    broken = []
    for doc in documents():
        rel = doc.relative_to(REPO)
        text = doc.read_text(encoding='utf-8')

        for match in QUALIFIED.finditer(text):
            stem, case = match.group(1), match.group(2)
            checked += 1
            if stem not in py:
                broken.append(f'{rel}: no test file `{stem}.test.py` for `{stem}::{case}`')
            elif case not in py[stem]:
                broken.append(f'{rel}: `{stem}.test.py` has no `{case}`')

        for match in GTEST.finditer(text):
            suite, case = match.group(1), match.group(2)
            checked += 1
            if (suite, case) not in gtests:
                broken.append(f'{rel}: no GTest case `{suite}.{case}`')

        for match in BARE.finditer(text):
            name = match.group(1)
            checked += 1
            resolved = (
                name in stems
                or any(name in cases for cases in py.values())
                or any(case == name for _, case in gtests)
            )
            if not resolved:
                broken.append(f'{rel}: `{name}` names no test file, Python case or GTest case')

    if checked < MIN_CITATIONS:
        print(
            f'FAIL: parsed only {checked} test citation(s) from '
            f'{len(list(documents()))} document(s) - the parser, not the docs, is broken.',
            file=sys.stderr,
        )
        return 1

    if broken:
        print(
            f'FAIL: {len(broken)} document citation(s) name a test that does not exist:',
            file=sys.stderr,
        )
        for line in broken:
            print(f'  {line}', file=sys.stderr)
        print(
            '\nA renamed test leaves the sentence that cites it asserting nothing. '
            'Update the citation, or restore the name.',
            file=sys.stderr,
        )
        return 1

    print(f'OK: {checked} test citation(s) in the documents all resolve')
    return 0


if __name__ == '__main__':
    sys.exit(main())
