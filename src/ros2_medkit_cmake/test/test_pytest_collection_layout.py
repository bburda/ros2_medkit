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

"""A launch test must not share a directory with a plain pytest file.

``launch_testing``'s collection hook runs for every file pytest considers, and
for a ``*.test.py`` it imports the file to find the launch entry point. The
module name it derives keeps the dot - ``foo.test.py`` becomes ``foo.test`` -
so the import asks for a package named ``foo`` that does not exist, and pytest
reports ``ModuleNotFoundError: No module named 'foo'``. The error is raised
against the DIRECTORY, so it takes down the collection of every plain pytest
file beside it, whichever one was actually asked for.

Nothing about that is specific to the file being imported: any ``*.test.py`` in
the directory is enough, which is why the shape has to be guarded rather than
the file. It has bitten this repository twice. It first appeared when a plain
pytest file joined a directory of launch tests, and it came back after a move
that took seven launch tests out of that directory and left the eighth, so the
same failure returned naming the file that stayed.

The check is a fact about the layout, deliberately not a reading of any
``CMakeLists.txt``: a directory either holds both kinds of file or it does not,
and no build configuration changes the answer.
"""

import pathlib

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent

# Directories that hold no test sources of ours and would only add noise.
PRUNED = {'build', 'install', 'log', '__pycache__', '.git', 'node_modules'}


def find_source_root():
    """Return the directory the packages sit in.

    In a workspace that is ``src``; in a build of this package alone it is
    whatever directory contains it, and the sweep below then covers exactly
    what is present. Either way the answer is a real tree, so the check never
    silently covers nothing.
    """
    for candidate in PACKAGE_ROOT.parents:
        if (candidate / 'src' / PACKAGE_ROOT.name / 'package.xml').is_file():
            return candidate / 'src'
    return PACKAGE_ROOT.parent


def collect_by_directory(source_root):
    """Map each directory to its launch tests and its plain pytest files."""
    launch_tests = {}
    plain_tests = {}
    for path in source_root.rglob('*.py'):
        if PRUNED & set(path.parts):
            continue
        parent = path.parent
        if path.name.endswith('.test.py'):
            launch_tests.setdefault(parent, []).append(path.name)
        elif path.name.startswith('test_'):
            plain_tests.setdefault(parent, []).append(path.name)
    return launch_tests, plain_tests


def test_no_launch_test_shares_a_directory_with_a_plain_pytest_file():
    """The layout that breaks pytest collection does not exist in the tree."""
    source_root = find_source_root()
    launch_tests, plain_tests = collect_by_directory(source_root)

    collisions = sorted(set(launch_tests) & set(plain_tests))
    described = [
        f'{directory.relative_to(source_root)}: '
        f'launch={sorted(launch_tests[directory])} '
        f'plain={sorted(plain_tests[directory])}'
        for directory in collisions
    ]

    assert described == [], (
        'a launch test shares a directory with a plain pytest file, so pytest '
        'cannot collect that directory and every plain test in it fails with '
        'ModuleNotFoundError before any of its assertions run. Move the '
        'launch test into an integration/ subdirectory beside it. Offending '
        f'directories: {described}'
    )


def test_the_sweep_reaches_the_packages_it_claims_to_cover():
    """Guards the instrument: a sweep that finds nothing proves nothing.

    The assertion above passes trivially if the walk resolved to an empty or
    wrong directory, and that failure mode looks exactly like success. Requiring
    that the sweep saw this package's own tests keeps it honest.
    """
    source_root = find_source_root()
    _, plain_tests = collect_by_directory(source_root)

    assert PACKAGE_ROOT / 'test' in plain_tests, (
        f"the sweep rooted at '{source_root}' did not reach "
        f"'{PACKAGE_ROOT / 'test'}', so it covered nothing and its verdict is "
        f'meaningless; directories seen: {sorted(map(str, plain_tests))}'
    )
