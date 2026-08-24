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

The check reads the layout rather than any ``CMakeLists.txt``. That is a
deliberate trade, and it is worth being exact about which way it errs.

It is STRICTER than "what is registered today": a directory holding both kinds
of file fails even if nothing currently points pytest at it. That is the point.
Registration moves, and the shape is what breaks when it does.

It is also INCOMPLETE, and reading CMake would not close the gap either.
``ament_add_pytest_test`` accepts a directory, and this repository uses that
form, so a launch test can be collected next to a file this check does not
recognise as a plain test - one that does not start with ``test_``, or a
``python_files`` setting that widens what pytest collects. Those are not
guarded here. What is guarded is the one shape that has actually broken the
build, twice.
"""

import pathlib

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent

# Directories that hold no test sources of ours and would only add noise.
PRUNED = {'build', 'install', 'log', '__pycache__', '.git', 'node_modules'}


def find_source_root():
    """Return the directory the packages sit in, and whether it is the workspace.

    In a workspace that is ``src``. Without one - a package unpacked on its own,
    as the build farm does - the fallback is this package and nothing above it.
    Walking the parent instead would sweep whatever else that directory happens
    to hold, which on a packaging worker is other projects and their build
    output, and this check would then fail on a layout that is none of our
    business.
    """
    for candidate in PACKAGE_ROOT.parents:
        if (candidate / 'src' / PACKAGE_ROOT.name / 'package.xml').is_file():
            return candidate / 'src', True
    return PACKAGE_ROOT, False


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
    source_root, _ = find_source_root()
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


def test_the_sweep_reaches_packages_other_than_this_one():
    """Guards the instrument: a sweep that finds nothing proves nothing.

    The assertion above passes trivially if the walk resolved to a directory
    holding no tests, and that failure mode looks exactly like success. Asking
    only whether this package's own tests were seen would not catch it: this
    file is itself one of them and sits in the directory the sweep starts from,
    so that answer is yes however badly the root resolved.

    What a correct workspace sweep can show and a broken one cannot is reach
    into OTHER packages. Without a workspace there are none to reach, and the
    honest report is then how far it did get.
    """
    source_root, is_workspace = find_source_root()
    launch_tests, plain_tests = collect_by_directory(source_root)

    packages = {
        directory.relative_to(source_root).parts[0]
        for directory in set(launch_tests) | set(plain_tests)
        if directory != source_root
    }
    others = sorted(packages - {PACKAGE_ROOT.name})

    if not is_workspace:
        assert source_root == PACKAGE_ROOT, (
            f'without a workspace the sweep must stay inside this package, but '
            f"it was rooted at '{source_root}'"
        )
        return

    assert len(others) >= 3, (
        f"the sweep rooted at '{source_root}' reached tests in {len(others)} "
        f'package(s) besides this one, so it is not covering the workspace and '
        f'its verdict says nothing about it; reached: {others}'
    )
