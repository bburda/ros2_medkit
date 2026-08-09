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

"""The guard on the shared lint configs being findable wherever the package is built.

The case this exists for is not the workspace, where every path resolves and
everything looks fine. It is the build of a single exported package directory
that a binary release performs, where nothing above the package exists. A
config addressed as ``${CMAKE_CURRENT_SOURCE_DIR}/../..`` is simply absent
there, and because bloom runs the test step as ``dh_auto_test || true`` the
resulting failure cannot stop the build. That is a linter that has failed on
every binary build of this package and reported it to nobody.

So the tests below run in both tree shapes on purpose, and the ones that carry
the deb-relevant claim - the configs ship with the package, the resolver finds
them, the resolver refuses to be quiet when it cannot - are exactly the ones
that do not depend on a repository being there.
"""

import os
import pathlib
import re
import shutil
import subprocess
import sys

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent
CMAKE_DIR = PACKAGE_ROOT / 'cmake'
LINTING_MODULE = CMAKE_DIR / 'ROS2MedkitLinting.cmake'

# The configs this package promises to ship. Every ros2_medkit package resolves
# its linter configuration to one of these through medkit_lint_config().
SHARED_CONFIGS = ('.clang-format', '.clang-tidy', '.flake8')

# A shared config named on the same line as a path segment that leaves the
# package directory. This is the defect in its general form: it resolves in a
# workspace checkout and in no other tree.
CONFIG_NAMED = re.compile('|'.join(re.escape(name) for name in SHARED_CONFIGS))
PARENT_SEGMENT = re.compile(r'(?:^|[\s"/])\.\.(?:[/\s"]|$)')

PROBE_TEMPLATE = """cmake_minimum_required(VERSION 3.8)
project(medkit_lint_config_probe NONE)
include("{module}")
medkit_lint_config({config} _resolved)
message(STATUS "PROBE_RESOLVED=${{_resolved}}")
"""


def find_repository_root():
    """Return the repository root, or None when only this package is present."""
    for candidate in PACKAGE_ROOT.parents:
        if (candidate / 'src' / PACKAGE_ROOT.name / 'package.xml').is_file():
            return candidate
    return None


def run_probe(tmp_path, module, config):
    """Configure a throwaway project that resolves one config, and return the result."""
    source = tmp_path / 'probe'
    source.mkdir()
    (source / 'CMakeLists.txt').write_text(
        PROBE_TEMPLATE.format(module=module.as_posix(), config=config)
    )
    return subprocess.run(
        [shutil.which('cmake') or 'cmake', '-S', str(source), '-B', str(tmp_path / 'build')],
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )


@pytest.mark.parametrize('name', SHARED_CONFIGS)
def test_config_ships_next_to_the_cmake_modules(name):
    """Each shared config is a real file of this package, not borrowed from above it."""
    path = CMAKE_DIR / name
    assert path.is_file(), (
        f'{name} is missing from {CMAKE_DIR}. The linters of every package in the tree '
        f'resolve to this copy, and it is the only one a binary package build can see.'
    )
    assert path.stat().st_size > 0, f'{path} is empty'


@pytest.mark.parametrize('name', SHARED_CONFIGS)
def test_config_is_installed_next_to_the_cmake_modules(name):
    """Each shared config is installed, so a dependent package resolves it after install."""
    text = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()
    blocks = [
        block
        for block in re.findall(r'install\((.*?)\n\)', text, re.S)
        if 'DESTINATION share/${PROJECT_NAME}/cmake' in block
    ]
    assert blocks, 'no install() block targets the cmake share directory'
    installed = '\n'.join(blocks)
    assert f'cmake/{name}' in installed, (
        f'{name} is not installed into share/ros2_medkit_cmake/cmake. Without it every '
        f'other package resolves its linter config to a path that does not exist.'
    )


def test_resolver_returns_the_shipped_config(tmp_path):
    """medkit_lint_config() resolves to the copy that ships next to the modules."""
    result = run_probe(tmp_path, LINTING_MODULE, '.flake8')
    assert result.returncode == 0, result.stderr
    resolved = re.search(r'PROBE_RESOLVED=(\S+)', result.stdout)
    assert resolved, result.stdout
    assert pathlib.Path(resolved.group(1)) == CMAKE_DIR / '.flake8'


def test_resolver_fails_loudly_when_the_config_is_missing(tmp_path):
    """A config that cannot be found stops the build instead of disabling the linter.

    The whole point of the change this pins: skipping the linter when its config
    is absent would swap a build that fails on every binary release for one that
    silently stops checking, which is the worse of the two.
    """
    stripped = tmp_path / 'modules'
    stripped.mkdir()
    shutil.copy(LINTING_MODULE, stripped / LINTING_MODULE.name)

    result = run_probe(tmp_path, stripped / LINTING_MODULE.name, '.flake8')
    assert result.returncode != 0, (
        'configuring succeeded with no config next to the module, so a package built '
        f'without {LINTING_MODULE.name} companions would lint against nothing:\n'
        f'{result.stdout}'
    )
    assert '.flake8' in result.stderr, result.stderr


def test_no_cmake_file_reaches_outside_its_package_for_a_lint_config():
    """No package addresses a lint config through a path that leaves its own directory.

    Repository-wide when the repository is there. When only this package is - a
    binary package build - there is no sibling to sweep, so the assertion is
    that the tree really has that shape rather than that the sweep found
    nothing.
    """
    repository_root = find_repository_root()
    if repository_root is None:
        siblings = [
            entry
            for entry in PACKAGE_ROOT.parent.iterdir()
            if entry != PACKAGE_ROOT and (entry / 'package.xml').is_file()
        ]
        assert not siblings, (
            'no repository root was found above this package, yet these sibling packages '
            f'exist and went unswept: {sorted(str(s) for s in siblings)}'
        )
        return

    offenders = []
    scanned = 0
    for path in sorted((repository_root / 'src').rglob('*')):
        if path.name != 'CMakeLists.txt' and path.suffix != '.cmake':
            continue
        scanned += 1
        for number, line in enumerate(path.read_text().splitlines(), start=1):
            if line.lstrip().startswith('#'):
                continue
            if CONFIG_NAMED.search(line) and PARENT_SEGMENT.search(line):
                offenders.append(f'{path.relative_to(repository_root)}:{number}: {line.strip()}')

    # A floor well under the current count, so the sweep cannot pass by having
    # walked an empty tree, and does not fail the day a package is retired.
    assert scanned >= 15, f'only {scanned} cmake files scanned under {repository_root / "src"}'
    assert not offenders, (
        'these reach outside their own package for a lint config, which resolves in a '
        'workspace checkout and in no other tree. Use medkit_lint_config() instead:\n'
        + '\n'.join(offenders)
    )


def test_there_is_one_copy_of_each_config():
    """The repository root paths and the shipped copies are the same file.

    Editors, pre-commit and scripts/clang-tidy-diff.sh all address the configs
    at the repository root. Two real files there and here would be hand-synced
    copies that drift without anything saying so. In a single package export
    there is no root path, and the claim that has to hold instead is that the
    export carries the configs itself rather than a link out of the tarball.
    """
    repository_root = find_repository_root()
    if repository_root is None:
        for name in SHARED_CONFIGS:
            path = CMAKE_DIR / name
            assert not path.is_symlink(), (
                f'{name} is a link inside an export with no repository above it, so it '
                f'points at nothing: {os.readlink(path)}'
            )
        return

    for name in SHARED_CONFIGS:
        root_path = repository_root / name
        assert root_path.is_symlink(), (
            f'{name} at the repository root is not a link to the copy this package ships; '
            f'two real files drift apart in silence'
        )
        assert os.path.realpath(root_path) == str(CMAKE_DIR / name)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
