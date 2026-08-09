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

"""The guard that catches a test which never goes through the wrapper.

Driven against real generated CTest properties, because that is the only input
it ever sees: a hand-written ``CTestTestfile.cmake`` is read by ``ctest
--show-only=json-v1`` exactly the way a package's generated one is.
"""

import os
import subprocess
import sys

import medkit_domain
import pytest

CHECK = os.path.join(medkit_domain.SCRIPT_DIR, 'check_test_domains.py')
RUNNER = os.path.join(medkit_domain.SCRIPT_DIR, 'medkit_domain_runner.py')
EXEC_WRAPPER = os.path.join(medkit_domain.SCRIPT_DIR, 'medkit_run_with_domain.py')
SELF_TEST = 'test_dds_domain_allocation'


def write_ctest_file(directory, body):
    (directory / 'CTestTestfile.cmake').write_text(body)


def run_check(directory, package='ros2_medkit_example', env=None):
    child_env = dict(os.environ)
    if env:
        child_env.update(env)
    return subprocess.run(
        [sys.executable, CHECK, '--build-dir', str(directory), '--package', package],
        capture_output=True,
        text=True,
        timeout=120,
        env=child_env,
        check=False,
    )


def wrapped_test(name, domains=1):
    return (
        f'add_test({name} "/usr/bin/python3" "-u" "{RUNNER}" "{name}.xml" '
        f'"--package-name" "ros2_medkit_example" "--command" "/bin/true")\n'
        f'set_tests_properties({name} PROPERTIES '
        f'ENVIRONMENT "MEDKIT_TEST_DOMAINS={domains};MEDKIT_TEST_DOMAIN_WAIT=30" '
        f'LABELS "unit" TIMEOUT 60)\n'
    )


def test_a_test_that_skips_the_wrapper_is_reported(tmp_path):
    write_ctest_file(
        tmp_path,
        wrapped_test('test_wrapped')
        + 'add_test(test_bare "/bin/true")\n'
        'set_tests_properties(test_bare PROPERTIES LABELS "unit")\n',
    )
    result = run_check(tmp_path)
    assert result.returncode != 0
    assert 'test_bare' in result.stdout
    assert 'test_wrapped' not in result.stdout.split('FAIL')[-1]


def test_a_fully_wrapped_package_passes(tmp_path):
    write_ctest_file(tmp_path, wrapped_test('test_one') + wrapped_test('test_two', domains=3))
    result = run_check(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_exec_wrapper_also_counts_as_going_through_the_wrapper(tmp_path):
    write_ctest_file(
        tmp_path,
        f'add_test(test_exec "/usr/bin/python3" "{EXEC_WRAPPER}" "--" "/bin/true")\n'
        'set_tests_properties(test_exec PROPERTIES '
        'ENVIRONMENT "MEDKIT_TEST_DOMAINS=1;MEDKIT_TEST_DOMAIN_WAIT=30" '
        'LABELS "integration" TIMEOUT 60)\n',
    )
    result = run_check(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_an_explicit_waiver_is_accepted_and_listed(tmp_path):
    write_ctest_file(
        tmp_path,
        wrapped_test('test_wrapped')
        + 'add_test(test_ros_free "/bin/true")\n'
        'set_tests_properties(test_ros_free PROPERTIES LABELS "unit;no_ros_domain")\n',
    )
    result = run_check(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'test_ros_free' in result.stdout


def test_linter_tests_are_not_expected_to_hold_a_domain(tmp_path):
    write_ctest_file(
        tmp_path,
        wrapped_test('test_wrapped')
        + 'add_test(flake8 "/bin/true")\n'
        'set_tests_properties(flake8 PROPERTIES LABELS "linter")\n',
    )
    result = run_check(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_a_package_whose_every_test_lost_its_wrapper_is_reported(tmp_path):
    write_ctest_file(
        tmp_path,
        'add_test(test_one "/bin/true")\n'
        'set_tests_properties(test_one PROPERTIES LABELS "unit")\n',
    )
    result = run_check(tmp_path)
    assert result.returncode != 0
    assert 'test_one' in result.stdout


def test_a_wrapped_test_whose_domain_count_was_dropped_is_reported(tmp_path):
    # medkit_add_gtest APPENDs MEDKIT_TEST_DOMAINS; a later
    # set_tests_properties(... PROPERTIES ENVIRONMENT ...) replaces it. The
    # command still goes through the wrapper, so only the count can show it.
    write_ctest_file(
        tmp_path,
        f'add_test(test_clobbered "/usr/bin/python3" "-u" "{RUNNER}" "x.xml" '
        f'"--package-name" "ros2_medkit_example" "--command" "/bin/true")\n'
        'set_tests_properties(test_clobbered PROPERTIES '
        'ENVIRONMENT "GATEWAY_TEST_PORT=9100" LABELS "unit")\n',
    )
    result = run_check(tmp_path)
    assert result.returncode != 0
    assert 'test_clobbered' in result.stdout
    assert 'MEDKIT_TEST_DOMAINS' in result.stdout


def test_a_package_with_no_tests_at_all_is_accepted(tmp_path):
    # The gate arms itself in every package now, including ones with nothing to
    # gate. Failing those would make the default-on registration unusable, and
    # a package with no tests has no test on domain 0 either. The workspace
    # sweep is what refuses to pass on an empty result, see below.
    write_ctest_file(tmp_path, '')
    result = run_check(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_check_fails_when_ctest_cannot_read_the_directory(tmp_path):
    missing = tmp_path / 'nowhere'
    with pytest.raises(Exception):
        subprocess.run(
            [sys.executable, CHECK, '--build-dir', str(missing), '--package', 'x'],
            capture_output=True,
            text=True,
            timeout=60,
            check=True,
        )


def test_a_waiver_lost_to_a_label_overwrite_is_reported(tmp_path):
    # medkit_test_needs_no_domain APPENDs the label; a set_tests_properties(...
    # PROPERTIES LABELS ...) after it replaces the whole list.
    write_ctest_file(
        tmp_path,
        wrapped_test('test_wrapped')
        + 'add_test(test_ros_free "/bin/true")\n'
        'set_tests_properties(test_ros_free PROPERTIES LABELS "unit")\n',
    )
    result = run_check(tmp_path)
    assert result.returncode != 0
    assert 'test_ros_free' in result.stdout


def run_sweep(workspace_dir):
    return subprocess.run(
        [sys.executable, CHECK, '--workspace-build-dir', str(workspace_dir)],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )


def gate_test(package):
    return (
        f'add_test({SELF_TEST} "/usr/bin/python3" "{CHECK}" "--build-dir" "." '
        f'"--package" "{package}")\n'
    )


def test_the_sweep_reports_a_package_that_carries_no_gate_at_all(tmp_path):
    # The case a per-package gate can never see: the package never found
    # ros2_medkit_cmake, so nothing armed a gate inside it.
    gated = tmp_path / 'pkg_gated'
    gated.mkdir()
    write_ctest_file(gated, wrapped_test('test_ok') + gate_test('pkg_gated'))
    ungated = tmp_path / 'pkg_outside'
    ungated.mkdir()
    write_ctest_file(
        ungated,
        'add_test(test_stray "/bin/true")\n'
        'set_tests_properties(test_stray PROPERTIES LABELS "unit")\n',
    )
    result = run_sweep(tmp_path)
    assert result.returncode != 0
    assert 'pkg_outside' in result.stdout
    assert SELF_TEST in result.stdout


def test_the_sweep_accepts_a_package_whose_only_tests_are_linters(tmp_path):
    # An interface-only package needs no gate, because it has nothing to gate.
    gated = tmp_path / 'pkg_gated'
    gated.mkdir()
    write_ctest_file(gated, wrapped_test('test_ok') + gate_test('pkg_gated'))
    linters = tmp_path / 'pkg_linters_only'
    linters.mkdir()
    write_ctest_file(
        linters,
        'add_test(xmllint "/bin/true")\n'
        'set_tests_properties(xmllint PROPERTIES LABELS "linter")\n',
    )
    result = run_sweep(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_sweep_reports_an_unwrapped_test_in_any_package(tmp_path):
    package = tmp_path / 'pkg_gated'
    package.mkdir()
    write_ctest_file(
        package,
        wrapped_test('test_ok')
        + gate_test('pkg_gated')
        + 'add_test(test_bare "/bin/true")\n'
        'set_tests_properties(test_bare PROPERTIES LABELS "unit")\n',
    )
    result = run_sweep(tmp_path)
    assert result.returncode != 0
    assert 'test_bare' in result.stdout


def test_a_sweep_that_found_nothing_is_a_failure_not_a_pass(tmp_path):
    result = run_sweep(tmp_path)
    assert result.returncode != 0
    assert 'proved nothing' in result.stdout


def test_the_sweep_ignores_packages_from_another_repository(tmp_path):
    # An overlay workspace that source-builds one of our dependencies next to us
    # produces build directories that look exactly like ours. Telling that
    # maintainer to rewrite their package would be wrong and rude.
    ours = tmp_path / 'build' / 'our_package'
    ours.mkdir(parents=True)
    theirs = tmp_path / 'build' / 'their_package'
    theirs.mkdir(parents=True)
    repo = tmp_path / 'repo'
    (repo / 'our_package').mkdir(parents=True)
    (tmp_path / 'elsewhere' / 'their_package').mkdir(parents=True)

    write_ctest_file(
        ours,
        f'# Source directory: {repo / "our_package"}\n'
        + wrapped_test('test_ok')
        + gate_test('our_package'),
    )
    write_ctest_file(
        theirs,
        f'# Source directory: {tmp_path / "elsewhere" / "their_package"}\n'
        'add_test(test_not_ours "/bin/true")\n'
        'set_tests_properties(test_not_ours PROPERTIES LABELS "unit")\n',
    )
    result = subprocess.run(
        [
            sys.executable, CHECK,
            '--workspace-build-dir', str(tmp_path / 'build'),
            '--repo-root', str(repo),
        ],
        capture_output=True, text=True, timeout=120, check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'their_package' not in result.stdout
    assert 'test_not_ours' not in result.stdout


def test_the_sweep_still_reports_our_own_ungated_package(tmp_path):
    repo = tmp_path / 'repo'
    (repo / 'gated').mkdir(parents=True)
    (repo / 'ungated').mkdir(parents=True)
    gated = tmp_path / 'build' / 'gated'
    gated.mkdir(parents=True)
    ungated = tmp_path / 'build' / 'ungated'
    ungated.mkdir(parents=True)
    write_ctest_file(
        gated,
        f'# Source directory: {repo / "gated"}\n'
        + wrapped_test('test_ok') + gate_test('gated'),
    )
    write_ctest_file(
        ungated,
        f'# Source directory: {repo / "ungated"}\n'
        'add_test(test_stray "/bin/true")\n'
        'set_tests_properties(test_stray PROPERTIES LABELS "unit")\n',
    )
    result = subprocess.run(
        [
            sys.executable, CHECK,
            '--workspace-build-dir', str(tmp_path / 'build'),
            '--repo-root', str(repo),
        ],
        capture_output=True, text=True, timeout=120, check=False,
    )
    assert result.returncode != 0
    assert 'ungated' in result.stdout


def wrapped_test_with(name, domains=1, wait=None, timeout=None):
    body = (
        f'add_test({name} "/usr/bin/python3" "-u" "{RUNNER}" "{name}.xml" '
        f'"--package-name" "ros2_medkit_example" "--command" "/bin/true")\n'
    )
    env = [f'MEDKIT_TEST_DOMAINS={domains}']
    if wait is not None:
        env.append(f'MEDKIT_TEST_DOMAIN_WAIT={wait}')
    props = f'ENVIRONMENT "{";".join(env)}" LABELS "unit"'
    if timeout is not None:
        props += f' TIMEOUT {timeout}'
    return body + f'set_tests_properties({name} PROPERTIES {props})\n'


def test_a_wait_budget_longer_than_the_timeout_is_reported(tmp_path):
    # CTest would kill the test while the allocator was still waiting, so the
    # explanation of why no domain could be had would never print.
    write_ctest_file(tmp_path, wrapped_test_with('test_doomed', wait=180, timeout=60))
    result = run_check(tmp_path)
    assert result.returncode != 0
    assert 'test_doomed' in result.stdout
    assert 'killed mid-wait' in result.stdout


def test_a_wrapped_test_with_no_wait_budget_is_reported(tmp_path):
    write_ctest_file(tmp_path, wrapped_test_with('test_unbudgeted', timeout=60))
    result = run_check(tmp_path)
    assert result.returncode != 0
    assert 'test_unbudgeted' in result.stdout
    assert 'MEDKIT_TEST_DOMAIN_WAIT' in result.stdout


def test_a_wait_budget_inside_the_timeout_is_accepted(tmp_path):
    write_ctest_file(tmp_path, wrapped_test_with('test_sane', wait=30, timeout=60))
    result = run_check(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_gate_reports_the_range_it_checked_the_band_against(tmp_path):
    # The band is only correct for a machine whose ip_local_port_range is the
    # default one. Printing what was actually read makes that visible rather
    # than assumed, on the machine that runs the tests.
    write_ctest_file(tmp_path, wrapped_test('test_one'))
    result = run_check(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'ephemeral ports' in result.stdout
    assert medkit_domain.EPHEMERAL_RANGE_FILE in result.stdout
    assert '1-100,215-231' in result.stdout


def test_the_gate_fails_on_a_machine_whose_range_swallows_the_band(tmp_path):
    write_ctest_file(tmp_path, wrapped_test('test_one'))
    result = run_check(
        tmp_path, env={medkit_domain.EPHEMERAL_RANGE_ENV: '1024-65535'}
    )
    assert result.returncode != 0
    assert 'net.ipv4.ip_local_port_range' in result.stdout
    # The tests themselves are fine; it is the machine that cannot host them.
    assert 'test_one' not in result.stdout.split('FAIL')[-1]


def test_the_gate_passes_on_a_narrowed_range_with_a_band_that_fits_it(tmp_path):
    write_ctest_file(tmp_path, wrapped_test('test_one'))
    result = run_check(
        tmp_path,
        env={
            medkit_domain.EPHEMERAL_RANGE_ENV: '10240-65535',
            medkit_domain.BAND_ENV: '1-10',
        },
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert '10240-65535' in result.stdout
