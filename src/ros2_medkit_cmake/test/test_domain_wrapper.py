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

"""The wrapper processes, driven the way CTest drives them.

Everything here starts real processes, because the property under test is that
the domain is held by a process that lives exactly as long as the command it
wraps - which no in-process assertion can show.
"""

import json
import os
import signal
import socket
import subprocess
import sys
import time

import medkit_domain
import pytest
from test_domain_band import reserve_band

RUNNER = os.path.join(medkit_domain.SCRIPT_DIR, 'medkit_domain_runner.py')
EXEC_WRAPPER = os.path.join(medkit_domain.SCRIPT_DIR, 'medkit_run_with_domain.py')

# Prints the domain variables the wrapper exported, as JSON, then exits.
REPORT_ENV = (
    'import json, os, sys; '
    "sys.stdout.write(json.dumps({'primary': os.environ.get('ROS_DOMAIN_ID'), "
    "'secondary': os.environ.get('MEDKIT_SECONDARY_DOMAINS')}))"
)


def run_wrapped(args, command, env=None, timeout=120):
    """Run the exec wrapper over *command* and return the CompletedProcess."""
    child_env = dict(os.environ)
    if env:
        child_env.update(env)
    return subprocess.run(
        [sys.executable, EXEC_WRAPPER] + args + ['--'] + command,
        capture_output=True,
        text=True,
        env=child_env,
        timeout=timeout,
        check=False,
    )


def reported_domains(completed):
    """Return (primary, [secondary...]) from a REPORT_ENV child's stdout."""
    payload = json.loads(completed.stdout.strip().splitlines()[-1])
    secondary = payload['secondary']
    extras = [int(part) for part in secondary.split(',')] if secondary else []
    return int(payload['primary']), extras


def port_is_free(domain):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(('', medkit_domain.PORT_BASE + domain))
    except OSError:
        return False
    finally:
        sock.close()
    return True


def wait_until(predicate, timeout, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def test_exec_wrapper_hands_the_command_a_domain_from_the_band():
    result = run_wrapped([], [sys.executable, '-c', REPORT_ENV])
    assert result.returncode == 0, result.stderr
    primary, extras = reported_domains(result)
    assert primary in medkit_domain.USABLE_DOMAINS
    assert extras == []


def test_exec_wrapper_hands_out_several_distinct_domains():
    result = run_wrapped(['--domains', '4'], [sys.executable, '-c', REPORT_ENV])
    assert result.returncode == 0, result.stderr
    primary, extras = reported_domains(result)
    allocated = [primary] + extras
    assert len(extras) == 3
    assert len(set(allocated)) == 4, f'domains repeated: {allocated}'
    for domain in allocated:
        assert domain in medkit_domain.USABLE_DOMAINS


def test_exec_wrapper_passes_the_command_exit_code_through():
    result = run_wrapped([], [sys.executable, '-c', 'raise SystemExit(7)'])
    assert result.returncode == 7


def test_exec_wrapper_overrides_a_domain_inherited_from_the_environment():
    # ament_add_ros_isolated_gtest skips allocation when ROS_DOMAIN_ID is
    # already set, which would collapse a whole run onto one domain.
    result = run_wrapped(
        [], [sys.executable, '-c', REPORT_ENV], env={'ROS_DOMAIN_ID': '0'}
    )
    assert result.returncode == 0, result.stderr
    primary, _ = reported_domains(result)
    assert primary != 0
    assert primary in medkit_domain.USABLE_DOMAINS


def test_the_domain_stays_locked_for_as_long_as_the_command_runs(tmp_path):
    started = tmp_path / 'started'
    release = tmp_path / 'release'
    child = (
        'import os, pathlib, time; '
        f"pathlib.Path({str(started)!r}).write_text(os.environ['ROS_DOMAIN_ID']); "
        '[time.sleep(0.05) for _ in iter('
        f'lambda: not os.path.exists({str(release)!r}), False)]'
    )
    proc = subprocess.Popen(
        [sys.executable, EXEC_WRAPPER, '--', sys.executable, '-c', child],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert wait_until(started.exists, 30), 'the wrapped command never started'
        domain = int(started.read_text())
        assert not port_is_free(domain), 'the lock was dropped while the command was running'
    finally:
        release.write_text('go')
        proc.wait(timeout=30)
    assert wait_until(lambda: port_is_free(domain), 10), 'the lock outlived the wrapper'


def test_a_wrapper_killed_with_sigkill_frees_its_domain(tmp_path):
    started = tmp_path / 'started'
    child = (
        'import os, pathlib, time; '
        f"pathlib.Path({str(started)!r}).write_text(os.environ['ROS_DOMAIN_ID']); "
        'time.sleep(600)'
    )
    proc = subprocess.Popen(
        [sys.executable, EXEC_WRAPPER, '--', sys.executable, '-c', child],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    assert wait_until(started.exists, 30), 'the wrapped command never started'
    domain = int(started.read_text())
    assert not port_is_free(domain)

    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    proc.wait(timeout=30)
    assert wait_until(lambda: port_is_free(domain), 10), 'SIGKILL left the domain locked'

    # "Frees it for the next test", not merely "closed a socket": the next
    # wrapper is given a band of exactly this domain, so taking it is the only
    # way it can succeed.
    result = run_wrapped(
        [], [sys.executable, '-c', REPORT_ENV],
        env={medkit_domain.BAND_ENV: str(domain)},
    )
    assert result.returncode == 0, result.stderr
    assert reported_domains(result)[0] == domain


def test_an_exhausted_band_fails_loudly_and_never_runs_the_command(tmp_path):
    marker = tmp_path / 'ran'
    child = f"import pathlib; pathlib.Path({str(marker)!r}).write_text('ran')"
    with reserve_band(3) as band:
        result = run_wrapped(
            ['--wait-timeout', '1'], [sys.executable, '-c', child],
            env={medkit_domain.BAND_ENV: band.spec},
        )
    assert result.returncode != 0
    assert not marker.exists(), 'the command ran without a domain'
    assert 'ROS_DOMAIN_ID' in result.stderr
    assert 'MEDKIT_TEST_DOMAIN_WAIT' in result.stderr


def test_a_machine_whose_ephemeral_range_swallows_the_band_never_runs_the_command(tmp_path):
    # The regression this guards: the band is a constant, and the kernel range it
    # is derived from is a sysctl. Set to "1024 65535" - common on CI images and
    # Kubernetes nodes - every domain in the band is inside the range the kernel
    # may hand to anything, and 45% of ephemeral ports land in a slice the band
    # calls safe. The wrapper has to refuse, on the machine that runs the tests,
    # rather than start a test whose nodes will die with a bind error somewhere
    # else entirely.
    marker = tmp_path / 'ran'
    child = f"import pathlib; pathlib.Path({str(marker)!r}).write_text('ran')"
    result = run_wrapped(
        ['--wait-timeout', '1'], [sys.executable, '-c', child],
        env={medkit_domain.EPHEMERAL_RANGE_ENV: '1024-65535'},
    )
    assert result.returncode != 0
    assert not marker.exists(), 'the command ran on a domain the kernel may reassign'
    assert 'net.ipv4.ip_local_port_range' in result.stderr
    assert '1024-65535' in result.stderr


def test_the_part_of_the_band_that_survives_a_narrowed_range_is_usable(tmp_path):
    # The advice the refusal gives has to work, or it is not advice. 10240-65535
    # leaves 1-10 safe, and a run narrowed to those has to go through.
    result = run_wrapped(
        [], [sys.executable, '-c', REPORT_ENV],
        env={
            medkit_domain.EPHEMERAL_RANGE_ENV: '10240-65535',
            medkit_domain.BAND_ENV: '1-10',
        },
    )
    assert result.returncode == 0, result.stderr
    primary, extras = reported_domains(result)
    assert primary in range(1, 11)
    assert extras == []


def test_a_machine_that_will_not_report_its_range_is_refused_by_the_wrapper(tmp_path):
    # No declaration and no readable file: the wrapper must say so rather than
    # assume the Linux default, which is the assumption the check exists to stop.
    marker = tmp_path / 'ran'
    child = f"import pathlib; pathlib.Path({str(marker)!r}).write_text('ran')"
    shadow = tmp_path / 'shadow'
    shadow.mkdir()
    (shadow / 'sitecustomize.py').write_text(
        'import medkit_domain\n'
        "medkit_domain.EPHEMERAL_RANGE_FILE = '/nonexistent/ip_local_port_range'\n"
    )
    env = {
        'PYTHONPATH': os.pathsep.join(
            [str(shadow), medkit_domain.SCRIPT_DIR, os.environ.get('PYTHONPATH', '')]
        ),
    }
    result = run_wrapped(['--wait-timeout', '1'], [sys.executable, '-c', child], env=env)
    assert result.returncode != 0
    assert not marker.exists(), 'the command ran without knowing the ephemeral range'
    assert medkit_domain.EPHEMERAL_RANGE_ENV in result.stderr


def test_more_holders_than_free_domains_wait_and_are_served_as_domains_free_up(tmp_path):
    # A band of four domains this process has proved it can hold, two of them
    # handed back. Four wrappers ask for one each: two must run, two must wait.
    # Releasing a domain must release exactly one waiter, and no two live
    # holders may ever share a domain.
    procs = []
    started = []
    release = tmp_path / 'release'
    try:
        with reserve_band(4) as band:
            free = sorted(band.release(2))
            for index in range(4):
                marker = tmp_path / f'started_{index}'
                started.append(marker)
                child = (
                    'import os, pathlib, time; '
                    f"pathlib.Path({str(marker)!r}).write_text(os.environ['ROS_DOMAIN_ID']); "
                    '[time.sleep(0.05) for _ in iter('
                    f'lambda: not os.path.exists({str(release)!r}), False)]'
                )
                procs.append(
                    subprocess.Popen(
                        [
                            sys.executable, EXEC_WRAPPER,
                            '--wait-timeout', '90', '--',
                            sys.executable, '-c', child,
                        ],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        env=dict(os.environ, **{medkit_domain.BAND_ENV: band.spec}),
                    )
                )

            def two_running():
                return sum(1 for marker in started if marker.exists()) == 2

            assert wait_until(two_running, 60), 'the two free domains were not taken'
            time.sleep(2.0)
            running = [marker for marker in started if marker.exists()]
            assert len(running) == 2, 'a third holder got a domain that was not free'
            assert sorted(int(marker.read_text()) for marker in running) == free

            # Change, not steady state: hand one domain back while two wait.
            band.release(1)

            def three_running():
                return sum(1 for marker in started if marker.exists()) == 3

            assert wait_until(three_running, 60), 'a waiter was not served when a domain freed'
            time.sleep(2.0)
            assert sum(1 for marker in started if marker.exists()) == 3

        # Band fully released: the fourth waiter gets served too.
        def all_running():
            return all(marker.exists() for marker in started)

        assert wait_until(all_running, 60), 'the last waiter was never served'
        domains = [int(marker.read_text()) for marker in started]
        assert len(set(domains)) == 4, f'a domain was issued twice: {domains}'
    finally:
        release.write_text('go')
        for proc in procs:
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=30)


def settle(sample, timeout, quiet_for=4.0, interval=0.5):
    """Return *sample* once it has stopped changing, or at the deadline.

    Replaces waiting for a fixed number of holders. The old form waited for the
    band to fill completely, which is a count no test can be owed: a domain
    whose lock port an unrelated process holds is not ours to take, so the
    condition could never become true and the wait ran to its full deadline
    before failing for the wrong reason.
    """
    deadline = time.monotonic() + timeout
    last, stable_since = None, time.monotonic()
    while time.monotonic() < deadline:
        current = sample()
        if current != last:
            last, stable_since = current, time.monotonic()
        elif time.monotonic() - stable_since >= quiet_for and current:
            return current
        time.sleep(interval)
    return sample()


def test_more_wrappers_than_the_band_has_domains(tmp_path):
    # Past the cap: more wrappers than the band they were given has domains.
    #
    # Against the REAL band this needed 122 concurrent processes holding up to
    # 117 coordination ports. Those ports live in the kernel ephemeral range,
    # and on a shared build agent - a binarydeb container runs with the host's
    # own network namespace - that is the agent's port space, not ours. Any
    # other job using domain_coordinator would have seen the whole band busy,
    # and ordinary processes would have lost ephemeral ports. The property is
    # about free domains against requesters, not about the number 117, so it is
    # pinned against a band this test reserved: six domains, nine wrappers.
    band_size = 6
    excess = 3
    release = tmp_path / 'release'
    markers = [tmp_path / f'domain_{index}' for index in range(band_size + excess)]
    procs = []
    try:
        with reserve_band(band_size) as band:
            band.release()  # hand all six to the allocator
            for marker in markers:
                script = (
                    f'printf %s "$ROS_DOMAIN_ID" > "{marker}"; '
                    f'while [ ! -e "{release}" ]; do sleep 0.1; done'
                )
                procs.append(
                    subprocess.Popen(
                        [
                            sys.executable, EXEC_WRAPPER, '--wait-timeout', '120', '--',
                            '/bin/sh', '-c', script,
                        ],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.PIPE,
                        text=True,
                        env=dict(os.environ, **{medkit_domain.BAND_ENV: band.spec}),
                    )
                )

            def running():
                return sum(1 for marker in markers if marker.exists())

            assert wait_until(lambda: running() == band_size, 120), (
                f'only {running()} of {band_size} domains were taken'
            )
            time.sleep(3.0)
            assert running() == band_size, (
                f'{running()} wrappers hold a domain at once, but the band has only '
                f'{band_size}; the excess did not wait'
            )
            held = [int(m.read_text()) for m in markers if m.exists()]
            assert sorted(held) == sorted(band.domains), 'a domain outside the band was used'
            assert len(set(held)) == len(held), 'a domain was issued to two wrappers at once'

            release.write_text('go')

            def everybody_ran():
                return all(marker.exists() for marker in markers)

            assert wait_until(everybody_ran, 180), 'the waiting wrappers were never served'
            for marker in markers:
                assert int(marker.read_text()) in band.domains
    finally:
        release.write_text('go')
        for proc in procs:
            try:
                proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=30)


def test_many_concurrent_wrappers_all_get_distinct_domains(tmp_path):
    # Concurrency without over-subscription: as many wrappers as the band has
    # domains, all of which must get one and none of which may share. Bounded
    # to a reserved band for the same reason as the test above - these are the
    # host's ports on a shared agent, not ours to take by the hundred.
    count = 8
    release = tmp_path / 'release'
    markers = [tmp_path / f'domain_{index}' for index in range(count)]
    procs = []
    try:
        with reserve_band(count) as band:
            band.release()
            for marker in markers:
                child = (
                    'import os, pathlib, time; '
                    f"pathlib.Path({str(marker)!r}).write_text(os.environ['ROS_DOMAIN_ID']); "
                    '[time.sleep(0.05) for _ in iter('
                    f'lambda: not os.path.exists({str(release)!r}), False)]'
                )
                procs.append(
                    subprocess.Popen(
                        [sys.executable, EXEC_WRAPPER, '--', sys.executable, '-c', child],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        env=dict(os.environ, **{medkit_domain.BAND_ENV: band.spec}),
                    )
                )
            assert wait_until(lambda: all(m.exists() for m in markers), 120), (
                'not every wrapper ran'
            )
            domains = [int(marker.read_text()) for marker in markers]
            assert len(set(domains)) == count, f'domains repeated: {sorted(domains)}'
            assert sorted(domains) == sorted(band.domains)
    finally:
        release.write_text('go')
        for proc in procs:
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=30)


def test_ament_runner_gives_the_test_command_a_domain(tmp_path):
    result_file = tmp_path / 'result.xml'
    output = tmp_path / 'seen.json'
    child = (
        'import json, os, pathlib; '
        f'pathlib.Path({str(output)!r}).write_text(json.dumps('
        "{'primary': os.environ.get('ROS_DOMAIN_ID'), "
        "'secondary': os.environ.get('MEDKIT_SECONDARY_DOMAINS')}))"
    )
    env = dict(os.environ)
    env['MEDKIT_TEST_DOMAINS'] = '3'
    completed = subprocess.run(
        [
            sys.executable, '-u', RUNNER, str(result_file),
            '--package-name', 'ros2_medkit_cmake',
            '--generate-result-on-success',
            '--command', sys.executable, '-c', child,
        ],
        capture_output=True,
        text=True,
        env=env,
        timeout=120,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    seen = json.loads(output.read_text())
    extras = [int(part) for part in seen['secondary'].split(',')]
    allocated = [int(seen['primary'])] + extras
    assert len(set(allocated)) == 3
    for domain in allocated:
        assert domain in medkit_domain.USABLE_DOMAINS
    assert result_file.exists(), 'the ament runner must still produce its result file'


def test_ament_runner_reports_the_domain_it_took(tmp_path):
    result_file = tmp_path / 'result.xml'
    completed = subprocess.run(
        [
            sys.executable, '-u', RUNNER, str(result_file),
            '--package-name', 'ros2_medkit_cmake',
            '--generate-result-on-success',
            '--command', sys.executable, '-c', 'pass',
        ],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert 'ROS_DOMAIN_ID' in completed.stdout


@pytest.mark.parametrize('count', [1, 2, 3, 4])
def test_domain_counts_across_the_supported_range(count):
    result = run_wrapped(['--domains', str(count)], [sys.executable, '-c', REPORT_ENV])
    assert result.returncode == 0, result.stderr
    primary, extras = reported_domains(result)
    assert len([primary] + extras) == count
    assert len(set([primary] + extras)) == count


def test_a_domain_count_below_one_is_rejected():
    result = run_wrapped(['--domains', '0'], [sys.executable, '-c', 'pass'])
    assert result.returncode != 0


def test_the_runner_leaves_a_result_file_even_when_it_never_reaches_the_test(tmp_path):
    # run_test.py writes a placeholder result before running anything, so a
    # killed or crashed test still leaves a visible failure. That protection
    # only starts once run_test.py is running, and the wrapper does work before
    # then. A failure in that window used to leave no result file at all, which
    # reads as an absent test rather than a failed one.
    result_file = tmp_path / 'some_package' / 'result.xml'
    env = dict(os.environ)
    env['MEDKIT_TEST_DOMAINS'] = '0'
    completed = subprocess.run(
        [
            sys.executable, '-u', RUNNER, str(result_file),
            '--package-name', 'some_package',
            '--command', sys.executable, '-c', 'pass',
        ],
        capture_output=True,
        text=True,
        env=env,
        timeout=120,
        check=False,
    )
    assert completed.returncode != 0
    assert result_file.exists(), 'the wrapper failed and took the test result with it'
    body = result_file.read_text()
    assert '<error' in body
    assert 'domain wrapper' in body


def test_the_reserved_result_file_is_replaced_on_an_ordinary_run(tmp_path):
    result_file = tmp_path / 'some_package' / 'result.xml'
    completed = subprocess.run(
        [
            sys.executable, '-u', RUNNER, str(result_file),
            '--package-name', 'some_package',
            '--generate-result-on-success',
            '--command', sys.executable, '-c', 'pass',
        ],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    body = result_file.read_text()
    assert 'domain wrapper' not in body, 'the reservation outlived the run it stands in for'


def test_a_failing_import_in_the_wrapper_still_leaves_a_result_file(tmp_path):
    # The reservation has to happen before the wrapper imports anything that
    # can fail. With the imports at module scope an ImportError killed the
    # process before a single byte was written, which is the exact failure the
    # reservation exists to prevent.
    result_file = tmp_path / 'some_package' / 'result.xml'
    shadow = tmp_path / 'shadow'
    shadow.mkdir()
    (shadow / 'medkit_domain.py').write_text('raise ImportError("deliberately broken")\n')
    env = dict(os.environ, PYTHONPATH=str(shadow))
    completed = subprocess.run(
        [
            sys.executable, '-u', RUNNER, str(result_file),
            '--package-name', 'some_package',
            '--command', sys.executable, '-c', 'pass',
        ],
        capture_output=True, text=True, env=env, timeout=120, check=False,
    )
    assert completed.returncode != 0
    assert result_file.exists(), 'a failing import took the test result with it'
    assert 'domain wrapper' in result_file.read_text()
    assert 'could not load what it needs' in completed.stderr
