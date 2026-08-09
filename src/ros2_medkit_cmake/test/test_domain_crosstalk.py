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

"""Real ROS nodes, allocated domains, and the control that makes the zero mean something.

The claim is that two independently allocated tests cannot hear each other. A
probe that reports zero messages proves that only if the same probe reports
messages when the isolation is removed, so the control runs first: the same two
peers, forced onto one domain, must hear each other.

Both peers are started by this test, as concurrent subprocesses it joins. An
earlier shape registered them as two CTest tests and relied on ``ctest -j`` to
run them side by side. CTest has no way to require that, and CI runs ctest
without ``-j``, so the peers were serialised and each waited out its barrier
alone: green under a parallel run and red under a serial one, deterministically.
Overlap is a property of this probe, not of the scheduler that happens to run
it, so this test owns it.
"""

import os
import subprocess
import sys
import uuid

import medkit_domain

PEER = os.path.join(medkit_domain.SCRIPT_DIR, 'domain_crosstalk_peer.py')
EXEC_WRAPPER = os.path.join(medkit_domain.SCRIPT_DIR, 'medkit_run_with_domain.py')

PEER_TIMEOUT = 180


def peer_command(role, group, extra=()):
    return [
        sys.executable, PEER,
        '--role', role,
        '--group', group,
        '--peers', '2',
        '--barrier-timeout', '60',
        '--observe-seconds', '8',
        *extra,
    ]


def run_pair(commands, env=None):
    """Start both peers, wait for both, and return their CompletedProcess-likes."""
    child_env = dict(os.environ)
    if env:
        child_env.update(env)
    procs = [
        subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=child_env)
        for cmd in commands
    ]
    results = []
    try:
        for proc in procs:
            stdout, stderr = proc.communicate(timeout=PEER_TIMEOUT)
            results.append((proc.returncode, stdout, stderr))
    finally:
        for proc in procs:
            if proc.poll() is None:
                proc.kill()
                proc.communicate(timeout=30)
    return results


def assert_peers_overlapped(results):
    """Fail unless both peers saw each other alive during the run.

    Without this the probe would pass by running two peers that never coexisted,
    which observes nothing and proves nothing.
    """
    for _, stdout, stderr in results:
        assert 'peers [' in stdout, (
            'a peer never reached rendezvous, so the two never overlapped and this '
            f'probe observed nothing:\n{stdout}\n{stderr}'
        )


def test_the_probe_can_see_cross_talk_when_the_domains_are_shared():
    # The control. Both peers run on THIS test's own allocated domain, so the
    # only thing removed is the isolation between them.
    domain = os.environ.get('ROS_DOMAIN_ID')
    assert domain and domain != '0', 'this test must itself run through the domain wrapper'
    group = f'control_{uuid.uuid4().hex[:8]}'
    results = run_pair(
        [
            peer_command('control_a', group, extra=('--expect-crosstalk',)),
            peer_command('control_b', group, extra=('--expect-crosstalk',)),
        ]
    )
    for returncode, stdout, stderr in results:
        assert returncode == 0, f'the control heard nothing:\n{stdout}\n{stderr}'
    assert_peers_overlapped(results)
    assert all(f'ROS_DOMAIN_ID={domain}' in stdout for _, stdout, _ in results)


def test_independently_allocated_peers_hear_nothing_from_each_other():
    group = f'isolated_{uuid.uuid4().hex[:8]}'
    wrapped = [
        [sys.executable, EXEC_WRAPPER, '--'] + peer_command('isolated_a', group),
        [sys.executable, EXEC_WRAPPER, '--'] + peer_command('isolated_b', group),
    ]
    results = run_pair(wrapped)
    for returncode, stdout, stderr in results:
        assert returncode == 0, f'a peer heard its neighbour:\n{stdout}\n{stderr}'
    assert_peers_overlapped(results)
    domains = []
    for _, stdout, _ in results:
        for line in stdout.splitlines():
            if line.startswith('[medkit-domain] ROS_DOMAIN_ID='):
                domains.append(int(line.split('=')[1].split()[0]))
    assert len(domains) == 2, f'could not read both allocated domains from:\n{results}'
    assert domains[0] != domains[1], 'the two peers were given the same domain'
