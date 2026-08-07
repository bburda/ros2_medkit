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

"""Check the DDS domains a package hands to its tests.

ROS2MedkitTestDomain.cmake checks the allocation table at configure time, on the
machine that builds. This runs on the machine that *tests*, against the test
properties CMake actually generated and against that machine's live ephemeral
port range, so it also catches a hand-written ROS_DOMAIN_ID that never went
through the table and a resource lock dropped by a property overwrite.

Fails when any of these does not hold:

* every ROS_DOMAIN_ID handed to a test comes from the package's pool
* every one of those domains maps to a UDP slice outside the kernel ephemeral
  port range, so no unrelated process can be given one of its ports
* every test that gets a ROS_DOMAIN_ID also carries that domain's RESOURCE_LOCK,
  which is what keeps two tests off a reused domain at the same time
"""

import argparse
from collections import Counter
import json
import subprocess
import sys

RTPS_PORT_BASE = 7400
RTPS_DOMAIN_GAIN = 250
EPHEMERAL_RANGE_FILE = '/proc/sys/net/ipv4/ip_local_port_range'
# Widen whatever the kernel reports to at least this, so a host configured with a
# narrow range cannot bless an allocation that breaks on a stock machine.
DEFAULT_EPHEMERAL_LOW = 32768
DEFAULT_EPHEMERAL_HIGH = 60999
MAX_UDP_PORT = 65535


def ephemeral_range():
    """Return (low, high, description) for the ports the kernel may hand out."""
    low, high = DEFAULT_EPHEMERAL_LOW, DEFAULT_EPHEMERAL_HIGH
    source = 'Linux default; kernel range not readable'
    try:
        with open(EPHEMERAL_RANGE_FILE) as handle:
            fields = handle.read().split()
        sys_low, sys_high = int(fields[0]), int(fields[1])
    except (OSError, ValueError, IndexError):
        return low, high, source
    source = f'{EPHEMERAL_RANGE_FILE} = {sys_low}-{sys_high}, widened to the Linux default'
    return min(low, sys_low), max(high, sys_high), source


def domain_slice(domain):
    """Return the (first, last) UDP port RTPS reserves for a domain."""
    first = RTPS_PORT_BASE + RTPS_DOMAIN_GAIN * domain
    return first, first + RTPS_DOMAIN_GAIN - 1


def parse_domain_list(text):
    return [int(part) for part in text.split(',') if part.strip()]


def load_tests(build_dir):
    """Return the CTest test list with its properties, straight from ctest."""
    completed = subprocess.run(
        ['ctest', '--show-only=json-v1'],
        cwd=build_dir,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f'ctest --show-only=json-v1 failed in {build_dir} '
            f'(exit {completed.returncode}):\n{completed.stderr.strip()}'
        )
    return json.loads(completed.stdout).get('tests', [])


def test_properties(test):
    """Return {property name: list of values} for one test entry."""
    result = {}
    for prop in test.get('properties', []):
        value = prop.get('value')
        if isinstance(value, str):
            value = [value]
        elif not isinstance(value, list):
            value = []
        result[prop['name']] = value
    return result


def domain_of(env_entries):
    """Return the ROS_DOMAIN_ID a test is given, or None.

    CTest applies ENVIRONMENT entries in order, so when a test carries more
    than one ROS_DOMAIN_ID entry (medkit_set_test_domain APPENDs, so a caller
    can add its own on top), the last one wins. Scan in reverse to match.
    """
    for entry in reversed(env_entries):
        name, _, value = entry.partition('=')
        if name == 'ROS_DOMAIN_ID':
            return int(value)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True, help='package build directory')
    parser.add_argument('--package', required=True, help='package that owns the pool')
    parser.add_argument('--pool', required=True, help='comma-separated domains the package owns')
    parser.add_argument('--secondary', default='', help='comma-separated shared secondary domains')
    args = parser.parse_args()

    pool = set(parse_domain_list(args.pool))
    secondary = set(parse_domain_list(args.secondary))
    low, high, source = ephemeral_range()
    failures = []

    print(f'package        : {args.package}')
    print(f'pool           : {sorted(pool)}')
    print(f'ephemeral range: {low}-{high} ({source})')

    # The pool itself, before looking at a single test. A pool that overlaps the
    # ephemeral range is broken whether or not a test happens to use that slot.
    for domain in sorted(pool | secondary):
        first, last = domain_slice(domain)
        if last > MAX_UDP_PORT:
            failures.append(
                f'pool domain {domain} maps to UDP {first}-{last}, past the {MAX_UDP_PORT} ceiling'
            )
        elif not (last < low or first > high):
            failures.append(
                f'pool domain {domain} maps to UDP {first}-{last}, inside the ephemeral range '
                f'{low}-{high}; any process can be handed one of those ports'
            )

    checked = 0
    domain_usage = Counter()
    for test in load_tests(args.build_dir):
        name = test.get('name', '<unnamed>')
        props = test_properties(test)
        domain = domain_of(props.get('ENVIRONMENT', []))
        if domain is None:
            stray_locks = sorted(
                lock for lock in props.get('RESOURCE_LOCK', [])
                if lock.startswith('medkit_dds_domain_')
            )
            if stray_locks:
                failures.append(
                    f'{name} carries {", ".join(stray_locks)} but no ROS_DOMAIN_ID. Its '
                    'ENVIRONMENT was overwritten after medkit_set_test_domain ran, most '
                    'likely by a set_tests_properties(... PROPERTIES ENVIRONMENT ...) call '
                    'that replaced instead of appending. Use set_property(TEST ... APPEND '
                    'PROPERTY ...) instead.'
                )
            continue
        checked += 1
        domain_usage[domain] += 1

        if domain not in pool:
            failures.append(
                f'{name} runs on domain {domain}, which is not in the pool for '
                f'{args.package}. Assign it with medkit_set_test_domain instead of by hand.'
            )

        expected_lock = f'medkit_dds_domain_{domain}'
        if expected_lock not in props.get('RESOURCE_LOCK', []):
            failures.append(
                f'{name} runs on domain {domain} without the {expected_lock} resource lock. '
                'Domains are reused inside a package, so without the lock ctest -j can run '
                'two tests on the same domain. A set_tests_properties(... PROPERTIES '
                'RESOURCE_LOCK ...) after medkit_set_test_domain overwrites it; use '
                'set_property(... APPEND ...).'
            )

    print(f'tests with a domain: {checked}')
    # Reuse is legal - medkit_set_test_domain wraps once the pool runs out, and
    # the RESOURCE_LOCK is what keeps two tests off a reused domain at the same
    # time. This is visibility into how hard a package leans on it, not a gate:
    # a package quietly drifting to several times its pool size would otherwise
    # have no signal at all.
    busiest_domain, busiest_count = max(
        domain_usage.items(), key=lambda item: item[1], default=(None, 0)
    )
    print(
        f'domain reuse       : {checked} domained tests over a pool of {len(pool)}, '
        f'busiest domain ({busiest_domain}) used by {busiest_count} test(s)'
    )
    if checked == 0:
        failures.append(
            f'no test in {args.package} carries a ROS_DOMAIN_ID, yet the package asked for a '
            'pool. Every assignment was lost, most likely to a property overwrite.'
        )

    if failures:
        print('')
        for failure in failures:
            print(f'FAIL: {failure}')
        return 1

    print('OK: every test domain is in-pool, outside the ephemeral range, and locked')
    return 0


if __name__ == '__main__':
    sys.exit(main())
