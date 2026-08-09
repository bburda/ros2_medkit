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

"""The ament test runner, with a DDS domain held around it.

``ament_add_test`` invokes ``python -u <RUNNER> <result file> ...``, so this
script takes the arguments ``run_test.py`` takes and hands them straight on. It
runs in place of ``run_test.py`` rather than in front of it: the domain is held
by the process that already supervises the test command, so no process is added
to the tree and nothing new can orphan a test on a CTest timeout.

How many domains the test needs comes from ``MEDKIT_TEST_DOMAINS`` in the test's
CTest environment, set by the helpers in ``ROS2MedkitTestDomain.cmake``.

This is deliberately not ``ament_add_ros_isolated_gtest``'s runner. That one
draws from upstream's 1-100 selector, and it skips allocation entirely when
``ROS_DOMAIN_ID`` is already set in the environment - which would put every test
of a run onto one domain the moment a developer or a colcon extension exported
one. Here an inherited ``ROS_DOMAIN_ID`` is replaced, always.
"""

import os
import sys
from xml.sax.saxutils import quoteattr

# ament_cmake_test and medkit_domain are imported inside main(), AFTER the
# result file has been reserved. At module scope an ImportError - a missing
# dependency, a half-installed overlay, a syntax error introduced in this
# package - would kill the process before anything was written, which is the
# very failure the reservation exists to prevent. Only the standard library is
# safe to import out here.


def reserve_result_file(result_file):
    """Leave a failing result file behind before anything that can fail runs.

    run_test.py writes one of these as its very first action, precisely so that
    a command which segfaults or is killed still leaves a visible failure. That
    guarantee only starts once run_test.py is running, and this wrapper does
    work before then: reading the domain count, waiting for a domain. A failure
    in that window used to leave no result file at all, which reads as an absent
    test rather than a failed one - a test that has stopped proving anything
    while still looking present.

    Overwritten by run_test.py the moment it starts, so on every ordinary path
    this file is never seen.
    """
    message = (
        'The domain wrapper did not reach the test. No result file was produced '
        'by the test itself, so this one stands in for it.'
    )
    package = os.path.basename(os.path.dirname(result_file))
    name = os.path.splitext(os.path.basename(result_file))[0]
    document = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<testsuite name="{package}" tests="1" failures="0" time="0.000"'
        ' errors="1" skipped="0">\n'
        f'  <testcase classname="{package}" name="{name}.missing_result" time="0.000">\n'
        f'    <error message={quoteattr(message)}/>\n'
        '  </testcase>\n'
        '</testsuite>\n'
    )
    try:
        os.makedirs(os.path.dirname(result_file), exist_ok=True)
        with open(result_file, 'w') as handle:
            handle.write(document)
    except OSError as error:
        print(
            f'[medkit-domain] could not reserve {result_file}: {error}',
            file=sys.stderr,
            flush=True,
        )


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    # ament_add_test always passes the result file first; guard the index anyway
    # so a malformed invocation reports itself rather than raising IndexError.
    if argv and not argv[0].startswith('-'):
        reserve_result_file(argv[0])

    try:
        import ament_cmake_test
        import medkit_domain
    except Exception as error:  # noqa: B902 - any import failure, reported not raised
        print(
            f'[medkit-domain] the domain wrapper could not load what it needs '
            f'({type(error).__name__}: {error}). The test was not started.',
            file=sys.stderr,
            flush=True,
        )
        return 1

    count = medkit_domain.requested_domain_count()
    try:
        with medkit_domain.hold_domains(count) as domains:
            medkit_domain.apply_to_environ(domains)
            print(
                f'[medkit-domain] running with ROS_DOMAIN_ID={domains[0]}'
                + (f', {medkit_domain.SECONDARY_ENV}={domains[1:]}' if len(domains) > 1 else ''),
                flush=True,
            )
            return ament_cmake_test.main(argv)
    except medkit_domain.DomainAllocationError as error:
        print(f'[medkit-domain] {error}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
