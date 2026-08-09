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

"""Run a command with a DDS domain of its own.

For CTest tests registered with a plain ``add_test`` - the ones that build their
own command line and so cannot take an ament test runner. Tests registered
through ``ament_add_test`` (every gtest and every launch test) use
``medkit_domain_runner.py`` instead, which does the same thing without adding a
process to the tree.

Usage:

    medkit_run_with_domain.py [--domains N] [--wait-timeout S] -- CMD [ARG...]

The domains are held by THIS process for exactly as long as ``CMD`` runs, and
released by the kernel if it is killed. The command's exit code is passed
through; a command killed by a signal is reported as ``128 + signal``, the shell
convention.
"""

import argparse
import os
import signal
import subprocess
import sys

import medkit_domain

_FORWARDED_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='medkit_run_with_domain.py',
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--domains',
        type=int,
        default=None,
        help='how many distinct domains the command needs (default: '
             f'${medkit_domain.DOMAIN_COUNT_ENV}, or 1)',
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=None,
        help='seconds to wait for a free domain before failing (default: '
             f'{medkit_domain.DEFAULT_WAIT_TIMEOUT:.0f})',
    )
    parser.add_argument('command', nargs=argparse.REMAINDER, help='-- followed by the command')
    args = parser.parse_args(argv)
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('no command given; pass it after a "--" separator')
    args.command = command
    return args


def run_child(command):
    """Run *command*, forwarding the signals CTest uses to stop a test."""
    child = subprocess.Popen(command)

    def forward(signum, _frame):
        try:
            child.send_signal(signum)
        except ProcessLookupError:
            pass

    previous = {}
    for signum in _FORWARDED_SIGNALS:
        previous[signum] = signal.signal(signum, forward)
    try:
        returncode = child.wait()
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    if returncode < 0:
        # Popen reports "killed by signal N" as -N; CTest wants a shell code.
        return 128 - returncode
    return returncode


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    count = args.domains
    if count is None:
        count = medkit_domain.requested_domain_count()
    if count < 1:
        print(
            f'--domains {count} is not a domain count; a test that needs no domain '
            'must not go through this wrapper at all.',
            file=sys.stderr,
        )
        return 2

    try:
        with medkit_domain.hold_domains(count, wait_timeout=args.wait_timeout) as domains:
            medkit_domain.apply_to_environ(domains, os.environ)
            print(
                f'[medkit-domain] ROS_DOMAIN_ID={domains[0]}'
                + (f' secondary={domains[1:]}' if len(domains) > 1 else ''),
                flush=True,
            )
            return run_child(args.command)
    except medkit_domain.DomainAllocationError as error:
        print(f'[medkit-domain] {error}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
