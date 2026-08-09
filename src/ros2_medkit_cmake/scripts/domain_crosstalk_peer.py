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

"""One half of a cross-talk probe: a real ROS node that shouts and listens.

Two of these, started independently and each holding its own allocated domain,
must not hear each other. Run with ``--expect-crosstalk`` and forced onto ONE
domain, the same two must hear each other - that is the control that makes the
zero mean something, and it is why this script asserts in both directions
rather than only the one we hope for.

The peers find each other through a rendezvous directory rather than through
ROS, because the whole point is that they cannot see each other over ROS. A peer
refreshes its own marker file while it runs, so a marker left behind by a killed
run ages out instead of being mistaken for a live peer.
"""

import argparse
import os
import pathlib
import re
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = '/medkit_domain_crosstalk'
PEER_ALIVE_WINDOW = 20.0
MARKER_REFRESH = 1.0


def rendezvous_dir(group):
    """Return the directory the peers of *group* use to find each other.

    Derived rather than passed in: the peers of a group can live in different
    packages, whose CMake has no shared directory to agree on.
    """
    path = pathlib.Path(tempfile.gettempdir()) / f'medkit_domain_crosstalk_{os.getuid()}' / group
    path.mkdir(parents=True, exist_ok=True)
    return path


def live_peers(directory, own_role):
    """Return the roles other than *own_role* whose marker is still fresh."""
    now = time.time()
    peers = []
    for marker in directory.glob('*.alive'):
        role = marker.name[: -len('.alive')]
        if role == own_role:
            continue
        try:
            if now - marker.stat().st_mtime <= PEER_ALIVE_WINDOW:
                peers.append(role)
        except OSError:
            continue
    return sorted(peers)


class CrosstalkPeer(Node):

    def __init__(self, role):
        super().__init__('medkit_crosstalk_' + re.sub(r'[^0-9a-zA-Z_]', '_', role))
        self.role = role
        self.heard = {}
        self._publisher = self.create_publisher(String, TOPIC, 10)
        self.create_subscription(String, TOPIC, self._on_message, 10)

    def _on_message(self, message):
        if message.data != self.role:
            self.heard[message.data] = self.heard.get(message.data, 0) + 1

    def shout(self):
        message = String()
        message.data = self.role
        self._publisher.publish(message)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', required=True, help='name this peer publishes as')
    parser.add_argument('--group', required=True, help='rendezvous group shared with the peers')
    parser.add_argument('--peers', type=int, default=2, help='how many peers the group has')
    parser.add_argument(
        '--barrier-timeout',
        type=float,
        default=30.0,
        help='seconds to wait for the other peers before giving up. Whoever starts '
             'the peers is responsible for starting them together, so a long wait '
             'here buys nothing and costs a CI job the whole of it.',
    )
    parser.add_argument(
        '--observe-seconds',
        type=float,
        default=8.0,
        help='seconds to keep publishing and listening once everybody is up',
    )
    parser.add_argument(
        '--expect-crosstalk',
        action='store_true',
        help='invert the assertion: this run is the control and MUST hear its peers',
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    domain = os.environ.get('ROS_DOMAIN_ID')
    if not domain or domain == '0':
        print(
            f'[{args.role}] ROS_DOMAIN_ID={domain!r}: this probe has to run through the '
            'domain wrapper, otherwise it observes the default domain and proves nothing.',
            file=sys.stderr,
        )
        return 2
    print(f'[{args.role}] ROS_DOMAIN_ID={domain}', flush=True)

    directory = rendezvous_dir(args.group)
    marker = directory / f'{args.role}.alive'
    rclpy.init()
    node = CrosstalkPeer(args.role)
    started = time.monotonic()
    try:
        deadline = started + args.barrier_timeout
        while True:
            marker.write_text(domain)
            node.shout()
            rclpy.spin_once(node, timeout_sec=0.05)
            present = live_peers(directory, args.role)
            if len(present) >= args.peers - 1:
                break
            if time.monotonic() >= deadline:
                print(
                    f'[{args.role}] no peer showed up in {args.barrier_timeout:.0f}s '
                    f'(saw {present}). This probe only proves something while its peers '
                    'run alongside it, so an absent peer is a failed run, not a pass.',
                    file=sys.stderr,
                )
                return 3
        rendezvous_at = time.monotonic()
        print(
            f'[{args.role}] peers {live_peers(directory, args.role)} up after '
            f'{rendezvous_at - started:.1f}s, observing for {args.observe_seconds:.0f}s',
            flush=True,
        )

        end = rendezvous_at + args.observe_seconds
        last_refresh = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_refresh >= MARKER_REFRESH:
                marker.write_text(domain)
                last_refresh = now
            node.shout()
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        marker.unlink(missing_ok=True)

    print(f'[{args.role}] heard {node.heard or "nothing"} on domain {domain}', flush=True)
    if args.expect_crosstalk:
        if not node.heard:
            print(
                f'[{args.role}] CONTROL FAILED: two peers sharing domain {domain} heard '
                'nothing from each other, so this probe cannot detect cross-talk and a '
                'zero from it means nothing.',
                file=sys.stderr,
            )
            return 1
        return 0
    if node.heard:
        print(
            f'[{args.role}] CROSS-TALK: heard {node.heard} while holding domain {domain} '
            'alone. Another test is on the same domain.',
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
