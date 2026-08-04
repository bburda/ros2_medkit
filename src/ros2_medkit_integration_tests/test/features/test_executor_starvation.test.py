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

"""Executor-starvation falsifier for the RPC callback-group split (issue #575).

The gateway's generic service clients complete their response futures via
executor callbacks. When those callbacks live in the node's default
MutuallyExclusive callback group - the same group as the discovery refresh
timers - one long refresh pass stalls every in-flight RPC response no matter
how many executor threads are configured. This test manufactures that stall
deterministically and asserts a service-backed operation still completes
fast, which requires the response callback to run in the shared Reentrant
RPC group where a second executor thread can dispatch it:

- Aggregation is enabled with a single peer pointing at an RFC 5737
  TEST-NET-1 address (192.0.2.1). TCP connects to that address are
  black-holed (SYN never answered), so every peer health check inside
  ``refresh_cache()`` blocks the refresh timer callback for the full
  health budget (capped at 1s inside ``PeerClient``) instead of failing
  fast. ``test_blackhole_peer_actually_stalls`` guards that precondition.
- ``refresh_interval_ms`` is set well below that stall, so the backstop
  refresh timer is permanently past due and refresh passes run back to
  back: the default callback group is continuously occupied by refresh
  work for the lifetime of the test.
- A service-backed operation is executed mid-stall. Its response can only
  be delivered by an executor callback. With the response callbacks in the
  shared Reentrant RPC group, a second executor thread dispatches the
  response immediately; with them in the default group, the response sits
  undeliverable until the 10s service budget expires and the gateway
  reports a bogus "Service call timed out".
"""

import socket
import time
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_time_scale
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

# RFC 5737 TEST-NET-1: guaranteed unassigned; TCP connects are black-holed
# (no SYN-ACK, no RST), so the peer health check blocks for its full budget.
BLACKHOLE_PEER_HOST = '192.0.2.1'
BLACKHOLE_PEER_PORT = 9100
BLACKHOLE_PEER_URL = f'http://{BLACKHOLE_PEER_HOST}:{BLACKHOLE_PEER_PORT}'

# Well below the ~1s per-pass health-check stall so the backstop refresh
# timer is always past due and a stalled pass is always in flight.
REFRESH_INTERVAL_MS = 200

# The gateway's service budget is 10s (service_call_timeout_sec default).
# The pre-fix behaviour was to burn the whole budget and fail with 500;
# the fixed gateway answers in well under a second even mid-stall. 8s keeps
# CI headroom while staying decisively below the failure mode.
#
# Scaled by MEDKIT_TEST_TIME_SCALE so the sanitizer jobs - which multiply
# every ctest timeout but cannot reach an assertion inside a test - do not
# read their own instrumentation overhead as a starved response. The
# unscaled value stays 8.0 so the falsifier keeps its edge where it was
# proven red (main burns the full 10s budget and answers 500, which no
# scaling can turn into a pass).
FAST_COMPLETION_BUDGET_SEC = 8.0 * get_time_scale()


def generate_test_description():
    return create_test_launch(
        demo_nodes=['calibration'],
        fault_manager=False,
        gateway_params={
            'refresh_interval_ms': REFRESH_INTERVAL_MS,
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [BLACKHOLE_PEER_URL],
            'aggregation.peer_names': ['blackhole_peer'],
        },
    )


class TestExecutorStarvation(GatewayTestCase):
    """A blocking-RPC response is dispatched while refresh hogs the default group."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'calibration'}

    def test_blackhole_peer_actually_stalls(self):
        """Precondition: this environment must black-hole TEST-NET-1 connects.

        If the network rejects 192.0.2.1 quickly (ICMP unreachable / RST),
        the refresh pass never stalls and the starvation assertion below
        would pass vacuously on broken code. Fail loudly in that case so
        the fixture gets fixed rather than silently proving nothing.
        """
        start = time.monotonic()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        try:
            sock.connect((BLACKHOLE_PEER_HOST, BLACKHOLE_PEER_PORT))
        except socket.timeout:
            pass  # expected: SYN black-holed until our own timeout fires
        except OSError:
            pass  # fast failure - caught by the elapsed assertion below
        finally:
            sock.close()
        elapsed = time.monotonic() - start
        self.assertGreater(
            elapsed, 1.5,
            f'TEST-NET-1 connect failed after only {elapsed:.2f}s instead of '
            f'black-holing - this environment cannot manufacture the refresh '
            f'stall, so the starvation falsifier would be vacuous',
        )

    def test_service_operation_completes_during_refresh_stall(self):
        """A service-backed operation must not be starved by refresh passes.

        The measured request runs while refresh passes (each stalled ~1s on
        the black-hole peer health check) occupy the default callback group
        back to back. Only the operation call itself is timed - discovery
        data lands late here because every pass includes the stall, so
        readiness gets its own generous poll first.
        """
        self.wait_for_operation('/apps/calibration', 'calibrate', max_wait=45.0)

        start = time.monotonic()
        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate/executions',
            json={'parameters': {}},
            timeout=30,
        )
        elapsed = time.monotonic() - start

        self.assertEqual(
            response.status_code, 200,
            f'Service-backed operation failed during the refresh stall '
            f'(HTTP {response.status_code} after {elapsed:.1f}s): '
            f'{response.text}',
        )
        self.assertLess(
            elapsed, FAST_COMPLETION_BUDGET_SEC,
            f'Operation took {elapsed:.1f}s - the service response was '
            f'starved behind the stalled default callback group instead of '
            f'being dispatched by a second executor thread',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exit cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
