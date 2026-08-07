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

"""Shared constants for ros2_medkit integration tests."""

import os

API_BASE_PATH = '/api/v1'
DEFAULT_PORT = int(os.environ.get('GATEWAY_TEST_PORT', '8080'))
DEFAULT_DOMAIN_ID = int(os.environ.get('ROS_DOMAIN_ID', '0'))
DEFAULT_BASE_URL = f'http://localhost:{DEFAULT_PORT}{API_BASE_PATH}'


def get_test_port(offset=0):
    """Return the assigned test port plus an optional offset.

    Each integration test gets a unique ``GATEWAY_TEST_PORT`` from CMake.
    Tests that launch multiple gateway instances use *offset* to get
    additional non-colliding ports (e.g. ``get_test_port(1)``).
    """
    return DEFAULT_PORT + offset


def _secondary_domains():
    """Return the shared secondary DDS domains, as handed down by CMake.

    ``MEDKIT_SECONDARY_DOMAINS`` is set on every integration test from
    ``MEDKIT_SECONDARY_DOMAIN_RANGE`` in ``ROS2MedkitTestDomain.cmake``,
    so the numbers are written once. Absent (a test run by hand outside
    CTest), there is nothing to fall back on that would be safe, so say so.
    """
    raw = os.environ.get('MEDKIT_SECONDARY_DOMAINS', '')
    return [int(part) for part in raw.split(',') if part.strip()]


def get_test_domain_id(offset=0):
    """Return a DDS domain ID for this test, optionally with an offset.

    Each integration test gets a ``ROS_DOMAIN_ID`` from its package pool in
    ``ROS2MedkitTestDomain.cmake``. For offset 0, returns that domain.

    Offsets above 0 are for multi-gateway tests that need a second or third
    domain at the same time. Those come from the shared secondary pool, which
    sits outside every package pool and is shared across all such tests; CTest
    serialises them with a ``RESOURCE_LOCK`` (see ``CMakeLists.txt``) so two of
    them never hold the same secondary domain at once.

    Only domains whose UDP port slice falls outside the kernel ephemeral port
    range are usable at all - see the header of ``ROS2MedkitTestDomain.cmake``.
    That is why the secondary pool is small and why the offset is bounded.
    """
    if offset == 0:
        return DEFAULT_DOMAIN_ID
    secondary = _secondary_domains()
    if not secondary:
        raise RuntimeError(
            'MEDKIT_SECONDARY_DOMAINS is not set, so no secondary DDS domain can be '
            'handed out. CTest sets it from the allocation table in '
            'ROS2MedkitTestDomain.cmake. To run this test by hand, set it yourself to '
            'the same value, for example MEDKIT_SECONDARY_DOMAINS=229,230,231.'
        )
    if not 1 <= offset <= len(secondary):
        raise ValueError(
            f'secondary DDS domain offset {offset} out of range 1..{len(secondary)} '
            f'(the shared secondary pool is {secondary}). Widen '
            'MEDKIT_SECONDARY_DOMAIN_RANGE in ROS2MedkitTestDomain.cmake before '
            'adding a new offset - it can only grow into the safe band.'
        )
    return secondary[offset - 1]


# Gateway startup
GATEWAY_STARTUP_TIMEOUT = 30.0
GATEWAY_STARTUP_INTERVAL = 0.5

# Discovery
DISCOVERY_TIMEOUT = 60.0
DISCOVERY_INTERVAL = 0.5  # seconds between discovery polls

# Parameter service readiness
# A node's ROS 2 parameter service can lag its graph discovery, and the
# configurations endpoint returns 503 until it responds. This lag is small
# without instrumentation but grows under the TSan job, where the service was
# still 503 more than 15s after discovery finished. Generous on purpose: a
# larger timeout costs nothing on the passing path (the poll returns the moment
# the endpoint answers 200) and only bounds how long a genuinely dead service
# waits before failing, well inside the sanitizer jobs' 360s ctest budget.
PARAM_SERVICE_TIMEOUT = 90.0

# Operations
ACTION_TIMEOUT = 30.0

# Faults
FAULT_TIMEOUT = 30.0
ROSBAG_TIMEOUT = 30.0
SNAPSHOT_TIMEOUT = 30.0

# Shutdown
ALLOWED_EXIT_CODES = {0, -2, -15}  # OK, SIGINT, SIGTERM
