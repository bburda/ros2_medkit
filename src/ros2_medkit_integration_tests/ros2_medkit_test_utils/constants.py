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
DEFAULT_BASE_URL = f'http://localhost:{DEFAULT_PORT}{API_BASE_PATH}'


def get_time_scale():
    """Return the multiplier for in-test wall-clock budgets.

    CTest timeouts are stretched by the sanitizer CI jobs (a generic x3
    rewrite of every declared ``TIMEOUT``), but budgets asserted *inside* a
    test are invisible to that rewrite: an ASan/TSan-instrumented gateway can
    blow a hard-coded "must answer within N seconds" assertion long before
    ctest's own clock runs out, and the failure then reads as a regression
    rather than as sanitizer overhead. Those jobs set
    ``MEDKIT_TEST_TIME_SCALE`` to the same factor they apply to ctest.

    Unset / unparseable / below 1.0 means "no scaling", so the normal jobs
    keep the tight budgets that give the assertions their falsifying power.
    """
    try:
        scale = float(os.environ.get('MEDKIT_TEST_TIME_SCALE', '1'))
    except ValueError:
        return 1.0
    return scale if scale >= 1.0 else 1.0


def get_test_port(offset=0):
    """Return the assigned test port plus an optional offset.

    Each integration test gets a unique ``GATEWAY_TEST_PORT`` from CMake.
    Tests that launch multiple gateway instances use *offset* to get
    additional non-colliding ports (e.g. ``get_test_port(1)``).
    """
    return DEFAULT_PORT + offset


def _secondary_domains():
    """Return the extra DDS domains this test was given, beyond its own.

    The domain wrapper allocates them when the test starts and publishes them as
    ``MEDKIT_SECONDARY_DOMAINS``; it holds every one of them, through an open
    socket, for as long as the test runs. Absent (a test run by hand outside
    CTest), there is nothing to fall back on that would be safe, so say so.
    """
    raw = os.environ.get('MEDKIT_SECONDARY_DOMAINS', '')
    return [int(part) for part in raw.split(',') if part.strip()]


def _primary_domain():
    """Return the domain the wrapper allocated for this test.

    Deliberately not ``int(os.environ.get('ROS_DOMAIN_ID', '0'))``. Domain 0 is
    the one value this whole scheme exists to keep tests off, and a default of
    it cannot be caught downstream: a test that fell back to 0 is still behind
    the wrapper and still carries MEDKIT_TEST_DOMAINS, so the registration gate
    sees nothing wrong and the test quietly shares a domain with every process
    on the machine. Refuse instead, the way the secondary domains already do.
    """
    raw = os.environ.get('ROS_DOMAIN_ID')
    if raw is None or raw.strip() == '':
        raise RuntimeError(
            'ROS_DOMAIN_ID is not set, so this test has no domain of its own. CTest '
            'runs every test behind medkit_domain_runner.py, which sets it. To run '
            'this test by hand, run it through medkit_run_with_domain.py.'
        )
    domain = int(raw)
    if domain == 0:
        raise RuntimeError(
            'ROS_DOMAIN_ID is 0, the ROS 2 default. A test on domain 0 sees every node '
            'on the machine and is seen by all of them, which is what the domain '
            'wrapper exists to prevent. Something exported it before the wrapper ran.'
        )
    return domain


#: Backwards-compatible name. A module attribute so that importing this module
#: outside CTest still works and only asking for the domain fails.
def __getattr__(name):
    if name == 'DEFAULT_DOMAIN_ID':
        return _primary_domain()
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


def get_test_domain_id(offset=0):
    """Return a DDS domain ID for this test, optionally with an offset.

    Every test gets a ``ROS_DOMAIN_ID`` of its own when it starts, from the
    wrapper described in ``ROS2MedkitTestDomain.cmake``. For offset 0, returns
    that domain.

    Offsets above 0 are for multi-gateway tests that run a second or third
    gateway at the same time. Those domains are allocated to this test alone -
    ``DOMAINS <n>`` on its ``medkit_add_launch_test`` call is what asks for them -
    so no other test can be on them while this one runs.
    """
    if offset == 0:
        return _primary_domain()
    secondary = _secondary_domains()
    if not secondary:
        raise RuntimeError(
            'MEDKIT_SECONDARY_DOMAINS is not set, so no secondary DDS domain can be '
            'handed out. The domain wrapper sets it for a test registered with '
            'DOMAINS greater than 1. To run this test by hand, run it through '
            'medkit_run_with_domain.py --domains <n>, or set ROS_DOMAIN_ID and '
            'MEDKIT_SECONDARY_DOMAINS yourself to domains nobody else is using.'
        )
    if not 1 <= offset <= len(secondary):
        raise ValueError(
            f'secondary DDS domain offset {offset} out of range 1..{len(secondary)} '
            f'(this test was given {secondary}). Raise DOMAINS on its '
            'medkit_add_launch_test call in CMakeLists.txt before adding a new offset.'
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
