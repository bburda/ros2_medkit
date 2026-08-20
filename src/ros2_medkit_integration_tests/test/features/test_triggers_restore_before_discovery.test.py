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

"""Feature test: a restored persistent trigger outlives its entity's discovery.

A gateway restores persistent triggers once, while it is being constructed,
and then sweeps triggers whose entity is missing from the discovery cache.
Straight after a restart the cache is empty for as long as DDS takes to report
the nodes that are already running, so for a restored trigger "missing from the
cache" means "not discovered yet", not "gone". A sweep that cannot tell those
apart deletes the trigger from the shared store, and because restore never runs
again the trigger is unrecoverable for the life of the process.

Nothing here is raced or timed. The restarted gateway lives on a DDS domain of
its own, the only node that can put its entity on that domain is started by the
test rather than by a clock, and the sweep cadence is pinned to the fastest the
parameter allows - so the window in which the entity is provably absent is as
long as the test says, whatever the machine is doing.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node

PORT_PRIMARY = get_test_port(0)
PORT_RESTARTED = get_test_port(1)

BASE_URL_PRIMARY = f'http://localhost:{PORT_PRIMARY}{API_BASE_PATH}'
BASE_URL_RESTARTED = f'http://localhost:{PORT_RESTARTED}{API_BASE_PATH}'

# The restarted gateway gets a domain to itself so the primary's demo node is
# invisible to it: the only ``temp_sensor`` it can ever discover is the one the
# test starts on that domain.
PRIMARY_DOMAIN = get_test_domain_id(0)
RESTARTED_DOMAIN = get_test_domain_id(1)

# Sweep cadence of the restarted gateway. 100 ms is the smallest
# ``refresh_interval_ms`` the gateway accepts, so this is the configuration in
# which the sweep is furthest from being a proxy for DDS convergence.
SWEEP_INTERVAL_MS = 100

# How long the restarted gateway is left running with its entity provably
# absent. Two orders of magnitude above the sweep cadence, so the restored
# trigger is offered to the sweep many times over.
SWEEP_WINDOW_SECONDS = 3.0

DB_PATH = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_restore_before_discovery_{os.getpid()}.db',
)

# Two handshakes, both written by the test. The restarted gateway must not
# start before the trigger is in the shared store, and its entity's node must
# not start before the test has watched the gateway run without it.
GATE_RESTART = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_restore_before_discovery_restart_{os.getpid()}',
)
GATE_ENTITY = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_restore_before_discovery_entity_{os.getpid()}',
)

APP_ID = 'temp_sensor'
RESOURCE_URI = f'/api/v1/apps/{APP_ID}/faults'


def _gate_process(name, path):
    """Return a process that exits once ``path`` exists."""
    return ExecuteProcess(
        cmd=['sh', '-c', f'while [ ! -e "{path}" ]; do sleep 0.2; done'],
        name=name,
        output='screen',
    )


def generate_test_description():
    """Launch a primary gateway and a restarted one whose entity arrives late."""
    primary = create_gateway_node(
        name=f'ros2_medkit_gateway_{PORT_PRIMARY}',
        port=PORT_PRIMARY,
        extra_params={
            'triggers.enabled': True,
            'triggers.storage.path': DB_PATH,
            'triggers.on_restart_behavior': 'reset',
        },
        extra_env={'ROS_DOMAIN_ID': str(PRIMARY_DOMAIN)},
    )
    restarted = create_gateway_node(
        name=f'ros2_medkit_gateway_{PORT_RESTARTED}',
        port=PORT_RESTARTED,
        extra_params={
            'triggers.enabled': True,
            'triggers.storage.path': DB_PATH,
            'triggers.on_restart_behavior': 'restore',
            'refresh_interval_ms': SWEEP_INTERVAL_MS,
        },
        extra_env={'ROS_DOMAIN_ID': str(RESTARTED_DOMAIN)},
    )

    primary_demo = create_demo_nodes(
        [APP_ID], lidar_faulty=False,
        extra_env={'ROS_DOMAIN_ID': str(PRIMARY_DOMAIN)},
    )
    restarted_demo = create_demo_nodes(
        [APP_ID], lidar_faulty=False,
        extra_env={'ROS_DOMAIN_ID': str(RESTARTED_DOMAIN)},
    )

    delayed_primary_demo = TimerAction(
        period=2.0,
        actions=primary_demo + [launch_testing.actions.ReadyToTest()],
    )

    restart_gate = _gate_process('restart_gate', GATE_RESTART)
    entity_gate = _gate_process('entity_gate', GATE_ENTITY)

    return (
        LaunchDescription([
            primary,
            restart_gate,
            RegisterEventHandler(
                OnProcessExit(target_action=restart_gate, on_exit=[restarted]),
            ),
            entity_gate,
            RegisterEventHandler(
                OnProcessExit(target_action=entity_gate, on_exit=restarted_demo),
            ),
            delayed_primary_demo,
        ]),
        {'primary': primary, 'restarted': restarted},
    )


def _remove_gates():
    """Drop the gate files so a rerun does not inherit an open gate."""
    for path in (GATE_RESTART, GATE_ENTITY):
        if os.path.exists(path):
            try:
                os.unlink(path)
            except OSError:
                pass


def _open_gate(path):
    """Write a gate file, releasing the process that waits on it."""
    with open(path, 'w', encoding='utf-8') as gate:
        gate.write('open')


def _wait_for_health(base_url, *, timeout=60.0):
    """Poll /health until 200 or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            r = requests.get(f'{base_url}/health', timeout=2)
            if r.status_code == 200:
                return
        except requests.exceptions.RequestException:
            pass
        time.sleep(0.5)
    raise AssertionError(f'Gateway at {base_url} not healthy after {timeout}s')


def _wait_for_app(base_url, app_id, *, timeout=60.0):
    """Poll /apps until the given app_id is discovered."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            r = requests.get(f'{base_url}/apps/{app_id}', timeout=2)
            if r.status_code == 200:
                return
        except requests.exceptions.RequestException:
            pass
        time.sleep(0.5)
    raise AssertionError(
        f'App {app_id!r} not discovered at {base_url} after {timeout}s'
    )


class TestTriggersRestoreBeforeDiscovery(GatewayTestCase):
    """A restored trigger survives until its entity is discovered."""

    BASE_URL = BASE_URL_PRIMARY

    MIN_EXPECTED_APPS = 0
    REQUIRED_APPS: set = set()
    REQUIRED_AREAS: set = set()

    _trigger_id: str = ''

    @classmethod
    def setUpClass(cls):
        """Wait for the primary gateway and its demo node."""
        _wait_for_health(BASE_URL_PRIMARY, timeout=60.0)
        _wait_for_app(BASE_URL_PRIMARY, APP_ID, timeout=60.0)
        cls.addClassCleanup(_remove_gates)

    # @verifies REQ_INTEROP_029
    def test_01_create_persistent_trigger(self):
        """POST a persistent trigger on the primary gateway, then restart."""
        body = {
            'resource': RESOURCE_URI,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'persistent': True,
            'lifetime': 3600,
        }
        r = requests.post(
            f'{BASE_URL_PRIMARY}/apps/{APP_ID}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig = r.json()
        self.assertTrue(trig.get('persistent'), 'trigger must be persistent')
        TestTriggersRestoreBeforeDiscovery._trigger_id = trig['id']

        # The row is in the shared store, which is the precondition the restore
        # path needs. Opening this gate starts the restarted gateway.
        _open_gate(GATE_RESTART)

    # @verifies REQ_INTEROP_096
    def test_02_restored_trigger_survives_until_entity_is_discovered(self):
        """The restored trigger is still there once its entity finally appears.

        The restarted gateway sweeps for orphaned triggers throughout a window
        in which its entity does not exist on its domain at all. The trigger
        must outlive that window: nothing has disappeared from this gateway's
        discovery, so nothing is orphaned.
        """
        self.assertTrue(
            self._trigger_id,
            'test_01 must set _trigger_id before test_02 runs',
        )

        _wait_for_health(BASE_URL_RESTARTED, timeout=60.0)

        # The entity is absent by construction: the only node that could put it
        # on this domain is started by the gate below, and nothing has opened
        # that gate yet.
        r = requests.get(f'{BASE_URL_RESTARTED}/apps/{APP_ID}', timeout=5)
        self.assertEqual(
            r.status_code, 404,
            f'{APP_ID!r} must not exist on the restarted gateway before its '
            f'gate is opened - without that window this test proves nothing, '
            f'got {r.status_code}',
        )

        time.sleep(SWEEP_WINDOW_SECONDS)

        _open_gate(GATE_ENTITY)
        _wait_for_app(BASE_URL_RESTARTED, APP_ID, timeout=60.0)

        r = requests.get(
            f'{BASE_URL_RESTARTED}/apps/{APP_ID}/triggers/{self._trigger_id}',
            timeout=5,
        )
        self.assertEqual(
            r.status_code, 200,
            f'Restored trigger {self._trigger_id!r} was dropped while its '
            f'entity was still being discovered: GET returned {r.status_code}: '
            f'{r.text}',
        )
        trig = r.json()
        self.assertEqual(trig['status'], 'active')
        self.assertTrue(trig.get('persistent'))
        self.assertEqual(trig.get('observed_resource'), RESOURCE_URI)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )

        # SQLite in WAL mode writes two sidecars next to the DB, and a run that
        # leaves them behind leaves state a later run can find.
        for path in (DB_PATH, f'{DB_PATH}-wal', f'{DB_PATH}-shm'):
            if os.path.exists(path):
                try:
                    os.unlink(path)
                except OSError:
                    pass
        _remove_gates()
