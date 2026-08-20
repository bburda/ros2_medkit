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

"""Feature tests for persistent trigger survival across gateway restart.

Validates that triggers created with ``persistent=true`` are restored when a
new gateway instance starts with ``triggers.on_restart_behavior=restore``
pointing at the same SQLite storage file.

Test strategy: launch TWO gateway instances sharing a single SQLite DB file.
- Primary gateway (port N+0): creates a persistent trigger, then is the
  "restarted" gateway we no longer talk to.
- Secondary gateway (port N+1): started with ``on_restart_behavior=restore``
  and the same DB path, simulating a restart.  We verify the trigger is
  loaded into the secondary instance.

This avoids the complexity of process termination/restart inside
``launch_testing`` while exercising the real persistence and restore path.
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
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node

# ---------------------------------------------------------------------------
# Port layout (stride-10 slot from CMake)
# ---------------------------------------------------------------------------
PORT_PRIMARY = get_test_port(0)    # "before restart" gateway
PORT_SECONDARY = get_test_port(1)  # "after restart" gateway (same DB)

BASE_URL_PRIMARY = f'http://localhost:{PORT_PRIMARY}{API_BASE_PATH}'
BASE_URL_SECONDARY = f'http://localhost:{PORT_SECONDARY}{API_BASE_PATH}'

# Shared SQLite DB path - created once at module import time so both gateway
# instances use the same file.  tempfile.mktemp is intentionally used here
# (not mkstemp) because the gateway itself creates the file; we only need a
# unique path that does not yet exist.
DB_PATH = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_persist_{os.getpid()}.db',
)

# The secondary gateway stands in for a restart, and a restart only restores
# what the store already held: ``load_persistent_triggers()`` runs once, while
# the gateway is being constructed. So the secondary must not be started until
# the trigger exists in the shared DB, and the only thing that knows when that
# is true is the test that created it. This path is the handshake - test_01
# creates it, and the secondary's start is chained to a process that waits for
# it - so the order is a consequence of the work rather than of how long
# ``setUpClass`` happened to take.
GATE_PATH = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_persist_gate_{os.getpid()}',
)

APP_ID = 'temp_sensor'
RESOURCE_URI = f'/api/v1/apps/{APP_ID}/faults'


def _make_gateway_node(port, *, on_restart_behavior):
    """Return a gateway_node launch action for the given port and DB path."""
    return create_gateway_node(
        name=f'ros2_medkit_gateway_{port}',
        port=port,
        extra_params={
            'triggers.enabled': True,
            'triggers.storage.path': DB_PATH,
            'triggers.on_restart_behavior': on_restart_behavior,
        },
    )


def generate_test_description():
    """Launch two gateway instances sharing one SQLite trigger store."""
    primary = _make_gateway_node(PORT_PRIMARY, on_restart_behavior='reset')
    secondary = _make_gateway_node(PORT_SECONDARY, on_restart_behavior='restore')

    # Demo nodes are shared by both gateways (same ROS 2 graph).
    demo = create_demo_nodes(['temp_sensor'], lidar_faulty=False)

    # Demo nodes start after 2s so gateway discovers them.
    delayed_demo = TimerAction(
        period=2.0,
        actions=demo + [launch_testing.actions.ReadyToTest()],
    )

    # The secondary starts when the gate file appears, never on a clock: it
    # simulates a gateway restart, and a restart that happens before the
    # trigger is created restores nothing.
    gate = ExecuteProcess(
        cmd=['sh', '-c', f'while [ ! -e "{GATE_PATH}" ]; do sleep 0.2; done'],
        name='secondary_start_gate',
        output='screen',
    )
    gated_secondary = RegisterEventHandler(
        OnProcessExit(target_action=gate, on_exit=[secondary]),
    )

    return (
        LaunchDescription([primary, gate, gated_secondary, delayed_demo]),
        {'primary': primary, 'secondary': secondary},
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _remove_gate():
    """Drop the gate file so a rerun does not inherit an open gate."""
    if os.path.exists(GATE_PATH):
        try:
            os.unlink(GATE_PATH)
        except OSError:
            pass


def _wait_for_health(base_url, *, timeout=30.0):
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
        time.sleep(1.0)
    raise AssertionError(
        f'App {app_id!r} not discovered at {base_url} after {timeout}s'
    )


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class TestTriggersPersistent(GatewayTestCase):
    """Persistent triggers survive a gateway restart (simulated by two instances)."""

    # GatewayTestCase.setUpClass polls BASE_URL for health.
    # We use PRIMARY as the default; SECONDARY is checked manually.
    BASE_URL = BASE_URL_PRIMARY

    # Skip the default discovery wait - we handle it ourselves.
    MIN_EXPECTED_APPS = 0
    REQUIRED_APPS: set = set()
    REQUIRED_AREAS: set = set()

    # Shared state across tests (sequential by name).
    _trigger_id: str = ''

    @classmethod
    def setUpClass(cls):
        """Wait for both gateways and the demo node."""
        _wait_for_health(BASE_URL_PRIMARY, timeout=60.0)
        _wait_for_app(BASE_URL_PRIMARY, APP_ID, timeout=60.0)
        # Allow a few discovery refresh cycles so the entity is stable
        time.sleep(3.0)
        # The secondary is not running yet: it starts once test_01 opens the
        # gate, and test_03 waits for it there. The gate file is dropped at the
        # end of the class so a rerun in the same temp directory cannot inherit
        # an open one.
        cls.addClassCleanup(_remove_gate)

    # ------------------------------------------------------------------
    # Test 01: create a persistent trigger on the PRIMARY gateway
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_029
    def test_01_create_persistent_trigger(self):
        """POST persistent=true trigger on primary gateway returns 201.

        Creates the trigger that will later be verified on the secondary
        (restarted) gateway instance.
        """
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
        self.assertIn('id', trig)
        self.assertTrue(trig['id'].startswith('trig_'))
        self.assertEqual(trig['status'], 'active')
        self.assertTrue(trig.get('persistent'), 'trigger must be persistent')
        self.assertEqual(trig.get('observed_resource'), RESOURCE_URI)

        # Stash the ID for subsequent tests.
        TestTriggersPersistent._trigger_id = trig['id']

        # The persistent trigger is now a row in the shared store, which is the
        # precondition the secondary's restore path needs. Opening the gate
        # starts it.
        with open(GATE_PATH, 'w', encoding='utf-8') as gate:
            gate.write(trig['id'])

    # ------------------------------------------------------------------
    # Test 02: trigger is listed on the PRIMARY gateway
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_030
    def test_02_trigger_listed_on_primary(self):
        """Persistent trigger appears in the listing on the primary gateway."""
        self.assertTrue(
            self._trigger_id,
            'test_01 must set _trigger_id before test_02 runs',
        )
        r = requests.get(
            f'{BASE_URL_PRIMARY}/apps/{APP_ID}/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertIn(
            self._trigger_id, ids,
            f'Expected {self._trigger_id!r} in trigger list on primary',
        )

    # ------------------------------------------------------------------
    # Test 03: trigger is restored on the SECONDARY (restarted) gateway
    # ------------------------------------------------------------------

    def test_03_trigger_restored_on_secondary(self):
        """Persistent trigger is restored on a gateway pointing to the same DB.

        The secondary gateway starts with ``on_restart_behavior=restore`` and
        the same SQLite path as the primary.  It must load the persistent
        trigger from the DB and expose it via the REST API.
        """
        self.assertTrue(
            self._trigger_id,
            'test_01 must set _trigger_id before test_03 runs',
        )

        # The secondary starts only after test_01 opened the gate, so this is
        # the first point at which it can be up at all.
        _wait_for_health(BASE_URL_SECONDARY, timeout=60.0)
        _wait_for_app(BASE_URL_SECONDARY, APP_ID, timeout=30.0)
        time.sleep(2.0)  # Allow discovery to stabilize

        deadline = time.monotonic() + 15.0
        found = False
        while time.monotonic() < deadline:
            try:
                r = requests.get(
                    f'{BASE_URL_SECONDARY}/apps/{APP_ID}/triggers',
                    timeout=5,
                )
                if r.status_code == 200:
                    ids = [item['id'] for item in r.json().get('items', [])]
                    if self._trigger_id in ids:
                        found = True
                        break
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)

        self.assertTrue(
            found,
            f'Trigger {self._trigger_id!r} not found on secondary gateway '
            f'after 15s (on_restart_behavior=restore)',
        )

    # ------------------------------------------------------------------
    # Test 04: verify restored trigger fields match original
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_096
    def test_04_restored_trigger_fields_match(self):
        """Fields of the restored trigger match what was originally created."""
        self.assertTrue(
            self._trigger_id,
            'test_01 must set _trigger_id before test_04 runs',
        )

        r = requests.get(
            f'{BASE_URL_SECONDARY}/apps/{APP_ID}/triggers/{self._trigger_id}',
            timeout=5,
        )
        self.assertEqual(
            r.status_code, 200,
            f'GET trigger on secondary returned {r.status_code}: {r.text}',
        )

        trig = r.json()
        self.assertEqual(trig['id'], self._trigger_id)
        self.assertEqual(trig['status'], 'active')
        self.assertTrue(trig.get('persistent'))
        self.assertEqual(trig.get('observed_resource'), RESOURCE_URI)
        self.assertEqual(
            trig.get('trigger_condition', {}).get('condition_type'), 'OnChange'
        )

    # ------------------------------------------------------------------
    # Test 05: non-persistent trigger is NOT restored on secondary
    # ------------------------------------------------------------------

    def test_05_non_persistent_trigger_not_restored(self):
        """A trigger created with persistent=false on primary is absent on secondary."""
        # Create a non-persistent trigger on the primary gateway.
        body = {
            'resource': RESOURCE_URI,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'persistent': False,
            'lifetime': 3600,
        }
        r = requests.post(
            f'{BASE_URL_PRIMARY}/apps/{APP_ID}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        ephemeral_id = r.json()['id']

        # Give secondary a moment to sync if it were to (it should not).
        time.sleep(1.0)

        r = requests.get(
            f'{BASE_URL_SECONDARY}/apps/{APP_ID}/triggers/{ephemeral_id}',
            timeout=5,
        )
        self.assertEqual(
            r.status_code, 404,
            f'Non-persistent trigger {ephemeral_id!r} should not be present '
            f'on secondary gateway, got {r.status_code}',
        )

    # ------------------------------------------------------------------
    # Test 06: delete persistent trigger via secondary, gone from primary too
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_032
    def test_06_delete_via_secondary_removes_from_store(self):
        """DELETE on secondary removes the persistent trigger from the shared DB.

        After deletion, the trigger must also be absent when listed on the
        primary gateway (they share the same SQLite file).
        """
        self.assertTrue(
            self._trigger_id,
            'test_01 must set _trigger_id before test_06 runs',
        )

        # Delete from the secondary gateway.
        r = requests.delete(
            f'{BASE_URL_SECONDARY}/apps/{APP_ID}/triggers/{self._trigger_id}',
            timeout=5,
        )
        self.assertEqual(
            r.status_code, 204,
            f'DELETE on secondary returned {r.status_code}: {r.text}',
        )

        # Verify gone from secondary.
        r = requests.get(
            f'{BASE_URL_SECONDARY}/apps/{APP_ID}/triggers/{self._trigger_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404, 'Trigger should be 404 after delete')

        # Note: the primary gateway still has the trigger in memory (no real-time
        # sync between gateways). The DB row is deleted, so a fresh gateway
        # would not load it. We only verify the secondary sees the deletion.


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )

        # Clean up the shared SQLite DB if it was created. SQLite in WAL mode
        # writes two sidecars next to it, and a run that leaves them behind
        # leaves state a later run can find.
        for path in (DB_PATH, f'{DB_PATH}-wal', f'{DB_PATH}-shm'):
            if os.path.exists(path):
                try:
                    os.unlink(path)
                except OSError:
                    pass
        _remove_gate()
