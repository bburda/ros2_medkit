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

Surviving the sweep is only half of it. A restored data trigger also has to end
up subscribed to its topic, and it is the subscription - not the record - that
decides whether the trigger can ever fire. A trigger that keeps its record but
loses its subscription reports itself active and stays silent, which is worse
than one that was deleted, because nothing says so. So the same window that
proves the record survives is used to prove the subscription attempt does: it
is longer than any budget the trigger subsystem gives a single attempt, and at
the end of it the trigger is required to deliver a real event.

Nothing here is raced or timed. The restarted gateway lives on a DDS domain of
its own, the only node that can put its entity on that domain is started by the
test rather than by a clock, and the sweep cadence is pinned to the fastest the
parameter allows - so the window in which the entity is provably absent is as
long as the test says, whatever the machine is doing.
"""

import json
import os
import tempfile
import threading
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
# absent. Two things set this length. It is hundreds of sweep cadences, so the
# restored trigger is offered to the sweep many times over. And it outlasts the
# 60 s budget the trigger subsystem gives a single subscription attempt, which
# is what makes the firing check below discriminating: a gateway that abandons
# the attempt on that budget survives any shorter window unnoticed.
ABSENCE_WINDOW_SECONDS = 75.0

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

# The topic the temp_sensor demo node publishes, and the data resource that
# maps onto it. A data trigger is the one that needs a live topic subscription
# behind it, so it is the one that can be silently dead.
DATA_TOPIC = '/powertrain/engine/temperature'
DATA_RESOURCE_URI = f'/api/v1/apps/{APP_ID}/data{DATA_TOPIC}'


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


def _wait_for_data_items(base_url, app_id, *, timeout=60.0):
    """Poll /apps/{id}/data until the app's topics are in the entity cache.

    The data trigger created below is meant to be a fully resolved one, the
    same shape an operator's is: that needs the app's topics discovered, which
    happens a refresh cycle after the app itself.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            r = requests.get(f'{base_url}/apps/{app_id}/data', timeout=5)
            if r.status_code == 200 and r.json().get('items'):
                return
        except requests.exceptions.RequestException:
            pass
        time.sleep(0.5)
    raise AssertionError(
        f'App {app_id!r} exposed no data items at {base_url} after {timeout}s'
    )


def _collect_trigger_events(events_url, wanted, *, timeout):
    """Read up to ``wanted`` SSE events from a trigger stream.

    The read runs on its own daemon thread and the caller's wait is bounded by
    a wall clock. A trigger with no subscription behind it holds its stream
    open and puts nothing on it, and a blocking read there hands ctest a killed
    file with no test name in it instead of a named assertion failure.
    """
    received = []
    done = threading.Event()

    def collect():
        try:
            with requests.get(events_url, stream=True, timeout=timeout) as resp:
                if resp.status_code != 200:
                    return
                for line in resp.iter_lines(decode_unicode=True):
                    if done.is_set():
                        return
                    if line and line.startswith('data: '):
                        received.append(json.loads(line[6:]))
                        if len(received) >= wanted:
                            return
        except requests.exceptions.RequestException:
            pass
        finally:
            done.set()

    thread = threading.Thread(target=collect, daemon=True)
    thread.start()
    done.wait(timeout=timeout)
    done.set()
    thread.join(timeout=5)
    return list(received)


class TestTriggersRestoreBeforeDiscovery(GatewayTestCase):
    """A restored trigger survives until its entity is discovered."""

    BASE_URL = BASE_URL_PRIMARY

    MIN_EXPECTED_APPS = 0
    REQUIRED_APPS: set = set()
    REQUIRED_AREAS: set = set()

    _trigger_id: str = ''
    _data_trigger_id: str = ''

    @classmethod
    def setUpClass(cls):
        """Wait for the primary gateway and its demo node."""
        _wait_for_health(BASE_URL_PRIMARY, timeout=60.0)
        _wait_for_app(BASE_URL_PRIMARY, APP_ID, timeout=60.0)
        _wait_for_data_items(BASE_URL_PRIMARY, APP_ID, timeout=60.0)
        cls.addClassCleanup(_remove_gates)

    def _create_persistent_trigger(self, resource_uri):
        """POST a persistent multishot OnChange trigger and return its body."""
        body = {
            'resource': resource_uri,
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
        self.assertEqual(
            r.status_code, 201, f'Create failed for {resource_uri}: {r.text}',
        )
        trig = r.json()
        self.assertTrue(trig.get('persistent'), 'trigger must be persistent')
        return trig

    # @verifies REQ_INTEROP_029
    def test_01_create_persistent_triggers(self):
        """POST both persistent triggers on the primary gateway, then restart.

        The faults trigger carries no subscription, so it only exercises the
        record. The data trigger needs a live topic subscription behind it,
        which is what the later window puts under strain.
        """
        cls = TestTriggersRestoreBeforeDiscovery
        cls._trigger_id = self._create_persistent_trigger(RESOURCE_URI)['id']
        cls._data_trigger_id = self._create_persistent_trigger(
            DATA_RESOURCE_URI,
        )['id']

        # Both rows are in the shared store, which is the precondition the
        # restore path needs. Opening this gate starts the restarted gateway.
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

        time.sleep(ABSENCE_WINDOW_SECONDS)

        # Still absent after the window. If this fails the window was not an
        # absence window at all and neither this case nor test_03 means
        # anything, so it is checked before the gate is opened.
        r = requests.get(f'{BASE_URL_RESTARTED}/apps/{APP_ID}', timeout=5)
        self.assertEqual(
            r.status_code, 404,
            f'{APP_ID!r} appeared on the restarted gateway during the window '
            f'that is supposed to prove its absence, got {r.status_code}',
        )

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

    # @verifies REQ_INTEROP_097
    def test_03_restored_data_trigger_fires_after_late_discovery(self):
        """The restored data trigger delivers events once its entity arrives.

        Its entity turned up later than the budget a single subscription
        attempt gets, so this is the case in which a trigger that keeps only
        its record goes quiet. Being listed is not evidence here - the trigger
        has to put a real sample on its event stream.
        """
        self.assertTrue(
            self._data_trigger_id,
            'test_01 must set _data_trigger_id before test_03 runs',
        )
        # test_02 opened the entity gate; the node needs to be discovered
        # before its topic can be subscribed.
        _wait_for_app(BASE_URL_RESTARTED, APP_ID, timeout=60.0)

        url = (
            f'{BASE_URL_RESTARTED}/apps/{APP_ID}/triggers/'
            f'{self._data_trigger_id}'
        )
        r = requests.get(url, timeout=5)
        self.assertEqual(
            r.status_code, 200,
            f'Restored data trigger {self._data_trigger_id!r} is not there: '
            f'{r.status_code}: {r.text}',
        )
        trig = r.json()
        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig.get('observed_resource'), DATA_RESOURCE_URI)

        events_url = (
            f'{BASE_URL_RESTARTED}'
            f'{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )
        # The demo node publishes at 2 Hz and deferred resolution runs on a 5 s
        # tick, so this covers several ticks over: a failure here means "never
        # fired", not "not yet".
        events = _collect_trigger_events(events_url, 1, timeout=45)

        self.assertGreaterEqual(
            len(events), 1,
            f'Restored data trigger {self._data_trigger_id!r} reports itself '
            f'active but never delivered an event after its entity appeared - '
            f'its subscription did not outlive the wait for that entity',
        )
        for event in events:
            self.assertIn('timestamp', event)
            self.assertIn('payload', event)

    def test_04_waiting_for_an_undiscovered_entity_is_reported(self, proc_output, restarted):
        """The gateway says which trigger is still waiting for its entity.

        Holding the attempt open indefinitely is right, and invisible only if
        nobody is told. The notice is what keeps an entity that never appears
        from looking like a trigger that resolved.
        """
        self.assertTrue(
            self._data_trigger_id,
            'test_01 must set _data_trigger_id before test_04 runs',
        )
        # Concatenated with no separator: proc_output yields raw stream chunks,
        # and joining with a newline splices one into the middle of a log line.
        text = ''.join(
            output.text.decode(errors='replace')
            for output in proc_output[restarted]
        )
        notices = [
            line for line in text.splitlines()
            if 'has not appeared in discovery' in line
        ]
        self.assertTrue(
            notices,
            'The restarted gateway spent the whole absence window unable to '
            'resolve a restored data trigger and never said so',
        )
        self.assertTrue(
            any(self._data_trigger_id in line for line in notices),
            f'The notice does not name the trigger that is waiting: {notices}',
        )


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
