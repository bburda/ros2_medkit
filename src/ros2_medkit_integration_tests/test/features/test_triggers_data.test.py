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

"""Feature tests for triggers on data topics (simple + complex types).

Validates trigger CRUD on data collections and SSE event delivery when
topic data changes. Uses the temp_sensor demo node which publishes
Float64 temperature data.

A second gateway runs with ``triggers.enabled: false`` so the feature gate on
the six trigger routes is exercised: with the trigger manager absent the
handlers are never constructed, and the gate is the only thing standing
between a request and a null dereference.

"""

import json
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node, create_test_launch

HTTP_METHODS = {'get', 'post', 'put', 'delete', 'patch', 'head', 'options'}

PORT_TRIGGERS_OFF = get_test_port(1)


def generate_test_description():
    launch_description, context = create_test_launch(
        demo_nodes=['temp_sensor', 'rpm_sensor', 'lidar_sensor'],
        fault_manager=False,
    )
    gateway_triggers_off = create_gateway_node(
        name='gateway_triggers_off',
        port=PORT_TRIGGERS_OFF,
        extra_params={
            'server.host': '127.0.0.1',
            'triggers.enabled': False,
        },
    )
    # Prepend so it comes up alongside the primary gateway, before ReadyToTest.
    launch_description.entities.insert(0, gateway_triggers_off)
    context['gateway_triggers_off'] = gateway_triggers_off
    return launch_description, context


class TestTriggersData(GatewayTestCase):
    """Trigger tests on data topic resources."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor'}

    app_id = None
    topic_id = None
    resource_uri = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway, discover temp sensor, and find a data topic."""
        super().setUpClass()

        cls.app_id = 'temp_sensor'

        # Wait for data availability - skip ROS 2 system topics
        system_topics = {'/parameter_events', '/rosout'}
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                r = requests.get(
                    f'{cls.BASE_URL}/apps/{cls.app_id}/data', timeout=5,
                )
                if r.status_code == 200:
                    items = r.json().get('items', [])
                    for item in items:
                        if item['id'] not in system_topics:
                            cls.topic_id = item['id']
                            cls.resource_uri = (
                                f'/api/v1/apps/{cls.app_id}/data{cls.topic_id}'
                            )
                            break
                    if cls.topic_id:
                        break
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)

        if not cls.topic_id:
            raise AssertionError(
                f'No data items available for app {cls.app_id}'
            )

    def _create_trigger(self, *, condition_type='OnChange',
                        condition_params=None, multishot=True,
                        lifetime=None):
        """Create a trigger on the discovered data topic."""
        body = {
            'resource': self.resource_uri,
            'trigger_condition': {
                'condition_type': condition_type,
            },
            'multishot': multishot,
        }
        if condition_params:
            body['trigger_condition'].update(condition_params)
        if lifetime is not None:
            body['lifetime'] = lifetime
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        return r.json()

    def _delete_trigger(self, trigger_id):
        """Delete a trigger, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trigger_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    # ------------------------------------------------------------------
    # CRUD operations
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_029
    def test_01_create_trigger_returns_201_with_correct_schema(self):
        """POST creates trigger with correct schema response."""
        trig = self._create_trigger(multishot=True, lifetime=120)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertIn('id', trig)
        self.assertTrue(trig['id'].startswith('trig_'))
        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], self.resource_uri)
        self.assertEqual(trig['protocol'], 'sse')
        self.assertTrue(trig['multishot'])

        # event_source must be a valid URI path ending in /events
        self.assertIn('event_source', trig)
        self.assertTrue(trig['event_source'].endswith('/events'))
        self.assertIn(trig['id'], trig['event_source'])

        # trigger_condition preserved
        self.assertIn('trigger_condition', trig)
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'OnChange'
        )

    # @verifies REQ_INTEROP_030
    def test_02_list_triggers_returns_created_trigger(self):
        """GET /triggers returns list with the created trigger."""
        trig = self._create_trigger()
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)

        data = r.json()
        self.assertIn('items', data)
        ids = [item['id'] for item in data['items']]
        self.assertIn(trig['id'], ids)

    # @verifies REQ_INTEROP_096
    def test_03_get_single_trigger(self):
        """GET /triggers/{id} returns the trigger details."""
        trig = self._create_trigger()
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)

        data = r.json()
        self.assertEqual(data['id'], trig['id'])
        self.assertEqual(data['status'], 'active')
        self.assertEqual(data['observed_resource'], self.resource_uri)

    # @verifies REQ_INTEROP_031
    def test_04_update_trigger_lifetime(self):
        """PUT /triggers/{id} updates the lifetime."""
        trig = self._create_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.put(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig["id"]}',
            json={'lifetime': 300},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()['lifetime'], 300)

    # @verifies REQ_INTEROP_032
    def test_05_delete_trigger_returns_204(self):
        """DELETE /triggers/{id} returns 204 and removes the trigger."""
        trig = self._create_trigger()

        r = requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify gone
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_06_empty_list_after_delete(self):
        """After deleting all triggers, list returns empty items."""
        trig = self._create_trigger()
        self._delete_trigger(trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertNotIn(trig['id'], ids)

    # ------------------------------------------------------------------
    # SSE event delivery
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_097
    def test_10_sse_stream_returns_correct_headers(self):
        """The SSE events endpoint returns text/event-stream content type."""
        trig = self._create_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        try:
            r = requests.get(events_url, stream=True, timeout=3)
            self.assertEqual(
                r.headers.get('Content-Type'), 'text/event-stream'
            )
            self.assertEqual(r.headers.get('Cache-Control'), 'no-cache')
            r.close()
        except requests.exceptions.ReadTimeout:
            pass  # Expected - SSE keeps connection open

    # @verifies REQ_INTEROP_097
    def test_11_sse_stream_delivers_data_events(self):
        """SSE stream delivers events when topic data changes.

        The temp_sensor publishes at ~1Hz (Float64), so OnChange triggers
        should fire on each new message.
        """
        trig = self._create_trigger(multishot=True, lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        received_events = []
        stop_event = threading.Event()

        def collect_events():
            try:
                with requests.get(events_url, stream=True, timeout=20) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop_event.is_set():
                            break
                        if line and line.startswith('data: '):
                            data = json.loads(line[6:])
                            received_events.append(data)
                            if len(received_events) >= 2:
                                stop_event.set()
                                break
            except requests.exceptions.RequestException:
                pass

        thread = threading.Thread(target=collect_events, daemon=True)
        thread.start()

        # temp_sensor publishes at ~1Hz, so 2 events should arrive within ~10s
        stop_event.wait(timeout=20)
        thread.join(timeout=5)

        self.assertGreaterEqual(
            len(received_events), 1,
            f'Expected at least 1 SSE event, got {len(received_events)}',
        )

        # Verify EventEnvelope schema
        for event in received_events:
            self.assertIn('timestamp', event, 'EventEnvelope must have timestamp')
            self.assertIn('payload', event, 'EventEnvelope must have payload')

    # @verifies REQ_INTEROP_032
    # @verifies REQ_INTEROP_097
    def test_12_sse_stream_closes_on_trigger_delete(self):
        """When a trigger is deleted, the SSE stream closes gracefully."""
        trig = self._create_trigger(multishot=True, lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        stream_ended = threading.Event()
        stream_connected = threading.Event()

        def read_stream():
            try:
                with requests.get(events_url, stream=True, timeout=15) as resp:
                    stream_connected.set()
                    for _line in resp.iter_lines(decode_unicode=True):
                        if stream_ended.is_set():
                            break
            except requests.exceptions.RequestException:
                pass
            finally:
                stream_connected.set()
                stream_ended.set()

        thread = threading.Thread(target=read_stream, daemon=True)
        thread.start()

        self.assertTrue(
            stream_connected.wait(timeout=5),
            'SSE stream failed to connect within 5s',
        )

        requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig["id"]}',
            timeout=5,
        )

        stream_ended.wait(timeout=10)
        self.assertTrue(
            stream_ended.is_set(), 'SSE stream did not close after DELETE'
        )

    # ------------------------------------------------------------------
    # Oneshot trigger
    # ------------------------------------------------------------------

    def test_13_oneshot_trigger_terminates_after_first_event(self):
        """A oneshot trigger (multishot=false) terminates after first event.

        Verifies that the trigger status transitions to 'terminated' after
        the condition fires for the first time. The temp_sensor publishes
        data continuously, so OnChange will fire on the first received
        message (no previous value => always fires).
        """
        trig = self._create_trigger(multishot=False, lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        # The trigger should terminate shortly after the first data
        # message arrives (OnChange fires on first eval with no previous).
        # Poll the trigger status until it becomes 'terminated'.
        deadline = time.monotonic() + 15.0
        final_status = 'active'
        while time.monotonic() < deadline:
            try:
                r = requests.get(
                    f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig["id"]}',
                    timeout=5,
                )
                if r.status_code == 200:
                    final_status = r.json().get('status', 'active')
                    if final_status == 'terminated':
                        break
                elif r.status_code == 404:
                    # Trigger was cleaned up after termination
                    final_status = 'terminated'
                    break
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)

        self.assertEqual(
            final_status, 'terminated',
            'Oneshot trigger should become terminated after first event',
        )

    # ------------------------------------------------------------------
    # Complex type trigger (sensor_msgs/LaserScan via lidar)
    # ------------------------------------------------------------------

    def test_14_trigger_on_complex_type(self):
        """Trigger CRUD works on complex message types (LaserScan via lidar)."""
        # Wait for lidar_sensor to appear
        self.poll_endpoint_until(
            '/apps',
            condition=lambda d: d if any(
                a.get('id') == 'lidar_sensor' for a in d.get('items', [])
            ) else None,
            timeout=15.0,
        )

        # Find a non-system data item for lidar_sensor
        system_topics = {'/parameter_events', '/rosout'}
        lidar_data = self.poll_endpoint_until(
            '/apps/lidar_sensor/data',
            condition=lambda d: d if any(
                item['id'] not in system_topics
                for item in d.get('items', [])
            ) else None,
            timeout=15.0,
        )
        topic_id = None
        for item in lidar_data.get('items', []):
            if item['id'] not in system_topics:
                topic_id = item['id']
                break

        if not topic_id:
            self.fail('No data items for lidar_sensor')

        resource = f'/api/v1/apps/lidar_sensor/data{topic_id}'

        # Create trigger on complex type
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/lidar_sensor/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig_id = r.json()['id']
        self.addCleanup(
            lambda: requests.delete(
                f'{self.BASE_URL}/apps/lidar_sensor/triggers/{trig_id}',
                timeout=5,
            )
        )

        self.assertEqual(r.json()['status'], 'active')
        self.assertIn('event_source', r.json())

    # ------------------------------------------------------------------
    # Error handling
    # ------------------------------------------------------------------

    def test_20_create_with_invalid_resource_returns_400(self):
        """POST with malformed resource URI returns 400."""
        body = {
            'resource': '/not/a/valid/resource',
            'trigger_condition': {'condition_type': 'OnChange'},
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_21_create_with_unknown_condition_returns_400(self):
        """POST with unknown condition_type returns 400."""
        body = {
            'resource': self.resource_uri,
            'trigger_condition': {'condition_type': 'UnknownCondition'},
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_22_create_with_missing_condition_returns_400(self):
        """POST without trigger_condition returns 400."""
        body = {
            'resource': self.resource_uri,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_23_get_nonexistent_trigger_returns_404(self):
        """GET for a nonexistent trigger returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/trig_nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_24_delete_nonexistent_trigger_returns_404(self):
        """DELETE for a nonexistent trigger returns 404."""
        r = requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/trig_nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_25_create_with_entity_mismatch_returns_400(self):
        """POST with resource URI for a different entity returns 400."""
        body = {
            'resource': '/api/v1/apps/other_entity/data/something',
            'trigger_condition': {'condition_type': 'OnChange'},
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_26_create_for_nonexistent_entity_returns_404(self):
        """POST on a nonexistent entity returns 404."""
        body = {
            'resource': '/api/v1/apps/nonexistent_entity/data/something',
            'trigger_condition': {'condition_type': 'OnChange'},
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/nonexistent_entity/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_097
    def test_27_events_for_nonexistent_trigger_returns_404(self):
        """GET /events for a non-existent trigger returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers/trig_nonexistent/events',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # ------------------------------------------------------------------
    # Component entity trigger
    # ------------------------------------------------------------------

    def test_30_trigger_on_component_data(self):
        """Triggers work on component entities, not just apps."""
        # Find the powertrain component
        r = requests.get(f'{self.BASE_URL}/components', timeout=5)
        components = r.json().get('items', [])
        if not components:
            self.fail('No components discovered')

        comp_id = components[0]['id']

        # Find data for this component
        r = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/data', timeout=5
        )
        data_items = r.json().get('items', [])
        if not data_items:
            self.fail(f'No data items for component {comp_id}')

        system_topics = {'/parameter_events', '/rosout'}
        topic = None
        for item in data_items:
            if item['id'] not in system_topics:
                topic = item['id']
                break
        if not topic:
            topic = data_items[0]['id']

        resource = f'/api/v1/components/{comp_id}/data{topic}'

        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 30,
        }
        r = requests.post(
            f'{self.BASE_URL}/components/{comp_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig_id = r.json()['id']
        self.addCleanup(
            lambda: requests.delete(
                f'{self.BASE_URL}/components/{comp_id}/triggers/{trig_id}',
                timeout=5,
            )
        )

        # List should include it
        r = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertIn(trig_id, ids)


class TestTriggersDisabled(GatewayTestCase):
    """Trigger routes on a gateway launched with ``triggers.enabled: false``.

    With the trigger manager absent the handlers are never constructed, so the
    registration gate is the only thing between a request and a null
    dereference. Every assertion here therefore does double duty: it checks the
    501 wire shape and it proves the gate actually fires.
    """

    BASE_URL = f'http://127.0.0.1:{PORT_TRIGGERS_OFF}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 0

    APP = 'temp_sensor'

    def assert_not_available(self, resp):
        """Every gated trigger route answers the same SOVD GenericError."""
        self.assertEqual(resp.status_code, 501, resp.text)
        body = resp.json()
        self.assertEqual(body['error_code'], 'not-implemented')
        self.assertEqual(body['message'], 'Triggers not available')

    def test_01_list_triggers_returns_501(self):
        """GET /apps/{id}/triggers is gated off."""
        self.assert_not_available(
            requests.get(f'{self.BASE_URL}/apps/{self.APP}/triggers', timeout=5)
        )

    def test_02_create_trigger_returns_501(self):
        """POST /apps/{id}/triggers is gated off."""
        self.assert_not_available(
            requests.post(
                f'{self.BASE_URL}/apps/{self.APP}/triggers',
                json={
                    'resource': f'{API_BASE_PATH}/apps/{self.APP}/faults',
                    'trigger_condition': {'condition_type': 'OnChange'},
                    'multishot': True,
                },
                timeout=5,
            )
        )

    def test_03_get_trigger_returns_501(self):
        """GET /apps/{id}/triggers/{trigger_id} is gated off."""
        self.assert_not_available(
            requests.get(
                f'{self.BASE_URL}/apps/{self.APP}/triggers/ghost', timeout=5
            )
        )

    def test_04_update_trigger_returns_501(self):
        """PUT /apps/{id}/triggers/{trigger_id} is gated off.

        The body has to be valid, because the gate runs after body parsing -
        an invalid one would be answered with 400 before the gate is reached.
        """
        self.assert_not_available(
            requests.put(
                f'{self.BASE_URL}/apps/{self.APP}/triggers/ghost',
                json={'lifetime': 60},
                timeout=5,
            )
        )

    def test_05_delete_trigger_returns_501(self):
        """DELETE /apps/{id}/triggers/{trigger_id} is gated off."""
        self.assert_not_available(
            requests.delete(
                f'{self.BASE_URL}/apps/{self.APP}/triggers/ghost', timeout=5
            )
        )

    def test_06_trigger_events_stream_returns_501(self):
        """GET /apps/{id}/triggers/{trigger_id}/events (SSE) is gated off.

        The SSE escape hatch has its own wrapper, so the gate has to be wired
        there separately from the typed CRUD wrappers.
        """
        self.assert_not_available(
            requests.get(
                f'{self.BASE_URL}/apps/{self.APP}/triggers/ghost/events',
                timeout=5,
            )
        )

    def test_07_malformed_json_returns_400_not_501(self):
        """A malformed body outranks the feature gate.

        The gate runs inside the typed wrapper, after body parsing. Trigger
        POST goes through a different wrapper than the update routes
        (``wrap_with_body`` rather than ``wrap_with_body_attachments``), so
        the ordering is worth proving on both.
        """
        resp = requests.post(
            f'{self.BASE_URL}/apps/{self.APP}/triggers',
            data='not{valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error_code'], 'invalid-request')

    # @verifies REQ_INTEROP_002
    def test_08_spec_declares_the_gated_status(self):
        """Every documented trigger operation declares the 501 the gate returns."""
        spec = self.get_json('/docs')
        checked = 0
        for path, item in spec['paths'].items():
            if '/triggers' not in path:
                continue
            for method, op in item.items():
                if method not in HTTP_METHODS:
                    continue
                checked += 1
                self.assertIn('501', op['responses'], f'{method.upper()} {path}')
        self.assertGreater(checked, 0, 'No trigger operations in the document')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
