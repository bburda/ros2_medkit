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

"""Integration tests for multi-collection cyclic subscriptions.

Validates that cyclic subscriptions can be created for data, faults,
and configurations collections, and that error cases (unsupported
collection, invalid URI, entity mismatch, unsupported protocol,
path traversal, resource path on a collection that is streamed whole)
return 400.

"""

import json
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    """Launch gateway with demo nodes for multi-collection subscription testing."""
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        gateway_params={
            'sse.max_clients': 10,
            'sse.max_subscriptions': 50,
        },
    )


class TestMultiCollectionSubscriptions(GatewayTestCase):
    """Test cyclic subscriptions across different resource collections."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    app_id = None
    topic_id = None
    data_resource_uri = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway, discover temp sensor, and find a data topic."""
        super().setUpClass()

        # Find the temp_sensor app
        try:
            r = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
            if r.status_code == 200:
                apps = r.json().get('items', [])
                for app in apps:
                    if 'temp_sensor' in app.get('id', '').lower() or \
                       'engine' in app.get('id', '').lower():
                        cls.app_id = app['id']
                        break
        except requests.exceptions.RequestException:
            pass

        if not cls.app_id:
            raise AssertionError('Demo temp sensor app not discovered')

        # Wait for data availability on the discovered app
        # Skip ROS 2 system topics that don't have continuous data flow
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
                            cls.data_resource_uri = (
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

    def _create_subscription(self, resource, interval='normal', duration=60,
                             protocol=None):
        """Create a subscription and return the raw response."""
        body = {
            'resource': resource,
            'interval': interval,
            'duration': duration,
        }
        if protocol is not None:
            body['protocol'] = protocol
        return requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )

    def _delete_subscription(self, sub_id):
        """Delete a subscription, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/apps/{self.app_id}'
                f'/cyclic-subscriptions/{sub_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    # ===================================================================
    # Happy path: data, faults, configurations subscriptions
    # ===================================================================

    def test_data_subscription_create(self):
        """Create a subscription on data collection (regression test).

        @verifies REQ_INTEROP_089
        """
        r = self._create_subscription(self.data_resource_uri)
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        self.assertIn('id', data)
        self.assertTrue(data['id'].startswith('sub_'))
        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'normal')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)
        self.assertTrue(data['event_source'].endswith('/events'))

    def test_faults_subscription_create(self):
        """Create a subscription on faults collection.

        @verifies REQ_INTEROP_089
        """
        resource = f'/api/v1/apps/{self.app_id}/faults'
        r = self._create_subscription(resource, interval='fast')
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'fast')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)

    def test_configurations_subscription_create(self):
        """Create a subscription on configurations collection.

        @verifies REQ_INTEROP_089
        """
        resource = f'/api/v1/apps/{self.app_id}/configurations'
        r = self._create_subscription(resource, interval='slow')
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'slow')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)

    def test_logs_subscription_create(self):
        """Create a subscription on logs collection.

        @verifies REQ_INTEROP_089
        """
        resource = f'/api/v1/apps/{self.app_id}/logs'
        r = self._create_subscription(resource, interval='slow')
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'slow')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)

    def test_logs_subscription_sse_stream(self):
        """SSE stream on logs collection delivers EventEnvelope with log items.

        Creates a cyclic subscription on /apps/{id}/logs, connects to the SSE
        events endpoint, and verifies that EventEnvelope payloads arrive with
        log entry items containing id, timestamp, severity, and message fields.

        @verifies REQ_INTEROP_089
        @verifies REQ_INTEROP_090
        """
        resource = f'/api/v1/apps/{self.app_id}/logs'
        r = self._create_subscription(resource, interval='slow', duration=30)
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        events_url = (
            f'{self.BASE_URL}{data["event_source"].removeprefix(API_BASE_PATH)}'
        )

        received_events = []
        stop_event = threading.Event()

        def collect_events():
            try:
                with requests.get(events_url, stream=True, timeout=15) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop_event.is_set():
                            break
                        if line and line.startswith('data: '):
                            event = json.loads(line[6:])
                            received_events.append(event)
                            if len(received_events) >= 2:
                                stop_event.set()
                                break
            except requests.exceptions.RequestException:
                pass

        thread = threading.Thread(target=collect_events, daemon=True)
        thread.start()

        # slow interval = 500ms, so 2 events should arrive within ~5s
        stop_event.wait(timeout=15)
        thread.join(timeout=5)

        self.assertGreaterEqual(
            len(received_events), 1,
            f'Expected at least 1 SSE log event, got {len(received_events)}',
        )

        for event in received_events:
            self.assertIn('timestamp', event, 'EventEnvelope must have timestamp')
            self.assertIn('payload', event, 'EventEnvelope must have payload')
            payload = event['payload']
            self.assertIn('items', payload, 'Logs payload must have items array')
            # Log entries from /rosout should be present after demo node startup
            if payload['items']:
                entry = payload['items'][0]
                self.assertIn('id', entry)
                self.assertIn('timestamp', entry)
                self.assertIn('severity', entry)
                self.assertIn('message', entry)

    def _collect_sse_events(self, event_source, wanted=2, timeout=15):
        """Open the SSE stream and return up to `wanted` parsed events."""
        events_url = (
            f'{self.BASE_URL}{event_source.removeprefix(API_BASE_PATH)}'
        )
        received = []
        stop_event = threading.Event()

        def collect():
            try:
                with requests.get(
                    events_url, stream=True, timeout=timeout,
                ) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop_event.is_set():
                            break
                        if line and line.startswith('data: '):
                            received.append(json.loads(line[6:]))
                            if len(received) >= wanted:
                                stop_event.set()
                                break
            except requests.exceptions.RequestException:
                pass

        thread = threading.Thread(target=collect, daemon=True)
        thread.start()
        stop_event.wait(timeout=timeout)
        thread.join(timeout=5)
        return received

    def test_configurations_subscription_streams_whole_collection(self):
        """Configurations without a resource path is accepted and streams.

        The gateway refuses a configurations URI that names a single
        parameter because the sampler answers with the whole collection.
        This pins the other side of that contract: the collection URI is
        still accepted and still delivers parameter items on the stream.

        @verifies REQ_INTEROP_089
        @verifies REQ_INTEROP_090
        """
        resource = f'/api/v1/apps/{self.app_id}/configurations'
        r = self._create_subscription(resource, interval='fast', duration=30)
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        events = self._collect_sse_events(data['event_source'])
        self.assertGreaterEqual(
            len(events), 1,
            f'Expected at least 1 SSE configurations event, got {len(events)}',
        )
        for event in events:
            self.assertIn('payload', event)
            self.assertIn(
                'items', event['payload'],
                'Configurations payload must have items array',
            )

    # ===================================================================
    # CRUD operations: list, get, update, delete
    # ===================================================================

    def test_list_subscriptions(self):
        """List subscriptions returns items array.

        @verifies REQ_INTEROP_025
        """
        # Create a subscription first
        r = self._create_subscription(self.data_resource_uri, duration=60)
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']
        self.addCleanup(self._delete_subscription, sub_id)

        # List
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIn('items', data)
        ids = [s['id'] for s in data['items']]
        self.assertIn(sub_id, ids)

    def test_get_subscription(self):
        """Get single subscription returns SOVD fields.

        @verifies REQ_INTEROP_026
        """
        r = self._create_subscription(self.data_resource_uri, duration=60)
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']
        self.addCleanup(self._delete_subscription, sub_id)

        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data['id'], sub_id)
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)
        self.assertIn('protocol', data)
        self.assertIn('interval', data)

    def test_update_subscription_interval(self):
        """Update subscription interval.

        @verifies REQ_INTEROP_027
        """
        r = self._create_subscription(
            self.data_resource_uri, interval='normal', duration=60,
        )
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']
        self.addCleanup(self._delete_subscription, sub_id)

        r = requests.put(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            json={'interval': 'fast'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()['interval'], 'fast')

    def test_delete_subscription(self):
        """Delete subscription returns 204.

        @verifies REQ_INTEROP_028
        """
        r = self._create_subscription(self.data_resource_uri, duration=60)
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']

        r = requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify it's gone
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_get_nonexistent_subscription_returns_404(self):
        """Get nonexistent subscription returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/nonexistent_sub',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # ===================================================================
    # Error cases
    # ===================================================================

    def test_unsupported_collection_returns_400(self):
        """Subscription on unknown collection returns 400."""
        resource = f'/api/v1/apps/{self.app_id}/unknown_collection'
        r = self._create_subscription(resource)
        self.assertEqual(r.status_code, 400)

    def test_invalid_resource_uri_returns_400(self):
        """Invalid resource URI format returns 400."""
        r = self._create_subscription('/invalid/uri')
        self.assertEqual(r.status_code, 400)

    def test_entity_mismatch_returns_400(self):
        """Resource URI referencing different entity returns 400."""
        resource = '/api/v1/apps/OTHER_nonexistent_entity/data/topic'
        r = self._create_subscription(resource)
        self.assertEqual(r.status_code, 400)

    def test_unsupported_protocol_returns_400(self):
        """Unsupported protocol returns 400."""
        r = self._create_subscription(
            self.data_resource_uri, protocol='mqtt',
        )
        self.assertEqual(r.status_code, 400)

    def test_path_traversal_returns_400(self):
        """Resource path with '..' returns 400."""
        resource = f'/api/v1/apps/{self.app_id}/data/../../../etc/passwd'
        r = self._create_subscription(resource)
        self.assertEqual(r.status_code, 400)

    def _assert_resource_path_refused(self, collection, resource_path):
        """Assert a resource path on a whole-collection stream is refused."""
        resource = (
            f'/api/v1/apps/{self.app_id}/{collection}/{resource_path}'
        )
        r = self._create_subscription(resource)
        self.assertEqual(
            r.status_code, 400,
            f'Expected 400 for {resource}, got {r.status_code}: {r.text}',
        )
        body = r.json()
        self.assertEqual(body['error_code'], 'vendor-error')
        self.assertEqual(
            body['vendor_code'], 'x-medkit-invalid-resource-uri',
        )
        self.assertIn(
            collection, body['message'],
            'Refusal must name the collection it applies to',
        )
        self.assertIn(
            'streamed as a whole', body['message'],
            'Refusal must say the collection is streamed whole',
        )
        self.assertEqual(body['parameters']['collection'], collection)

    def test_configurations_with_resource_path_returns_400(self):
        """A single-parameter configurations URI is refused, not widened."""
        self._assert_resource_path_refused('configurations', 'use_sim_time')

    def test_faults_with_resource_path_returns_400(self):
        """A single-fault URI is refused, not widened to the fault list."""
        self._assert_resource_path_refused('faults', 'fault_001')

    def test_logs_with_resource_path_returns_400(self):
        """A single-log-entry URI is refused, not widened to the log list."""
        self._assert_resource_path_refused('logs', 'entry_7')

    def test_data_without_resource_path_returns_400(self):
        """Data without a topic is still refused - the sampler needs one."""
        resource = f'/api/v1/apps/{self.app_id}/data'
        r = self._create_subscription(resource)
        self.assertEqual(r.status_code, 400, f'Got {r.status_code}: {r.text}')
        body = r.json()
        self.assertEqual(body['error_code'], 'vendor-error')
        self.assertEqual(
            body['vendor_code'], 'x-medkit-invalid-resource-uri',
        )
        self.assertIn('requires a resource path', body['message'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
