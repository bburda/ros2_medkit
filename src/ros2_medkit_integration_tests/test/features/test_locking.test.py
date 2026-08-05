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

"""Integration tests for SOVD resource locking (issue #205).

Tests cover:
- Lock lifecycle (acquire, list, get, extend, release) on apps
- Lock enforcement (mutating operations blocked by other client's lock)
- Scoped locks (only locked collections are blocked)
- Lock breaking
- Error cases (missing header, invalid body)
"""

import time
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    """Launch gateway with locking enabled and short cleanup interval."""
    return create_test_launch(
        demo_nodes=['temp_sensor', 'calibration'],
        gateway_params={
            'locking.enabled': True,
            'locking.default_max_expiration': 3600,
            'locking.cleanup_interval': 1,
        },
        fault_manager=False,
    )


class TestLocking(GatewayTestCase):
    """SOVD resource locking integration tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def _post_lock(self, entity_type, entity_id, client_id, body):
        """Acquire a lock via POST and return the response."""
        return requests.post(
            f'{self.BASE_URL}/{entity_type}/{entity_id}/locks',
            json=body,
            headers={'X-Client-Id': client_id},
            timeout=10,
        )

    def _delete_lock(self, entity_type, entity_id, lock_id, client_id):
        """Release a lock via DELETE and return the response."""
        return requests.delete(
            f'{self.BASE_URL}/{entity_type}/{entity_id}/locks/{lock_id}',
            headers={'X-Client-Id': client_id},
            timeout=10,
        )

    def _get_locks(self, entity_type, entity_id, client_id=None):
        """List locks via GET and return the response."""
        headers = {}
        if client_id:
            headers['X-Client-Id'] = client_id
        return requests.get(
            f'{self.BASE_URL}/{entity_type}/{entity_id}/locks',
            headers=headers,
            timeout=10,
        )

    def _cleanup_lock(self, entity_type, entity_id, client_id='__cleanup__'):
        """Force-acquire then release to clean up any existing lock."""
        resp = self._post_lock(
            entity_type, entity_id, client_id,
            {'lock_expiration': 1, 'break_lock': True},
        )
        if resp.status_code == 201:
            lock_id = resp.json()['id']
            self._delete_lock(entity_type, entity_id, lock_id, client_id)

    def setUp(self):
        super().setUp()
        self._cleanup_lock('apps', 'temp_sensor')

    # =================================================================
    # Lock lifecycle
    # =================================================================

    # @verifies REQ_INTEROP_100
    def test_acquire_lock_on_app(self):
        """Acquire a lock on an app and verify the response."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        data = resp.json()
        self.assertIn('id', data)
        self.assertTrue(data['owned'])
        self.assertIn('lock_expiration', data)
        # ISO 8601 datetime format
        self.assertIn('T', data['lock_expiration'])
        self.assertIn('Z', data['lock_expiration'])

        # The 201 must name the lock it created, and that URI has to resolve.
        self.assertEqual(
            resp.headers.get('Location'),
            f'{API_BASE_PATH}/apps/temp_sensor/locks/{data["id"]}')
        follow = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/locks/{data["id"]}',
            headers={'X-Client-Id': 'client_a'}, timeout=10)
        self.assertEqual(follow.status_code, 200, follow.text)

    # @verifies REQ_INTEROP_100
    def test_acquire_lock_location_survives_a_trailing_slash(self):
        """The 201 `Location` must resolve even when the caller adds a slash.

        Route regexes end `/?$`, so `POST .../locks/` routes and the path is
        not normalised. Appending the lock id to it produced
        `.../locks//lock_1`, which the item route cannot match - `([^/]+)`
        refuses an empty segment - so the header handed back a dead URI. This
        is on the shipped default: `locking.enabled` is true out of the box.
        """
        resp = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/locks/',
            json={'lock_expiration': 300},
            headers={'X-Client-Id': 'client_slash'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']
        self.addCleanup(
            self._delete_lock, 'apps', 'temp_sensor', lock_id, 'client_slash')

        location = resp.headers.get('Location')
        self.assertEqual(location, f'{API_BASE_PATH}/apps/temp_sensor/locks/{lock_id}')

        follow = requests.get(
            self.BASE_URL + location[len(API_BASE_PATH):],
            headers={'X-Client-Id': 'client_slash'}, timeout=10)
        self.assertEqual(follow.status_code, 200, follow.text)

    # @verifies REQ_INTEROP_100
    def test_acquire_lock_with_scopes(self):
        """Acquire a scoped lock."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300, 'scopes': ['data', 'configurations']},
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        data = resp.json()
        self.assertIn('scopes', data)
        self.assertEqual(len(data['scopes']), 2)

    # @verifies REQ_INTEROP_101
    def test_list_locks(self):
        """Acquire then list locks."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300, 'scopes': ['configurations']},
        )
        self.assertEqual(resp.status_code, 201, resp.text)

        resp = self._get_locks('apps', 'temp_sensor', 'client_a')
        self.assertEqual(resp.status_code, 200, resp.text)
        data = resp.json()
        self.assertIn('items', data)
        self.assertEqual(len(data['items']), 1)
        self.assertTrue(data['items'][0]['owned'])
        self.assertEqual(data['items'][0]['scopes'], ['configurations'])

    # @verifies REQ_INTEROP_101
    def test_list_locks_empty(self):
        """List locks when none exist returns empty items."""
        resp = self._get_locks('apps', 'temp_sensor')
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(len(resp.json()['items']), 0)

    # @verifies REQ_INTEROP_102
    def test_get_lock_details(self):
        """Acquire then get lock details by lock ID."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']

        resp = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/locks/{lock_id}',
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(resp.json()['id'], lock_id)
        self.assertTrue(resp.json()['owned'])

    # @verifies REQ_INTEROP_103
    def test_extend_lock(self):
        """Acquire then extend a lock's expiration."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']

        resp = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/locks/{lock_id}',
            json={'lock_expiration': 600},
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 204, resp.text)

    # @verifies REQ_INTEROP_104
    def test_release_lock(self):
        """Acquire then release a lock, verify it's gone."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']

        resp = self._delete_lock('apps', 'temp_sensor', lock_id, 'client_a')
        self.assertEqual(resp.status_code, 204, resp.text)

        resp = self._get_locks('apps', 'temp_sensor')
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(len(resp.json()['items']), 0)

    # =================================================================
    # Lock enforcement
    # =================================================================

    # @verifies REQ_INTEROP_100
    def test_lock_blocks_other_client(self):
        """Lock by client A blocks data write by client B (409)."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)

        resp = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/data/engine_temperature',
            json={'type': 'std_msgs/msg/Float64', 'data': {'data': 99.0}},
            headers={'X-Client-Id': 'client_b'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 409, resp.text)

    # @verifies REQ_INTEROP_100
    def test_scoped_lock_allows_other_collections(self):
        """Scoped lock on configurations does not block data writes."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300, 'scopes': ['configurations']},
        )
        self.assertEqual(resp.status_code, 201, resp.text)

        # Data write should NOT be blocked (configurations-only lock)
        resp = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/data/engine_temperature',
            json={'type': 'std_msgs/msg/Float64', 'data': {'data': 99.0}},
            headers={'X-Client-Id': 'client_b'},
            timeout=10,
        )
        self.assertNotEqual(resp.status_code, 409, resp.text)

    # =================================================================
    # Lock breaking
    # =================================================================

    # @verifies REQ_INTEROP_100
    def test_break_lock(self):
        """Client B can break client A's lock with break_lock=true."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)

        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_b',
            {'lock_expiration': 600, 'break_lock': True},
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        self.assertTrue(resp.json()['owned'])

    # =================================================================
    # Owned field per client
    # =================================================================

    # @verifies REQ_INTEROP_101
    def test_owned_field_per_client(self):
        """The 'owned' field reflects the requesting client."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)

        # Owner sees owned=true
        resp = self._get_locks('apps', 'temp_sensor', 'client_a')
        self.assertTrue(resp.json()['items'][0]['owned'])

        # Other client sees owned=false
        resp = self._get_locks('apps', 'temp_sensor', 'client_b')
        self.assertFalse(resp.json()['items'][0]['owned'])

        # No client header sees owned=false
        resp = self._get_locks('apps', 'temp_sensor')
        self.assertFalse(resp.json()['items'][0]['owned'])

    # =================================================================
    # Error cases
    # =================================================================

    # @verifies REQ_INTEROP_100
    def test_acquire_without_client_id_returns_400(self):
        """Acquire without X-Client-Id header returns 400."""
        resp = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/locks',
            json={'lock_expiration': 300},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 400, resp.text)

    # @verifies REQ_INTEROP_100
    def test_acquire_without_expiration_returns_400(self):
        """Acquire without lock_expiration returns 400."""
        resp = self._post_lock('apps', 'temp_sensor', 'client_a', {})
        self.assertEqual(resp.status_code, 400, resp.text)

    # @verifies REQ_INTEROP_100
    def test_already_locked_returns_409(self):
        """Second acquire by different client returns 409."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)

        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_b',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 409, resp.text)

    # @verifies REQ_INTEROP_104
    def test_release_not_owner_returns_403(self):
        """Releasing another client's lock returns 403."""
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']

        resp = self._delete_lock('apps', 'temp_sensor', lock_id, 'client_b')
        self.assertEqual(resp.status_code, 403, resp.text)

    # =================================================================
    # Lock expiration
    # =================================================================

    # @verifies REQ_INTEROP_100
    def test_lock_expires_automatically(self):
        """Lock with short TTL expires and no longer blocks access."""
        # Acquire with 2s TTL
        resp = self._post_lock(
            'apps', 'temp_sensor', 'client_a',
            {'lock_expiration': 2},
        )
        self.assertEqual(resp.status_code, 201, resp.text)

        # Immediately: lock should exist
        resp = self._get_locks('apps', 'temp_sensor')
        self.assertEqual(len(resp.json()['items']), 1)

        # Wait for expiry + cleanup (TTL=2s, cleanup_interval=1s)
        time.sleep(4)

        # Lock should be gone
        resp = self._get_locks('apps', 'temp_sensor')
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(len(resp.json()['items']), 0)

    # =================================================================
    # Error cases
    # =================================================================

    # @verifies REQ_INTEROP_100
    def test_nonexistent_entity_returns_404(self):
        """Acquiring lock on nonexistent entity returns 404."""
        resp = self._post_lock(
            'apps', 'does_not_exist', 'client_a',
            {'lock_expiration': 300},
        )
        self.assertEqual(resp.status_code, 404, resp.text)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
