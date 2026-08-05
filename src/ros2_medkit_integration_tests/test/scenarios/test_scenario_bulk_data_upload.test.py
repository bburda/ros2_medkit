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

"""Scenario: Bulk data upload — full CRUD lifecycle for file uploads.

Launches the gateway with bulk_data categories configured (calibration,
firmware) on a unique port, plus a single demo node for entity targets.
Tests upload (POST), list, download, delete, and error handling.

"""

import json
import tempfile
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH, get_test_port
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node

TEST_PORT = get_test_port()
BULK_DATA_BASE_URL = f'http://localhost:{TEST_PORT}{API_BASE_PATH}'


def generate_test_description():
    """Generate launch description with gateway configured for bulk data."""
    tmpdir = tempfile.mkdtemp(prefix='bulk_data_test_')

    gateway_node = create_gateway_node(
        port=TEST_PORT,
        extra_params={
            'server.host': '127.0.0.1',
            'bulk_data.storage_dir': tmpdir,
            'bulk_data.max_upload_size': 104857600,
            'bulk_data.categories': ['calibration', 'firmware'],
        },
    )

    # A single demo node so we have at least one app entity to test with
    demo_nodes = create_demo_nodes(['temp_sensor'])

    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway_node,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


class TestScenarioBulkDataUpload(GatewayTestCase):
    """Scenario: Bulk data upload/delete REST API — full CRUD lifecycle.

    A diagnostic technician uploads calibration and firmware files to
    specific entities, verifies they appear in listings, downloads them,
    and cleans up by deleting.

    Steps:
    1. Upload small file — verify 201 with descriptor
    2. Upload returns Location header
    3. Upload with description field
    4. Upload with metadata JSON field
    5. Upload missing file field — 400
    6. Upload to unknown category — 400
    7. Upload rosbags category rejected — 400
    8. Upload to nonexistent entity — 404
    9. Upload to areas — 405
    10. Upload to functions — 405
    11. Delete uploaded item — 204
    12. Delete nonexistent item — 404
    13. Delete rosbags category — 400
    14. Delete on areas — 405
    15. List categories includes configured ones
    16. List descriptors after upload
    17. Download uploaded file content
    18. Download nonexistent returns 404
    19. List empty after delete
    20. Full CRUD cycle
    """

    BASE_URL = BULK_DATA_BASE_URL
    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    test_app_id = 'temp_sensor'
    test_component_id = None
    test_area_id = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway and discover temp_sensor, a component, and an area."""
        super().setUpClass()

        # Find a component
        try:
            cr = requests.get(f'{cls.BASE_URL}/components', timeout=5)
            if cr.status_code == 200:
                comps = cr.json().get('items', [])
                if comps:
                    cls.test_component_id = comps[0].get('id', '')
        except requests.exceptions.RequestException:
            pass

        # Find an area
        try:
            ar = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
            if ar.status_code == 200:
                areas = ar.json().get('items', [])
                if areas:
                    cls.test_area_id = areas[0].get('id', '')
        except requests.exceptions.RequestException:
            pass

    # === Upload (POST) tests ===

    def test_01_upload_small_file(self):
        """POST multipart with small file returns 201 with descriptor.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('test.bin', b'x' * 100, 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 201)
        data = r.json()
        self.assertIn('id', data)
        self.assertEqual(data['name'], 'test.bin')
        self.assertEqual(data['mimetype'], 'application/octet-stream')
        self.assertEqual(data['size'], 100)
        self.assertIn('creation_date', data)

    def test_02_upload_returns_location_header(self):
        """201 response has Location header pointing to uploaded resource.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('loc.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 201)
        location = r.headers.get('Location', '')
        prefix = (
            f'/api/v1/apps/{self.test_app_id}/bulk-data/calibration/'
        )
        self.assertTrue(location.startswith(prefix))
        # Location should end with the item ID
        item_id = r.json()['id']
        self.assertTrue(location.endswith(item_id))

    def test_03_upload_with_description(self):
        """POST with description field includes it in descriptor.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('desc.bin', b'data', 'application/octet-stream')}
        data = {'description': 'Test calibration data'}
        r = requests.post(url, files=files, data=data, timeout=10)
        self.assertEqual(r.status_code, 201)
        self.assertEqual(r.json()['description'], 'Test calibration data')

    def test_04_upload_with_metadata(self):
        """POST with metadata JSON field includes x-medkit in descriptor.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('meta.bin', b'data', 'application/octet-stream')}
        data = {'metadata': json.dumps({'sensor': 'lidar', 'version': 2})}
        r = requests.post(url, files=files, data=data, timeout=10)
        self.assertEqual(r.status_code, 201)
        resp = r.json()
        self.assertIn('x-medkit', resp)
        self.assertEqual(resp['x-medkit']['sensor'], 'lidar')

    def test_05_upload_missing_file_field(self):
        """POST without file field returns 400.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        # Send empty multipart with only description, no file
        data = {'description': 'no file'}
        r = requests.post(url, data=data, timeout=10)
        self.assertEqual(r.status_code, 400)

    def test_06_upload_unknown_category(self):
        """POST to unknown category returns 400.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/bogus'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 400)

    def test_07_upload_rosbags_rejected(self):
        """POST to rosbags category returns 400.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/rosbags'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 400)
        self.assertIn('fault system', r.json().get('message', ''))

    def test_08_upload_nonexistent_entity(self):
        """POST to nonexistent entity returns 404.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/fake_entity_12345/bulk-data/calibration'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 404)

    def test_09_upload_to_areas_405(self):
        """POST to areas returns 405.

        @verifies REQ_INTEROP_074
        """
        area_id = getattr(self, 'test_area_id', 'powertrain')
        url = f'{self.BASE_URL}/areas/{area_id}/bulk-data/calibration'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 405)

    def test_10_upload_to_functions_405(self):
        """POST to functions returns 405.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/functions/some_func/bulk-data/calibration'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 405)

    # === Delete (DELETE) tests ===

    def test_11_delete_uploaded_item(self):
        """Upload then DELETE returns 204.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('del.bin', b'delete_me', 'application/octet-stream')}
        upload_r = requests.post(url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        del_url = f'{url}/{item_id}'
        del_r = requests.delete(del_url, timeout=10)
        self.assertEqual(del_r.status_code, 204)

    def test_12_delete_nonexistent_item(self):
        """DELETE nonexistent ID returns 404.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration/nonexistent_id'
        r = requests.delete(url, timeout=10)
        self.assertEqual(r.status_code, 404)

    def test_13_delete_rosbags_rejected(self):
        """DELETE rosbags category returns 400.

        @verifies REQ_INTEROP_074
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/rosbags/some_id'
        r = requests.delete(url, timeout=10)
        self.assertEqual(r.status_code, 400)

    def test_14_delete_to_areas_405(self):
        """DELETE on areas returns 405.

        @verifies REQ_INTEROP_074
        """
        area_id = getattr(self, 'test_area_id', 'powertrain')
        url = f'{self.BASE_URL}/areas/{area_id}/bulk-data/calibration/some_id'
        r = requests.delete(url, timeout=10)
        self.assertEqual(r.status_code, 405)

    # === List / Download (GET — extended) tests ===

    def test_15_list_categories_includes_configured(self):
        """GET /bulk-data returns rosbags + configured categories.

        @verifies REQ_INTEROP_071
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data'
        r = requests.get(url, timeout=10)
        self.assertEqual(r.status_code, 200)
        items = r.json().get('items', [])
        self.assertIn('rosbags', items)
        self.assertIn('calibration', items)
        self.assertIn('firmware', items)

    def test_16_list_descriptors_after_upload(self):
        """Upload 2 files, list shows 2 items.

        @verifies REQ_INTEROP_072
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/firmware'
        for name in ['fw1.bin', 'fw2.bin']:
            files = {'file': (name, b'firmware_data', 'application/octet-stream')}
            r = requests.post(url, files=files, timeout=10)
            self.assertEqual(r.status_code, 201)

        r = requests.get(url, timeout=10)
        self.assertEqual(r.status_code, 200)
        items = r.json().get('items', [])
        self.assertGreaterEqual(len(items), 2)

    def test_17_download_uploaded_file(self):
        """Upload 'hello' then download returns same content.

        @verifies REQ_INTEROP_073
        """
        base_url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('hello.txt', b'hello', 'text/plain')}
        upload_r = requests.post(base_url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        dl_url = f'{base_url}/{item_id}'
        dl_r = requests.get(dl_url, timeout=10)
        self.assertEqual(dl_r.status_code, 200)
        self.assertEqual(dl_r.content, b'hello')
        # Check Content-Disposition
        cd = dl_r.headers.get('Content-Disposition', '')
        self.assertIn('hello.txt', cd)
        # The download is served through a range-aware content provider, so it
        # advertises the unit it accepts (RFC 9110 14.3). cpp-httplib only fills
        # this in for HEAD, so a missing value means the framework stopped
        # setting it and the OpenAPI document now advertises a header we drop.
        self.assertEqual(dl_r.headers.get('Accept-Ranges'), 'bytes')

    def test_18_download_nonexistent_returns_404(self):
        """GET download with fake ID returns 404.

        @verifies REQ_INTEROP_073
        """
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration/nonexistent_id'
        r = requests.get(url, timeout=10)
        self.assertEqual(r.status_code, 404)

    def test_19_list_empty_after_delete(self):
        """Upload, delete, then list shows no items from this upload.

        @verifies REQ_INTEROP_074
        """
        # Use firmware category to avoid interference from other tests
        base_url = f'{self.BASE_URL}/components/{self.test_component_id}/bulk-data/firmware'
        files = {'file': ('gone.bin', b'data', 'application/octet-stream')}
        upload_r = requests.post(base_url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        # Delete
        del_r = requests.delete(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(del_r.status_code, 204)

        # Check item is gone
        dl_r = requests.get(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(dl_r.status_code, 404)

    # === Full CRUD flow ===

    def test_20_full_crud_cycle(self):
        """Upload -> List (present) -> Download (content) -> Delete -> List (absent).

        @verifies REQ_INTEROP_071
        @verifies REQ_INTEROP_074
        """
        base_url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        payload = b'full_crud_test_data'
        files = {'file': ('crud.bin', payload, 'application/octet-stream')}

        # Upload
        upload_r = requests.post(base_url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        # List — verify present
        list_r = requests.get(base_url, timeout=10)
        self.assertEqual(list_r.status_code, 200)
        item_ids = [i['id'] for i in list_r.json().get('items', [])]
        self.assertIn(item_id, item_ids)

        # Download — verify content
        dl_r = requests.get(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(dl_r.status_code, 200)
        self.assertEqual(dl_r.content, payload)

        # Delete
        del_r = requests.delete(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(del_r.status_code, 204)

        # List — verify absent
        list_r2 = requests.get(base_url, timeout=10)
        self.assertEqual(list_r2.status_code, 200)
        item_ids2 = [i['id'] for i in list_r2.json().get('items', [])]
        self.assertNotIn(item_id, item_ids2)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
