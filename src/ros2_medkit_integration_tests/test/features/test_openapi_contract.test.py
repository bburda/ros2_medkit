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

"""Contract tests for the generated OpenAPI document.

Document-wide invariants that no single handler owns. Every rule here is a
rule a generated client depends on, so a violation is a client-visible defect
even when each individual endpoint behaves correctly.

The gateway is launched with every optional feature gate on, so the maximal
route surface is present and assertions about gated routes cannot pass
vacuously.
"""

import json
import re
import tempfile
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_test_launch,
    full_feature_gateway_params,
)

HTTP_METHODS = {'get', 'post', 'put', 'delete', 'patch', 'head', 'options'}

_SCRIPTS_DIR = tempfile.mkdtemp(prefix='medkit-contract-scripts-')

PYTHON_SCRIPT = '#!/usr/bin/env python3\nimport json\nprint(json.dumps({"result": "ok"}))\n'


def generate_test_description():
    return create_test_launch(
        demo_nodes=['calibration', 'temp_sensor'],
        fault_manager=True,
        gateway_params=full_feature_gateway_params(_SCRIPTS_DIR),
    )


def refs_in(node):
    """Return every #/components/... reference reachable inside a subtree."""
    return set(re.findall(r'#/components/[A-Za-z0-9_/]+', json.dumps(node)))


class TestOpenApiContract(GatewayTestCase):
    """Document-wide invariants of GET /api/v1/docs."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'calibration', 'temp_sensor'}

    _spec = None

    def spec(self):
        """Fetch and cache the served OpenAPI document."""
        if TestOpenApiContract._spec is None:
            TestOpenApiContract._spec = self.poll_endpoint_until(
                '/docs',
                lambda d: d if 'openapi' in d else None,
            )
        return TestOpenApiContract._spec

    def operations(self):
        """Yield (path, method, operation) for every documented operation."""
        for path, item in self.spec()['paths'].items():
            for method, operation in item.items():
                if method in HTTP_METHODS:
                    yield path, method, operation

    def test_document_is_openapi_31(self):
        """Served document is OpenAPI 3.1.0 and carries the SOVD version.

        @verifies REQ_INTEROP_002
        """
        spec = self.spec()
        self.assertEqual(spec['openapi'], '3.1.0')
        self.assertEqual(spec['info']['x-sovd-version'], '1.0.0')

    def test_every_operation_is_identified(self):
        """Every operation has a unique operationId, a summary and a tag."""
        seen = {}
        for path, method, op in self.operations():
            where = f'{method.upper()} {path}'
            self.assertTrue(op.get('summary'), f'{where}: missing summary')
            self.assertTrue(op.get('tags'), f'{where}: missing tag')
            op_id = op.get('operationId')
            self.assertTrue(op_id, f'{where}: missing operationId')
            self.assertNotIn(
                op_id, seen, f'{where}: operationId collides with {seen.get(op_id)}')
            seen[op_id] = where

    def test_every_tag_used_is_declared(self):
        """No operation carries a tag missing from the document tag list."""
        declared = {t['name'] for t in self.spec().get('tags', [])}
        for path, method, op in self.operations():
            for tag in op.get('tags', []):
                self.assertIn(
                    tag, declared, f'{method.upper()} {path}: tag "{tag}" not declared')

    def test_no_malformed_path_keys(self):
        """Path keys have a leading slash and no empty segments."""
        for path in self.spec()['paths']:
            self.assertTrue(path.startswith('/'), f'{path}: missing leading slash')
            self.assertNotIn('//', path, f'{path}: empty path segment')

    def declared_success_status(self, path, method):
        """Return the single 2xx status the document declares for an operation."""
        op = self.spec()['paths'][path][method]
        codes = sorted(c for c in op.get('responses', {}) if c.startswith('2'))
        self.assertEqual(
            len(codes), 1,
            f'{method.upper()} {path}: expected exactly one declared 2xx, got {codes}')
        return int(codes[0])

    def test_trigger_create_answers_with_the_declared_status(self):
        """POST /apps/{app_id}/triggers answers with its declared 2xx."""
        declared = self.declared_success_status('/apps/{app_id}/triggers', 'post')
        resp = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/triggers',
            json={
                'resource': '/api/v1/apps/temp_sensor/faults',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
            },
            timeout=10,
        )
        self.assertEqual(resp.status_code, declared, resp.text)
        self.addCleanup(
            requests.delete,
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{resp.json()["id"]}',
            timeout=10,
        )

    def test_lock_acquire_answers_with_the_declared_status(self):
        """POST /apps/{app_id}/locks answers with its declared 2xx."""
        declared = self.declared_success_status('/apps/{app_id}/locks', 'post')
        resp = requests.post(
            f'{self.BASE_URL}/apps/calibration/locks',
            json={'lock_expiration': 60},
            headers={'X-Client-Id': 'contract_client'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, declared, resp.text)
        self.addCleanup(
            requests.delete,
            f'{self.BASE_URL}/apps/calibration/locks/{resp.json()["id"]}',
            headers={'X-Client-Id': 'contract_client'},
            timeout=10,
        )

    def test_subscription_create_answers_with_the_declared_status(self):
        """POST /apps/{app_id}/cyclic-subscriptions answers with its declared 2xx."""
        declared = self.declared_success_status(
            '/apps/{app_id}/cyclic-subscriptions', 'post')
        resp = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/cyclic-subscriptions',
            json={
                'resource': '/api/v1/apps/temp_sensor/faults',
                'interval': 'normal',
                'duration': 60,
            },
            timeout=10,
        )
        self.assertEqual(resp.status_code, declared, resp.text)
        self.addCleanup(
            requests.delete,
            f'{self.BASE_URL}/apps/temp_sensor/cyclic-subscriptions/'
            f'{resp.json()["id"]}',
            timeout=10,
        )

    def test_script_routes_answer_with_the_declared_status(self):
        """Script upload (201) and execution start (202) match the document.

        The execution POST is the only converted route in this fixture whose
        declared success status is 202, so it is what proves an Accepted<T>
        return type reaches the wire as 202 rather than 200.
        """
        upload_declared = self.declared_success_status('/apps/{app_id}/scripts', 'post')
        upload = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts',
            files={'file': ('contract.py', PYTHON_SCRIPT, 'application/octet-stream')},
            timeout=10,
        )
        self.assertEqual(upload.status_code, upload_declared, upload.text)
        script_id = upload.json()['id']
        self.addCleanup(
            requests.delete,
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}',
            timeout=10,
        )

        exec_declared = self.declared_success_status(
            '/apps/{app_id}/scripts/{script_id}/executions', 'post')
        self.assertEqual(exec_declared, 202)
        start = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/scripts/{script_id}/executions',
            json={'execution_type': 'now'},
            timeout=10,
        )
        self.assertEqual(start.status_code, exec_declared, start.text)

    def test_every_success_response_is_described(self):
        """No declared 2xx ships without prose a client can show."""
        undescribed = []
        for path, method, op in self.operations():
            for code, resp in op.get('responses', {}).items():
                if code.startswith('2') and not resp.get('description'):
                    undescribed.append(f'{op.get("operationId")}: {code}')
        self.assertEqual(undescribed, [], f'no description: {undescribed}')

    def test_no_success_response_is_described_as_no_content(self):
        """Only 204 may be called "No content".

        The description is auto-filled from the declared status, so a 201 or
        202 labelled "No content" means the fill lost track of the status - the
        same drift between status and document this suite exists to catch, one
        layer down in the prose.
        """
        mislabelled = []
        checked = 0
        for path, method, op in self.operations():
            for code, resp in op.get('responses', {}).items():
                if not code.startswith('2') or code == '204':
                    continue
                checked += 1
                if 'no content' in (resp.get('description') or '').lower():
                    mislabelled.append(
                        f'{op.get("operationId")}: {code} = {resp["description"]!r}')
        # The body-less 202 routes (lifecycle transitions, update prepare /
        # execute / automated) are the ones that used to be labelled "No
        # content"; if the fixture stops exposing them this test would pass
        # while guarding nothing.
        self.assertGreater(checked, 0, 'No non-204 success responses to check')
        self.assertEqual(mislabelled, [], f'mislabelled: {mislabelled}')

    def test_no_operation_declares_a_status_it_cannot_return(self):
        """Multiple 2xx codes only where the handler returns a variant."""
        offenders = []
        for path, method, op in self.operations():
            codes = {c for c in op.get('responses', {}) if c.startswith('2')}
            if len(codes) < 2 or op.get('x-medkit-alternates'):
                continue
            offenders.append(f'{op.get("operationId")}: {sorted(codes)}')
        self.assertEqual(offenders, [], f'phantom success: {offenders}')

    def test_every_ref_resolves(self):
        """No $ref points at a component the document does not define."""
        spec = self.spec()
        dangling = []
        for ref in refs_in(spec):
            section, name = ref[len('#/components/'):].split('/', 1)
            if name not in spec.get('components', {}).get(section, {}):
                dangling.append(ref)
        self.assertEqual(dangling, [], f'dangling refs: {dangling}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
