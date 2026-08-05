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

# Operations the document must mark `x-medkit-lock-guarded`.
#
# HAND-MAINTAINED, and honestly so. The list cannot be generated: the header
# read that decides the 409 sits in `HandlerContext::validate_lock_access`,
# which 12 handlers across 6 files call, and mapping one of those call sites to
# an operationId would mean evaluating the runtime string concatenation inside
# the four-entity-type registration loop in `rest_server.cpp`. So what this
# list gates is the *document* drifting away from it - dropping a
# `.lock_guarded()` from a registration turns the suite red. It does NOT catch
# the list drifting away from the handlers: a new route that calls
# `validate_lock_access` and forgets both the decorator and an entry here
# passes. Adding a lock check to a handler means editing this list by hand.
#
# Grouped by the handler whose lock check they inherit. Entity types are
# `Area` / `Component` / `App` / `Function`; bulk-data writes exist only for
# `Component` / `App` (areas and functions get hidden 405 stubs).
EXPECTED_LOCK_GUARDED = {
    # DataHandlers::put_data_item -> validate_lock_access("data")
    'putAreaDataItem', 'putComponentDataItem', 'putAppDataItem',
    'putFunctionDataItem',
    # OperationHandlers::create_execution -> validate_lock_access("operations")
    'executeAreaOperation', 'executeComponentOperation', 'executeAppOperation',
    'executeFunctionOperation',
    # OperationHandlers::update_execution -> validate_lock_access("operations")
    'updateAreaExecution', 'updateComponentExecution', 'updateAppExecution',
    'updateFunctionExecution',
    # OperationHandlers::cancel_execution -> validate_lock_access("operations")
    'cancelAreaExecution', 'cancelComponentExecution', 'cancelAppExecution',
    'cancelFunctionExecution',
    # ConfigHandlers::set_configuration -> validate_lock_access("configurations")
    'setAreaConfiguration', 'setComponentConfiguration', 'setAppConfiguration',
    'setFunctionConfiguration',
    # ConfigHandlers::delete_configuration -> validate_lock_access("configurations")
    'deleteAreaConfiguration', 'deleteComponentConfiguration',
    'deleteAppConfiguration', 'deleteFunctionConfiguration',
    # ConfigHandlers::delete_all_configurations -> validate_lock_access("configurations")
    'deleteAllAreaConfigurations', 'deleteAllComponentConfigurations',
    'deleteAllAppConfigurations', 'deleteAllFunctionConfigurations',
    # FaultHandlers::clear_fault -> validate_lock_access("faults")
    'clearAreaFault', 'clearComponentFault', 'clearAppFault',
    'clearFunctionFault',
    # FaultHandlers::clear_all_faults -> validate_lock_access("faults")
    'clearAllAreaFaults', 'clearAllComponentFaults', 'clearAllAppFaults',
    'clearAllFunctionFaults',
    # LogHandlers::put_logs_configuration -> validate_lock_access("logs")
    'setAreaLogConfiguration', 'setComponentLogConfiguration',
    'setAppLogConfiguration', 'setFunctionLogConfiguration',
    # BulkDataHandlers::upload -> validate_lock_access("bulk-data")
    'uploadComponentBulkData', 'uploadAppBulkData',
    # BulkDataHandlers::remove -> validate_lock_access("bulk-data")
    'deleteComponentBulkData', 'deleteAppBulkData',
}

# Properties whose meaning a client cannot recover from the property name and
# the JSON type alone, so the document has to spell it out. Not a list of
# "everything important" - it is the set this file asserts on, and it grows by
# hand when a field turns out to need prose.
#
# Every name was read out of the DTO header before being written here:
# `dto/locks.hpp` (AcquireLockRequest), `dto/health.hpp` (Health, whose
# dto_name is `HealthStatus`), `dto/triggers.hpp` (TriggerCreateRequest) and
# `dto/logs.hpp` (LogConfiguration, which has exactly two members).
LOAD_BEARING = {
    'AcquireLockRequest': ['lock_expiration', 'scopes', 'break_lock'],
    'HealthStatus': ['timestamp'],
    'TriggerCreateRequest': ['trigger_condition', 'protocol', 'lifetime', 'path'],
    'LogConfiguration': ['severity_filter', 'max_entries'],
}

# Operations whose request body carries a copyable example.
#
# One operationId per body-carrying route family, not all of them: the four
# entity types are registered from a single call site in a loop, so the App
# variant is what proves the call site has the example. A family whose App
# variant lost its `.body_example(...)` turns this red.
EXAMPLE_BODIES = {
    'createAppTrigger',
    'acquireAppLock',
    'executeAppOperation',
    'setAppConfiguration',
    'startAppScriptExecution',
}

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

    def test_load_bearing_properties_are_described(self):
        """The properties in LOAD_BEARING carry prose, on whichever branch holds them.

        Scoped to that list - it says nothing about the rest of the document.
        An optional member renders as ``{anyOf: [<inner>, {type: null}]}``, so
        the description can sit on the property or on the non-null branch, and
        both are accepted: which one a property uses is a consequence of its
        C++ type, not a fact a client cares about.
        """
        schemas = self.spec()['components']['schemas']
        offenders = []
        for name, props in LOAD_BEARING.items():
            schema = schemas.get(name)
            self.assertIsNotNone(schema, f'{name} missing from components/schemas')
            for prop in props:
                node = schema.get('properties', {}).get(prop)
                self.assertIsNotNone(node, f'{name}.{prop} missing')
                described = node.get('description') or any(
                    b.get('description') for b in node.get('anyOf', [])
                    if isinstance(b, dict))
                if not described:
                    offenders.append(f'{name}.{prop}')
        self.assertEqual(offenders, [], f'undocumented: {offenders}')

    def test_every_operation_has_a_description(self):
        """No operation ships without prose.

        ``summary`` is a one-liner a picker shows; ``description`` is where the
        behaviour a caller has to know about goes. An operation with only the
        former is one whose caveats live in the source and nowhere a client can
        read them.
        """
        missing = [f'{m.upper()} {p}' for p, m, op in self.operations()
                   if not op.get('description')]
        self.assertEqual(missing, [], f'{len(missing)} operations without description')

    def test_non_trivial_request_bodies_carry_an_example(self):
        """Each operation in EXAMPLE_BODIES publishes a copyable body.

        A ``$ref`` tells a client the field names and types; it does not tell it
        that ``trigger_condition`` needs a ``condition_type`` key or that
        ``interval`` is a word rather than a number. The example is the only
        part of the document a caller can paste into a request unmodified.
        """
        missing = sorted(oid for _, _, op in self.operations()
                         if (oid := op.get('operationId')) in EXAMPLE_BODIES
                         and not any('example' in b or 'examples' in b
                                     for b in (op.get('requestBody') or {})
                                     .get('content', {}).values()))
        self.assertEqual(missing, [], f'no example: {missing}')
        # Guard against a vacuous pass: an operationId that stopped existing
        # would silently drop out of the comprehension above.
        present = {op.get('operationId') for _, _, op in self.operations()}
        self.assertEqual(
            EXAMPLE_BODIES - present, set(),
            'EXAMPLE_BODIES names an operation the document lacks')

    def test_every_tag_used_is_declared(self):
        """No operation carries a tag missing from the document tag list."""
        declared = {t['name'] for t in self.spec().get('tags', [])}
        for path, method, op in self.operations():
            for tag in op.get('tags', []):
                self.assertIn(
                    tag, declared, f'{method.upper()} {path}: tag "{tag}" not declared')

    def test_capability_description_endpoints_are_documented(self):
        """The document describes the endpoints that serve it.

        Both `/docs` routes used to be mounted straight onto the HTTP server,
        so the one document every client fetches was the one that never
        mentioned how it got there. The scoped `<entity-path>/docs`
        sub-documents were worse off: nothing in any response links to one, so
        outside the hand-written prose in `docs/api/rest.rst` there was no
        machine-readable statement that they exist at all.
        """
        paths = self.spec()['paths']
        self.assertIn('/docs', paths)
        self.assertIn('/{entity_path}/docs', paths)
        # The regex the route is actually mounted on must not leak into a
        # path key: `(.+)/docs$` is not something a client can fill in.
        self.assertEqual(
            [p for p in paths if '(' in p], [],
            'a cpp-httplib regex reached the document as a path key')

    def test_docs_routes_keep_their_json_content_type(self):
        """Both `/docs` routes answer `application/json`, indented, and repeat byte-for-byte.

        The generator caches these documents serialized rather than as parsed
        DOMs, and both routes write that text straight to the response. Three
        things a client depends on have to survive that: the header it selects
        on, the 2-space indentation the gateway's JSON convention promises, and
        a repeat request answering the same bytes - the last being what says a
        cache hit and a cache miss are indistinguishable on the wire.

        The two routes reach the response by different writers (the typed
        router for `/docs`, the raw handler for the scoped one), so both are
        checked. Exact `dump(2)` canonicality is asserted in
        ``test_docs_handlers.cpp``, where nlohmann itself is available to
        define it; Python's ``json.dumps`` is not byte-equivalent.
        """
        for endpoint in ('/docs', '/apps/docs'):
            with self.subTest(endpoint=endpoint):
                response = self.get_raw(endpoint)
                self.assertEqual(
                    response.headers.get('Content-Type'), 'application/json',
                    f'{endpoint}: Content-Type moved')
                body = response.content.decode('utf-8')
                json.loads(body)  # must still parse
                second_line = body.split('\n')[1]
                self.assertTrue(
                    second_line.startswith('  "') and not second_line.startswith('   '),
                    f'{endpoint}: body is not indented with 2 spaces')
                repeat = self.get_raw(endpoint).content.decode('utf-8')
                self.assertEqual(
                    body, repeat,
                    f'{endpoint}: a repeat request answered different bytes')

    def test_plugin_routes_are_documented(self):
        """A route a plugin serves and an entity links to is in the document.

        This fixture loads the graph provider, which exports
        `describe_plugin_routes`. Plugin routes are mounted outside the
        `RouteRegistry`, so nothing else in the document would mention them.
        """
        op = self.spec()['paths'].get('/functions/{function_id}/x-medkit-graph', {}).get('get')
        self.assertIsNotNone(op, 'the graph provider route is not documented')
        self.assertTrue(
            op.get('x-medkit-plugin-served'),
            'a folded plugin operation must say it is plugin-served - '
            'test_openapi_error_coverage reads that marker')
        # The gateway declares whatever tag a plugin picks, so the document
        # cannot acquire an undeclared one from a plugin.
        declared = {t['name'] for t in self.spec().get('tags', [])}
        for tag in op.get('tags', []):
            self.assertIn(tag, declared)

    def test_no_operation_publishes_a_role_when_auth_is_off(self):
        """A document this gateway serves does not claim a role it never checks.

        Scoped to every operation, not to the plugin-served ones: the rule is a
        property of the gateway, not of where an operation came from, and the
        gateway applies it once over the finished document. Today only the
        plugin-folded operation declares a role - `RouteEntry` has no role API
        yet - so this reads as a plugin assertion, but it is the same assertion
        that must hold for the gateway's own routes once they declare one, and
        it will start covering them without an edit.

        This fixture runs with `auth.enabled` false, and
        `AuthManager::requires_authentication` returns false outright in that
        configuration - every caller is admitted. The graph provider's
        `describe_plugin_routes` declares `requires_role("admin")`, and the
        gateway strips the requirement rather than publish one it does not
        honour. The scheme *definition* stays: a definition nothing requires
        claims nothing, and it is what lets the auth-on document name it.

        The opposite direction - auth on, requirement present - is
        `test_auth::test_02b_plugin_operation_publishes_its_role_when_auth_is_on`.
        """
        spec = self.spec()
        self.assertFalse(
            spec.get('security'),
            'no document-level requirement with authentication disabled')
        offenders = sorted(
            op.get('operationId', f'{m.upper()} {p}')
            for p, m, op in self.operations() if 'security' in op)
        self.assertEqual(
            offenders, [],
            f'operations publishing a role a disabled gateway ignores: {offenders}')
        # Guard against a vacuous pass: the plugin operation whose description
        # declares a role has to be in this document for the rule above to have
        # had anything to strip.
        graph = spec['paths'].get(
            '/functions/{function_id}/x-medkit-graph', {}).get('get')
        self.assertIsNotNone(graph, 'nothing here declared a role to strip')
        self.assertIn(
            'bearerAuth', spec.get('components', {}).get('securitySchemes', {}),
            'the scheme definition is registered whatever auth.enabled says')

    def test_no_malformed_path_keys(self):
        """Path keys have a leading slash and no empty segments."""
        for path in self.spec()['paths']:
            self.assertTrue(path.startswith('/'), f'{path}: missing leading slash')
            self.assertNotIn('//', path, f'{path}: empty path segment')

    def test_entity_id_parameters_follow_the_projection_naming(self):
        """Every entity-id path parameter is `<singular>_id`.

        ``CapabilityGenerator::entity_template`` derives the parameter name of
        an entity segment by dropping a trailing ``s`` and appending ``_id``,
        and the ``<entity-path>/docs`` projection substitutes the caller's id on
        that name. A route registered as ``/components/{comp_id}/...`` would
        therefore never be substituted, and would disappear from the
        **entity-scoped** spec while still being served - silently, because the
        projection reports the routes it recognises rather than failing on one
        it does not.

        Scoped deliberately: it survives in the *collection*-level spec.
        ``generate_entity_collection`` passes no bindings, so it substitutes
        nothing and filters on the collection prefix alone -
        ``GET /components/docs`` still lists ``/components/{comp_id}/hosts``.
        Only a document whose prefix carries the id loses the route.

        The convention is otherwise only stated in a comment. A total drift
        would show up in ``test_docs_endpoint``; a single renamed route would
        not, which is what this closes.

        Two limits, since this checks less than its name suggests. The keyword
        tuple below is hardcoded while the route side reads the explicit
        singular table in ``rest_server.cpp``, so a **new** entity type is
        unchecked here and this still passes. And the non-vacuity floor is
        loose: 141 pairs exist today, so losing every ``functions``, ``areas``,
        ``subareas`` and ``subcomponents`` pair would leave 83 and still clear
        it. Both bound how much drift this notices, not whether what it
        notices is real.

        @verifies REQ_INTEROP_002
        """
        keywords = ('areas', 'subareas', 'components', 'subcomponents',
                    'apps', 'functions')
        offenders = []
        checked = 0
        for path in self.spec()['paths']:
            segments = path.strip('/').split('/')
            for parent, child in zip(segments, segments[1:]):
                if parent not in keywords or not child.startswith('{'):
                    continue
                checked += 1
                expected = '{' + parent[:-1] + '_id}'
                if child != expected:
                    offenders.append(f'{path}: {child} should be {expected}')
        self.assertEqual(offenders, [], f'entity id parameter naming: {offenders}')
        # A document with no entity-scoped templates would pass vacuously.
        self.assertGreater(checked, 50, f'only {checked} entity segments examined')

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
        """Multiple 2xx codes only where a second one is genuinely reachable.

        Exactly two things make a second 2xx reachable, and each declares
        itself: a handler returning a ``std::variant``
        (``x-medkit-alternates``), and a range-capable download where the HTTP
        layer answers 206 to a ``Range`` request
        (``x-medkit-partial-content``). They are separate markers on purpose -
        one marker covering both would wave through a route that declares a
        status it can never return.
        """
        offenders = []
        for path, method, op in self.operations():
            codes = {c for c in op.get('responses', {}) if c.startswith('2')}
            if len(codes) < 2:
                continue
            if op.get('x-medkit-alternates') or op.get('x-medkit-partial-content'):
                continue
            offenders.append(f'{op.get("operationId")}: {sorted(codes)}')
        self.assertEqual(offenders, [], f'phantom success: {offenders}')

    def test_partial_content_routes_declare_the_range_response(self):
        """A route marked partial-content declares 206 with ``Content-Range``.

        The marker is what lets the rule above accept a second 2xx, so it must
        not be usable as a blanket exemption: whatever carries it has to
        actually describe the partial response.
        """
        marked = []
        for path, method, op in self.operations():
            if not op.get('x-medkit-partial-content'):
                continue
            marked.append(f'{method.upper()} {path}')
            partial = op.get('responses', {}).get('206')
            self.assertIsNotNone(partial, f'{method.upper()} {path}: no 206 declared')
            self.assertIn(
                'Content-Range', partial.get('headers', {}),
                f'{method.upper()} {path}: 206 without Content-Range')
            full = op.get('responses', {}).get('200')
            self.assertIsNotNone(full, f'{method.upper()} {path}: no 200 declared')
            self.assertIn(
                'Accept-Ranges', full.get('headers', {}),
                f'{method.upper()} {path}: 200 without Accept-Ranges')
        self.assertTrue(marked, 'No partial-content routes in the document')

    def header_params(self, op):
        """Return {name: parameter} for every header parameter of an operation."""
        return {
            p['name']: p
            for p in op.get('parameters', [])
            if p.get('in') == 'header'
        }

    def test_no_response_declares_a_null_schema(self):
        """No response object carries ``"schema": null``.

        A guard, not a fix for a known defect. ``nlohmann::json`` default-
        constructs to ``null``, so any emitter that stops checking for an empty
        schema before writing it publishes ``schema: null`` - which a code
        generator reads as "a body typed null", not as "no body".
        """
        offenders = []
        checked = 0
        for path, method, op in self.operations():
            for code, resp in op.get('responses', {}).items():
                for media_type, media in resp.get('content', {}).items():
                    checked += 1
                    if 'schema' in media and media['schema'] is None:
                        offenders.append(
                            f'{op.get("operationId")}: {code} {media_type}')
        self.assertGreater(checked, 0, 'No response content to check')
        self.assertEqual(offenders, [], f'null schema: {offenders}')

    def test_binary_downloads_are_not_declared_as_json(self):
        """A range-aware download declares byte media types, never JSON.

        The download body is raw file content. Declaring it under
        ``application/json`` told every generated client to parse a rosbag as
        JSON; the schema that came with it,
        ``{"type": "string", "format": "binary"}``, is an OpenAPI 3.0 idiom
        that 3.1 dropped, so it was wrong on both axes at once.
        """
        checked = 0
        for path, method, op in self.operations():
            if not op.get('x-medkit-partial-content'):
                continue
            for code in ('200', '206'):
                content = op.get('responses', {}).get(code, {}).get('content')
                self.assertTrue(
                    content, f'{method.upper()} {path}: {code} declares no content')
                checked += 1
                self.assertNotIn(
                    'application/json', content,
                    f'{method.upper()} {path}: {code} advertises a binary body as JSON')
                for media_type, media in content.items():
                    self.assertNotIn(
                        'schema', media,
                        f'{method.upper()} {path}: {code} {media_type} carries a schema; '
                        'a non-JSON body is declared by media type alone')
        self.assertGreater(checked, 0, 'No binary downloads in the document')

    def test_sse_routes_declare_the_event_stream_media_type(self):
        """Every SSE route declares ``text/event-stream`` and its frame shape.

        The media type is what cpp-httplib is handed at
        ``set_chunked_content_provider``, so the document and the wire come
        from one fact.

        The frame schema is per-family, not global: the three families put
        different shapes in ``data:``, so ``sse()`` declares none and each
        registration attaches its own. Declaring a schema against
        ``text/event-stream`` is sound because a ``data:`` field is a JSON
        document - unlike a binary download, where the media type is the whole
        description and ``response()`` still refuses a schema.
        """
        streams = {}
        for path, method, op in self.operations():
            content = op.get('responses', {}).get('200', {}).get('content', {})
            if 'text/event-stream' not in content:
                continue
            where = f'{method.upper()} {path}'
            self.assertEqual(
                list(content), ['text/event-stream'],
                f'{where}: an event stream declares one media type')
            schema = content['text/event-stream'].get('schema', {})
            self.assertIn(
                '$ref', schema, f'{where}: event stream without a frame schema')
            streams[where] = schema['$ref'].split('/')[-1]
        # Four trigger-event streams (one per entity type), three subscription
        # streams (apps / components / functions) and the global fault stream.
        self.assertEqual(
            len(streams), 8, f'expected 8 SSE routes, found {sorted(streams)}')
        # Three distinct shapes across the eight routes; a single one would
        # mean a family had been given another family's schema.
        self.assertEqual(
            set(streams.values()),
            {'TriggerEventFrame', 'SubscriptionEventFrame', 'FaultStreamEvent'},
            f'frame schemas: {streams}')
        self.assertEqual(streams['GET /faults/stream'], 'FaultStreamEvent')

    def test_sse_routes_keep_their_stream_headers(self):
        """Naming the frame shape does not cost the stream its headers.

        The frame schema is attached after ``sse()`` has already declared the
        200 and its headers. Attaching it with a second ``response(200, ...)``
        would replace that response object wholesale and silently drop them,
        which is exactly the kind of loss a reader would not notice.
        """
        checked = 0
        for path, method, op in self.operations():
            ok = op.get('responses', {}).get('200', {})
            if 'text/event-stream' not in ok.get('content', {}):
                continue
            checked += 1
            headers = ok.get('headers', {})
            self.assertIn('Cache-Control', headers, f'{method.upper()} {path}')
            self.assertIn('X-Accel-Buffering', headers, f'{method.upper()} {path}')
        self.assertEqual(checked, 8)

    def test_fault_stream_declares_the_replay_header(self):
        """``Last-Event-ID`` is declared, because the stream sets ``id:``.

        The fault stream is the only one that numbers its frames, and it parses
        ``Last-Event-ID`` to resume after a drop. Undeclared, the ``id:`` field
        is a number a client can see and cannot use.
        """
        op = self.spec()['paths']['/faults/stream']['get']
        header = self.header_params(op).get('Last-Event-ID')
        self.assertIsNotNone(header, 'fault stream without Last-Event-ID')
        self.assertFalse(
            header.get('required'),
            'a first connection has no last event id')

    def test_binary_downloads_still_refuse_a_frame_schema(self):
        """The SSE relaxation did not open the door for binary bodies.

        ``response()`` publishes a schema against a non-JSON media type only
        when every declared type carries a JSON document. This pins the other
        side of that rule: a rosbag download must stay described by its media
        type alone.
        """
        checked = 0
        for path, method, op in self.operations():
            if not op.get('x-medkit-partial-content'):
                continue
            for code in ('200', '206'):
                for media_type, media in op['responses'][code]['content'].items():
                    checked += 1
                    self.assertNotIn(
                        'schema', media,
                        f'{method.upper()} {path}: {code} {media_type} carries a schema')
        self.assertGreater(checked, 0, 'No binary downloads in the document')

    def test_partial_content_routes_declare_the_range_request(self):
        """The response half of the Range contract has a request half.

        ``Accept-Ranges`` invites the client to send ``Range``; an undeclared
        ``Range`` parameter leaves a generated client no way to accept.
        """
        checked = 0
        for path, method, op in self.operations():
            if not op.get('x-medkit-partial-content'):
                continue
            checked += 1
            where = f'{method.upper()} {path}'
            headers = self.header_params(op)
            self.assertIn('Range', headers, f'{where}: 206 declared without a Range parameter')
            self.assertFalse(
                headers['Range'].get('required'),
                f'{where}: a download without Range must still serve the whole body')
        self.assertGreater(checked, 0, 'No partial-content routes in the document')

    def test_every_operation_declares_the_range_rejection(self):
        """416 is declared on every operation, because httplib answers it there.

        cpp-httplib rejects an unparseable ``Range`` header in
        ``Server::process_request``, before routing and before any handler, so
        the status is reachable on every operation rather than only on the six
        downloads where a ``Range`` is *useful*.
        """
        missing = []
        wrong_shape = []
        for path, method, op in self.operations():
            resp = op.get('responses', {}).get('416')
            if resp is None:
                missing.append(f'{method.upper()} {path}')
                continue
            if resp.get('$ref') != '#/components/responses/GenericError':
                wrong_shape.append(f'{method.upper()} {path}')
        self.assertEqual(missing, [], f'operations not declaring 416: {missing}')
        self.assertEqual(
            wrong_shape, [], f'416 not declared as a GenericError: {wrong_shape}')

    def test_range_rejection_is_answered_on_a_route_that_declares_it(self):
        """Drive the declared 416 on the wire, on a route that is not a download.

        The status recorder cannot see this one - no handler runs - so without
        a wire check the registry-wide declaration would rest on reading
        cpp-httplib rather than on observing it. ``/health`` is deliberately
        not a download: if 416 only ever appeared on the six binary routes,
        declaring it on all of them would be an over-declaration.

        Reading the vendored header alone gets the body wrong. cpp-httplib
        writes 416 with no body at all; the gateway's own
        ``set_error_handler`` then fills any body-less error response with a
        ``GenericError``, which is why the declaration is a ``$ref`` and not a
        body-less response object. That is only observable against a real
        gateway, so it is asserted here rather than in the registry unit tests.
        """
        declared = self.spec()['paths']['/health']['get']['responses']
        self.assertIn('416', declared, 'the assertion below would prove nothing')

        r = requests.get(
            f'{self.BASE_URL}/health', headers={'Range': 'furlongs=1-2'}, timeout=10)
        self.assertEqual(r.status_code, 416, r.text)
        body = r.json()
        for field in ('error_code', 'message', 'parameters'):
            self.assertIn(
                field, body, f'416 body is not the declared GenericError shape: {body}')

        # And the same route answers normally without the header, so the 416
        # above is the Range rejection and not a broken endpoint.
        ok = requests.get(f'{self.BASE_URL}/health', timeout=10)
        self.assertEqual(ok.status_code, 200)

    def test_lock_guarded_set_matches_the_handlers(self):
        """Every route whose handler checks a lock declares the contract.

        Read the failure message before editing either side: the expected set
        is hand-maintained (see EXPECTED_LOCK_GUARDED), so a mismatch means
        either a registration lost its ``.lock_guarded()`` or a handler gained
        a lock check nobody recorded here.
        """
        marked = {op.get('operationId') for _, _, op in self.operations()
                  if op.get('x-medkit-lock-guarded')}
        self.assertEqual(marked, EXPECTED_LOCK_GUARDED)

    def test_lock_guarded_routes_declare_the_contract(self):
        """The marker is not a bare label: it carries a header and a 409.

        A route that says it takes part in locking without publishing the
        ``X-Client-Id`` it reads, or without the 409 it answers to the wrong
        client, hands a generated client a marker it cannot act on.
        """
        marked = 0
        for path, method, op in self.operations():
            if not op.get('x-medkit-lock-guarded'):
                continue
            marked += 1
            where = f'{method.upper()} {path}'
            headers = self.header_params(op)
            self.assertIn(
                'X-Client-Id', headers, f'{where}: lock-guarded without X-Client-Id')
            self.assertFalse(
                headers['X-Client-Id'].get('required'),
                f'{where}: X-Client-Id declared required, but a header-less '
                f'request succeeds while nothing is locked')
            self.assertIn(
                '409', op.get('responses', {}), f'{where}: lock-guarded without 409')
        self.assertTrue(marked, 'No lock-guarded routes in the document')

    def test_lock_guarded_route_answers_the_409_it_declares(self):
        """A real write by the wrong client answers the documented 409.

        The marker and the 409 are hand-declared, so on their own they prove
        only that somebody typed them. This drives the lock through the real
        gateway: client A takes the lock, client B writes, and the status the
        wire returns has to be the one the document declares.
        """
        op = self.spec()['paths']['/apps/{app_id}/data/{data_id}']['put']
        self.assertTrue(op.get('x-medkit-lock-guarded'), 'putAppDataItem lost the marker')
        self.assertIn('409', op.get('responses', {}))

        acquired = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/locks',
            json={'lock_expiration': 300},
            headers={'X-Client-Id': 'lock_contract_a'},
            timeout=10,
        )
        self.assertEqual(acquired.status_code, 201, acquired.text)
        self.addCleanup(
            requests.delete,
            f'{self.BASE_URL}/apps/temp_sensor/locks/{acquired.json()["id"]}',
            headers={'X-Client-Id': 'lock_contract_a'},
            timeout=10,
        )

        blocked = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/data/engine_temperature',
            json={'type': 'std_msgs/msg/Float64', 'data': {'data': 42.0}},
            headers={'X-Client-Id': 'lock_contract_b'},
            timeout=10,
        )
        self.assertEqual(blocked.status_code, 409, blocked.text)

    def test_global_fault_clear_reads_the_client_id_without_declaring_409(self):
        """``DELETE /faults`` publishes the header but not the lock marker.

        It reads ``X-Client-Id`` like the lock-guarded writes but never answers
        409: locked entities are silently skipped and the request still answers
        204, with nothing on the response naming what was skipped. Marking it
        lock-guarded would publish a status it cannot return.

        The 204 also carries ``X-Medkit-Local-Only``, which is asserted here
        only to keep the two from being confused: it reports that aggregated
        peers were not cleared, and says nothing about locks.
        """
        op = self.spec()['paths']['/faults']['delete']
        self.assertIn('X-Client-Id', self.header_params(op))
        self.assertNotIn('x-medkit-lock-guarded', op)
        local_only = op['responses']['204'].get('headers', {}).get(
            'X-Medkit-Local-Only')
        self.assertIsNotNone(local_only)
        self.assertNotIn(
            'lock', (local_only.get('description') or '').lower(),
            'X-Medkit-Local-Only is about peers, not locks')

    def test_no_fan_out_header_is_declared_as_a_string(self):
        """``X-Medkit-No-Fan-Out`` is presence-only, so it is not a boolean.

        The gateway tests ``has_header`` and never reads the value, so
        ``X-Medkit-No-Fan-Out: false`` still suppresses fan-out. A boolean
        schema would promise a generated client the opposite.
        """
        declared = 0
        for path, method, op in self.operations():
            param = self.header_params(op).get('X-Medkit-No-Fan-Out')
            if param is None:
                continue
            declared += 1
            where = f'{method.upper()} {path}'
            self.assertEqual(
                param.get('schema', {}).get('type'), 'string',
                f'{where}: X-Medkit-No-Fan-Out is presence-only, not typed')
            self.assertFalse(
                param.get('required'), f'{where}: opting out cannot be mandatory')
        self.assertTrue(declared, 'No fan-out-aware routes in the document')

    def test_every_created_or_accepted_declares_location(self):
        """Every 201/202 publishes the `Location` header the handler sets.

        A 201 that does not name the resource it created, and a 202 that does
        not name the resource whose status tracks the request, force a
        generated client to guess the URI it was just handed. The gateway sets
        `Location` on both; the document has to say so.
        """
        missing = []
        checked = 0
        for path, method, op in self.operations():
            for code, resp in op.get('responses', {}).items():
                if code not in ('201', '202'):
                    continue
                checked += 1
                if 'Location' not in resp.get('headers', {}):
                    missing.append(f'{op.get("operationId")}: {code}')
        self.assertGreater(checked, 0, 'No 201/202 responses to check')
        self.assertEqual(missing, [], f'no Location declared: {missing}')

    def test_created_response_sends_the_location_it_declares(self):
        """The declared `Location` reaches the wire, in the documented form.

        Declaring the header is only half the contract - this drives a real
        201 and asserts the header is present and is the absolute, prefixed
        path form every `href` in the document uses.
        """
        op = self.spec()['paths']['/apps/{app_id}/triggers']['post']
        self.assertIn('Location', op['responses']['201'].get('headers', {}))
        resp = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/triggers',
            json={
                'resource': '/api/v1/apps/temp_sensor/faults',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
            },
            timeout=10,
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        trigger_id = resp.json()['id']
        self.addCleanup(
            requests.delete,
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trigger_id}',
            timeout=10,
        )
        self.assertEqual(
            resp.headers.get('Location'),
            f'/api/v1/apps/temp_sensor/triggers/{trigger_id}',
        )

    def test_shipped_route_set_declares_complete_metadata(self, proc_output):
        """The gateway's own route-metadata check finds nothing on this fixture.

        `RouteRegistry::validate_completeness()` catches what the document
        contract cannot see from the served JSON alone: `errors()` handed a
        sub-400 status, `response_header()` aimed at a status no response
        declares, a route registered without a tag or without a success schema.
        The gateway runs it at start-up and logs a summary; without this
        assertion that summary is a line nobody reads, which is the same defect
        as collecting the issues and discarding them.

        This fixture launches every optional feature gate, so the route set
        under test is the maximal one - a route added behind any gate is
        covered here.
        """
        # The `check: ` prefix is load-bearing - matching a bare `0 error(s)`
        # would also match `10 error(s)` and pass on a broken route set.
        proc_output.assertWaitFor('OpenAPI metadata check: 0 error(s)', timeout=20)

    def test_every_ref_resolves(self):
        """No $ref points at a component the document does not define."""
        spec = self.spec()
        dangling = []
        for ref in refs_in(spec):
            section, name = ref[len('#/components/'):].split('/', 1)
            if name not in spec.get('components', {}).get(section, {}):
                dangling.append(ref)
        self.assertEqual(dangling, [], f'dangling refs: {dangling}')

    def test_auth_endpoints_declare_the_oauth2_error_shape(self):
        """RFC 6749 endpoints must not declare the SOVD error body.

        `/auth/*` routes set `.error_renderer(kOAuth2Error)`, so every error
        their handler returns reaches the wire as
        ``{error, error_description}``. The document used to say
        ``GenericError`` on all of them, which told a generated client to read
        ``error_code`` on a body that has no such key.
        """
        checked = 0
        for path, method, op in self.operations():
            if not path.startswith('/auth/'):
                continue
            for code in ('400', '500'):
                ref = op['responses'].get(code, {}).get('$ref', '<absent>')
                checked += 1
                self.assertIn(
                    'OAuth2Error', ref, f'{method.upper()} {path} [{code}]: {ref}')
        self.assertTrue(checked, 'No /auth/ operations in the document')

    def test_auth_token_routes_declare_the_credential_rejection(self):
        """The two credential-checking routes declare their 401.

        Both answer 401 from the handler on bad credentials or a dead refresh
        token, independently of ``auth.enabled`` - so unlike the middleware's
        401 it is always reachable. ``/auth/revoke`` is deliberately excluded:
        RFC 7009 section 2.2 forbids it from revealing whether the token was
        valid, so it answers 200 either way and declaring a 401 there would
        publish a status it cannot return.
        """
        for path in ('/auth/authorize', '/auth/token'):
            op = self.spec()['paths'][path]['post']
            self.assertIn('401', op['responses'], f'{path}: no 401 declared')
            self.assertIn('OAuth2Error', op['responses']['401'].get('$ref', ''))
        revoke = self.spec()['paths']['/auth/revoke']['post']
        self.assertNotIn(
            '401', revoke['responses'],
            '/auth/revoke answers 200 for an invalid token (RFC 7009 2.2)')

    def test_auth_endpoints_accept_form_encoding(self):
        """RFC 6749 clients default to form encoding; the spec must allow it.

        ``AuthorizeRequest::parse_request`` takes either encoding of the same
        payload. ``/auth/revoke`` parses JSON only and is excluded on purpose.
        """
        for path in ('/auth/authorize', '/auth/token'):
            media = set(
                (self.spec()['paths'][path]['post'].get('requestBody') or {})
                .get('content', {}))
            self.assertEqual(
                media, {'application/json', 'application/x-www-form-urlencoded'}, path)
        revoke_media = set(
            (self.spec()['paths']['/auth/revoke']['post'].get('requestBody') or {})
            .get('content', {}))
        self.assertEqual(revoke_media, {'application/json'}, '/auth/revoke is JSON-only')

    def test_generic_error_declares_the_vendor_code(self):
        """Vendor errors carry a 4th key the schema must declare.

        ``write_generic_error`` rewrites ``error_code`` to the ``vendor-error``
        sentinel and moves the real ``x-medkit-*`` code into ``vendor_code``.
        A schema without that key tells a client every vendor failure is the
        same error.
        """
        props = self.spec()['components']['schemas']['GenericError']['properties']
        self.assertIn('vendor_code', props)
        self.assertTrue(props['vendor_code'].get('description'))

    def test_generic_error_declares_the_recoverable_parameters(self):
        """The `parameters` keys a client branches on are schema, not prose.

        `parameters` is open-ended, so its C++ type publishes
        ``anyOf: [{}, {"type": "null"}]`` - which does not even say "object".
        The two keys that carry a recovery path are declared as real
        properties: ``existing_lock_id`` (the lock to break after a 409 from
        the acquire) and ``lock_id`` (the lock that blocked a guarded write).
        ``additionalProperties`` stays open, because the diagnostic keys are an
        open set and a plugin can add its own.
        """
        params = self.spec()['components']['schemas']['GenericError']['properties']['parameters']
        self.assertEqual(params.get('type'), 'object')
        self.assertTrue(params.get('additionalProperties'))
        declared = params.get('properties', {})
        for key in ('existing_lock_id', 'lock_id'):
            self.assertIn(key, declared, f'{key} is a recovery key, not diagnostic prose')
            self.assertEqual(declared[key].get('type'), 'string')
            self.assertTrue(declared[key].get('description'))

    def test_no_request_body_is_an_untyped_bag(self):
        """A request body is typed, or explicitly marked opaque.

        Two shapes both mean "some JSON, good luck": the
        ``{"type": "object", "additionalProperties": true}`` placeholder the
        multipart helper used to install, and the bare ``{"type": "object"}``
        a hand-written ``request_body`` produced. Neither tells a generated
        client a single field name, so neither can be used to build a request.

        ``x-medkit-opaque`` is the escape hatch, and it is deliberately a
        marker a call site has to write: a body whose shape a plugin decides
        says so, rather than being indistinguishable from one nobody typed.
        """
        offenders = []
        for path, method, op in self.operations():
            for media, body in (op.get('requestBody') or {}).get('content', {}).items():
                schema = body.get('schema', {})
                if '$ref' in schema or schema.get('x-medkit-opaque'):
                    continue
                if schema.get('type') == 'object' and 'properties' not in schema:
                    offenders.append(f'{method.upper()} {path} [{media}]')
        self.assertEqual(offenders, [], f'untyped request bodies: {offenders}')

    def test_multipart_bodies_name_their_parts(self):
        """Every multipart body declares its parts, and binary parts an encoding.

        The rule above only proves a multipart body has *some* properties. This
        one proves the declaration is usable: a part a client must send is in
        ``required``, and a part carrying bytes says so through
        ``encoding.<part>.contentType`` - which in OpenAPI 3.1 is the only way
        to say it, ``format: binary`` having been dropped with the rest of the
        pre-2020-12 vocabulary.
        """
        checked = 0
        for path, method, op in self.operations():
            body = (op.get('requestBody') or {}).get(
                'content', {}).get('multipart/form-data')
            if body is None:
                continue
            checked += 1
            where = f'{method.upper()} {path}'
            schema = body.get('schema', {})
            self.assertTrue(schema.get('properties'), f'{where}: no parts declared')
            self.assertIn('file', schema['properties'], f'{where}: no file part')
            self.assertIn(
                'file', schema.get('required', []),
                f'{where}: the file part is rejected when absent, so it is required')
            self.assertEqual(
                body.get('encoding', {}).get('file', {}).get('contentType'),
                'application/octet-stream',
                f'{where}: binary part without an encoding contentType')
        # Two script uploads (apps / components) and two bulk-data uploads.
        self.assertEqual(checked, 4, f'expected 4 multipart bodies, found {checked}')

    def test_no_operation_has_a_content_free_body(self):
        """A schema is typed, or explicitly opaque with a reason.

        ``{"type": "object"}`` and nothing else is not documentation - it is
        the absence of it, and a generated client turns it into an untyped
        map. Some bodies genuinely have no fixed shape because a plugin owns
        it; those stay opaque and say so, naming who decides the shape and
        where a client discovers it. What this rule forbids is the silent
        version.
        """
        schemas = self.spec()['components']['schemas']
        opaque = {n for n, s in schemas.items()
                  if s.get('type') == 'object' and 'properties' not in s}
        undocumented = sorted(n for n in opaque if not schemas[n].get('description'))
        self.assertEqual(undocumented, [], f'opaque without a reason: {undocumented}')

    def test_no_unreachable_schemas(self):
        """Every named schema is reachable from some operation.

        `components/schemas` is unconditionally all of `dto::AllDtos`, so a DTO
        that stops being a route's response type - or was never one - keeps
        shipping a named type every generated client materialises and none can
        ever receive. The walk mirrors `openapi::unreachable_schemas()`, which
        the gateway runs over the same document and warns about at request time.

        This fixture launches every optional feature gate on purpose: the
        script and update routes are conditional, so with them off their
        schemas would show up here as false orphans.
        """
        spec = self.spec()
        schemas = spec['components']['schemas']
        seen, frontier = set(), refs_in(spec['paths'])
        while frontier:
            ref = frontier.pop()
            if ref in seen:
                continue
            seen.add(ref)
            section, name = ref[len('#/components/'):].split('/', 1)
            body = spec['components'].get(section, {}).get(name, {})
            frontier |= refs_in(body) - seen
        used = {r.split('/')[-1] for r in seen if '/schemas/' in r}
        self.assertEqual(sorted(set(schemas) - used), [], 'unreachable schemas')

    def test_every_advertised_collection_is_served(self):
        """Nothing an entity advertises answers 404.

        Two surfaces advertise an entity's resource collections and they are
        built from two different lists: the ``capabilities`` array on
        ``GET /{type}/{id}`` comes from a per-handler ``CapabilityBuilder``
        call, the entity's ``/docs`` sub-document is projected out of the route
        registry. Both are followed here, for all four entity types, because a
        collection can be right in one list and wrong in the other.

        Only the sub-document paths that declare a ``get`` are followed. Since
        the sub-document became a projection it holds every method the gateway
        serves under the entity, and ``PUT /{type}/{id}/status/restart`` has no
        GET to answer - a 404 there would say nothing about whether the route
        exists.

        A 501 is a served answer - the route exists and reports that the
        backend does not. A 404 is what this pins: an href the gateway
        published that no route answers.

        @verifies REQ_INTEROP_002
        """
        offenders = []
        covered = {}
        for entity_type in ('areas', 'components', 'apps', 'functions'):
            items = self.get_json(f'/{entity_type}').get('items', [])
            if not items:
                # This fixture's demo nodes produce no areas. Skipping keeps
                # the assertion below honest about what was actually probed;
                # the per-type lists themselves are pinned by the
                # `EntityCapabilities` unit tests.
                continue
            entity_id = items[0]['id']
            detail = self.get_json(f'/{entity_type}/{entity_id}')
            subtree = self.get_json(f'/{entity_type}/{entity_id}/docs')
            advertised = {c['href'] for c in detail.get('capabilities', [])}
            advertised |= {f'/api/v1{p}'
                           for p, item in subtree['paths'].items()
                           if 'get' in item}
            followed = 0
            for href in sorted(advertised):
                if '{' in href:
                    # A templated path names no concrete resource to fetch.
                    continue
                resp = requests.get(
                    f'{self.BASE_URL}{href[len("/api/v1"):]}', timeout=10)
                followed += 1
                if resp.status_code == 404:
                    offenders.append(f'{entity_type}: {href}')
            covered[entity_type] = followed
        self.assertEqual(offenders, [], f'advertised but 404: {offenders}')
        # Guard against a vacuous pass: an entity type that advertised nothing,
        # or a listing that came back empty, must not read as green.
        for entity_type in ('components', 'apps', 'functions'):
            self.assertGreater(
                covered.get(entity_type, 0), 8,
                f'{entity_type}: only {covered.get(entity_type, 0)} hrefs followed')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
