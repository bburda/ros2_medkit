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

"""OpenAPI response-schema drift test.

For every GET endpoint declared in the runtime OpenAPI spec served at
GET /docs, this test fetches a real response from the live gateway and
validates it against the response schema declared in the same spec for
status 200. The validator runs jsonschema in its default mode, so what
this catches is the *subset* of drift that breaks the schema's positive
constraints:

- ``required`` fields the handler does not emit
- type mismatches (string vs number, etc.)
- ``enum`` values the handler emits but the schema does not list
- malformed nested objects against declared sub-schemas

What this does **not** catch (because most schemas in
``schema_builder.cpp`` do not set ``additionalProperties: false``):

- handler emits an extra top-level field the schema never declares
  (this was exactly the failure mode that allowed flat
  ``x-medkit-phase`` to slip through before issue #385 was filed)

A closed-object check would surface dozens of pre-existing schema gaps
that are out of scope for #385; the structural fix (compile-time link
from each emitter to its schema) lives under issue #338. Until then,
the explicit ``test_update_status_payload_uses_nested_x_medkit`` guards
the specific regression #385 closes.

If this test fails, fix the schema or the handler; do not silence it.
"""

import os
import re
import unittest

from ament_index_python.packages import get_package_prefix
import launch_testing
import requests

# Humble (Ubuntu 22.04) ships python3-jsonschema 3.2 which only has draft-7;
# Jazzy (Ubuntu 24.04) / Lyrical (Ubuntu 26.04) ship 4.10+ with Draft202012.
# Prefer the newest draft for OpenAPI 3.1 alignment, fall back to Draft7 on Humble.
# The properties we validate (required, type, properties) behave identically across drafts.
try:
    from jsonschema.validators import Draft202012Validator as _Validator
except ImportError:
    from jsonschema.validators import Draft7Validator as _Validator

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    PARAM_SERVICE_TIMEOUT,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def _test_update_backend_path():
    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_gateway', 'libtest_update_backend.so'
    )


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor', 'calibration'],
        gateway_params={
            'updates.enabled': True,
            'plugins': ['test_update_backend'],
            'plugins.test_update_backend.path': _test_update_backend_path(),
        },
        fault_manager=True,
    )


# Path placeholders -> entity collection used for substitution.
_ENTITY_PARAM_MAP = {
    'area_id': 'areas',
    'component_id': 'components',
    'app_id': 'apps',
    'function_id': 'functions',
}


class _RefResolutionError(Exception):
    """Raised when a $ref in the spec cannot be resolved or forms a cycle.

    Returning an empty schema on these conditions would let jsonschema
    validate any payload, masking the drift the test is meant to detect.
    """


def _inline_refs(schema, schemas, seen=None):
    """Recursively inline $refs into a schema so jsonschema can validate it.

    The runtime spec uses $refs that point at #/components/schemas/, but
    Draft202012Validator constructed without a registry will not follow
    them. Cycles and unresolved refs raise ``_RefResolutionError`` rather
    than silently degrading the validator to "anything goes".
    """
    if seen is None:
        seen = set()
    if isinstance(schema, dict):
        if '$ref' in schema:
            ref = schema['$ref']
            if ref in seen:
                raise _RefResolutionError(
                    f'Cycle in $ref chain: {" -> ".join(sorted(seen))} -> {ref}'
                )
            if not ref.startswith('#/components/schemas/'):
                raise _RefResolutionError(f'Unsupported $ref form: {ref}')
            name = ref.rsplit('/', 1)[-1]
            target = schemas.get(name)
            if target is None:
                raise _RefResolutionError(f'Unknown $ref target: {ref}')
            return _inline_refs(target, schemas, seen | {ref})
        return {k: _inline_refs(v, schemas, seen) for k, v in schema.items()}
    if isinstance(schema, list):
        return [_inline_refs(item, schemas, seen) for item in schema]
    return schema


def _is_sse_endpoint(operation):
    """Detect SSE streaming endpoints that would hang on a plain GET."""
    summary = (operation.get('summary') or '') + (operation.get('description') or '')
    return 'SSE' in summary or 'stream' in summary.lower()


class _MissingEntityType(Exception):
    """Raised when a path needs an entity type that was not discovered.

    Substituting a placeholder (e.g. ``'test_entity'``) would yield a path
    the gateway returns 404 for; the drift loop skips non-200 responses,
    so the endpoint would silently exit unvalidated. Callers must catch
    this and explicitly account for the skipped path.
    """


def _substitute_path_params(path, entity_map, resource_id='test_id'):
    def replacer(match):
        param = match.group(1)
        if param in _ENTITY_PARAM_MAP:
            etype = _ENTITY_PARAM_MAP[param]
            if etype not in entity_map:
                raise _MissingEntityType(
                    f'Path param {{{param}}} requires entity type '
                    f'{etype!r} but none was discovered'
                )
            return entity_map[etype]
        return resource_id

    return re.sub(r'\{([^}]+)\}', replacer, path)


class TestOpenApiResponseDrift(GatewayTestCase):
    """Catches handler-vs-spec drift on GET response bodies.

    Scope: GET endpoints declared at runtime under /docs only. POST/PUT/
    DELETE request bodies and non-200 response shapes are out of scope;
    extending coverage to all verbs is tracked under issue #338.
    """

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'calibration'}
    # Conservative lower bound on the number of paths a fully registered
    # gateway exposes (currently ~50 with the test fixture). _fetch_spec
    # polls until at least this many paths are present so a slow plugin
    # registration cannot let the test read back a partial spec and
    # silently skip the endpoints registered after the snapshot.
    MIN_EXPECTED_PATHS = 30

    @classmethod
    def setUpClass(cls):
        super().setUpClass()

        # Discover real entity IDs to use in path substitution. Each
        # entity type may legitimately be empty in some fixture
        # configurations (e.g. runtime_only mode without manifest-declared
        # areas). Endpoints needing a type that was not discovered are
        # skipped explicitly in the validation loop and reported in test
        # output - no longer silently substituted with a placeholder that
        # produces 404s the loop drops.
        cls._entity_map = {}
        for etype in ('areas', 'components', 'apps', 'functions'):
            resp = requests.get(f'{cls.BASE_URL}/{etype}', timeout=5)
            resp.raise_for_status()
            items = resp.json().get('items', [])
            if items:
                cls._entity_map[etype] = items[0].get('id')

        # Register and prepare a test update package so /updates/{id}/status
        # returns an interesting payload (status + x-medkit.phase).
        cls._update_pkg_id = 'drift-test-pkg'
        # Best-effort cleanup of any leftover from a previous interrupted run.
        requests.delete(
            f'{cls.BASE_URL}/updates/{cls._update_pkg_id}', timeout=5
        )
        post = requests.post(
            f'{cls.BASE_URL}/updates',
            json={
                'id': cls._update_pkg_id,
                'update_name': 'Drift test package',
                'automated': False,
                'origins': ['proximity'],
            },
            timeout=5,
        )
        post.raise_for_status()
        prepare = requests.put(
            f'{cls.BASE_URL}/updates/{cls._update_pkg_id}/prepare', timeout=5
        )
        prepare.raise_for_status()

    @classmethod
    def tearDownClass(cls):
        requests.delete(
            f'{cls.BASE_URL}/updates/{cls._update_pkg_id}', timeout=5
        )
        super().tearDownClass()

    def _fetch_spec(self):
        return self.poll_endpoint_until(
            '/docs',
            lambda d: d if (
                d.get('paths') and len(d['paths']) >= self.MIN_EXPECTED_PATHS
            ) else None,
        )

    def _validate_response(self, schema, body, schemas):
        """Validate body against schema with $refs inlined."""
        inlined = _inline_refs(schema, schemas)
        validator = _Validator(inlined)
        return sorted(validator.iter_errors(body), key=lambda e: e.path)

    def _resource_for_path(self, path):
        """Pick a resource_id that makes a given path return 200 when possible.

        Used for paths like /updates/{id}/status where 'test_id' would 404.
        """
        if path.startswith('/updates/{'):
            return self._update_pkg_id
        return 'test_id'

    def test_get_responses_match_declared_schema(self):
        """GET 200 responses must validate against the declared schema.

        Iterates GET endpoints from /docs, fetches each, and validates
        bodies against the response schema declared for status 200. Schema
        drift (handler emits a field schema does not declare as required,
        or vice versa) raises an error.

        Coverage limited to GET verbs; POST/PUT/DELETE request bodies and
        non-200 response shapes are out of scope (tracked under issue
        #338). Endpoints whose path placeholders need an entity type that
        was not discovered (e.g. /functions/{function_id}/... when no
        functions exist) are skipped explicitly with an entry in
        ``skipped_for_missing_entity`` rather than substituted with a
        bogus value that would silently 404 unvalidated.

        @verifies REQ_INTEROP_002
        """
        spec = self._fetch_spec()
        schemas = spec.get('components', {}).get('schemas', {})
        violations = []
        skipped_for_missing_entity = []
        validated = 0

        for path, path_item in spec.get('paths', {}).items():
            operation = path_item.get('get')
            if not operation or _is_sse_endpoint(operation):
                continue

            response_def = operation.get('responses', {}).get('200')
            if not response_def:
                continue
            schema = (
                response_def.get('content', {})
                .get('application/json', {})
                .get('schema')
            )
            if not schema:
                continue

            try:
                uri = _substitute_path_params(
                    path, self._entity_map, self._resource_for_path(path)
                )
            except _MissingEntityType as exc:
                skipped_for_missing_entity.append(f'{path}: {exc}')
                continue
            resp = requests.get(f'{self.BASE_URL}{uri}', timeout=10)

            # Non-200 responses fall outside this drift check (handler may
            # legitimately return 404/501 for the chosen path params); the
            # callability test covers status code shape separately.
            if resp.status_code != 200:
                continue
            if 'application/json' not in resp.headers.get('content-type', ''):
                continue

            body = resp.json()
            try:
                errors = self._validate_response(schema, body, schemas)
            except _RefResolutionError as exc:
                violations.append(f'GET {uri}: spec error: {exc}')
                continue
            if errors:
                detail = '; '.join(
                    f'{".".join(str(p) for p in e.absolute_path) or "<root>"}: {e.message}'
                    for e in errors[:5]
                )
                violations.append(f'GET {uri}: schema drift: {detail}')
            else:
                validated += 1

        self.assertGreater(
            validated, 0, 'No endpoints validated - test fixture broken?'
        )
        # Surface paths skipped due to missing optional entity types
        # (e.g. /functions/{function_id}/... when no functions were
        # declared). The previous fallback substituted 'test_entity',
        # which produced 404s the validation loop silently dropped -
        # hiding entire entity types from the test. Print is captured
        # in test output so the gap is visible without failing CI when
        # the fixture legitimately omits an optional type.
        if skipped_for_missing_entity:
            print(
                f'[drift] {len(skipped_for_missing_entity)} endpoint(s) '
                f'unvalidated (missing optional entity type):\n'
                + '\n'.join(f'  - {s}' for s in skipped_for_missing_entity)
            )
        self.assertFalse(
            violations,
            f'{len(violations)} drift violation(s); validated {validated}:\n'
            + '\n'.join(violations),
        )

    def test_update_routes_answer_with_the_declared_status(self):
        """POST /updates and PUT /updates/{id}/prepare match the document.

        The drift loop above only exercises GET against 200, so the derived
        success status of a write route has no other wire coverage. The
        prepare PUT is the body-less 202 shape (``Accepted<NoContent>``) the
        lifecycle transitions share; no lifecycle provider ships with the
        gateway, so this is where that shape is proven end to end.

        @verifies REQ_INTEROP_002
        """
        spec = self._fetch_spec()

        def declared(path, method):
            op = spec['paths'][path][method]
            codes = sorted(c for c in op.get('responses', {}) if c.startswith('2'))
            self.assertEqual(
                len(codes), 1,
                f'{method.upper()} {path}: expected exactly one declared 2xx, '
                f'got {codes}')
            return int(codes[0])

        pkg_id = 'declared-status-pkg'
        requests.delete(f'{self.BASE_URL}/updates/{pkg_id}', timeout=5)
        self.addCleanup(
            requests.delete, f'{self.BASE_URL}/updates/{pkg_id}', timeout=5
        )

        register_declared = declared('/updates', 'post')
        self.assertEqual(register_declared, 201)
        register = requests.post(
            f'{self.BASE_URL}/updates',
            json={
                'id': pkg_id,
                'update_name': 'Declared status package',
                'automated': False,
                'origins': ['proximity'],
            },
            timeout=5,
        )
        self.assertEqual(register.status_code, register_declared, register.text)

        prepare_declared = declared('/updates/{update_id}/prepare', 'put')
        self.assertEqual(prepare_declared, 202)
        prepare = requests.put(
            f'{self.BASE_URL}/updates/{pkg_id}/prepare', timeout=5
        )
        self.assertEqual(prepare.status_code, prepare_declared, prepare.text)
        self.assertEqual(prepare.text, '', 'Accepted<NoContent> must send no body')

    def test_update_status_payload_uses_nested_x_medkit(self):
        """Specific guard for issue #385: /updates/{id}/status payload.

        The handler must emit ``x-medkit: {phase: ...}`` (nested object), not
        the legacy flat ``x-medkit-phase`` key. The drift test above already
        catches this via the schema declared in update_status_schema(); this
        explicit assertion makes the regression intent obvious.

        @verifies REQ_INTEROP_002
        """
        resp = requests.get(
            f'{self.BASE_URL}/updates/{self._update_pkg_id}/status', timeout=5
        )
        self.assertEqual(resp.status_code, 200)
        body = resp.json()
        self.assertNotIn('x-medkit-phase', body, 'Legacy flat key still emitted')
        self.assertIn('x-medkit', body)
        self.assertIn('phase', body['x-medkit'])

    def test_configurations_payload_uses_nested_x_medkit(self):
        """Specific guard for issue #385: configurations vendor fields nested.

        Items at the top level (only in aggregated mode) and parameters
        inside ``x-medkit.parameters[]`` (always emitted) must carry vendor
        source attribution under a nested ``x-medkit`` object, never as flat
        ``x-medkit-source`` / ``x-medkit-node`` keys. The drift test cannot
        catch reintroduction of the legacy flat keys because
        ``ConfigurationMetaData`` does not set ``additionalProperties: false``.

        ``temp_sensor`` declares four ROS 2 parameters in the test fixture,
        which exercises the per-parameter emit path that #385 migrated.

        @verifies REQ_INTEROP_002
        """
        # The configurations endpoint reads the node's parameter service, which
        # returns 503 until it is ready just after discovery. Poll for it rather
        # than asserting 200 on a single early GET.
        body = self.poll_endpoint_until(
            '/apps/temp_sensor/configurations',
            timeout=PARAM_SERVICE_TIMEOUT,
        )

        items = body.get('items', [])
        self.assertGreater(
            len(items), 0,
            'Fixture broken: temp_sensor should declare ROS 2 parameters'
        )
        for item in items:
            self.assertNotIn(
                'x-medkit-source', item, f'Legacy flat key on item: {item}'
            )
            self.assertNotIn(
                'x-medkit-node', item, f'Legacy flat key on item: {item}'
            )

        parameters = body.get('x-medkit', {}).get('parameters', [])
        self.assertGreater(len(parameters), 0)
        for param in parameters:
            self.assertNotIn(
                'x-medkit-source', param, f'Legacy flat key on parameter: {param}'
            )
            self.assertNotIn(
                'x-medkit-node', param, f'Legacy flat key on parameter: {param}'
            )
            self.assertIn('x-medkit', param)
            self.assertEqual(param['x-medkit'].get('source'), 'temp_sensor')
            self.assertIn('node', param['x-medkit'])

    def test_substitute_path_params_uses_discovered_entity_id(self):
        """Helper resolves placeholders against the entity_map."""
        uri = _substitute_path_params(
            '/areas/{area_id}/components',
            {'areas': 'my_area', 'components': 'my_comp'},
            resource_id='resX',
        )
        self.assertEqual(uri, '/areas/my_area/components')

    def test_substitute_path_params_resource_id_substituted(self):
        """Non-entity placeholders use the resource_id argument."""
        uri = _substitute_path_params(
            '/updates/{id}/status', {}, resource_id='pkg-7'
        )
        self.assertEqual(uri, '/updates/pkg-7/status')

    def test_substitute_path_params_raises_on_missing_entity_type(self):
        """Missing entity type must raise rather than substitute a fake.

        The previous fallback substituted 'test_entity' for unknown
        types, producing 404s the validation loop silently dropped.
        Now the helper forces callers to handle the missing type
        explicitly (skip + report).
        """
        with self.assertRaises(_MissingEntityType):
            _substitute_path_params('/functions/{function_id}/data', {})
        with self.assertRaises(_MissingEntityType):
            _substitute_path_params(
                '/areas/{area_id}/components', {'components': 'c1'}
            )


@launch_testing.post_shutdown_test()
class TestExitCodes(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with code {info.returncode}',
            )
