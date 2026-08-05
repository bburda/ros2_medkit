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

"""A gateway without a LockManager does not sell locking in its document.

``locking.enabled`` is a deployment setting: ``GatewayNode`` builds the
``LockManager`` only when it is set, and with it off
``HandlerContext::validate_lock_access`` returns success without reading
``X-Client-Id``. No write can then be refused for a lock.

``RouteEntry::lock_guarded()`` is a registration-time declaration and cannot
see that. It used to publish its three pieces - the
``x-medkit-lock-guarded`` marker, the ``X-Client-Id`` parameter and the 409 -
on every gateway, so a gateway whose own root reported
``capabilities.locking: false`` still described serialised writes on 44
operations. A client trusting that description gets silent lost updates.

The locking-*on* half of this pair is
``test_openapi_contract.test.py::test_lock_guarded_set_matches_the_handlers``,
which runs the shipped default. This file is the only fixture in the suite
with locking off, which is why the claim needs its own launch.

No ``@verifies``: this is a claim about the generated document matching the
deployment, not about a SOVD locking endpoint behaving as the spec requires.
``test_locking.test.py`` carries the endpoint requirements.
"""

import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_gateway_node,
    full_feature_gateway_params,
)

LOCK_PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{LOCK_PORT}{API_BASE_PATH}'

HTTP_METHODS = {'get', 'post', 'put', 'delete', 'patch'}

# The lock CRUD itself. These routes are served whatever `locking.enabled`
# says - they answer 501 when it is off, which they already declare - and they
# declare an `X-Client-Id` of their own that has nothing to do with
# `lock_guarded()`. Matched on the path, not on the marker, precisely because
# the marker is what this file expects to be absent.
LOCK_COLLECTION_SUFFIXES = ('/locks', '/locks/{lock_id}')

_SCRIPTS_DIR = tempfile.mkdtemp(prefix='medkit-nolock-')


@pytest.mark.launch_test
def generate_test_description():
    """Launch one full-feature gateway with locking turned off."""
    params = dict(full_feature_gateway_params(_SCRIPTS_DIR))
    # `full_feature_gateway_params` turns every optional gate on, which is what
    # gives this file the maximal route surface. Locking is the one gate it
    # exists to turn back off - spread first, override second, so a future
    # change to that helper cannot silently re-enable it here.
    params['locking.enabled'] = False

    gateway_node = create_gateway_node(
        port=LOCK_PORT,
        name='gateway_no_locking',
        extra_params={'server.host': '127.0.0.1', **params},
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


def _is_lock_collection(path):
    """Report whether `path` is one of the lock CRUD routes."""
    return path.endswith(LOCK_COLLECTION_SUFFIXES)


class TestLockingDisabledContract(unittest.TestCase):
    """With no LockManager, nothing in the document promises one."""

    @classmethod
    def setUpClass(cls):
        """Wait for the gateway, then read its root and its document."""
        cls.session = requests.Session()
        deadline = time.time() + 60.0
        last = None
        while time.time() < deadline:
            try:
                response = cls.session.get(f'{BASE_URL}/health', timeout=5)
                if response.status_code == 200:
                    break
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        else:
            raise AssertionError(f'gateway not ready within 60s (last: {last})')

        cls.root = cls.session.get(BASE_URL, timeout=15).json()
        cls.spec = cls.session.get(f'{BASE_URL}/docs', timeout=15).json()

    @classmethod
    def tearDownClass(cls):
        cls.session.close()

    def operations(self):
        """Yield (path, method, operation) for every documented operation."""
        for path, item in self.spec['paths'].items():
            for method, operation in item.items():
                if method in HTTP_METHODS:
                    yield path, method, operation

    def test_the_gateway_really_has_no_lock_manager(self):
        """Grounding, in the document's own terms and on the wire.

        Every assertion below is about the absence of something, and absence
        passes for free on the wrong gateway. This is what makes the rest mean
        anything: the root reports the same `get_lock_manager() != nullptr`
        the registry now reads, and a lock acquired against a *real* entity
        comes back 501 rather than 404 - which is the handler's
        `check_locking_enabled` speaking, not a missing route.
        """
        self.assertFalse(self.root['capabilities']['locking'])

        apps = self.session.get(f'{BASE_URL}/apps', timeout=15).json()
        items = apps.get('items', [])
        self.assertTrue(items, 'no apps discovered; the probe below would 404 '
                               'for the wrong reason')
        entity_id = items[0]['id']
        response = self.session.post(
            f'{BASE_URL}/apps/{entity_id}/locks',
            json={'lock_expiration': 60},
            headers={'X-Client-Id': 'locking-disabled-probe'},
            timeout=15,
        )
        self.assertEqual(
            response.status_code, 501,
            f'POST /apps/{entity_id}/locks answered {response.status_code}, so '
            f'this gateway is not the locking-off gateway: {response.text[:200]}')

    def test_no_operation_carries_the_lock_guarded_marker(self):
        """The marker says a 409 can arrive. None can."""
        marked = sorted(f'{m.upper()} {p}' for p, m, op in self.operations()
                        if op.get('x-medkit-lock-guarded'))
        self.assertEqual(
            marked, [],
            f'{len(marked)} operations publish x-medkit-lock-guarded on a '
            f'gateway with no LockManager: {marked[:20]}')

    def test_no_operation_outside_the_lock_crud_reads_a_client_id(self):
        """`X-Client-Id` is only declared where it is still read.

        The lock CRUD keeps its own - those routes are served and answer 501,
        and their parameter was never `lock_guarded()`'s. Everywhere else the
        header is now ignored, and a parameter description promising that a
        competing client is answered 409 would be the most directly misleading
        piece of the three.
        """
        offenders = sorted(
            f'{m.upper()} {p}' for p, m, op in self.operations()
            if not _is_lock_collection(p)
            and any(q.get('name') == 'X-Client-Id'
                    for q in op.get('parameters', []))
        )
        self.assertEqual(
            offenders, [],
            f'{len(offenders)} operations outside the lock CRUD still declare '
            f'X-Client-Id: {offenders[:20]}')

    def test_no_operation_declares_a_lock_conflict(self):
        """A 409 that travels with `X-Client-Id` is a lock 409.

        The three pieces come from one call, so this is the shape the defect
        had: 44 operations declaring a 409 *and* the header that decides it.
        Other 409s survive and should - a script execution already running, an
        update in the wrong state, a lifecycle transition in flight - and this
        is keyed so as not to touch them.
        """
        offenders = sorted(
            f'{m.upper()} {p}' for p, m, op in self.operations()
            if not _is_lock_collection(p)
            and '409' in op.get('responses', {})
            and any(q.get('name') == 'X-Client-Id'
                    for q in op.get('parameters', []))
        )
        self.assertEqual(
            offenders, [],
            f'{len(offenders)} operations declare a lock conflict: '
            f'{offenders[:20]}')

    def test_the_lock_crud_is_still_documented_and_still_served(self):
        """Anti-vacuous, and the boundary of the change.

        Turning locking off removes the *claims other routes made about it*,
        not the lock endpoints. If this fixture had simply dropped every lock
        route, all three assertions above would pass having checked nothing.
        """
        lock_paths = sorted({p for p, _, _ in self.operations()
                             if _is_lock_collection(p)})
        self.assertTrue(
            lock_paths, 'the lock CRUD is absent from the document')
        # Their own X-Client-Id survived the filter that removed the marker's.
        acquire = [op for p, m, op in self.operations()
                   if _is_lock_collection(p) and m == 'post']
        self.assertTrue(acquire, 'no lock acquire operation documented')
        self.assertTrue(
            any(q.get('name') == 'X-Client-Id'
                for op in acquire for q in op.get('parameters', [])),
            'the lock CRUD lost the X-Client-Id it declares itself')
        # And the surface really is the full one, so the loops above ran over
        # the gated routes rather than the handful of always-on endpoints.
        self.assertGreater(len(list(self.operations())), 200)


@launch_testing.post_shutdown_test()
class TestLockingDisabledShutdown(unittest.TestCase):
    """The gateway exits cleanly."""

    def test_exit_codes(self, proc_info):
        """Check that the gateway process exited with an allowed code."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
