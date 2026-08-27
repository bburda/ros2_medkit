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

"""Boot the gateway from the SHIPPED config file and check the posture.

Every other test in this suite builds its parameters inline, which is fine for
testing behaviour but means nothing here ever loaded
``config/gateway_params.yaml``. That left the branch in an odd state: the file
could be reverted to ``auth.enabled: false`` and the whole suite would stay
green, because each test supplies the values it needs itself.

This file closes that gap. It launches with ``--params-file`` pointing at the
installed copy of the shipped config, overriding only the port, the signing
secret and the client - the three things a real deployment must supply and the
file deliberately leaves empty - and then checks that what ships is closed.

TLS is turned off here and only here. The shipped file has it on, which is
correct, but a certificate is a deployment artefact and generating one would
test the certificate rather than the posture. ``test_tls_protocol_floor``
covers TLS itself against real handshakes.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import os
import socket
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.coverage import get_coverage_env

PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{PORT}{API_BASE_PATH}'

SHIPPED_PARAMS = os.path.join(
    get_package_share_directory('ros2_medkit_gateway'), 'config', 'gateway_params.yaml'
)

JWT_SECRET = 'shipped_defaults_integration_secret_key_0123456789'
CLIENT_ID = 'shipped'
CLIENT_SECRET = 'shipped_client_secret'


@pytest.mark.launch_test
def generate_test_description():
    """Launch the gateway with the shipped params file, plus the required secrets."""
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[
            SHIPPED_PARAMS,
            {
                'server.host': '127.0.0.1',
                'server.port': PORT,
                'refresh_interval_ms': 1000,
                # A certificate is a deployment artefact, not part of the
                # posture under test here.
                'server.tls.enabled': False,
                # What the shipped file leaves empty on purpose.
                'auth.jwt_secret': JWT_SECRET,
                'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
            },
        ],
        additional_env=dict(get_coverage_env()),
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


def _wait_listening(port, timeout=90.0):
    """Block until the gateway accepts a connection.

    launch_testing starts the tests when the process is spawned, not when it is
    serving. Without this the first request is refused by a gateway that simply
    has not opened its socket yet, which looks nothing like the posture this
    file is about.

    The timeout is generous because this gateway loads the full shipped config,
    which does more work at startup than the inline parameter sets the rest of
    the suite uses.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=2):
                return
        except OSError:
            time.sleep(0.25)
    raise AssertionError(f'gateway on port {port} never started listening within {timeout}s')


class TestShippedDefaults(unittest.TestCase):
    """What config/gateway_params.yaml actually produces."""

    @classmethod
    def setUpClass(cls):
        _wait_listening(PORT)
        resp = requests.post(
            f'{BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        )
        assert resp.status_code == 200, f'token request failed: {resp.status_code} {resp.text}'
        cls.auth = {'Authorization': f'Bearer {resp.json()["access_token"]}'}

    def test_01_the_shipped_file_turns_authentication_on(self):
        """Reverting auth.enabled in the shipped file must fail here.

        No other test would notice: they all pass auth.enabled themselves.
        """
        resp = requests.get(f'{BASE_URL}/areas', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            'the shipped config served /areas to an anonymous caller'
        )

    def test_02_the_shipped_file_covers_reads_not_just_writes(self):
        """Pins require_auth_for: "all" as the shipped value.

        Under "write" every one of these answers 200 without a credential.
        """
        for path in ('/', '/areas', '/components', '/apps', '/functions', '/version-info'):
            with self.subTest(path=path):
                resp = requests.get(f'{BASE_URL}{path}', timeout=15)
                self.assertIn(resp.status_code, (401, 403))

    def test_03_health_stays_reachable(self):
        """The exemption has to survive in the shipped file too.

        Without it a container supervisor cannot probe the gateway and every
        deployment restart-loops, so this is as much a part of the shipped
        posture as the refusals above.
        """
        resp = requests.get(f'{BASE_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 200)

    def test_04_a_configured_client_still_works(self):
        """The mirror: a gateway that refused everyone would pass the rest."""
        resp = requests.get(f'{BASE_URL}/areas', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200)

    def test_05_the_shipped_file_is_the_one_under_test(self):
        """Guard against this test silently drifting off the real file.

        If the installed config stops declaring the values this file exists to
        check, the assertions above would still pass for the wrong reason.
        """
        with open(SHIPPED_PARAMS, encoding='utf-8') as handle:
            text = handle.read()
        self.assertIn('require_auth_for: "all"', text)
        self.assertIn('enabled: true', text)


@launch_testing.post_shutdown_test()
class TestShippedDefaultsShutdown(unittest.TestCase):
    """Gateway exits cleanly."""

    def test_exit_codes(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=gateway_node
        )
