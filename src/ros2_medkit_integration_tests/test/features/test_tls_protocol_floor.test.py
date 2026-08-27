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

"""Check the TLS protocol floor and client-certificate verification.

Both are driven by a real client against a real gateway.

Both properties are about what happens during the TLS handshake, before any
HTTP request exists, so they cannot be observed from Python's requests or from
a unit test that checks a setter was called. Every assertion here comes from
``openssl s_client`` completing or failing a handshake at a pinned version.

The client is run with ``-cipher ALL:@SECLEVEL=0``. Without it a modern
OpenSSL client refuses to OFFER TLS 1.0/1.1 on its own, and the test would pass
while proving nothing about the server: it has to be the server that says no.

Two gateways run side by side, one with min_version 1.2 and one with 1.3, so
the floor is shown to MOVE with the setting rather than happening to sit where
OpenSSL's own default put it.

@verifies REQ_INTEROP_086
"""

import os
import re
import shutil
import socket
import subprocess
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

PORT_TLS12 = get_test_port(0)
PORT_TLS13 = get_test_port(1)
PORT_MTLS = get_test_port(2)

_CERT_DIR = tempfile.mkdtemp(prefix='medkit_tls_floor_')


def _run(*args):
    subprocess.run(args, check=True, capture_output=True)


def _make_ca(name):
    """Build a CA key plus its self-signed certificate."""
    key = os.path.join(_CERT_DIR, f'{name}-ca-key.pem')
    crt = os.path.join(_CERT_DIR, f'{name}-ca.pem')
    _run('openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
         '-keyout', key, '-out', crt, '-days', '1',
         '-subj', f'/CN=medkit-test-{name}-ca')
    return key, crt


def _make_leaf(name, ca_key, ca_crt, cn):
    """Build a leaf key and certificate signed by the given CA."""
    key = os.path.join(_CERT_DIR, f'{name}-key.pem')
    csr = os.path.join(_CERT_DIR, f'{name}.csr')
    crt = os.path.join(_CERT_DIR, f'{name}.pem')
    _run('openssl', 'req', '-newkey', 'rsa:2048', '-nodes',
         '-keyout', key, '-out', csr, '-subj', f'/CN={cn}')
    _run('openssl', 'x509', '-req', '-in', csr, '-CA', ca_crt, '-CAkey', ca_key,
         '-CAcreateserial', '-out', crt, '-days', '1')
    return key, crt


# The CA that signs the server certificate and the legitimate client.
CA_KEY, CA_CRT = _make_ca('trusted')
SRV_KEY, SRV_CRT = _make_leaf('server', CA_KEY, CA_CRT, 'localhost')
CLI_KEY, CLI_CRT = _make_leaf('client', CA_KEY, CA_CRT, 'medkit-test-client')

# A second CA the gateway was never told about, for the certificate that is
# well-formed and correctly signed but by the wrong authority.
ROGUE_KEY, ROGUE_CRT = _make_ca('rogue')
ROGUE_CLI_KEY, ROGUE_CLI_CRT = _make_leaf('rogue-client', ROGUE_KEY, ROGUE_CRT, 'rogue')


def _tls_params(port, min_version, ca_file=''):
    params = {
        'server.host': '127.0.0.1',
        'server.tls.enabled': True,
        'server.tls.cert_file': SRV_CRT,
        'server.tls.key_file': SRV_KEY,
        'server.tls.min_version': min_version,
        # Auth off: this file is about the handshake, and a 401 would arrive
        # long after the point under test has already been decided.
        'auth.enabled': False,
    }
    if ca_file:
        params['server.tls.ca_file'] = ca_file
    return params


@pytest.mark.launch_test
def generate_test_description():
    """Three gateways: floor at 1.2, floor at 1.3, and one demanding a client cert."""
    nodes = [
        create_gateway_node(port=PORT_TLS12, name='gateway_tls12',
                            extra_params=_tls_params(PORT_TLS12, '1.2')),
        create_gateway_node(port=PORT_TLS13, name='gateway_tls13',
                            extra_params=_tls_params(PORT_TLS13, '1.3')),
        create_gateway_node(port=PORT_MTLS, name='gateway_mtls',
                            extra_params=_tls_params(PORT_MTLS, '1.2', ca_file=CA_CRT)),
    ]
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()]), {
        'gateway_tls12': nodes[0],
        'gateway_tls13': nodes[1],
        'gateway_mtls': nodes[2],
    }


def _handshake(port, version, client_cert=None, client_key=None, timeout=20):
    """Attempt one handshake. True only when a cipher was actually agreed.

    `openssl s_client` exits 0 in cases where no session was established, and
    it prints the protocol it ATTEMPTED whether or not the server accepted it.
    "Cipher is (NONE)" is the reliable tell for a handshake that did not
    complete, so that is what is read here rather than the exit status.
    """
    cmd = ['openssl', 's_client', f'-{version}',
           '-cipher', 'ALL:@SECLEVEL=0',
           '-connect', f'127.0.0.1:{port}']
    if client_cert:
        cmd += ['-cert', client_cert, '-key', client_key]
    try:
        proc = subprocess.run(cmd, input=b'', capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False
    out = (proc.stdout + proc.stderr).decode(errors='replace')

    # "Cipher is <name>" is NOT proof that the handshake completed, and reading
    # it that way is how an earlier version of this file reported mutual TLS as
    # broken when it was working. Under TLS 1.2 the cipher suite is agreed
    # before the client certificate is examined, so a server that then rejects
    # the certificate still leaves a cipher name in the output, followed by a
    # fatal alert. Verified by hand against this gateway: a client with no
    # certificate printed "Cipher is ECDHE-RSA-AES256-GCM-SHA384" AND
    # "sslv3 alert handshake failure", while curl against the same endpoint got
    # no HTTP response at all.
    #
    # So a fatal alert is the signal, and "Cipher is (NONE)" covers the case
    # where the version itself was refused before any suite was picked.
    if 'Cipher is (NONE)' in out:
        return False
    if re.search(r'alert (handshake failure|protocol version|certificate|unknown ca)', out):
        return False
    return 'Cipher is ' in out


def _wait_listening(port, timeout=60.0):
    """Block until the port accepts a TCP connection.

    launch_testing starts the tests as soon as the processes are spawned, not
    when they are serving, and a gateway that is not listening yet refuses
    every connection. That looks identical to "the server rejected this
    handshake", so without this gate the refusal assertions pass for the wrong
    reason and the acceptance assertions fail at random. Observed directly:
    the same file reported two failures, then two, then one, across three runs.

    TCP only, deliberately. A TLS handshake cannot be the readiness probe here
    because on the mutual-TLS gateway a probe without a client certificate is
    supposed to fail.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=2):
                return
        except OSError:
            time.sleep(0.25)
    raise AssertionError(f'gateway on port {port} never started listening within {timeout}s')


class TestTlsProtocolFloor(unittest.TestCase):
    """The floor moves with min_version, and it is the server that enforces it."""

    @classmethod
    def setUpClass(cls):
        _wait_listening(PORT_TLS12)
        _wait_listening(PORT_TLS13)

    def test_01_floor_12_accepts_12_and_13(self):
        """The mirror of the refusals below.

        Without this, a gateway that refused every version would pass the
        whole file while serving nobody.
        """
        self.assertTrue(_handshake(PORT_TLS12, 'tls1_2'), 'TLS 1.2 must be accepted at floor 1.2')
        self.assertTrue(_handshake(PORT_TLS12, 'tls1_3'), 'TLS 1.3 must be accepted at floor 1.2')

    def test_02_floor_12_refuses_11_and_10(self):
        """SOVD requires TLS 1.2 as the minimum, so 1.1 and 1.0 must not connect.

        The vendored cpp-httplib asks OpenSSL for a floor of TLS 1.1
        (SSL_CTX_set_min_proto_version(ctx_, TLS1_1_VERSION)), so without the
        gateway setting its own floor this is the version that decides whether
        we comply, and it is not a value this project chose.
        """
        self.assertFalse(_handshake(PORT_TLS12, 'tls1_1'), 'TLS 1.1 must be refused at floor 1.2')
        self.assertFalse(_handshake(PORT_TLS12, 'tls1'), 'TLS 1.0 must be refused at floor 1.2')

    def test_03_floor_13_refuses_12(self):
        """The test that fails if min_version is inert.

        A gateway configured for 1.3 that still completes a 1.2 handshake is
        exactly the state this branch shipped before: the value was read,
        logged, and then ignored. TLS 1.2 is accepted by the OTHER gateway in
        this same launch, so a failure here cannot be blamed on the client or
        on the certificate.
        """
        self.assertFalse(_handshake(PORT_TLS13, 'tls1_2'), 'TLS 1.2 must be refused at floor 1.3')
        self.assertFalse(_handshake(PORT_TLS13, 'tls1_1'), 'TLS 1.1 must be refused at floor 1.3')

    def test_04_floor_13_accepts_13(self):
        self.assertTrue(_handshake(PORT_TLS13, 'tls1_3'), 'TLS 1.3 must be accepted at floor 1.3')


class TestMutualTls(unittest.TestCase):
    """With ca_file set, a client certificate is required and verified."""

    @classmethod
    def setUpClass(cls):
        _wait_listening(PORT_MTLS)
        _wait_listening(PORT_TLS12)

    def test_05_no_client_certificate_is_refused(self):
        """ca_file set means SSL_VERIFY_FAIL_IF_NO_PEER_CERT: no cert, no session."""
        self.assertFalse(
            _handshake(PORT_MTLS, 'tls1_2'),
            'a client presenting no certificate must not complete the handshake'
        )

    def test_06_a_certificate_from_the_configured_ca_is_accepted(self):
        self.assertTrue(
            _handshake(PORT_MTLS, 'tls1_2', client_cert=CLI_CRT, client_key=CLI_KEY),
            'a client certificate signed by the configured CA must be accepted'
        )

    def test_07_a_certificate_from_another_ca_is_refused(self):
        """Well-formed and correctly signed, but by an authority we never trusted.

        This separates "verification is on" from "any certificate will do",
        which test_05 alone cannot.
        """
        self.assertFalse(
            _handshake(PORT_MTLS, 'tls1_2', client_cert=ROGUE_CLI_CRT, client_key=ROGUE_CLI_KEY),
            'a client certificate from an unconfigured CA must be refused'
        )

    def test_08_a_gateway_without_ca_file_does_not_demand_one(self):
        """The default stays server-only TLS.

        SOVD authenticates with bearer tokens, so requiring a client
        certificate by default would put us outside the spec. mTLS is opt-in
        and this pins that it is.
        """
        self.assertTrue(
            _handshake(PORT_TLS12, 'tls1_2'),
            'a gateway with no ca_file must still serve a client that has no certificate'
        )


@launch_testing.post_shutdown_test()
class TestTlsFloorShutdown(unittest.TestCase):
    """All three gateways exit cleanly."""

    def test_exit_codes(self, proc_info, gateway_tls12, gateway_tls13, gateway_mtls):
        for proc in (gateway_tls12, gateway_tls13, gateway_mtls):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(_CERT_DIR, ignore_errors=True)
