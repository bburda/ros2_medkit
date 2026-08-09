#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""
Secured OPC-UA Alarms & Conditions integration test (issue #477).

This is the success-path counterpart to the unit tests around
``apply_security_config()``: instead of only exercising the parsers, it
negotiates a real encrypted SecureChannel against a live open62541 server and
drives the native A&C bridge through it.

Topology (all on the loopback, ephemeral ports):

    test_alarm_server --secure   (Basic256Sha256 Sign + SignAndEncrypt,
       |                          username/password, anonymous DISABLED)
       | encrypted SecureChannel + UserName token
    gateway_node (ros2_medkit_opcua)  ->  fault_manager_node

POSITIVE (good credentials):
  * SOVD x-plc-status reports connected == true (channel established).
  * SOVD x-plc-data returns the Tank.Level value (a real read over the
    encrypted channel, not just an event subscription).
  * Firing an alarm over the server's stdin surfaces a CONFIRMED fault in
    /api/v1/faults (secured A&C event subscription end-to-end).
  * The server logs ``SECURE_SESSION ... securityMode=3`` (SignAndEncrypt),
    proving the channel is encrypted and not None.

NEGATIVE (wrong password):
  * The gateway never reaches connected == true: fail-closed.

Skips with the CTest convention (exit 77) unless
``ROS2_MEDKIT_OPCUA_SECURE_REQUIRE`` is set, mirroring the asyncua smoke-test
skip. When the flag is set, a missing prerequisite is a hard failure so a CI
job that loses a dependency cannot silently bypass the secured check.

Usage: test_opcua_secured.test.py <test_alarm_server> <gen_test_certs.sh>
"""

import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

CTEST_SKIP = 77
USERNAME = 'medkit'
PASSWORD = 'secret'
WRONG_PASSWORD = 'definitely-not-the-password'
APP_URI_SERVER = 'urn:test:alarms:server'
APP_URI_CLIENT = 'urn:selfpatch:medkit:opcua-client'

ALARM_CODE = 'PLC_OVERPRESSURE'
# Source-level catch-all code a non-condition event would be mis-mapped to if the
# poller's ConditionId guard regressed.
NONCOND_CODE = 'PLC_ALARM'


def require_flag():
    """Return True when the suite must run for real (CI) instead of skipping."""
    return os.environ.get('ROS2_MEDKIT_OPCUA_SECURE_REQUIRE', '0') not in ('', '0')


def skip_or_fail(reason):
    """Return the CTest skip code, or fail hard when the run is required."""
    if require_flag():
        print(f'REQUIRED but prerequisite unmet: {reason}', file=sys.stderr)
        return 1
    print(f'SKIP (set ROS2_MEDKIT_OPCUA_SECURE_REQUIRE to force): {reason}')
    return CTEST_SKIP


def free_port():
    """Grab an OS-assigned free TCP port on the loopback and release it."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def find_plugin():
    """Locate libros2_medkit_opcua_plugin.so under AMENT_PREFIX_PATH."""
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        for root, _dirs, files in os.walk(prefix):
            if 'libros2_medkit_opcua_plugin.so' in files:
                return os.path.join(root, 'libros2_medkit_opcua_plugin.so')
    return None


def have_executable(package, executable):
    """Return True when ``ros2 run <package> <executable>`` is resolvable."""
    try:
        out = subprocess.run(
            ['ros2', 'pkg', 'executables', package],
            capture_output=True, text=True, timeout=30, check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return f'{package} {executable}' in out.stdout


def http_json(url, timeout=2):
    """GET <url> and parse JSON, or return None on any failure."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except (urllib.error.URLError, OSError, ValueError):
        return None


def wait_json(url, predicate, deadline=60, period=2.0):
    """Poll <url> until predicate(json) is true; return the final json or None."""
    last = None
    for _ in range(int(deadline / period) + 1):
        last = http_json(url)
        if last is not None:
            try:
                if predicate(last):
                    return last
            except (KeyError, TypeError, ValueError):
                pass
        time.sleep(period)
    return last


def wait_log(path, needle, deadline=30, period=0.5):
    """Poll a log file until <needle> appears; return True/False."""
    for _ in range(int(deadline / period) + 1):
        try:
            if needle in Path(path).read_text(errors='replace'):
                return True
        except OSError:
            pass
        time.sleep(period)
    return False


def write_node_map(path):
    """Map the readable Tank.Level node + the 3 alarm sources into one entity."""
    path.write_text(
        'area_id: plc_systems\n'
        'component_id: alarm_test_runtime\n'
        'nodes:\n'
        '  - node_id: "ns=2;s=Tank.Level"\n'
        '    entity_id: tank_process\n'
        '    data_name: tank_level\n'
        '    data_type: float\n'
        'event_alarms:\n'
        '  - alarm_source: "ns=2;s=Alarms.Overpressure"\n'
        '    entity_id: tank_process\n'
        '    fault_code: PLC_OVERPRESSURE\n'
        '  - alarm_source: "ns=2;s=Alarms.Overheat"\n'
        '    entity_id: tank_process\n'
        '    fault_code: PLC_OVERHEAT\n'
        '  - alarm_source: "ns=2;s=Alarms.SensorLost"\n'
        '    entity_id: tank_process\n'
        '    fault_code: PLC_SENSOR_LOST\n'
    )


def write_catchall_node_map(path):
    """
    Node map with a single explicit event_alarms catch-all on the Server object.

    Every real AlarmConditionType event on i=2253 maps to PLC_ALARM via the
    source-level fault_code. A non-condition BaseEventType on the same
    EventNotifier resolves its ConditionId to null and must be dropped by the
    poller's guard rather than mapped to PLC_ALARM. Only this one source is
    mapped, so a real condition (which is also a notifier of i=2253) surfaces
    exactly one PLC_ALARM fault - no second monitored item, no fault_code race.
    """
    path.write_text(
        'area_id: plc_systems\n'
        'component_id: alarm_test_runtime\n'
        'nodes:\n'
        '  - node_id: "ns=2;s=Tank.Level"\n'
        '    entity_id: tank_process\n'
        '    data_name: tank_level\n'
        '    data_type: float\n'
        'event_alarms:\n'
        '  - alarm_source: "ns=0;i=2253"\n'
        '    entity_id: tank_process\n'
        '    fault_code: PLC_ALARM\n'
    )


def write_gateway_params(path, *, port, plugin, server_port, node_map, manifest,
                         certs, password):
    """Render a gateway params file pointing the opcua plugin at the secure server."""
    path.write_text(
        'ros2_medkit_gateway:\n'
        '  ros__parameters:\n'
        '    server:\n'
        '      host: "127.0.0.1"\n'
        f'      port: {port}\n'
        '    plugins: ["opcua"]\n'
        f'    plugins.opcua.path: "{plugin}"\n'
        f'    plugins.opcua.endpoint_url: "opc.tcp://127.0.0.1:{server_port}"\n'
        f'    plugins.opcua.node_map_path: "{node_map}"\n'
        '    plugins.opcua.poll_interval_ms: 500\n'
        '    plugins.opcua.security_policy: "Basic256Sha256"\n'
        '    plugins.opcua.security_mode: "SignAndEncrypt"\n'
        f'    plugins.opcua.client_cert_path: "{certs}/client_cert.der"\n'
        f'    plugins.opcua.client_key_path: "{certs}/client_key.pem"\n'
        f'    plugins.opcua.application_uri: "{APP_URI_CLIENT}"\n'
        f'    plugins.opcua.trust_list_paths: ["{certs}/server_cert.der"]\n'
        '    plugins.opcua.reject_untrusted: true\n'
        '    plugins.opcua.user_auth_mode: "UsernamePassword"\n'
        f'    plugins.opcua.username: "{USERNAME}"\n'
        f'    plugins.opcua.password: "{password}"\n'
        '    discovery.mode: "hybrid"\n'
        f'    discovery.manifest_path: "{manifest}"\n'
        '    discovery.manifest_strict_validation: false\n'
    )


def start_gateway(params_file, log_path, env):
    """Launch a gateway_node bound to the given params file."""
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
         '--ros-args', '--params-file', str(params_file)],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
    )
    proc._log = log  # keep handle alive for close on teardown
    return proc


def send_cmd(server, server_log, cmd):
    """
    Write one alarm-CLI command, reporting a dead server instead of EPIPE.

    A bare write to an exited server raises BrokenPipeError from whichever
    command comes next, naming neither the server nor a reason. Check first and
    dump the server log, which carries the UA_Server_run status.

    Returns True on success, False if the server is gone (caller should fail).
    """
    def report():
        rc = server.returncode
        print(f'FAIL: alarm server gone (rc={rc}) before {cmd!r}', file=sys.stderr)
        print('server log:\n' + Path(server_log).read_text(errors='replace'),
              file=sys.stderr)
        return False

    if server.poll() is not None:
        return report()
    try:
        server.stdin.write(cmd + '\n')
        server.stdin.flush()
    except (BrokenPipeError, ValueError):
        # The poll() above is a point-in-time check: the server can still exit
        # between it and the flush. Report the same way instead of letting the
        # exception escape, which is the failure mode this helper exists to
        # remove. ValueError covers a stdin already closed on teardown.
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        return report()
    return True


def terminate(proc):
    """
    Best-effort SIGTERM then SIGKILL of a child process *group*.

    The node processes are launched through ``ros2 run``, so the direct child
    is the CLI wrapper and the node itself is a grandchild. Signalling only the
    wrapper leaves the node reparented to init, still holding the DDS domain
    CTest assigned this run from the ros2_medkit_opcua pool, so the leaked
    node then joins a later run's graph. Every process *passed to this
    function* is started with
    ``start_new_session=True``, which puts it in its own process group, so the
    whole group can be signalled.

    That guarantee does not extend to processes this function never sees. A
    plain ``subprocess.run(['ros2', ...])`` call outside this function is not
    started with ``start_new_session=True`` and is not tracked here, so if it
    spawns a ros2cli daemon for itself, the daemon lands in this test script's
    own process group and terminate() cannot reach it. Such call sites need
    ``--no-daemon`` (or their own cleanup) instead of relying on this function.
    """
    if proc is None:
        return
    if proc.returncode is not None:
        # Already reaped by an earlier proc.poll() in the readiness loop. The OS can have
        # recycled this PID since then, so looking up and signalling a process group for it
        # would risk hitting an unrelated process instead of a no-op - skip both.
        pgid = None
    else:
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pgid = None

    def signal_group(sig):
        if pgid is None:
            return
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass

    signal_group(signal.SIGTERM)
    proc.terminate()
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        signal_group(signal.SIGKILL)
        proc.kill()
        proc.wait()
    # The wrapper can exit before the node it spawned does. SIGKILL the group
    # unconditionally afterwards so nothing outlives this call.
    signal_group(signal.SIGKILL)
    log = getattr(proc, '_log', None)
    if log is not None:
        log.close()


def main():
    if len(sys.argv) < 3:
        print('usage: test_opcua_secured.test.py <test_alarm_server> <gen_test_certs.sh>',
              file=sys.stderr)
        return 2
    server_bin = Path(sys.argv[1]).resolve()
    gen_certs = Path(sys.argv[2]).resolve()

    # ROS_DOMAIN_ID comes from the domain wrapper this test is registered behind
    # (see ros2_medkit_opcua's CMakeLists.txt and ROS2MedkitTestDomain.cmake),
    # which holds it for as long as this process runs. A missing value is a
    # wiring bug, not something to paper over with a literal, so fail loudly
    # instead of guessing a domain.
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    if not ros_domain_id:
        print('ROS_DOMAIN_ID is not set: this test must be launched by CTest, which '
              'runs it behind medkit_run_with_domain.py', file=sys.stderr)
        return 1

    # --- Prerequisite gating (skip 77 unless required) ---------------------
    if shutil.which('openssl') is None:
        return skip_or_fail('openssl CLI not available')
    if shutil.which('ros2') is None:
        return skip_or_fail('ros2 CLI not available')
    if not (server_bin.is_file() and os.access(server_bin, os.X_OK)):
        return skip_or_fail(f'secure fixture missing: {server_bin}')
    if not gen_certs.is_file():
        return skip_or_fail(f'cert generator missing: {gen_certs}')
    plugin = find_plugin()
    if plugin is None:
        return skip_or_fail('libros2_medkit_opcua_plugin.so not found')
    if not have_executable('ros2_medkit_fault_manager', 'fault_manager_node'):
        return skip_or_fail('ros2_medkit_fault_manager/fault_manager_node not built')
    if not have_executable('ros2_medkit_gateway', 'gateway_node'):
        return skip_or_fail('ros2_medkit_gateway/gateway_node not built')

    workdir = Path(tempfile.mkdtemp(prefix='opcua_secured_'))
    certs = workdir / 'certs'
    server_log = workdir / 'server.log'
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)

    server = fault_mgr = gw_good = gw_bad = gw_catch = None
    try:
        # --- Certificates (regenerated every run, never committed) ---------
        subprocess.run(['bash', str(gen_certs), str(certs)], check=True,
                       capture_output=True, text=True)

        node_map = workdir / 'alarm_nodes.yaml'
        manifest = workdir / 'manifest.yaml'
        write_node_map(node_map)
        manifest.write_text('manifest_version: "1.0"\n')

        server_port = free_port()
        good_port = free_port()
        bad_port = free_port()

        # --- Secure server fixture (stdin pipe for the alarm CLI) ----------
        slog = open(server_log, 'w')
        server = subprocess.Popen(
            [str(server_bin), '--secure', '--port', str(server_port),
             '--cert', str(certs / 'server_cert.der'),
             '--key', str(certs / 'server_key.pem'),
             '--trust', str(certs / 'client_cert.der'),
             '--app-uri', APP_URI_SERVER,
             '--username', USERNAME, '--password', PASSWORD],
            stdin=subprocess.PIPE, stdout=slog, stderr=subprocess.STDOUT,
            text=True, start_new_session=True,
        )
        if not wait_log(server_log, 'READY ', deadline=20):
            print('secure server did not become READY', file=sys.stderr)
            print(Path(server_log).read_text(errors='replace'), file=sys.stderr)
            return 1

        # --- fault_manager_node --------------------------------------------
        # storage_type:=memory, as every other integration test does. The
        # production default is sqlite at /var/lib/ros2_medkit/faults.db, which
        # only a root process can create. Without this the node aborts on an
        # uncaught filesystem_error and the run fails much later, at the alarm
        # assertion, naming the wrong component.
        fault_mgr_log = workdir / 'fault_manager.log'
        fault_mgr = subprocess.Popen(
            ['ros2', 'run', 'ros2_medkit_fault_manager', 'fault_manager_node',
             '--ros-args', '-p', 'storage_type:=memory'],
            stdout=open(fault_mgr_log, 'w'),
            stderr=subprocess.STDOUT, env=env, start_new_session=True,
        )
        # Poll for the service the opcua plugin calls. Falling through without
        # it is not recoverable: every fault assertion below would then fail on
        # its own deadline and blame the alarm path instead of this node.
        fault_mgr_ready = False
        for _ in range(40):
            if fault_mgr.poll() is not None:
                break
            out = subprocess.run(['ros2', 'service', 'list', '--no-daemon'], capture_output=True,
                                 text=True, env=env, timeout=15, check=False)
            if '/fault_manager/report_fault' in out.stdout:
                fault_mgr_ready = True
                break
            time.sleep(0.5)
        # The service appearing is not on its own proof the node survived: an orphaned
        # fault_manager_node left behind by an earlier run of this same test can still be
        # sitting on this domain and answer for the service. Require both, and re-check
        # liveness after a settle.
        if fault_mgr_ready:
            time.sleep(2)
        if not fault_mgr_ready or fault_mgr.poll() is not None:
            reason = ('exited with code %r' % fault_mgr.poll() if fault_mgr.poll() is not None
                      else 'never offered /fault_manager/report_fault')
            print(f'FAIL: fault_manager_node {reason}', file=sys.stderr)
            print('fault_manager log:\n'
                  + fault_mgr_log.read_text(errors='replace'), file=sys.stderr)
            return 1

        # === POSITIVE: good credentials ====================================
        good_params = workdir / 'gateway_good.yaml'
        write_gateway_params(good_params, port=good_port, plugin=plugin,
                             server_port=server_port, node_map=node_map,
                             manifest=manifest, certs=certs, password=PASSWORD)
        gw_good = start_gateway(good_params, workdir / 'gateway_good.log', env)

        base = f'http://127.0.0.1:{good_port}/api/v1'
        status = wait_json(
            f'{base}/components/alarm_test_runtime/x-plc-status',
            lambda j: j.get('connected') is True, deadline=70)
        if not (status and status.get('connected') is True):
            print('FAIL: gateway did not connect over the secure channel', file=sys.stderr)
            print('server log:\n' + Path(server_log).read_text(errors='replace'), file=sys.stderr)
            gw_tail = Path(workdir / 'gateway_good.log').read_text(errors='replace')[-3000:]
            print('gateway log tail:\n' + gw_tail, file=sys.stderr)
            return 1
        print('  OK secured connect: x-plc-status connected == true')

        # Proof the channel is encrypted (SignAndEncrypt = securityMode 3),
        # emitted by the server on session activation. Fail hard on None.
        if not wait_log(server_log, 'securityMode=3', deadline=20):
            print('FAIL: server never logged an encrypted (SignAndEncrypt) session',
                  file=sys.stderr)
            print(Path(server_log).read_text(errors='replace'), file=sys.stderr)
            return 1
        if 'securityPolicyUri=http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256' \
                not in Path(server_log).read_text(errors='replace'):
            print('FAIL: negotiated SecurityPolicy was not Basic256Sha256', file=sys.stderr)
            print(Path(server_log).read_text(errors='replace'), file=sys.stderr)
            return 1
        print('  OK encrypted channel: Basic256Sha256 securityMode=3 (SignAndEncrypt)')

        # Secured read: Tank.Level value over the encrypted channel.
        data = wait_json(
            f'{base}/apps/tank_process/x-plc-data',
            lambda j: any(i.get('name') == 'tank_level' and i.get('value') is not None
                          for i in j.get('items', [])), deadline=30)
        value = None
        if data:
            value = next((i.get('value') for i in data.get('items', [])
                          if i.get('name') == 'tank_level'), None)
        if value is None:
            print('FAIL: secured tag read returned no value', file=sys.stderr)
            print(json.dumps(data), file=sys.stderr)
            return 1
        print(f'  OK secured read: tank_level = {value}')

        # Secured A&C: fire an alarm over the server stdin -> CONFIRMED fault.
        if not send_cmd(server, server_log, 'fire Overpressure 750'):
            return 1
        # Generous deadline for slow CI runners: the alarm has already fired on
        # the server, but report -> fault_manager -> REST propagation can take
        # a while under load. A longer deadline only slows the failure path.
        faults = wait_json(
            f'{base}/faults',
            lambda j: any(i.get('fault_code') == ALARM_CODE and i.get('status') == 'CONFIRMED'
                          for i in j.get('items', [])), deadline=120)
        if not (faults and any(i.get('fault_code') == ALARM_CODE and i.get('status') == 'CONFIRMED'
                               for i in faults.get('items', []))):
            print('FAIL: alarm did not surface as a CONFIRMED fault', file=sys.stderr)
            print(json.dumps(faults), file=sys.stderr)
            print('server log:\n' + Path(server_log).read_text(errors='replace'), file=sys.stderr)
            return 1
        print('  OK secured A&C: PLC_OVERPRESSURE CONFIRMED via encrypted event subscription')

        terminate(gw_good)
        gw_good = None

        # === NON-CONDITION GUARD: explicit event_alarms catch-all on i=2253 =
        # A catch-all on the Server object maps every real condition to
        # PLC_ALARM. A non-condition BaseEventType on the SAME EventNotifier
        # (the fixture's 'sysevent' command) carries a null ConditionId and must
        # be dropped by the poller's guard - it must never surface as PLC_ALARM,
        # while a real condition on the catch-all still faults. Reset the still-
        # active Overpressure condition first so its ConditionRefresh replay does
        # not pre-map PLC_ALARM before the sysevent is even fired.
        if not send_cmd(server, server_log, 'clear Overpressure'):
            return 1

        catch_node_map = workdir / 'catchall_nodes.yaml'
        write_catchall_node_map(catch_node_map)
        catch_port = free_port()
        catch_params = workdir / 'gateway_catch.yaml'
        write_gateway_params(catch_params, port=catch_port, plugin=plugin,
                             server_port=server_port, node_map=catch_node_map,
                             manifest=manifest, certs=certs, password=PASSWORD)
        catch_log = workdir / 'gateway_catch.log'
        gw_catch = start_gateway(catch_params, catch_log, env)
        catch_base = f'http://127.0.0.1:{catch_port}/api/v1'
        catch_status = wait_json(
            f'{catch_base}/components/alarm_test_runtime/x-plc-status',
            lambda j: j.get('connected') is True, deadline=70)
        if not (catch_status and catch_status.get('connected') is True):
            print('FAIL: catch-all gateway did not connect over the secure channel',
                  file=sys.stderr)
            print(catch_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1

        # Fire the non-condition system event; the guard must drop it.
        if not send_cmd(server, server_log, 'sysevent'):
            return 1
        leaked = wait_json(
            f'{catch_base}/faults',
            lambda j: any(i.get('fault_code') == NONCOND_CODE for i in j.get('items', [])),
            deadline=20, period=2.0)
        if leaked and any(i.get('fault_code') == NONCOND_CODE for i in leaked.get('items', [])):
            print('FAIL: non-condition event surfaced as PLC_ALARM (guard regressed)',
                  file=sys.stderr)
            print(json.dumps(leaked), file=sys.stderr)
            print('server log:\n' + Path(server_log).read_text(errors='replace'),
                  file=sys.stderr)
            return 1
        print('  OK non-condition guard: sysevent did NOT raise PLC_ALARM')

        # A real condition on the catch-all source still faults (proves the
        # pipeline can produce PLC_ALARM, so the absence above is meaningful).
        if not send_cmd(server, server_log, 'fire Overpressure 750'):
            return 1
        catch_faults = wait_json(
            f'{catch_base}/faults',
            lambda j: any(i.get('fault_code') == NONCOND_CODE and i.get('status') == 'CONFIRMED'
                          for i in j.get('items', [])), deadline=120)
        got_alarm = catch_faults and any(
            i.get('fault_code') == NONCOND_CODE and i.get('status') == 'CONFIRMED'
            for i in catch_faults.get('items', []))
        if not got_alarm:
            print('FAIL: real condition on the i=2253 catch-all did not surface as PLC_ALARM',
                  file=sys.stderr)
            print(json.dumps(catch_faults), file=sys.stderr)
            print('server log:\n' + Path(server_log).read_text(errors='replace'),
                  file=sys.stderr)
            return 1
        print('  OK catch-all: real condition surfaced as PLC_ALARM CONFIRMED')

        terminate(gw_catch)
        gw_catch = None

        # === NEGATIVE: wrong password must fail closed =====================
        bad_params = workdir / 'gateway_bad.yaml'
        write_gateway_params(bad_params, port=bad_port, plugin=plugin,
                             server_port=server_port, node_map=node_map,
                             manifest=manifest, certs=certs, password=WRONG_PASSWORD)
        bad_log = workdir / 'gateway_bad.log'
        gw_bad = start_gateway(bad_params, bad_log, env)

        bad_base = f'http://127.0.0.1:{bad_port}/api/v1'
        # Fail-closed = the server rejects the wrong password and NO secured read
        # ever succeeds. We assert both: (1) the client logs BadUserAccessDenied
        # (the session never activates), and (2) the tag value is never served.
        # We key off the secured read rather than x-plc-status.connected because
        # a rejected ActivateSession leaves the underlying SecureChannel briefly
        # open, which can make the coarse connected flag flap; no authenticated
        # read can ever complete over it, and that is the real security property.
        rejected = wait_log(bad_log, 'BadUserAccessDenied', deadline=40)
        read_leaked = False
        for _ in range(10):
            d = http_json(f'{bad_base}/apps/tank_process/x-plc-data')
            if d and any(i.get('name') == 'tank_level' and i.get('value') is not None
                         for i in d.get('items', [])):
                read_leaked = True
                break
            time.sleep(2)
        if not rejected or read_leaked:
            print('FAIL: wrong password did NOT fail closed '
                  f'(auth_rejected={rejected}, read_leaked={read_leaked})', file=sys.stderr)
            print('gateway(bad) log tail:\n'
                  + bad_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        print('  OK fail-closed: wrong password rejected (BadUserAccessDenied)')

        print('PASS: secured OPC-UA A&C integration test')
        return 0
    finally:
        terminate(gw_good)
        terminate(gw_catch)
        terminate(gw_bad)
        terminate(fault_mgr)
        if server is not None:
            try:
                if server.stdin:
                    server.stdin.close()
            except OSError:
                pass
            terminate(server)
        try:
            slog.close()
        except (NameError, OSError):
            pass
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
