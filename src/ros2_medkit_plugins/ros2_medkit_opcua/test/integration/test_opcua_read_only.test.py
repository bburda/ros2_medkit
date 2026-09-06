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

"""
Drive the read-only / write-capable OPC-UA plugin over real HTTP.

A live gateway loads the plugin against the in-tree OPC-UA fixture server and
every claim is checked at the wire, not in a mock:

  * a node map asking for ``writable: true`` on a node the server really does
    let us write;
  * the ``infer_writable`` sweep (true / false / absent) over the address-space
    walk, against the same node;
  * the SOVD write endpoints and the vendor write route;
  * what the entity tree advertises;
  * and a reconnect, so the answer does not change when the address space is
    walked a second time.

The expected variant is passed in from CMake, never read from a runtime
parameter: the whole point is that a read-only build is a property of the
binary, and a test that could be told otherwise at run time would not be
testing that.

Usage: test_opcua_read_only.test.py <test_alarm_server> <read-only|write-capable>
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

# The fixture registers these under the Objects folder in namespace 2. The two
# Int32 registers carry AccessLevel READ|WRITE, so the server would let a
# write-capable client change them; Tank.Level is READ only.
WRITABLE_NODE = 'ns=2;s=StatusWord'
READ_ONLY_NODE = 'ns=2;s=Tank.Level'
SECOND_WRITABLE_NODE = 'ns=2;s=FaultCode'

ENTITY = 'plc_app'
COMPONENT = 'read_only_runtime'

# The vendor code the gateway puts on the wire for any plugin provider refusal
# (primitives.cpp maps every x-medkit-* code into error_code "vendor-specific"
# plus this vendor_code). The build property is named in the message.
PLUGIN_VENDOR_CODE = 'x-medkit-plugin-error'
BUILD_PROPERTY = 'MEDKIT_OPCUA_READ_ONLY'

failures = []


def check(condition, message):
    """Record a failed expectation and keep going, so one run reports them all."""
    if condition:
        print(f'  OK   {message}')
    else:
        print(f'  FAIL {message}', file=sys.stderr)
        failures.append(message)
    return bool(condition)


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


def http(url, method='GET', body=None, timeout=5):
    """Return (status, parsed_json_or_raw_text) for one request; (0, None) on transport failure."""
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header('Content-Type', 'application/json')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode()
            status = resp.status
    except urllib.error.HTTPError as e:
        raw = e.read().decode()
        status = e.code
    except (urllib.error.URLError, OSError):
        return 0, None
    try:
        return status, json.loads(raw)
    except ValueError:
        return status, raw


def wait_json(url, predicate, deadline=90, period=2.0):
    """Poll <url> until predicate(json) holds; return the final json or None."""
    last = None
    for _ in range(int(deadline / period) + 1):
        _status, last = http(url)
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


def terminate(proc):
    """SIGTERM then SIGKILL a child process group started with start_new_session."""
    if proc is None:
        return
    if proc.returncode is not None:
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
    signal_group(signal.SIGKILL)
    log = getattr(proc, '_log', None)
    if log is not None:
        log.close()


def start_server(server_bin, port, log_path):
    """Start the plain (unsecured) fixture server and wait for its READY line."""
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        [str(server_bin), '--port', str(port)],
        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
        text=True, start_new_session=True,
    )
    proc._log = log
    if not wait_log(log_path, 'READY ', deadline=25):
        terminate(proc)
        return None
    return proc


def write_params(path, *, port, plugin, server_port, node_map, manifest, auto_browse):
    """Render a gateway params file for one leg of the sweep."""
    lines = [
        'ros2_medkit_gateway:',
        '  ros__parameters:',
        '    server:',
        '      host: "127.0.0.1"',
        f'      port: {port}',
        '    plugins: ["opcua"]',
        f'    plugins.opcua.path: "{plugin}"',
        f'    plugins.opcua.endpoint_url: "opc.tcp://127.0.0.1:{server_port}"',
        f'    plugins.opcua.node_map_path: "{node_map}"',
        '    plugins.opcua.poll_interval_ms: 500',
        '    discovery.mode: "hybrid"',
        f'    discovery.manifest_path: "{manifest}"',
        '    discovery.manifest_strict_validation: false',
    ]
    # `absent` leaves infer_writable to its own default, which is the shape most
    # deployments actually run; the other two pin the documented endpoints.
    if auto_browse is not None:
        lines.append('    plugins.opcua.auto_browse.enabled: true')
        if auto_browse != 'absent':
            lines.append(f'    plugins.opcua.auto_browse.infer_writable: {auto_browse}')
    path.write_text('\n'.join(lines) + '\n')


def start_gateway(params_file, log_path, env):
    """Launch a gateway_node bound to the given params file."""
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
         '--ros-args', '--params-file', str(params_file)],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
    )
    proc._log = log
    return proc


def item_by_name(payload, name):
    """Pick one entry out of an x-plc-data / data collection response."""
    for item in (payload or {}).get('items', []):
        if item.get('name') == name or item.get('id') == name:
            return item
    return None


def refusal_is_the_build(status, payload, what):
    """Assert one HTTP answer is the read-only build's 403 refusal, naming the build."""
    ok = check(status == 403, f'{what}: 403 (got {status})')
    if not isinstance(payload, dict):
        check(False, f'{what}: JSON error body (got {payload!r})')
        return
    ok &= check(payload.get('vendor_code') == PLUGIN_VENDOR_CODE,
                f'{what}: vendor_code {PLUGIN_VENDOR_CODE} (got {payload.get("vendor_code")!r})')
    ok &= check(BUILD_PROPERTY in str(payload.get('message', '')),
                f'{what}: message names {BUILD_PROPERTY} (got {payload.get("message")!r})')
    return ok


def node_map_text():
    """Return a map sweeping writable true / false / absent over three real server nodes."""
    return (
        'area_id: plc_systems\n'
        f'component_id: {COMPONENT}\n'
        'nodes:\n'
        f'  - node_id: "{WRITABLE_NODE}"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: status_word\n'
        '    data_type: int\n'
        '    writable: true\n'
        f'  - node_id: "{SECOND_WRITABLE_NODE}"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: fault_code\n'
        '    data_type: int\n'
        '    writable: false\n'
        f'  - node_id: "{READ_ONLY_NODE}"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: tank_level\n'
        '    data_type: float\n'
    )


def auto_browse_map_text():
    """Return a map that only turns the address-space walk on, with no hand-written nodes."""
    return (
        'area_id: plc_systems\n'
        f'component_id: {COMPONENT}\n'
        'auto_browse: true\n'
    )


def run_node_map_leg(workdir, env, plugin, server_port, manifest, read_only):
    """Run the hand-written node map leg: what the tree advertises, and what a write does."""
    print('--- node map leg (writable: true / false / absent) ---')
    node_map = workdir / 'nodes.yaml'
    node_map.write_text(node_map_text())
    port = free_port()
    params = workdir / 'gateway_nodemap.yaml'
    write_params(params, port=port, plugin=plugin, server_port=server_port,
                 node_map=node_map, manifest=manifest, auto_browse=None)
    log = workdir / 'gateway_nodemap.log'
    gw = start_gateway(params, log, env)
    try:
        base = f'http://127.0.0.1:{port}/api/v1'
        data = wait_json(f'{base}/apps/{ENTITY}/x-plc-data',
                         lambda j: item_by_name(j, 'status_word') is not None)
        if not check(item_by_name(data, 'status_word') is not None,
                     'x-plc-data serves the mapped points'):
            print(log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return

        # 1. The writable flag on the wire.
        for name, mapped in (('status_word', True), ('fault_code', False), ('tank_level', None)):
            item = item_by_name(data, name)
            expected = bool(mapped) and not read_only
            check(item is not None and item.get('writable') is expected,
                  f'{name} (map writable={mapped}) reports writable={expected}')

        # 2. What the entity advertises.
        _status, detail = http(f'{base}/apps/{ENTITY}')
        names = {c.get('name') for c in (detail or {}).get('capabilities', [])}
        check(('x-plc-operations' in names) is not read_only,
              f'x-plc-operations capability advertised: {not read_only}')

        # 3. The SOVD operations collection.
        _status, ops = http(f'{base}/apps/{ENTITY}/operations')
        op_ids = {o.get('id') for o in (ops or {}).get('items', [])}
        check(('set_status_word' in op_ids) is not read_only,
              f'set_status_word offered: {not read_only} (got {sorted(op_ids)})')

        # 4. PUT /data/{id} on the point the map and the server both allow.
        status, body = http(f'{base}/apps/{ENTITY}/data/status_word', 'PUT', {'value': 7})
        if read_only:
            refusal_is_the_build(status, body, 'PUT data on a map-writable point')
        else:
            check(status == 200, f'PUT data succeeds (got {status}: {body!r})')
            got = wait_json(f'{base}/apps/{ENTITY}/x-plc-data',
                            lambda j: (item_by_name(j, 'status_word') or {}).get('value') == 7,
                            deadline=20)
            check((item_by_name(got, 'status_word') or {}).get('value') == 7,
                  'the written value reached the server')

        # 5. POST an execution on the operation the map asks for.
        exec_url = f'{base}/apps/{ENTITY}/operations/set_status_word/executions'
        status, body = http(exec_url, 'POST', {'value': 9})
        if read_only:
            refusal_is_the_build(status, body, 'POST execution on set_status_word')
        else:
            check(status in (200, 202), f'POST execution succeeds (got {status}: {body!r})')
            got = wait_json(f'{base}/apps/{ENTITY}/x-plc-data',
                            lambda j: (item_by_name(j, 'status_word') or {}).get('value') == 9,
                            deadline=20)
            check((item_by_name(got, 'status_word') or {}).get('value') == 9,
                  'the executed operation reached the server')

        # 6. The vendor route is not registered at all in a read-only build.
        status, body = http(f'{base}/apps/{ENTITY}/x-plc-operations/set_status_word',
                            'POST', {'value': 11})
        if read_only:
            check(status == 404,
                  f'POST x-plc-operations is not routed (got {status}: {body!r})')
        else:
            check(status == 200, f'POST x-plc-operations succeeds (got {status}: {body!r})')

        # 7. Points the map did NOT mark writable.
        for name in ('fault_code', 'tank_level'):
            status, body = http(f'{base}/apps/{ENTITY}/data/{name}', 'PUT', {'value': 3})
            if read_only:
                refusal_is_the_build(status, body, f'PUT data on {name}')
            else:
                check(status == 400,
                      f'PUT data on the read-only point {name} is 400 (got {status})')

        # 8. The startup warning naming the build property.
        if read_only:
            check(wait_log(log, BUILD_PROPERTY, deadline=5),
                  f'the gateway log names {BUILD_PROPERTY} for the ignored writable: true')
    finally:
        terminate(gw)


def auto_browse_writable(base, deadline=90):
    """Return the writable flag of the auto-browsed StatusWord point, or None."""
    payload = wait_json(f'{base}/apps', lambda j: j.get('items'), deadline=deadline)
    for app in (payload or {}).get('items', []):
        _status, data = http(f'{base}/apps/{app.get("id")}/x-plc-data')
        item = item_by_name(data, 'statusword') or item_by_name(data, 'StatusWord')
        if item is not None:
            return item.get('writable')
    return None


def run_auto_browse_leg(workdir, env, plugin, server_bin, server, server_port,
                        manifest, read_only, infer_writable, rebrowse):
    """One infer_writable setting: the walk must never mark a point writable here."""
    label = f'infer_writable={infer_writable}'
    print(f'--- auto_browse leg ({label}) ---')
    node_map = workdir / f'auto_{infer_writable}.yaml'
    node_map.write_text(auto_browse_map_text())
    port = free_port()
    params = workdir / f'gateway_auto_{infer_writable}.yaml'
    write_params(params, port=port, plugin=plugin, server_port=server_port,
                 node_map=node_map, manifest=manifest, auto_browse=infer_writable)
    log = workdir / f'gateway_auto_{infer_writable}.log'
    gw = start_gateway(params, log, env)
    try:
        base = f'http://127.0.0.1:{port}/api/v1'
        writable = auto_browse_writable(base)
        # The server sets CurrentWrite on StatusWord, so a write-capable build
        # infers writable unless the setting says otherwise. A read-only build
        # never consults the bit.
        expected = (not read_only) and infer_writable != 'false'
        check(writable is expected,
              f'{label}: auto-browsed StatusWord reports writable={expected} (got {writable!r})')

        if rebrowse:
            # CHANGE: the plugin re-walks the address space on a fresh session
            # (maybe_rebrowse_on_reconnect). Restarting the server is the only
            # runtime path that redoes the walk - the node map itself is read
            # once at configure() and has no reload.
            print(f'--- auto_browse leg ({label}) after a reconnect ---')
            terminate(server[0])
            server[0] = start_server(server_bin, server_port,
                                     workdir / 'alarm_server_restarted.log')
            if not check(server[0] is not None, 'fixture server restarted on the same port'):
                return
            reconnected = wait_json(f'{base}/components/{COMPONENT}/x-plc-status',
                                    lambda j: j.get('connected') is True, deadline=120)
            if not check(bool(reconnected) and reconnected.get('connected') is True,
                         'the plugin reconnected after the server restart'):
                print(log.read_text(errors='replace')[-3000:], file=sys.stderr)
                return
            writable = auto_browse_writable(base, deadline=60)
            check(writable is expected,
                  f'{label}: after the re-walk, writable={expected} (got {writable!r})')
    finally:
        terminate(gw)


def main():
    if len(sys.argv) < 3:
        print('usage: test_opcua_read_only.test.py <test_alarm_server> <read-only|write-capable>',
              file=sys.stderr)
        return 2
    server_bin = Path(sys.argv[1]).resolve()
    variant = sys.argv[2]
    if variant not in ('read-only', 'write-capable'):
        print(f'unknown build variant {variant!r}', file=sys.stderr)
        return 2
    read_only = variant == 'read-only'
    print(f'build variant under test: {variant}')

    # ROS_DOMAIN_ID comes from the domain wrapper this test is registered
    # behind; a missing value is a wiring bug, not something to guess around.
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    if not ros_domain_id:
        print('ROS_DOMAIN_ID is not set: this test must be launched by CTest, which '
              'runs it behind medkit_run_with_domain.py', file=sys.stderr)
        return 1

    # Every prerequisite below is produced by the same build that produces this
    # test, so a missing one is a broken build rather than an environment this
    # run should tiptoe around.
    for tool in ('ros2', 'nm'):
        if shutil.which(tool) is None:
            print(f'FAIL: {tool} not on PATH', file=sys.stderr)
            return 1
    if not (server_bin.is_file() and os.access(server_bin, os.X_OK)):
        print(f'FAIL: fixture server missing: {server_bin}', file=sys.stderr)
        return 1
    plugin = find_plugin()
    if plugin is None:
        print('FAIL: libros2_medkit_opcua_plugin.so not found under AMENT_PREFIX_PATH',
              file=sys.stderr)
        return 1

    workdir = Path(tempfile.mkdtemp(prefix='opcua_read_only_'))
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    manifest = workdir / 'manifest.yaml'
    manifest.write_text('manifest_version: "1.0"\n')
    server_port = free_port()
    server = [start_server(server_bin, server_port, workdir / 'alarm_server.log')]
    try:
        if server[0] is None:
            print('FAIL: fixture server did not become READY', file=sys.stderr)
            print((workdir / 'alarm_server.log').read_text(errors='replace'), file=sys.stderr)
            return 1

        run_node_map_leg(workdir, env, plugin, server_port, manifest, read_only)
        for infer_writable, rebrowse in (('absent', True), ('true', False), ('false', False)):
            run_auto_browse_leg(workdir, env, plugin, server_bin, server, server_port,
                                manifest, read_only, infer_writable, rebrowse)

        if failures:
            print(f'FAIL: {len(failures)} expectation(s) not met:', file=sys.stderr)
            for f in failures:
                print(f'  - {f}', file=sys.stderr)
            return 1
        print(f'PASS: the {variant} plugin behaves as its build declares')
        return 0
    finally:
        terminate(server[0])
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
