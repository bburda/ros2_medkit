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
Inspect the built OPC-UA plugin object for the write surface it declares.

The check reads the object itself rather than any configuration.

A read-only build is a property of the binary: `MEDKIT_OPCUA_READ_ONLY=ON`
compiles the OPC-UA value-write path out, so no symbol that can put a value on
the wire is emitted. A runtime flag could be flipped; an absent symbol cannot.

The check runs in BOTH variants with opposite expectations, and that is what
makes it meaningful:

  read-only     every WRITE_MARKER absent, every READ_MARKER present
  write-capable every WRITE_MARKER present, every READ_MARKER present

Asserting only the read-only direction would pass for a marker list that
matches nothing at all - a renamed or fully inlined symbol would read as
"the write path is gone". The write-capable direction is the control that keeps
the marker list honest, and READ_MARKER is the control that fails on an empty,
stripped or wrong object.

Usage: inspect_build_variant.py <plugin.so> --expect {read-only,write-capable}
"""

import argparse
import shutil
import subprocess
import sys

# Symbols that exist only when the value-write path is compiled in. Each is on
# the road from the provider entry points to the wire:
#   - OpcuaClient::write_value is the plugin's own single write entry point;
#   - OpcuaPlugin::handle_plc_operations is the vendor route that reaches it;
#   - parse_coerce_validate is the value coercion the three write paths share;
#   - Node<Client>::writeValueScalar / writeValue are the open62541pp templates
#     where the value is encoded for the Write service;
#   - services::write / services::writeAttribute<Client> are the layer beneath
#     them, the last C++ frame before open62541's own client machinery.
#
# Every marker is verified to discriminate in an OPTIMIZED build, which is what
# CI and every release produce. That rules out opcua::services::writeValue and
# writeAttributeImpl<AttributeId 13, Client>: they are fully inlined at -O2 and
# emit no symbol in either variant, so they would read as "the write path is
# gone" in both builds and prove nothing.
#
# open62541's own primitives - __UA_Client_writeAttribute and the rest of the
# UA_Client_write* family - are absent from BOTH objects and therefore cannot
# discriminate either. They used to be present, and exported, in both: nothing
# in the plugin referenced them (open62541pp reaches the Write service through
# services::write, not through them), but the whole archive member was linked in
# and -fvisibility=hidden does not reach a static archive, so dlsym could call
# one and drive a controller the REST contract never exposed.
# -Wl,--exclude-libs,ALL plus -ffunction-sections/-fdata-sections and
# -Wl,--gc-sections removed them from the object outright. EXPORT_DENY below is
# what keeps that true: what remains of open62541 in a read-only object - the
# generic __UA_Client_Service dispatcher the read path needs, and the generated
# type descriptors the UA_TYPES table pins - is unreachable precisely because
# the module exports none of it.
WRITE_MARKERS = (
    'ros2_medkit_gateway::OpcuaClient::write_value(',
    'ros2_medkit_gateway::OpcuaPlugin::handle_plc_operations(',
    'parse_coerce_validate(',
    'opcua::Node<opcua::Client>::writeValueScalar<',
    'opcua::Node<opcua::Client>::writeValue(opcua::Variant const&)',
    'opcua::services::write(opcua::Client&, opcua::WriteRequest const&)',
    'opcua::services::writeAttribute<opcua::Client>',
)

# The read path the plugin needs in every variant. Present in both builds, so a
# stripped, truncated or simply wrong object fails here instead of silently
# satisfying the "no write symbols" half. These are the plugin's own out-of-line
# definitions rather than open62541pp templates, for the same optimization
# reason as above: services::detail::readAttributeImpl is inlined away at -O2.
READ_MARKERS = (
    'ros2_medkit_gateway::OpcuaClient::read_value(',
    'ros2_medkit_gateway::OpcuaClient::read_values(',
    'ros2_medkit_gateway::OpcuaClient::read_access_level(',
    'ros2_medkit_gateway::OpcuaClient::browse_detailed(',
)

# Nothing from the OPC-UA stack may appear in the module's dynamic symbol table,
# in either variant. The gateway dlopens the plugin and needs its extern "C"
# entry points and nothing else; anything else exported is a dlsym handle on
# machinery no route exposes. Checked as a prefix match on the demangled name,
# which covers both the C library (UA_*, __UA_*) and the C++ wrapper (opcua::*).
EXPORT_DENY = ('UA_', 'opcua::')

# Symbols the gateway resolves out of the plugin. If a link-time change ever
# hides these, the plugin still builds and still passes every symbol check above
# while failing to load at runtime, so they are asserted here rather than left
# to an integration test to discover.
REQUIRED_EXPORTS = (
    'create_plugin',
    'plugin_api_version',
    'get_introspection_provider',
    'get_data_provider',
    'get_operation_provider',
    'get_fault_provider',
)

# An optimized plugin object still carries a few thousand symbols. Anything near
# zero means nm read something that is not the plugin, or an object whose symbol
# table was stripped, and every "absent" verdict below would then be vacuous.
MIN_SYMBOLS = 1000


def nm(args, path):
    """Run nm with the given flags and return its stdout lines."""
    out = subprocess.run(
        ['nm', *args, path], capture_output=True, text=True, check=False)
    if out.returncode != 0:
        print(f'FAIL: nm {" ".join(args)} {path} exited {out.returncode}\n{out.stderr}',
              file=sys.stderr)
        sys.exit(1)
    return out.stdout.splitlines()


def count(lines, marker):
    """Return the number of symbol lines containing the marker substring."""
    return sum(1 for line in lines if marker in line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plugin', help='path to libros2_medkit_opcua_plugin.so')
    parser.add_argument('--expect', required=True,
                        choices=('read-only', 'write-capable'),
                        help='the write surface this build declares')
    args = parser.parse_args()

    if shutil.which('nm') is None:
        print('FAIL: nm (binutils) not on PATH - the build inspection cannot run',
              file=sys.stderr)
        return 1

    defined = nm(['-C'], args.plugin)
    exported = nm(['-DC', '--defined-only'], args.plugin)
    print(f'{args.plugin}: {len(defined)} symbols, {len(exported)} dynamic exports, '
          f'expecting {args.expect}')

    failures = []
    if len(defined) < MIN_SYMBOLS:
        failures.append(f'only {len(defined)} symbols (< {MIN_SYMBOLS}) - '
                        'object is stripped, truncated or not the plugin')

    for marker in READ_MARKERS:
        n = count(defined, marker)
        print(f'  read  {n:>4}  {marker}')
        if n == 0:
            failures.append(f'read path symbol missing: {marker}')

    want_writes = args.expect == 'write-capable'
    for marker in WRITE_MARKERS:
        n = count(defined, marker)
        print(f'  write {n:>4}  {marker}')
        if want_writes and n == 0:
            failures.append(f'write-capable build is missing: {marker}')
        if not want_writes and n != 0:
            failures.append(f'read-only build still contains {n} x: {marker}')

    # The export table is an invariant, not a variant property: neither build
    # may hand dlsym a way into the OPC-UA stack.
    leaked = [line.split(' ', 2)[-1] for line in exported
              if any(line.split(' ', 2)[-1].startswith(p) for p in EXPORT_DENY)]
    print(f'  export {len(leaked):>4}  OPC-UA symbols in the dynamic symbol table')
    for name in leaked[:10]:
        failures.append(f'dynamic symbol table exports OPC-UA machinery: {name}')

    export_names = {line.split(' ', 2)[-1] for line in exported}
    missing = [name for name in REQUIRED_EXPORTS if name not in export_names]
    print(f'  export {len(REQUIRED_EXPORTS) - len(missing):>4}'
          f'/{len(REQUIRED_EXPORTS)} plugin entry points')
    for name in missing:
        failures.append(f'plugin entry point not exported: {name}')

    if failures:
        print(f'FAIL ({args.expect}):', file=sys.stderr)
        for f in failures:
            print(f'  - {f}', file=sys.stderr)
        return 1

    print(f'PASS: object matches the {args.expect} build variant')
    return 0


if __name__ == '__main__':
    sys.exit(main())
