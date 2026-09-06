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
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

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
# machinery no route exposes.
#
# A regex, not a prefix match on "UA_": open62541's internal entry points are
# spelled with leading underscores (__UA_Client_writeAttribute,
# __UA_Client_Service), so a startswith check waves through exactly the symbol
# that started this - a version script exporting it alongside the six entry
# points passed a prefix check while handing dlsym a working write primitive.
# Leading underscores are optional in the match for that reason.
EXPORT_DENY = re.compile(r'^_*UA_|^opcua::')

# open62541's own Write service primitives, which the plugin never calls: the
# C++ wrapper reaches the service through opcua::services::write. They must be
# absent from a read-only object even as LOCAL symbols, not merely unexported -
# a symbol nothing exports is still a gadget for anything running in the same
# process, and their presence means the archive member was pulled in, which is
# the state a link or optimisation change can silently restore. Checked as an
# absence invariant rather than a variant marker because they are absent from
# the write-capable object too, so they discriminate nothing between builds.
WRITE_PRIMITIVES = re.compile(r'_*UA_(Client|Server)_write')

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


def symbol_name(line):
    """Return the demangled name from one nm line, without the address and type."""
    return line.split(' ', 2)[-1]


def check_object(plugin, expect, report):
    """Check one object against a variant and return the list of failures."""
    defined = nm(['-C'], plugin)
    exported = nm(['-DC', '--defined-only'], plugin)
    report(f'{plugin}: {len(defined)} symbols, {len(exported)} dynamic exports, '
           f'expecting {expect}')

    failures = []
    if len(defined) < MIN_SYMBOLS:
        failures.append(f'only {len(defined)} symbols (< {MIN_SYMBOLS}) - '
                        'object is stripped, truncated or not the plugin')

    for marker in READ_MARKERS:
        n = count(defined, marker)
        report(f'  read  {n:>4}  {marker}')
        if n == 0:
            failures.append(f'read path symbol missing: {marker}')

    want_writes = expect == 'write-capable'
    for marker in WRITE_MARKERS:
        n = count(defined, marker)
        report(f'  write {n:>4}  {marker}')
        if want_writes and n == 0:
            failures.append(f'write-capable build is missing: {marker}')
        if not want_writes and n != 0:
            failures.append(f'read-only build still contains {n} x: {marker}')

    # The export table is an invariant, not a variant property: neither build
    # may hand dlsym a way into the OPC-UA stack.
    leaked = [symbol_name(line) for line in exported
              if EXPORT_DENY.search(symbol_name(line))]
    report(f'  export {len(leaked):>4}  OPC-UA symbols in the dynamic symbol table')
    for name in leaked[:10]:
        failures.append(f'dynamic symbol table exports OPC-UA machinery: {name}')

    export_names = {symbol_name(line) for line in exported}
    missing = [name for name in REQUIRED_EXPORTS if name not in export_names]
    report(f'  export {len(REQUIRED_EXPORTS) - len(missing):>4}'
           f'/{len(REQUIRED_EXPORTS)} plugin entry points')
    for name in missing:
        failures.append(f'plugin entry point not exported: {name}')

    # And the library's Write primitives must not be in the read-only object at
    # all, exported or not. Independent of build type: upstream only compiles
    # with -ffunction-sections for Release and MinSizeRel, so without the flags
    # this package sets on the object libraries a default-type build kept twelve
    # of them as local symbols while every other check here still passed.
    if not want_writes:
        primitives = [symbol_name(line) for line in defined
                      if WRITE_PRIMITIVES.search(symbol_name(line))]
        report(f'  absent {len(primitives):>4}  open62541 UA_*_write* primitives')
        for name in primitives[:10]:
            failures.append(f'read-only object still carries a write primitive: {name}')

    return failures


# The reviewer's reproducer, kept as a test of the test: a module that exports
# __UA_Client_writeAttribute next to the six entry points. It is what a version
# script, or an -fvisibility slip on the vendored library, produces, and the
# prefix match this check used to do waved it through - the leading underscores
# meant it did not start with "UA_". The object is otherwise a plausible plugin
# (the read markers and entry points are there), so a rejection can only come
# from the export rule under test.
SELF_CHECK_SOURCE = r"""
#include <stdio.h>
#define EXPORT __attribute__((visibility("default")))
EXPORT int __UA_Client_writeAttribute(void) { return 0; }
EXPORT void *create_plugin(void) { return 0; }
EXPORT int plugin_api_version(void) { return 1; }
EXPORT void *get_introspection_provider(void *p) { return p; }
EXPORT void *get_data_provider(void *p) { return p; }
EXPORT void *get_operation_provider(void *p) { return p; }
EXPORT void *get_fault_provider(void *p) { return p; }
"""


def build_self_check_object(workdir):
    """Compile the reproducer module; return its path, or None with a reason."""
    compiler = shutil.which('cc') or shutil.which('gcc')
    if compiler is None:
        return None, 'no C compiler on PATH'
    src = workdir / 'leaky_plugin.c'
    obj = workdir / 'leaky_plugin.so'
    src.write_text(SELF_CHECK_SOURCE)
    out = subprocess.run(
        [compiler, '-shared', '-fPIC', '-fvisibility=hidden', '-o', str(obj), str(src)],
        capture_output=True, text=True, check=False)
    if out.returncode != 0:
        return None, f'compile failed: {out.stderr.strip()}'
    return obj, ''


def self_check():
    """Prove the rules reject what they are meant to reject. Returns an exit code."""
    quiet = (lambda *a, **k: None)
    problems = []

    # The name rules, on symbol lines rather than on a whole object, so the
    # spellings that matter are pinned one by one.
    must_deny = ('__UA_Client_writeAttribute', 'UA_Client_writeValueAttribute',
                 '_UA_Server_write', '__UA_Client_Service',
                 'opcua::services::write(opcua::Client&, opcua::WriteRequest const&)')
    must_allow = ('create_plugin', 'plugin_api_version', 'medkit_UA_helper',
                  'std::__cxx11::basic_string<char>::~basic_string()')
    for name in must_deny:
        if not EXPORT_DENY.search(name):
            problems.append(f'EXPORT_DENY fails to match {name}')
    for name in must_allow:
        if EXPORT_DENY.search(name):
            problems.append(f'EXPORT_DENY wrongly matches {name}')

    must_be_primitives = ('__UA_Client_writeAttribute', 'UA_Client_writeArrayDimensionsAttribute',
                          'UA_Server_writeValue', '__UA_Server_write')
    must_not_be_primitives = ('__UA_Client_readAttribute', 'UA_Client_Service_read',
                              'UA_WriteRequest_init')
    for name in must_be_primitives:
        if not WRITE_PRIMITIVES.search(name):
            problems.append(f'WRITE_PRIMITIVES fails to match {name}')
    for name in must_not_be_primitives:
        if WRITE_PRIMITIVES.search(name):
            problems.append(f'WRITE_PRIMITIVES wrongly matches {name}')

    # And the same rule end to end, on a real object built the way the reviewer
    # built one. A compiler is present wherever this test runs, because the
    # package it inspects was just compiled.
    workdir = Path(tempfile.mkdtemp(prefix='opcua_self_check_'))
    try:
        obj, reason = build_self_check_object(workdir)
        if obj is None:
            print(f'FAIL: cannot build the reproducer object - {reason}', file=sys.stderr)
            return 1
        failures = check_object(str(obj), 'read-only', quiet)
        leak = [f for f in failures if '__UA_Client_writeAttribute' in f
                and 'exports OPC-UA machinery' in f]
        if not leak:
            problems.append('an object exporting __UA_Client_writeAttribute was not '
                            'rejected by the export rule')
        entry_points = [f for f in failures if 'entry point not exported' in f]
        if entry_points:
            problems.append('the reproducer object is not shaped like a plugin, so its '
                            f'rejection proves nothing: {entry_points}')
    finally:
        shutil.rmtree(workdir, ignore_errors=True)

    if problems:
        print('FAIL (self-check):', file=sys.stderr)
        for p in problems:
            print(f'  - {p}', file=sys.stderr)
        return 1
    print(f'PASS: self-check - {len(must_deny) + len(must_allow)} export-rule cases, '
          f'{len(must_be_primitives) + len(must_not_be_primitives)} write-primitive cases, '
          'and a linked object exporting __UA_Client_writeAttribute is rejected')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plugin', nargs='?',
                        help='path to libros2_medkit_opcua_plugin.so')
    parser.add_argument('--expect', choices=('read-only', 'write-capable'),
                        help='the write surface this build declares')
    parser.add_argument('--self-check', action='store_true',
                        help='check the rules against objects and names built to break them')
    args = parser.parse_args()

    if args.self_check:
        return self_check()
    if not args.plugin or not args.expect:
        parser.error('a plugin path and --expect are required unless --self-check is given')

    if shutil.which('nm') is None:
        print('FAIL: nm (binutils) not on PATH - the build inspection cannot run',
              file=sys.stderr)
        return 1

    failures = check_object(args.plugin, args.expect, print)
    if failures:
        print(f'FAIL ({args.expect}):', file=sys.stderr)
        for f in failures:
            print(f'  - {f}', file=sys.stderr)
        return 1

    print(f'PASS: object matches the {args.expect} build variant')
    return 0


if __name__ == '__main__':
    sys.exit(main())
