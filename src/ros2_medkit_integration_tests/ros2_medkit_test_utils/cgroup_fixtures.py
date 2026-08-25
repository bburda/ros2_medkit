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

"""Synthetic ``/proc`` and ``/sys`` trees for the container introspection plugin.

Every file the plugin reads is resolved under its configured ``proc_root``, so a
tree written here lets a real gateway answer real HTTP requests for cgroup
layouts the host kernel does not provide. That matters because no supported CI
distribution ships a cgroup v1 or hybrid hierarchy: a unified-only kernel has no
v1 controllers to mount, so those layouts are reachable no other way.

The tree carries the file *contents*; the gateway, the plugin, the PID cache,
the routing and the JSON serialisation are all real.
"""

import os

# A root filesystem that is not a container: a block device, no overlay.
HOST_MOUNTINFO = (
    '25 1 259:2 / / rw,relatime - ext4 /dev/nvme0n1p2 rw\n'
    '26 25 0:25 / /sys/fs/cgroup rw,nosuid,nodev,noexec,relatime - cgroup2 cgroup rw\n'
)

# What a container's root looks like: an overlay mounted at "/".
OVERLAY_MOUNTINFO = (
    '1700 461 0:154 / / rw,relatime - overlay overlay rw,lowerdir=/a,upperdir=/b\n'
    '1706 1705 0:25 / /sys/fs/cgroup ro,nosuid,nodev,noexec,relatime - cgroup2 cgroup rw\n'
)

# 64 lowercase hex characters, the shape every runtime uses for a container ID.
DOCKER_ID = 'aabb112233445566' * 4

MIB = 1024 * 1024


def write_file(root, relative, content):
    """Write ``content`` at ``relative`` under ``root``, creating parents."""
    path = os.path.join(root, relative.lstrip('/'))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as handle:
        handle.write(content)
    return path


def write_process(root, pid, fqn, cgroup, mountinfo=HOST_MOUNTINFO, markers=()):
    """Write a ``/proc/<pid>`` entry the PID cache resolves to ``fqn``.

    No process needs to exist at ``pid``: the cache scans ``<root>/proc`` for
    numeric directories and reads ``cmdline`` for ``__ns:=`` and ``__node:=``.
    ``markers`` are paths inside the process's own root, e.g. ``.dockerenv``.
    """
    namespace, _, name = fqn.rpartition('/')
    args = [
        'demo_node',
        '--ros-args',
        '-r',
        '__ns:={}'.format(namespace or '/'),
        '-r',
        '__node:={}'.format(name),
    ]
    write_file(root, 'proc/{}/cmdline'.format(pid), '\0'.join(args) + '\0')
    write_file(root, 'proc/{}/cgroup'.format(pid), cgroup)
    write_file(root, 'proc/{}/mountinfo'.format(pid), mountinfo)
    for marker in markers:
        write_file(root, 'proc/{}/root/{}'.format(pid, marker.lstrip('/')), '')


def v1_cgroup(path):
    """Build a ``/proc/<pid>/cgroup`` in the legacy format, one line per hierarchy."""
    return '12:memory:{p}\n11:cpu,cpuacct:{p}\n10:pids:{p}\n'.format(p=path)


def v2_cgroup(path):
    """Build a ``/proc/<pid>/cgroup`` in the unified format: a single ``0::`` line."""
    return '0::{}\n'.format(path)


def hybrid_cgroup(unified_path, v1_path):
    """Build a cgroup file with both hierarchies, as on a partly migrated host."""
    return '{}{}'.format(v2_cgroup(unified_path), v1_cgroup(v1_path))


def v1_limits(root, cgroup_dir, memory_bytes=None, quota_us=None, period_us=100000):
    """Write legacy controller files for the cgroup at ``cgroup_dir``.

    ``cgroup_dir`` is empty for the mount point itself, which is where a
    container sees its own cgroup when the controller is bind-mounted over it.
    """
    if memory_bytes is not None:
        write_file(
            root,
            'sys/fs/cgroup/memory{}/memory.limit_in_bytes'.format(cgroup_dir),
            '{}\n'.format(memory_bytes),
        )
    if quota_us is not None:
        write_file(
            root,
            'sys/fs/cgroup/cpu,cpuacct{}/cpu.cfs_quota_us'.format(cgroup_dir),
            '{}\n'.format(quota_us),
        )
        write_file(
            root,
            'sys/fs/cgroup/cpu,cpuacct{}/cpu.cfs_period_us'.format(cgroup_dir),
            '{}\n'.format(period_us),
        )


def v2_limits(root, cgroup_dir, memory=None, cpu=None, mount='sys/fs/cgroup'):
    """Write unified interface files for the cgroup at ``cgroup_dir``.

    ``mount`` is ``sys/fs/cgroup/unified`` on a hybrid host, where the unified
    hierarchy sits beside the legacy controllers rather than replacing them.
    """
    if memory is not None:
        write_file(root, '{}{}/memory.max'.format(mount, cgroup_dir), '{}\n'.format(memory))
    if cpu is not None:
        write_file(root, '{}{}/cpu.max'.format(mount, cgroup_dir), '{}\n'.format(cpu))
