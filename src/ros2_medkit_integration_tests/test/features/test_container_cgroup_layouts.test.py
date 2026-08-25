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

"""Container introspection over every supported cgroup layout.

The gateway runs for real and answers real HTTP requests. Only the contents of
``/proc`` and ``/sys`` are synthetic, supplied through the plugin's ``proc_root``
setting. That is the only way to reach cgroup v1 and the hybrid layout at all:
a unified-only kernel has no v1 controllers to mount, and every supported
distribution boots unified.

Each demo node is mapped to a different layout, so one gateway covers them all.
"""

import atexit
import os
import shutil
import tempfile
import time
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.cgroup_fixtures import (
    DOCKER_ID,
    HOST_MOUNTINFO,
    hybrid_cgroup,
    OVERLAY_MOUNTINFO,
    v1_cgroup,
    v1_limits,
    v2_cgroup,
    v2_limits,
    write_process,
)
from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

# 512 MiB and half a CPU, the numbers every layout below is set to report.
MEMORY_BYTES = 536870912
QUOTA_US = 50000
PERIOD_US = 100000

# Further containers, so grouping has more than one entry to keep apart.
OTHER_ID = 'ccdd445566778899' * 4
THIRD_ID = 'eeff001122334455' * 4
FOURTH_ID = '0011223344556677' * 4

PROC_ROOT = tempfile.mkdtemp(prefix='medkit_cgroup_layouts_')
atexit.register(shutil.rmtree, PROC_ROOT, True)


def _build_tree(root):
    """One synthetic tree holding a different cgroup layout per demo node."""
    docker_path = '/docker/' + DOCKER_ID

    # cgroup v1, gateway on the host: the reported path leads into the
    # hierarchy, so the limits sit under the mount point joined with it.
    write_process(root, 4101, '/powertrain/engine/temp_sensor', v1_cgroup(docker_path))
    v1_limits(root, docker_path, memory_bytes=MEMORY_BYTES, quota_us=QUOTA_US, period_us=PERIOD_US)

    # cgroup v1, same container, seen from inside it: Docker bind-mounts the
    # container's own cgroup over /sys/fs/cgroup/<controller>, so the reported
    # path resolves nowhere and the files are at the mount point itself.
    write_process(root, 4102, '/powertrain/engine/rpm_sensor', v1_cgroup(docker_path),
                  mountinfo=OVERLAY_MOUNTINFO, markers=['.dockerenv'])
    v1_limits(root, '', memory_bytes=MEMORY_BYTES, quota_us=QUOTA_US, period_us=PERIOD_US)

    # cgroup v1, second container. Its hierarchy root reads as unlimited, which
    # must not mask the limit set on the container's own cgroup below it.
    other_path = '/docker/' + OTHER_ID
    write_process(root, 4103, '/chassis/brakes/pressure_sensor', v1_cgroup(other_path))
    v1_limits(root, other_path, memory_bytes=MEMORY_BYTES, quota_us=QUOTA_US, period_us=PERIOD_US)

    # Hybrid: the unified line is the bare root, so the container is named only
    # on the legacy hierarchies and the limits come from there.
    write_process(root, 4104, '/body/door/front_left/status_sensor',
                  hybrid_cgroup('/', docker_path))

    # cgroup v2 with no limits configured: unlimited, not missing.
    scope = '/system.slice/docker-' + THIRD_ID + '.scope'
    write_process(root, 4105, '/chassis/brakes/actuator', v2_cgroup(scope))
    v2_limits(root, scope, memory='max', cpu='max {}'.format(PERIOD_US))

    # cgroup v2 whose limit file cannot be parsed: unreadable, not unlimited.
    calib_scope = '/system.slice/docker-' + FOURTH_ID + '.scope'
    write_process(root, 4106, '/powertrain/engine/calibration', v2_cgroup(calib_scope))
    v2_limits(root, calib_scope, memory='not-a-number', cpu='{} {}'.format(QUOTA_US, PERIOD_US))

    # A process on a plain host: no container id, no marker, no overlay root.
    write_process(root, 4107, '/body/lights/controller',
                  v2_cgroup('/user.slice/user-1000.slice/session-1.scope'),
                  mountinfo=HOST_MOUNTINFO)


def _get_plugin_path(plugin_so_name):
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_linux_introspection')
    return os.path.join(pkg_prefix, 'lib', 'ros2_medkit_linux_introspection', plugin_so_name)


def generate_test_description():
    _build_tree(PROC_ROOT)
    return create_test_launch(
        demo_nodes=[
            'temp_sensor',
            'rpm_sensor',
            'pressure_sensor',
            'status_sensor',
            'actuator',
            'calibration',
            'controller',
        ],
        fault_manager=False,
        gateway_params={
            'plugins': ['container'],
            'plugins.container.path': _get_plugin_path('libcontainer_introspection.so'),
            'plugins.container.proc_root': PROC_ROOT,
            'plugins.container.pid_cache_ttl_seconds': 1,
        },
    )


class TestContainerCgroupLayouts(GatewayTestCase):
    """Every cgroup layout the reader supports, over the real HTTP endpoint.

    @verifies REQ_INTEROP_003
    """

    MIN_EXPECTED_APPS = 7
    REQUIRED_APPS = {
        'temp_sensor',
        'rpm_sensor',
        'pressure_sensor',
        'status_sensor',
        'actuator',
        'calibration',
        'controller',
    }

    def _container(self, app_id):
        """Poll the container endpoint until it answers, then return the body."""

        def _ready(data):
            return data if 'memory_limit_state' in data else None

        return self.poll_endpoint_until(
            '/apps/{}/x-medkit-container'.format(app_id), _ready
        )

    def _assert_limited(self, data, app_id):
        self.assertEqual(data['memory_limit_state'], 'limited', app_id)
        self.assertEqual(data['memory_limit_bytes'], MEMORY_BYTES, app_id)
        self.assertEqual(data['cpu_quota_state'], 'limited', app_id)
        self.assertEqual(data['cpu_quota_us'], QUOTA_US, app_id)
        self.assertEqual(data['cpu_period_us'], PERIOD_US, app_id)

    def test_01_v1_host_namespace_joined_path(self):
        """Legacy hierarchy with the limits under the mount point plus the reported path."""
        data = self._container('temp_sensor')
        self.assertEqual(data['container_id'], DOCKER_ID)
        self.assertEqual(data['runtime'], 'docker')
        self._assert_limited(data, 'temp_sensor')

    def test_02_v1_inside_container_bare_mount(self):
        """Legacy hierarchy where the reported path resolves nowhere inside the container."""
        data = self._container('rpm_sensor')
        self.assertEqual(data['container_id'], DOCKER_ID)
        self._assert_limited(data, 'rpm_sensor')

    def test_03_v1_hierarchy_root_does_not_mask_the_limit(self):
        """The always-unlimited v1 root must not win over the container's own cgroup."""
        data = self._container('pressure_sensor')
        self.assertEqual(data['container_id'], OTHER_ID)
        self._assert_limited(data, 'pressure_sensor')

    def test_04_hybrid_layout_reads_the_legacy_hierarchy(self):
        """With the unified line at the root, the container is named by cgroup v1."""
        data = self._container('status_sensor')
        self.assertEqual(data['container_id'], DOCKER_ID)
        self._assert_limited(data, 'status_sensor')

    def test_05_no_limit_configured_reports_unlimited(self):
        """An unconstrained container is 'unlimited', and carries no number."""
        data = self._container('actuator')
        self.assertEqual(data['memory_limit_state'], 'unlimited')
        self.assertNotIn('memory_limit_bytes', data)
        self.assertEqual(data['cpu_quota_state'], 'unlimited')
        self.assertNotIn('cpu_quota_us', data)
        # The period is still a fact about the cgroup with no quota on it.
        self.assertEqual(data['cpu_period_us'], PERIOD_US)

    def test_06_unparsable_limit_reports_unreadable(self):
        """A limit file that cannot be parsed is not the same as no limit."""
        data = self._container('calibration')
        self.assertEqual(data['memory_limit_state'], 'unreadable')
        self.assertNotIn('memory_limit_bytes', data)
        # The CPU leg of the same container is fine, so it still reports.
        self.assertEqual(data['cpu_quota_state'], 'limited')
        self.assertEqual(data['cpu_quota_us'], QUOTA_US)

    def test_07_host_process_is_not_containerized(self):
        """A process on a plain host gets 404, not a report of the root cgroup."""
        deadline = time.monotonic() + 20.0
        body = {}
        while time.monotonic() < deadline:
            r = requests.get(
                '{}/apps/controller/x-medkit-container'.format(self.BASE_URL), timeout=5
            )
            body = r.json()
            if r.status_code == 404 and body.get('vendor_code') == 'x-medkit-not-containerized':
                return
            time.sleep(0.5)
        self.fail('controller never settled on not-containerized, last body: {}'.format(body))

    def test_08_component_groups_by_container(self):
        """Apps in one container share an entry; different containers do not merge."""
        components = requests.get(
            '{}/components'.format(self.BASE_URL), timeout=5
        ).json().get('items', [])
        self.assertTrue(components, 'no components discovered')

        def _ready(data):
            return data if len(data.get('containers', [])) >= 2 else None

        data = self.poll_endpoint_until(
            '/components/{}/x-medkit-container'.format(components[0]['id']), _ready
        )
        by_id = {c['container_id']: c for c in data['containers']}

        self.assertIn(DOCKER_ID, by_id)
        self.assertIn(OTHER_ID, by_id)
        # temp_sensor, rpm_sensor and status_sensor all report the same
        # container, so they belong to one entry rather than three.
        self.assertEqual(
            set(by_id[DOCKER_ID]['node_ids']),
            {'temp_sensor', 'rpm_sensor', 'status_sensor'},
        )
        self.assertEqual(by_id[DOCKER_ID]['memory_limit_bytes'], MEMORY_BYTES)
        # A different container keeps its own entry and its own limits.
        self.assertEqual(set(by_id[OTHER_ID]['node_ids']), {'pressure_sensor'})
        # controller is on the host and must not appear at all.
        for entry in data['containers']:
            self.assertNotIn('controller', entry['node_ids'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Check the gateway and demo nodes exited cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
