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
"""Shared launch_testing harness for ros2_medkit_graph_watchdog e2e tests.

Brings up a real gateway_node with the graph_watchdog plugin (.so) loaded,
a real fault_manager (memory storage, healing enabled), and caller-supplied
demo nodes - the only way to prove that nested plugin config actually
reaches a live detector through the real config-delivery path
(extract_plugin_config -> detector_config.hpp -> a detector's own
configure()). A C++ unit test cannot prove this: it hand-builds already-
nested JSON and calls configure() directly, bypassing the real gateway
parameter plumbing entirely - which is exactly what hid the flat/nested
bug this harness exists to catch a regression of.

Reuses the `ros2_medkit_test_utils` launch factories
(create_gateway_node, create_fault_manager_node, create_demo_nodes) so this
harness stays in lockstep with how the real gateway/fault_manager are
launched elsewhere. Requires `ros2_medkit_integration_tests` as a
package.xml test_depend (see CMakeLists.txt) so that package is importable
on PYTHONPATH once the workspace's install/setup.bash is sourced.
"""

import json
import os
import time

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing.actions
import requests
from ros2_medkit_test_utils.constants import get_test_port
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_fault_manager_node,
    create_gateway_node,
)

API_BASE_PATH = '/api/v1'


def create_watchdog_test_launch(
    *,
    detector_params=None,
    extra_gateway_params=None,
    demo_nodes=None,
    port=None,
    demo_delay=2.0,
    healing_enabled=True,
):
    """Build a ``LaunchDescription`` that loads graph_watchdog into a real gateway.

    Parameters
    ----------
    detector_params : dict or None
        Extra ``plugins.graph_watchdog.*`` ROS parameters (nested-dotted YAML
        keys, e.g. ``'plugins.graph_watchdog.detectors.param_drift.mode'``),
        merged on top of the ``plugins``/``plugins.graph_watchdog.path``
        parameters this factory always sets.
    extra_gateway_params : dict or None
        Extra ROS parameters for the gateway node itself rather than for the
        plugin (e.g. ``'parameter_service_negative_cache_sec'``). Kept separate
        from `detector_params` so a gateway-scope knob is not filed under a
        plugin-scope name; both end up in the same node config.
    demo_nodes : list of str or None
        Node keys from ``ros2_medkit_test_utils.launch_helpers.DEMO_NODE_REGISTRY``.
        ``None`` means no demo nodes.
    port : int or None
        Gateway HTTP port. ``None`` uses ``get_test_port()`` (the
        ``GATEWAY_TEST_PORT`` env var set per-CTest-target by CMake).
    demo_delay : float
        Seconds to delay demo nodes + fault_manager after gateway start.
    healing_enabled : bool
        Passed to the fault_manager as ``healing_enabled`` so a detector's
        level-triggered PASSED clears can actually advance the debounce
        counter to HEALED (see the plugin README, "Closing the loop").

    Returns
    -------
    tuple[LaunchDescription, dict]
        ``(launch_description, context_dict)`` expected by launch_testing.
        ``context_dict`` maps ``'gateway_node'`` to the gateway Node action.

    """
    if port is None:
        port = get_test_port()

    params = {
        'plugins': ['graph_watchdog'],
        'plugins.graph_watchdog.path': graph_watchdog_plugin_path(),
    }
    if detector_params:
        params.update(detector_params)
    if extra_gateway_params:
        params.update(extra_gateway_params)

    gateway_node = create_gateway_node(port=port, extra_params=params)

    delayed_actions = create_demo_nodes(demo_nodes if demo_nodes is not None else [])
    delayed_actions.append(create_fault_manager_node(
        extra_params={
            'healing_enabled': healing_enabled,
            # AUTOSAR DEM-style debounce: -2 confirms a fault on its very first
            # FAILED report instead of requiring several ticks to accumulate,
            # so the positive-control assertion is fast and deterministic.
            'confirmation_threshold': -2,
        },
    ))

    delayed = TimerAction(period=demo_delay, actions=delayed_actions)

    launch_description = LaunchDescription([
        gateway_node,
        delayed,
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {'gateway_node': gateway_node},
    )


def graph_watchdog_plugin_path():
    """Resolve the built ``libros2_medkit_graph_watchdog.so`` path.

    Unlike the other packages this harness launches (installed and
    ament-index-registered via colcon), ``ros2_medkit_graph_watchdog`` is a
    plugin whose .so is located from the build tree in local/CI builds -
    ``ament_index_python.get_package_prefix()`` cannot resolve it. CMake
    instead injects the exact build-output path via the
    ``GRAPH_WATCHDOG_PLUGIN_PATH`` env var (see ``set_tests_properties`` in
    CMakeLists.txt).

    Returns
    -------
    str
        Absolute path to the built plugin ``.so``.

    Raises
    ------
    RuntimeError
        If ``GRAPH_WATCHDOG_PLUGIN_PATH`` is not set.

    """
    path = os.environ.get('GRAPH_WATCHDOG_PLUGIN_PATH')
    if not path:
        raise RuntimeError(
            'GRAPH_WATCHDOG_PLUGIN_PATH not set - set it via '
            'set_tests_properties(... ENVIRONMENT ...) in CMakeLists.txt to '
            'the built libros2_medkit_graph_watchdog.so path.'
        )
    return path


def wait_until_watchdog_armed(port, timeout=60.0, interval=0.5, app_id=None):
    """Poll ``GET /api/v1/x-medkit-watchdog`` until the plugin is live and armed.

    Any assertion that a fault is ABSENT must gate on this first. Absence is the
    default outcome of a stack that never came up, and nothing else in a watchdog
    launch fails on one: the gateway logs and carries on when a plugin fails to
    dlopen, ``load_plugins``' return count is never checked, and a REST bind
    failure is caught inside the server thread. All of those still exit 0, so the
    post-shutdown exit-code check stays green too, and ``poll_faults`` swallows
    every transport error and returns ``None`` on timeout - exactly what an
    ``assertIsNone`` wants to see.

    This route is registered by the plugin itself (``get_routes()``), and its
    entity list is filled by the tick thread's gate update, so a 200 carrying
    entities proves BOTH that the .so loaded and that the tick loop ran.
    ``global_state == 'armed'`` additionally proves the bringup grace elapsed,
    i.e. the gate would have permitted a raise during the window that follows.

    `app_id` narrows that from "some app is armed" to "THIS app is armed", which
    is what a test that is about to perturb one specific node needs. The gate is
    what decides whether a detector may target an app at all (param_drift builds
    its read set from ``reliability_allows`` - see its tick()), so an armed
    entry is the precondition for the very first read of that node: gating on it
    means a perturbation can no longer land before the detector was even
    permitted to look, and a stack that comes up late fails HERE, naming the
    node, instead of surfacing later as an unexplained missing fault.

    The check mirrors ``ReliabilityGate::allows_raise``: the payload's
    per-entity ``state`` is ``armed`` only when the warmup has elapsed AND the
    lifecycle watcher considers the node ok, which is exactly the conjunction
    that permits a raise. The boolean ``armed`` field alone is only the warmup
    half.

    What this does NOT prove is that a detector has already READ the app - no
    e2e surface exposes that. The plugin's only route reports gate state, and a
    detector's level-triggered PASSED for a fault that does not exist yet is
    dropped by the fault_manager (see ``report_fault_event``), so a clear is
    invisible until something has been raised. Callers that need a captured
    baseline must still allow the sweep a window after this returns.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    timeout : float
        Maximum time to wait in seconds.
    interval : float
        Sleep between retries in seconds.
    app_id : str or None
        App id (as the gateway derives it from the ROS node name) that must be
        present in ``entities`` with ``state == 'armed'``. ``None`` only
        requires some entity plus a global ``armed`` state.

    Returns
    -------
    bool
        ``True`` once the plugin reports a global ``armed`` state together with
        at least one entity (or with `app_id` armed, when given), ``False`` on
        timeout.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    last_seen = 'GET /x-medkit-watchdog was never answered at all'
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/x-medkit-watchdog', timeout=5)
            if response.status_code != 200:
                # Worth naming rather than folding into the generic message: the route is
                # registered by the plugin itself, so a 404 from a gateway that is otherwise
                # answering says the .so never loaded.
                last_seen = f'HTTP {response.status_code} from GET /x-medkit-watchdog'
            else:
                status = response.json().get('x-medkit-watchdog', {})
                last_seen = json.dumps(status)
                entities = status.get('entities') or []
                if app_id is None:
                    armed = bool(entities)
                else:
                    armed = any(
                        e.get('id') == app_id and e.get('state') == 'armed' for e in entities)
                if armed and status.get('global_state') == 'armed':
                    return True
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /x-medkit-watchdog failed: {exc}'
        time.sleep(interval)
    # The caller turns this into an assertion failure, but only it knows what it
    # was waiting for; the gate state itself is the evidence for WHY, and it is
    # gone once the launch tears down. Print it while it still exists.
    print(f'wait_until_watchdog_armed(app_id={app_id!r}) timed out after {timeout}s; '
          f'last watchdog status: {last_seen}')
    return False


def poll_faults(port, code, timeout=30.0, interval=0.5):
    """Poll the GLOBAL ``GET /faults`` endpoint until `code` appears.

    Uses the default (no ``status`` query param) filter, which the gateway
    resolves to pending+confirmed only (active faults) - see
    ``FaultStatusFilter`` defaults in ``http_utils.hpp``.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` to look for (e.g. ``'GRAPH_PARAM_DRIFT'``).
    timeout : float
        Maximum time to wait in seconds.
    interval : float
        Sleep between retries in seconds.

    Returns
    -------
    dict or None
        The matching fault list item, or ``None`` if it never appears
        within `timeout` seconds.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', timeout=5)
            if response.status_code == 200:
                for item in response.json().get('items', []):
                    if item.get('fault_code') == code:
                        return item
        except requests.exceptions.RequestException:
            pass
        time.sleep(interval)
    return None


def poll_entity_faults(port, entity_path, code, timeout=30.0, interval=0.5):
    """Poll a SCOPED fault list (``GET /<entity_path>/faults``) until `code` appears.

    The flat ``/faults`` list carries every fault regardless of scoping, so it cannot
    tell whether a fault is actually reachable from an entity. This can: the gateway
    resolves an entity's scope to a set of reporting-source FQNs and drops any fault
    whose sources are not in it, so a fault raised under an id that owns no scope is
    absent here while still present in ``/faults``.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    entity_path : str
        Entity path below the API base, e.g. ``'apps/graph_watchdog'``.
    code : str
        The ``fault_code`` to look for.
    timeout : float
        Maximum time to wait in seconds.
    interval : float
        Sleep between retries in seconds.

    Returns
    -------
    dict or None
        The matching fault list item, or ``None`` if it never appears.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/{entity_path}/faults', timeout=5)
            if response.status_code == 200:
                for item in response.json().get('items', []):
                    if item.get('fault_code') == code:
                        return item
        except requests.exceptions.RequestException:
            pass
        time.sleep(interval)
    return None


def poll_cleared(port, code, timeout=30.0, interval=0.5):
    """Poll the GLOBAL ``GET /faults`` endpoint until `code` is absent.

    Uses the same default (active-only) query as :func:`poll_faults`, so
    this succeeds once `code` is no longer pending/confirmed - i.e. it has
    been cleared or healed.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` expected to disappear from the active-fault list.
    timeout : float
        Maximum time to wait in seconds.
    interval : float
        Sleep between retries in seconds.

    Returns
    -------
    bool
        ``True`` once `code` is absent from the default query, ``False`` on
        timeout.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', timeout=5)
            if response.status_code == 200:
                codes = {item.get('fault_code') for item in response.json().get('items', [])}
                if code not in codes:
                    return True
        except requests.exceptions.RequestException:
            pass
        time.sleep(interval)
    return False
