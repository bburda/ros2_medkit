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

"""Factory functions for building launch descriptions in integration tests.

Replaces ~500 lines of inline launch boilerplate previously duplicated across
9+ test files. Each test file's ``generate_test_description()`` reduces to a
single ``create_test_launch(...)`` call.

Node names, namespaces, and parameters match ``demo_nodes.launch.py`` exactly.
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions

from ros2_medkit_test_utils.constants import DEFAULT_PORT
from ros2_medkit_test_utils.coverage import get_coverage_env

# ---------------------------------------------------------------------------
# Demo node registry
# ---------------------------------------------------------------------------
# Maps a short node key to the (executable, ros_name, namespace) triple.
# - executable: CMake target name (installed by ros2_medkit_integration_tests)
# - ros_name: ROS 2 node name (the ``name=`` parameter)
# - namespace: ROS 2 namespace (leading ``/`` is added by launch_ros)
#
# These MUST match demo_nodes.launch.py exactly.
# ---------------------------------------------------------------------------

DEMO_NODE_REGISTRY = {
    # Sensors
    'temp_sensor': ('demo_engine_temp_sensor', 'temp_sensor', '/powertrain/engine'),
    # Subscribes to /powertrain/engine/temperature. Pairs with temp_sensor so the
    # demo stack has a real publisher/subscriber edge inside one Function.
    'temp_monitor': ('demo_engine_temp_monitor', 'temp_monitor', '/powertrain/engine'),
    'rpm_sensor': ('demo_rpm_sensor', 'rpm_sensor', '/powertrain/engine'),
    'pressure_sensor': ('demo_brake_pressure_sensor', 'pressure_sensor', '/chassis/brakes'),
    'status_sensor': ('demo_door_status_sensor', 'status_sensor', '/body/door/front_left'),
    'lidar_sensor': ('demo_lidar_sensor', 'lidar_sensor', '/perception/lidar'),
    # Actuators
    'actuator': ('demo_brake_actuator', 'actuator', '/chassis/brakes'),
    'controller': ('demo_light_controller', 'controller', '/body/lights'),
    # Operations (services / actions)
    'calibration': ('demo_calibration_service', 'calibration', '/powertrain/engine'),
    # Two services under one node whose ROS paths differ only above the last
    # segment, so one provider carries the operation short name `calibrate`
    # twice. Its own namespace keeps that collision away from `calibration`.
    'dual_calibration': ('demo_dual_calibration_service', 'dual_calibration', '/testrig/dual'),
    'long_calibration': ('demo_long_calibration_action', 'long_calibration', '/powertrain/engine'),
    # Lifecycle demo (stays unconfigured by default; auto_activate:=true activates it)
    'managed_lifecycle': ('managed_lifecycle', 'managed_lifecycle', ''),
    # Same executable, launched with auto_activate:=true so it self-activates to
    # the "active" lifecycle state (-> status "ready"). Distinct node name so it
    # can run alongside the unconfigured 'managed_lifecycle' in the same test.
    'managed_lifecycle_active': ('managed_lifecycle', 'managed_lifecycle_active', ''),
    # Proof fixture for GRAPH_NODE_UNREADABLE: advertises get_state/change_state like a
    # managed lifecycle node, but get_state never answers until its start_answering
    # parameter is set true (see unreadable_lifecycle_node.cpp's file doc).
    'unreadable_lifecycle': ('unreadable_lifecycle', 'unreadable_lifecycle', ''),
    # Regression fixture (#531): parameter services are discoverable
    # (wait_for_service succeeds) but list_parameters never replies.
    'unresponsive_param': ('demo_unresponsive_param_node', 'unresponsive_param', ''),
}

# Convenience groupings for callers that want subsets of demo nodes.
SENSOR_NODES = ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'status_sensor', 'lidar_sensor']
ACTUATOR_NODES = ['actuator', 'controller']
SERVICE_NODES = ['calibration', 'long_calibration']
ALL_DEMO_NODES = SENSOR_NODES + ACTUATOR_NODES + SERVICE_NODES

# Default lidar parameters that trigger deterministic faults:
# - min_range > max_range -> LIDAR_RANGE_INVALID
# - scan_frequency > 20.0 -> LIDAR_FREQ_UNSUPPORTED
# - is_calibrated=false -> LIDAR_CALIBRATION_REQUIRED
LIDAR_FAULTY_PARAMS = {
    'min_range': 10.0,
    'max_range': 5.0,
    'scan_frequency': 25.0,
    'angular_resolution': 0.5,
}


# ---------------------------------------------------------------------------
# Factory: gateway node
# ---------------------------------------------------------------------------

def create_gateway_node(*, port=DEFAULT_PORT, name='ros2_medkit_gateway',
                        extra_params=None, coverage=True, extra_env=None,
                        respawn=False, respawn_delay=1.0):
    """Create a ``gateway_node`` launch action with standard config.

    Parameters
    ----------
    port : int
        HTTP server port (default: 8080).
    name : str
        ROS node name. Override when a test launches more than one gateway
        so their names do not collide (e.g. ``gateway_with_scripts``).
    extra_params : dict or None
        Additional ROS parameters merged into the node config.
    coverage : bool
        If True, set GCOV_PREFIX env vars for code coverage collection.
    extra_env : dict or None
        Additional environment variables merged into ``additional_env`` on
        top of the coverage env. Useful for setting ``ROS_DOMAIN_ID`` to
        isolate a multi-gateway test's peers into distinct DDS domains.
    respawn : bool
        If True, ``launch`` restarts the gateway whenever it exits before the
        launch itself is shutting down. Only for tests whose subject IS a
        gateway restart (state that must survive one, or must not be
        resurrected by one): a test kills the process by PID and lets launch
        bring the same configuration back. Off by default, so an unexpected
        gateway death stays a visible failure everywhere else.
    respawn_delay : float
        Seconds ``launch`` waits before restarting. Non-zero so the HTTP port
        and the DDS participant are released before the replacement binds
        them, which also gives a test a window in which the port is provably
        down - the only way to tell "restarted" from "never died".

    Returns
    -------
    launch_ros.actions.Node
        Ready-to-use gateway node launch action.

    """
    params = {'refresh_interval_ms': 1000, 'server.port': port}
    if extra_params:
        params.update(extra_params)

    env = dict(get_coverage_env() if coverage else {})
    if extra_env:
        env.update(extra_env)

    return launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name=name,
        output='screen',
        parameters=[params],
        additional_env=env,
        respawn=respawn,
        respawn_delay=respawn_delay,
        # Default SIGINT->SIGTERM escalation is 5s and SIGTERM->SIGKILL is 5s.
        # Under TSan/ASan/coverage the gateway shutdown sequence (mdns stop,
        # REST server stop, transport teardown, plugin shutdown, plus flushing
        # gcov data) easily exceeds 5s on slower CI runners, causing
        # launch_testing to escalate to SIGKILL and the
        # TestShutdown.test_exit_codes check to report -9. The process still
        # shuts down cleanly given enough time, so widen both windows.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


# ---------------------------------------------------------------------------
# Factory: fault manager node
# ---------------------------------------------------------------------------

def create_fault_manager_node(
    *,
    name='fault_manager',
    storage_type='memory',
    rosbag_enabled=True,
    rosbag_topics=None,
    snapshot_topics=None,
    extra_params=None,
    coverage=True,
    extra_env=None,
):
    """Create a ``fault_manager_node`` with test-friendly defaults.

    Parameters
    ----------
    name : str
        ROS node name. Override when a test launches more than one fault
        manager so their names do not collide.
    storage_type : str
        Storage backend: 'memory' (default, avoids filesystem issues in CI)
        or 'sqlite'.
    rosbag_enabled : bool
        Enable rosbag snapshot capture (default: True).
    rosbag_topics : list of str or None
        Explicit topic list for rosbag capture. Defaults to
        ``['/perception/lidar/scan']``.
    snapshot_topics : list of str or None
        Default topics for freeze_frame snapshot capture. Defaults to
        ``['/perception/lidar/scan']``.
    extra_params : dict or None
        Additional ROS parameters merged into the node config.
        Useful for overriding fault manager settings like
        ``confirmation_threshold``.
    coverage : bool
        If True, set GCOV_PREFIX env vars for code coverage collection.
    extra_env : dict or None
        Additional environment variables merged into ``additional_env`` on
        top of the coverage env (e.g. ``ROS_DOMAIN_ID`` for a peer).

    Returns
    -------
    launch_ros.actions.Node
        Ready-to-use fault manager node launch action.

    """
    if rosbag_topics is None:
        rosbag_topics = ['/perception/lidar/scan']
    if snapshot_topics is None:
        snapshot_topics = ['/perception/lidar/scan']

    params = {
        'storage_type': storage_type,
        'snapshots.default_topics': snapshot_topics,
        'snapshots.rosbag.enabled': rosbag_enabled,
        'snapshots.rosbag.duration_sec': 0.1,
        'snapshots.rosbag.duration_after_sec': 0.1,
        'snapshots.rosbag.topics': 'explicit',
        'snapshots.rosbag.include_topics': rosbag_topics,
    }
    if extra_params:
        params.update(extra_params)

    # Route this process's .gcda to the fault_manager build dir, not the
    # gateway default, so its coverage is not written to the wrong package.
    env = dict(get_coverage_env('ros2_medkit_fault_manager') if coverage else {})
    if extra_env:
        env.update(extra_env)

    return launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name=name,
        output='screen',
        additional_env=env,
        parameters=[params],
        # Same rationale as create_gateway_node: under coverage the node must
        # flush gcov data at shutdown, which can exceed the 5s launch default
        # and get the process SIGKILLed to -9. Widen both windows.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


# ---------------------------------------------------------------------------
# Factory: greenwave_monitor node
# ---------------------------------------------------------------------------

def create_greenwave_node(
    *,
    monitored_topics=None,
    frequency_monitored_topics=None,
    time_check_preset='header_with_nodetime_fallback',
    name='greenwave_monitor',
    extra_params=None,
):
    """Create an NVIDIA ``greenwave_monitor`` node for real ``/diagnostics`` metrics.

    ``greenwave_monitor`` resolves its monitored topics ONCE at node
    startup and never re-resolves them: if a topic has no live publisher
    yet when this node starts, greenwave logs "No topics to monitor" (or
    simply never reports on that topic) and nothing is ever emitted for
    it afterwards. Callers MUST schedule this node's launch action on a
    LATER ``TimerAction`` than the publishers of every topic it should
    monitor - see ``create_demo_nodes``'s ``TimerAction`` usage for the
    earlier stage and give this node its own, later-firing timer.

    ``greenwave_monitor`` also publishes ``/diagnostics`` at only ~1 Hz,
    with the first useful (non-transient) reading typically a few seconds
    after it starts, so callers polling for its output need budgets of at
    least 30 seconds, not the usual 15.

    Parameters
    ----------
    monitored_topics : list of str or None
        Fully-qualified topic names (leading ``/``) for the simple
        ``gw_monitored_topics`` parameter (no expected-frequency
        tracking). Omitted entirely from the parameter set when empty -
        an empty list parameter can crash ``launch`` on type inference.
        greenwave stamps ``DiagnosticStatus.name`` with exactly this
        string, and the graph provider plugin keys its metrics map on it,
        so these must match the ROS topic names byte-for-byte.
    frequency_monitored_topics : dict or None
        Topic -> ``{'expected_frequency': float, 'tolerance': float}`` for
        the ``gw_frequency_monitored_topics`` parameter. When set here, a
        topic's ``expected_frequency`` is stamped into its ``/diagnostics``
        status and wins over any graph-provider-side config (function-level
        override or global default) for that topic. Omitted entirely when
        empty, for the same launch-type-inference reason as above.
    time_check_preset : str
        ``gw_time_check_preset`` value (default matches greenwave's own
        example config: check header timestamp with a node-time fallback).
    name : str
        ROS node name. With no namespace, the node's fully-qualified name
        is ``/<name>`` - this is also the value the graph provider plugin
        resolves into ``metrics.source`` for every topic this node reports
        on (via publisher GID matching against ``/diagnostics``).
    extra_params : dict or None
        Additional ROS parameters merged into the node config.

    Returns
    -------
    launch_ros.actions.Node
        Ready-to-use ``greenwave_monitor`` node launch action.

    """
    params = {'gw_time_check_preset': time_check_preset}
    if monitored_topics:
        params['gw_monitored_topics'] = monitored_topics
    if frequency_monitored_topics:
        params['gw_frequency_monitored_topics'] = frequency_monitored_topics
    if extra_params:
        params.update(extra_params)

    return launch_ros.actions.Node(
        package='greenwave_monitor',
        executable='greenwave_monitor',
        name=name,
        output='screen',
        parameters=[params],
    )


# ---------------------------------------------------------------------------
# Factory: demo nodes
# ---------------------------------------------------------------------------

def create_demo_nodes(nodes=None, *, lidar_faulty=True, coverage=True,
                      extra_env=None):
    """Create demo node launch actions.

    Parameters
    ----------
    nodes : list of str or None
        Node keys from ``DEMO_NODE_REGISTRY``. ``None`` means
        ``ALL_DEMO_NODES``.
    lidar_faulty : bool
        If True (default), launch ``lidar_sensor`` with invalid parameters
        that trigger deterministic faults. If False, launch with defaults.
    coverage : bool
        If True, set GCOV_PREFIX env vars for code coverage collection.
    extra_env : dict or None
        Additional environment variables merged into each node's
        ``additional_env``. Useful for setting ``ROS_DOMAIN_ID`` to
        isolate nodes into a specific DDS domain.

    Returns
    -------
    list of launch_ros.actions.Node
        Ordered list of demo node launch actions.

    Raises
    ------
    KeyError
        If a node key is not in ``DEMO_NODE_REGISTRY``.

    """
    if nodes is None:
        nodes = ALL_DEMO_NODES

    # Demo executables are built in ros2_medkit_integration_tests, so route
    # their .gcda there instead of polluting the gateway build dir.
    env = dict(get_coverage_env('ros2_medkit_integration_tests') if coverage else {})
    if extra_env:
        env.update(extra_env)
    actions = []

    for key in nodes:
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[key]

        node_kwargs = {
            'package': 'ros2_medkit_integration_tests',
            'executable': executable,
            'name': ros_name,
            'namespace': namespace,
            'output': 'screen',
            'additional_env': env,
            # Give the node room to flush coverage data at shutdown before
            # SIGKILL, matching the gateway/fault_manager helpers.
            'sigterm_timeout': '30',
            'sigkill_timeout': '15',
        }

        # Apply faulty parameters for lidar_sensor
        if key == 'lidar_sensor' and lidar_faulty:
            node_kwargs['parameters'] = [LIDAR_FAULTY_PARAMS]

        # The activated lifecycle node self-configures + activates on start.
        if key == 'managed_lifecycle_active':
            node_kwargs['parameters'] = [{'auto_activate': True}]

        actions.append(launch_ros.actions.Node(**node_kwargs))

    return actions


# ---------------------------------------------------------------------------
# Gateway parameter presets
# ---------------------------------------------------------------------------

def graph_provider_params():
    """Gateway parameters that load the graph provider plugin.

    The only in-tree plugin that exports ``describe_plugin_routes``, so it is
    what any test about plugin-served routes appearing in the OpenAPI document
    has to load. Its ``.so`` path is resolved here, in one place, rather than
    at each fixture.

    Returns
    -------
    dict
        Parameter overrides for ``create_gateway_node(extra_params=...)``.

    """
    graph_plugin = os.path.join(
        get_package_prefix('ros2_medkit_graph_provider'), 'lib',
        'ros2_medkit_graph_provider', 'libros2_medkit_graph_provider_plugin.so')
    return {
        'plugins': ['graph_provider'],
        'plugins.graph_provider.path': graph_plugin,
    }


def full_feature_gateway_params(scripts_dir):
    """Gateway parameters that turn on every optional feature gate.

    The OpenAPI contract test asserts over the maximal route surface, so
    scripts, updates, triggers, locking and the graph provider plugin must all
    be live. Without this the gated routes are absent and assertions about them
    pass vacuously.

    Authentication is deliberately NOT enabled here. The document a gateway
    serves describes that gateway, so the per-operation ``security`` a plugin
    declares is stripped while ``auth.enabled`` is false - which makes this
    fixture the auth-off half of that pair. The auth-on half is
    ``test_auth.test.py``.

    Parameters
    ----------
    scripts_dir : str
        Directory for uploaded diagnostic scripts. A non-empty value is what
        enables the scripts feature.

    Returns
    -------
    dict
        Parameter overrides for ``create_test_launch(gateway_params=...)``.

    """
    return {
        'updates.enabled': True,
        'scripts.scripts_dir': scripts_dir,
        'triggers.enabled': True,
        'locking.enabled': True,
        **graph_provider_params(),
    }


# ---------------------------------------------------------------------------
# Factory: complete test launch description
# ---------------------------------------------------------------------------

def create_test_launch(
    *,
    port=DEFAULT_PORT,
    demo_nodes=None,
    gateway_params=None,
    fault_manager=True,
    fault_manager_params=None,
    lidar_faulty=True,
    demo_delay=2.0,
):
    """Build a complete ``LaunchDescription`` for an integration test.

    This is the primary entry point. Each test file's
    ``generate_test_description()`` delegates to this function.

    Parameters
    ----------
    port : int
        Gateway HTTP port (default: 8080).
    demo_nodes : list of str or None
        Node keys from ``DEMO_NODE_REGISTRY``. ``None`` means all nodes.
    gateway_params : dict or None
        Extra ROS parameters for the gateway node.
    fault_manager : bool
        If True (default), include the fault manager node.
    fault_manager_params : dict or None
        Extra ROS parameters for the fault manager node (e.g.
        ``{'confirmation_threshold': -2}`` for debounce tuning).
    lidar_faulty : bool
        If True (default), launch lidar_sensor with fault-triggering params.
    demo_delay : float
        Seconds to delay demo nodes after gateway start (default: 2.0).

    Returns
    -------
    tuple[LaunchDescription, dict]
        ``(launch_description, context_dict)`` tuple expected by
        ``launch_testing``. ``context_dict`` maps ``'gateway_node'`` to the
        gateway ``Node`` action.

    """
    gateway_node = create_gateway_node(
        port=port,
        extra_params=gateway_params,
    )

    delayed_actions = create_demo_nodes(
        demo_nodes,
        lidar_faulty=lidar_faulty,
    )

    if fault_manager:
        delayed_actions.append(
            create_fault_manager_node(extra_params=fault_manager_params)
        )

    delayed = TimerAction(
        period=demo_delay,
        actions=delayed_actions,
    )

    launch_description = LaunchDescription([
        gateway_node,
        delayed,
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {'gateway_node': gateway_node},
    )
