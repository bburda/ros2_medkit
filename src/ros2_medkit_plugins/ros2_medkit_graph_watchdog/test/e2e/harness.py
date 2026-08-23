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

import http.server
import json
import os
import threading
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
    healing_threshold=3,
    gateway_respawn=False,
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
    healing_threshold : int
        Passed to the fault_manager as ``healing_threshold`` - the debounce
        counter value a fault's consecutive PASSED reports must reach before
        it heals. Defaults to 3, the fault_manager's own built-in default
        (see ``FaultManagerNode``'s ``declare_parameter``), so passing it
        through here changes no existing scenario's behaviour. A scenario
        whose subject IS the debounce counter (a low threshold makes it
        sensitive to even a single spurious PASSED) overrides this directly
        rather than adding a second mechanism.
    gateway_respawn : bool
        If True, ``launch`` restarts the gateway when it exits. For the one
        scenario whose subject is a gateway restart: the fault_manager and the
        demo nodes are separate processes, so killing only the gateway leaves
        a raised fault in the store and brings the plugin back with every
        detector counter at zero - the state no single-process launch can
        reach. Off by default so an unexpected gateway death stays a visible
        failure in every other scenario.

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

    gateway_node = create_gateway_node(port=port, extra_params=params, respawn=gateway_respawn)

    delayed_actions = create_demo_nodes(demo_nodes if demo_nodes is not None else [])
    delayed_actions.append(create_fault_manager_node(
        extra_params={
            'healing_enabled': healing_enabled,
            'healing_threshold': healing_threshold,
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


def watchdog_detector_status(port, detector_id, timeout=5.0):
    """Read one detector's own status block from ``GET /x-medkit-watchdog``.

    The route's payload carries the reliability gate's state plus, under ``detectors``, a
    block per detector that has something to say about itself. A detector-scoped condition
    (the lifecycle_expectation tracked-node cap being saturated, say) is otherwise visible
    only in the gateway's log, which no assertion at this tier can read.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    detector_id : str
        The detector's own ``id()``, e.g. ``'lifecycle_expectation'``.
    timeout : float
        Per-request timeout in seconds.

    Returns
    -------
    dict or None
        The detector's status block, or ``None`` when the route did not answer 200 or
        carries no block for that detector.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    try:
        response = requests.get(f'{base}/x-medkit-watchdog', timeout=timeout)
    except requests.exceptions.RequestException:
        return None
    if response.status_code != 200:
        return None
    status = response.json().get('x-medkit-watchdog', {})
    return (status.get('detectors') or {}).get(detector_id)


def poll_detector_status(port, detector_id, field, expected, timeout=30.0, interval=0.5):
    """Poll ``GET /x-medkit-watchdog`` until a detector status field equals `expected`.

    Returns ``True`` once it does, ``False`` on timeout (after printing what was last seen,
    which is gone once the launch tears down).
    """
    deadline = time.monotonic() + timeout
    last_seen = 'no detectors block was ever returned'
    while time.monotonic() < deadline:
        block = watchdog_detector_status(port, detector_id)
        if block is not None:
            last_seen = json.dumps(block)
            if block.get(field) == expected:
                return True
        time.sleep(interval)
    print(f'poll_detector_status({detector_id!r}, {field!r}=={expected!r}) timed out after '
          f'{timeout}s; last seen: {last_seen}')
    return False


def wait_until_faults_endpoint_live(port, timeout=30.0, interval=0.5):
    """Poll ``GET /faults`` until it answers HTTP 200. ``True`` once it does.

    The second half of the gate every absence assertion needs.
    :func:`wait_until_watchdog_armed` proves the plugin side is alive - the .so loaded,
    the tick loop ran - but it is served by the plugin INSIDE the gateway process and
    says nothing about the fault surface. The gateway answers ``GET /faults`` with 503
    while the fault_manager's service is unavailable, and :func:`poll_faults` inspects
    only 200 responses and swallows every transport error, returning ``None`` on timeout -
    which is exactly what an ``assertIsNone`` wants to see. So a launch whose
    fault_manager crashed, hung, or never DDS-matched turns a silence assertion green for
    the one reason it must never be green.

    A 200 (whatever the body) proves the gateway reached the fault_manager in THIS launch.
    What it does NOT prove is that the PLUGIN's own ReportFault client matched - only a
    raise proves that, and that proof lives in the scenario that raises.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    timeout : float
        Maximum time to wait in seconds.
    interval : float
        Sleep between retries in seconds.

    Returns
    -------
    bool
        ``True`` once ``GET /faults`` answers 200, ``False`` on timeout.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    last_seen = 'GET /faults was never answered at all'
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', timeout=5)
            if response.status_code == 200:
                return True
            # 503 is the specific shape of "the fault_manager is not reachable" - worth
            # naming, because it is the failure this gate exists for.
            last_seen = f'HTTP {response.status_code} from GET /faults'
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /faults failed: {exc}'
        time.sleep(interval)
    print(f'wait_until_faults_endpoint_live timed out after {timeout}s; last seen: {last_seen}')
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


def poll_fault_describing(port, code, needles, timeout=60.0, interval=0.5):
    """Poll the GLOBAL ``GET /faults`` until `code` is present AND names every needle.

    :func:`poll_faults` returns the moment the code appears, which is not the
    same thing as the fault being complete. An aggregating detector raises as
    soon as its FIRST subject crosses the persistence gate and then rewrites the
    description on later ticks as more subjects cross it. A test that wants two
    subjects named has to wait for the second one, or it reads whichever
    description happened to be current and fails on a slower runner.

    Note the description is capped (``kMaxDescriptionChars``), so this can also
    time out because the text was truncated rather than because a subject was
    missing. The returned description lets the caller say which.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` to look for.
    needles : iterable of str
        Substrings that must all appear in the fault description.
    timeout : float
        Maximum time to wait in seconds.
    interval : float
        Sleep between retries in seconds.

    Returns
    -------
    tuple of (bool, str)
        ``(True, description)`` once every needle is present, otherwise
        ``(False, last_description)`` on timeout - empty if the code never
        appeared at all.

    """
    deadline = time.monotonic() + timeout
    last_description = ''
    while time.monotonic() < deadline:
        fault = poll_faults(port, code, timeout=interval, interval=interval)
        if fault is None:
            # Nothing to see. poll_faults already slept `interval` before
            # giving up, so sleeping again here would halve the sampling rate.
            # Drop what we saw earlier too: the fault cleared or never
            # appeared, and a timeout must not report a stale description.
            last_description = ''
            continue
        last_description = fault.get('description', '')
        if all(needle in last_description for needle in needles):
            return True, last_description
        # Present but incomplete - the case this helper exists for. poll_faults
        # returns the moment it matches the code, BEFORE its own sleep, so this
        # branch has to pace itself or it becomes a bare retry loop hammering
        # GET /faults for the whole timeout, against the gateway whose detector
        # we are waiting on.
        time.sleep(interval)
    return False, last_description


def assert_fault_absent_throughout(test_case, port, code, duration, interval=0.5):
    """Poll ``GET /faults`` every `interval` across the WHOLE `duration`.

    Fails the moment either the channel or the claim breaks.
    ``poll_faults`` is built for "wait until X appears" and is unsuitable for a silence
    proof: it swallows every non-200 response and every transport error and returns
    ``None`` on timeout - exactly what ``assertIsNone`` wants to see, so a ``/faults``
    that died three seconds into a twenty-second silence window still passes. This
    instead asks the whole way through: every single poll must answer 200 ("asked, and
    there is no such fault") or the assertion fails naming which poll and why ("could not
    ask"), and `code` must never appear in any of them. Use this (not
    ``assertIsNone(poll_faults(...))``) for every scenario whose claim is sustained
    silence over a window, not merely "absent right now".

    Parameters
    ----------
    test_case : unittest.TestCase
        Used for the actual assertion calls, so a failure here reports through the normal
        unittest failure path rather than a bare ``AssertionError`` from a free function.
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` that must never appear.
    duration : float
        Total seconds to keep polling.
    interval : float
        Sleep between polls in seconds.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + duration
    polls = 0
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', timeout=5)
        except requests.exceptions.RequestException as exc:
            test_case.fail(
                f'/faults became unreachable {polls} poll(s) into a {duration}s silence '
                f'window (could not ask, which is not the same as "asked, and there is no '
                f'such fault"): {exc}')
            return
        if response.status_code != 200:
            test_case.fail(
                f'/faults answered HTTP {response.status_code} {polls} poll(s) into a '
                f'{duration}s silence window - the channel died mid-window, which a '
                'silence assertion must not read as "no such fault"')
            return
        codes = {item.get('fault_code') for item in response.json().get('items', [])}
        test_case.assertNotIn(
            code, codes,
            f'{code} appeared {polls} poll(s) into a {duration}s window that was supposed '
            'to stay silent')
        polls += 1
        time.sleep(interval)
    test_case.assertGreater(
        polls, 0,
        f'the {duration}s silence window never actually polled /faults - duration must be '
        f'>= interval ({interval}s)')


def assert_fault_persists_throughout(test_case, port, code, duration, interval=0.5):
    """Fail unless `code` is present on EVERY poll for `duration` seconds.

    The mirror of assert_fault_absent_throughout, and it owes the same proof: a fault
    surface that stops answering must fail the assertion rather than satisfy it. Poll the
    same endpoint the same way; a request error, a non-200, or a body that does not parse
    is a failure, not a skipped sample. Waiting for a single sighting
    (``assertIsNotNone(poll_faults(...))``) proves the fault appeared once, not that it
    stayed - it returns on the first match and says nothing about a channel that goes dark
    on poll three of twenty. Use this for every scenario whose claim is sustained presence
    over a window, not merely "present right now".

    Parameters
    ----------
    test_case : unittest.TestCase
        Used for the actual assertion calls, so a failure here reports through the normal
        unittest failure path rather than a bare ``AssertionError`` from a free function.
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` that must be present on every poll.
    duration : float
        Total seconds to keep polling.
    interval : float
        Sleep between polls in seconds.

    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + duration
    polls = 0
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', timeout=5)
        except requests.exceptions.RequestException as exc:
            test_case.fail(
                f'/faults became unreachable {polls} poll(s) into a {duration}s persistence '
                f'window (could not ask, which is not the same as "asked, and {code} is '
                f'still there"): {exc}')
            return
        if response.status_code != 200:
            test_case.fail(
                f'/faults answered HTTP {response.status_code} {polls} poll(s) into a '
                f'{duration}s persistence window - the channel died mid-window, which this '
                'assertion must not read as "still there"')
            return
        codes = {item.get('fault_code') for item in response.json().get('items', [])}
        test_case.assertIn(
            code, codes,
            f'{code} was missing {polls} poll(s) into a {duration}s window that was supposed '
            'to stay confirmed the whole way through')
        polls += 1
        time.sleep(interval)
    test_case.assertGreater(
        polls, 0,
        f'the {duration}s persistence window never actually polled /faults - duration must '
        f'be >= interval ({interval}s)')


def assert_fault_describes_only(
        test_case, port, code, required, forbidden, duration, interval=0.5):
    """Fail unless `code` is present with a matching description on every poll.

    On every poll for `duration` seconds, `code` must be present and its description must
    contain every needle in `required` and none in `forbidden`. `required` and `forbidden`
    are iterables of plain substrings. The aggregated fault
    names several entities in one description, so this is how a scenario says "this node is
    named and that one is not" without depending on ordering. Polls the whole window, not a
    single sample: a description this assertion must hold true FOR THE DURATION (one node
    named, a sibling never named) is exactly the kind of claim a channel that goes dark
    mid-window can falsely satisfy if only checked once.

    Parameters
    ----------
    test_case : unittest.TestCase
        Used for the actual assertion calls, so a failure here reports through the normal
        unittest failure path rather than a bare ``AssertionError`` from a free function.
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` that must be present, with a matching description, on every poll.
    required : iterable of str
        Substrings that must all appear in the fault's description on every poll.
    forbidden : iterable of str
        Substrings that must never appear in the fault's description on any poll.
    duration : float
        Total seconds to keep polling.
    interval : float
        Sleep between polls in seconds.

    """
    required = list(required)
    forbidden = list(forbidden)
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + duration
    polls = 0
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', timeout=5)
        except requests.exceptions.RequestException as exc:
            test_case.fail(
                f'/faults became unreachable {polls} poll(s) into a {duration}s description '
                f'window (could not ask, which is not the same as "asked, and the '
                f'description still holds"): {exc}')
            return
        if response.status_code != 200:
            test_case.fail(
                f'/faults answered HTTP {response.status_code} {polls} poll(s) into a '
                f'{duration}s description window - the channel died mid-window, which this '
                'assertion must not read as "the description still holds"')
            return
        items = {item.get('fault_code'): item for item in response.json().get('items', [])}
        fault = items.get(code)
        if fault is None:
            test_case.fail(
                f'{code} was missing {polls} poll(s) into a {duration}s window that was '
                'supposed to keep describing it')
        description = fault.get('description', '')
        for needle in required:
            test_case.assertIn(
                needle, description,
                f"{code}'s description dropped required text {needle!r} {polls} poll(s) "
                f'into a {duration}s window: {description!r}')
        for needle in forbidden:
            test_case.assertNotIn(
                needle, description,
                f"{code}'s description carries forbidden text {needle!r} {polls} poll(s) "
                f'into a {duration}s window: {description!r}')
        polls += 1
        time.sleep(interval)
    test_case.assertGreater(
        polls, 0,
        f'the {duration}s description window never actually polled /faults - duration must '
        f'be >= interval ({interval}s)')


def assert_fault_never_names(test_case, port, code, forbidden, duration, interval=0.5):
    """Fail if `code`'s description EVER names a forbidden needle, whatever `code` itself does.

    Distinct from the three assertions above in what it does NOT require: it takes no position
    on whether `code` is raised at all. ``assert_fault_absent_throughout`` is too strong for a
    claim like "no disappearance names THIS node" - a correct detector raising `code` for some
    OTHER entity would trip it for a reason that has nothing to do with the node under test.
    ``assert_fault_describes_only`` is too strong the other way: it REQUIRES `code` present on
    every poll, which is wrong for a scenario where the code staying fully absent is itself a
    correct outcome. This is the narrower claim in between: whenever `code` happens to be
    present, on any poll, none of `forbidden` may appear in its description; `code` being absent
    on a given poll is not a failure.

    Same channel-alive discipline as the other window assertions: a request error or a non-200
    is a failure naming which poll and why, never a silently-skipped sample.

    Parameters
    ----------
    test_case : unittest.TestCase
        Used for the actual assertion calls, so a failure here reports through the normal
        unittest failure path rather than a bare ``AssertionError`` from a free function.
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` whose description must never name a forbidden needle. Its own
        presence or absence is not judged.
    forbidden : iterable of str
        Substrings that must never appear in `code`'s description, on any poll where `code` is
        present.
    duration : float
        Total seconds to keep polling.
    interval : float
        Sleep between polls in seconds.

    """
    forbidden = list(forbidden)
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + duration
    polls = 0
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', timeout=5)
        except requests.exceptions.RequestException as exc:
            test_case.fail(
                f'/faults became unreachable {polls} poll(s) into a {duration}s window (could '
                f'not ask, which is not the same as "asked, and {code} never named a forbidden '
                f'entity"): {exc}')
            return
        if response.status_code != 200:
            test_case.fail(
                f'/faults answered HTTP {response.status_code} {polls} poll(s) into a '
                f'{duration}s window - the channel died mid-window, which this assertion must '
                'not read as "never named a forbidden entity"')
            return
        items = {item.get('fault_code'): item for item in response.json().get('items', [])}
        fault = items.get(code)
        if fault is not None:
            description = fault.get('description', '')
            for needle in forbidden:
                test_case.assertNotIn(
                    needle, description,
                    f"{code}'s description named forbidden text {needle!r} {polls} poll(s) "
                    f'into a {duration}s window: {description!r}')
        polls += 1
        time.sleep(interval)
    test_case.assertGreater(
        polls, 0,
        f'the {duration}s window never actually polled /faults - duration must be >= interval '
        f'({interval}s)')


class _FlakyFaultsHandler(http.server.BaseHTTPRequestHandler):
    """Stands in for a gateway whose ``GET /faults`` answers normally, then dies.

    Answers 200 with ``{'items': healthy_items}`` for the first ``healthy_polls`` requests
    to ``{API_BASE_PATH}/faults`` (both class attributes, set per instantiation via
    `_FlakyFaultsServer`), then drops the connection with no response at all for every
    request after that - the same "channel gone" shape a crashed or hung fault_manager
    produces, distinct from an ordinary 503 (also exercised via ``dead_status``).
    `healthy_items` defaults to empty (a healthy-but-silent fault surface, what the
    absence proof needs); the persistence and describes-only proofs set it to a list
    holding the fault they poll for, so the same handler can stand in for a surface that
    is healthy AND SAYING SOMETHING before it goes dark.
    """

    healthy_polls = 2
    dead_status = None  # None = drop the connection; an int = answer with that status instead
    healthy_items = []  # the `items` list served while healthy

    def do_GET(self):
        if self.path != f'{API_BASE_PATH}/faults':
            self.send_response(404)
            self.end_headers()
            return
        type(self).seen = getattr(type(self), 'seen', 0) + 1
        if type(self).seen > type(self).healthy_polls:
            if type(self).dead_status is None:
                self.close_connection = True  # drop it: no response at all
                return
            self.send_response(type(self).dead_status)
            self.end_headers()
            return
        body = json.dumps({'items': type(self).healthy_items}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):  # noqa: A002 - matches the base class's own name
        pass  # keep test output quiet - this is expected traffic, not diagnostics


class _FlakyFaultsServer:
    """Context manager for a local `_FlakyFaultsHandler` HTTP server.

    Starts on a free port in a daemon thread and tears itself down on exit - the
    boilerplate every leg of `prove_silence_proof_catches_a_dead_fault_surface` (and its
    persistence/describes-only siblings) below needs, factored out once so each leg reads
    as the claim it is checking rather than server plumbing.
    """

    def __init__(self, healthy_polls, dead_status, healthy_items=None):
        handler = type(
            '_Handler', (_FlakyFaultsHandler,),
            {'healthy_polls': healthy_polls, 'dead_status': dead_status,
             'healthy_items': [] if healthy_items is None else healthy_items})
        self._server = http.server.HTTPServer(('127.0.0.1', 0), handler)
        self.port = self._server.server_address[1]
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc_info):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=5)


def prove_silence_proof_catches_a_dead_fault_surface(test_case):
    """Prove `assert_fault_absent_throughout` catches a `/faults` that dies mid-window.

    The test of the test - the exact hole `assertIsNone(poll_faults(...))` could not
    see (see `assert_fault_absent_throughout`'s own docstring). Self-contained: a real
    local HTTP server stands in for the gateway, answering normally for the first
    couple of polls and then going dark, so this needs no ROS graph, no real gateway,
    no fault_manager - only a real socket, so the failure mode under test (a channel
    that goes silent) is real rather than mocked away.

    Three things are proven per dead-channel shape, all required to make the RED
    result mean something: (1) the OLD pattern this fix replaces - `poll_faults` +
    `assertIsNone` - actually DOES pass against this exact dead server (the concrete
    "previously passed" the brief asks this to demonstrate, not merely asserted in
    prose); (2) `assert_fault_absent_throughout` raises against the SAME server
    (`dead_status=None`, a dropped connection, and again `dead_status=503`, the
    specific shape a real `/faults` gives while the fault_manager is unreachable - a
    helper that only caught one dead-channel shape would leave the other exactly as
    blind as `poll_faults` always was); (3), after the loop below, that the fixed
    helper does NOT raise while the channel never dies - a helper wired to always fail
    would make the RED result meaningless.

    Raises whatever `test_case`'s own assertions raise on failure; callers use it from
    inside a test method exactly like any other assertion helper.
    """
    for dead_status, label in ((None, 'a dropped connection'), (503, 'a 503 response')):
        with _FlakyFaultsServer(healthy_polls=2, dead_status=dead_status) as fake:
            # (1) The OLD pattern this fix replaces: prove it actually passes on this
            # exact dead channel - the concrete "previously passed" this test exists
            # to show, not merely asserted in prose.
            test_case.assertIsNone(
                poll_faults(fake.port, 'GRAPH_NODE_INACTIVE', timeout=2.0, interval=0.2),
                f'the OLD pattern (poll_faults + assertIsNone) did NOT pass against a '
                f'/faults that died via {label} - this self-test no longer '
                'demonstrates the hole the fix closes')
            # (2) The fixed helper must fail against the identical dead channel.
            with test_case.assertRaises(
                    AssertionError,
                    msg=f'assert_fault_absent_throughout did not fail when /faults '
                        f'died mid-window via {label} - it would pass on the exact '
                        f'channel-death this fix exists to catch'):
                assert_fault_absent_throughout(
                    test_case, fake.port, 'GRAPH_NODE_INACTIVE', duration=2.0, interval=0.2)

    # (3) The companion proof: a channel that never dies must not trip the assertion,
    # or the RED result above would be meaningless (a helper wired to always fail
    # "catches" everything by never being usable).
    with _FlakyFaultsServer(healthy_polls=10_000, dead_status=None) as fake:
        assert_fault_absent_throughout(
            test_case, fake.port, 'GRAPH_NODE_INACTIVE', duration=1.0, interval=0.2)


def prove_persistence_proof_catches_a_dead_fault_surface(test_case):
    """Prove `assert_fault_persists_throughout` catches a `/faults` that dies mid-window.

    The mirror of `prove_silence_proof_catches_a_dead_fault_surface`, for the opposite
    claim: a fault that must stay CONFIRMED for a whole window, not one that must stay
    absent. Self-contained, same stand-in server, same three-part proof:

    (1) the naive pattern this fix replaces - `poll_faults` + `assertIsNotNone` - actually
    DOES pass against a `/faults` that answers healthily a couple of times and then dies:
    it returns the moment `code` is first seen, before the channel goes dark, so it never
    observes the death at all; (2) `assert_fault_persists_throughout` raises against the
    SAME dying channel, for both dead-channel shapes (a dropped connection and a 503); (3)
    a channel that never dies, and keeps answering `code` present the whole time, does NOT
    trip the fixed helper - otherwise the RED result above would be meaningless.

    Raises whatever `test_case`'s own assertions raise on failure; callers use it from
    inside a test method exactly like any other assertion helper.
    """
    code = 'GRAPH_NODE_INACTIVE'
    healthy_items = [{'fault_code': code, 'description': f'{code} present'}]
    for dead_status, label in ((None, 'a dropped connection'), (503, 'a 503 response')):
        with _FlakyFaultsServer(healthy_polls=2, dead_status=dead_status,
                                healthy_items=healthy_items) as fake:
            # (1) The OLD pattern this fix replaces: prove it actually passes on this
            # exact dying channel - it returns on the first sighting, before the death.
            test_case.assertIsNotNone(
                poll_faults(fake.port, code, timeout=2.0, interval=0.2),
                f'the OLD pattern (poll_faults + assertIsNotNone) did NOT pass against a '
                f'/faults that later died via {label} - this self-test no longer '
                'demonstrates the hole the fix closes')
            # (2) The fixed helper must fail against the identical dying channel.
            with test_case.assertRaises(
                    AssertionError,
                    msg=f'assert_fault_persists_throughout did not fail when /faults died '
                        f'mid-window via {label} - it would pass on the exact '
                        'channel-death this fix exists to catch'):
                assert_fault_persists_throughout(
                    test_case, fake.port, code, duration=2.0, interval=0.2)

    # (3) The companion proof: a channel that never dies, and keeps answering `code`
    # present, must not trip the assertion, or the RED result above would be meaningless.
    with _FlakyFaultsServer(healthy_polls=10_000, dead_status=None,
                            healthy_items=healthy_items) as fake:
        assert_fault_persists_throughout(test_case, fake.port, code, duration=1.0, interval=0.2)


def prove_describes_only_proof_catches_a_dead_fault_surface(test_case):
    """Prove `assert_fault_describes_only` catches a `/faults` that dies mid-window.

    Same shape as the silence and persistence self-tests above, for the third claim this
    package's scenarios rely on: a description that must hold ("names alpha, never beta")
    for a whole window, not merely on one lucky read before the channel goes dark.

    (1) `poll_fault_describing` (the existing single-shot reader) DOES pass against a
    `/faults` that answers correctly a couple of times and then dies - it returns the
    moment the description matches, never observing the death; (2)
    `assert_fault_describes_only` raises against the SAME dying channel, for both
    dead-channel shapes; (3) a channel that never dies, and keeps describing the fault
    correctly the whole time, does NOT trip the fixed helper.
    """
    code = 'GRAPH_NODE_INACTIVE'
    healthy_items = [{'fault_code': code, 'description': 'alpha is gone, beta is fine'}]
    for dead_status, label in ((None, 'a dropped connection'), (503, 'a 503 response')):
        with _FlakyFaultsServer(healthy_polls=2, dead_status=dead_status,
                                healthy_items=healthy_items) as fake:
            # (1) The OLD pattern this fix replaces: a single successful description read
            # proves nothing about the rest of the window.
            found, _description = poll_fault_describing(
                fake.port, code, ['alpha'], timeout=2.0, interval=0.2)
            test_case.assertTrue(
                found,
                f'the OLD pattern (poll_fault_describing) did NOT pass against a /faults '
                f'that later died via {label} - this self-test no longer demonstrates the '
                'hole the fix closes')
            # (2) The fixed helper must fail against the identical dying channel.
            with test_case.assertRaises(
                    AssertionError,
                    msg=f'assert_fault_describes_only did not fail when /faults died '
                        f'mid-window via {label} - it would pass on the exact '
                        'channel-death this fix exists to catch'):
                assert_fault_describes_only(
                    test_case, fake.port, code, required=['alpha'], forbidden=['gamma'],
                    duration=2.0, interval=0.2)

    # (3) The companion proof: a channel that never dies, and keeps the description
    # correct the whole time, must not trip the assertion.
    with _FlakyFaultsServer(healthy_polls=10_000, dead_status=None,
                            healthy_items=healthy_items) as fake:
        assert_fault_describes_only(
            test_case, fake.port, code, required=['alpha'], forbidden=['gamma'],
            duration=1.0, interval=0.2)


def prove_never_names_proof_catches_a_wrongly_scoped_absence(test_case):
    """Prove `assert_fault_never_names` succeeds where a whole-code absence check would not.

    The gap this closes: a claim like "no disappearance names THIS node" is not the same claim
    as "this code never appears at all". A correct detector raising `code` for some OTHER entity
    should leave a "never names X" assertion GREEN - it is exactly the case
    `assert_fault_absent_throughout` cannot tell apart from a real defect, since it only ever
    looks at whether the code is present, never at what it names.

    (1) A `/faults` that answers healthily throughout, with `code` present but naming only an
    unrelated entity, fails the OLD pattern (`assert_fault_absent_throughout` on the bare code) -
    the concrete false failure this fix replaces, demonstrated rather than asserted in prose. (2)
    The SAME server does not trip `assert_fault_never_names`. (3) A server whose `code` DOES name
    the forbidden entity at some point trips the new helper - or it would never fail on the
    actual defect it exists to catch. (4) The same channel-alive discipline the other three
    window assertions already carry: a dead channel mid-window fails this one too.

    Raises whatever `test_case`'s own assertions raise on failure; callers use it from inside a
    test method exactly like any other assertion helper.
    """
    code = 'GRAPH_NODE_DISAPPEARED'
    forbidden_entity = '/target/allowlisted'
    unrelated_items = [
        {'fault_code': code, 'description': f'{code}: node /other/unrelated is gone'}]
    naming_items = [
        {'fault_code': code, 'description': f'{code}: node {forbidden_entity} is gone'}]

    with _FlakyFaultsServer(healthy_polls=10_000, dead_status=None,
                            healthy_items=unrelated_items) as fake:
        # (1) The OLD pattern this fix replaces: a whole-code absence check trips on a raise for
        # an unrelated entity - the false failure this fix exists to stop, demonstrated rather
        # than asserted.
        with test_case.assertRaises(
                AssertionError,
                msg='assert_fault_absent_throughout did NOT fail against a code raised for an '
                    'unrelated entity - this self-test no longer demonstrates the false failure '
                    'the fix replaces'):
            assert_fault_absent_throughout(test_case, fake.port, code, duration=1.0, interval=0.2)

        # (2) The fixed helper must NOT fail on the identical server: the code is present, but
        # never names the forbidden entity.
        assert_fault_never_names(
            test_case, fake.port, code, forbidden=[forbidden_entity], duration=1.0, interval=0.2)

    with _FlakyFaultsServer(healthy_polls=10_000, dead_status=None,
                            healthy_items=naming_items) as fake:
        # (3) The fixed helper MUST fail once the description actually names the forbidden
        # entity - or it would never catch the defect it exists for.
        with test_case.assertRaises(
                AssertionError,
                msg='assert_fault_never_names did not fail when the description named the '
                    'forbidden entity - it would pass on the exact defect this fix exists to '
                    'catch'):
            assert_fault_never_names(
                test_case, fake.port, code, forbidden=[forbidden_entity],
                duration=1.0, interval=0.2)

    # (4) Channel-alive discipline, matching the other three window assertions: a dead channel
    # mid-window must fail this one too, not be read as "never named a forbidden entity".
    for dead_status, label in ((None, 'a dropped connection'), (503, 'a 503 response')):
        with _FlakyFaultsServer(healthy_polls=2, dead_status=dead_status,
                                healthy_items=unrelated_items) as fake:
            with test_case.assertRaises(
                    AssertionError,
                    msg=f'assert_fault_never_names did not fail when /faults died mid-window '
                        f'via {label} - it would pass on the exact channel-death this fix '
                        'exists to catch'):
                assert_fault_never_names(
                    test_case, fake.port, code, forbidden=[forbidden_entity],
                    duration=2.0, interval=0.2)


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
