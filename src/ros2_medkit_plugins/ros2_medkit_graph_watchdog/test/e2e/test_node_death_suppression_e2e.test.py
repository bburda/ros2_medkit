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
"""node_death suppression e2e: allowlist, self-suppression and pruning against the REAL stack.

Sibling of test_node_death_e2e.test.py, same shape: each RAISE row proves the fault actually
appears, naming the missing-fault case as the failure mode when it does not. An ABSENCE row
cannot, by itself, discriminate a correct suppressor from no detector at all, because both
produce the same silence; each such row's own class docstring says so and states what the row
catches instead.

Runs as FIVE separate CTest targets (see CMakeLists.txt), each launching its OWN
gateway+fault_manager+demo-node stack - the plugin reads its config once at set_context() time, so
different configs need different gateway launches. WATCHDOG_E2E_SCENARIO selects which launch and
which assertions run:

- "allowlist_suppresses": ``detectors.node_death.allowlist: [TARGET_NODE]`` with
  ``suppress: ["allowlist"]`` - the node is armed, then killed. No GRAPH_NODE_DISAPPEARED names
  it, for a sustained window - narrower than "the code never appears at all": a correct
  detector raising the code for some OTHER entity must leave this row green, so the assertion
  is needle-scoped (assert_fault_never_names) rather than a bare absence check. CANNOT
  DISCRIMINATE BY ITSELF: silence here is consistent with a suppressor correctly
  honouring the allowlist, and equally consistent with one that never consults it, since both
  leave the code unraised.
- "allowlist_not_named_is_inert": the SAME allowlist, but ``suppress`` does not name
  "allowlist" - naming an entry on the allowlist is not, by itself, a request to suppress it;
  only listing the mechanism in ``suppress`` opts it in, unlike the ported source, which
  activated a configured allowlist unconditionally. The fault raises and names the node - a
  genuine RAISE row - and a startup warning says the list is inert. The warning half is read
  straight off the
  gateway process's own stderr via launch_testing's ``proc_output`` fixture, the same instrument
  test_startup_param_clamp_warnings.test.py already uses for an identical "did we say so" claim -
  no new harness helper, just a capability already available at this tier and never exercised in
  this package's own e2e suite before. The match itself is a compiled regex, not the bare
  substring "allowlist": a line mentioning the word for an unrelated reason (reporting its size,
  say) must not satisfy a warning this specific, so the pattern requires "allowlist" and
  "suppress" on the SAME line at WARN severity - see _INERT_ALLOWLIST_WARNING_RE.
- "lifecycle_clean_shutdown": self-suppression - a node this package already watches through
  lifecycle_expectation is suppressed only when it departed while not active, not a
  lifecycle-agnostic notion of "clean". Two identical managed_lifecycle instances, both listed in
  ``detectors.lifecycle_expectation.require_active`` (self-suppression needs BOTH "this node is
  require_active-owned" and "it departed while not active" - the lifecycle label alone would
  over-suppress a node nobody is watching), AND ``detectors.node_death.suppress: ["lifecycle"]``
  naming the self-suppression mechanism this row is about - suppression is opt-in by ruling, so
  without that entry a correct detector reports BOTH departed nodes and this row's own "the clean
  one is never named" half would be false against a correct implementation. One is driven through
  a REAL ``lifecycle_msgs/srv/ChangeState`` UNCONFIGURED_SHUTDOWN transition (reaching
  "finalized") before its process is killed; the other reaches "active" (auto_activate) and is
  killed outright, still active. The finalized one is never named; the active one is - proven
  with assert_fault_describes_only so the claim holds over the whole window, not one lucky read.
- "suppression_is_opt_in": the allowlist key is set, but ``suppress`` is not configured AT ALL -
  merely naming a node on the allowlist must not suppress it by itself. The fault raises and
  names the node.
- "prune_no_false_heal": TWO independent nodes. TARGET_NODE carries
  ``allowlist: [TARGET_NODE]`` with ``suppress: ["allowlist"]`` - the ONLY shape pruning ever
  applies to (``NodeLivenessTracker::prune()`` reclaims a key only once it has been DURABLY
  suppressed; an unsuppressed death has no reclaim path at all - see node_liveness_tracker.hpp's
  own doc). SECOND_NODE carries no suppression at all, so its death is an ordinary, permanently
  outstanding fault - the control this row needs, since a single-node shape cannot tell "prune()
  reclaimed the right key" from "prune() reclaimed (or healed) something it should not have".
  Both are killed; TARGET_NODE's bookkeeping is then observed reclaimed past
  ``detectors.node_death.prune_grace`` as a `tracked_count` DELTA of exactly one off a baseline
  (never against zero - the aggregate also carries every other armed App in the graph, so it can
  never read exactly zero), while SECOND_NODE's fault must stay CONFIRMED and keep naming it,
  never TARGET_NODE, for a window spanning straight through the tick TARGET_NODE's own reclaim
  happens on.

Every scenario gates on wait_until_watchdog_armed(PORT, app_id=...) BEFORE asserting anything -
see harness.py's own docstring for why a stack that never came up otherwise produces exactly the
silence an absence claim is looking for. "lifecycle_clean_shutdown" is the one exception: both its
nodes are non-active managed_lifecycle instances, and ReliabilityGate reports a tracked node with
a known non-active label as "warming_up" by design (LifecycleWatcher::node_ok() is false for
exactly the node this whole detector class exists to catch) - so a per-entity armed gate on either
one would wait for something that can never become true. It gates on the GLOBAL armed state
instead, exactly as test_lifecycle_expectation_e2e.test.py's own "main" scenario does for the
identical reason.

"allowlist_not_named_is_inert" and "lifecycle_clean_shutdown" both configure
``suppress: ["lifecycle"]`` for the second suppression mechanism this port carries (see
lifecycle_shutdown_suppressor.hpp). In "allowlist_not_named_is_inert" it is chosen so the row
is a well-formed "suppress names something, but not allowlist" rather than a malformed-entry
row (C2's own territory); in "lifecycle_clean_shutdown" it is the entry that actually has to be
present for the row's own claim to be testable at all - self-suppression is opt-in like every
other mechanism here, so without naming it a correct detector reports both departed nodes.
"""

import os
import re
import signal
import sys
import time
import unittest

from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.node import Node
import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above, so this
# import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    API_BASE_PATH,
    assert_fault_describes_only,
    assert_fault_never_names,
    create_watchdog_test_launch,
    poll_detector_status,
    poll_faults,
    prove_never_names_proof_catches_a_wrongly_scoped_absence,
    wait_until_faults_endpoint_live,
    wait_until_watchdog_armed,
    watchdog_detector_status,
)

from ros2_medkit_test_utils.constants import (  # noqa: E402
    ALLOWED_EXIT_CODES,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.coverage import get_coverage_env  # noqa: E402

# No default on purpose - see harness-consuming siblings' identical rationale: a default makes
# this file FAIL OPEN. A KeyError is loud.
SCENARIO = os.environ['WATCHDOG_E2E_SCENARIO']
PORT = get_test_port()

FAULT_CODE = 'GRAPH_NODE_DISAPPEARED'
_NODE_DEATH_PREFIX = 'plugins.graph_watchdog.detectors.node_death'
_LIFECYCLE_PREFIX = 'plugins.graph_watchdog.detectors.lifecycle_expectation'

# Same fast cadence and miss_grace convention test_node_death_e2e.test.py uses: comfortably past
# the documented 3000 ms floor (config sweep C1 owns the floor's own boundary values) without
# accidentally landing on it.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3
MISS_GRACE = 20

# The plain node every scenario but "lifecycle_clean_shutdown" kills - same DEMO_NODE_REGISTRY
# fixture test_node_death_e2e.test.py uses, built by hand for the same reason: a scenario that
# signals it needs a PID, and create_demo_nodes() gives none back.
TARGET_NODE = 'calibration'
TARGET_EXECUTABLE = 'demo_calibration_service'
TARGET_NAMESPACE = '/powertrain/engine'

# The budgets below are scaled by MEDKIT_TEST_TIME_SCALE, which the sanitizer jobs set to the
# same factor they apply to every declared CTest timeout. A deadline asserted INSIDE a test is
# invisible to that rewrite, so an instrumented graph that takes longer to forget a departed
# node blows a budget here and the failure reads as a detector that never reported - the exact
# red this suite exists to produce for a real defect. Unset elsewhere, so the normal jobs keep
# the tight budgets that give these assertions their falsifying edge.
#
# Poll intervals, enforced respawn delays and the sustained-observation windows are NOT scaled.
# Those are not give-up bounds: stretching a window a scenario watches for silence buys no
# confidence and spends the whole package's test budget to do it.
TIME_SCALE = get_time_scale()
ARM_TIMEOUT_SEC = 60.0 * TIME_SCALE
FAULTS_LIVE_TIMEOUT_SEC = 30.0 * TIME_SCALE
DEPARTURE_TIMEOUT_SEC = 30.0 * TIME_SCALE
RAISE_TIMEOUT_SEC = 60.0 * TIME_SCALE
# How long an absent or a persisting fault is watched for. Short (well under a minute) because
# every window here is measured from the arming gate, not process start, so bringup cannot eat
# it - matching test_node_death_e2e.test.py's SUSTAINED_WINDOW_SEC.
SUSTAINED_WINDOW_SEC = 20.0
# How long the "allowlist is inert" startup warning is waited for on the gateway's own stderr.
# Config parsing happens at plugin set_context() time, at process start - well before the demo
# node and fault_manager even come up (create_watchdog_test_launch's own demo_delay) - so this
# only needs to be generous against process-startup jitter, not against anything downstream.
WARNING_TIMEOUT_SEC = 30.0 * TIME_SCALE

# Matches a single stderr line that names BOTH "allowlist" and "suppress" - the two config
# keys an inert allowlist is about - at WARN severity, not the bare substring "allowlist",
# which a detector could satisfy with an unrelated line ("allowlist contains 1 entry") while
# never claiming anything is inert. Neither token pins node_death's own exact wording, but the
# codebase's own convention (test_startup_param_clamp_warnings.test.py) is to echo a config key
# verbatim in its warning, and an allowlist that suppress does not name must say so - i.e. be
# logged via RCLCPP_WARN, whose default format always stamps the literal "[WARN]" tag. Requiring
# all three on one line, in any order, rules out a routine config-echo (no [WARN]), a bare size
# report (no "suppress"), and an unrelated warning that happens to mention "suppress" for a
# different reason (no "allowlist") - none of which is the inert-allowlist warning this row is
# about.
_INERT_ALLOWLIST_WARNING_RE = re.compile(
    r'^(?=.*\[WARN\])(?=.*\ballowlist\b)(?=.*\bsuppress\b).*$', re.IGNORECASE | re.MULTILINE)

# ---- "lifecycle_clean_shutdown" scenario's own fixtures ---------------------------------------
# Distinct, self-describing names so this scenario's own two nodes are never confused with the
# plain TARGET_NODE above or with anything a shared demo fixture might also be named.
CLEAN_NODE = 'suppress_lifecycle_clean'
KILLED_NODE = 'suppress_lifecycle_active'
LIFECYCLE_EXECUTABLE = 'managed_lifecycle'  # DEMO_NODE_REGISTRY's own executable for both variants
LIFECYCLE_GRACE = 5

# ---- "prune_no_false_heal" scenario's own prune_grace ------------------------------------------
# Must be >= miss_grace + 1 (config sweep C1's own clamp) or it is silently raised to that floor.
# Set to exactly the floor so pruning becomes eligible as soon as this scenario's own window can
# show it.
PRUNE_GRACE = MISS_GRACE + 1

# ---- "prune_no_false_heal" scenario's own second, UNSUPPRESSED node ----------------------------
# A second, independent identity carrying no suppression at all, so its death is an ordinary,
# permanently-outstanding fault - the control this row needs to prove that reclaiming
# TARGET_NODE's (suppressed, prune-eligible) bookkeeping never touches it.
SECOND_NODE = 'prune_second_unsuppressed'
SECOND_NAMESPACE = '/powertrain/unsuppressed'
# How long the reclaim-delta baseline is allowed to take to SETTLE (see
# _poll_stable_tracked_count) before either node departs. wait_until_watchdog_armed(app_id=...)
# only proves the gate considers TARGET_NODE/SECOND_NODE armed, not that node_death's own tick
# has already added their keys to tracker_ (see that helper's own docstring) - this is generous
# against that residual lag, not against anything this scenario's own cycle does. Mirrors
# test_node_death_e2e.test.py's identical STABLE_TRACKED_COUNT_TIMEOUT_SEC, same detector, same
# hazard.
STABLE_TRACKED_COUNT_TIMEOUT_SEC = 40.0 * TIME_SCALE


def _target_node_action():
    """One manually-constructed TARGET_NODE action, with a PID handle the test can signal.

    Mirrors DEMO_NODE_REGISTRY's own 'calibration' entry exactly, the same way
    test_node_death_e2e.test.py's identical helper does - built by hand only because every
    scenario in this file needs to SIGTERM it, and create_demo_nodes() hands back no PID.
    Every scenario here kills it exactly once and never respawns it, so respawn is not a
    parameter.
    """
    return launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable=TARGET_EXECUTABLE,
        name=TARGET_NODE,
        namespace=TARGET_NAMESPACE,
        output='screen',
        additional_env=get_coverage_env('ros2_medkit_integration_tests'),
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


def _second_node_action():
    """SECOND_NODE: a second, independent, never-allowlisted identity for "prune_no_false_heal".

    Same executable as TARGET_NODE, under its own name/namespace so it is a genuinely distinct
    node - built by hand for the same PID-handle reason _target_node_action() is.
    """
    return launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable=TARGET_EXECUTABLE,
        name=SECOND_NODE,
        namespace=SECOND_NAMESPACE,
        output='screen',
        additional_env=get_coverage_env('ros2_medkit_integration_tests'),
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


def _lifecycle_node_action(name, *, auto_activate):
    """One managed_lifecycle instance under `name`, optionally self-activating.

    Same executable DEMO_NODE_REGISTRY's 'managed_lifecycle'/'managed_lifecycle_active' entries
    use, launched by hand under this scenario's own names (not create_demo_nodes(), which hands
    back no PID) so both instances can run side by side without colliding on the bare node name.
    """
    node_kwargs = {
        'package': 'ros2_medkit_integration_tests',
        'executable': LIFECYCLE_EXECUTABLE,
        'name': name,
        'namespace': '',
        'output': 'screen',
        'additional_env': get_coverage_env('ros2_medkit_integration_tests'),
        'sigterm_timeout': '30',
        'sigkill_timeout': '15',
    }
    if auto_activate:
        node_kwargs['parameters'] = [{'auto_activate': True}]
    return launch_ros.actions.Node(**node_kwargs)


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
    }
    demo_nodes = []

    if SCENARIO == 'allowlist_suppresses':
        detector_params[f'{_NODE_DEATH_PREFIX}.allowlist'] = [TARGET_NODE]
        detector_params[f'{_NODE_DEATH_PREFIX}.suppress'] = ['allowlist']
    elif SCENARIO == 'allowlist_not_named_is_inert':
        detector_params[f'{_NODE_DEATH_PREFIX}.allowlist'] = [TARGET_NODE]
        # Non-empty and NOT "allowlist" - see the module docstring's own note on why this
        # cannot be an empty list (ROS 2 launch crashes on an empty list parameter) and why
        # this specific string is a placeholder for the framework's second mechanism rather
        # than a malformed entry.
        detector_params[f'{_NODE_DEATH_PREFIX}.suppress'] = ['lifecycle']
    elif SCENARIO == 'lifecycle_clean_shutdown':
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [CLEAN_NODE, KILLED_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = LIFECYCLE_GRACE
        # Suppression is opt-in by ruling: without naming the self-suppression mechanism here,
        # a correct detector reports BOTH departed nodes, and this scenario's own "the clean one
        # is never named" half would be false against a correct implementation.
        detector_params[f'{_NODE_DEATH_PREFIX}.suppress'] = ['lifecycle']
    elif SCENARIO == 'suppression_is_opt_in':
        detector_params[f'{_NODE_DEATH_PREFIX}.allowlist'] = [TARGET_NODE]
        # No 'suppress' key at all - the whole point of this scenario.
    elif SCENARIO == 'prune_no_false_heal':
        detector_params[f'{_NODE_DEATH_PREFIX}.miss_grace'] = MISS_GRACE
        detector_params[f'{_NODE_DEATH_PREFIX}.prune_grace'] = PRUNE_GRACE
        # Durable suppression is the only path prune() ever reclaims through - see the
        # module docstring's own note on why an unsuppressed death cannot exercise it.
        detector_params[f'{_NODE_DEATH_PREFIX}.allowlist'] = [TARGET_NODE]
        detector_params[f'{_NODE_DEATH_PREFIX}.suppress'] = ['allowlist']
    else:
        raise RuntimeError(f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} has no launch configuration')

    launch_description, context = create_watchdog_test_launch(
        detector_params=detector_params,
        demo_nodes=demo_nodes,
        port=PORT,
    )

    if SCENARIO in ('allowlist_suppresses', 'allowlist_not_named_is_inert',
                    'suppression_is_opt_in', 'prune_no_false_heal'):
        target = _target_node_action()
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'prune_no_false_heal':
        second = _second_node_action()
        launch_description.add_action(TimerAction(period=2.0, actions=[second]))
        context['second_node'] = second

    if SCENARIO == 'lifecycle_clean_shutdown':
        clean = _lifecycle_node_action(CLEAN_NODE, auto_activate=False)
        killed = _lifecycle_node_action(KILLED_NODE, auto_activate=True)
        launch_description.add_action(TimerAction(period=2.0, actions=[clean, killed]))
        context['clean_node'] = clean
        context['killed_node'] = killed

    return launch_description, context


# ---------------------------------------------------------------------------------------
# Local helpers - read the same endpoints harness.py's own helpers do, or a service this
# file's own scenario needs that no shared helper covers.
# ---------------------------------------------------------------------------------------

def _poll_apps_absent(port, app_id, timeout=30.0, interval=0.5):
    """Poll ``GET /apps`` until `app_id` is no longer listed. ``True`` once it is gone."""
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    last_seen = 'GET /apps was never answered at all'
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/apps', timeout=5)
            if response.status_code == 200:
                ids = [item.get('id') for item in response.json().get('items', [])]
                last_seen = str(ids)
                if app_id not in ids:
                    return True
            else:
                last_seen = f'HTTP {response.status_code} from GET /apps'
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /apps failed: {exc}'
        time.sleep(interval)
    print(f'_poll_apps_absent({app_id!r}) timed out after {timeout}s; last seen: {last_seen}')
    return False


def _poll_watchdog_entity(port, app_id, lifecycle, timeout=30.0, interval=0.5):
    """Poll GET /x-medkit-watchdog until `app_id` appears with this lifecycle label.

    Same instrument, same rationale, as test_lifecycle_expectation_e2e.test.py's identical
    local helper: proves the target's label was actually READ by the live stack, which
    wait_until_watchdog_armed cannot (an unread label is treated as benign, so a run in which
    the label never arrived would look identical to one where it did). '' (empty string) is
    "asked, and still waiting", never "no data yet" - LifecycleWatcher seeds a tracked entry's
    label to "" and only overwrites it once a real read succeeds.

    Returns the matching entity dict, or ``None`` on timeout (after printing the last-seen
    payload, which is gone once the launch tears down).
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    last_seen = 'GET /x-medkit-watchdog was never answered at all'
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/x-medkit-watchdog', timeout=5)
            if response.status_code != 200:
                last_seen = f'HTTP {response.status_code} from GET /x-medkit-watchdog'
            else:
                status = response.json().get('x-medkit-watchdog', {})
                last_seen = str(status)
                for entity in status.get('entities') or []:
                    if entity.get('id') == app_id and entity.get('lifecycle') == lifecycle:
                        return entity
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /x-medkit-watchdog failed: {exc}'
        time.sleep(interval)
    print(f'_poll_watchdog_entity(app_id={app_id!r}, lifecycle={lifecycle!r}) timed out after '
          f'{timeout}s; last watchdog status: {last_seen}')
    return None


def _call_change_state_once(client_node, service_name, transition_id, timeout=30.0):
    """One ``ChangeState`` call against `service_name`. ``True`` on ``result.success``.

    Same shape as test_node_death_e2e.test.py's identical local helper.
    """
    client = client_node.create_client(ChangeState, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=timeout)
        result = future.result()
        return result is not None and result.success
    finally:
        client_node.destroy_client(client)


def _poll_stable_tracked_count(port, detector_id, timeout, stable_seconds=10.0, interval=0.5):
    """Poll the detector's status block until ``tracked_count`` holds steady.

    "Holds steady" means the SAME value for a continuous `stable_seconds` before `timeout`
    elapses. A single sample proves nothing about whether node_death has actually caught up:
    wait_until_watchdog_armed(app_id=...) only proves the RELIABILITY GATE considers an entity
    armed, not that node_death's own tick has already added its key to tracker_ - see that
    helper's own docstring ("What this does NOT prove is that a detector has already READ the
    app... Callers that need a captured baseline must still allow the sweep a window after this
    returns"). A baseline read immediately after both TARGET_NODE and SECOND_NODE gate-arm can
    therefore land before either one (or both) has actually joined tracked_count, understating
    the baseline - and since this row's own claim is a tracked_count DELTA off that baseline, an
    understated baseline makes the later reclaim assertion fail for a reason that has nothing to
    do with prune(). Same helper, same detector, same hazard as
    test_node_death_e2e.test.py's identical ``_poll_stable_tracked_count``.

    Returns ``(status, stable)`` - `status` is the LAST status block read (possibly still
    unsettled, so a caller can still report what it saw), `stable` is False if the value never
    held for a continuous `stable_seconds` inside `timeout`.
    """
    deadline = time.monotonic() + timeout
    status = None
    last_count = None
    streak_start = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        status = watchdog_detector_status(port, detector_id)
        count = status.get('tracked_count') if status is not None else None
        if count != last_count:
            last_count = count
            streak_start = now
        elif now - streak_start >= stable_seconds:
            return status, True
        time.sleep(interval)
    return status, False


# ---------------------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------------------

class TestSuppressionAllowlistSuppresses(unittest.TestCase):
    """A node named in the allowlist, with suppress opting the allowlist in, is never named.

    The claim is needle-scoped ("no GRAPH_NODE_DISAPPEARED names it"), not "the code never
    appears at all": a correct detector raising the code for some OTHER entity in this launch
    (the fault_manager's own node, say) must leave this row green, since that has nothing to do
    with whether allowlist-suppression works. A bare code-absence check cannot tell the two
    apart, so this uses assert_fault_never_names.

    CANNOT DISCRIMINATE BY ITSELF: silence here is consistent with a suppressor correctly
    honouring the allowlist, and equally consistent with one that never consults it, since both
    leave the code unraised. Recorded as such rather than presented as proof of correct
    suppression - ``test_allowlist_suppressor.cpp`` pins the matcher's own discrimination
    directly, and ``prune_no_false_heal`` below runs the SAME allowlist against a second,
    unlisted node under the identical config to show the two are told apart.
    """

    def test_allowlisted_death_never_raises(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed - the plugin did not load, '
            f'{TARGET_NODE} was never discovered, or the bringup grace never elapsed, so no '
            'absence below could mean anything',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - the absence below would prove nothing')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM - this scenario never produced '
            'the departure it is named for',
        )

        assert_fault_never_names(
            self, PORT, FAULT_CODE, forbidden=[TARGET_NODE], duration=SUSTAINED_WINDOW_SEC)

    def test_self_test_catches_a_wrongly_scoped_absence(self):
        """The test of the test for assert_fault_never_names, run once per this file.

        Self-contained (a local HTTP server stands in for the gateway) - runs alongside the
        real assertion above without depending on it, the same convention
        test_node_death_e2e.test.py's own "raise" scenario and
        test_lifecycle_expectation_e2e.test.py's "main" scenario already use for their own new
        harness helpers.
        """
        prove_never_names_proof_catches_a_wrongly_scoped_absence(self)


class TestSuppressionAllowlistNotNamedIsInert(unittest.TestCase):
    """An allowlist that `suppress` does not name has no effect, and says so."""

    def test_inert_allowlist_still_raises_and_warns(self, target_node, proc_output, gateway_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE} never raised after {TARGET_NODE} exited - an allowlist entry '
                "that 'suppress' does not name must not suppress it")
        self.assertIn(TARGET_NODE, fault.get('description', ''))

        # The warning is read straight off the gateway's own stderr - the same instrument
        # test_startup_param_clamp_warnings.test.py already uses for an identical "did we say
        # so" claim. No exact wording is pinned (node_death's own log text is free to change);
        # this asks that a WARN-severity line names BOTH "allowlist" and "suppress" together -
        # see _INERT_ALLOWLIST_WARNING_RE for why the bare substring "allowlist" is not enough.
        self.assertTrue(
            proc_output.waitFor(
                _INERT_ALLOWLIST_WARNING_RE, process=gateway_node, stream='stderr',
                timeout=WARNING_TIMEOUT_SEC),
            "no startup WARNING naming both 'allowlist' and 'suppress' on the same line "
            f"appeared on the gateway's stderr within {WARNING_TIMEOUT_SEC}s - a configured "
            'allowlist that suppress does not name must say so at WARN severity',
        )


class TestSuppressionLifecycleCleanShutdown(unittest.TestCase):
    """Self-suppression: a require_active node that departed while not active is not named.

    Two managed_lifecycle instances, both require_active-owned - self-suppression needs "this
    node is owned by lifecycle_expectation", "it departed while not active", AND
    ``detectors.node_death.suppress`` naming the mechanism (suppression is opt-in like every
    other mechanism this package carries); the lifecycle label alone would over-suppress a node
    nobody is watching, and leaving suppress unset would make a correct detector report both
    departed nodes. CLEAN_NODE is driven through a real UNCONFIGURED_SHUTDOWN transition
    (reaching "finalized", a non-active label) before its process dies; KILLED_NODE reaches
    "active" on its own and is killed outright, still active. Only the second is a departure
    lifecycle_expectation has nothing else to say about.

    GRAPH_NODE_DISAPPEARED raises and names KILLED_NODE. The "CLEAN_NODE is never named" half
    cannot, by itself, discriminate a working self-suppressor from no detector at all - both
    leave CLEAN_NODE unnamed - so it is proven alongside the positive KILLED_NODE claim rather
    than on its own.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('node_death_suppression_e2e_shutdown_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_clean_shutdown_not_named_active_kill_is(self, clean_node, killed_node):
        # Global gate, not app_id: both nodes are require_active-tracked and neither starts
        # active, so ReliabilityGate reports each of them "warming_up" (LifecycleWatcher's
        # node_ok() is false for exactly this case) until KILLED_NODE activates - a per-entity
        # gate on CLEAN_NODE specifically would wait for something that can never become true.
        # Same reasoning test_lifecycle_expectation_e2e.test.py's "main" scenario gives for its
        # identical choice.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC),
            'graph_watchdog never reported an armed global state')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, CLEAN_NODE, 'unconfigured', timeout=ARM_TIMEOUT_SEC),
            f'{CLEAN_NODE} never read as "unconfigured" - the fixture was not discovered as a '
            'managed node, or this scenario never set up the trigger it is named for',
        )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, KILLED_NODE, 'active', timeout=ARM_TIMEOUT_SEC),
            f'{KILLED_NODE} never reached "active" on its own - the contrast this scenario '
            'needs (an active node killed outright) was never set up',
        )

        change_state_service = f'/{CLEAN_NODE}/change_state'
        self.assertTrue(
            _call_change_state_once(
                type(self)._client_node, change_state_service,
                Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, timeout=30.0),
            f'the real UNCONFIGURED_SHUTDOWN transition on {CLEAN_NODE} was rejected or never '
            'answered',
        )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, CLEAN_NODE, 'finalized', timeout=30.0),
            f'{CLEAN_NODE} never read as "finalized" after its own UNCONFIGURED_SHUTDOWN - the '
            'clean-shutdown trigger this scenario is about was never actually reached',
        )

        os.kill(clean_node.process_details['pid'], signal.SIGTERM)
        os.kill(killed_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, CLEAN_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{CLEAN_NODE} never left GET /apps after SIGTERM')
        self.assertTrue(
            _poll_apps_absent(PORT, KILLED_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{KILLED_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE} never raised after {KILLED_NODE} (active, killed outright) '
                'exited - a departure lifecycle_expectation has nothing to say about must still '
                'be named')

        assert_fault_describes_only(
            self, PORT, FAULT_CODE,
            required=[KILLED_NODE], forbidden=[CLEAN_NODE],
            duration=SUSTAINED_WINDOW_SEC)


class TestSuppressionOptIn(unittest.TestCase):
    """An allowlist entry with no `suppress` key at all does not suppress by itself."""

    def test_no_suppress_key_still_raises(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE} never raised after {TARGET_NODE} exited - an allowlist entry '
                'with no suppress key at all must not suppress by itself (suppression is '
                'opt-in)')
        self.assertIn(TARGET_NODE, fault.get('description', ''))


class TestSuppressionPruneNoFalseHeal(unittest.TestCase):
    """Reclaiming an allowlisted key's bookkeeping never heals an unrelated, still-true fault.

    Two independent nodes. TARGET_NODE is on the allowlist (suppress: ["allowlist"]) - the
    ONLY shape prune() ever reclaims through: node_liveness_tracker.hpp's own doc states the
    consequence directly - an unsuppressed death has no reclaim path at all and so must stay
    reported "permanent until acknowledged" for as long as it is actually dead. That is
    exactly why this row cannot point its reclaim proof at an unsuppressed node: prune() would
    never reclaim it, so a reclaim assertion against it could only ever fail. SECOND_NODE,
    carrying no suppression, is instead the permanently-outstanding control, not a second
    reclaim candidate.

    Reclaim itself is proven as a tracked_count DELTA off a baseline, not against zero: the
    aggregate also carries every other armed App in the graph (the gateway's own entity among
    them), so it can never read exactly zero regardless of whether pruning works.
    """

    def test_pruning_the_suppressed_key_never_heals_the_unsuppressed_one(
            self, target_node, second_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=SECOND_NODE),
            f'graph_watchdog never reported {SECOND_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - the claims below would prove nothing about the '
            'detector if the channel itself never came up',
        )

        # Baseline for the reclaim delta: both nodes are present and tracked NOW, before
        # either departs. wait_until_watchdog_armed above proves the GATE considers both
        # armed, not that node_death's own tick has already added their keys to tracker_ (see
        # _poll_stable_tracked_count's own docstring) - so the baseline is read only once
        # tracked_count has genuinely settled, not off the first sample after both gate-arm.
        baseline, stable = _poll_stable_tracked_count(
            PORT, 'node_death', timeout=STABLE_TRACKED_COUNT_TIMEOUT_SEC)
        self.assertTrue(
            stable,
            f'node_death tracked_count never held steady before either node departed (last '
            f'read: {baseline!r}) - the baseline below could not be trusted to already '
            f'include {TARGET_NODE} and {SECOND_NODE}, which would make the reclaim delta '
            'below meaningless',
        )
        self.assertIsNotNone(baseline, 'no node_death status block on GET /x-medkit-watchdog')
        baseline_count = baseline['tracked_count']

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        os.kill(second_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')
        self.assertTrue(
            _poll_apps_absent(PORT, SECOND_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{SECOND_NODE} never left GET /apps after SIGTERM')

        # SECOND_NODE carries no suppression: it must raise, ordinarily, like any other death.
        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(f'{FAULT_CODE} never raised after {SECOND_NODE} exited')

        # PRUNE_GRACE consecutive suppressed ticks reclaims TARGET_NODE's own bookkeeping -
        # tracked_count dropping by exactly one from the baseline, proving prune() genuinely
        # ran (not merely that TARGET_NODE was never raised, which a deleted prune() would
        # also produce). SECOND_NODE's own departed-but-unsuppressed entry stays tracked
        # throughout - only TARGET_NODE's disappears - so the delta is exactly one.
        self.assertTrue(
            poll_detector_status(
                PORT, 'node_death', 'tracked_count', baseline_count - 1,
                timeout=RAISE_TIMEOUT_SEC),
            f'{TARGET_NODE} was never reclaimed - detectors.node_death.tracked_count never '
            f'dropped by exactly one from its baseline ({baseline_count}) even after '
            'PRUNE_GRACE, so this row cannot tell a working prune() from one that never runs')

        # The reclaim above must never have touched SECOND_NODE's own, unrelated, genuinely
        # outstanding fault: it must stay CONFIRMED and keep naming SECOND_NODE - never
        # TARGET_NODE - for a window spanning straight through the reclaim tick just observed.
        # describes_only, not a bare presence check: a detector that let pruning heal the
        # wrong entry, or empty the description, must turn this row red.
        assert_fault_describes_only(
            self, PORT, FAULT_CODE, required=[SECOND_NODE], forbidden=[TARGET_NODE],
            duration=SUSTAINED_WINDOW_SEC)


# Each CTest target launches this file with one scenario, so only that scenario's case may
# run. Removing the others from the module (rather than skipping them) means each run reports
# exactly one case, and a missing result is a real failure rather than an expected line of
# output - see test_node_death_e2e.test.py's identical rationale.
_SCENARIO_CASES = {
    'allowlist_suppresses': 'TestSuppressionAllowlistSuppresses',
    'allowlist_not_named_is_inert': 'TestSuppressionAllowlistNotNamedIsInert',
    'lifecycle_clean_shutdown': 'TestSuppressionLifecycleCleanShutdown',
    'suppression_is_opt_in': 'TestSuppressionOptIn',
    'prune_no_false_heal': 'TestSuppressionPruneNoFalseHeal',
}
if SCENARIO not in _SCENARIO_CASES:
    raise RuntimeError(
        f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} is not one of {sorted(_SCENARIO_CASES)}; the '
        'CTest target and this file disagree about which scenarios exist')
for _scenario, _case_name in _SCENARIO_CASES.items():
    if _scenario != SCENARIO:
        del globals()[_case_name]


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway/fault_manager/demo stack exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
