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
"""Presence ownership e2e: which detector owns the departure of an UNMEASURED managed node.

The division of labour between the two detectors rests on one question - will node_death be
able to report this node's departure - and that question has to be answered from knowledge, not
from the reliability gate's silence about a lifecycle state nobody has read. The gate answers
"fine" for a managed node whose label is still empty exactly as it does for one reading
"active", and those are not the same fact.

test_node_death_boundary_e2e.test.py's B6 row already covers the shape with a REAL managed node
that answers "unconfigured": a required node that never activates, killed below `grace`, must be
reported by GRAPH_NODE_INACTIVE and never by GRAPH_NODE_DISAPPEARED. What it cannot do is hold
the label EMPTY on purpose - a real rclcpp_lifecycle node answers GetState immediately, so the
unmeasured window there is a race between the seed and the warmup, wide open on a loaded runner
and shut on an idle one. This file makes that window permanent by using the fixture whose
GetState never answers at all, and adds the discriminating other half: the same fixture, told to
start answering, must end up owned by the presence detector after all.

Ignorance is BOUNDED, and that bound is what these scenarios are about. Presence ownership is
withheld only while the ignorance can still resolve: LifecycleWatcher charges its GetState
re-seed budget per node and only for a read that actually ran, so an empty label with attempts
left is "we have not finished asking" and an empty label with the budget spent is "we asked and
failed". Refusing ownership past that point is not a handover, it is a silence - nothing else
will ever measure the node, and `lifecycle_expectation` returns early unless an operator named
it in `require_active`, which defaults to empty.

Runs as FIVE separate CTest targets (see CMakeLists.txt). WATCHDOG_E2E_SCENARIO selects which
launch and which assertions run:

- "budget_spent_then_owned": the fixture is never told to answer, so its label stays "" for its
  whole life; it is killed once the watcher has spent every GetState attempt it will ever make.
  GRAPH_NODE_DISAPPEARED must raise and name it, and GRAPH_NODE_UNREADABLE must raise too - both
  are true at once, and neither excuses the other. `require_active` names the node here.
- "no_require_active_still_owned": the same fixture and the same kill, with NO `require_active`
  entry anywhere in the configuration - the SHIPPED default. Nothing but node_death can report
  this death, so GRAPH_NODE_DISAPPEARED must raise and name it. Paired with the row above, it
  also rules out an implementation that decides ownership from `require_active` membership
  rather than from lifecycle knowledge: the two rows differ in exactly that configuration and
  expect the same answer.
- "answered_then_owned": the same fixture under the SAME configuration as
  "budget_spent_then_owned" (`require_active` names it), unmeasured across the whole warmup
  (proven from the status route, not assumed), then told to answer via its `start_answering`
  parameter. Once the label reads "active" the node is killed, and GRAPH_NODE_DISAPPEARED must
  raise naming it. The only variable against its sibling is what the node's lifecycle state
  turned out to be.
- "measured_then_unmeasured": the reverse transition, on a live node, followed by its departure.
  `droppable_lifecycle_node` answers "active" (measured, and owned), then stops advertising its
  lifecycle services on command - the watcher drops the record and the status route reports
  `lifecycle: null` - and only then is it killed. GRAPH_NODE_DISAPPEARED must still raise: a
  node whose measurement goes away does not stop being the presence detector's.
- "provisional_yields_to_a_real_label": the row that separates the two GROUNDS for ownership.
  The same never-answering fixture is left until its budget is spent, so the presence detector
  owns it PROVISIONALLY, and only then is it told to announce "inactive" - the
  ~/transition_event subscription outlives the seed budget, so a real label genuinely can still
  arrive. Once the status route reports that label the node is killed. GRAPH_NODE_INACTIVE must
  raise and name it, and GRAPH_NODE_DISAPPEARED must never appear: the only reason the presence
  detector held this node was that nobody else could report it, and the label ends that. It is
  also the one row that rejects a detector wired to the permissive `reliability_allows()`, which
  admits the node during the unread window and, tracking being sticky, never lets go.
- "restart_inside_warmup": a crash loop whose uptime is shorter than the warmup. The node is
  killed, reported, and comes back - and while it is merely RE-WARMING the gate has no claim to
  state either way, which is not the same fact as the graph having measured the node as another
  detector's. A key already admitted must survive that, or its next death is reported by nobody
  and the detector spends the whole outage saying PASSED. The row kills the returned node
  without waiting for it to arm, and asserts both halves: the second death advances the stored
  record's `last_occurred`, and `last_passed` stays frozen for as long as the node is dead.
- "gate_unaffected": nothing is killed. With the same unmeasured managed node in the graph, the
  three other detectors must still report. GRAPH_PARAM_DRIFT is the discriminating leg - it is
  the only one of the three whose raise passes through the app-keyed gate at all
  (param_drift_detector.cpp calls reliability_allows(ctx.gate, app_id)), so it can only name
  this node if the gate still permits an unread label. GRAPH_QOS_MISMATCH and GRAPH_ORPHAN are
  keyed by topic rather than by App::id (orphan_detector.cpp says so in its own class doc, and
  qos_mismatch_detector.cpp never consults the gate), so the app-keyed predicate can never
  suppress them; their legs guard against a regression that stopped either detector reporting
  at all in a graph carrying such a node, and the QoS leg's mismatched pair includes the
  fixture's OWN publisher.

### Which arming gate

Every scenario here gates on the `app_id` of the node it perturbs. That works
precisely because of the behaviour under test: LifecycleWatcher::node_ok() treats an unread
label as ok, so the gate does reach the per-entity "armed" state for this fixture even while
nothing has ever been measured. A scenario whose target never becomes per-entity armed (a node
sitting at "unconfigured", say) has to use the global form instead - see
test_node_death_boundary_e2e.test.py's own "Which arming gate" section.
"""

import os
import signal
import sys
import time
import unittest

from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
from lifecycle_msgs.msg import TransitionEvent
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import requests
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above, so this
# import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    API_BASE_PATH,
    assert_fault_absent_throughout,
    assert_fault_never_names,
    create_watchdog_test_launch,
    poll_fault_describing,
    poll_faults,
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
from ros2_medkit_test_utils.launch_helpers import DEMO_NODE_REGISTRY  # noqa: E402

# No default on purpose - see the harness-consuming siblings' identical rationale: a default
# makes this file FAIL OPEN, because generate_test_description() would then silently build some
# other scenario's launch while the assertions still ran. A KeyError is loud and immediate.
SCENARIO = os.environ['WATCHDOG_E2E_SCENARIO']
PORT = get_test_port()

FAULT_CODE_DISAPPEARED = 'GRAPH_NODE_DISAPPEARED'
FAULT_CODE_INACTIVE = 'GRAPH_NODE_INACTIVE'
FAULT_CODE_UNREADABLE = 'GRAPH_NODE_UNREADABLE'
FAULT_CODE_PARAM_DRIFT = 'GRAPH_PARAM_DRIFT'
FAULT_CODE_QOS = 'GRAPH_QOS_MISMATCH'
FAULT_CODE_ORPHAN = 'GRAPH_ORPHAN'

DETECTOR_ID_NODE_DEATH = 'node_death'

_LIFECYCLE_PREFIX = 'plugins.graph_watchdog.detectors.lifecycle_expectation'
_NODE_DEATH_PREFIX = 'plugins.graph_watchdog.detectors.node_death'
_PARAM_DRIFT_PREFIX = 'plugins.graph_watchdog.detectors.param_drift'

# The fixture: a plain rclcpp::Node that advertises get_state/change_state of the right SERVICE
# TYPES (all find_lifecycle_get_state_path() checks) and never answers GetState until its
# `start_answering` parameter is set - see unreadable_lifecycle_node.cpp's file doc.
UNREADABLE_NODE = 'unreadable_lifecycle'
ANSWER_PARAM = 'start_answering'

# Fast tick cadence + short warmup so the whole story (gateway/fault_manager startup, fixture
# discovery, per-app arm, global bringup grace) fits inside the poll timeouts below without a
# hand-tuned pre-sleep - the same rationale every e2e file in this package states.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3

# "restart_inside_warmup" needs the RE-WARM window to be wide enough for the test to observe
# the returned node un-armed and kill it again inside that window, which the shipped-size
# warmup above cannot give it: 3 ticks is 600 ms, shorter than one entity-cache refresh. 30
# ticks is 6 s at this cadence, and the row asserts the un-armed state from the status route
# rather than trusting the arithmetic.
RESTART_WARMUP_CYCLES = 30

# The plain demo node that row restarts. No lifecycle interface at all, so the gate's ownership
# answer for it is EARNED throughout and the row turns on nothing but arming.
PLAIN_NODE = 'calibration'

# How long the plain node stays down after each kill - an enforced floor under the outage, not
# a sleep. It has to clear two windows, not one: the detector's own miss window
# ([MISS_GRACE + 1] * TICK_INTERVAL_MS = 3400 ms) plus the entity cache's refresh debounce,
# before the death can be reported at all; and then the whole no-PASSED window this row watches
# afterwards, because the node coming back is a legitimate reason to report PASSED and would
# land inside that window otherwise. 30 s covers ~5 s of detection plus SILENCE_WINDOW_SEC with
# margin to spare.
RESTART_RESPAWN_DELAY_SEC = 30.0

# node_death's grace, in TICKS. Nominal window is [MISS_GRACE + 1] * TICK_INTERVAL_MS = 3400 ms,
# comfortably past the 3000 ms wall-clock floor configure() would otherwise raise it to and two
# ticks clear of that floor's own boundary value (14 at this tick period) - the "comfortably
# past, not exactly on" convention this package's other miss_grace values follow. Small on
# purpose: every scenario here either waits for a departure to be reported or watches for one
# that must never be, and both want the detector's own decision to have been made long before
# the assertion is read.
MISS_GRACE = 16

# The budgets below are scaled by MEDKIT_TEST_TIME_SCALE, which the sanitizer jobs set to the
# same factor they apply to every declared CTest timeout; a deadline asserted INSIDE a test is
# invisible to that rewrite. The sustained-observation window is deliberately NOT scaled: it is
# not a give-up bound, and stretching a window that watches for silence buys no confidence.
TIME_SCALE = get_time_scale()
ARM_TIMEOUT_SEC = 60.0 * TIME_SCALE
FAULTS_LIVE_TIMEOUT_SEC = 30.0 * TIME_SCALE
LABEL_TIMEOUT_SEC = 30.0 * TIME_SCALE
DEPARTURE_TIMEOUT_SEC = 30.0 * TIME_SCALE
PARAM_SET_TIMEOUT_SEC = 30.0 * TIME_SCALE
RAISE_TIMEOUT_SEC = 60.0 * TIME_SCALE

# GRAPH_NODE_UNREADABLE needs kUnmeasuredHoldTicks (60, fixed - not configurable) consecutive
# unmeasured ticks, ~12 s at this cadence, plus the absence grace once the node is killed. 90 s
# is the same shape of budget the "unreadable" scenario in test_lifecycle_expectation_e2e.test.py
# sizes for the identical hold at a comparable cadence.
UNREADABLE_RAISE_TIMEOUT_SEC = 90.0 * TIME_SCALE

# How long the restart row waits for the respawned node to reappear: the enforced respawn delay
# plus the entity cache's own refresh, with margin.
RESTART_RETURN_TIMEOUT_SEC = (RESTART_RESPAWN_DELAY_SEC + 30.0) * TIME_SCALE

# How long a code that must NOT appear is watched for, once the code that must appear already
# has. Not a give-up bound and deliberately not scaled: it is a window in which every poll has
# to answer 200 and carry no such fault.
SILENCE_WINDOW_SEC = 10.0

# The QoS leg's mismatched pair. The fixture publishes ~/transition_event with no deadline (the
# most permissive offer); a subscriber that REQUESTS a finite deadline is RxO-incompatible with
# it - see qos_policy.hpp's qos_incompatibility() deadline branch, and test_qos_e2e.test.py's
# test_04, which pins the same dimension against a purpose-built pair.
QOS_TOPIC = f'/{UNREADABLE_NODE}/transition_event'
QOS_SUB_DEADLINE_SEC = 0.1

# The "measured_then_unmeasured" fixture: a plain rclcpp::Node that looks managed, answers
# get_state with a chosen label, and destroys both lifecycle services when `drop_services` is
# set - see test/e2e/droppable_lifecycle_node.cpp. It is the only fixture in this package that
# can take a node from MEASURED to UNMEASURED without killing it.
DROPPABLE_NODE = 'presence_ownership_droppable'
DROP_PARAM = 'drop_services'

# The label the never-answering fixture announces once it is told to answer. "inactive" is a
# measured, NON-active state: it says the node was never the presence detector's, which is
# exactly what a provisional grant has to yield to.
# The rclpy node this row's own test process spins to drive the fixture's parameter. It is an
# App like any other, so node_death admits it too - and an admission landing between the
# release and the sample that looks for it hides the release completely, because tracked_count
# is a graph-wide total rather than a per-key flag. The row therefore waits for this one to arm
# before it takes its baseline, which leaves the fixture's release the only admission event
# still to come.
LATE_LABEL_CLIENT_NODE = 'presence_ownership_late_label_client'

TRANSITION_LABEL_PARAM = 'transition_label'
LATE_LABEL = 'inactive'

# lifecycle_expectation's own grace, in ticks, for the row where GRAPH_NODE_INACTIVE is the
# positive control. Small so the violation confirms while the node is still present, which
# keeps the row about ownership rather than about absence maturing a streak.
LATE_LABEL_GRACE = 2

# The orphan leg's near-miss pair: one publisher-only topic and one subscriber-only topic, same
# type, same namespace, leaf edit distance 1. Both endpoints belong to this test process; the
# detector keys its finding on the TOPIC, so there is no way to make either of them the
# fixture's own (its only non-system topic already carries endpoints on both sides).
ORPHAN_TYPO_TOPIC = f'/{UNREADABLE_NODE}/scam'
ORPHAN_TARGET_TOPIC = f'/{UNREADABLE_NODE}/scan'


def _droppable_node_action():
    """Build the droppable fixture as a launch action with a PID handle the test can signal.

    Answers "active" from the start, so the node is MEASURED - and therefore owned by the
    presence detector - before anything is done to it.
    """
    return launch_ros.actions.Node(
        package='ros2_medkit_graph_watchdog',
        executable='droppable_lifecycle_node',
        name=DROPPABLE_NODE,
        output='screen',
        parameters=[{'state_label': 'active'}],
        additional_env=get_coverage_env('ros2_medkit_graph_watchdog'),
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


def _plain_node_action():
    """Build the plain demo node as a respawning launch action with a PID handle.

    No lifecycle services at all, so the gate answers kEarned for it whenever it is armed and
    nothing about this row depends on a lifecycle label. `respawn` is what brings the SAME node
    name back after the test kills it, under a new pid the action reports.
    """
    executable, ros_name, namespace = DEMO_NODE_REGISTRY[PLAIN_NODE]
    return launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable=executable,
        name=ros_name,
        namespace=namespace,
        output='screen',
        additional_env=get_coverage_env('ros2_medkit_integration_tests'),
        sigterm_timeout='30',
        sigkill_timeout='15',
        respawn=True,
        respawn_delay=RESTART_RESPAWN_DELAY_SEC,
    )


def _unreadable_node_action(transition_label=None):
    """Build the fixture as a launch action with a PID handle the test can signal.

    Uses DEMO_NODE_REGISTRY's own (executable, ros_name, namespace) triple so this stays in
    lockstep with demo_nodes.launch.py, built by hand only because the scenarios here signal the
    process directly and create_demo_nodes() hands back no PID.
    """
    executable, ros_name, namespace = DEMO_NODE_REGISTRY[UNREADABLE_NODE]
    # Only passed when a scenario needs the fixture to announce something other than its
    # default: every other row here depends on the default being what it always was.
    parameters = ([{TRANSITION_LABEL_PARAM: transition_label}] if transition_label is not None
                  else [])
    return launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable=executable,
        name=ros_name,
        namespace=namespace,
        parameters=parameters,
        output='screen',
        additional_env=get_coverage_env('ros2_medkit_integration_tests'),
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
        f'{_NODE_DEATH_PREFIX}.miss_grace': MISS_GRACE,
    }

    if SCENARIO == 'provisional_yields_to_a_real_label':
        # require_active is what makes GRAPH_NODE_INACTIVE available as the positive control:
        # the node has to be reported by SOMEBODY once the presence detector hands it back, or
        # the row cannot tell a correct handover from a plain silence.
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [UNREADABLE_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = LATE_LABEL_GRACE
    elif SCENARIO in ('budget_spent_then_owned', 'answered_then_owned'):
        # The SAME configuration for both, deliberately: require_active names the fixture in
        # each, so the only thing that differs between the two rows is what the node's
        # lifecycle state turned out to be. It also makes GRAPH_NODE_UNREADABLE available as
        # this row's corroboration that lifecycle_expectation is watching the same node.
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [UNREADABLE_NODE]
    elif SCENARIO == 'no_require_active_still_owned':
        # Deliberately nothing: require_active defaults to empty, which is the shipped
        # configuration, and in it lifecycle_expectation returns before it looks at anything.
        pass
    elif SCENARIO == 'measured_then_unmeasured':
        # node_death is zero-config, and this row is entirely about it.
        pass
    elif SCENARIO == 'restart_inside_warmup':
        # A wide warmup is the whole instrument here: it makes the re-warm window observable
        # and long enough to act inside, instead of something the test would have to race.
        detector_params['plugins.graph_watchdog.warmup_cycles'] = RESTART_WARMUP_CYCLES
    elif SCENARIO == 'gate_unaffected':
        # `baseline: false` plus one absolute `expect` pin: the pin fires on a STATIC graph, with
        # no runtime perturbation to race, and it only fires for a node that HAS the parameter
        # (ParamDriftPolicy::evaluate skips a pin the observed set does not carry). start_answering
        # exists on the fixture and nowhere else, so GRAPH_PARAM_DRIFT can only be about this
        # node - an aggregate that named half the graph could be truncated past it.
        detector_params[f'{_PARAM_DRIFT_PREFIX}.baseline'] = False
        detector_params[f'{_PARAM_DRIFT_PREFIX}.expect.{ANSWER_PARAM}'] = True
    else:
        raise RuntimeError(f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} has no launch configuration')

    launch_description, context = create_watchdog_test_launch(
        detector_params=detector_params,
        demo_nodes=None,
        port=PORT,
    )

    if SCENARIO == 'measured_then_unmeasured':
        target = _droppable_node_action()
    elif SCENARIO == 'restart_inside_warmup':
        target = _plain_node_action()
    elif SCENARIO == 'provisional_yields_to_a_real_label':
        target = _unreadable_node_action(transition_label=LATE_LABEL)
    else:
        target = _unreadable_node_action()
    launch_description.add_action(TimerAction(period=2.0, actions=[target]))
    context['target_node'] = target
    return launch_description, context


def _poll_watchdog_entity(port, app_id, lifecycle, timeout=30.0, interval=0.5,
                          measurement_pending=None):
    """Poll GET /x-medkit-watchdog until `app_id` appears with this lifecycle label.

    The PAIR is the instrument these scenarios need, not `armed` on its own: `armed` reads the
    same for a plain node and for a managed one whose state nobody has ever read, and the whole
    claim is about telling those two apart. ``''`` is not "no data yet" - LifecycleWatcher seeds
    a TRACKED entry's label to ``''`` and only overwrites it once a real read succeeds, so it
    means "asked, and still waiting", while a node with no managed record at all reports null.

    `measurement_pending`, when given, additionally requires the route's field of the same
    name. An unread label alone does not say whether the watcher still intends to ask, and the
    two answers put the node in opposite hands: still asking means the presence detector must
    keep off it, finished asking means the presence detector is all it has. A row that turns on
    that difference has to READ it rather than wait out an amount of time that looks about
    right.

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
                    if entity.get('id') != app_id or entity.get('lifecycle') != lifecycle:
                        continue
                    if (measurement_pending is not None
                            and entity.get('measurement_pending') is not measurement_pending):
                        continue
                    return entity
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /x-medkit-watchdog failed: {exc}'
        time.sleep(interval)
    print(f'_poll_watchdog_entity(app_id={app_id!r}, lifecycle={lifecycle!r}, '
          f'measurement_pending={measurement_pending!r}) timed out after {timeout}s; '
          f'last watchdog status: {last_seen}')
    return None


def _poll_apps_absent(port, app_id, timeout=30.0, interval=0.5):
    """Poll ``GET /apps`` until `app_id` is no longer listed. ``True`` once it is gone.

    Confirms the SIGTERM'd fixture is really out of the operator-visible entity graph before
    anything is asserted about a fault: ``GET /apps`` and the detector's own per-tick snapshot
    read the same ThreadSafeEntityCache, so this polls the precise input the detector sees.
    """
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


def _fault_record(port, code, timeout=30.0, interval=0.5):
    """Poll ``GET /faults?status=all`` until `code` appears, whatever its status.

    ``poll_faults`` uses the default (pending + confirmed) filter, so a record that HEALED
    while the node was briefly back drops out of it. The restart row needs the record itself -
    ``last_occurred``, which advances on every FAILED, and ``last_passed``, which advances on
    every PASSED - and both survive healing. Returns the matching item dict, or ``None``.
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', params={'status': 'all'}, timeout=5)
            if response.status_code == 200:
                for item in response.json().get('items', []):
                    if item.get('fault_code') == code:
                        return item
        except requests.exceptions.RequestException:
            pass
        time.sleep(interval)
    return None


def _poll_watchdog_entity_state(port, app_id, timeout=30.0, interval=0.5):
    """Return the entity's own ``state`` from GET /x-medkit-watchdog, or ``None``.

    The restart row needs the OPPOSITE of what wait_until_watchdog_armed waits for: proof that
    a node which has just come back is NOT yet armed, so the kill that follows lands inside the
    re-warm window rather than after it.
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/x-medkit-watchdog', timeout=5)
            if response.status_code == 200:
                status = response.json().get('x-medkit-watchdog', {})
                for entity in status.get('entities') or []:
                    if entity.get('id') == app_id:
                        return entity.get('state')
        except requests.exceptions.RequestException:
            pass
        time.sleep(interval)
    return None


def _poll_apps_present(port, app_id, timeout=30.0, interval=0.5):
    """Poll ``GET /apps`` until `app_id` IS listed. ``True`` once it appears."""
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    last_seen = 'GET /apps was never answered at all'
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/apps', timeout=5)
            if response.status_code == 200:
                ids = [item.get('id') for item in response.json().get('items', [])]
                last_seen = str(ids)
                if app_id in ids:
                    return True
            else:
                last_seen = f'HTTP {response.status_code} from GET /apps'
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /apps failed: {exc}'
        time.sleep(interval)
    print(f'_poll_apps_present({app_id!r}) timed out after {timeout}s; last seen: {last_seen}')
    return False


def _poll_tracked_count_below(port, detector_id, threshold, timeout=30.0, interval=0.2):
    """Poll until `detector_id` reports a `tracked_count` strictly below `threshold`.

    The one observable on this route that reflects what the DETECTOR did rather than what the
    watcher knows. Everything else an ownership row could gate on is published straight from
    LifecycleWatcher: `lifecycle` is its cached label, and the per-entity `armed`/`state` pair
    is warmup plus node_ok(), so all three flip the moment a transition_event lands, whole
    detector ticks before node_death has looked at the graph again. Its own `tracked_count`
    comes off the tracker, republished once per tick, so it only moves when a key is actually
    admitted or released.

    It is a graph-WIDE total, though, and that costs the caller two obligations. The count must
    have stopped moving before it is used as a threshold, or the admission this release will
    later undo is not inside it. And every other App that will ever be admitted must already
    be admitted, or one landing after the release refills the total and hides it: measured on a
    live stack, a release dipped the count for 200 ms before the test's own client node was
    admitted, which a 200 ms poll reads as no release at all. `_poll_stable_tracked_count`
    covers the first; gating on every other node being armed covers the second.

    Returns ``True`` once the count drops, ``False`` on timeout (after printing the last value,
    which is gone once the launch tears down).
    """
    deadline = time.monotonic() + timeout
    last_seen = 'the node_death status block was never answered at all'
    while time.monotonic() < deadline:
        block = watchdog_detector_status(port, detector_id, timeout=5.0)
        if block is not None:
            last_seen = str(block)
            count = block.get('tracked_count')
            if isinstance(count, int) and count < threshold:
                return True
        time.sleep(interval)
    print(f'_poll_tracked_count_below({detector_id!r}, {threshold}) timed out after {timeout}s; '
          f'last status: {last_seen}')
    return False


def _poll_stable_tracked_count(port, detector_id, samples=5, interval=0.25, timeout=30.0):
    """Return `detector_id`'s `tracked_count` once it has held the same value `samples` times.

    A baseline read the instant the gate answers PROVISIONAL is taken too early: the gate is
    answering out of LifecycleWatcher, and the admission it licenses happens on the detector's
    next tick. Sampling until the value stops moving is what puts the admission inside the
    baseline instead of after it - without that, the release later returns the count to exactly
    the number this captured and "fell below" is never true.

    Returns the stable value, or ``None`` on timeout.
    """
    deadline = time.monotonic() + timeout
    history = []
    while time.monotonic() < deadline:
        block = watchdog_detector_status(port, detector_id, timeout=5.0)
        count = block.get('tracked_count') if block else None
        history = (history + [count]) if count is not None else []
        if len(history) >= samples and len(set(history[-samples:])) == 1:
            return history[-1]
        time.sleep(interval)
    print(f'_poll_stable_tracked_count({detector_id!r}) never settled in {timeout}s; '
          f'last samples: {history[-samples:]}')
    return None


def _set_bool_parameter(client_node, service_name, param_name, value, timeout=30.0):
    """Set one bool parameter on a REMOTE node via its own ``set_parameters`` service.

    Every ``rclcpp::Node`` starts that service automatically, and the fixture registers an
    on-set callback that answers its held GetState requests and publishes the one
    ~/transition_event the watcher needs - see unreadable_lifecycle_node.cpp's file doc for why
    the event, not the answer, is what moves the cached label at this point.

    Returns ``True`` once the remote node accepts the change.
    """
    client = client_node.create_client(SetParameters, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        return False
    request = SetParameters.Request()
    request.parameters = [Parameter(
        name=param_name,
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value),
    )]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=timeout)
    result = future.result()
    if result is None or not result.results:
        return False
    return bool(result.results[0].successful)


class TestBudgetSpentThenOwned(unittest.TestCase):
    """A managed node the watcher asked and could not read is the presence detector's.

    The gate's raise permission is deliberately permissive for an unread label - every other
    detector depends on that, or a node whose lifecycle service is broken would have every
    fault suppressed - so ownership cannot be admitted on that answer alone. But refusing it
    forever is not a handover either: once LifecycleWatcher has spent the GetState attempts it
    charges per node, nothing will ever ASK this node again, and the only detector that could
    report its departure instead is `lifecycle_expectation`, which looks at nothing unless an
    operator named it in `require_active`. (Asking is not the only channel: ~/transition_event
    stays subscribed, so a label can still arrive unasked - which is what the sibling row
    "provisional_yields_to_a_real_label" is about.)

    So the answer is bounded: withheld while the ignorance can still resolve, taken once it
    cannot. Here `require_active` DOES name the node, so both codes are available and both must
    appear - GRAPH_NODE_DISAPPEARED because the node died and nobody else can say so, and
    GRAPH_NODE_UNREADABLE because its lifecycle promise was never verified. Neither excuses the
    other.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_settled_ignorance_is_reported_by_presence_and_by_lifecycle(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=UNREADABLE_NODE),
            f'graph_watchdog never reported {UNREADABLE_NODE} armed - the permissive gate answer '
            'this row is about was never reached, so nothing below would prove anything')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        entity = _poll_watchdog_entity(PORT, UNREADABLE_NODE, '', timeout=LABEL_TIMEOUT_SEC)
        self.assertIsNotNone(
            entity,
            f'{UNREADABLE_NODE} never appeared with an EMPTY lifecycle label - the fixture was '
            'either not tracked as managed at all or its GetState was answered, and this row '
            'needs a node that is tracked and unmeasured')
        self.assertTrue(
            entity.get('armed'),
            f'{UNREADABLE_NODE} is unmeasured but NOT armed, so node_death would decline it for '
            'the warmup reason instead of the ownership reason this row is about')

        # The budget is spent within a handful of ticks of discovery (kReseedAttempts issued
        # reads, each cut off by the reader's own timeout), and everything above has already
        # cost far more than that. Killing here therefore lands in the SETTLED half of the
        # bound, which is what this row is about.
        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, UNREADABLE_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_DISAPPEARED} never raised for {UNREADABLE_NODE} - the watcher had '
                'already asked for its state as often as it ever will, so withholding ownership '
                'here leaves the death to a detector that only ever looks at require_active '
                'entries')
        self.assertIn(UNREADABLE_NODE, fault.get('description', ''))

        # The other code, and it is not an alternative to the one above: the node also never
        # kept the lifecycle promise an operator wrote down for it.
        found, description = poll_fault_describing(
            PORT, FAULT_CODE_UNREADABLE, [UNREADABLE_NODE],
            timeout=UNREADABLE_RAISE_TIMEOUT_SEC)
        self.assertTrue(
            found,
            f'{FAULT_CODE_UNREADABLE} never raised naming {UNREADABLE_NODE} - the presence code '
            'raising must not have cost the lifecycle code its own report '
            f'(last description: {description!r})')


class TestNoRequireActiveStillOwned(unittest.TestCase):
    """The same death in the SHIPPED configuration, where nothing else is watching at all.

    `require_active` defaults to empty and `lifecycle_expectation` returns before it looks at
    anything, so in this launch node_death is the only detector that can report this node's
    departure. A predicate that refuses a managed node whose state it could never read produces
    a deterministic silence here - strictly worse than the racy one the bound exists to close,
    because no configuration and no timing makes it come back.

    Its pairing with the sibling row is what rules out an implementation deciding ownership
    from `require_active` membership: the two rows differ in exactly that key and expect the
    same answer.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_a_node_nobody_watches_is_still_reported_when_it_dies(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=UNREADABLE_NODE),
            f'graph_watchdog never reported {UNREADABLE_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        entity = _poll_watchdog_entity(PORT, UNREADABLE_NODE, '', timeout=LABEL_TIMEOUT_SEC)
        self.assertIsNotNone(
            entity,
            f'{UNREADABLE_NODE} never appeared with an EMPTY lifecycle label, so this row is not '
            'about an unmeasured managed node at all')
        self.assertTrue(entity.get('armed'))

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, UNREADABLE_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_DISAPPEARED} never raised for {UNREADABLE_NODE} - with no '
                'require_active entry anywhere, this death was reported by nobody at all')
        self.assertIn(UNREADABLE_NODE, fault.get('description', ''))

        # And nothing else picked it up, which is the point of the row: in this configuration
        # the lifecycle codes are structurally unreachable, so the presence code standing alone
        # is the whole of the operator's warning.
        assert_fault_absent_throughout(
            self, PORT, FAULT_CODE_UNREADABLE, SILENCE_WINDOW_SEC)


class TestMeasuredNodeIsOwnedByPresence(unittest.TestCase):
    """The same fixture, told to answer: once its state IS known, node_death owns its departure.

    Runs under the SAME configuration as "budget_spent_then_owned" - `require_active` names the
    fixture in both - so the only variable between the two rows is what the node's lifecycle
    state turned out to be. Refusing ownership for every managed node, or granting it to
    everything, is ruled out by the pair rather than by either row alone: this one is measured
    "active" and must be reported by the presence code, its sibling is never measured at all and
    must be too, and B6 in test_node_death_boundary_e2e.test.py is measured NOT-active and must
    not be.

    The node is unmeasured across the whole warmup first (proven from the status route before
    anything else happens), then answers, then dies, so what this row exercises is a CHANGE in
    knowledge rather than a node that started out measurable.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('presence_ownership_answer_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_a_node_that_starts_answering_is_owned_by_presence(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=UNREADABLE_NODE),
            f'graph_watchdog never reported {UNREADABLE_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        # The precondition that makes this a CHANGE rather than a steady state: armed, and
        # unmeasured, before the flip.
        entity = _poll_watchdog_entity(PORT, UNREADABLE_NODE, '', timeout=LABEL_TIMEOUT_SEC)
        self.assertIsNotNone(
            entity,
            f'{UNREADABLE_NODE} was never observed armed-and-unmeasured, so the flip below would '
            'not be a transition from ignorance to knowledge')
        self.assertTrue(entity.get('armed'))

        self.assertTrue(
            _set_bool_parameter(
                type(self)._client_node, f'/{UNREADABLE_NODE}/set_parameters',
                ANSWER_PARAM, True, timeout=PARAM_SET_TIMEOUT_SEC),
            f'setting {ANSWER_PARAM}:=true on /{UNREADABLE_NODE}/set_parameters never succeeded - '
            'the fixture never started answering, so this row would test the sibling scenario')

        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNREADABLE_NODE, 'active', timeout=LABEL_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} never reported an "active" lifecycle label after the flip - its '
            'state is still unmeasured, so a silent GRAPH_NODE_DISAPPEARED below would be '
            'correct rather than a defect')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, UNREADABLE_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_DISAPPEARED} never raised for {UNREADABLE_NODE} - a node whose '
                "lifecycle state WAS measured active before it died is the presence detector's "
                'to report, and refusing every managed node would look exactly like this')
        self.assertIn(UNREADABLE_NODE, fault.get('description', ''))


class TestMeasuredThenUnmeasured(unittest.TestCase):
    """A node whose MEASUREMENT goes away, on a live node, and then it dies.

    The reverse of the sibling rows, and the direction an injected label cannot reach: this
    fixture is measured "active" first - so the presence detector owns it outright - and then
    stops advertising its lifecycle services, which makes the watcher drop the record
    altogether. The status route reports `lifecycle: null` afterwards, the same value a node
    that never had a lifecycle reports, and that is the state the node is in when it is killed.

    GRAPH_NODE_DISAPPEARED must still raise. Ownership is latched from the fact the gate gave
    while the node was measurable, and a node with no managed record is owned outright anyway,
    so both routes to the answer agree here. What the row rules out is an implementation that
    re-derives ownership from whatever the watcher happens to say at the moment of death and
    finds nothing to go on.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('presence_ownership_drop_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_a_node_that_stops_being_measurable_is_still_owned(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=DROPPABLE_NODE),
            f'graph_watchdog never reported {DROPPABLE_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, DROPPABLE_NODE, 'active', timeout=LABEL_TIMEOUT_SEC),
            f'{DROPPABLE_NODE} never reported an "active" lifecycle label, so the measurement '
            'this row is about losing was never made in the first place')

        self.assertTrue(
            _set_bool_parameter(
                type(self)._client_node, f'/{DROPPABLE_NODE}/set_parameters',
                DROP_PARAM, True, timeout=PARAM_SET_TIMEOUT_SEC),
            f'setting {DROP_PARAM}:=true on /{DROPPABLE_NODE}/set_parameters never succeeded - '
            'the node never stopped looking like a managed lifecycle node')

        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, DROPPABLE_NODE, None, timeout=LABEL_TIMEOUT_SEC),
            f'{DROPPABLE_NODE} still carries a lifecycle label after its services were dropped - '
            'the watcher never let go of the record, so the transition this row needs did not '
            'happen')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, DROPPABLE_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{DROPPABLE_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_DISAPPEARED} never raised for {DROPPABLE_NODE} - a node that was '
                'measured active and then lost its lifecycle interface is still the presence '
                "detector's to report")
        self.assertIn(DROPPABLE_NODE, fault.get('description', ''))


class TestRestartInsideWarmup(unittest.TestCase):
    """A crash loop whose uptime is shorter than the warmup: the second death must be reported.

    Handing a key back is for one situation only - the graph has MEASURED the node as something
    the presence detector must not own. A node that is merely re-warming has been measured as
    nothing at all: the gate withholds a claim because warmup is incomplete, which is the
    absence of knowledge, not knowledge of another owner. Releasing a key on that answer costs
    two things at once, and this row asserts both. The node's next death goes unreported,
    because the key is no longer tracked; and the detector reports PASSED every tick while the
    node is dead, because an empty dead set reads as health.

    The other restart rows in this package all wait for the returned node to arm before killing
    it again, so none of them can see this: they only ever kill an armed node. This one kills
    inside the re-warm window on purpose, and proves it was inside by reading the entity's own
    state off the watchdog route rather than by timing.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_a_second_death_inside_the_rewarm_window_is_still_reported(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=PLAIN_NODE),
            f'graph_watchdog never reported {PLAIN_NODE} armed, so the first death below could '
            'never have been tracked at all')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, PLAIN_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{PLAIN_NODE} never left GET /apps after the first SIGTERM')
        first = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if first is None:
            self.fail(
                f'{FAULT_CODE_DISAPPEARED} never raised for the first death of {PLAIN_NODE} - '
                'there is no first death here for a second one to follow')
        self.assertIn(PLAIN_NODE, first.get('description', ''))

        # The node comes back under the same name and a new pid. Everything after this point
        # happens inside its re-warm window, which is why nothing here waits for arming.
        self.assertTrue(
            _poll_apps_present(PORT, PLAIN_NODE, timeout=RESTART_RETURN_TIMEOUT_SEC),
            f'{PLAIN_NODE} never came back into GET /apps after its respawn')
        state = _poll_watchdog_entity_state(PORT, PLAIN_NODE, timeout=LABEL_TIMEOUT_SEC)
        self.assertEqual(
            state, 'warming_up',
            f'{PLAIN_NODE} was already {state!r} when it came back, so the kill below would land '
            'AFTER the re-warm window and this row would be a second copy of the restart-loop '
            'scenarios that already pass')

        before = _fault_record(PORT, FAULT_CODE_DISAPPEARED, timeout=DEPARTURE_TIMEOUT_SEC)
        self.assertIsNotNone(
            before, f'{FAULT_CODE_DISAPPEARED} left the store entirely once {PLAIN_NODE} came '
                    'back, so there is no record for the second death to move')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, PLAIN_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{PLAIN_NODE} never left GET /apps after the second SIGTERM')

        deadline = time.monotonic() + RAISE_TIMEOUT_SEC
        after = before
        while time.monotonic() < deadline:
            after = _fault_record(PORT, FAULT_CODE_DISAPPEARED, timeout=5.0) or after
            if after.get('last_occurred') != before.get('last_occurred'):
                break
            time.sleep(0.5)
        self.assertNotEqual(
            after.get('last_occurred'), before.get('last_occurred'),
            f'{FAULT_CODE_DISAPPEARED} was never reported again for the second death of '
            f'{PLAIN_NODE} (last_occurred stayed {before.get("last_occurred")!r}) - the key was '
            'handed back while the node was merely re-warming, so nothing was tracking it when '
            'it died')

        # The other half of the same defect: with nothing tracked, an empty dead set reads as
        # health and the detector reports PASSED for a node that is lying dead in front of it.
        passed_at_confirmation = after.get('last_passed')
        deadline = time.monotonic() + SILENCE_WINDOW_SEC
        polls = 0
        while time.monotonic() < deadline:
            record = _fault_record(PORT, FAULT_CODE_DISAPPEARED, timeout=5.0)
            self.assertIsNotNone(
                record, f'{FAULT_CODE_DISAPPEARED} left the store {polls} poll(s) into the '
                        'window that was supposed to show no PASSED at all')
            self.assertEqual(
                record.get('last_passed'), passed_at_confirmation,
                f'{FAULT_CODE_DISAPPEARED} was reported PASSED {polls} poll(s) after it was '
                f'confirmed dead again (last_passed moved to {record.get("last_passed")!r}) - '
                f'{PLAIN_NODE} is still gone, so that is a heal for a node nobody is watching')
            polls += 1
            time.sleep(0.5)
        self.assertGreater(polls, 0, 'the no-PASSED window never actually polled the store')


class TestProvisionalOwnershipYieldsToARealLabel(unittest.TestCase):
    """A provisional owner hands the node back the moment the graph says whose it is.

    Ownership granted because the asking stopped is granted for one reason only: nobody else
    can report this node. That reason is not permanent. LifecycleWatcher stops issuing GetState
    when the re-seed budget runs out, but it never drops the ~/transition_event subscription, so
    a real label can still arrive afterwards - and a label saying the node is inactive says the
    node was lifecycle_expectation's all along.

    Waiting for the budget alone would only DELAY the defect this slice exists to close: the
    presence detector would latch the node during the unread window and still be holding it when
    it died inactive. So the row drives the whole sequence - unread, budget spent, label arrives
    non-active, node killed - and asserts the handover happened: GRAPH_NODE_INACTIVE names the
    node, GRAPH_NODE_DISAPPEARED never appears.

    This is also the row that rejects a detector wired to the permissive `reliability_allows()`
    where `presence_ownership()` belongs: that predicate is TRUE for an unread label, tracking is
    sticky, and the node would be reported dead here.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node(LATE_LABEL_CLIENT_NODE)

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_a_late_non_active_label_takes_the_node_back(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=UNREADABLE_NODE),
            f'graph_watchdog never reported {UNREADABLE_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        # The provisional grant, OBSERVED rather than waited out: an unread label whose
        # measurement is no longer pending is exactly the state in which the presence detector
        # takes the node for want of anyone else.
        entity = _poll_watchdog_entity(
            PORT, UNREADABLE_NODE, '', timeout=LABEL_TIMEOUT_SEC, measurement_pending=False)
        self.assertIsNotNone(
            entity,
            f'{UNREADABLE_NODE} never reached an unread label with the asking finished - without '
            'that state there is no provisional grant for the label below to revoke')
        self.assertTrue(entity.get('armed'))

        # Everything else that will ever be admitted must be admitted FIRST, or the baseline
        # below is measured against a moving total. This process's own client node is the one
        # that moves it: it is an ordinary App, node_death admits it like any other, and an
        # admission landing between the release and the sample looking for it puts the count
        # straight back where it started. Measured on a live stack: the release dipped the
        # count for 200 ms and the client node's admission refilled it, which a 200 ms poll
        # sees as no release at all.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC,
                                      app_id=LATE_LABEL_CLIENT_NODE),
            f'{LATE_LABEL_CLIENT_NODE} never armed, so an admission of it could still land on '
            'top of the release this row has to see')

        # And the baseline has to include the fixture's OWN admission, which the gate licenses
        # a tick before the detector acts on it. Sampling until the total stops moving is what
        # puts it inside; capturing on the gate's answer alone captures the number the release
        # will later return to, and "fell below" is then never true however long it waits.
        tracked_before = _poll_stable_tracked_count(PORT, DETECTOR_ID_NODE_DEATH,
                                                    timeout=LABEL_TIMEOUT_SEC)
        self.assertIsNotNone(
            tracked_before,
            "node_death's tracked_count never settled, so there is no baseline here that the "
            'release below could be measured against')
        self.assertGreater(
            tracked_before, 0,
            'node_death is tracking nothing at all, so there is no admitted key here for the '
            'label to take back')

        self.assertTrue(
            _set_bool_parameter(
                type(self)._client_node, f'/{UNREADABLE_NODE}/set_parameters',
                ANSWER_PARAM, True, timeout=PARAM_SET_TIMEOUT_SEC),
            f'setting {ANSWER_PARAM}:=true on /{UNREADABLE_NODE}/set_parameters never succeeded - '
            'the late label this row turns on never arrived')

        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNREADABLE_NODE, LATE_LABEL, timeout=LABEL_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} never reported the "{LATE_LABEL}" label - the graph never said '
            'whose node this is, so a presence report below would not be a defect')

        # The label being READABLE is the watcher's fact, and it is not the one this row needs.
        # node_death hands a provisionally owned key back inside its own tick, and only on a
        # tick that still sees the app present - a kill landing between the label appearing on
        # this route and that tick finds the key admitted, and the death is then reported by a
        # detector the label had already disqualified. Measured on a live stack: the label
        # became visible 210 ms before tracked_count dropped. So wait for the DETECTOR's own
        # view to move before killing anything; gating on the label alone tests the watcher and
        # calls it the detector.
        self.assertTrue(
            _poll_tracked_count_below(PORT, DETECTOR_ID_NODE_DEATH, tracked_before,
                                      timeout=LABEL_TIMEOUT_SEC),
            f'node_death never released {UNREADABLE_NODE} after the "{LATE_LABEL}" label - its '
            f'tracked_count never fell below {tracked_before}, so the key was still admitted '
            'and the kill below would be measuring the window rather than the handover')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, UNREADABLE_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} never left GET /apps after SIGTERM')

        # The positive control, and it runs first: the node IS reported, by the detector the
        # label handed it to. An absence assertion after a silent stack would prove nothing.
        found, description = poll_fault_describing(
            PORT, FAULT_CODE_INACTIVE, [UNREADABLE_NODE], timeout=RAISE_TIMEOUT_SEC)
        self.assertTrue(
            found,
            f'{FAULT_CODE_INACTIVE} never raised naming {UNREADABLE_NODE} - the node the presence '
            'detector handed back was then reported by nobody at all '
            f'(last description: {description!r})')

        # Needle-scoped, and the distinction is not cosmetic: the code-only form fails for a
        # GRAPH_NODE_DISAPPEARED about ANY node, so its red cannot say whether the handover
        # broke or some unrelated entity died, and a reader has to go and find out. This row's
        # claim is about one node, so the assertion names it and the next failure carries the
        # answer. Anything else disappearing here is not this row's business.
        assert_fault_never_names(
            self, PORT, FAULT_CODE_DISAPPEARED, forbidden=[UNREADABLE_NODE],
            duration=SILENCE_WINDOW_SEC)


class TestGateStaysPermissiveForTheOtherDetectors(unittest.TestCase):
    """The three other detectors still report in a graph carrying an unmeasured managed node.

    The guard on having left ReliabilityGate::allows_raise() and LifecycleWatcher::node_ok()
    alone. Tightening either one would have been the shorter fix and would have silenced every
    fault about a node whose GetState never answers.

    The three legs are not equal, and saying so is the point:

    - GRAPH_PARAM_DRIFT is app-keyed. param_drift builds its read set from
      reliability_allows(ctx.gate, app_id), so this fixture is read at all only while the gate
      stays permissive about an unread label. This leg discriminates.
    - GRAPH_QOS_MISMATCH and GRAPH_ORPHAN are keyed by topic, not by App::id, and neither
      detector consults the per-entity gate - orphan_detector.cpp states it outright. They
      cannot be suppressed by an app-keyed predicate however it is written, so these two legs
      are regression guards on the detectors still running at all in this graph, not evidence
      about the gate. The QoS pair does include the fixture's own publisher.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # A finite REQUESTED deadline against the fixture's own no-deadline OFFER on
        # ~/transition_event: RxO-incompatible, so the mismatched pair has the unmeasured
        # managed node on one end of it. Never spun - DDS endpoint discovery is what the
        # detector reads, and it is populated by DDS's own background discovery.
        cls._qos_node = Node('presence_ownership_qos_sub')
        cls._qos_sub = cls._qos_node.create_subscription(
            TransitionEvent, QOS_TOPIC, lambda _msg: None,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       deadline=Duration(seconds=QOS_SUB_DEADLINE_SEC)))

        # One publisher-only topic and one subscriber-only topic, same type, same namespace,
        # leaf edit distance 1 - the remap-typo signature the orphan detector looks for.
        cls._orphan_node = Node('presence_ownership_orphan')
        cls._orphan_pub = cls._orphan_node.create_publisher(
            LaserScan, ORPHAN_TYPO_TOPIC, QoSProfile(depth=10))
        cls._orphan_sub = cls._orphan_node.create_subscription(
            LaserScan, ORPHAN_TARGET_TOPIC, lambda _msg: None, QoSProfile(depth=10))

    @classmethod
    def tearDownClass(cls):
        cls._orphan_node.destroy_node()
        cls._qos_node.destroy_node()
        rclpy.shutdown()

    def test_01_param_drift_still_reads_the_unmeasured_managed_node(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=UNREADABLE_NODE),
            f'graph_watchdog never reported {UNREADABLE_NODE} armed')
        entity = _poll_watchdog_entity(PORT, UNREADABLE_NODE, '', timeout=LABEL_TIMEOUT_SEC)
        self.assertIsNotNone(
            entity,
            f'{UNREADABLE_NODE} never appeared with an EMPTY lifecycle label - without an '
            'unmeasured managed node in the graph none of the three legs below is about anything')

        # The needle is the descriptor param_drift actually writes: "<app_id>:<name> expected=..",
        # with the NAME json-dumped (ParamDriftPolicy::safe_name), so it carries its own quotes.
        found, description = poll_fault_describing(
            PORT, FAULT_CODE_PARAM_DRIFT, [f'{UNREADABLE_NODE}:"{ANSWER_PARAM}"'],
            timeout=RAISE_TIMEOUT_SEC)
        self.assertTrue(
            found,
            f"{FAULT_CODE_PARAM_DRIFT} never named {UNREADABLE_NODE}'s {ANSWER_PARAM} - the "
            'app-keyed gate stopped permitting a node whose lifecycle label was never read, so '
            f'param_drift never read it (last description: {description!r})')

    def test_02_qos_mismatch_still_reports_a_pair_including_that_node(self):
        found, description = poll_fault_describing(
            PORT, FAULT_CODE_QOS, [QOS_TOPIC], timeout=RAISE_TIMEOUT_SEC)
        self.assertTrue(
            found,
            f'{FAULT_CODE_QOS} never named {QOS_TOPIC} - a deadline-incompatible subscriber '
            "against the fixture's own publisher went unreported "
            f'(last description: {description!r})')

    def test_03_orphan_still_reports_a_near_miss_pair(self):
        found, description = poll_fault_describing(
            PORT, FAULT_CODE_ORPHAN, [ORPHAN_TYPO_TOPIC, ORPHAN_TARGET_TOPIC],
            timeout=RAISE_TIMEOUT_SEC)
        self.assertTrue(
            found,
            f'{FAULT_CODE_ORPHAN} never named the {ORPHAN_TYPO_TOPIC} / {ORPHAN_TARGET_TOPIC} '
            f'pair (last description: {description!r})')


# Each CTest target launches this file with one scenario, so only that scenario's case may run.
# Removing the others from the module (rather than skipping them) means each run reports exactly
# one case, and a missing result is a real failure rather than an expected line of output - see
# the sibling e2e files' identical rationale.
_SCENARIO_CASES = {
    'restart_inside_warmup': 'TestRestartInsideWarmup',
    'provisional_yields_to_a_real_label': 'TestProvisionalOwnershipYieldsToARealLabel',
    'budget_spent_then_owned': 'TestBudgetSpentThenOwned',
    'no_require_active_still_owned': 'TestNoRequireActiveStillOwned',
    'answered_then_owned': 'TestMeasuredNodeIsOwnedByPresence',
    'measured_then_unmeasured': 'TestMeasuredThenUnmeasured',
    'gate_unaffected': 'TestGateStaysPermissiveForTheOtherDetectors',
}
if SCENARIO not in _SCENARIO_CASES:
    raise RuntimeError(
        f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} is not one of {sorted(_SCENARIO_CASES)}; the CTest '
        'target and this file disagree about which scenarios exist')
for _scenario, _case_name in _SCENARIO_CASES.items():
    if _scenario != SCENARIO:
        del globals()[_case_name]


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway/fault_manager/fixture stack exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
