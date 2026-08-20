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

Runs as THREE separate CTest targets (see CMakeLists.txt). WATCHDOG_E2E_SCENARIO selects which
launch and which assertions run:

- "unmeasured_not_owned": the fixture is never told to answer, so its lifecycle label stays ""
  for its whole life, and it is killed. GRAPH_NODE_UNREADABLE raises and names it (the positive
  control: the detector stack is alive, the node was matched, and its departure was processed),
  and GRAPH_NODE_DISAPPEARED must never appear - node_death cannot own a node whose state it has
  never once measured. The absence window opens AFTER the positive control has already come
  back, so an implementation that does own it has had many times the configured miss grace to
  raise, and node_death's own fault does not heal on its own while the node stays gone.
- "answered_then_owned": the same fixture, unmeasured across the whole warmup (proven from the
  status route, not assumed), is then told to answer via its `start_answering` parameter. Once
  the label reads "active" the node is killed, and GRAPH_NODE_DISAPPEARED MUST raise naming it.
  This is what stops the fix being "never own a managed node": ownership follows the current
  knowledge, and a node that becomes measurable becomes owned.
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

Every scenario here gates on `app_id=unreadable_lifecycle`, the node it perturbs. That works
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
    create_watchdog_test_launch,
    poll_fault_describing,
    poll_faults,
    wait_until_faults_endpoint_live,
    wait_until_watchdog_armed,
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
FAULT_CODE_UNREADABLE = 'GRAPH_NODE_UNREADABLE'
FAULT_CODE_PARAM_DRIFT = 'GRAPH_PARAM_DRIFT'
FAULT_CODE_QOS = 'GRAPH_QOS_MISMATCH'
FAULT_CODE_ORPHAN = 'GRAPH_ORPHAN'

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

# How long GRAPH_NODE_DISAPPEARED is watched for after the departure has already been reported by
# another detector. Many times MISS_GRACE's own 3400 ms nominal window, and node_death's fault
# does not heal while the node stays gone, so a raise that happened at any point after the kill
# is still standing when this window opens.
SUSTAINED_WINDOW_SEC = 20.0

# The QoS leg's mismatched pair. The fixture publishes ~/transition_event with no deadline (the
# most permissive offer); a subscriber that REQUESTS a finite deadline is RxO-incompatible with
# it - see qos_policy.hpp's qos_incompatibility() deadline branch, and test_qos_e2e.test.py's
# test_04, which pins the same dimension against a purpose-built pair.
QOS_TOPIC = f'/{UNREADABLE_NODE}/transition_event'
QOS_SUB_DEADLINE_SEC = 0.1

# The orphan leg's near-miss pair: one publisher-only topic and one subscriber-only topic, same
# type, same namespace, leaf edit distance 1. Both endpoints belong to this test process; the
# detector keys its finding on the TOPIC, so there is no way to make either of them the
# fixture's own (its only non-system topic already carries endpoints on both sides).
ORPHAN_TYPO_TOPIC = f'/{UNREADABLE_NODE}/scam'
ORPHAN_TARGET_TOPIC = f'/{UNREADABLE_NODE}/scan'


def _unreadable_node_action():
    """Build the fixture as a launch action with a PID handle the test can signal.

    Uses DEMO_NODE_REGISTRY's own (executable, ros_name, namespace) triple so this stays in
    lockstep with demo_nodes.launch.py, built by hand only because the scenarios here signal the
    process directly and create_demo_nodes() hands back no PID.
    """
    executable, ros_name, namespace = DEMO_NODE_REGISTRY[UNREADABLE_NODE]
    return launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable=executable,
        name=ros_name,
        namespace=namespace,
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

    if SCENARIO == 'unmeasured_not_owned':
        # require_active is what makes lifecycle_expectation look at this node at all, and its
        # GRAPH_NODE_UNREADABLE is this scenario's positive control.
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [UNREADABLE_NODE]
    elif SCENARIO == 'answered_then_owned':
        # No require_active: node_death is zero-config and this row is entirely about it, so
        # leaving lifecycle_expectation with nothing to watch keeps the fault surface clean.
        pass
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

    target = _unreadable_node_action()
    launch_description.add_action(TimerAction(period=2.0, actions=[target]))
    context['target_node'] = target
    return launch_description, context


def _poll_watchdog_entity(port, app_id, lifecycle, timeout=30.0, interval=0.5):
    """Poll GET /x-medkit-watchdog until `app_id` appears with this lifecycle label.

    The PAIR is the instrument these scenarios need, not `armed` on its own: `armed` reads the
    same for a plain node and for a managed one whose state nobody has ever read, and the whole
    claim is about telling those two apart. ``''`` is not "no data yet" - LifecycleWatcher seeds
    a TRACKED entry's label to ``''`` and only overwrites it once a real read succeeds, so it
    means "asked, and still waiting", while a node with no managed record at all reports null.

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


class TestUnmeasuredNodeIsNotOwnedByPresence(unittest.TestCase):
    """A managed node whose state was never measured departs: UNREADABLE, never DISAPPEARED.

    node_death may only report a node it can reliably observe, and it decides that from the
    reliability gate. The gate's per-entity answer is deliberately permissive about a lifecycle
    label that has never been read - every other detector depends on that, or a node whose
    GetState never answers would have every fault suppressed forever - so tracking cannot be
    admitted on that answer alone. Reading it as ownership took the departure away from the one
    detector that could still report it and gave it to one that structurally cannot.

    Both halves are checked: GRAPH_NODE_UNREADABLE must raise and name the node (the departure
    was seen, by a live stack, for the node this row is about), and GRAPH_NODE_DISAPPEARED must
    stay absent for a window many times the configured miss grace.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_unmeasured_departure_is_lifecycles_and_never_presences(self, target_node):
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

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, UNREADABLE_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} never left GET /apps after SIGTERM')

        # Positive control, and it runs FIRST on purpose: an absence proves nothing about a
        # stack that stopped reporting, and this shows the detectors processed THIS departure.
        # Whichever way the timing falls, the window below still catches a wrong owner: it is
        # 20 s of continuous polling against a 3.4 s nominal miss grace, so a raise either
        # happens inside it or is already standing when it opens - node_death's fault does not
        # heal while the node stays gone.
        found, description = poll_fault_describing(
            PORT, FAULT_CODE_UNREADABLE, [UNREADABLE_NODE],
            timeout=UNREADABLE_RAISE_TIMEOUT_SEC)
        self.assertTrue(
            found,
            f'{FAULT_CODE_UNREADABLE} never raised naming {UNREADABLE_NODE} - the lifecycle '
            f'promise this node never kept went unreported (last description: {description!r})')

        assert_fault_absent_throughout(
            self, PORT, FAULT_CODE_DISAPPEARED, SUSTAINED_WINDOW_SEC)


class TestMeasuredNodeIsOwnedByPresence(unittest.TestCase):
    """The same fixture, told to answer: once its state IS known, node_death owns its departure.

    The discriminating half. Refusing ownership for every managed node would satisfy the sibling
    scenario just as well, and would silently disable GRAPH_NODE_DISAPPEARED for every real
    lifecycle node on the graph. Ownership follows the CURRENT knowledge: the fixture is
    unmeasured across the whole warmup (proven from the status route before anything else
    happens), then answers, then dies, and the presence code must report it.
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
    'unmeasured_not_owned': 'TestUnmeasuredNodeIsNotOwnedByPresence',
    'answered_then_owned': 'TestMeasuredNodeIsOwnedByPresence',
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
