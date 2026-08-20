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
"""node_death boundary, config and instrument e2e: the seam with lifecycle_expectation.

Sibling of test_node_death_e2e.test.py and test_node_death_suppression_e2e.test.py, same shape.
What makes this file different is B1 and B3: their claims are about lifecycle_expectation ALONE,
independent of node_death, so an absence assertion there still discriminates a correct detector
from no detector at all - see their own class docstrings for what was actually observed.

Runs as EIGHT separate CTest targets (see CMakeLists.txt). WATCHDOG_E2E_SCENARIO selects which
launch and which assertions run:

- "b1_inactive_present": a require_active node (managed_lifecycle, never activated) sits in the
  graph past `grace`. GRAPH_NODE_INACTIVE raises and names it - GENUINELY GREEN, because
  lifecycle_expectation already ships this. GRAPH_NODE_DISAPPEARED stays absent for the whole
  window - absence alone cannot distinguish a detector that correctly stays silent here from
  one that raises nothing at all, but staying silent is also the permanently correct behaviour,
  since this node never leaves the graph.
- "b2_inactive_below_grace_then_gone": the same node, now driven to "active" FIRST (arming it
  for node_death - see TestBoundaryInactiveBelowGraceThenGone's own class docstring for why
  that is not optional), then to "inactive" via a real DEACTIVATE transition, then killed after
  being observed non-active for only a couple of ticks - comfortably below `grace`. Absence may
  continue a violation that has already matured, but must never mature one that has not: this row
  pins the defect where GRAPH_NODE_INACTIVE used to mature from evidence gathered entirely after
  the node could no longer be observed, proven with assert_fault_absent_throughout rather than
  presented as prose. The below-grace precondition is itself proven from an OBSERVABLE
  (GRAPH_NODE_INACTIVE still absent from /faults) immediately before the kill, not merely
  inferred from how little time has elapsed - status_json() exposes no per-node violation-streak
  count (see lifecycle_expectation_detector.cpp), so the fault surface is the only external
  evidence available, and with confirmation_threshold=-2 the detector's first FAILED report IS
  the confirmation. Both halves of the row are now reachable together: no GRAPH_NODE_INACTIVE is
  born from the departure, and GRAPH_NODE_DISAPPEARED raises because the node was armed before
  it died.
- "b3_matured_then_gone": a require_active node driven to "active" first (arming it for
  node_death - see TestBoundaryMaturedThenGone's own class docstring for why that is not
  optional), then to "inactive" via a real DEACTIVATE transition and left there long enough
  for GRAPH_NODE_INACTIVE to CONFIRM, before it is killed. The confirmed fault survives the
  departure AND keeps naming the node (content follows the clocks, not the snapshot - already
  shipped, GENUINELY proven here with assert_fault_describes_only rather than a code-only
  persistence check, which a detector that dropped this node while keeping the code raised for
  another one would still pass), and GRAPH_NODE_DISAPPEARED now joins it too, since the node
  was genuinely armed before it died.
- "b4_healthy_then_gone": the self-activating variant (managed_lifecycle_active) reaches "active"
  and is then killed outright. No GRAPH_NODE_INACTIVE is born from a healthy departure
  (release_uncorroborated - already shipped, genuinely proven here), but GRAPH_NODE_DISAPPEARED
  never raises. RED.
- "b5_restart_loop_still_caught": the same require_active node, now reaching "active" on EVERY
  respawn (not merely the first start - see TestBoundaryRestartLoopStillCaught's own class
  docstring for why a node that never arms would make this row unsatisfiable by any correct
  implementation), restart-looping under this test's own control, killed and confirmed back
  three times over. Nothing here checks GRAPH_NODE_INACTIVE - that is B2's and B3's row. This
  one is entirely about whether the presence code catches the departure EVERY cycle, which is
  what makes it safe to forbid absence from maturing an unmatured streak: once
  GRAPH_NODE_DISAPPEARED independently catches a restart loop, lifecycle_expectation no longer
  has to evade it via an unmatured streak maturing on absence. Configures
  detectors.node_death.miss_grace explicitly (B5_MISS_GRACE, comfortably past the documented 3000
  ms floor) and drives the node's own respawn on a delay
  (B5_RESPAWN_DELAY_SEC) safely longer than that nominal grace, so a correct detector CAN
  report every cycle - left at the 1.5 s launch-respawn floor, the outage would be shorter than
  the floor the detector enforces and this row could never turn green for a right
  implementation, only a wrong one lucky enough to report anyway.
- "c4_config_endpoint_e2e": ONE gateway, ONE armed node, killed once, with a LARGE
  detectors.node_death.miss_grace configured (C4_MISS_GRACE_LARGE, comfortably under the
  documented 3600-tick ceiling). Proves the knob governs an observable by checking the SAME
  gateway at two points on ONE timeline rather than comparing two gateways: a window
  (C4_EARLY_WINDOW_SEC) long enough that a near-floor config (this suite's own ~4s convention -
  B5_MISS_GRACE, D2_MISS_GRACE) would already have raised, in which this large-grace gateway
  must stay silent - needle-scoped (assert_fault_never_names), so a raise for some unrelated
  entity cannot decide the row - then, once the configured grace has had time to elapse, the
  fault must still arrive and name the node: the large value delays the report, it does not
  swallow it. RED.
- "b6_never_armed_below_grace_then_gone": TARGET_NODE launched WITHOUT auto_activate, so it
  sits at "unconfigured" its entire life and never once reaches "active" - the gate's
  per-entity `armed` precondition (LifecycleWatcher::node_ok()) is therefore false for the
  whole test, and node_death can never track it (NodeLivenessTracker only ever tracks a key
  the gate has armed at least once). Killed after being observed non-active for only a
  couple of ticks - comfortably below `grace`, proven the same way B2 proves it (an
  immediate, single-instant read of GET /faults right before the kill, not inferred from
  elapsed time). With node_death structurally unable to report this departure,
  GRAPH_NODE_INACTIVE is the only detector that ever could - so unlike B2, absence maturing
  the below-grace streak is not a defect here, it is the whole point: this is the row that
  proves the silence a node like this used to fall into is gone.
- "d2_ungated_clear": a death is confirmed, then the GATEWAY is restarted with a generous
  warmup_cycles so the pre-arm window is wide enough to sample. During that window - before the
  restarted plugin's own gate has armed anything - the stored fault's `last_passed` must stay
  unset the whole time: an ungated detector tick must never report PASSED for a node it has not
  actually measured in this process's lifetime. The window is bracketed by two single-shot reads
  of the watchdog's own global_state, one immediately before it opens and one immediately after
  it closes, both asserted != "armed" - restart-plus-recovery eating the whole nominal warmup
  would otherwise let the sampled window land AFTER arming, and a legitimate post-arm PASSED
  would then be misread as the ungated-clear bug this row exists to catch.

### Which arming gate, and why B1 alone uses the global form

The default rule every scenario in this file follows: gate on `app_id=<the node a scenario
perturbs>`, and reserve the global form for a scenario that perturbs the gateway itself. B1 and
B6 are the documented exceptions, for a reason specific to a require_active node rather than a
style choice: ONE of
`ReliabilityGate`'s own preconditions for a per-entity `armed` state is
`LifecycleWatcher::node_ok()`, which is false for exactly a tracked node that is not (yet)
"active" - the very state both targets sit in for their whole life, on purpose, since neither
drives its node past "unconfigured" at all - that IS each row's claim (B1: still present; B6:
gone before its own grace). Gating on `app_id=managed_lifecycle` would therefore wait for
something that never becomes true in either scenario. This is not a new call:
test_lifecycle_expectation_e2e.test.py's own "main" scenario already gates the identical fixture
globally, for the identical reason, and this file follows that precedent rather than inventing a
new one.

B2, B3, B4 and B5 all use the app_id form instead, because for each of them the per-entity
precondition genuinely becomes true: B4's target (managed_lifecycle_active) has always
self-activated on its own; B2, B3 and B5 now launch TARGET_NODE (managed_lifecycle) with
auto_activate too, specifically so node_death can track it at all - see
`_lifecycle_node_action`'s own docstring and B2/B3/B5's own class docstrings for why a target
that never reaches "active" would make GRAPH_NODE_DISAPPEARED structurally unreachable for any
of the three, not merely slower to catch. B6 is the one row where that same unreachability is
not a precondition to avoid but the claim under test, so it deliberately launches TARGET_NODE
the OTHER way - without auto_activate - and asserts GRAPH_NODE_DISAPPEARED stays silent rather
than waiting for it.

B2 is the row where getting this wrong is easiest to miss: gating globally and never activating
the target would still be correct for the INACTIVE half (which needs the node observed below
`grace`, so it must never mature past it) while being fatal for the DISAPPEARED half (which
needs node_death to have tracked the node at all, and a node that never arms is structurally
invisible to it regardless of what kills it) - a row built that way could pass while silently
proving nothing about the half it got wrong. B2 therefore follows the same rule every other row
in this file uses: reach "active" first, however briefly, before doing anything else to the node
a DISAPPEARED claim depends on.
"""

import os
import signal
import sys
import time
import unittest

from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node
import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above, so this
# import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    API_BASE_PATH,
    assert_fault_absent_throughout,
    assert_fault_describes_only,
    assert_fault_never_names,
    assert_fault_persists_throughout,
    create_watchdog_test_launch,
    poll_cleared,
    poll_detector_status,
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

# No default on purpose - see harness-consuming siblings' identical rationale: a default makes
# this file FAIL OPEN. A KeyError is loud.
SCENARIO = os.environ['WATCHDOG_E2E_SCENARIO']
PORT = get_test_port()

FAULT_CODE_INACTIVE = 'GRAPH_NODE_INACTIVE'
FAULT_CODE_DISAPPEARED = 'GRAPH_NODE_DISAPPEARED'
DETECTOR_ID_LIFECYCLE = 'lifecycle_expectation'
_LIFECYCLE_PREFIX = f'plugins.graph_watchdog.detectors.{DETECTOR_ID_LIFECYCLE}'
_NODE_DEATH_PREFIX = 'plugins.graph_watchdog.detectors.node_death'

TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3

# The require_active node B1/B2/B3/B5 share: DEMO_NODE_REGISTRY's own 'managed_lifecycle' key -
# stays "unconfigured" (never active) unless driven otherwise, the trigger every one of those
# rows needs. B4 uses the self-activating 'managed_lifecycle_active' sibling instead.
TARGET_NODE = 'managed_lifecycle'
ACTIVE_NODE = 'managed_lifecycle_active'

# grace small enough that B1 and B3 mature quickly - same value
# test_lifecycle_expectation_e2e.test.py's own GRACE constant uses, for the same reason.
GRACE = 3
# B2's own grace: large enough that "killed after a couple of observed ticks" is unambiguously
# BELOW it, whatever small overshoot this scenario's own polling interval costs.
B2_GRACE = 15
# B6's own grace: same magnitude as B2_GRACE and for the identical reason - "killed after a
# couple of observed ticks" must be unambiguously BELOW it.
B6_GRACE = 15
# B5's own grace: large relative to one restart cycle's uptime. Not load-bearing for what this
# row actually asserts (see the module docstring - B5 checks GRAPH_NODE_DISAPPEARED only), kept
# generous so lifecycle_expectation's own behaviour cannot accidentally become this row's story.
B5_GRACE = 50
# B5's own node_death.miss_grace, explicitly configured rather than left at whatever default a
# future detector ships with. min_node_death_miss_grace(TICK_INTERVAL_MS) - the wall-clock floor
# node_death's own configure() silently raises an under-sized value to - is 14 at a 200 ms tick
# (detector_config_keys.hpp: ceil(3000 / 200) - 1). "config sweep C1"
# (NodeDeathIntegrationTest.C1_* in test_node_death_integration.cpp) is the suite that pins the
# floor's own boundary values, but it never exercises tick_interval_ms=200 at all (its own cases
# run at 1, 1000 and 3000 ms), so there is no boundary value at THIS tick period to land on by
# accident - 16 carries two ticks of headroom purely to keep this row visibly off the floor
# itself, the same "comfortably past, not exactly on" convention every miss_grace in this
# package follows. Nominal grace is [B5_MISS_GRACE + 1] * TICK_INTERVAL_MS = 3400 ms;
# B5_RESPAWN_DELAY_SEC below is sized against this value.
B5_MISS_GRACE = 16
# How long B5's own node is held down every cycle, overriding the launch-wide RESPAWN_DELAY_SEC
# floor (1.5 s - too short to ever satisfy the 3000 ms floor above, let alone B5_MISS_GRACE's own
# 3400 ms). respawn_delay is an ENFORCED floor under how soon launch may even attempt to restart
# a respawn=True process (see _lifecycle_node_action's own docstring), so every cycle's actual
# outage is guaranteed to be at least this long - a correct detector can therefore always report
# it, which is the property this row needs to be satisfiable by a right implementation rather
# than only by a wrong one.
#
# The margin over B5_MISS_GRACE's own nominal grace (3400 ms) has to cover two things a tick
# count alone does not see: the entity cache a tick reads is rebuilt on a debounced graph event
# "always serviced within refresh_debounce_ms + one 100 ms tick" (gateway_node.cpp) - 1100 ms
# worst case before either end of the outage is even visible to node_death's own snapshot - and
# the plugin's tick loop wakes on a condition-variable timed wait, not a hardware clock
# (GraphWatchdogPlugin::wait_for_next_tick), with no enforced upper bound on how far CI
# contention can slow it below its configured 200 ms period. Margin under the 1100 ms debounce
# ceiling alone can be erased by a single unlucky refresh regardless of tick-loop speed at all,
# and NodeLivenessTracker::update() resets a key's miss count to zero the instant it is seen
# present again, with no way to recover a miss already lost that way - a cycle that loses this
# race does not report late, it never reports at all. 12.5 s clears 3 * 3400 + 1100 = 11300 ms
# (a tick loop running at a third of its configured rate, plus the full debounce ceiling on top)
# by 1200 ms - worst-case observed window 12500 - 1100 = 11400 ms, 3.35x the nominal grace.
B5_RESPAWN_DELAY_SEC = 12.5

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
PRESENCE_TIMEOUT_SEC = 30.0 * TIME_SCALE
DEPARTURE_TIMEOUT_SEC = 30.0 * TIME_SCALE
RAISE_TIMEOUT_SEC = 60.0 * TIME_SCALE
CLEAR_TIMEOUT_SEC = 60.0 * TIME_SCALE
# How long an absent or a persisting fault is watched for - measured from the arming gate, not
# process start, so bringup cannot eat it. Matches test_node_death_e2e.test.py's identical
# constant.
SUSTAINED_WINDOW_SEC = 20.0

# launch will not even ATTEMPT to restart a respawn=True process before this elapses - an
# enforced floor under every kill-then-return gap this file measures.
RESPAWN_DELAY_SEC = 1.5

# ---- "b5_restart_loop_still_caught" scenario's own target --------------------------------------
# The claim under test is that the presence code catches EVERY cycle of a restart loop, not
# merely the first: three consecutive kill/raise/clear cycles establish that - this is a loop,
# and each turn of it is caught. A fourth or fifth cycle would repeat an already-proven claim at
# the full cost of B5_RESPAWN_DELAY_SEC apiece, buying no additional discriminating power against
# the failure this row exists to catch (a detector that only reports the first departure, or
# stops reporting after one). If a future change ever needs more cycles to expose something this
# one does not, raising this single constant is the whole edit.
B5_CYCLES = 3

# ---- "c4_config_endpoint_e2e" scenario's own fixtures -------------------------------------------
C4_TARGET_NODE = 'calibration'
C4_TARGET_EXECUTABLE = 'demo_calibration_service'
C4_TARGET_NAMESPACE = '/powertrain/engine'
# Comfortably under the documented ceiling (3600 ticks - node_death's own kMaxNodeDeathGraceTicks
# applies to miss_grace and prune_grace alike), chosen only to sit unambiguously outside
# C4_EARLY_WINDOW_SEC below - 500 ticks at TICK_INTERVAL_MS is 100s, four times that window.
C4_MISS_GRACE_LARGE = 500
# Long enough that a near-floor config would already have raised several times over by the time
# this window ends - the near-floor graces in this package run 16 to 20 ticks, 3.4s to 4.2s
# nominal at TICK_INTERVAL_MS, see B5_MISS_GRACE and D2_MISS_GRACE - so staying silent through it
# is evidence the large value is actually GOVERNING behaviour, not merely accepted and ignored;
# short enough to sit comfortably inside C4_MISS_GRACE_LARGE's own ~100s nominal grace.
C4_EARLY_WINDOW_SEC = 25.0
# Measured from the END of C4_EARLY_WINDOW_SEC, not from the kill: comfortably covers the
# remaining ~75s to C4_MISS_GRACE_LARGE's own nominal grace-crossing point plus reporting
# latency.
C4_LATE_RAISE_TIMEOUT_SEC = 100.0 * TIME_SCALE

# ---- "d2_ungated_clear" scenario's own fixtures -------------------------------------------------
D2_TARGET_NODE = 'calibration'
D2_TARGET_EXECUTABLE = 'demo_calibration_service'
D2_TARGET_NAMESPACE = '/powertrain/engine'
D2_MISS_GRACE = 20
# Deliberately large so the RESTARTED gateway's own pre-arm window is wide enough to sample
# several times over - the whole point of this scenario is watching what happens DURING that
# window, not merely before and after it. 150 ticks at TICK_INTERVAL_MS is 30s nominal warmup:
# generous headroom over the restart itself, which is not instantaneous - create_gateway_node's
# own respawn_delay (1.0s default) is an ENFORCED floor before launch even attempts to start
# the replacement process, on top of that process's own ROS init and HTTP bind. Measured live at
# 25 ticks (5s nominal): GET /faults was still completely unreachable (connection refused, not
# even a 503) at the very first poll after the old port was confirmed down - the replacement
# gateway had not bound its port yet. 150 leaves room for that plus real bringup variance.
D2_WARMUP_CYCLES = 150
# How long the post-restart, pre-arm window is watched for a premature PASSED, once the fault
# surface is confirmed reachable (see test_02's own wait_until_faults_endpoint_live call before
# this window opens). Comfortably inside D2_WARMUP_CYCLES * TICK_INTERVAL_MS (~30s nominal), so
# a poll landing here is provably still INSIDE the ungated window rather than after it.
D2_UNGATED_WATCH_SEC = 3.5


def _lifecycle_node_action(
        name, *, respawn=False, respawn_delay=RESPAWN_DELAY_SEC, auto_activate=None):
    """One managed_lifecycle instance under `name`, with a PID handle the test can signal.

    Uses DEMO_NODE_REGISTRY's own (executable, ros_name, namespace) triple for `name` so this
    stays in lockstep with demo_nodes.launch.py, built by hand only because every scenario here
    signals the process directly and create_demo_nodes() hands back no PID.

    `auto_activate` is a ROS PARAMETER, not part of the registry triple, so it has to be set
    here explicitly rather than inferred from it. Defaults to whether `name` is ACTIVE_NODE -
    the historical convention every scenario but B3/B5 relies on - but a caller launching
    TARGET_NODE (managed_lifecycle) for a scenario that needs it to actually ARM for
    node_death passes `auto_activate=True` explicitly: node_death only ever tracks a node whose
    departure the gate has said it OWNS at least once (presence_ownership() needs a
    MANAGED node's lifecycle state positively known to read "active"), so a require_active node
    that never activates is invisible to it no matter what happens to it afterwards. See
    TestBoundaryMaturedThenGone and TestBoundaryRestartLoopStillCaught's own class docstrings.
    """
    executable, ros_name, namespace = DEMO_NODE_REGISTRY[name]
    if auto_activate is None:
        auto_activate = name == ACTIVE_NODE
    node_kwargs = {
        'package': 'ros2_medkit_integration_tests',
        'executable': executable,
        'name': ros_name,
        'namespace': namespace,
        'output': 'screen',
        'additional_env': get_coverage_env('ros2_medkit_integration_tests'),
        'sigterm_timeout': '30',
        'sigkill_timeout': '15',
        'respawn': respawn,
        'respawn_delay': respawn_delay,
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
    gateway_respawn = False

    if SCENARIO == 'b1_inactive_present':
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = GRACE
        demo_nodes = [TARGET_NODE]  # never killed - the whole point of this row
    elif SCENARIO == 'b2_inactive_below_grace_then_gone':
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = B2_GRACE
    elif SCENARIO == 'b3_matured_then_gone':
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = GRACE
    elif SCENARIO == 'b4_healthy_then_gone':
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [ACTIVE_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = GRACE
    elif SCENARIO == 'b5_restart_loop_still_caught':
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = B5_GRACE
        detector_params[f'{_NODE_DEATH_PREFIX}.miss_grace'] = B5_MISS_GRACE
    elif SCENARIO == 'b6_never_armed_below_grace_then_gone':
        detector_params[f'{_LIFECYCLE_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_LIFECYCLE_PREFIX}.grace'] = B6_GRACE
    elif SCENARIO == 'c4_config_endpoint_e2e':
        detector_params[f'{_NODE_DEATH_PREFIX}.miss_grace'] = C4_MISS_GRACE_LARGE
    elif SCENARIO == 'd2_ungated_clear':
        detector_params[f'{_NODE_DEATH_PREFIX}.miss_grace'] = D2_MISS_GRACE
        detector_params['plugins.graph_watchdog.warmup_cycles'] = D2_WARMUP_CYCLES
        gateway_respawn = True
    else:
        raise RuntimeError(f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} has no launch configuration')

    launch_description, context = create_watchdog_test_launch(
        detector_params=detector_params,
        demo_nodes=demo_nodes,
        port=PORT,
        gateway_respawn=gateway_respawn,
    )

    if SCENARIO == 'b2_inactive_below_grace_then_gone':
        # auto_activate=True: this row's SECOND claim (GRAPH_NODE_DISAPPEARED) needs
        # node_death to have tracked the node at all, which requires it to have been armed -
        # see TestBoundaryInactiveBelowGraceThenGone's own class docstring and the module
        # docstring's "Which arming gate" section.
        target = _lifecycle_node_action(TARGET_NODE, respawn=False, auto_activate=True)
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'b3_matured_then_gone':
        # auto_activate=True: this row's SECOND claim (GRAPH_NODE_DISAPPEARED) needs
        # node_death to have tracked the node at all, which requires it to have been armed -
        # see TestBoundaryMaturedThenGone's own class docstring and
        # _lifecycle_node_action's.
        target = _lifecycle_node_action(TARGET_NODE, respawn=False, auto_activate=True)
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'b4_healthy_then_gone':
        target = _lifecycle_node_action(ACTIVE_NODE, respawn=False)
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'b6_never_armed_below_grace_then_gone':
        # auto_activate deliberately omitted (defaults False for TARGET_NODE): this row's
        # whole claim is that the node NEVER arms - see the module docstring's "Which arming
        # gate" section and TestBoundaryNeverArmedBelowGraceThenGone's own class docstring.
        target = _lifecycle_node_action(TARGET_NODE, respawn=False)
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'b5_restart_loop_still_caught':
        # auto_activate=True: every respawned instance needs to reach "active" on its own
        # for node_death to ever track it - see TestBoundaryRestartLoopStillCaught's own
        # class docstring and _lifecycle_node_action's.
        target = _lifecycle_node_action(
            TARGET_NODE, respawn=True, respawn_delay=B5_RESPAWN_DELAY_SEC, auto_activate=True)
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'd2_ungated_clear':
        target = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=D2_TARGET_EXECUTABLE,
            name=D2_TARGET_NODE,
            namespace=D2_TARGET_NAMESPACE,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
            # No respawn: the node must STAY gone across the gateway restart, or there would be
            # nothing "stored" left for the warmup window to wrongly move toward healing.
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'c4_config_endpoint_e2e':
        target = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=C4_TARGET_EXECUTABLE,
            name=C4_TARGET_NODE,
            namespace=C4_TARGET_NAMESPACE,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

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


def _watchdog_global_state(port, timeout=5.0):
    """One immediate read of GET /x-medkit-watchdog's own global_state field.

    Not a polling helper: D2's pre-arm proof needs to know the state AT ONE INSTANT
    (immediately before opening the ungated window, and again immediately after it closes),
    not wait until some condition becomes true - a poll loop would blur exactly the boundary
    this is meant to pin. Mirrors ReliabilityGate::status_json()'s own field (see
    wait_until_watchdog_armed's docstring in harness.py: "armed" or "warming_up").

    Returns the string, or ``None`` if the endpoint did not answer 200.
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    try:
        response = requests.get(f'{base}/x-medkit-watchdog', timeout=timeout)
    except requests.exceptions.RequestException:
        return None
    if response.status_code != 200:
        return None
    return response.json().get('x-medkit-watchdog', {}).get('global_state')


def _wait_until_port_is_down(port, timeout=60.0, interval=0.2):
    """Wait until the gateway's HTTP port stops answering. ``True`` once it does."""
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            requests.get(f'{base}/health', timeout=2)
        except requests.exceptions.RequestException:
            return True
        time.sleep(interval)
    return False


def _fault_record(port, code, timeout=30.0, interval=0.5):
    """Poll ``GET /faults?status=all`` until `code` appears, whatever its status.

    ``poll_faults`` uses the default (pending+confirmed) filter, so a HEALED or CLEARED fault
    disappears from it. D2 needs the record itself, including ``last_passed``, which survives
    both. Returns the matching item dict, or ``None`` on timeout.
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


def _fault_present_now(port, code, timeout=5.0):
    """One immediate GET /faults check for whether `code` is in the active-fault list.

    Not a polling helper: B2's below-grace precondition has to be read from an OBSERVABLE AT
    ONE INSTANT, immediately before the kill that follows it - a poll loop's own sampling
    interval would widen exactly the race this exists to shrink. Uses the same default
    (pending+confirmed) filter as poll_faults, so "False" here means the same thing a
    poll_faults timeout does: not (yet) raised.

    Returns ``True``/``False``, or ``None`` if the endpoint did not answer 200 - the caller must
    tell that apart from ``False``, the same channel-alive discipline every window assertion in
    this package's harness already applies.
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    try:
        response = requests.get(f'{base}/faults', timeout=timeout)
    except requests.exceptions.RequestException:
        return None
    if response.status_code != 200:
        return None
    codes = {item.get('fault_code') for item in response.json().get('items', [])}
    return code in codes


def _assert_never_passed_throughout(test_case, port, code, duration, interval=0.2):
    """Fail unless `code`'s stored record has ``last_passed is None`` on EVERY poll.

    D2's own claim ("no clear reaches the fault manager before the detector has measured") is
    about a FIELD staying unset across a window, not about the fault's presence/absence -
    neither ``assert_fault_absent_throughout`` nor ``assert_fault_persists_throughout`` reads
    ``last_passed``, so this file carries its own, narrow local helper rather than stretching
    either one to fit. Same channel-alive discipline as harness.py's own window assertions: a
    request error or a non-200 fails the assertion naming which poll and why, rather than being
    swallowed the way a bare ``poll_cleared``/``assertFalse`` pair would be.

    Parameters
    ----------
    test_case : unittest.TestCase
    port : int
        Gateway HTTP port.
    code : str
        The ``fault_code`` whose record must show no PASSED report.
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
            response = requests.get(f'{base}/faults', params={'status': 'all'}, timeout=5)
        except requests.exceptions.RequestException as exc:
            test_case.fail(
                f'/faults became unreachable {polls} poll(s) into a {duration}s ungated-window '
                f'check (could not ask, which is not the same as "asked, and {code} was never '
                f'reported PASSED"): {exc}')
            return
        if response.status_code != 200:
            test_case.fail(
                f'/faults answered HTTP {response.status_code} {polls} poll(s) into a '
                f'{duration}s ungated-window check - the channel died mid-window, which this '
                'assertion must not read as "never reported PASSED"')
            return
        record = next(
            (item for item in response.json().get('items', []) if item.get('fault_code') == code),
            None)
        if record is None:
            test_case.fail(
                f'{code} was missing from the store {polls} poll(s) into a {duration}s '
                'ungated-window check that was supposed to keep watching its record')
            return
        test_case.assertIsNone(
            record.get('last_passed'),
            f'{code} was reported PASSED (last_passed={record.get("last_passed")!r}) '
            f'{polls} poll(s) into a {duration}s window that starts before the restarted '
            "detector's own gate has armed anything - an ungated clear reached the fault "
            'manager before anything was actually measured',
        )
        polls += 1
        time.sleep(interval)
    test_case.assertGreater(
        polls, 0,
        f'the {duration}s ungated-window check never actually polled /faults - duration must '
        f'be >= interval ({interval}s)')


def _get_lifecycle_label(client_node, service_name, timeout=10.0):
    """One ``GetState`` call against `service_name`. Returns the state label, or ``None``.

    Creates its own client rather than taking one, matching
    test_node_death_e2e.test.py's identical helper.
    """
    client = client_node.create_client(GetState, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=timeout)
        result = future.result()
        return None if result is None else result.current_state.label
    finally:
        client_node.destroy_client(client)


def _poll_lifecycle_label(client_node, service_name, expected_label, timeout=30.0, interval=0.5):
    """Poll ``GetState`` until `service_name` answers `expected_label`. ``True`` once it does.

    One client for the whole poll, matching test_node_death_e2e.test.py's identical helper.
    """
    client = client_node.create_client(GetState, service_name)
    try:
        deadline = time.monotonic() + timeout
        last_seen = f'{service_name} was never answered at all'
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=interval):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(client_node, future, timeout_sec=interval)
                result = future.result()
                if result is not None:
                    last_seen = result.current_state.label
                    if last_seen == expected_label:
                        return True
            time.sleep(interval)
        print(f'_poll_lifecycle_label({service_name!r}, expected={expected_label!r}) timed '
              f'out after {timeout}s; last seen: {last_seen!r}')
        return False
    finally:
        client_node.destroy_client(client)


def _call_change_state_once(client_node, service_name, transition_id, timeout=30.0):
    """One ``ChangeState`` call against `service_name`. ``True`` on ``result.success``."""
    client = client_node.create_client(ChangeState, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        return False
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=timeout)
    result = future.result()
    return result is not None and result.success


# ---------------------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------------------

class TestBoundaryInactivePresent(unittest.TestCase):
    """B1: a required node inactive past grace, still present: INACTIVE only.

    GENUINELY GREEN, not a placeholder: lifecycle_expectation already ships, so the raise below
    exercises real, already-merged code (LifecycleExpectationTracker's violation-streak clock
    crossing `grace`). The GRAPH_NODE_DISAPPEARED absence half cannot, by itself, distinguish a
    correct node_death from no detector at all - but staying silent is also the permanently
    correct behaviour here, since this node never leaves the graph.
    """

    def test_inactive_past_grace_raises_no_disappeared(self):
        # No target_node fixture: TARGET_NODE is launched via demo_nodes=[...] (create_demo_nodes
        # gives no PID handle back, and this row never needs one - it kills nothing). Only the
        # scenarios below that hand-build the node populate a 'target_node' context entry.
        # Global gate: see the module docstring's "Which arming gate" section.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC),
            'graph_watchdog never reported an armed global state')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')
        self.assertTrue(
            _poll_apps_present(PORT, TARGET_NODE, timeout=PRESENCE_TIMEOUT_SEC),
            f'{TARGET_NODE} never appeared on GET /apps - there is no present node here for '
            'this row to measure',
        )

        fault = poll_faults(PORT, FAULT_CODE_INACTIVE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(f'{FAULT_CODE_INACTIVE} never raised for {TARGET_NODE} past grace')
        self.assertIn(TARGET_NODE, fault.get('description', ''))

        assert_fault_absent_throughout(self, PORT, FAULT_CODE_DISAPPEARED, SUSTAINED_WINDOW_SEC)

        # The trigger was pinned before the window; confirm it survived to the end too, or most
        # of the window measured a graph this row was not actually about.
        self.assertTrue(
            _poll_apps_present(PORT, TARGET_NODE, timeout=5.0),
            f'{TARGET_NODE} is no longer present at the end of the window - the trigger did '
            'not survive it',
        )


class TestBoundaryInactiveBelowGraceThenGone(unittest.TestCase):
    """B2: a required node inactive for FEWER ticks than grace, then it vanishes.

    Absence may continue a violation that has already matured, but must never mature one that
    has not: the row it exists to pin is a defect where absence used to CONTINUE a violation
    streak that had not yet matured when the node was last observed - necessary only because no
    presence detector existed to catch a restart loop any other way. Once GRAPH_NODE_DISAPPEARED
    has one, that continuation is no longer needed and absence must stop maturing an unmatured
    streak.

    Both of this row's claims are satisfiable together, which took getting the arming gate
    wrong once to learn: the target must reach "active" FIRST, or GRAPH_NODE_DISAPPEARED can
    never report its death, whatever kills it - presence_ownership() needs a MANAGED
    node's lifecycle state positively known to read "active", so a managed_lifecycle instance
    that stays "unconfigured" its whole life is structurally invisible to node_death, exactly as
    TestBoundaryMaturedThenGone's own docstring explains for its target. The launch therefore
    drives TARGET_NODE through "active" first, THEN a real DEACTIVATE transition - the same two
    steps B3 takes - but where B3 leaves the node inactive long enough to CONFIRM (past
    `grace`), B2 kills it almost immediately after: comfortably below `grace`, which is this
    row's own claim and the reason it cannot simply reuse B3's fixture wholesale.

    The below-grace precondition is proven from an OBSERVABLE immediately before the kill, not
    inferred from how little time elapsed since the DEACTIVATE transition: status_json()
    exposes no per-node violation-streak count (see lifecycle_expectation_detector.cpp's own
    status_json, which reports only tracking_saturated/tracked_nodes/tracked_node_cap), so the
    fault surface itself is the only external evidence of whether the streak has crossed
    `grace`, and with confirmation_threshold=-2 the detector's first FAILED report IS the
    confirmation - "absent from /faults" and "not yet matured" are the same fact, one HTTP
    round trip apart. A failure at that check names a timing problem in this test's OWN setup
    (B2_GRACE too tight against how long the DEACTIVATE round trip took this run), never the
    absence-must-not-mature claim the kill exists to test - which is the point of reading an
    observable immediately before acting instead of trusting a margin.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('node_death_boundary_b2_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_below_grace_departure_never_matures_inactive(self, target_node):
        get_state_service = f'/{TARGET_NODE}/get_state'
        change_state_service = f'/{TARGET_NODE}/change_state'

        # app_id form: this node reaches "active" on its own now (auto_activate=True), so the
        # per-entity armed precondition (LifecycleWatcher::node_ok()) genuinely becomes true -
        # see the module docstring's "Which arming gate" section and this class's own
        # docstring.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        self.assertTrue(
            _poll_lifecycle_label(
                type(self)._client_node, get_state_service, 'active',
                timeout=PRESENCE_TIMEOUT_SEC),
            f'{TARGET_NODE} never reached "active" on its own - the trigger this row needs (a '
            'node armed for node_death, then driven non-active BELOW grace) was never set up',
        )
        self.assertTrue(
            _call_change_state_once(
                type(self)._client_node, change_state_service,
                Transition.TRANSITION_DEACTIVATE, timeout=PRESENCE_TIMEOUT_SEC),
            'the real DEACTIVATE transition was rejected or never answered',
        )
        self.assertTrue(
            _poll_lifecycle_label(
                type(self)._client_node, get_state_service, 'inactive',
                timeout=PRESENCE_TIMEOUT_SEC),
            f'{TARGET_NODE} never reached "inactive" after DEACTIVATE',
        )

        # tracked_nodes becoming 1 is the tracker's own proof that it matched TARGET_NODE at
        # least once - the earliest moment a kill is guaranteed not to land before the detector
        # was even permitted to look. B2_GRACE is generous enough that the handful of extra
        # ticks the DEACTIVATE round trip and this poll's own interval can cost before the kill
        # below still leaves the node comfortably below grace.
        self.assertTrue(
            poll_detector_status(
                PORT, DETECTOR_ID_LIFECYCLE, 'tracked_nodes', 1, timeout=ARM_TIMEOUT_SEC),
            f'lifecycle_expectation never reported tracking {TARGET_NODE} - it was never '
            'matched at all, so killing it below would prove nothing about absence maturing '
            'an unmatured streak',
        )

        # The row's own precondition, read from an observable immediately before the kill - see
        # the class docstring for why this is the tightest proof available without new
        # instrumentation. `is` rather than a plain falsy check: None (channel unreachable) must
        # not be read as "confirmed absent".
        below_grace = _fault_present_now(PORT, FAULT_CODE_INACTIVE)
        self.assertIs(
            below_grace, False,
            f'{FAULT_CODE_INACTIVE} could not be proven absent immediately before the kill '
            f'below (checked value: {below_grace!r}) - either the fault surface is unreachable, '
            f'or B2_GRACE ({B2_GRACE} ticks) already matured before the DEACTIVATE round trip '
            "finished this run, so the kill below would test B3's claim (already matured), not "
            "this row's below-grace claim",
        )

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        # The row's first claim: GRAPH_NODE_INACTIVE must never appear, however long the node
        # stays gone - a below-grace streak is held by absence, never matured by it.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE_INACTIVE, SUSTAINED_WINDOW_SEC)

        # The row's second claim, reachable because TARGET_NODE was armed before it died - see
        # the class docstring.
        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(f'{FAULT_CODE_DISAPPEARED} never raised for the departed {TARGET_NODE}')
        self.assertIn(TARGET_NODE, fault.get('description', ''))


class TestBoundaryMaturedThenGone(unittest.TestCase):
    """B3: a required node reaches active, is driven non-active past grace, then vanishes.

    Two independent claims, BOTH satisfiable now. The first - "content follows the clocks, not
    the snapshot" keeps a matured GRAPH_NODE_INACTIVE violation in place through the departure,
    still naming the node - is already-shipped lifecycle_expectation behaviour, proven here
    with assert_fault_describes_only against the real stack rather than a code-only persistence
    check (a detector that dropped TARGET_NODE from the description while keeping some OTHER
    node's violation raised under the same code would still pass a bare presence check), the
    same claim test_lifecycle_expectation_e2e.test.py's own "departure_keeps" scenario proves
    for the UNREADABLE code.

    The second - GRAPH_NODE_DISAPPEARED also joining once the node is gone, still naming it -
    needs node_death to have TRACKED the node at all, which only ever happens for a node the gate
    has admitted for presence ownership at least once: presence_ownership() needs a
    MANAGED node's lifecycle state positively known to read "active". A managed_lifecycle
    instance that stays "unconfigured" its whole life is never admitted and so is structurally
    invisible to node_death whatever happens to it afterwards - the second claim would be
    unreachable by ANY correct implementation against that fixture, not merely an unimplemented
    one. This is why the launch drives
    TARGET_NODE through "active" FIRST (arming it - the same mechanism B4's target uses), THEN
    a real DEACTIVATE transition (maturing GRAPH_NODE_INACTIVE the same way the original
    "stays unconfigured" fixture did), THEN kills it: the node this row's second claim needs
    has to have been alive and armed at some point before it can ever be reported disappeared.
    A future edit that reverts TARGET_NODE to launching without auto_activate would silently
    make the second half of this row unreachable again.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('node_death_boundary_b3_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_matured_inactive_survives_departure_and_disappeared_joins(self, target_node):
        get_state_service = f'/{TARGET_NODE}/get_state'
        change_state_service = f'/{TARGET_NODE}/change_state'

        # app_id form: this node reaches "active" on its own now (auto_activate=True), so the
        # per-entity armed precondition (LifecycleWatcher::node_ok()) genuinely becomes true -
        # see the module docstring's "Which arming gate" section.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        self.assertTrue(
            _poll_lifecycle_label(
                type(self)._client_node, get_state_service, 'active',
                timeout=PRESENCE_TIMEOUT_SEC),
            f'{TARGET_NODE} never reached "active" on its own - the trigger this row needs (a '
            'node armed for node_death, then driven non-active) was never set up',
        )
        self.assertTrue(
            _call_change_state_once(
                type(self)._client_node, change_state_service,
                Transition.TRANSITION_DEACTIVATE, timeout=PRESENCE_TIMEOUT_SEC),
            'the real DEACTIVATE transition was rejected or never answered',
        )
        self.assertTrue(
            _poll_lifecycle_label(
                type(self)._client_node, get_state_service, 'inactive',
                timeout=PRESENCE_TIMEOUT_SEC),
            f'{TARGET_NODE} never reached "inactive" after DEACTIVATE',
        )

        fault = poll_faults(PORT, FAULT_CODE_INACTIVE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_INACTIVE} never raised for {TARGET_NODE} past grace - there is '
                'nothing ALREADY CONFIRMED for the kill below to test the survival of')
        self.assertIn(TARGET_NODE, fault.get('description', ''))

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        # Already-shipped lifecycle_expectation behaviour: proves the matured violation is not
        # healed by the departure, and that the surviving content still names TARGET_NODE
        # rather than merely keeping the code raised (see the class docstring for why a
        # code-only check is not enough here).
        assert_fault_describes_only(
            self, PORT, FAULT_CODE_INACTIVE, required=[TARGET_NODE], forbidden=[],
            duration=SUSTAINED_WINDOW_SEC)

        # Reached only because the assertion above passed, unlike B2's sibling call. Reachable
        # AT ALL because TARGET_NODE was armed before it died - see the class docstring.
        second = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if second is None:
            self.fail(f'{FAULT_CODE_DISAPPEARED} never raised for the departed {TARGET_NODE}')
        self.assertIn(TARGET_NODE, second.get('description', ''))


class TestBoundaryHealthyThenGone(unittest.TestCase):
    """B4: a required node reaches active, then shuts down: DISAPPEARED only.

    The INACTIVE-absence half is GENUINELY GREEN, already-shipped behaviour
    (`release_uncorroborated`: a node last measured healthy that then departs starts no
    violation) - proven here against the real stack, checked FIRST so its own result is never
    masked by the DISAPPEARED half's own timeout budget. The DISAPPEARED half proves node_death
    raises and names a node that was healthy right up to the moment it departed.
    """

    def test_healthy_departure_raises_only_disappeared(self, target_node):
        # app_id form: unlike B1's target, this node reaches "active" on its own, so the
        # per-entity armed precondition (LifecycleWatcher::node_ok()) genuinely becomes true
        # here - see the module docstring's "Which arming gate" section.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=ACTIVE_NODE),
            f'graph_watchdog never reported {ACTIVE_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, ACTIVE_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{ACTIVE_NODE} never left GET /apps after SIGTERM')

        # Already-shipped lifecycle_expectation behaviour: an active-then-departed node never
        # becomes INACTIVE content.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE_INACTIVE, SUSTAINED_WINDOW_SEC)

        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(f'{FAULT_CODE_DISAPPEARED} never raised for the departed {ACTIVE_NODE}')
        self.assertIn(ACTIVE_NODE, fault.get('description', ''))


class TestBoundaryRestartLoopStillCaught(unittest.TestCase):
    """B5: a required node in a restart loop is still caught, every cycle, by the presence code.

    The most important row in this file: it is what makes it safe for B2 to forbid absence from
    maturing an unmatured streak. Once GRAPH_NODE_DISAPPEARED independently catches a departure
    regardless of how briefly the node was up, lifecycle_expectation no longer has to lean on an
    unmatured streak maturing on absence to keep a restart-looping node from evading every code.
    This row does not touch GRAPH_NODE_INACTIVE at all - that claim belongs to B2 and B3.

    The target launches with auto_activate (the same mechanism B4's target uses) and keeps that
    parameter across every respawn, not merely the first start: node_death only ever tracks a
    node whose departure the gate has said it OWNS at least once (presence_ownership()
    needs a managed node's lifecycle state positively known to read "active"), so a fixture that
    never activates would make GRAPH_NODE_DISAPPEARED structurally unreachable for every cycle
    here, not merely slow to catch - this row carries the evidence that forbidding absence from
    maturing an unmatured
    streak is safe, so it has to run against a node that can actually be reported. Each cycle's
    respawned instance is re-armed (app_id-scoped wait_until_watchdog_armed, not merely
    presence) before the NEXT kill for the identical reason a fresh process needs it the first
    time: a kill landing before a
    just-restarted instance reaches "active" would leave that cycle's death untracked too, and
    the following poll_faults would time out for a precondition reason having nothing to do
    with the claim this row is about. A future edit that reverts TARGET_NODE to launching
    without auto_activate would silently make every cycle of this row unreachable again.

    Every cycle's outage is held down for B5_RESPAWN_DELAY_SEC, safely longer than the
    explicitly-configured B5_MISS_GRACE this scenario launches with (see the module docstring):
    left at launch's own 1.5 s respawn floor with no miss_grace configured, a correct detector's
    own wall-clock floor could make the outage unreportable regardless of how many cycles this
    test waits out, which is a row a right implementation cannot satisfy - not evidence of a
    defect in one. Every raise is also checked by NAME, not merely by code, so a detector
    reporting the same code for some other entity every cycle could not pass in TARGET_NODE's
    place.
    """

    def test_every_restart_cycle_raises_and_clears(self, target_node):
        # app_id form: this node reaches "active" on its own now (auto_activate=True), so the
        # per-entity armed precondition genuinely becomes true - see the module docstring's
        # "Which arming gate" section and this class's own docstring.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        pid = target_node.process_details['pid']
        for cycle in range(1, B5_CYCLES + 1):
            os.kill(pid, signal.SIGTERM)
            self.assertTrue(
                _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
                f'cycle {cycle}: {TARGET_NODE} never left GET /apps after SIGTERM',
            )

            fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
            if fault is None:
                self.fail(f'cycle {cycle}: {FAULT_CODE_DISAPPEARED} never raised')
            self.assertIn(
                TARGET_NODE, fault.get('description', ''),
                f'cycle {cycle}: {FAULT_CODE_DISAPPEARED} raised but did not name {TARGET_NODE}')

            # app_id-scoped again, not just present: the respawned instance is a NEW process
            # node_death has not armed yet, and the next iteration is about to kill it - the
            # same precondition the FIRST kill above needed (see the class docstring).
            self.assertTrue(
                wait_until_watchdog_armed(
                    PORT, timeout=DEPARTURE_TIMEOUT_SEC + B5_RESPAWN_DELAY_SEC,
                    app_id=TARGET_NODE),
                f'cycle {cycle}: {TARGET_NODE} never came back armed after the SIGTERM - '
                'launch did not respawn it, or the respawned instance never reached "active"',
            )
            # A fresh pid every cycle: read it back rather than assume launch's respawn keeps
            # the value this test already has.
            pid = target_node.process_details['pid']

            self.assertTrue(
                poll_cleared(PORT, FAULT_CODE_DISAPPEARED, timeout=CLEAR_TIMEOUT_SEC),
                f'cycle {cycle}: {FAULT_CODE_DISAPPEARED} never cleared once {TARGET_NODE} '
                'came back - this row is about EVERY cycle being caught AND released, not a '
                'single continuous outage',
            )


class TestBoundaryNeverArmedBelowGraceThenGone(unittest.TestCase):
    """B6: a required node that never arms, killed below grace: INACTIVE only, and it must raise.

    The row B2 cannot cover. B2's target reaches "active" first, so node_death can track it and
    GRAPH_NODE_DISAPPEARED is the one to report a below-grace departure - which is exactly why
    absence must NOT mature GRAPH_NODE_INACTIVE there. This target never reaches "active" at
    all: LifecycleWatcher::node_ok() is false for its whole life, the gate never arms it
    (per-entity `armed` requires "active" for a managed node), and node_death only ever tracks a
    key the gate has armed at least once - so GRAPH_NODE_DISAPPEARED is structurally unable to
    report this departure, whatever kills it. lifecycle_expectation is the only detector that
    ever could, and closing that silence means absence has to be allowed to mature a below-grace
    streak here, the opposite of B2's claim for the opposite reason.

    Both halves are checked: GRAPH_NODE_INACTIVE must raise and name the node (the silence
    closing), and GRAPH_NODE_DISAPPEARED must stay absent throughout (the structural reason the
    silence existed at all - if this half ever raised, node_death would have tracked a node the
    gate never armed, which would be its own defect, not evidence this row's fix works).
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('node_death_boundary_b6_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_never_armed_departure_matures_inactive_disappeared_silent(self, target_node):
        get_state_service = f'/{TARGET_NODE}/get_state'

        # Global gate: see the module docstring's "Which arming gate" section - this target
        # never reaches "active", so the app_id-scoped form would wait for something that
        # never becomes true.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC),
            'graph_watchdog never reported an armed global state')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')
        self.assertTrue(
            _poll_apps_present(PORT, TARGET_NODE, timeout=PRESENCE_TIMEOUT_SEC),
            f'{TARGET_NODE} never appeared on GET /apps - there is no present node here for '
            'this row to measure',
        )
        self.assertTrue(
            _poll_lifecycle_label(
                type(self)._client_node, get_state_service, 'unconfigured',
                timeout=PRESENCE_TIMEOUT_SEC),
            f'{TARGET_NODE} did not sit at "unconfigured" - the trigger this row needs (a '
            'required node that never arms) was never set up',
        )

        # tracked_nodes becoming 1 is the tracker's own proof that it matched TARGET_NODE at
        # least once - the earliest moment a kill is guaranteed not to land before the
        # detector was even permitted to look. B6_GRACE is generous enough that the handful
        # of extra ticks this poll's own interval can cost before the kill below still leaves
        # the node comfortably below grace.
        self.assertTrue(
            poll_detector_status(
                PORT, DETECTOR_ID_LIFECYCLE, 'tracked_nodes', 1, timeout=ARM_TIMEOUT_SEC),
            f'lifecycle_expectation never reported tracking {TARGET_NODE} - it was never '
            'matched at all, so killing it below would prove nothing about absence maturing '
            'a violation the presence detector could never have reported',
        )

        # The row's own precondition, read from an observable immediately before the kill -
        # same instrument as B2's identical check, for the identical reason: `is` rather than
        # a plain falsy check, since None (channel unreachable) must not be read as
        # "confirmed absent".
        below_grace = _fault_present_now(PORT, FAULT_CODE_INACTIVE)
        self.assertIs(
            below_grace, False,
            f'{FAULT_CODE_INACTIVE} could not be proven absent immediately before the kill '
            f'below (checked value: {below_grace!r}) - either the fault surface is '
            f'unreachable, or B6_GRACE ({B6_GRACE} ticks) already matured while the node was '
            'still present, so the kill below would prove nothing about absence maturing the '
            'violation',
        )

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        # The row this file exists to add: with node_death structurally unable to track a
        # node that was never armed, lifecycle_expectation's own absence handling has to be
        # the one that matures the below-grace streak, or the departure is reported by
        # nothing at all.
        fault = poll_faults(PORT, FAULT_CODE_INACTIVE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_INACTIVE} never raised for the departed {TARGET_NODE} - a node '
                'the presence detector could never have reported went unreported by every '
                'detector')
        self.assertIn(TARGET_NODE, fault.get('description', ''))

        # The structural half: GRAPH_NODE_DISAPPEARED must never raise for this node. It was
        # never armed, so node_death could never have tracked it in the first place - a raise
        # here would mean node_death tracked a node the gate never armed, not that this row's
        # fix works.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE_DISAPPEARED, SUSTAINED_WINDOW_SEC)


class TestBoundaryConfigEndpointE2E(unittest.TestCase):
    """C4: `detectors.node_death.miss_grace` visibly changes when a death is reported.

    One gateway, one armed node, killed once, with a LARGE miss_grace configured
    (C4_MISS_GRACE_LARGE, comfortably under the documented 3600-tick ceiling). Proves the
    knob governs an OBSERVABLE, not merely that the plugin accepts it at startup, by checking the
    SAME gateway at two points on ONE timeline rather than comparing two gateways: first, a
    window (C4_EARLY_WINDOW_SEC) long enough that a near-floor config (this suite's own ~4s
    convention - B5_MISS_GRACE, D2_MISS_GRACE) would already have raised, in which this
    large-grace gateway must stay silent - needle-scoped (assert_fault_never_names), so a
    detector raising for some unrelated entity cannot decide the row; second, once the
    configured grace has had time to elapse, the fault must still arrive and name the node - the
    large value delays the report, it does not swallow it.
    """

    def test_large_miss_grace_delays_then_still_reports(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=C4_TARGET_NODE),
            f'graph_watchdog never reported {C4_TARGET_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - nothing below would prove anything')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, C4_TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{C4_TARGET_NODE} never left GET /apps after SIGTERM')

        # Needle-scoped: a raise for some OTHER entity must not decide this row (see
        # assert_fault_never_names's own docstring). Long enough that a near-floor config would
        # already have raised; short enough to sit well inside C4_MISS_GRACE_LARGE's own grace.
        assert_fault_never_names(
            self, PORT, FAULT_CODE_DISAPPEARED, forbidden=[C4_TARGET_NODE],
            duration=C4_EARLY_WINDOW_SEC)

        # Past the configured grace, the same knob that delayed the raise must not have
        # suppressed it outright.
        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=C4_LATE_RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_DISAPPEARED} never raised for {C4_TARGET_NODE} even past the '
                'configured C4_MISS_GRACE_LARGE grace')
        self.assertIn(C4_TARGET_NODE, fault.get('description', ''))


class TestBoundaryUngatedClear(unittest.TestCase):
    """D2: an ungated clear must not push a stored fault toward healing before anything measured.

    test_01 confirms the death and its stored record. test_02 then restarts the GATEWAY with a
    wide warmup_cycles, and watches the pre-arm window that follows for a premature PASSED on
    the stored record - the restarted detector must not report anything about a node it has
    never actually measured in this process's lifetime.
    """

    def test_01_the_fault_raises_before_the_restart(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=D2_TARGET_NODE),
            f'graph_watchdog never reported {D2_TARGET_NODE} armed')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, D2_TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{D2_TARGET_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE_DISAPPEARED, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(
                f'{FAULT_CODE_DISAPPEARED} never raised for the departed {D2_TARGET_NODE} - '
                'there is nothing STORED for the restart below to wrongly move toward healing')
        self.assertIn(D2_TARGET_NODE, fault.get('description', ''))

        record = _fault_record(PORT, FAULT_CODE_DISAPPEARED, timeout=DEPARTURE_TIMEOUT_SEC)
        if record is None:
            self.fail(f'{FAULT_CODE_DISAPPEARED} vanished from the store entirely')
        self.assertIsNone(
            record.get('last_passed'),
            f'{FAULT_CODE_DISAPPEARED} was already reported PASSED before the restart '
            f'(last_passed={record.get("last_passed")!r}) - there is nothing left here for '
            'the restart below to wrongly preserve or wrongly heal',
        )

    def test_02_the_pre_arm_window_never_reports_passed(self, gateway_node, target_node):
        del target_node  # stays gone by design; not signalled again in this method
        old_pid = gateway_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _wait_until_port_is_down(PORT, timeout=60.0),
            f'the gateway (pid {old_pid}) kept answering after SIGTERM - nothing restarted')

        # Gate on the fault surface being reachable AT ALL before opening the ungated-window
        # clock. A restarted gateway's HTTP server - let alone its round trip to the
        # fault_manager, a separate process this restart never touched - is not up the instant
        # the OLD port stops answering: create_gateway_node's own respawn_delay is an ENFORCED
        # floor before launch even starts the replacement, before that process's own ROS init
        # and HTTP bind. A poll issued before either is up cannot ask this row's own question
        # ("was PASSED reported") at all - it can only observe a channel that does not exist
        # yet, which _assert_never_passed_throughout correctly reports as unreachable rather
        # than silently reading as "never reported PASSED". Confirmed live: without this gate,
        # the very first poll after the old port went down found GET /faults refusing the
        # connection outright.
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 after the restart - the ungated-window check below '
            'would have no live channel to ask anything on',
        )

        # The window has to be PROVEN pre-arm, not merely assumed from D2_WARMUP_CYCLES's own
        # nominal budget: if restart-plus-HTTP-recovery above ate most or all of that nominal
        # warmup, the watchdog could already be armed by the time the window below opens, and
        # any PASSED it then observes would be legitimate post-arm behaviour, not the
        # ungated-clear bug this row exists to catch. One immediate read, not a poll - the point
        # is the state AT THIS INSTANT, immediately before the window starts.
        pre_state = _watchdog_global_state(PORT)
        self.assertNotEqual(
            pre_state, 'armed',
            f'the gateway was already armed (global_state={pre_state!r}) before the ungated '
            'window below could even open - restart-plus-recovery consumed the whole nominal '
            f'warmup (D2_WARMUP_CYCLES={D2_WARMUP_CYCLES}), so any PASSED observed below would '
            'be legitimate post-arm behaviour, not the ungated-clear bug this row exists to '
            'catch; widen D2_WARMUP_CYCLES rather than trusting this window without proof',
        )

        # The claim under test: for D2_UNGATED_WATCH_SEC once the surface is reachable, the
        # stored fault's last_passed must stay None - comfortably before D2_WARMUP_CYCLES *
        # TICK_INTERVAL_MS (~30s nominal) elapses, so the window is provably still INSIDE the
        # period during which the restarted plugin's own gate has not armed anything.
        _assert_never_passed_throughout(
            self, PORT, FAULT_CODE_DISAPPEARED, D2_UNGATED_WATCH_SEC)

        # The other bracket: prove the window closed still inside pre-arm too, or the samples
        # above are not provably pre-arm from end to end - only from their own start.
        post_state = _watchdog_global_state(PORT)
        self.assertNotEqual(
            post_state, 'armed',
            f'the gateway armed (global_state={post_state!r}) DURING the {D2_UNGATED_WATCH_SEC}s '
            'ungated window above - the samples it collected are no longer provably pre-arm '
            'from end to end, so they cannot be read as evidence about the ungated-clear bug '
            'this row exists to catch',
        )

        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=90.0),
            'the gateway never came back armed after the restart - the warmup window this row '
            'is about never actually ended, so the check above proved nothing about a window '
            'that closes')
        self.assertNotEqual(
            gateway_node.process_details['pid'], old_pid,
            'the gateway process id did not change, so this test never restarted anything')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 after the restart - the persistence check below '
            'would prove nothing',
        )
        self.assertFalse(
            _poll_apps_present(PORT, D2_TARGET_NODE, timeout=5.0),
            f'{D2_TARGET_NODE} is back in GET /apps - this row is about a restart with the '
            'node STILL gone, and it came back instead',
        )

        assert_fault_persists_throughout(self, PORT, FAULT_CODE_DISAPPEARED, SUSTAINED_WINDOW_SEC)

        record = _fault_record(PORT, FAULT_CODE_DISAPPEARED, timeout=DEPARTURE_TIMEOUT_SEC)
        if record is None:
            self.fail(f'{FAULT_CODE_DISAPPEARED} vanished from the store after the restart')
        self.assertIsNone(
            record.get('last_passed'),
            f'the restarted gateway reported {FAULT_CODE_DISAPPEARED} PASSED for a node it can '
            f'never have observed in this process (last_passed={record.get("last_passed")!r})',
        )


# Each CTest target launches this file with one scenario, so only that scenario's case may
# run. Removing the others from the module (rather than skipping them) means each run reports
# exactly one case, and a missing result is a real failure rather than an expected line of
# output - see test_node_death_e2e.test.py's identical rationale.
_SCENARIO_CASES = {
    'b1_inactive_present': 'TestBoundaryInactivePresent',
    'b2_inactive_below_grace_then_gone': 'TestBoundaryInactiveBelowGraceThenGone',
    'b3_matured_then_gone': 'TestBoundaryMaturedThenGone',
    'b4_healthy_then_gone': 'TestBoundaryHealthyThenGone',
    'b5_restart_loop_still_caught': 'TestBoundaryRestartLoopStillCaught',
    'b6_never_armed_below_grace_then_gone': 'TestBoundaryNeverArmedBelowGraceThenGone',
    'c4_config_endpoint_e2e': 'TestBoundaryConfigEndpointE2E',
    'd2_ungated_clear': 'TestBoundaryUngatedClear',
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
