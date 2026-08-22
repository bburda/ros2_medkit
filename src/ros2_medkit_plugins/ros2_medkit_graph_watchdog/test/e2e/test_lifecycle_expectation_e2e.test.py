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
"""Lifecycle-expectation e2e: GRAPH_NODE_INACTIVE raises and heals through the REAL stack.

Unlike test_lifecycle_expectation_integration.cpp (which drives the detector against a
fake ReportFault service, a fixture-owned snapshot, and lifecycle labels injected via
ReliabilityGate::set_lifecycle_state_for_test() - it can only puppet the label, never
prove the real lifecycle machinery feeds it), this launches the REAL gateway process
with the graph_watchdog plugin (.so) loaded and a real fault_manager, and polls the
operator-visible ``GET /api/v1/faults`` surface while driving the demo
``managed_lifecycle`` node (``ros2_medkit_test_utils``' ``DEMO_NODE_REGISTRY``) through
REAL ``lifecycle_msgs/srv/ChangeState`` transitions.

Runs as TWELVE separate CTest targets (see CMakeLists.txt), each launching its OWN
gateway+fault_manager+demo-node stack: the plugin reads its config once at
set_context() time, so different configs need different gateway launches, not test
methods sharing one. WATCHDOG_E2E_SCENARIO selects which launch + assertions run:

- "main": ``require_active: ['managed_lifecycle']`` against the node in its default
  unconfigured state -> the fault raises (globally AND on the entity-scoped surface,
  naming the node), survives a real CONFIGURE (inactive is still not active), survives a
  real gateway RESTART (the withheld-clear guard: the fault_manager is a separate process
  and keeps the fault, while the restarted plugin starts with every detector counter at
  zero), and heals after a real ACTIVATE. This is the only scenario launched with
  ``gateway_respawn``.
- "default_config": NO ``detectors.lifecycle_expectation`` config at all, same demo
  node. The shipped default is an empty ``require_active``, documented as "nothing is
  checked, zero false positives" - this is the only test at any tier that could falsify
  that claim against a live unconfigured lifecycle node.
- "negative_control": ``require_active: ['managed_lifecycle_active']`` with the
  self-activating variant of the same executable. A required node that IS active must
  never raise, over a sustained window, under the same grace and cadence as "main" -
  the entry names the active variant, so the discriminating variable between this
  launch and "main" is the node's actual lifecycle state.
- "unreadable": ``require_active: ['unreadable_lifecycle']`` against
  ``unreadable_lifecycle_node.cpp`` - a fixture that advertises get_state/change_state
  like a managed lifecycle node but never answers get_state until told to. Proves
  GRAPH_NODE_UNREADABLE's own raise and clear through a REAL, sustained GetState
  failure (the class the other scenarios above cannot reach - see
  TestLifecycleExpectationUnreadable's docstring below for the full story) and that
  GRAPH_NODE_INACTIVE stays silent for a node whose state was never read at all.
- "departure_keeps": the same ``unreadable_lifecycle_node.cpp`` fixture, but this
  scenario SIGTERMs it once GRAPH_NODE_UNREADABLE has raised and proves the fault
  SURVIVES the departure - a node whose lifecycle promise was never verified does not
  become verified by leaving, so its evidence is retained and its description switches to
  saying the node is gone. See TestLifecycleExpectationDepartureKeeps's docstring below.
- "not_managed": ``require_active: ['calibration']`` against a PLAIN demo node with no
  lifecycle interface at all (``DEMO_NODE_REGISTRY``'s ``calibration``, a bare service
  server) - proves GRAPH_NODE_NOT_MANAGED's own raise (naming the node, WARN severity,
  mutually exclusive with the other two codes) and the same retention across a departure,
  the sibling proof to "unreadable"/"departure_keeps" for the OTHER cause the unmeasured
  clock is blind to. Needed no purpose-built fixture, unlike UNREADABLE. See
  TestLifecycleExpectationNotManaged's docstring below for the full story.
- "restart_loop": the same ``calibration`` node, but respawning, SIGTERM'd over and over
  on a cadence that never lets it accumulate the 60 consecutive PRESENT ticks the hold
  needs. Proves the point of the whole model on a real stack: a required node in a crash
  loop is REPORTED rather than silent, because the evidence it accumulates while present
  is not discarded every time it goes away. See TestLifecycleExpectationRestartLoop.
- "cap_pressure": ``tracked_node_cap: 1`` against TWO required nodes, both instances of
  ``droppable_lifecycle_node.cpp`` (this package's own fixture - looks managed, answers a
  chosen label, stops advertising its lifecycle services on command). Proves that a refused
  node is visible on ``GET /x-medkit-watchdog``, that a refusal WITHHOLDS
  GRAPH_NODE_INACTIVE's clear, and that an entry for a DEPARTED node is collapsed so a
  present, genuinely broken node is checked instead. See
  TestLifecycleExpectationCapPressure.
- "unsettled_departure": two more ``droppable_lifecycle_node.cpp`` instances, both healthy
  and ACTIVE. One drops its lifecycle services and is killed immediately (a single missed
  sweep before a clean shutdown - nothing may be reported); the other holds the dropped
  state long past the settling budget before being killed (genuinely not managed when it
  left - it must still be reported). See TestLifecycleExpectationUnsettledDeparture.
- "wide_grace": ``grace`` at the value that used to be the accepted maximum
  (``INT_MAX - 1``), against the stuck ``managed_lifecycle`` node. Under such a value the
  detector can neither raise nor heal GRAPH_NODE_INACTIVE for days; the value must be
  refused and the documented default applied. See TestLifecycleExpectationWideGrace.
- "restart_departed": the stuck ``managed_lifecycle`` node is killed while its fault is
  outstanding, then the gateway itself is restarted. RECORDS the boundary of "a departure
  never heals a fault" - it holds within a gateway lifetime, and a restart re-baselines.
  See TestLifecycleExpectationRestartDeparted.

The silence scenarios gate on three facts BEFORE
asserting absence, because absence is
the default outcome of a stack that never came up and every bringup failure mode on this
launch still exits 0 (see the harness docstring): (1) the plugin is live and armed, via
its own GET /x-medkit-watchdog route (harness.wait_until_watchdog_armed); (2) the
gateway reaches the fault_manager in THIS launch, via GET /faults answering 200
(harness.wait_until_faults_endpoint_live) - the watchdog route is served inside the
gateway process and proves nothing about the fault surface the assertion reads, which
answers 503 and polls to None when the fault_manager is gone; (3) the target node's
lifecycle label was actually READ, via the same watchdog route - the tracker treats an
unread label as benign, so without it the silence could just as well mean the label
never arrived. Fact (3) is re-checked AFTER the window as well, since a trigger that
exits mid-window would leave most of the window measuring an empty graph.

The "main" scenario gates on the GLOBAL armed state, not on
``app_id='managed_lifecycle'``: the gate reports a tracked node with a KNOWN non-active
label as ``warming_up`` by design (ReliabilityGate::status_json's suppressed branch ANDs
LifecycleWatcher::node_ok, which is false for exactly the inactive node this detector
exists to catch), so a per-entity armed gate on the target would wait for the fault to
be impossible. The raise itself is gated on the SOURCE entity's arming (the aggregate is
raised under source 'graph_watchdog', see AggregatedFault), for which the global armed
state is the precise precondition. Once the node reaches "active" the per-entity gate
IS reachable, and the heal leg uses it to prove the real lifecycle machinery fed the
watcher before measuring the clear.
"""

import json
import os
import signal
import sys
import threading
import time
import unittest

from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above,
# so this import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    API_BASE_PATH,
    assert_fault_absent_throughout,
    create_watchdog_test_launch,
    poll_cleared,
    poll_detector_status,
    poll_entity_faults,
    poll_faults,
    prove_silence_proof_catches_a_dead_fault_surface,
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

# No default on purpose - a default makes this file FAIL OPEN (see
# test_config_plumbing_e2e.test.py's identical rationale): if the CTest ENVIRONMENT
# property never reaches the process, the launch and the assertions would degrade
# together into one scenario and still report 1/1 passed. A KeyError is loud.
SCENARIO = os.environ['WATCHDOG_E2E_SCENARIO']
PORT = get_test_port()

# Every GIVE-UP bound below is multiplied by MEDKIT_TEST_TIME_SCALE, which the sanitizer jobs
# set to the same factor they apply to each declared CTest timeout. A deadline asserted INSIDE
# a test is invisible to that rewrite, so an instrumented build that takes longer to notice a
# departure blows a poll here and the failure reads as a detector that never reported - the
# exact red this suite exists to produce for a real defect. Unset elsewhere, so the normal jobs
# keep the tight budgets that give these assertions their falsifying edge.
#
# Deliberately NOT scaled, and each for its own reason: the sustained-observation windows
# (SILENT_WINDOW_SEC, UNREADABLE_INACTIVE_SILENCE_SEC, MUTUAL_EXCLUSION_WINDOW_SEC,
# CAP_WITHHOLD_WINDOW_SEC), because watching for silence longer buys no confidence; the enforced
# launch delays (HEALING_RESPAWN_DELAY_SEC, RESTART_LOOP_RESPAWN_DELAY_SEC,
# CAP_REFUSED_NODE_DELAY_SEC), because they are floors the launch imposes rather than bounds
# this file waits out; the deliberate settles (DEPARTURE_SETTLE_SEC); the HTTP per-request
# timeouts inside this file's own helpers, which bound one call and not a wait; and every
# quantity that MEASURES a detector window rather than budgeting for one - the absence-grace
# arithmetic in _blink() above all, since scaling that would stop the blink proving it stayed
# inside the grace at all.
TIME_SCALE = get_time_scale()

# Fast tick cadence + short warmup so the whole story (gateway/fault_manager startup,
# demo-node discovery, lifecycle label seeding, global bringup grace) comfortably fits
# inside the poll timeouts below - same rationale as the other e2e files here. grace is
# a few ticks so the raise is not instantaneous (the detector counts CONSECUTIVE
# not-active ticks) while staying well inside every poll timeout below.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3
GRACE = 3

FAULT_CODE = 'GRAPH_NODE_INACTIVE'
# Node names from DEMO_NODE_REGISTRY, which are also the App ids the gateway derives
# for nodes in the root namespace - so each constant is at once the launch key, the
# gate entity id, and what the fault description must name.
TARGET_NODE = 'managed_lifecycle'
ACTIVE_NODE = 'managed_lifecycle_active'
CHANGE_STATE_SERVICE = f'/{TARGET_NODE}/change_state'

_DETECTOR_PREFIX = 'plugins.graph_watchdog.detectors.lifecycle_expectation'

# How long a fault must stay ABSENT for the two silence scenarios. Counted from the
# arming gate, not process start, so bringup cannot eat it. At the 200 ms tick this is
# ~100 ticks against a grace of 3 (default 5): a detector that wrongly counted the
# node would have raised - and been CONFIRMED by the harness's
# confirmation_threshold: -2 fault_manager - dozens of times over.
SILENT_WINDOW_SEC = 20.0

# The "healing_threshold" scenario needs real WALL-CLOCK margin, not just tick count: it
# kills and respawns a real OS process (the target node) and needs the gateway's entity
# cache to actually notice the departure and the return, all comfortably inside the
# tracker's own absence_grace (a fixed 3 ticks - kDefaultAbsenceGrace in
# lifecycle_expectation_tracker.hpp, not configurable). A much slower cadence than the
# other scenarios buys that margin: at 1 s/tick (the shipped production default) the
# absence budget is a full 3 s, comfortably longer than a SIGTERM + respawn_delay +
# rediscovery round trip ever needs.
HEALING_TICK_INTERVAL_MS = 1000
# kDefaultAbsenceGrace, lifecycle_expectation_tracker.hpp - fixed, not configurable.
# Mirrored here (not imported - this is a separate Python process) so _blink() can
# compute the exact wall-clock budget a blink is supposed to stay inside.
HEALING_ABSENCE_GRACE_TICKS = 3
# Sped up from the gateway's own 1000 ms default so the entity cache reflects a kill or a
# respawn quickly relative to the absence budget above, not eat most of it.
HEALING_REFRESH_DEBOUNCE_MS = 300
# launch will not even ATTEMPT to restart the process before this elapses, so it is a
# real, enforced floor under each blink's duration - not just a hint. It has to exceed ONE
# tick interval: a node that leaves and returns inside a single tick is never sampled
# absent at all, so the absence grace this scenario is named for is not exercised and the
# blink proves nothing. The ceiling is the other side of the same window - the whole
# absence must stay under HEALING_ABSENCE_GRACE_TICKS ticks, or what is being exercised is
# the ordinary sustained-absence path instead. Both edges are asserted per blink rather
# than assumed from this constant.
HEALING_RESPAWN_DELAY_SEC = 1.4
# Slept after the SIGTERM before polling for the node's return: gives the respawn_delay
# above, the new process's own startup, and DDS rediscovery room to finish before the
# poll below starts, without itself risking the 3 s absence budget.
HEALING_BLINK_SLEEP_SEC = 1.5
# Two separate blink episodes, matching the scenario this detector's design doc walks
# through: a node the detector has already reported vanishing and returning twice must
# never be healed by either one.
HEALING_BLINK_COUNT = 2

# The "unreadable" scenario needs to actually cross kUnmeasuredHoldTicks (60,
# lifecycle_expectation_detector.cpp - fixed, not configurable) consecutive
# matched-and-unread ticks before GRAPH_NODE_UNREADABLE raises, unlike every other
# scenario here which only needs a handful of `grace` ticks. A much faster cadence
# than TICK_INTERVAL_MS above keeps that affordable: 60 ticks pass in single-digit
# seconds rather than the ~4 minutes the shipped 1 s default would cost.
UNREADABLE_TICK_INTERVAL_MS = 100
UNREADABLE_NODE = 'unreadable_lifecycle'
FAULT_CODE_UNREADABLE = 'GRAPH_NODE_UNREADABLE'
# Same 100-tick shape as SILENT_WINDOW_SEC above (~100 ticks there too, at that
# scenario's 200 ms cadence), scaled to this scenario's faster tick: 100 *
# UNREADABLE_TICK_INTERVAL_MS. Proving GRAPH_NODE_INACTIVE's absence does not
# actually need this many ticks - only a MEASURED not-active read ever advances the
# violation streak (lifecycle_expectation_tracker.hpp), so an unread label raising here
# would be a real bug, not a late one - but a window this wide leaves room to let the hold below
# start accumulating in the background while this one runs.
UNREADABLE_INACTIVE_SILENCE_SEC = 10.0
# The raise poll's budget. Real expected cost: (kUnmeasuredHoldTicks + 1) consecutive
# matched ticks - the hold is "frozen one past itself", so the raise lands on the SAME
# tick the counter first exceeds 60, not one tick later - at UNREADABLE_TICK_INTERVAL_MS
# is 61 * 0.1 = 6.1 s of pure tick cadence. On top of that, THREE of those ticks (the
# initial LifecycleWatcher seed plus its two re-seed attempts - kReseedAttempts in
# lifecycle_watcher.cpp, spent once and never replenished) each run a BLOCKING GetState
# call that only returns once the reader's own timeout expires (500 ms,
# Ros2LifecycleStateReader's default) because this fixture never answers - and that
# block happens INSIDE tick() itself, so it adds to the gap before the NEXT tick rather
# than eating into UNREADABLE_TICK_INTERVAL_MS. Budget: 6.1 + 3 * 0.5 = 7.6 s of
# tick-driven work, plus the same generous bringup margin (gateway/fault_manager start,
# demo_delay, DDS discovery, entity-cache debounce) the sibling scenarios' 60 s "raise
# poll" budgets already carry for a MUCH smaller expected cost - kept here rather than
# tightened, since CI slowness is the variable this margin exists for, not the tick math.
UNREADABLE_RAISE_TIMEOUT_SEC = 60.0 * TIME_SCALE

# The "departure_keeps" scenario's own budgets, for what a DEPARTURE does to an
# already-reported unmeasured fault: nothing. A SIGTERM's DDS "participant left" propagates
# in well under a second and HEALING_REFRESH_DEBOUNCE_MS (reused below for the exact reason
# healing_threshold defines it: giving the entity cache room to reflect a kill quickly)
# bounds the entity-cache catch-up. The generous margin below is the same CI-slowness
# allowance every other poll budget in this file carries, not a measurement of that cost.
ABSENCE_DEPARTURE_TIMEOUT_SEC = 30.0 * TIME_SCALE
ABSENCE_CLEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE
# Slept after a departure has been CONFIRMED on GET /apps, before asserting the fault
# survived it. Long enough to cover every horizon that could have discarded the node's
# evidence at this scenario's 100 ms cadence: the tracker's absence grace (3 ticks),
# its unmeasured hold (60 ticks) and a level-triggered clear reaching the fault_manager's
# healing threshold - so "the fault is still here" is a measurement, not a race won.
DEPARTURE_SETTLE_SEC = 10.0
# A `prune_grace` at the widest accepted value (kMaxPruneGrace, 3600 ticks - 360 s at this
# cadence), which also exercises that key's upper endpoint through the real config-delivery
# path. It can no longer change any outcome in these scenarios: the age horizon reaches
# IDLE bookkeeping only, and a node carrying a matured unmeasured clock is never idle.
ABSENCE_PRUNE_GRACE_TICKS = 3600
# How long a mutually-exclusive sibling code must stay absent. Short (a few ticks at every
# cadence in this file) because the claim is "not at the same time as", not "never" - but a
# WINDOW with the channel checked on every poll, never a single poll that timed out
# silently, which cannot tell "no such fault" from "could not ask".
MUTUAL_EXCLUSION_WINDOW_SEC = 3.0

# The "restart_loop" scenario: the whole point of holding evidence across absence. A node
# that crash-loops - up for a while, gone for a while, forever - touches absence
# periodically BY CONSTRUCTION, so any horizon that discards its evidence on absence makes
# exactly that node permanently invisible. Reuses `calibration` (a plain node with no
# lifecycle interface, so the NOT-MANAGED cause) with launch's own respawn, and SIGTERMs it
# on a cadence that never lets it accumulate the 60 consecutive PRESENT ticks the hold would
# otherwise need.
RESTART_LOOP_TICK_INTERVAL_MS = 100
# Seconds the node is left alive between kills. At the 100 ms tick that is at most ~25
# ticks present per cycle - comfortably under kUnmeasuredHoldTicks (60), so a run in which
# the clock restarted on every absence could never mature however many cycles it ran.
RESTART_LOOP_UPTIME_SEC = 2.0
# launch will not even ATTEMPT to restart the process before this elapses, so it is a real,
# enforced floor under each absence: 1.0 s is 10 ticks, well past the tracker's 3-tick
# absence grace, i.e. every cycle genuinely crosses the horizon under test.
RESTART_LOOP_RESPAWN_DELAY_SEC = 1.0
# Wall-clock budget for the kill loop. The clock advances on every tick once the node has
# been seen, so ~61 ticks (6.1 s) plus the held blink ticks is the real expected cost; this
# is several times that, and a detector that discarded evidence on absence stays silent for
# all of it however many cycles fit.
RESTART_LOOP_WINDOW_SEC = 45.0 * TIME_SCALE
# Kills the loop must have completed before a sighting of the fault is accepted. Three, so
# the run is unambiguously a LOOP rather than one departure, and so the fault is proven to
# survive the restarts that follow its first appearance. At ~3.5 s per cycle that is ~11 s,
# comfortably inside the window above.
RESTART_LOOP_MIN_CYCLES = 3

# The "not_managed" scenario's own fixture: unlike "unreadable" (which needs a purpose-built
# node that advertises get_state/change_state but never answers), NOT_MANAGED only needs a
# node with NO lifecycle interface at all - and a plain one already exists in this package's
# demo fixtures (DEMO_NODE_REGISTRY's 'calibration', a plain rclcpp::Node service server with
# no lifecycle_msgs services whatsoever), so nothing new had to be built for this. Reuses the
# same fast cadence as "unreadable"/"departure_keeps": kUnmeasuredHoldTicks (60, fixed) is far
# more ticks than the shipped 1 s default affords inside any reasonable poll budget.
NOT_MANAGED_TICK_INTERVAL_MS = 100
NOT_MANAGED_NODE = 'calibration'
FAULT_CODE_NOT_MANAGED = 'GRAPH_NODE_NOT_MANAGED'
# Same shape and derivation as UNREADABLE_RAISE_TIMEOUT_SEC above - the expected cost is the
# same (kUnmeasuredHoldTicks + 1) consecutive matched ticks at this scenario's own cadence,
# and NOT_MANAGED needs no blocking GetState round trips at all (there is no lifecycle
# service to call), so if anything this scenario's real cost is lower, not higher - the same
# generous CI-slowness margin is kept rather than tightened.
NOT_MANAGED_RAISE_TIMEOUT_SEC = 60.0 * TIME_SCALE
# Same margins as ABSENCE_DEPARTURE_TIMEOUT_SEC / ABSENCE_CLEAR_TIMEOUT_SEC above, for the
# same OTHER way an unmeasured code clears: an already-reported node leaving the graph and
# staying away past the absence grace, never by being read (there is nothing to read here).
NOT_MANAGED_DEPARTURE_TIMEOUT_SEC = 30.0 * TIME_SCALE
NOT_MANAGED_CLEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE

# The detector id its own status block is filed under on GET /x-medkit-watchdog.
DETECTOR_ID = 'lifecycle_expectation'

# ---- the "cap_pressure" scenario: what a FULL tracked-node cap does -------------------
#
# Two required nodes and a cap of ONE, so exactly one of them can be tracked and the other
# is refused every tick. Reachable only because `tracked_node_cap` is a config key; with a
# compile-time 512 this whole scenario would need 513 real lifecycle nodes.
CAP_TICK_INTERVAL_MS = 100
# The lexicographically FIRST of the two required nodes, so it is the one that wins the
# single slot (the tracker keys its per-tick map by fqn, a std::map, so the sweep is
# lexicographic). It answers "unconfigured", i.e. a measured violation.
CAP_TRACKED_NODE = 'cap_droppable'
# The one that is REFUSED: present, required, matched, and never checked.
CAP_REFUSED_NODE = 'cap_stuck'
# One node, one slot.
CAP_NODE_CAP = 1
# When the REFUSED node is started, against the tracked one's own 2.0 s. The slot goes to
# whichever required node is matched first, so the two must not race: eight seconds is 80
# ticks at this scenario's cadence, far longer than discovery plus the grace the tracked node
# needs to be carrying evidence by the time the second node exists at all.
CAP_REFUSED_NODE_DELAY_SEC = 10.0
# Long enough for the unmeasured hold (60 ticks, fixed) to run out after the tracked node's
# lifecycle services are dropped, plus the usual CI-slowness margin every raise poll here
# carries: 61 * 0.1 = 6.1 s of tick cadence.
CAP_NOT_MANAGED_RAISE_TIMEOUT_SEC = 60.0 * TIME_SCALE
# How long GRAPH_NODE_INACTIVE's clear must stay withheld while a required node is refused.
# ~100 ticks at this cadence, and the fault_manager runs at healing_threshold 1 here, so a
# single spurious PASSED anywhere in the window is enough to fail the assertion.
CAP_WITHHOLD_WINDOW_SEC = 10.0
# Budget for the refused node to be admitted and confirmed once the departed entry is
# collapsed: absence grace (3 ticks) + grace (3 ticks) + the report reaching the store.
CAP_ADMISSION_TIMEOUT_SEC = 60.0 * TIME_SCALE

# ---- the "unsettled_departure" scenario: what ONE bad sweep may decide ----------------
#
# A deliberately SLOW cadence. The claim is about how many consecutive ticks an unmeasured
# observation is corroborated for before a departure is allowed to continue it, so the test
# has to place its actions between ticks rather than race them: at 500 ms a single deliberate
# hold is one tick, and the entity cache catching up with a SIGTERM costs well under the
# settling budget rather than most of it.
UNSETTLED_TICK_INTERVAL_MS = 500
# kDefaultObservationSettleTicks (lifecycle_expectation_tracker.hpp) - consecutive matched
# ticks an UNMEASURED observation must hold before absence may continue it. Mirrored here
# (this is a separate Python process) so both legs below can be paced against the real bound.
UNSETTLED_SETTLE_TICKS = 6
# The node whose lifecycle services vanish for a SHORT spell and which then exits cleanly.
UNSETTLED_BLINK_NODE = 'blink_departer'
# The node whose lifecycle services vanish and STAY gone long enough to be corroborated
# before it exits - the positive control that makes the blinker's silence mean something.
UNSETTLED_SETTLED_NODE = 'settled_departer'
# Ticks the settled leg holds its dropped state for: comfortably past the settle budget, so
# its departure genuinely continues a corroborated not-managed observation.
UNSETTLED_SETTLED_HOLD_TICKS = 10
# The unmeasured hold (60 ticks, fixed) plus the absence grace, at this cadence: 63 * 0.5 =
# 31.5 s, plus the usual margin. Both legs are measured against this same budget - the
# settled one must raise inside it, the blinking one must never raise at all.
UNSETTLED_RAISE_TIMEOUT_SEC = 90.0 * TIME_SCALE
# Sped up from the gateway's own 1000 ms default so the entity cache reflects a kill quickly
# relative to the settling budget the blink leg has to stay inside.
UNSETTLED_REFRESH_DEBOUNCE_MS = 300

# ---- the "wide_grace" scenario: a `grace` past the accepted maximum -------------------
#
# The value that used to be the accepted maximum (INT_MAX - 1). A streak that has to reach
# it advances one per tick, so a node under it is never CONFIRMED and never CLEARED either:
# it sits in the tracker's pending set, the withheld-clear guard returns early every tick,
# and GRAPH_NODE_INACTIVE is silent for the life of the process - about 24 days at the
# shipped 1 s cadence, and about 2.5 days at this scenario's own.
WIDE_GRACE_TICK_INTERVAL_MS = 100
WIDE_GRACE_VALUE = 2147483646
# The raise poll's budget. With the value refused and the documented default (5) in force,
# the node is confirmed within a handful of ticks; the margin is the usual bringup allowance.
WIDE_GRACE_RAISE_TIMEOUT_SEC = 60.0 * TIME_SCALE

# ---- the "restart_departed" scenario: what a gateway RESTART does ---------------------
#
# Characterisation, not a fix: it records where "a departure never heals a fault" actually
# ends. Inside one gateway process it holds; across a restart the tracker starts empty, the
# departed node is not in the graph, and its require_active entry matches nothing - which the
# detector cannot tell apart from a misspelt entry, so the deliberately bounded never-matched
# hold expires and the clear flows.
RESTART_TICK_INTERVAL_MS = 100
# kDefaultUnmeasuredHoldTicks (60, fixed): the never-matched hold's own bound, after which
# the clear is released. 60 * 0.1 = 6 s, plus healing at healing_threshold 1 and the usual
# CI margin.
RESTART_HEAL_TIMEOUT_SEC = 90.0 * TIME_SCALE


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
    }
    demo_node = TARGET_NODE
    extra_gateway_params = None
    healing_threshold = 3
    if SCENARIO == 'main':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
    elif SCENARIO == 'negative_control':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [ACTIVE_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        demo_node = ACTIVE_NODE
    elif SCENARIO == 'healing_threshold':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = HEALING_TICK_INTERVAL_MS
        extra_gateway_params = {'discovery.refresh_debounce_ms': HEALING_REFRESH_DEBOUNCE_MS}
        # A low healing_threshold is the whole point of this scenario: it makes even ONE
        # spurious PASSED (from a blink the withheld-clear guard fails to hold) enough to
        # move the debounce counter noticeably - the README's own recommended value.
        healing_threshold = 1
        # Launched separately below, with a handle this scenario can SIGTERM and respawn
        # under its own control - create_demo_nodes() gives no such handle back.
        demo_node = None
    elif SCENARIO == 'unreadable':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [UNREADABLE_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = UNREADABLE_TICK_INTERVAL_MS
        demo_node = UNREADABLE_NODE
    elif SCENARIO == 'departure_keeps':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [UNREADABLE_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = UNREADABLE_TICK_INTERVAL_MS
        # The upper endpoint of prune_grace's documented range, through the real
        # config-delivery path - see ABSENCE_PRUNE_GRACE_TICKS's own comment for why it can
        # no longer change this scenario's outcome either way.
        detector_params[f'{_DETECTOR_PREFIX}.prune_grace'] = ABSENCE_PRUNE_GRACE_TICKS
        # Same speed-up as healing_threshold, and for the same reason: the entity cache
        # needs to reflect this scenario's kill quickly relative to the (fixed, 3-tick)
        # absence grace the assertions are timed against.
        extra_gateway_params = {'discovery.refresh_debounce_ms': HEALING_REFRESH_DEBOUNCE_MS}
        # Launched separately below, with a handle this scenario can SIGTERM - permanently,
        # unlike healing_threshold's respawning blink - create_demo_nodes() gives no such
        # handle back.
        demo_node = None
    elif SCENARIO == 'not_managed':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [NOT_MANAGED_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = NOT_MANAGED_TICK_INTERVAL_MS
        detector_params[f'{_DETECTOR_PREFIX}.prune_grace'] = ABSENCE_PRUNE_GRACE_TICKS
        extra_gateway_params = {'discovery.refresh_debounce_ms': HEALING_REFRESH_DEBOUNCE_MS}
        # Launched separately below, with a handle this scenario can SIGTERM - a plain demo
        # node has no lifecycle to drive, so a permanent kill is the only interesting thing
        # left to do to it once it has raised.
        demo_node = None
    elif SCENARIO == 'restart_loop':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [NOT_MANAGED_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = RESTART_LOOP_TICK_INTERVAL_MS
        extra_gateway_params = {'discovery.refresh_debounce_ms': HEALING_REFRESH_DEBOUNCE_MS}
        # Launched separately below with respawn, so the test can SIGTERM it over and over
        # and have launch bring it straight back - the crash loop this scenario is about.
        demo_node = None
    elif SCENARIO == 'cap_pressure':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [
            CAP_TRACKED_NODE, CAP_REFUSED_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params[f'{_DETECTOR_PREFIX}.tracked_node_cap'] = CAP_NODE_CAP
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = CAP_TICK_INTERVAL_MS
        extra_gateway_params = {'discovery.refresh_debounce_ms': HEALING_REFRESH_DEBOUNCE_MS}
        # A sensitive healing_threshold, for the same reason healing_threshold uses one:
        # this scenario's central claim is that NO clear is emitted while a required node is
        # refused, and at 1 a single spurious PASSED walks the fault straight to HEALED.
        healing_threshold = 1
        demo_node = None  # both fixtures are launched below, with handles this test can drive
    elif SCENARIO == 'unsettled_departure':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [
            UNSETTLED_BLINK_NODE, UNSETTLED_SETTLED_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = UNSETTLED_TICK_INTERVAL_MS
        extra_gateway_params = {'discovery.refresh_debounce_ms': UNSETTLED_REFRESH_DEBOUNCE_MS}
        demo_node = None
    elif SCENARIO == 'wide_grace':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = WIDE_GRACE_VALUE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = WIDE_GRACE_TICK_INTERVAL_MS
        extra_gateway_params = {'discovery.refresh_debounce_ms': HEALING_REFRESH_DEBOUNCE_MS}
        demo_node = None  # launched below, so the test can SIGTERM it
    elif SCENARIO == 'restart_departed':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = RESTART_TICK_INTERVAL_MS
        extra_gateway_params = {'discovery.refresh_debounce_ms': HEALING_REFRESH_DEBOUNCE_MS}
        # The heal this scenario RECORDS has to be observable, so the debounce counter must
        # reach HEALED promptly once the clear starts flowing.
        healing_threshold = 1
        demo_node = None  # launched below, so the test can SIGTERM it before the gateway restart
    # default_config: deliberately NO detectors.lifecycle_expectation keys at all -
    # the launch under test is the shipped default itself.

    launch_description, context = create_watchdog_test_launch(
        detector_params=detector_params,
        extra_gateway_params=extra_gateway_params,
        # 'managed_lifecycle' stays unconfigured (auto_activate defaults to false):
        # present in the graph, alive, but not active - exactly the silent state this
        # detector exists to catch. 'managed_lifecycle_active' is the same executable
        # launched with auto_activate:=true, so it reaches "active" on its own.
        demo_nodes=[demo_node] if demo_node else [],
        port=PORT,
        # The clear must reach HEALED, i.e. leave the default active-fault query,
        # which the fault_manager only does when healing is enabled - see harness.py
        # and the README's "Closing the loop".
        healing_enabled=True,
        healing_threshold=healing_threshold,
        # Only "main" and "restart_departed" restart the gateway. The other scenarios must
        # keep an unexpected gateway (or, for healing_threshold, target-node) death visible
        # as a failure everywhere it is not the deliberate subject of the test.
        gateway_respawn=(SCENARIO in ('main', 'restart_departed')),
    )

    if SCENARIO == 'healing_threshold':
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[TARGET_NODE]
        target_node_action = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=executable,
            name=ros_name,
            namespace=namespace,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
            # Respawns under the SAME node name after the test's own SIGTERM - see
            # TestLifecycleExpectationHealingThreshold._blink(). This is the "node that
            # respawned stuck" case the withheld-clear guard's design doc names.
            respawn=True,
            respawn_delay=HEALING_RESPAWN_DELAY_SEC,
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[target_node_action]))
        context['target_node'] = target_node_action

    if SCENARIO == 'departure_keeps':
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[UNREADABLE_NODE]
        target_node_action = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=executable,
            name=ros_name,
            namespace=namespace,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
            # No respawn=True: unlike healing_threshold's blink, this scenario kills the
            # fixture PERMANENTLY - the whole point is proving the clear that follows a
            # real, sustained departure, never a bounce back.
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[target_node_action]))
        context['target_node'] = target_node_action

    if SCENARIO == 'not_managed':
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[NOT_MANAGED_NODE]
        target_node_action = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=executable,
            name=ros_name,
            namespace=namespace,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
            # No respawn=True: same permanent-kill shape as "departure_keeps", proving the
            # fault survives a real, sustained departure rather than a bounce back.
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[target_node_action]))
        context['target_node'] = target_node_action

    if SCENARIO == 'restart_loop':
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[NOT_MANAGED_NODE]
        target_node_action = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=executable,
            name=ros_name,
            namespace=namespace,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
            # The crash loop itself: launch brings the same node straight back after every
            # SIGTERM the test sends, and respawn_delay is an enforced floor under each
            # absence (see RESTART_LOOP_RESPAWN_DELAY_SEC).
            respawn=True,
            respawn_delay=RESTART_LOOP_RESPAWN_DELAY_SEC,
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[target_node_action]))
        context['target_node'] = target_node_action

    if SCENARIO in ('wide_grace', 'restart_departed'):
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[TARGET_NODE]
        target_node_action = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=executable,
            name=ros_name,
            namespace=namespace,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
            # No respawn: both scenarios kill it permanently and then measure what happens
            # to the fault it left behind.
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[target_node_action]))
        context['target_node'] = target_node_action

    if SCENARIO == 'cap_pressure':
        # Only the tracked one is ever driven (its lifecycle services are dropped, then it is
        # killed); the refused one just has to exist and stay stuck.
        #
        # STAGGERED, not launched together: with a cap of one, the single slot goes to
        # whichever required node the detector matches FIRST, and two nodes started at the
        # same instant are discovered in whatever order DDS and the entity cache happen to
        # settle on. That would make which node is tracked and which is refused a coin flip,
        # and every assertion below names one of them. The gap is many ticks at this
        # scenario's 100 ms cadence, so the first node is tracked and carrying evidence long
        # before the second one exists to be refused.
        tracked = _droppable_node(CAP_TRACKED_NODE, 'unconfigured')
        refused = _droppable_node(CAP_REFUSED_NODE, 'unconfigured')
        launch_description.add_action(TimerAction(period=2.0, actions=[tracked]))
        launch_description.add_action(
            TimerAction(period=CAP_REFUSED_NODE_DELAY_SEC, actions=[refused]))
        context['tracked_node'] = tracked
        context['refused_node'] = refused

    if SCENARIO == 'unsettled_departure':
        blink = _droppable_node(UNSETTLED_BLINK_NODE, 'active')
        settled = _droppable_node(UNSETTLED_SETTLED_NODE, 'active')
        launch_description.add_action(TimerAction(period=2.0, actions=[blink, settled]))
        context['blink_node'] = blink
        context['settled_node'] = settled

    return launch_description, context


def _droppable_node(name, state_label):
    """One ``droppable_lifecycle_node`` instance under `name`, answering `state_label`.

    The fixture (``test/e2e/droppable_lifecycle_node.cpp``, built and installed by this
    package's own CMakeLists) looks like a managed lifecycle node to the gateway's discovery
    layer, answers ``get_state`` with `state_label`, and stops advertising both lifecycle
    services when ``drop_services:=true`` is set on it. Launching two instances under
    different names is how a scenario gets two independently drivable required nodes out of
    one executable.
    """
    return launch_ros.actions.Node(
        package='ros2_medkit_graph_watchdog',
        executable='droppable_lifecycle_node',
        name=name,
        output='screen',
        parameters=[{'state_label': state_label}],
        additional_env=get_coverage_env('ros2_medkit_graph_watchdog'),
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


def _fault_record(port, code, timeout=30.0 * TIME_SCALE, interval=0.5):
    """Poll ``GET /faults?status=all`` until `code` appears, whatever its status.

    ``poll_faults`` uses the default (pending+confirmed) filter, so a fault that
    HEALED disappears from it - which is indistinguishable from a fault that was
    never raised, and from a gateway that is up but has not reached the
    fault_manager yet. The restart test needs the record itself, including the
    fields that survive a heal, so it asks for every status.

    Returns the matching item dict, or ``None`` on timeout.
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


def _poll_fault_description_contains(port, code, needle, timeout=30.0 * TIME_SCALE, interval=0.5):
    """Poll ``GET /faults?status=all`` until `code`'s description contains `needle`.

    A single read of the description is a snapshot of whatever the last emitted tick said,
    so a claim about the description CHANGING (a newly admitted node appearing in it, a
    departed one starting to say it has left) needs a poll rather than one sample plus a
    sleep long enough to be safe. ``True`` once it matches, ``False`` on timeout, after
    printing the last description seen - which is gone once the launch tears down.
    """
    deadline = time.monotonic() + timeout
    last_seen = f'{code} was never in the store at all'
    while time.monotonic() < deadline:
        record = _fault_record(port, code, timeout=interval)
        if record is not None:
            last_seen = record.get('description', '')
            if needle in last_seen:
                return True
        time.sleep(interval)
    print(f'_poll_fault_description_contains({code!r}, {needle!r}) timed out after '
          f'{timeout}s; last description: {last_seen!r}')
    return False


def _wait_until_port_is_down(port, timeout=60.0 * TIME_SCALE, interval=0.2):
    """Wait until the gateway's HTTP port stops answering. True once it does."""
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            requests.get(f'{base}/health', timeout=2)
        except requests.exceptions.RequestException:
            return True
        time.sleep(interval)
    return False


def _poll_watchdog_entity(port, app_id, lifecycle, timeout=30.0 * TIME_SCALE, interval=0.5):
    """Poll GET /x-medkit-watchdog until `app_id` appears with this lifecycle label.

    The absence scenarios need one fact wait_until_watchdog_armed cannot give them:
    that the target's lifecycle label was actually READ by the live stack. The armed
    gate treats an unread label as ok (node_ok defaults open for unknown state), and
    the tracker treats an unread label as benign - so a run in which the label never
    arrived produces the same silence the assertion is looking for. Pinning the label
    through the plugin's own status route closes that hole: 'unconfigured' proves the
    false-positive trigger was fully present, 'active' proves the negative control is
    actually exercising an active node rather than an unread one, and '' (empty string)
    proves the "unreadable" scenario's fixture was matched and its GetState genuinely
    never came back - LifecycleWatcher seeds a tracked entry's label to "" and only
    overwrites it once a real read succeeds (lifecycle_watcher.cpp), so '' is not "no
    data yet", it is "asked, and still waiting".

    Returns the matching entity dict, or ``None`` on timeout (after printing the
    last-seen payload, which is gone once the launch tears down).
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
    print(f'_poll_watchdog_entity(app_id={app_id!r}, lifecycle={lifecycle!r}) timed out '
          f'after {timeout}s; last watchdog status: {last_seen}')
    return None


def _poll_apps_absent(port, app_id, timeout=30.0 * TIME_SCALE, interval=0.5):
    """Poll ``GET /apps`` until `app_id` is no longer listed. ``True`` once it is gone.

    The "departure_keeps" scenario's own gate: confirms the fixture process it just
    SIGTERM'd is really gone from the operator-visible SOVD entity graph before
    asserting anything about the fault, or the clear that follows would prove nothing -
    it could just as well be racing a graph the entity cache has not caught up with yet.
    ``GET /apps`` and the detector's own per-tick snapshot read the exact same
    ``ThreadSafeEntityCache`` (``DiscoveryHandlers::get_apps()`` and
    ``PluginContextImpl::get_entity_snapshot()`` both call
    ``node_->get_thread_safe_cache()``), so this is a poll of the precise input the
    guard's absence counting is driven from, not a proxy for it.

    Parameters
    ----------
    port : int
        Gateway HTTP port.
    app_id : str
        The App id expected to disappear from ``GET /apps``.
    timeout : float
        Maximum time to wait in seconds.
    interval : float
        Sleep between retries in seconds.

    Returns
    -------
    bool
        ``True`` once `app_id` is absent from ``GET /apps``, ``False`` on timeout (after
        printing the last-seen id list).

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


def _poll_apps_present(port, app_id, timeout=30.0 * TIME_SCALE, interval=0.5):
    """Poll ``GET /apps`` until `app_id` IS listed. ``True`` once it appears.

    The "not_managed" scenario's own gate: `calibration` is launched via a delayed
    ``TimerAction`` (2 s) plus its own process startup and DDS discovery, so at the moment
    test_01 runs it may genuinely not have appeared yet. `_poll_apps_absent` above cannot
    stand in for this the other way round with a short timeout: it returns True on the
    FIRST poll that does not see the node, which on a fresh launch is every early poll
    before the node has started - the opposite of what "present" needs. This instead loops
    until the node genuinely appears (or the deadline is reached).
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
                if app_id in ids:
                    return True
            else:
                last_seen = f'HTTP {response.status_code} from GET /apps'
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /apps failed: {exc}'
        time.sleep(interval)
    print(f'_poll_apps_present({app_id!r}) timed out after {timeout}s; last seen: {last_seen}')
    return False


def _set_bool_parameter(client_node, service_name, param_name, value, timeout=30.0 * TIME_SCALE):
    """Set one bool parameter on a REMOTE node via its own ``~/set_parameters`` service.

    Drives ``unreadable_lifecycle_node.cpp``'s ``start_answering`` parameter at a time
    the TEST controls - after it has independently confirmed GRAPH_NODE_UNREADABLE
    actually raised - rather than racing a wall-clock timer in the fixture against the
    60-tick hold this scenario needs to observe first. Every ``rclcpp::Node`` starts
    this service automatically (``start_parameter_services`` defaults to true, and
    this fixture never overrides it), so no special wiring is needed on the node side
    beyond the ``add_on_set_parameters_callback`` it already registers.

    Returns ``True`` once the remote node accepts the change, ``False`` if the service
    never became available or the call did not complete/succeed.
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


def _pump_sse_stream(response, frames, stop_event):
    """Collect SSE frames as dicts of field -> value, same shape as data.py's fixture.

    Mirrors test_sse_fault_stream.test.py's ``_pump_stream`` - this file's row-6 test is
    the only reader of ``GET /faults/stream`` in this suite, so the parsing lives here
    rather than being shared, matching the rest of this module's harness.py-vs-local split.
    """
    current = {}
    try:
        for line in response.iter_lines(decode_unicode=True):
            if stop_event.is_set():
                break
            if line is None:
                continue
            if line == '':
                if current:
                    frames.append(current)
                    current = {}
                continue
            if line.startswith(':'):
                continue  # keepalive comment
            key, _, value = line.partition(':')
            current[key.strip()] = value.strip()
    except Exception:  # noqa: BLE001 - closed socket on test teardown
        pass


class TestLifecycleExpectationMain(unittest.TestCase):
    """GRAPH_NODE_INACTIVE raises for a required-but-unconfigured node, heals on activate."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('lifecycle_expectation_e2e_client')
        cls._change_state = cls._client_node.create_client(ChangeState, CHANGE_STATE_SERVICE)

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def _call_transition(self, transition_id, reached_label,
                         timeout=30.0 * TIME_SCALE, attempts=3):
        """Drive one real lifecycle transition, retrying a call that never comes back.

        The whole scenario hangs off two of these, and a single-shot call makes one lost
        response (or a demo-node executor stalled under a sanitizer at the wrong moment)
        fail the entire multi-minute run on one RPC. The retry does NOT widen the budget:
        the same total wait is split across `attempts`, so a genuinely dead service still
        fails after the same wall time, with the last attempt's outcome named.

        `reached_label` is the lifecycle state this transition ends in. It is needed
        because a retry introduces a case a single-shot call does not have: if the FIRST
        attempt was applied and only its response was lost, the retried transition is
        invalid from the state the node has already reached and comes back rejected. That
        is a success, not a failure, and the only way to tell it from a genuinely refused
        transition is to look at where the node actually is.
        """
        self.assertTrue(
            type(self)._change_state.wait_for_service(timeout_sec=timeout),
            f'{CHANGE_STATE_SERVICE} never became available',
        )
        per_attempt = timeout / attempts
        result = None
        retried = False
        for attempt in range(attempts):
            request = ChangeState.Request()
            request.transition.id = transition_id
            future = type(self)._change_state.call_async(request)
            rclpy.spin_until_future_complete(
                type(self)._client_node, future, timeout_sec=per_attempt)
            result = future.result()
            if result is not None:
                break
            # Drop the request the server may still answer, so the next attempt's future
            # cannot be completed by a straggling response to this one.
            type(self)._change_state.remove_pending_request(future)
            retried = True
            print(f'ChangeState (transition {transition_id}) attempt {attempt + 1} of '
                  f'{attempts} did not complete within {per_attempt:.1f}s; retrying')
        self.assertIsNotNone(
            result,
            f'ChangeState call (transition {transition_id}) never completed in {attempts} '
            f'attempts over {timeout}s',
        )
        if result.success:
            return
        self.assertTrue(
            retried,
            f'ChangeState call (transition {transition_id}) was rejected on its first '
            'attempt - the node refused a transition that must be valid from where it was',
        )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, reached_label, timeout=10.0 * TIME_SCALE),
            f'ChangeState (transition {transition_id}) was rejected after a retry and '
            f'{TARGET_NODE} is not in "{reached_label}" either - the transition neither '
            'applied on the lost attempt nor succeeded on the retry',
        )

    def test_01_present_but_unconfigured_raises_naming_it(self):
        # Gate on the plugin being live and globally armed BEFORE polling for the raise.
        # Not on app_id=TARGET_NODE, deliberately: the gate reports a tracked node with a
        # known non-active label as 'warming_up' (its node_ok is false - the exact
        # condition under test), so that per-entity state stays suppressed for as long
        # as the fault condition exists. What the raise actually needs is the SOURCE
        # entity's arming (the aggregate goes out under 'graph_watchdog'), for which
        # global armed is the precise precondition - and the gate still proves the .so
        # loaded and the tick loop ran, so a bringup failure dies here naming itself
        # instead of as a 60 s poll timeout blaming the detector.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load, its tick loop never ran, or the bringup grace never elapsed, so no '
            'raise below could mean anything',
        )

        # managed_lifecycle launches unconfigured: present in the graph, alive, but not
        # active. That alone must raise once it persists past the configured grace.
        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0 * TIME_SCALE)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} never raised while {TARGET_NODE} stayed unconfigured',
        )
        self.assertIn(TARGET_NODE, fault.get('description', ''))

        # The flat /faults list carries a fault whatever its source is; only the
        # entity-scoped surface proves an operator can OPEN it somewhere (see
        # test_qos_e2e.test.py's identical rationale for the same entity).
        self.assertIsNotNone(
            poll_entity_faults(PORT, 'apps/graph_watchdog', FAULT_CODE, timeout=30.0 * TIME_SCALE),
            f'{FAULT_CODE} is not reachable at /apps/graph_watchdog/faults - the '
            'entity the plugin publishes does not own the fault it raises',
        )

    def test_02_configured_but_inactive_still_raises(self):
        # Explicitly reach "inactive" (unconfigured -> configuring -> inactive) and
        # deliberately do NOT activate. Inactive is still not active, so the
        # level-triggered raise must keep the fault active - a detector that treated
        # "configured" as good enough would heal it here.
        self._call_transition(Transition.TRANSITION_CONFIGURE, reached_label='inactive')

        fault = poll_faults(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} did not stay raised once {TARGET_NODE} reached "inactive"',
        )
        self.assertIn(TARGET_NODE, fault.get('description', ''))

    def test_02b_gateway_restart_never_heals_a_still_inactive_node(self, gateway_node):
        """The withheld-clear guard, at the tier that counts: a real restart.

        The guard exists because a gateway restart brings every detector counter back
        to zero while the fault it raised is still in the fault_manager's store (a
        separate process, so it survives). For a whole `grace` window after the
        restart the tracker reports nothing affected even though the detector's own
        reads say the node is still not active, and the level-triggered emitter turns
        that into a clear - which, with healing enabled, walks a CONFIRMED
        GRAPH_NODE_INACTIVE to HEALED on a robot that never moved.

        Nothing below this tier can reach that state: the C++ integration test rebuilds
        the detector against a fake ReportFault sink, and the other e2e scenarios run
        one gateway process from start to finish.

        The instrument is ``last_passed``, not the fault's current status. A heal here
        is TRANSIENT - the re-raise follows within a few ticks - so a status sample
        taken after the fact sees CONFIRMED again and proves nothing. ``last_passed``
        is set by the fault_manager on the FIRST PASSED report a fault ever receives
        and is never unset, so it catches a single spurious clear whether or not it
        ever reached HEALED.
        """
        before = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            before,
            f'{FAULT_CODE} is not in the store before the restart, so there is nothing '
            'a restart could wrongly heal',
        )
        self.assertIsNone(
            before.get('last_passed'),
            'the detector had already emitted a clear for this fault BEFORE the restart '
            f'(last_passed={before.get("last_passed")!r}), so the assertion after the '
            'restart could not attribute anything to it',
        )

        old_pid = gateway_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _wait_until_port_is_down(PORT, timeout=60.0 * TIME_SCALE),
            f'the gateway (pid {old_pid}) kept answering after SIGTERM - nothing was '
            'restarted, so the rest of this test would measure the original process',
        )

        # launch brings the same configuration back. Gate on the plugin being live and
        # globally armed again: a raise (or a clear) is only possible past that point,
        # so measuring before it would measure a stack that had not started detecting.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=90.0 * TIME_SCALE),
            'the gateway never came back armed after the restart',
        )
        new_pid = gateway_node.process_details['pid']
        self.assertNotEqual(
            new_pid, old_pid,
            'the gateway process id did not change, so this test never restarted anything',
        )

        # Long enough for the whole spurious-heal sequence to have played out: the
        # detector needs GRACE ticks to walk its fresh streak back up, and the
        # fault_manager needs healing_threshold (default 3) PASSED reports to heal.
        time.sleep(max(4.0, (GRACE + 8) * TICK_INTERVAL_MS / 1000.0))

        after = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            after,
            f'{FAULT_CODE} vanished from the store entirely after the restart',
        )
        self.assertIsNone(
            after.get('last_passed'),
            'the restarted gateway reported GRAPH_NODE_INACTIVE as PASSED while '
            f'{TARGET_NODE} was still inactive (last_passed='
            f'{after.get("last_passed")!r}) - the restart healed a fault that is still '
            'real',
        )
        self.assertNotEqual(
            after.get('status'), 'healed',
            f'{FAULT_CODE} is HEALED while {TARGET_NODE} has never left "inactive"',
        )

        # And the fault is still on the operator-visible active list, i.e. the withhold
        # preserved it rather than merely delaying the damage.
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE),
            f'{FAULT_CODE} is no longer an active fault after the gateway restart',
        )

    def test_03_activated_arms_the_gate_and_heals(self):
        # "Cleared" is an absence, and it is the default state of a fault that was never
        # raised: on a stack test_01 just failed against, poll_cleared answers True
        # immediately. Confirm the fault is actually THERE first - BEFORE the activate,
        # because after it the heal is racing this check - so what poll_cleared measures
        # is a heal of a proven-present fault (same pairing as
        # test_param_drift_e2e.test.py's heal leg).
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE),
            f'{FAULT_CODE} is not active at the start of the heal test, so there is '
            'nothing here that could heal and the clear below would pass without '
            'measuring anything',
        )

        self._call_transition(Transition.TRANSITION_ACTIVATE, reached_label='active')

        # Now - and only now - the per-entity armed gate is reachable: the entity leaves
        # 'warming_up' exactly when the watcher has read "active" for it. Waiting on it
        # splits the two ways the heal could fail: if the REAL lifecycle machinery never
        # fed the label to the gate, the failure is here and names the node; if the
        # label arrived but the clear never flowed, poll_cleared below is the one that
        # fails.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE, app_id=TARGET_NODE),
            f'the gate never armed {TARGET_NODE} after a successful ACTIVATE - the '
            'live lifecycle machinery (transition_event / GetState) never delivered '
            'the "active" label to the watcher',
        )

        cleared = poll_cleared(PORT, FAULT_CODE, timeout=60.0 * TIME_SCALE)
        self.assertTrue(
            cleared,
            f'{FAULT_CODE} did not heal after {TARGET_NODE} reached "active"',
        )

        # The clear above does not by itself prove the READ caused it: a fixture exit
        # right after the "active" observation would clear the same fault through the
        # absence path (AbsenceAfterRaiseClearsNotNodeDeathsDomain's own mechanism, at
        # the tracker tier), which would make this heal test pass for the wrong reason.
        # Pin that the node is still present and reading "active" once the clear was
        # observed.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'active', timeout=15.0 * TIME_SCALE),
            f'{TARGET_NODE} is no longer present and reading "active" once the clear was '
            'observed - the clear could have come from the absence path (a fixture exit) '
            'rather than the ACTIVATE that this test is actually about',
        )


class TestLifecycleExpectationDefaultConfig(unittest.TestCase):
    """The shipped default (no config at all) raises nothing for an inactive node."""

    def test_default_config_stays_silent(self):
        # Gate BEFORE asserting absence: a stack that never came up produces exactly the
        # same "no fault" result, and every bringup failure mode on this launch still
        # exits 0 (see the harness docstring).
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load or its tick loop never ran, so an absent fault proves nothing',
        )
        # The armed gate is served by the plugin inside the gateway process, so it says
        # nothing about the surface the assertion below actually reads. A launch whose
        # fault_manager died, hung, or never DDS-matched answers GET /faults with 503,
        # poll_faults swallows that and returns None, and the absence assertion passes
        # for the one reason it must never pass.
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so an absent fault proves nothing about the '
            'detector',
        )
        # Prove the false-positive trigger is fully present: the stack discovered the
        # node AND read its non-active label. Without this the silence below could just
        # as well mean the label never arrived.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=30.0 * TIME_SCALE),
            f'the gate never reported {TARGET_NODE} with lifecycle "unconfigured" - '
            'the node was not discovered or its label was never read, so the silence '
            'below would be vacuous',
        )
        # Proving absence means waiting out the full window, with the channel proven
        # alive for every poll across it - not merely a poll that timed out silently
        # (see assert_fault_absent_throughout's own docstring for why poll_faults +
        # assertIsNone cannot tell "no such fault" from "could not ask"): with no
        # detectors.lifecycle_expectation config, require_active is empty and the
        # detector must check nothing - the README's zero-false-positive default.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, SILENT_WINDOW_SEC)
        # The trigger was pinned at the START of the window; nothing observed it after
        # that. A demo node that exits 0 two seconds in is tolerated by the exit-code
        # check and invisible to the tracker (a departed node is the presence class's
        # problem), which would leave 2 s of trigger and 18 s of empty graph.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=15.0 * TIME_SCALE),
            f'{TARGET_NODE} is no longer reported as "unconfigured" at the END of the '
            'silence window - the trigger did not survive it, so most of the window '
            'proved nothing',
        )

    def test_silence_proof_catches_a_dead_fault_surface(self):
        """The test of the test.

        `assert_fault_absent_throughout` must fail when the channel it watches dies
        mid-window - the exact hole `assertIsNone(poll_faults(...))` could not see (see
        the harness docstring and `test_default_config_stays_silent` above, which now
        uses the fixed helper). Self-contained - a local HTTP server stands in for
        `/faults`, not this launch's real gateway - so it runs alongside the real
        assertion above without depending on it, and needs no ROS graph of its own.
        """
        prove_silence_proof_catches_a_dead_fault_surface(self)


class TestLifecycleExpectationNegativeControl(unittest.TestCase):
    """A required node that IS active never raises."""

    def test_active_required_node_never_raises(self):
        # Gate on THIS app being armed: for an active node the per-entity state is
        # reachable, and it is the strongest form of the arming gate - it proves the
        # .so loaded, the tick loop ran, the node was discovered, its warmup elapsed,
        # and the watcher considers it ok.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE, app_id=ACTIVE_NODE),
            f'graph_watchdog never reported {ACTIVE_NODE} as an armed entity - the '
            'plugin did not load, its tick loop never ran, or the gateway never '
            'discovered the node, so an absent fault proves nothing',
        )
        # And that the surface the assertion reads is alive in THIS launch: this is the
        # scenario whose job is "a buggy detector raising for an active node must be
        # caught", and a raise that is emitted but lost - fault_manager dead, hung, or
        # never DDS-matched - leaves GET /faults answering 503, which poll_faults
        # swallows into exactly the None the assertion below wants.
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so a wrong raise would have been lost rather '
            'than caught',
        )
        # Armed alone is not enough for THIS absence: node_ok also holds for an unread
        # label, and the tracker treats unread as benign - a run whose label never
        # arrived would stay silent for the wrong reason. Pin that "active" was READ.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, ACTIVE_NODE, 'active', timeout=30.0 * TIME_SCALE),
            f'the gate never reported {ACTIVE_NODE} with lifecycle "active" - its '
            'label was never read, so the silence below would be vacuous',
        )
        # Same grace and cadence as the "main" scenario, so the only variable between
        # the launch that must raise and this one is the node's actual lifecycle state.
        # The channel is proven alive for every poll across the window (see
        # assert_fault_absent_throughout's own docstring), not just a poll that timed
        # out silently.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, SILENT_WINDOW_SEC)
        # The required node must still be there, and still active, at the END of the
        # window: a node that exited early leaves an empty graph, which is silent for a
        # reason that has nothing to do with the detector being right.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, ACTIVE_NODE, 'active', timeout=15.0 * TIME_SCALE),
            f'{ACTIVE_NODE} is no longer reported as "active" at the END of the silence '
            'window - the required node did not survive it, so most of the window was '
            'not exercising an active node at all',
        )


class TestLifecycleExpectationUnreadable(unittest.TestCase):
    """GRAPH_NODE_UNREADABLE raises for a node whose lifecycle state is never read.

    Unlike every scenario above - which all drive ``managed_lifecycle_node.cpp``, a REAL
    ``rclcpp_lifecycle::LifecycleNode`` that always answers GetState - this launches
    ``unreadable_lifecycle_node.cpp``: a fixture that advertises ``get_state``/
    ``change_state`` with the right service TYPES (what ``find_lifecycle_get_state_path``
    actually checks - see that fixture's file doc) but whose ``get_state`` never responds
    until told to. That is the only way to reach GRAPH_NODE_UNREADABLE's raise against a
    REAL, sustained GetState failure: nothing else in this package can hold a real node
    unreadable for 60 consecutive ticks without either racing DDS timing or a demo
    executable built exactly for this.

    Proves, in order, the four facts the split between GRAPH_NODE_INACTIVE and
    GRAPH_NODE_UNREADABLE exists to keep separate:

    1. The required node is present and matched, and its lifecycle state has never been
       read (test_01) - asserted BEFORE anything else, or the rest proves nothing.
    2. GRAPH_NODE_INACTIVE is NOT raised for it (test_02) - not merely "not yet": an
       unread label feeds the unmeasured clock, never the violation streak
       (lifecycle_expectation_tracker.hpp), so this is impossible by construction, and
       this poll is what proves that live rather than trusting the source.
    3. GRAPH_NODE_UNREADABLE IS raised once the 60-tick hold expires, names the node, and
       carries SEVERITY_WARN on the stored record (test_03) - and GRAPH_NODE_INACTIVE is
       STILL silent at that same moment: a node is content of at most one of the two,
       never both.
    4. Once the fixture starts answering "active", the watcher reads it and
       GRAPH_NODE_UNREADABLE clears (test_04) - the real GetState round trip, not a
       label injected through a test seam.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('lifecycle_expectation_unreadable_e2e_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_01_present_and_unread_before_anything_else(self):
        # Gate on the plugin being live and globally armed before anything else - a
        # bringup failure must die here, naming itself, instead of as a raise-poll
        # timeout that blames the detector.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load, its tick loop never ran, or the bringup grace never elapsed, so no '
            'assertion below could mean anything',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so a wrong raise would have been lost rather '
            'than caught',
        )

        # The fact this whole scenario stands on, asserted first: the node is present,
        # matched by the require_active entry, and its lifecycle label reads exactly ""
        # - LifecycleWatcher seeded it once and nothing has answered since (see
        # _poll_watchdog_entity's docstring for why "" is not "no data yet").
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNREADABLE_NODE, '', timeout=30.0 * TIME_SCALE),
            f'the gate never reported {UNREADABLE_NODE} with an empty (never-read) '
            'lifecycle label - the fixture was not discovered as a managed node, or its '
            'GetState answered when it must not have, so nothing below would prove '
            'anything about an UNREADABLE node',
        )

    def test_02_confirmed_inactive_is_never_raised_for_an_unread_node(self):
        # GRAPH_NODE_INACTIVE must stay silent for the whole window, with the channel
        # proven alive for every poll across it (see assert_fault_absent_throughout's own
        # docstring): an unread label can never become a "confirmed violation" (see the
        # class docstring). The ticks this window spends also count toward the 60-tick
        # UNREADABLE hold in the background, so nothing here is wasted time.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, UNREADABLE_INACTIVE_SILENCE_SEC)
        # And the node must still be genuinely unread at the END of the window, or the
        # silence above measured an empty graph rather than a real unreadable node.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNREADABLE_NODE, '', timeout=15.0 * TIME_SCALE),
            f'{UNREADABLE_NODE} no longer reads an empty (never-read) lifecycle label '
            'at the end of the silence window - the trigger did not survive it, so '
            'most of the window proved nothing',
        )

    def test_03_unreadable_raises_naming_the_node_at_warn_severity(self):
        fault = poll_faults(PORT, FAULT_CODE_UNREADABLE, timeout=UNREADABLE_RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE_UNREADABLE} never raised while {UNREADABLE_NODE} stayed '
            'unreadable past the 60-tick hold',
        )
        self.assertIn(UNREADABLE_NODE, fault.get('description', ''))
        self.assertEqual(
            fault.get('severity_label'), 'WARN',
            f'{FAULT_CODE_UNREADABLE} must carry SEVERITY_WARN on the stored record '
            f'(a merely UNVERIFIED promise, not a confirmed violation), got '
            f'{fault.get("severity_label")!r}',
        )

        # Still true at the exact moment UNREADABLE is raised: a node is content of at
        # most one of the two codes, never both. Proven across a whole window with the
        # channel checked on every poll, not by a single poll that timed out silently -
        # see assert_fault_absent_throughout's own docstring.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, MUTUAL_EXCLUSION_WINDOW_SEC)

        # The flat /faults list carries a fault whatever its source is; only the
        # entity-scoped surface proves an operator can OPEN it somewhere (see
        # TestLifecycleExpectationMain.test_01's identical rationale for the sibling code).
        self.assertIsNotNone(
            poll_entity_faults(PORT, 'apps/graph_watchdog', FAULT_CODE_UNREADABLE,
                               timeout=30.0 * TIME_SCALE),
            f'{FAULT_CODE_UNREADABLE} is not reachable at /apps/graph_watchdog/faults - '
            'the entity the plugin publishes does not own the fault it raises',
        )

    def test_04_answering_active_clears_it(self):
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE_UNREADABLE, timeout=30.0 * TIME_SCALE),
            f'{FAULT_CODE_UNREADABLE} is not active at the start of the heal test, so '
            'there is nothing here that could heal and the clear below would pass '
            'without measuring anything',
        )

        self.assertTrue(
            _set_bool_parameter(
                type(self)._client_node, f'/{UNREADABLE_NODE}/set_parameters',
                'start_answering', True, timeout=30.0 * TIME_SCALE),
            f'setting start_answering:=true on /{UNREADABLE_NODE}/set_parameters '
            'never succeeded - the fixture never got the signal to start answering '
            'GetState, so no heal below could mean anything',
        )

        # Proves the REAL GetState round trip fed the watcher - not merely that the
        # parameter was accepted. Only now is the per-entity armed gate meaningful for
        # this node: node_ok() was already true while unread (an unread label defaults
        # open, see lifecycle_watcher.cpp), so this is the read itself, not the gate.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNREADABLE_NODE, 'active', timeout=30.0 * TIME_SCALE),
            f'the gate never reported {UNREADABLE_NODE} with lifecycle "active" after '
            'start_answering:=true - the fixture accepted the parameter but its '
            'GetState still never delivered "active" to the watcher',
        )

        cleared = poll_cleared(PORT, FAULT_CODE_UNREADABLE, timeout=30.0 * TIME_SCALE)
        self.assertTrue(
            cleared,
            f'{FAULT_CODE_UNREADABLE} did not heal after {UNREADABLE_NODE} started '
            'answering "active"',
        )


class TestLifecycleExpectationHealingThreshold(unittest.TestCase):
    """healing_threshold=1 must not let repeated snapshot blinks heal a still-violating fault.

    Reproduces the exact mechanism the design doc describes: a required node stuck
    inactive is raised, then blinks out of the graph - a real SIGTERM + relaunch of the
    SAME node under the same name, not a lifecycle transition - twice, each blink well
    inside the tracker's own absence grace (a fixed 3 ticks, not configurable). Before
    this slice's fix, every absent tick emitted a spurious PASSED regardless of grace,
    which at a sensitive healing_threshold of 1 (the value this package's own README
    recommends) is enough to walk a still-violating fault to HEALED.

    The instrument is ``last_passed`` (same as test_02b's restart leg above) and the
    fault's ``status``, both read from the real ``GET /api/v1/faults`` surface - not
    whether a PASSED reached some intercepted service. This is the one tier that runs
    the REAL fault_manager debounce state machine at all: nothing below e2e ever touches
    the actual debounce counter, only a fake ReportFault sink.

    The same run also carries this package's ONLY e2e coverage of a node oscillating
    readable/unreadable producing no fault EVENT churn: each blink's respawn window
    briefly re-seeds the required node's lifecycle label (present, but unread) before it
    settles back to "unconfigured". GRAPH_NODE_INACTIVE carries a fixed SEVERITY_ERROR
    (no per-tick severity choice on this fault - each of GRAPH_NODE_INACTIVE and its
    sibling GRAPH_NODE_UNREADABLE is now a separate, fixed-severity record, see the
    design doc), so a severity regression would still show up here. ``GET /faults/stream``
    is read raw for the whole blink sequence and checked for two things a poll of
    ``GET /faults`` cannot guarantee it would catch: no ``fault_cleared`` frame for
    GRAPH_NODE_INACTIVE at any point (the raw-event twin of the ``last_passed`` check),
    and every ``fault_confirmed``/``fault_updated`` frame for GRAPH_NODE_INACTIVE carrying
    ``severity_label: "ERROR"`` (its only possible value now). This run's blinks are far
    too short to ever convert the required node into GRAPH_NODE_UNREADABLE content (that
    needs 60 consecutive matched ticks, not a few seconds of respawn), so no GRAPH_NODE_UNREADABLE
    frame is expected on this stream either. Reaching GRAPH_NODE_UNREADABLE's own raise
    through a real, sustained (60-tick) GetState failure is not this scenario's job: the
    demo node used here (``managed_lifecycle``) answers GetState as soon as it is
    discovered. See the "unreadable" scenario in this same file
    (``TestLifecycleExpectationUnreadable``) for that proof, driven by
    ``unreadable_lifecycle_node.cpp``'s deferred-response fixture; the integration tier's
    ``ManagedNodeWhoseGetStateNeverAnswersIsReportedUnreadableNotInactive`` and the
    injection-seam tests beside it still own the fake-ReportFault-sink version of the same
    claim.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def _blink(self, node_action, blink_number):
        """SIGTERM the current target-node process, then confirm it came back stuck.

        ``respawn=True`` on the launch action brings a brand-new process back under the
        same ROS node name after ``HEALING_RESPAWN_DELAY_SEC``, so the node returns
        exactly as unconfigured (still violating) as it left - only its identity as a
        DDS participant changed. SIGTERM, not SIGKILL, so the exit code the eventual
        gateway-teardown check sees stays inside ``ALLOWED_EXIT_CODES`` (same reason
        test_02b restarts the gateway with SIGTERM rather than SIGKILL).
        """
        old_pid = node_action.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)

        # This scenario is named for a departure that stays INSIDE the tracker's
        # absence grace (kDefaultAbsenceGrace, 3 ticks - fixed, not configurable) - not
        # merely "a respawn happened eventually". Sample GET /apps while the respawn
        # settles so the blink is PROVEN absent, rather than assumed from
        # HEALING_BLINK_SLEEP_SEC's own comment alone.
        absence_started = time.monotonic()
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=HEALING_BLINK_SLEEP_SEC),
            f'blink {blink_number}: {TARGET_NODE} was never observed absent from GET /apps '
            f'within {HEALING_BLINK_SLEEP_SEC}s of the SIGTERM - this blink may not have '
            'exercised a real departure at all, so nothing below proves the absence grace '
            'held',
        )
        # The return has to be proven from the DISCOVERY view, not from the watchdog's
        # entity list: ReliabilityGate::status_json builds its entities from
        # WarmupTracker::entries(), which RETAINS an entry for forget_grace_ ticks after
        # the node stops being present. Polling the watchdog for "unconfigured" therefore
        # matches the entry left over from before the SIGTERM and returns while the
        # replacement process is milliseconds old - so everything measured from it,
        # including the absence duration below, describes a node the tracker never saw
        # leave. GET /apps carries no such retention.
        self.assertTrue(
            _poll_apps_present(PORT, TARGET_NODE, timeout=15.0 * TIME_SCALE),
            f'blink {blink_number}: {TARGET_NODE} never came back into GET /apps - the '
            'respawn or its rediscovery did not complete, so this blink cannot be trusted '
            'to have stayed inside the absence grace',
        )
        absence_duration = time.monotonic() - absence_started
        # kDefaultAbsenceGrace ticks at this scenario's own (sped-up) cadence - the exact
        # budget the withheld-clear guard's absence leg holds for. A blink that overran
        # it would be exercising the ordinary (and separately proven) sustained-absence
        # clear instead of the guard this scenario is actually about.
        grace_budget_sec = (HEALING_ABSENCE_GRACE_TICKS * HEALING_TICK_INTERVAL_MS) / 1000.0
        tick_sec = HEALING_TICK_INTERVAL_MS / 1000.0
        # The lower edge of the window, and the one that used to be missing entirely. The
        # tracker only counts an absent TICK; an absence shorter than one tick interval can
        # fall wholly between two samples, in which case the node was never observed away
        # and this blink exercised nothing at all. An absence longer than one interval must
        # contain at least one sample.
        self.assertGreater(
            absence_duration, tick_sec,
            f'blink {blink_number}: {TARGET_NODE} was away for only {absence_duration:.2f}s, '
            f'less than the {tick_sec:.1f}s tick interval - the tracker can have sampled it '
            'as present on every tick, so this blink does not exercise the absence grace',
        )
        self.assertLess(
            absence_duration, grace_budget_sec,
            f'blink {blink_number}: {TARGET_NODE} took {absence_duration:.2f}s to return - '
            f'past the {grace_budget_sec:.1f}s absence-grace budget this scenario depends '
            'on staying inside, so this blink no longer proves the withheld-clear guard, '
            'only the ordinary sustained-absence clear',
        )
        # Polled, not read once: launch updates process_details from its own event loop, so
        # a single read here races the respawn it is meant to confirm - the gateway can see
        # the replacement through DDS before launch has recorded it.
        pid_deadline = time.monotonic() + 10.0
        new_pid = node_action.process_details['pid']
        while new_pid == old_pid and time.monotonic() < pid_deadline:
            time.sleep(0.1)
            new_pid = node_action.process_details['pid']
        self.assertNotEqual(
            new_pid, old_pid,
            f'blink {blink_number}: the node process id did not change, so nothing '
            'actually blinked',
        )

    def test_repeated_blinks_do_not_heal_a_still_violating_fault(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state for the '
            'healing_threshold scenario - no assertion below could mean anything',
        )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=30.0 * TIME_SCALE),
            f'{TARGET_NODE} never read "unconfigured" - the false-positive trigger this '
            'test needs was never actually present',
        )

        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0 * TIME_SCALE)
        self.assertIsNotNone(fault, f'{FAULT_CODE} never raised for the stuck {TARGET_NODE}')
        before = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(before, f'{FAULT_CODE} is not in the store before the blinks')
        self.assertIsNone(
            before.get('last_passed'),
            f'{FAULT_CODE} already had a PASSED on record before any blink '
            f'(last_passed={before.get("last_passed")!r}), so the assertions below could '
            'not attribute anything to the blinks below',
        )

        # Row-6 instrument: GET /faults/stream, opened BEFORE the blinks and read raw for
        # their whole duration. Polling GET /faults (above and below) can step over a
        # transient frame between two samples; the SSE stream cannot - every event the
        # fault_manager published is either in the replay buffer or was delivered to this
        # live connection. Each blink is a real node oscillating readable (the required
        # node's own lifecycle label, unrelated to THIS fault) between present and
        # absent/re-seeding, and GRAPH_NODE_INACTIVE is emitted every tick from a
        # class-scoped, fixed-severity AggregatedFault member (see the class docstring
        # above) - so this is where the level-triggered re-send racing the blink, or a
        # stray event this fixed-severity design should make impossible, would show up as
        # an extra or wrong event.
        base = f'http://127.0.0.1:{PORT}{API_BASE_PATH}'
        sse_frames = []
        sse_stop = threading.Event()
        sse_response = requests.get(f'{base}/faults/stream', stream=True, timeout=(5, 120))
        self.assertEqual(sse_response.status_code, 200)
        sse_pump = threading.Thread(
            target=_pump_sse_stream, args=(sse_response, sse_frames, sse_stop), daemon=True)
        sse_pump.start()
        try:
            for i in range(1, HEALING_BLINK_COUNT + 1):
                frames_before_blink = len(sse_frames)
                self._blink(target_node, i)
                # A stream that silently died (connection still open, nothing being
                # delivered - the status-code check at open time above cannot catch
                # that) would let every "no fault_cleared frame" assertion below pass
                # on zero evidence. Prove the reader was alive THROUGH this blink, not
                # merely that it connected before the first one.
                self.assertTrue(
                    sse_pump.is_alive(),
                    f'the SSE reader thread died during blink {i} - a dead stream would '
                    'let every assertion below pass without having observed anything',
                )
                # And that it actually delivered something across the blink: GRAPH_NODE_INACTIVE
                # is emitted every tick this fault stays raised (see the class docstring
                # above), so a real blink at this scenario's 1 s tick cadence must produce
                # at least one fresh frame - a settle window here (not merely at the very
                # end) is what lets THIS blink's own frame(s) be observed before moving on.
                time.sleep(2.0)
                self.assertGreater(
                    len(sse_frames), frames_before_blink,
                    f'no SSE frame arrived during or after blink {i} - the stream may have '
                    'died silently (still connected, nothing delivered), which the '
                    'status-code check at open time cannot catch',
                )

            # A settle window past the last blink: any spurious PASSED the blinks produced
            # has long since reached the fault_manager and been folded into the debounce
            # counter by now, and any SSE frame it would have produced has been pumped.
            time.sleep(3.0)
        finally:
            sse_stop.set()
            sse_response.close()
            sse_pump.join(timeout=5)

        after = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            after, f'{FAULT_CODE} vanished from the store entirely after the blinks')
        self.assertIsNone(
            after.get('last_passed'),
            f'a blink produced a spurious PASSED for {FAULT_CODE} while {TARGET_NODE} was '
            f'still stuck inactive (last_passed={after.get("last_passed")!r}) - the '
            "withheld-clear guard's pending leg did not hold through the absence grace",
        )
        self.assertNotEqual(
            after.get('status'), 'healed',
            f'{FAULT_CODE} healed at healing_threshold=1 while {TARGET_NODE} never left '
            'its stuck lifecycle state - a debounce counter that should never have moved',
        )

        # And the node is still genuinely stuck, so the outcome above measured something
        # real rather than an empty graph.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=15.0 * TIME_SCALE),
            f'{TARGET_NODE} is no longer reading "unconfigured" at the end of the test - '
            'the trigger did not survive the blinks, so the assertions above proved '
            'nothing',
        )

        # Row 6: no fault_cleared frame for FAULT_CODE reached the stream across the
        # whole blink sequence - the raw-event twin of the last_passed check above, at
        # the one tier that cannot miss a transient frame between two polls.
        cleared_frames = [
            json.loads(f['data']) for f in sse_frames
            if f.get('event') == 'fault_cleared' and 'data' in f
        ]
        cleared_frames = [
            p for p in cleared_frames if p.get('fault', {}).get('fault_code') == FAULT_CODE]
        self.assertEqual(
            cleared_frames, [],
            f'a fault_cleared event for {FAULT_CODE} reached /faults/stream during the '
            f'blinks - the withheld-clear guard let a spurious clear through, and unlike '
            f'the last_passed check above this instrument cannot have missed it: {cleared_frames}',
        )

        # Every fault_confirmed/fault_updated frame for FAULT_CODE across the run must
        # carry the confirmed severity (ERROR), never the unreadable one (WARN). This is
        # not merely improbable given this scenario's timing (the required node never
        # stays unreadable the 60 consecutive matched ticks GRAPH_NODE_UNREADABLE needs to
        # convert) - it is impossible IN PRINCIPLE for FAULT_CODE specifically, since
        # GRAPH_NODE_INACTIVE is now a class-scoped, fixed-severity AggregatedFault member
        # (SEVERITY_ERROR always; see the class docstring above and "Two independent
        # faults, not one shared record" in the design doc): there is no per-tick choice
        # left anywhere in the code that could produce WARN on this fault_code, only on
        # its sibling GRAPH_NODE_UNREADABLE's own, separate record. A WARN here would mean
        # the two records' content had been cross-wired, not that some tick's timing went
        # wrong.
        content_frames = [
            json.loads(f['data']) for f in sse_frames
            if f.get('event') in ('fault_confirmed', 'fault_updated') and 'data' in f
        ]
        content_frames = [
            p for p in content_frames if p.get('fault', {}).get('fault_code') == FAULT_CODE]
        self.assertTrue(
            content_frames,
            'no fault_confirmed/fault_updated frame for {FAULT_CODE} reached the stream at '
            f'all - the SSE instrument saw nothing, so the severity check below would prove '
            f'nothing; frames seen: {sse_frames}',
        )
        wrong_severity = [
            p['fault'].get('severity_label') for p in content_frames
            if p['fault'].get('severity_label') != 'ERROR'
        ]
        self.assertEqual(
            wrong_severity, [],
            f'{FAULT_CODE} carried a severity other than ERROR on at least one tick - it is a '
            f'fixed-severity fault (SEVERITY_ERROR, see AggregatedFault in the design doc), so '
            f'nothing on any tick can make it carry anything else: {wrong_severity}',
        )


class TestLifecycleExpectationDepartureKeeps(unittest.TestCase):
    """GRAPH_NODE_UNREADABLE SURVIVES an already-raised node leaving the graph.

    The evidence-retention half of the model, at the tier that runs the real
    fault_manager: a node whose lifecycle promise was never verified does not become
    verified by leaving. A sibling launch to TestLifecycleExpectationUnreadable above,
    not an extension of it: that class's test_04 proves the clear-by-READ path by
    flipping unreadable_lifecycle_node.cpp's start_answering parameter, and unittest
    runs a class's test methods, in one gateway process, in alphabetical order. A test
    that permanently kills the fixture (no respawn - the whole point here is a real,
    sustained departure, never a bounce back) cannot share that process: it would
    either race test_04's assumption that the fixture is still alive, or - if ordered
    to run first - leave the fixture dead before test_04 ever gets to set the
    parameter. So this launches its own gateway + fault_manager +
    unreadable_lifecycle_node.cpp stack, holding a handle to the fixture's own
    launch_ros.actions.Node action (``context['target_node']``, the same shape
    TestLifecycleExpectationHealingThreshold uses for its own SIGTERM'able node) -
    create_demo_nodes() gives no such handle back.

    Proves, in order, against the real stack through GET /api/v1/faults:

    1. The same starting state TestLifecycleExpectationUnreadable reaches: the fixture
       present, matched, and unread (test_01), then GRAPH_NODE_UNREADABLE actually
       raised past the 60-tick hold (test_02) - asserted before anything else, or the
       rest would not be about an ALREADY-RAISED fault at all.
    2. The fixture process SIGTERM'd, and confirmed GONE from the operator-visible
       graph via GET /apps - the same ThreadSafeEntityCache the detector's own
       per-tick snapshot reads (see _poll_apps_absent's docstring) - before anything is
       asserted about the fault (test_03). Without this gate the fault still being
       there could not be told apart from a graph the entity cache had not caught up
       with yet.
    3. After a settle window longer than every horizon that could have discarded the
       node's evidence, the fault is STILL active, has never once been reported PASSED
       (``last_passed``, the instrument that catches even a transient clear), and its
       description now says the node has left the graph rather than continuing to
       describe a present-but-unread one. GRAPH_NODE_INACTIVE stays silent throughout:
       an unread label can never become CONFIRMED content, before or after the kill.

    The integration tier's own proof of this claim,
    ``UnreadableNodeAlreadyReportedThatVanishesKeepsItsOwnRecord``, drives the detector
    directly against a fake ReportFault service and a hand-built snapshot; it cannot
    prove the real discovery layer ever notices a process leaving DDS, or that the real
    fault_manager's debounce state machine never receives a PASSED. This closes that gap.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_01_present_and_unread_before_anything_else(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load, its tick loop never ran, or the bringup grace never elapsed, so no '
            'assertion below could mean anything',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so a wrong raise would have been lost rather '
            'than caught',
        )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNREADABLE_NODE, '', timeout=30.0 * TIME_SCALE),
            f'the gate never reported {UNREADABLE_NODE} with an empty (never-read) '
            'lifecycle label - the fixture was not discovered as a managed node, or its '
            'GetState answered when it must not have, so nothing below would prove '
            'anything about this clear path',
        )

    def test_02_unreadable_raises_before_the_kill(self):
        fault = poll_faults(PORT, FAULT_CODE_UNREADABLE, timeout=UNREADABLE_RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE_UNREADABLE} never raised while {UNREADABLE_NODE} stayed '
            'unreadable past the 60-tick hold - there is nothing ALREADY RAISED for the '
            'kill below to clear',
        )
        self.assertIn(UNREADABLE_NODE, fault.get('description', ''))
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, MUTUAL_EXCLUSION_WINDOW_SEC)

    def test_03_killed_node_confirmed_gone_and_the_fault_survives_it(self, target_node):
        old_pid = target_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)

        self.assertTrue(
            _poll_apps_absent(PORT, UNREADABLE_NODE, timeout=ABSENCE_DEPARTURE_TIMEOUT_SEC),
            f'{UNREADABLE_NODE} (pid {old_pid}) is still listed on GET /apps after '
            'SIGTERM - it never actually left the graph, so nothing below would prove '
            'anything about a DEPARTED node',
        )

        # Well past the absence grace (3 ticks at this scenario's 100 ms cadence) and past
        # the detector's own 60-tick hold, so a mechanism that discarded the node's
        # evidence on absence has had every opportunity to do so.
        time.sleep(DEPARTURE_SETTLE_SEC)

        record = _fault_record(PORT, FAULT_CODE_UNREADABLE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            record,
            f'{FAULT_CODE_UNREADABLE} vanished from the store entirely after '
            f'{UNREADABLE_NODE} left the graph',
        )
        self.assertIsNone(
            record.get('last_passed'),
            f'{FAULT_CODE_UNREADABLE} was reported PASSED once {UNREADABLE_NODE} left the '
            f'graph (last_passed={record.get("last_passed")!r}) - the node\'s lifecycle '
            'promise is no more verified now than it was while the node was present, so '
            'its departure must not heal the fault',
        )
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE_UNREADABLE, timeout=ABSENCE_CLEAR_TIMEOUT_SEC),
            f'{FAULT_CODE_UNREADABLE} is no longer an active fault once {UNREADABLE_NODE} '
            'stayed gone - a departure erased the evidence against it',
        )

        # And the description now says the node is gone, rather than continuing to
        # describe a graph it left.
        self.assertIn(
            'has since left the graph',
            _fault_record(PORT, FAULT_CODE_UNREADABLE,
                          timeout=10.0 * TIME_SCALE).get('description', ''),
            f'{FAULT_CODE_UNREADABLE} still describes {UNREADABLE_NODE} as a present '
            'node whose state cannot be read, after it left the graph',
        )

        # GRAPH_NODE_INACTIVE stays silent throughout: the node was never read at any point
        # in this whole scenario, so it has no path to CONFIRMED content whether it is in
        # the graph or not.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, MUTUAL_EXCLUSION_WINDOW_SEC)


class TestLifecycleExpectationNotManaged(unittest.TestCase):
    """GRAPH_NODE_NOT_MANAGED raises for a required node with no tracked lifecycle at all.

    The sibling of TestLifecycleExpectationUnreadable/TestLifecycleExpectationDepartureKeeps,
    proving the OTHER "I cannot measure this" cause the unmeasured clock this slice adds is
    blind to: unlike ``unreadable_lifecycle_node.cpp`` (which advertises GetState/ChangeState
    but never answers), ``calibration`` (``DEMO_NODE_REGISTRY``'s plain
    ``demo_calibration_service``) advertises no lifecycle interface whatsoever, so
    ``lifecycle_state_of()`` reads ``nullopt`` for it from the moment it is discovered - no
    purpose-built fixture was needed for this cause, unlike UNREADABLE's.

    Proves, in order, against the real stack through GET /api/v1/faults:

    1. The required node is present and matched, and it carries NO tracked lifecycle at all
       (test_01) - via GET /apps rather than the watchdog status route's `lifecycle` field,
       since a never-tracked node's own presence is the fact under test here, not a label.
    2. GRAPH_NODE_NOT_MANAGED raises once the 60-tick hold expires, names the node, and
       carries SEVERITY_WARN - and neither GRAPH_NODE_INACTIVE nor GRAPH_NODE_UNREADABLE
       ever raises for it: a node is content of at most one of the three (test_02).
    3. The node leaving the graph and staying away does NOT clear
       GRAPH_NODE_NOT_MANAGED - the same evidence retention the unreadable sibling
       gets, proven independently rather than assumed from sharing one clock (test_03).
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_01_present_and_not_managed_before_anything_else(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load, its tick loop never ran, or the bringup grace never elapsed, so no '
            'assertion below could mean anything',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so a wrong raise would have been lost rather '
            'than caught',
        )
        # The fact this whole scenario stands on: the node is present in the graph (a
        # plain service node, no lifecycle interface) - proven via the same ThreadSafeEntityCache
        # the detector's own per-tick snapshot reads (see _poll_apps_present's docstring),
        # not the watchdog status route's `lifecycle` field, since "never tracked" is a
        # statement about ABSENCE of tracking, which GET /apps + the raise below prove
        # together more directly than a nullable JSON field would.
        self.assertTrue(
            _poll_apps_present(PORT, NOT_MANAGED_NODE, timeout=30.0 * TIME_SCALE),
            f'{NOT_MANAGED_NODE} never appeared on GET /apps - the fixture never came up, '
            'so nothing below would prove anything about NOT-MANAGED',
        )

    def test_02_not_managed_raises_naming_the_node_at_warn_severity(self):
        fault = poll_faults(PORT, FAULT_CODE_NOT_MANAGED, timeout=NOT_MANAGED_RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE_NOT_MANAGED} never raised while {NOT_MANAGED_NODE} stayed '
            'not-managed past the 60-tick hold',
        )
        self.assertIn(NOT_MANAGED_NODE, fault.get('description', ''))
        self.assertEqual(
            fault.get('severity_label'), 'WARN',
            f'{FAULT_CODE_NOT_MANAGED} must carry SEVERITY_WARN on the stored record '
            f'(an UNVERIFIED promise, not a confirmed violation), got '
            f'{fault.get("severity_label")!r}',
        )

        # A node is content of at most one of the three codes, never more than one - proven
        # across a window with the channel checked on every poll, not by a single poll that
        # timed out silently.
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, MUTUAL_EXCLUSION_WINDOW_SEC)
        assert_fault_absent_throughout(
            self, PORT, FAULT_CODE_UNREADABLE, MUTUAL_EXCLUSION_WINDOW_SEC)

        self.assertIsNotNone(
            poll_entity_faults(PORT, 'apps/graph_watchdog', FAULT_CODE_NOT_MANAGED,
                               timeout=30.0 * TIME_SCALE),
            f'{FAULT_CODE_NOT_MANAGED} is not reachable at /apps/graph_watchdog/faults - '
            'the entity the plugin publishes does not own the fault it raises',
        )

    def test_03_departed_node_confirmed_gone_and_the_fault_survives_it(self, target_node):
        fault = poll_faults(PORT, FAULT_CODE_NOT_MANAGED, timeout=NOT_MANAGED_RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE_NOT_MANAGED} is not active at the start of the departure leg, so '
            'there is nothing here for the departure below to preserve',
        )

        old_pid = target_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)

        self.assertTrue(
            _poll_apps_absent(PORT, NOT_MANAGED_NODE, timeout=NOT_MANAGED_DEPARTURE_TIMEOUT_SEC),
            f'{NOT_MANAGED_NODE} (pid {old_pid}) is still listed on GET /apps after '
            'SIGTERM - it never actually left the graph, so nothing below would prove '
            'anything about a DEPARTED node',
        )
        time.sleep(DEPARTURE_SETTLE_SEC)

        record = _fault_record(PORT, FAULT_CODE_NOT_MANAGED, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            record,
            f'{FAULT_CODE_NOT_MANAGED} vanished from the store entirely after '
            f'{NOT_MANAGED_NODE} left the graph',
        )
        self.assertIsNone(
            record.get('last_passed'),
            f'{FAULT_CODE_NOT_MANAGED} was reported PASSED once {NOT_MANAGED_NODE} left '
            f'the graph (last_passed={record.get("last_passed")!r}) - the node was '
            'required to be active, never was a managed lifecycle node at all, and is now '
            'gone, none of which is health',
        )
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE_NOT_MANAGED, timeout=NOT_MANAGED_CLEAR_TIMEOUT_SEC),
            f'{FAULT_CODE_NOT_MANAGED} is no longer an active fault once {NOT_MANAGED_NODE} '
            'stayed gone - a departure erased the evidence against it',
        )
        self.assertIn(
            'has since left the graph',
            _fault_record(PORT, FAULT_CODE_NOT_MANAGED,
                          timeout=10.0 * TIME_SCALE).get('description', ''),
            f'{FAULT_CODE_NOT_MANAGED} still describes {NOT_MANAGED_NODE} as a present '
            'node, after it left the graph',
        )
        assert_fault_absent_throughout(self, PORT, FAULT_CODE, MUTUAL_EXCLUSION_WINDOW_SEC)


class TestLifecycleExpectationRestartLoop(unittest.TestCase):
    """A required node in a CRASH LOOP is reported, not silent.

    The scenario the whole evidence-retention model exists for, on the real stack. A node
    that keeps dying and coming back touches absence periodically by construction, so any
    horizon that discards its accumulated evidence when it is absent makes exactly that
    node permanently invisible - and a crash-looping required node is the single case this
    detector most needs to catch.

    The loop is driven by real SIGTERMs against a real, respawning process, not by a
    hand-built snapshot: the node is left alive for RESTART_LOOP_UPTIME_SEC (at most ~25
    ticks at this scenario's 100 ms cadence, comfortably under the 60 consecutive
    matched ticks kUnmeasuredHoldTicks would otherwise need), then killed. Each absence
    is proven, not assumed: GET /apps - the same ThreadSafeEntityCache the detector's own
    per-tick snapshot reads - is polled until the node is gone and again until it is back,
    and the measured gap must exceed the tracker's 3-tick absence grace, on top of
    launch's own enforced respawn_delay floor. So every cycle genuinely crosses the
    horizon under test, and a detector that reset its clock there would stay silent for
    the whole window however many cycles it ran.

    `calibration` (a plain service node with no lifecycle interface at all) supplies the
    NOT-MANAGED cause, so no purpose-built fixture is needed and no blocking GetState
    round trip is paid per tick.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def _kill_and_wait_for_the_return(self, node_action, cycle):
        """SIGTERM the node, prove it left the graph past the absence grace, wait it back."""
        old_pid = node_action.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, NOT_MANAGED_NODE, timeout=30.0 * TIME_SCALE, interval=0.05),
            f'cycle {cycle}: {NOT_MANAGED_NODE} (pid {old_pid}) was never observed absent '
            'from GET /apps after the SIGTERM - this cycle never crossed the absence '
            'horizon it exists to cross',
        )
        absence_observed = time.monotonic()
        self.assertTrue(
            _poll_apps_present(PORT, NOT_MANAGED_NODE, timeout=30.0 * TIME_SCALE, interval=0.05),
            f'cycle {cycle}: {NOT_MANAGED_NODE} never came back after the SIGTERM - launch '
            'did not respawn it, so this is a single departure, not a restart loop',
        )
        # Measured between two observations of the SAME cache the detector reads, so this
        # is a real lower bound on how long the detector saw the node absent for.
        absent_for = time.monotonic() - absence_observed
        grace_budget_sec = (HEALING_ABSENCE_GRACE_TICKS * RESTART_LOOP_TICK_INTERVAL_MS) / 1000.0
        self.assertGreater(
            absent_for, grace_budget_sec,
            f'cycle {cycle}: {NOT_MANAGED_NODE} was only observed absent for '
            f'{absent_for:.2f}s, inside the {grace_budget_sec:.1f}s absence grace - this '
            'cycle was a blink, which was always tolerated, so it does not exercise the '
            'horizon past which evidence used to be discarded',
        )

    def test_crash_looping_node_is_reported(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load, its tick loop never ran, or the bringup grace never elapsed, so no '
            'assertion below could mean anything',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so a missing fault would prove nothing',
        )
        self.assertTrue(
            _poll_apps_present(PORT, NOT_MANAGED_NODE, timeout=30.0 * TIME_SCALE),
            f'{NOT_MANAGED_NODE} never appeared on GET /apps - the fixture never came up, '
            'so there is no node here to crash-loop',
        )

        # Keep killing until the fault has appeared AND the node has genuinely looped:
        # accepting the first sighting would let a single restart stand in for a loop, and
        # would make the cycle count depend on how fast this machine is. Written this way
        # the loop also proves the fault SURVIVES the restarts that follow it, not merely
        # that it appeared once.
        deadline = time.monotonic() + RESTART_LOOP_WINDOW_SEC
        cycles = 0
        fault = None
        while time.monotonic() < deadline:
            time.sleep(RESTART_LOOP_UPTIME_SEC)
            fault = poll_faults(PORT, FAULT_CODE_NOT_MANAGED, timeout=0.1, interval=0.1)
            if fault is not None and cycles >= RESTART_LOOP_MIN_CYCLES:
                break
            cycles += 1
            self._kill_and_wait_for_the_return(target_node, cycles)

        self.assertGreaterEqual(
            cycles, RESTART_LOOP_MIN_CYCLES,
            f'the node was only restarted {cycles} time(s) - too few for this to be a crash '
            'LOOP rather than a single departure, so the result below does not discriminate',
        )
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE_NOT_MANAGED} never raised for a required node that crash-looped '
            f'for {RESTART_LOOP_WINDOW_SEC}s across {cycles} restarts - every absence '
            'discarded the evidence it had accumulated while present, so the node this '
            'detector most exists to catch stays silent forever',
        )
        self.assertIn(NOT_MANAGED_NODE, fault.get('description', ''))
        self.assertEqual(
            fault.get('severity_label'), 'WARN',
            f'{FAULT_CODE_NOT_MANAGED} must carry SEVERITY_WARN on the stored record, got '
            f'{fault.get("severity_label")!r}',
        )


class TestLifecycleExpectationCapPressure(unittest.TestCase):
    """A FULL tracked-node cap must never turn into unreported health.

    The tracker keeps state for at most `tracked_node_cap` nodes. This launch sets it to
    ONE against TWO required nodes, so one of them is refused on every single tick -
    reachable at this tier only because the cap is a config key; against the compile-time
    512 it would need 513 real lifecycle nodes.

    Three separate claims, in the order the run reaches them:

    1. **The refusal is visible** (test_01). A required node is going unchecked, which is
       exactly the state a detector for silent faults must not be silent about. The
       gateway log is not readable from here; the detector's own status block on
       ``GET /x-medkit-watchdog`` is.
    2. **A refusal WITHHOLDS the GRAPH_NODE_INACTIVE clear** (test_02). The tracked node's
       lifecycle services are dropped, so its unmeasured clock matures, ownership passes to
       GRAPH_NODE_NOT_MANAGED and GRAPH_NODE_INACTIVE's own content goes empty - with
       nothing pending, which is precisely when the level-triggered emitter would clear. It
       must not: the detector declined to check the other required node, so it cannot assert
       that every required node is healthy. The instrument is ``last_passed``, which the
       fault_manager sets on the first PASSED a fault ever receives and never unsets, so a
       single transient clear is caught even though the fault later re-raises.
    3. **A DEPARTED entry never crowds out a PRESENT one** (test_03). The tracked node is
       killed. Its entry can never become idle again - becoming idle needs a real
       measurement of a node that is gone - so under a cap full of the dead the present,
       genuinely broken node would be refused forever and the detector would report health
       it had refused to check. The departed entry is collapsed into a count instead (which
       keeps its own code's content non-empty, so its departure still heals nothing) and the
       present node is admitted and reported.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('lifecycle_expectation_cap_e2e_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_01_the_refused_node_is_reported_as_saturation(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load, its tick loop never ran, or the bringup grace never elapsed, so no '
            'assertion below could mean anything',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the fault_manager '
            'in this launch',
        )
        # Both required nodes must actually be present and matched, or "one of them was
        # refused" would just be "one of them never came up".
        for node_id in (CAP_TRACKED_NODE, CAP_REFUSED_NODE):
            self.assertTrue(
                _poll_apps_present(PORT, node_id, timeout=30.0 * TIME_SCALE),
                f'{node_id} never appeared on GET /apps - with only one required node '
                'present the cap of one is not full and nothing is refused',
            )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, CAP_TRACKED_NODE, 'unconfigured',
                                  timeout=30.0 * TIME_SCALE),
            f'{CAP_TRACKED_NODE} never read "unconfigured" - the node that must win the '
            'single slot was never measured as a violation at all',
        )

        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0 * TIME_SCALE)
        self.assertIsNotNone(
            fault, f'{FAULT_CODE} never raised for {CAP_TRACKED_NODE}, which holds the one slot')
        self.assertIn(CAP_TRACKED_NODE, fault.get('description', ''))

        # The refusal itself, on the operator-visible surface.
        self.assertTrue(
            poll_detector_status(PORT, DETECTOR_ID, 'tracking_saturated', True,
                                 timeout=30.0 * TIME_SCALE),
            'GET /x-medkit-watchdog never reported the lifecycle_expectation tracked-node '
            f'cap as saturated while {CAP_REFUSED_NODE} was present, required and refused - '
            'a required node is going unchecked and nothing an operator can read says so',
        )
        block = watchdog_detector_status(PORT, DETECTOR_ID)
        self.assertIsNotNone(block, 'the detector status block vanished between two reads')
        self.assertEqual(
            block.get('tracked_node_cap'), CAP_NODE_CAP,
            'the status route reports a different tracked_node_cap than the one this launch '
            f'configured ({CAP_NODE_CAP}) - the key did not reach the detector: {block}')

    def test_02_a_refused_node_withholds_the_clear(self):
        before = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            before, f'{FAULT_CODE} is not in the store, so there is nothing a clear could heal')
        self.assertIsNone(
            before.get('last_passed'),
            f'{FAULT_CODE} already had a PASSED on record before this test '
            f'(last_passed={before.get("last_passed")!r}), so nothing below could be '
            'attributed to the withhold',
        )

        # Drop the tracked node's lifecycle services. It stays present and stays non-idle
        # (an unmeasured clock is evidence), so it keeps the single slot - but once that
        # clock matures, ownership passes to GRAPH_NODE_NOT_MANAGED and GRAPH_NODE_INACTIVE
        # has no content and nothing pending. That is the tick the clear would flow on.
        self.assertTrue(
            _set_bool_parameter(
                type(self)._client_node, f'/{CAP_TRACKED_NODE}/set_parameters',
                'drop_services', True, timeout=30.0 * TIME_SCALE),
            f'setting drop_services:=true on /{CAP_TRACKED_NODE}/set_parameters never '
            'succeeded, so the state this test needs was never reached',
        )
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE_NOT_MANAGED, timeout=CAP_NOT_MANAGED_RAISE_TIMEOUT_SEC),
            f'{FAULT_CODE_NOT_MANAGED} never raised for {CAP_TRACKED_NODE} after its '
            'lifecycle services were dropped - its unmeasured clock never matured, so '
            f'{FAULT_CODE} still has content and the withhold below would prove nothing',
        )
        time.sleep(CAP_WITHHOLD_WINDOW_SEC)
        after = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(after, f'{FAULT_CODE} vanished from the store entirely')
        self.assertIsNone(
            after.get('last_passed'),
            f'{FAULT_CODE} was reported PASSED while {CAP_REFUSED_NODE} was present, '
            f'required and REFUSED by the tracked-node cap (last_passed='
            f'{after.get("last_passed")!r}) - the detector asserted that every required '
            'node is healthy after declining to check one of them',
        )
        self.assertNotEqual(
            after.get('status'), 'healed',
            f'{FAULT_CODE} healed while a required node was never checked')

        # And the window really was about a REFUSAL: the tracked node is not idle (an
        # unmeasured clock is evidence), so it kept the single slot the whole way through and
        # the other required node was never admitted behind our back.
        self.assertTrue(
            poll_detector_status(PORT, DETECTOR_ID, 'tracking_saturated', True,
                                 timeout=15.0 * TIME_SCALE),
            'the cap stopped reporting itself saturated once the tracked node went '
            'not-managed - the refused node was admitted, so the window above was not about '
            'a withheld clear at all',
        )

    def test_03_a_departed_entry_is_collapsed_so_the_present_node_is_checked(self, tracked_node):
        old_pid = tracked_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, CAP_TRACKED_NODE, timeout=30.0 * TIME_SCALE),
            f'{CAP_TRACKED_NODE} (pid {old_pid}) is still listed on GET /apps after SIGTERM '
            '- it never left the graph, so its entry was never a DEPARTED one',
        )
        self.assertTrue(
            _poll_apps_present(PORT, CAP_REFUSED_NODE, timeout=30.0 * TIME_SCALE),
            f'{CAP_REFUSED_NODE} is not on GET /apps - the node that must now be admitted '
            'is not even present',
        )

        # The whole hypothesis: the slot goes to the PRESENT node.
        self.assertTrue(
            _poll_fault_description_contains(
                PORT, FAULT_CODE, CAP_REFUSED_NODE, timeout=CAP_ADMISSION_TIMEOUT_SEC),
            f'{FAULT_CODE} never named {CAP_REFUSED_NODE} once {CAP_TRACKED_NODE} left the '
            'graph - a present, genuinely broken required node is being refused by an entry '
            'for a node that is gone and can never become idle again, so the detector '
            'reports health it has refused to check',
        )
        # And the refusal is over, so the latch that reports it has re-armed for a later one.
        self.assertTrue(
            poll_detector_status(PORT, DETECTOR_ID, 'tracking_saturated', False,
                                 timeout=30.0 * TIME_SCALE),
            'the cap still reports itself saturated after the departed entry was collapsed '
            'and the present node admitted - a later, real saturation would be indistinguishable',
        )
        # The departed node's own fault is not healed by its departure: its evidence lives on
        # as a count, which is what keeps that code's content non-empty.
        record = _fault_record(PORT, FAULT_CODE_NOT_MANAGED, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(
            record, f'{FAULT_CODE_NOT_MANAGED} vanished from the store after the departure')
        self.assertIsNone(
            record.get('last_passed'),
            f'{FAULT_CODE_NOT_MANAGED} was reported PASSED once {CAP_TRACKED_NODE} left '
            f'(last_passed={record.get("last_passed")!r}) - collapsing its entry to free a '
            'slot threw its evidence away instead of keeping it as a count',
        )
        # Nothing anywhere in this run ever cleared GRAPH_NODE_INACTIVE.
        final = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNone(
            final.get('last_passed'),
            f'{FAULT_CODE} was reported PASSED at some point in this run '
            f'(last_passed={final.get("last_passed")!r}) - a required node was refused or '
            'stuck for the whole of it',
        )


class TestLifecycleExpectationUnsettledDeparture(unittest.TestCase):
    """One bad sweep must not turn a healthy departure into a permanent fault.

    A present, healthy, MANAGED node reads "not a managed lifecycle node" for a tick or two
    whenever its ``get_state`` path is missing from one sweep - ``LifecycleWatcher::update()``
    drops a tracked id whose path is absent from the current sweep, and ``discover_apps()``
    can yield an app with no services when a sweep races service enumeration. Absence
    continues whatever the node was last observed as, so if that tick happens to be the last
    one before a clean shutdown, a healthy departure matures into a permanent
    GRAPH_NODE_NOT_MANAGED about a node that left in good health.

    Two legs of the SAME fixture, differing only in how long the dropped state is held, so
    the pair is discriminating rather than a bare silence proof:

    - ``blink_departer`` drops its lifecycle services and is killed immediately, well inside
      the settling budget. Nothing may ever be reported about it.
    - ``settled_departer`` drops its lifecycle services and holds that state long past the
      settling budget before being killed. It IS genuinely not managed when it leaves, and
      must still be reported - the positive control that proves the fixture, the discovery
      path and the detector all work, so the blinker's silence is a measurement rather than
      a stack that never came up.

    The blink leg's own budget is MEASURED, not assumed: the elapsed time from the drop to
    the node being gone from ``GET /apps`` - the same ThreadSafeEntityCache the detector's
    per-tick snapshot reads - must be under the settling budget, or the leg is silently
    exercising the settled case instead and proves the opposite of what it claims.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('lifecycle_expectation_unsettled_e2e_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def _drop_services(self, node_id):
        self.assertTrue(
            _set_bool_parameter(
                type(self)._client_node, f'/{node_id}/set_parameters',
                'drop_services', True, timeout=30.0 * TIME_SCALE),
            f'setting drop_services:=true on /{node_id}/set_parameters never succeeded - '
            'the node never stopped looking like a managed lifecycle node',
        )

    def test_01_both_required_nodes_are_present_and_healthy(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state, so no assertion below '
            'could mean anything',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the fault_manager '
            'in this launch, so an absent fault would prove nothing',
        )
        for node_id in (UNSETTLED_BLINK_NODE, UNSETTLED_SETTLED_NODE):
            self.assertIsNotNone(
                _poll_watchdog_entity(PORT, node_id, 'active', timeout=30.0 * TIME_SCALE),
                f'{node_id} never read lifecycle "active" - it was not discovered as a '
                'managed node, so it cannot be a HEALTHY managed node whose services then '
                'disappear',
            )

    def test_02_a_one_sweep_blink_before_a_clean_exit_is_measured_as_such(self, blink_node):
        self._drop_services(UNSETTLED_BLINK_NODE)
        dropped_at = time.monotonic()
        # Prove the detector's own view actually went not-managed for at least one tick:
        # the watchdog route reports a null lifecycle exactly when the watcher no longer
        # tracks the id, which is what the tracker classifies as kNotManaged.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNSETTLED_BLINK_NODE, None, timeout=15.0 * TIME_SCALE),
            f'{UNSETTLED_BLINK_NODE} never lost its tracked lifecycle state after its '
            'services were dropped - this leg never produced the unmeasured observation it '
            'is about',
        )
        os.kill(blink_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, UNSETTLED_BLINK_NODE,
                              timeout=30.0 * TIME_SCALE, interval=0.05),
            f'{UNSETTLED_BLINK_NODE} never left GET /apps after SIGTERM')
        unmanaged_for = time.monotonic() - dropped_at
        settle_budget_sec = (UNSETTLED_SETTLE_TICKS * UNSETTLED_TICK_INTERVAL_MS) / 1000.0
        self.assertLess(
            unmanaged_for, settle_budget_sec,
            f'{UNSETTLED_BLINK_NODE} was observable as not-managed for {unmanaged_for:.2f}s, '
            f'past the {settle_budget_sec:.1f}s settling budget - this leg exercised a '
            'SETTLED not-managed departure, which must be reported, so its silence below '
            'would be a failure rather than the property under test',
        )

    def test_03_a_settled_not_managed_departure_is_still_reported(self, settled_node):
        self._drop_services(UNSETTLED_SETTLED_NODE)
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, UNSETTLED_SETTLED_NODE, None, timeout=15.0 * TIME_SCALE),
            f'{UNSETTLED_SETTLED_NODE} never lost its tracked lifecycle state after its '
            'services were dropped',
        )
        # Held well past the settling budget, so the observation is corroborated before the
        # node leaves - the difference, and the only difference, from the blink leg above.
        time.sleep((UNSETTLED_SETTLED_HOLD_TICKS * UNSETTLED_TICK_INTERVAL_MS) / 1000.0)
        os.kill(settled_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, UNSETTLED_SETTLED_NODE, timeout=30.0 * TIME_SCALE),
            f'{UNSETTLED_SETTLED_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE_NOT_MANAGED, timeout=UNSETTLED_RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE_NOT_MANAGED} never raised for {UNSETTLED_SETTLED_NODE}, which was '
            'genuinely not a managed lifecycle node for many consecutive ticks before it '
            'left the graph - the departure carve-out swallowed a real finding',
        )
        self.assertIn(UNSETTLED_SETTLED_NODE, fault.get('description', ''))

    def test_04_the_blinking_node_is_never_reported_at_all(self):
        # By now the settled leg has been reported, which means every horizon that could
        # have matured the blinker's own one-sweep reading has been passed on this same
        # stack: it dropped its services first and left the graph first.
        for code in (FAULT_CODE_NOT_MANAGED, FAULT_CODE_UNREADABLE, FAULT_CODE):
            record = _fault_record(PORT, code, timeout=5.0 * TIME_SCALE)
            description = (record or {}).get('description', '')
            self.assertNotIn(
                UNSETTLED_BLINK_NODE, description,
                f'{code} names {UNSETTLED_BLINK_NODE}, a healthy managed node whose '
                'lifecycle services were missing from one sweep before it shut down '
                f'cleanly: {description!r}',
            )


class TestLifecycleExpectationWideGrace(unittest.TestCase):
    """A `grace` past the accepted maximum must not silence the detector for days.

    ``grace`` used to be accepted all the way to ``INT_MAX - 1``. A node under such a value
    is never CONFIRMED - its streak advances one per tick and would need billions of them -
    and never cleared either: a streak above zero puts the node in the tracker's pending set,
    the withheld-clear guard returns early on every tick, and GRAPH_NODE_INACTIVE can neither
    raise nor heal for ANY node for the life of the process. That is not a wide tolerance, it
    is an off switch with no warning attached.

    Accepting a value that large is the defect, so the fix is a sane accepted maximum: this
    launch configures the old maximum and the detector must refuse it and keep the documented
    default, which the run then measures the ordinary way - by the fault actually appearing.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_01_an_out_of_range_grace_is_refused_and_the_default_applies(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state, so a missing fault would '
            'prove nothing',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0 * TIME_SCALE),
            'GET /faults never answered 200 - the gateway never reached the fault_manager, '
            'so a missing fault would prove nothing',
        )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=30.0 * TIME_SCALE),
            f'{TARGET_NODE} never read "unconfigured" - the violation this test needs was '
            'never present',
        )

        fault = poll_faults(PORT, FAULT_CODE, timeout=WIDE_GRACE_RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} never raised for a node stuck inactive under grace='
            f'{WIDE_GRACE_VALUE}. The value was accepted, so the streak has to climb to it '
            'before anything is reported and the node sits in the tracker pending set '
            'meanwhile, which also withholds the clear - GRAPH_NODE_INACTIVE can neither '
            'raise nor heal for any node for days',
        )
        self.assertIn(TARGET_NODE, fault.get('description', ''))

    def test_02_the_fault_survives_the_node_leaving(self, target_node):
        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=30.0 * TIME_SCALE),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')
        self.assertTrue(
            _poll_fault_description_contains(
                PORT, FAULT_CODE, 'has since left the graph', timeout=30.0 * TIME_SCALE),
            f'{FAULT_CODE} still describes {TARGET_NODE} as a present node after it left '
            'the graph',
        )
        record = _fault_record(PORT, FAULT_CODE, timeout=10.0 * TIME_SCALE)
        self.assertIsNone(
            record.get('last_passed'),
            f'{FAULT_CODE} was reported PASSED once {TARGET_NODE} left the graph '
            f'(last_passed={record.get("last_passed")!r}) - within one gateway lifetime a '
            'departure never heals a fault',
        )


class TestLifecycleExpectationRestartDeparted(unittest.TestCase):
    """Where "a departure never heals a fault" actually ends: a gateway restart.

    This test does not fix anything - it RECORDS a boundary, so it is a decision on file
    rather than a surprise. Inside one gateway process a node that leaves while violating
    keeps its fault raised for as long as the process lives. Across a restart it does not,
    and nothing else in this suite says so.

    Why it cannot: after a restart the tracker is empty and the departed node is not in the
    graph, so its ``require_active`` entry matches nothing. An entry that matches nothing is
    indistinguishable from a misspelt one - the only component that knows the difference is
    the fault store, which this detector does not read at startup - and the hold for a
    never-matched entry is deliberately bounded, because a typo must not block healing
    forever. Once it expires there is no content and nothing pending, so the level-triggered
    clear flows and the record heals without any measurement having been taken.

    Re-seeding the tracker from the fault store at startup would change this. That is a real
    feature and deliberately not what this test asks for; the test asks only that the
    boundary stop being invisible.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_01_the_fault_raises_and_survives_the_node_leaving(self, target_node):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0 * TIME_SCALE),
            'graph_watchdog never reported an armed global state')
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=30.0 * TIME_SCALE),
            f'{TARGET_NODE} never read "unconfigured"')
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE, timeout=60.0 * TIME_SCALE),
            f'{FAULT_CODE} never raised for the stuck {TARGET_NODE}')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=30.0 * TIME_SCALE),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')
        time.sleep(max(4.0, (GRACE + 8) * RESTART_TICK_INTERVAL_MS / 1000.0))
        record = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(record, f'{FAULT_CODE} vanished from the store')
        self.assertIsNone(
            record.get('last_passed'),
            f'{FAULT_CODE} was reported PASSED while the gateway that measured '
            f'{TARGET_NODE} was still running (last_passed={record.get("last_passed")!r}) - '
            'within one gateway lifetime a departure never heals a fault',
        )

    def test_02_a_gateway_restart_re_baselines_and_the_record_heals(self, gateway_node):
        old_pid = gateway_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _wait_until_port_is_down(PORT, timeout=60.0 * TIME_SCALE),
            f'the gateway (pid {old_pid}) kept answering after SIGTERM - nothing restarted')
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=90.0 * TIME_SCALE),
            'the gateway never came back armed after the restart')
        self.assertNotEqual(
            gateway_node.process_details['pid'], old_pid,
            'the gateway process id did not change, so this test never restarted anything')

        # The recorded boundary: the restarted detector has no measurement of the departed
        # node and cannot tell its entry from a typo, so the bounded never-matched hold
        # expires and the clear flows. This is the documented scope of "a departure never
        # heals a fault" - within a gateway lifetime.
        self.assertTrue(
            poll_cleared(PORT, FAULT_CODE, timeout=RESTART_HEAL_TIMEOUT_SEC),
            f'{FAULT_CODE} is still an active fault long past the never-matched hold after '
            'a gateway restart. That is not what the code does today, so either the hold '
            'stopped being bounded or something now re-seeds the tracker at startup - '
            'either way the documented scope of the promise needs rewriting, not this test',
        )
        record = _fault_record(PORT, FAULT_CODE, timeout=30.0 * TIME_SCALE)
        self.assertIsNotNone(record, f'{FAULT_CODE} vanished from the store entirely')
        self.assertIsNotNone(
            record.get('last_passed'),
            f'{FAULT_CODE} left the active list without ever being reported PASSED - it was '
            'not the detector that healed it, so this test is no longer measuring the '
            'boundary it is named for',
        )


# Each CTest target launches this file with one scenario, so only that scenario's case
# may run. Removing the others from the module (rather than skipping them) means each
# run reports exactly one case, and a missing result is a real failure rather than an
# expected line of output - see test_config_plumbing_e2e.test.py's identical rationale.
_SCENARIO_CASES = {
    'main': 'TestLifecycleExpectationMain',
    'default_config': 'TestLifecycleExpectationDefaultConfig',
    'negative_control': 'TestLifecycleExpectationNegativeControl',
    'unreadable': 'TestLifecycleExpectationUnreadable',
    'healing_threshold': 'TestLifecycleExpectationHealingThreshold',
    'departure_keeps': 'TestLifecycleExpectationDepartureKeeps',
    'not_managed': 'TestLifecycleExpectationNotManaged',
    'restart_loop': 'TestLifecycleExpectationRestartLoop',
    'cap_pressure': 'TestLifecycleExpectationCapPressure',
    'unsettled_departure': 'TestLifecycleExpectationUnsettledDeparture',
    'wide_grace': 'TestLifecycleExpectationWideGrace',
    'restart_departed': 'TestLifecycleExpectationRestartDeparted',
}
if SCENARIO not in _SCENARIO_CASES:
    raise RuntimeError(
        f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} is not one of {sorted(_SCENARIO_CASES)}; '
        'the CTest target and this file disagree about which scenarios exist')
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
