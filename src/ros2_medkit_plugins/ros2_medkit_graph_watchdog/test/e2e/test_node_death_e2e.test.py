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
"""node_death e2e: GRAPH_NODE_DISAPPEARED against the REAL gateway + fault_manager + graph.

A scenario that asserts a RAISE proves the fault actually appears, naming the missing-fault
case as the failure mode when it does not. A scenario that asserts an ABSENCE (a node that must
never be called dead) cannot, by itself, discriminate a correct detector from no detector at
all, because both produce the same silence; those scenarios are marked as such in each class's
own docstring below.

Runs as TEN separate CTest targets (see CMakeLists.txt), each launching its OWN
gateway+fault_manager+demo-node stack, exactly as test_lifecycle_expectation_e2e.test.py
does and for the same reason: the plugin reads its config once at set_context() time, so
different configs need different gateway launches. WATCHDOG_E2E_SCENARIO selects which
launch and which assertions run:

- "raise": a plain node comes up, is armed, then its process exits. GRAPH_NODE_DISAPPEARED
  appears and names it. Also runs both new harness self-tests
  (prove_persistence_proof_catches_a_dead_fault_surface,
  prove_describes_only_proof_catches_a_dead_fault_surface) - self-contained, so they run
  alongside the real assertions without depending on them, the way the lifecycle e2e's
  "default_config" scenario runs the silence one.
- "clear_on_return": the same, then the node is started again. The fault clears once the
  node is back - the occurrence model's own "outage genuinely ended" case: no forced
  transition, the detector closes a cycle when the graph actually recovers.
- "no_heal_standalone": the same drive as clear_on_return, but the fault_manager runs with
  healing OFF. The fault does NOT clear - not a defect, the documented shape of a debounce
  hysteresis latch that only a HEALED-enabled config can cross - proven as a SUSTAINED
  claim over a window via assert_fault_persists_throughout, not a single sample.
- "deactivated_not_dead": a managed lifecycle node reaches active on its own
  (managed_lifecycle_active's auto_activate) and is then driven to DEACTIVATE by this test
  through a real ChangeState call, while its process keeps running throughout. No
  GRAPH_NODE_DISAPPEARED for the whole window - node_death is a presence detector; a
  lifecycle transition is not a departure, and telling the two apart is
  lifecycle_expectation's job, not this one's (see B1-B5 in
  test_node_death_boundary_e2e.test.py, none of them written here). See
  TestNodeDeathDeactivatedNotDead's own docstring for what this absence does and does not
  prove.
- "manifest_never_online": a hybrid-mode manifest declares an App whose ROS binding never
  starts. No GRAPH_NODE_DISAPPEARED for it, ever - the promise the public issue leads with:
  a manifest node keeps its App in the snapshot with the online flag cleared, so a detector
  that counted snapshot membership alone would call it immortal - see
  TestNodeDeathManifestNeverOnline's own docstring for what this absence does and does not
  prove.
- "ros2cli_ignored": a node renamed to carry the ros2cli hidden-node prefix is armed then
  killed, three times over with distinct names. No fault, and the detector's own
  tracked-count status field does not grow across the three cycles - a node matching the
  naming convention must not accumulate as permanent tracked state, regardless of what
  process gave it that name. A second check pins that fixture's prefix against the
  installed ros2cli package's own naming constants, so the fixture cannot silently drift
  from what ros2cli itself actually uses - see TestNodeDeathRos2cliIgnored's own docstring
  for what this absence does and does not prove.
- "bare_name_collision": two nodes named 'calibration' in different namespaces; one exits.
  The fault names the one that exited and does not name the one still running - proven with
  assert_fault_describes_only so the claim holds over the WHOLE window, not one lucky read.
- "fast_tick_floor": a short tick_interval_ms, with a node continuously present. No fault.
  Narrower than the row this is named for: it does not force a stale graph-cache
  generation (config sweep C1 owns the miss_grace floor's own boundary values), only that
  fast ticking alone, with nothing perturbing the graph, is safe - see
  TestNodeDeathFastTickFloor's own docstring for what was investigated and why forcing the
  stronger condition was not implemented - see TestNodeDeathDeactivatedNotDead's own note
  for what this absence does and does not prove either way.
- "restart_loop_occurrences": a node killed and restarted three times, each cycle closed
  with an explicit acknowledge before the next kill. The fault's occurrence_count reaches
  3 - the fault manager's own occurrence model (a FAILED event reactivating a CLEARED
  record bumps the count; a re-report on a still-active fault does not) rather than any
  trick the detector plays. See TestNodeDeathRestartLoopOccurrences's own docstring for why
  this class stops at occurrence_count and does not attempt the per-occurrence RECORDING
  half of this row.
- "restart_rebaseline": a node dies, the fault confirms, then the gateway is killed by PID
  and comes back with the node still gone. RECORDS the boundary, the way
  TestLifecycleExpectationRestartDeparted does for the sibling detector: the fault stays
  CONFIRMED, proven as a sustained claim across the restart via
  assert_fault_persists_throughout, not discovered by a single sample.

Every scenario gates on wait_until_watchdog_armed(PORT) BEFORE asserting anything, and
every scenario that asserts an ABSENCE additionally gates on
wait_until_faults_endpoint_live(PORT) - see harness.py's own docstrings for why a stack
that never came up otherwise produces exactly the silence an absence claim is looking for.
"""

import os
import signal
import subprocess
import sys
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import HIDDEN_NODE_PREFIX, Node
import requests
from ros2cli.node import NODE_NAME_PREFIX

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above,
# so this import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    API_BASE_PATH,
    assert_fault_absent_throughout,
    assert_fault_describes_only,
    assert_fault_persists_throughout,
    create_watchdog_test_launch,
    poll_cleared,
    poll_entity_faults,
    poll_faults,
    prove_describes_only_proof_catches_a_dead_fault_surface,
    prove_persistence_proof_catches_a_dead_fault_surface,
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

# No default on purpose - a default makes this file FAIL OPEN (see harness-consuming
# siblings' identical rationale): if the CTest ENVIRONMENT property never reaches the
# process, the launch and the assertions would degrade together into one scenario and
# still report 1/1 passed. A KeyError is loud.
SCENARIO = os.environ['WATCHDOG_E2E_SCENARIO']
PORT = get_test_port()

FAULT_CODE = 'GRAPH_NODE_DISAPPEARED'
# The detector's own id, matched against GET /x-medkit-watchdog's `detectors` block by
# watchdog_detector_status(). node_death has no config to isolate a fault ON (it is
# zero-config: every App the graph carries is in scope, not an explicit require_active-style
# list), so unlike lifecycle_expectation's _DETECTOR_PREFIX-scoped keys, only `miss_grace`
# and `prune_grace` below are ever set under it.
DETECTOR_ID = 'node_death'
_DETECTOR_PREFIX = f'plugins.graph_watchdog.detectors.{DETECTOR_ID}'

# Fast tick cadence + a miss_grace comfortably PAST the documented 3000 ms floor
# (min_node_death_miss_grace(200) == 14, detector_config_keys.hpp) without sitting exactly on
# it. "config sweep C1" (NodeDeathIntegrationTest.C1_* in test_node_death_integration.cpp) owns
# the floor's own boundary values, but never exercises tick_interval_ms=200 at all (its own
# cases run at 1, 1000 and 3000 ms), so there is no boundary value at this tick period to land
# on by accident - 16 carries two ticks of headroom purely to keep this row visibly off the
# floor itself. 16 ticks * 200 ms = 3400 ms nominal grace; RESPAWN_DELAY_SEC below is sized
# against this value.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3
MISS_GRACE = 16

# The plain node most scenarios kill: DEMO_NODE_REGISTRY's own executable/name/namespace
# for the 'calibration' key, constructed by hand (not via demo_nodes=[...]) wherever a
# scenario needs a PID to signal - create_demo_nodes() gives no handle back. Single
# occurrence of the bare name 'calibration' in this launch, so the discovery layer's
# collision-avoidance never engages and the App id IS the bare name (verified against
# ros2_runtime_introspection.cpp's own collision rule: only namespace-prefixed when the
# SAME bare name appears in more than one namespace at once).
TARGET_NODE = 'calibration'
TARGET_EXECUTABLE = 'demo_calibration_service'
TARGET_NAMESPACE = '/powertrain/engine'

# launch will not even ATTEMPT to restart a respawn=True process before this elapses, so
# it is a real, enforced floor under every kill-then-return gap this file measures. Must
# clear the detector's OWN raise threshold, not just be "over one tick interval": a death is
# reported once misses EXCEED miss_grace, i.e. after (MISS_GRACE + 1) * TICK_INTERVAL_MS =
# 17 * 200ms = 3400ms of real absence. A respawn_delay short of that heals the outage before
# a correct detector could ever confirm it, so clear_on_return/no_heal_standalone/
# restart_loop_occurrences (the three scenarios that actually respawn TARGET_NODE) would
# time out waiting for a raise that is never supposed to happen - not a detector defect.
# Mirrors B5_RESPAWN_DELAY_SEC in the boundary e2e file (same detector, same tick interval,
# same derivation - see that constant's own comment for the full arithmetic). 3400ms nominal
# grace is not the whole budget a tick-counted miss_grace needs: the entity cache is refreshed
# on a debounced graph event, up to 1100ms of latency on either end of the outage
# (gateway_node.cpp), and the plugin's own tick loop can run slower than its configured period
# under CI contention with no enforced upper bound. Margin under the 1100ms debounce ceiling
# alone can be erased by a single unlucky refresh regardless of tick-loop speed, and
# NodeLivenessTracker::update() resets a key's miss count to zero the instant it is seen
# present again, so a cycle that loses this race does not raise late, it never raises at all.
# 12.5s clears 3*3400+1100=11300ms (a tick loop running at a third of its configured rate,
# plus the full debounce ceiling on top) by 1200ms - worst-case observed window
# 12500-1100=11400ms, 3.35x the nominal grace.
RESPAWN_DELAY_SEC = 12.5

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
# How long a fault must stay ABSENT, or CONFIRMED, for the scenarios whose claim is
# sustained rather than momentary. Short (well under a minute) because every window here
# is measured from the arming gate, not process start, so bringup cannot eat it - matching
# SILENT_WINDOW_SEC's own rationale in the lifecycle e2e file.
SUSTAINED_WINDOW_SEC = 20.0

# The entity that owns the aggregated fault this plugin raises - proven already by
# test_lifecycle_expectation_e2e.test.py's own poll_entity_faults(PORT,
# 'apps/graph_watchdog', ...) calls, reused here for the REST-scoped clear_fault call
# restart_loop_occurrences needs.
SOURCE_ENTITY_PATH = 'apps/graph_watchdog'

# ---- the "fast_tick_floor" scenario's own cadence ---------------------------------------
#
# Deliberately tiny: at 50 ms/tick, a miss_grace of 1 tick is 50 ms of NOMINAL grace - far
# under the documented 3000 ms floor (config sweep C1). If the floor did not exist, a
# single stale graph-cache generation between two ticks would be enough to call a healthy,
# present node dead. The claim under test is that it is not enough, because the floor
# raises the EFFECTIVE grace regardless of how small tick_interval_ms is configured.
FAST_TICK_INTERVAL_MS = 50
FAST_MISS_GRACE = 1

# ---- the "bare_name_collision" scenario's own namespaces ---------------------------------
#
# Short and distinctive so they cannot be confused with any area/namespace used elsewhere
# in this file or in the shared demo fixtures.
COLLISION_NAMESPACE_A = '/coll_a'
COLLISION_NAMESPACE_B = '/coll_b'
# ros2_runtime_introspection.cpp's own collision-avoidance rule, applied by hand: two nodes
# named 'calibration' in different namespaces get namespace-prefixed ids
# ('<ns-without-leading-slash-slashes-as-underscores>_<name>'), so the bare 'calibration'
# alone is no longer sufficient to tell them apart - which is exactly what this scenario
# means by "the fault names the one that exited and does not name the one still running".
COLLISION_APP_ID_A = 'coll_a_calibration'
COLLISION_APP_ID_B = 'coll_b_calibration'
# How long the "names the dead one, not the alive one" claim is watched for. Short (a few
# ticks at TICK_INTERVAL_MS) because the claim is "never confuses the two", not "forever" -
# but a WINDOW checked on every poll, never a single sample.
MUTUAL_NAMING_WINDOW_SEC = 5.0

# ---- the "ros2cli_ignored" scenario's own fixtures -----------------------------------------
# The naming convention node_death filters on: a leaf node name
# starting with this prefix, regardless of what process created it. The load-bearing half
# of this scenario proves the convention itself, not any particular producer of it - see
# TestNodeDeathRos2cliIgnored's own docstring for why that distinction is the whole point.
# Pinned against the installed `ros2cli` package's own NODE_NAME_PREFIX, not just assumed
# to stay correct - see this class's test_02.
ROS2CLI_FAKE_NODE_PREFIX = '_ros2cli_fake_'
# How many renamed-node arm/kill cycles the scenario runs, and how many distinct
# ROS2CLI_FAKE_NODE_PREFIX-named nodes it exercises.
ROS2CLI_CYCLES = 3
# How long the before/after tracked_count samples may take to settle (see
# _poll_stable_tracked_count) - generous against gateway-internal infrastructure (confirmed
# live: a hidden `_param_client_node`, visible only because this scenario turns off
# filter_internal_nodes) arming on its own schedule, not against anything this scenario's
# own cycles do. Must comfortably exceed _poll_stable_tracked_count's own stable_seconds
# default (10.0), or the poll could time out before stability was ever even reachable.
STABLE_TRACKED_COUNT_TIMEOUT_SEC = 40.0 * TIME_SCALE

# ---- the "restart_loop_occurrences" scenario's own target -------------------------------
# The claim under test is that occurrence_count tracks the number of genuine deaths. Three
# distinct occurrences demonstrate that exactly as well as five: what would falsify the claim
# is a count that stops incrementing (a detector that goes silent after the first cycle) or
# double-counts (an ungated clear or a duplicate FAILED inflating the number), and either
# failure mode shows up by the third cycle - a fourth or fifth would only repeat the same
# proof at the full cost of RESPAWN_DELAY_SEC apiece. If a future change ever needs more
# cycles to expose something this one does not, raising this single constant is the whole
# edit.
RESTART_LOOP_OCCURRENCES_TARGET = 3


def _target_node_action(*, respawn=False, respawn_delay=RESPAWN_DELAY_SEC):
    """One manually-constructed TARGET_NODE action, with a PID handle the test can signal.

    Mirrors DEMO_NODE_REGISTRY's own 'calibration' entry exactly (same package, executable,
    name, namespace) so this is the identical fixture every other scenario in this package
    gets via demo_nodes=['calibration'] - just launched by hand because a scenario that
    kills it needs the launch action's own process_details, which create_demo_nodes() does
    not hand back.
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
        respawn=respawn,
        respawn_delay=respawn_delay,
    )


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
        f'{_DETECTOR_PREFIX}.miss_grace': MISS_GRACE,
    }
    extra_gateway_params = None
    demo_nodes = []
    healing_enabled = True
    gateway_respawn = False

    if SCENARIO == 'raise':
        pass  # target node launched by hand below, killed once, never restarted
    elif SCENARIO == 'clear_on_return':
        pass  # target node launched by hand below, killed then respawned
    elif SCENARIO == 'no_heal_standalone':
        healing_enabled = False  # the whole point of this scenario
    elif SCENARIO == 'deactivated_not_dead':
        demo_nodes = ['managed_lifecycle_active']  # self-activates; this test deactivates it
    elif SCENARIO == 'manifest_never_online':
        pkg_share = get_package_share_directory('ros2_medkit_gateway')
        manifest_path = os.path.join(pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml')
        extra_gateway_params = {
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            # The manifest's own validator can turn an informational notice into a load
            # failure under strict validation, which in hybrid mode silently degrades the
            # gateway to runtime_only - see this package's CLAUDE.md gotcha and
            # test_discovery_gap_fill.test.py's identical setting for the same manifest.
            'discovery.manifest_strict_validation': False,
        }
        # Only 'calibration' comes online - it satisfies 'engine-calibration-service's
        # ros_binding, giving the arming gate something to report on. Every OTHER app the
        # manifest declares (e.g. 'lidar-sensor', bound to a 'lidar_sensor' node this
        # launch never starts) stays in the snapshot with its online flag cleared and
        # nothing ever links it - the exact fixture this scenario needs, already sitting in
        # the manifest that ships as this repo's own integration-test fixture.
        demo_nodes = ['calibration']
    elif SCENARIO == 'ros2cli_ignored':
        # No demo_nodes: test_01 spawns and kills its own renamed nodes by hand, and
        # test_02 is a pure constant comparison against the installed ros2cli package -
        # neither needs anything else running.
        extra_gateway_params = {
            # OFF for this scenario only. Left at its true default (on), a node named
            # like the ros2cli convention would be invisible upstream of EVERYTHING -
            # GET /apps, the entity cache, any IntrospectionProvider plugin gets fed
            # from it - which would make this scenario pass for the wrong reason: never
            # tracked at all, rather than tracked and then correctly excluded by name.
            # This scenario is about node_death's OWN name-based exclusion, which needs a
            # node the detector's input can actually see. See the class docstring.
            'discovery.runtime.filter_internal_nodes': False,
        }
    elif SCENARIO == 'bare_name_collision':
        pass  # two hand-built nodes below, same bare name, different namespaces
    elif SCENARIO == 'fast_tick_floor':
        detector_params['plugins.graph_watchdog.tick_interval_ms'] = FAST_TICK_INTERVAL_MS
        detector_params[f'{_DETECTOR_PREFIX}.miss_grace'] = FAST_MISS_GRACE
        demo_nodes = ['calibration']
    elif SCENARIO == 'restart_loop_occurrences':
        pass  # target node launched by hand below, respawning under this test's control
    elif SCENARIO == 'restart_rebaseline':
        gateway_respawn = True  # this scenario's own subject, like lifecycle's "main"
    else:
        raise RuntimeError(f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} has no launch configuration')

    launch_description, context = create_watchdog_test_launch(
        detector_params=detector_params,
        extra_gateway_params=extra_gateway_params,
        demo_nodes=demo_nodes,
        port=PORT,
        healing_enabled=healing_enabled,
        gateway_respawn=gateway_respawn,
    )

    if SCENARIO in ('raise', 'restart_rebaseline'):
        # No respawn: both scenarios kill the node PERMANENTLY and measure what happens to
        # the fault it left behind - a bounce back would defeat the departure each is
        # actually about.
        target = _target_node_action(respawn=False)
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO in ('clear_on_return', 'no_heal_standalone', 'restart_loop_occurrences'):
        # respawn=True: every one of these scenarios kills the node and then needs it back
        # - once for clear_on_return/no_heal_standalone, three times in a loop for
        # restart_loop_occurrences - and create_demo_nodes() gives no PID handle to SIGTERM
        # by hand in the first place, so this fixture is built here regardless.
        target = _target_node_action(respawn=True, respawn_delay=RESPAWN_DELAY_SEC)
        launch_description.add_action(TimerAction(period=2.0, actions=[target]))
        context['target_node'] = target

    if SCENARIO == 'bare_name_collision':
        node_a = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=TARGET_EXECUTABLE,
            name=TARGET_NODE,
            namespace=COLLISION_NAMESPACE_A,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
        )
        node_b = launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=TARGET_EXECUTABLE,
            name=TARGET_NODE,
            namespace=COLLISION_NAMESPACE_B,
            output='screen',
            additional_env=get_coverage_env('ros2_medkit_integration_tests'),
            sigterm_timeout='30',
            sigkill_timeout='15',
        )
        launch_description.add_action(TimerAction(period=2.0, actions=[node_a, node_b]))
        context['node_a'] = node_a
        context['node_b'] = node_b

    return launch_description, context


# ---------------------------------------------------------------------------------------
# Local helpers. These read the SAME endpoints harness.py's own helpers do, but with
# either a different status filter (occurrence_count and last_passed survive a HEAL or a
# CLEAR, both of which drop out of the default active-only listing) or a service this
# file's scenarios need that no existing harness helper covers.
# ---------------------------------------------------------------------------------------

def _fault_record(port, code, timeout=30.0, interval=0.5):
    """Poll ``GET /faults?status=all`` until `code` appears, whatever its status.

    ``poll_faults`` uses the default (pending+confirmed) filter, so a HEALED or CLEARED
    fault disappears from it - indistinguishable from one that was never raised. The
    restart and occurrence scenarios need the record itself, including fields
    (``last_passed``, ``occurrence_count``) that survive both. Returns the matching item
    dict, or ``None`` on timeout.
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


def _poll_occurrence_count(port, code, expected, timeout=60.0, interval=0.5):
    """Poll until `code`'s stored ``occurrence_count`` equals `expected`. ``True`` once it does.

    Not "at least" - a detector that fires an extra spurious reactivation would satisfy
    "at least 5" just as well as one that fires exactly 5, so an exact match is what
    actually discriminates. Prints the last-seen record on timeout, which is gone once the
    launch tears down.
    """
    deadline = time.monotonic() + timeout
    last_seen = f'{code} was never in the store at all'
    while time.monotonic() < deadline:
        record = _fault_record(port, code, timeout=interval)
        if record is not None:
            last_seen = str(record)
            if record.get('occurrence_count') == expected:
                return True
        time.sleep(interval)
    print(f'_poll_occurrence_count({code!r}, expected={expected!r}) timed out after '
          f'{timeout}s; last seen: {last_seen}')
    return False


def _wait_until_port_is_down(port, timeout=60.0, interval=0.2):
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


def _clear_fault(port, entity_path, code, timeout=10.0):
    """``DELETE {entity_path}/faults/{code}`` - the REST acknowledge, ``~/clear_fault``.

    Unconditional: it writes CLEARED whatever the fault's current status is
    (``fault_storage.cpp``'s own ``clear_fault`` takes no status guard), which is exactly
    why it - and not a heal race - is what restart_loop_occurrences uses to close one
    outage cycle before the next kill reactivates it as a fresh occurrence. Returns
    ``True`` on any 2xx response.
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    try:
        response = requests.delete(f'{base}/{entity_path}/faults/{code}', timeout=timeout)
    except requests.exceptions.RequestException:
        return False
    return 200 <= response.status_code < 300


def _get_lifecycle_label(client_node, service_name, timeout=10.0):
    """One ``GetState`` call against `service_name`. Returns the state label, or ``None``.

    Creates its own client rather than taking one, since this is also called standalone
    (the end-of-window check in "deactivated_not_dead") where polling's own reused client
    would be the wrong lifetime to share.
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

    One client for the whole poll (not one per iteration, unlike a single
    `_get_lifecycle_label` call) - a 30 s poll at the default 0.5 s interval is up to 60
    iterations, and creating and destroying a client every single one of them is needless
    churn for what is otherwise an identical repeated call.
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


def _poll_stable_tracked_count(port, detector_id, timeout, stable_seconds=10.0, interval=1.0):
    """Poll the detector's status block until ``tracked_count`` holds steady.

    "Holds steady" means the SAME value for a continuous `stable_seconds` before `timeout`
    elapses. A single sample says nothing about whether the graph has settled: node_death is
    zero-config and tracks every armed App, not just this scenario's own three renamed
    fixtures. With `discovery.runtime.filter_internal_nodes` off (this scenario's own
    requirement - see the class docstring), that includes normally-hidden gateway-internal
    nodes too: confirmed live, the gateway keeps its own hidden `_param_client_node` for
    querying newly-discovered nodes' parameters (the same path that logs "Parameter service
    not available for node: ..." for each renamed fixture below), and that node is present
    from early on but can still be inside its OWN per-entity warmup - and so absent from
    tracked_count - at the moment a single sample happens to land, only to arm and join the
    tracked set a few seconds later. A single `before` sample can therefore catch it
    mid-warmup while a single `after` sample catches it already armed, reading as
    tracked_count GROWTH that has nothing to do with the ros2cli exclusion this row is
    actually about. `stable_seconds` has to be long enough to let that settle BEFORE either
    sample is trusted, not merely long enough to smooth over network jitter - a handful of
    consecutive reads close together would still land inside the SAME few-second warmup
    window and call it "stable" too early.

    Returns ``(status, stable)`` - `status` is the LAST status block read (possibly still
    unsettled, so a caller can still report what it saw), `stable` is False if the value
    never held for a continuous `stable_seconds` inside `timeout`. A status that reads None
    on every poll (no `detectors.node_death` block at all - meaning no such detector is
    registered) counts as stable at None: the caller's existing ``if before is not None``
    gate still skips the comparison the same way a single-sample None always did.
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

class TestNodeDeathRaise(unittest.TestCase):
    """A plain node's process exits: GRAPH_NODE_DISAPPEARED names it."""

    def test_process_exit_raises_naming_the_node(self, target_node):
        # app_id=TARGET_NODE, not the global gate: the global gate is satisfied by ANY
        # entity being armed, so it can go true from something else in the graph while
        # TARGET_NODE itself has not been read even once. Killing it before that read
        # happens is a kill a correct detector was never permitted to see - the raise
        # below would then time out for a precondition reason and read exactly like the
        # detector-missing red this suite is supposed to produce.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed - the plugin did not load, '
            f'{TARGET_NODE} was never discovered, or the bringup grace never elapsed, so '
            'no raise below could mean anything',
        )

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM - this scenario never '
            'produced the departure it is named for',
        )

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        if fault is None:
            self.fail(f'{FAULT_CODE} never raised after {TARGET_NODE} exited')
        self.assertIn(TARGET_NODE, fault.get('description', ''))

        # The flat /faults list carries a fault whatever its source is; only the
        # entity-scoped surface proves an operator can OPEN it somewhere.
        self.assertIsNotNone(
            poll_entity_faults(
                PORT, SOURCE_ENTITY_PATH, FAULT_CODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{FAULT_CODE} is not reachable at /{SOURCE_ENTITY_PATH}/faults - the entity '
            'the plugin publishes does not own the fault it raises',
        )

    def test_self_tests_catch_a_dead_fault_surface(self):
        """The tests of the tests for both new harness helpers, run once per this file.

        Self-contained (a local HTTP server stands in for the gateway) - runs alongside
        the real assertions above without depending on them, the way the lifecycle e2e's
        "default_config" scenario runs the silence one.
        """
        prove_persistence_proof_catches_a_dead_fault_surface(self)
        prove_describes_only_proof_catches_a_dead_fault_surface(self)


class TestNodeDeathClearOnReturn(unittest.TestCase):
    """A dead node coming back clears the fault.

    The occurrence model's own "outage genuinely ended" case.
    """

    def test_node_returning_clears_the_fault(self, target_node):
        # app_id=TARGET_NODE: see TestNodeDeathRaise's identical gate for why the global
        # gate alone is not enough before a kill.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')

        old_pid = target_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(fault, f'{FAULT_CODE} never raised after {TARGET_NODE} exited')

        # launch's own respawn (this scenario's target node is launched with respawn=True)
        # brings the same node back under the same name.
        self.assertTrue(
            _poll_apps_present(
                PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC + RESPAWN_DELAY_SEC),
            f'{TARGET_NODE} never came back after the SIGTERM - launch did not respawn it, '
            'so there is nothing here that could clear the fault by returning',
        )

        self.assertTrue(
            poll_cleared(PORT, FAULT_CODE, timeout=CLEAR_TIMEOUT_SEC),
            f'{FAULT_CODE} did not clear after {TARGET_NODE} came back - the outage ended '
            'but the fault manager still reports it active',
        )


class TestNodeDeathNoHealStandalone(unittest.TestCase):
    """The same drive as clear_on_return, with the fault_manager's healing turned OFF.

    The fault must NOT clear - the documented shape of the debounce hysteresis latch
    (compute_debounce_status: a CONFIRMED fault stays put unless healing is enabled and the
    counter reaches the healing threshold), not a defect. Proven as a SUSTAINED claim over
    a window via assert_fault_persists_throughout, not a single sample that could just be
    "not healed yet".
    """

    def test_node_returning_does_not_clear_the_fault(self, target_node):
        # app_id=TARGET_NODE: see TestNodeDeathRaise's identical gate for why the global
        # gate alone is not enough before a kill.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - a persistence claim below would prove '
            'nothing about the detector if the channel itself never came up',
        )

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(fault, f'{FAULT_CODE} never raised after {TARGET_NODE} exited')

        self.assertTrue(
            _poll_apps_present(
                PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC + RESPAWN_DELAY_SEC),
            f'{TARGET_NODE} never came back after the SIGTERM',
        )

        assert_fault_persists_throughout(self, PORT, FAULT_CODE, SUSTAINED_WINDOW_SEC)


class TestNodeDeathDeactivatedNotDead(unittest.TestCase):
    """A managed node that DEACTIVATES but keeps running is never called dead.

    Absence alone cannot distinguish a detector that correctly ignores lifecycle state
    from one that raises nothing at all - both leave GRAPH_NODE_DISAPPEARED silent here.
    What this row catches is a detector that conflates "not active" with "gone":
    node_death tracks graph PRESENCE only, so a node that deactivates without leaving the
    graph must never be named.

    Uses managed_lifecycle_active (self-activates via launch's own auto_activate
    parameter) rather than extending droppable_lifecycle_node.cpp: that fixture's own
    ChangeState handler is a documented stub ("Never driven: only
    find_lifecycle_get_state_path()'s type check needs it to exist") that always returns
    success without changing state, so it cannot be driven through a REAL
    active-then-deactivate transition at all - it can only ever answer a label FIXED at
    launch. managed_lifecycle is a real rclcpp_lifecycle::LifecycleNode and already
    supports genuine ChangeState transitions (see test_lifecycle_expectation_e2e.test.py's
    "main" scenario), so it needs no fixture changes for this scenario.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('node_death_e2e_deactivate_client')

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def test_deactivate_while_alive_never_raises(self):
        get_state_service = '/managed_lifecycle_active/get_state'
        change_state_service = '/managed_lifecycle_active/change_state'

        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC),
            'graph_watchdog never reported an armed global state')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - the absence below would prove nothing')

        self.assertTrue(
            _poll_lifecycle_label(
                type(self)._client_node, get_state_service, 'active',
                timeout=PRESENCE_TIMEOUT_SEC),
            'managed_lifecycle_active never reached "active" on its own - the trigger '
            'this scenario needs (an active node that then deactivates) was never set up',
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
            'managed_lifecycle_active never reached "inactive" after DEACTIVATE',
        )

        assert_fault_absent_throughout(self, PORT, FAULT_CODE, SUSTAINED_WINDOW_SEC)

        # The trigger was pinned at the START of the window; confirm it survived to the
        # end too, or most of the window measured an empty graph instead.
        self.assertEqual(
            _get_lifecycle_label(type(self)._client_node, get_state_service, timeout=15.0),
            'inactive',
            'managed_lifecycle_active is no longer reading "inactive" at the END of the '
            'silence window - the trigger did not survive it',
        )


class TestNodeDeathManifestNeverOnline(unittest.TestCase):
    """A manifest-declared App that never comes online is never called dead.

    Absence alone cannot distinguish a detector that correctly reads the online flag from
    one that raises nothing at all - see TestNodeDeathDeactivatedNotDead's own note. What
    it catches: a manifest node keeps its App in the snapshot with the online flag
    cleared, so a detector that counted snapshot membership alone would call it immortal;
    node_death instead arms only apps it has read online at least once.
    """

    NEVER_ONLINE_APP_ID = 'lidar-sensor'  # declared in demo_nodes_manifest.yaml; its
    # ros_binding (node 'lidar_sensor') is never launched by this scenario.
    # The manifest's own id for the app bound to the 'calibration' node. In hybrid mode a
    # linked App is exposed under the manifest's declared id, not the runtime-derived bare
    # node name - unlike the other scenarios in this file, which run runtime_only.
    ONLINE_APP_ID = 'engine-calibration-service'

    def test_never_online_manifest_app_never_raises(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC),
            'graph_watchdog never reported an armed global state - if the manifest failed '
            'to load (see discovery.manifest_strict_validation) this could be silently '
            'measuring runtime_only instead of the hybrid launch this scenario needs',
        )
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - the absence below would prove nothing')
        self.assertTrue(
            _poll_apps_present(PORT, self.ONLINE_APP_ID, timeout=PRESENCE_TIMEOUT_SEC),
            f'{self.ONLINE_APP_ID} never appeared on GET /apps - without at least one '
            'online app this launch gives the arming gate nothing to report on',
        )
        self.assertTrue(
            _poll_apps_present(PORT, self.NEVER_ONLINE_APP_ID, timeout=PRESENCE_TIMEOUT_SEC),
            f'{self.NEVER_ONLINE_APP_ID} is not even in the manifest-derived snapshot - '
            'this scenario needs it present-but-offline, not absent from discovery '
            'entirely, or the trigger it is named for was never set up',
        )

        assert_fault_absent_throughout(self, PORT, FAULT_CODE, SUSTAINED_WINDOW_SEC)


class TestNodeDeathRos2cliIgnored(unittest.TestCase):
    """A node named like ros2cli's own hidden-node convention is never tracked or raised.

    What node_death actually implements for this case is a NAME test: it takes the leaf
    after the last ``/`` and checks whether it starts with
    ``_ros2cli_``. Nothing in that check knows or cares that a real `ros2` CLI process
    produced the node - so the fixture that measures it should not depend on one either.

    An earlier version of this scenario drove a real ``ros2 topic echo`` process for
    every cycle and polled for its ephemeral ``_ros2cli_<pid>`` node. That instrument was
    wrong for what it was measuring: it put the ros2cli daemon (`NodeStrategy`'s
    daemon-vs-direct-node dispatch), ros2cli's own entry-point process layout, and
    ephemeral-process DDS discovery timing all between the test and the one branch this
    scenario is actually about - three sources of failure that have nothing to do with
    the claim. Verified live, repeatedly: cycles failed in different, unrelated-looking
    ways across separate runs (a wrong default spin-time, a bare-vs-FQN mismatch, then -
    even after both of those were fixed and the mechanism rebuilt around a persistent
    rclpy client node - a specific cycle timing out waiting for presence, and which cycle
    failed was not even consistent between runs). That instability was in the instrument,
    not in anything this suite is trying to prove.

    test_01 (load-bearing) replaces it: an ordinary, long-lived `demo_engine_temp_sensor`
    process, renamed via ``-r __node:={ROS2CLI_FAKE_NODE_PREFIX}<cycle>`` so the ONLY
    thing distinguishing it from any other target node in this package is its name. Being
    long-lived and entirely under this test's own control (started and killed on
    purpose, not exiting on its own schedule), it can be armed and gated exactly like
    every other scenario's target - something an ephemeral CLI subprocess's short, racy
    lifetime never allowed. Three cycles, three distinct names, each proven present and
    armed BEFORE being killed, so a kill never lands on a node the gate never saw.

    This scenario's gateway launches with `discovery.runtime.filter_internal_nodes`
    explicitly OFF (see `generate_test_description`). Left at its true default (on), a
    ``_ros2cli_``-named node would be invisible upstream of everything - GET /apps, the
    entity cache, any `IntrospectionProvider` plugin's own input - which would make this
    scenario pass for the exact wrong reason: never tracked at all, rather than tracked
    and then correctly excluded by name. This scenario is about the detector's OWN
    name-based exclusion (belt-and-braces on top of,
    not a substitute for, the gateway's separate and already-covered generic filter), and
    that requires a node the detector's input can actually see.

    test_02 does not drive a process at all. Two earlier attempts to keep a live realism
    check both failed to hold up on this stack: a per-cycle ``ros2 topic echo`` (one real
    CLI process per cycle) failed in different, unrelated-looking ways across separate
    runs (see above), and a single, demoted invocation with a 60 s budget - this method's
    own prior version - still did not observe its ephemeral node appear in the ROS graph
    within that budget. What test_01's fixture actually needs to stay honest is not proof
    that a live CLI process exists, but proof that ROS2CLI_FAKE_NODE_PREFIX is still built
    from the same prefix the installed `ros2cli` package would use - a plain constant
    comparison against `ros2cli.node.NODE_NAME_PREFIX`, deterministic and immediate, that
    fails loudly the day the convention changes instead of quietly testing a prefix
    nothing real uses anymore.

    The tracked-count half of test_01's claim compares two STABLE reads
    (_poll_stable_tracked_count), not two single samples: node_death is zero-config and
    tracks every armed App in the graph, not just this scenario's own three renamed
    fixtures, and with `discovery.runtime.filter_internal_nodes` off (this scenario's own
    requirement, see above) that includes gateway-internal hidden nodes too. Confirmed live:
    the gateway keeps a hidden `_param_client_node` for querying newly-discovered nodes'
    parameters - present from early on, but still inside its OWN per-entity warmup, and so
    briefly absent from tracked_count, at the moment a bare single sample happens to land. A
    `before`/`after` pair of single samples straddling that node's own warmup completion
    would read as tracked_count growth that has nothing to do with the ros2cli exclusion
    this row is actually about - `_poll_stable_tracked_count` requires the value to hold for
    a continuous stable_seconds specifically so that kind of late arrival is waited out
    before either sample is trusted, not merely smoothed over. If
    `watchdog_detector_status(PORT, DETECTOR_ID)` returns None on every read - no
    `detectors.node_death` block at all, meaning no such detector is registered - that
    counts as "stable at None" and is the one condition allowed to skip the comparison
    entirely; once a detector exists the block stops being None and a subsequent
    disappearance, a malformed shape, or a tracked_count that never actually settles fails
    loudly instead of being silently discarded (see test_01's own comments).
    """

    def _spawn_renamed_node(self, name):
        """Start an ordinary demo node renamed to carry the ros2cli hidden-node prefix.

        `demo_engine_temp_sensor` (`DEMO_NODE_REGISTRY`'s 'temp_sensor' executable) needs
        no parameters to start; ``-r __node:={name}`` is a plain ROS 2 remap, not a
        ros2cli mechanism.

        Invokes the installed binary directly - NOT ``ros2 run`` - so the returned
        `Popen`'s own pid IS the node's pid. `ros2run.api.run_executable`
        (confirmed directly against the installed `ros2run` package, not assumed) spawns
        the target executable as ITS OWN child via a second `subprocess.Popen`, and only
        ever forwards `KeyboardInterrupt` (SIGINT) to it; nothing in that function
        forwards SIGTERM. A SIGTERM sent to a `ros2 run` wrapper's own pid therefore
        kills only the wrapper - by Python's default disposition, not any handler ros2run
        installs - and orphans the real node underneath it, which is then never signaled
        at all and never leaves the graph. Reproduced live: an earlier version of this
        method went through `ros2 run` and left an actual `demo_engine_temp_sensor`
        process running with ppid 1 (reparented to init) for over an hour after its
        scenario's own SIGTERM. The process this spawns instead knows nothing about
        ros2cli or ros2run - it lives exactly as long as the caller lets it, and a
        SIGTERM reaches it directly, the same way every other scenario in this file kills
        ITS target node.
        """
        exe = os.path.join(
            get_package_prefix('ros2_medkit_integration_tests'), 'lib',
            'ros2_medkit_integration_tests', 'demo_engine_temp_sensor')
        return subprocess.Popen(
            [exe, '--ros-args', '-r', f'__node:={name}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def test_01_renamed_node_matching_the_convention_is_never_tracked(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC),
            'graph_watchdog never reported an armed global state')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - the absence below would prove nothing')

        before, before_stable = _poll_stable_tracked_count(
            PORT, DETECTOR_ID, timeout=STABLE_TRACKED_COUNT_TIMEOUT_SEC)
        self.assertTrue(
            before_stable or before is None,
            f'tracked_count never settled before the renamed-node cycles even started (last '
            f'read: {before!r}) - the before/after comparison below would measure a moving '
            "target, not this row's own claim",
        )
        for cycle in range(1, ROS2CLI_CYCLES + 1):
            name = f'{ROS2CLI_FAKE_NODE_PREFIX}{cycle}'
            proc = self._spawn_renamed_node(name)
            try:
                self.assertTrue(
                    _poll_apps_present(PORT, name, timeout=PRESENCE_TIMEOUT_SEC),
                    f'cycle {cycle}: {name} never appeared on GET /apps even with '
                    'discovery.runtime.filter_internal_nodes off for this scenario - '
                    'the arm/kill sequence below would prove nothing about the '
                    'detector, only that this node failed to start or be discovered',
                )
                # app_id=name, not the global gate: see TestNodeDeathRaise's identical
                # gate for why the global gate is not enough - it can go true from
                # something else in the graph while this cycle's own node has not been
                # read even once.
                self.assertTrue(
                    wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=name),
                    f'cycle {cycle}: graph_watchdog never reported {name} armed - '
                    'killing it before that would be a kill a correct detector was '
                    'never permitted to see',
                )

                os.kill(proc.pid, signal.SIGTERM)
                self.assertTrue(
                    _poll_apps_absent(PORT, name, timeout=DEPARTURE_TIMEOUT_SEC),
                    f'cycle {cycle}: {name} never left GET /apps after SIGTERM - this '
                    'cycle never produced the departure it is about',
                )
                proc.wait(timeout=15)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=15)
        after, after_stable = _poll_stable_tracked_count(
            PORT, DETECTOR_ID, timeout=STABLE_TRACKED_COUNT_TIMEOUT_SEC)
        # `before` reads None only if GET /x-medkit-watchdog carries no `detectors.node_death`
        # block at all - the ONLY condition allowed to skip the comparison, kept for
        # structural symmetry with `_poll_stable_tracked_count`'s own "stable at None"
        # contract, not because it is expected here. With the detector registered, `before`
        # is populated and the comparison below always runs: checking `before is not None`
        # alone (not also `after is not None`) is what makes a status block that vanishes or
        # loses its shape MID-scenario fail loudly instead of being discarded along with the
        # genuinely-inapplicable case.
        if before is not None:
            self.assertTrue(
                after_stable,
                f'tracked_count never settled after the renamed-node cycles (last read: '
                f'{after!r}) - cannot tell whether the ros2cli exclusion held or something '
                'else in the graph is simply still arming, so reporting this rather than '
                'comparing against a moving target',
            )
            self.assertIsNotNone(
                after,
                f'the node_death detector status block was present before the '
                f'renamed-node cycles ({before!r}) and is gone after them - it '
                'disappeared mid-scenario',
            )
            self.assertIn(
                'tracked_count', after,
                f"the node_death detector status block lost its 'tracked_count' field "
                f'after the renamed-node cycles ({before!r} -> {after!r})',
            )
            if before.get('tracked_count') != after.get('tracked_count'):
                try:
                    apps_url = f'http://127.0.0.1:{PORT}{API_BASE_PATH}/apps'
                    apps_now = requests.get(apps_url, timeout=5).json().get('items', [])
                    ids_now = [item.get('id') for item in apps_now]
                except requests.exceptions.RequestException as exc:
                    ids_now = f'GET /apps failed: {exc}'
                self.fail(
                    f'the node_death detector status block reports a different tracked_count '
                    f'after {ROS2CLI_CYCLES} renamed-node arm/kill cycles ({before!r} -> '
                    f'{after!r}) - GET /apps right now: {ids_now!r}',
                )

        assert_fault_absent_throughout(self, PORT, FAULT_CODE, SUSTAINED_WINDOW_SEC)

    def test_02_the_fake_prefix_still_matches_ros2clis_own_convention(self):
        """Pins test_01's fixture to ros2cli's OWN naming convention, not a copy of it.

        What this protects against: test_01 kills nodes it names itself
        (``ROS2CLI_FAKE_NODE_PREFIX`` + a cycle number). If the installed `ros2cli`
        package ever changed what prefix it hands its own hidden nodes, test_01 would
        keep passing - proving a detector ignores a prefix WE made up - while the actual
        convention moved out from under it, and `node_death`'s exclusion would silently
        stop matching reality. Nothing else in this scenario would notice that drift.

        Reads the installed package rather than driving a process because a live check
        was tried first, twice, and neither held up on this stack: a per-cycle ``ros2
        topic echo`` (one real CLI process per cycle, three cycles) failed in different,
        unrelated-looking ways across separate runs (a wrong default spin-time, a
        bare-vs-FQN mismatch, then - even after both were fixed - a specific cycle timing
        out waiting for presence, inconsistently between runs); a single, demoted
        invocation with a 60 s budget (this method's own prior version) still did not
        observe its ephemeral node appear in the ROS graph within that budget. Both tries
        were proving a live process exists, which this scenario does not actually need -
        it needs to know what NAME ros2cli would give that process, and that is a plain
        constant lookup, not a fact about a live graph. Deterministic, no subprocess, no
        discovery, and it fails loudly the day the convention changes - exactly what the
        two process-driving attempts before it were for, and could not reliably do here.

        `rclpy.node.HIDDEN_NODE_PREFIX` is the general ROS 2 hidden-node convention
        (``'_'``); `ros2cli.node.NODE_NAME_PREFIX` is ros2cli's own
        ``HIDDEN_NODE_PREFIX + 'ros2cli'`` - both imported from the installed packages,
        not hardcoded here, so this check tracks them automatically. See also
        `ros2cli.node.direct.DirectNode`, which builds an actual CLI invocation's node
        name as ``NODE_NAME_PREFIX + '_%d' % os.getpid()`` - the shape both live attempts
        above were trying, and failing, to observe.
        """
        self.assertEqual(
            NODE_NAME_PREFIX, HIDDEN_NODE_PREFIX + 'ros2cli',
            f'ros2cli.node.NODE_NAME_PREFIX ({NODE_NAME_PREFIX!r}) is no longer built '
            f"from rclpy's own hidden-node prefix ({HIDDEN_NODE_PREFIX!r}) the way this "
            'test assumed - the composition itself changed, not just the value',
        )
        self.assertTrue(
            ROS2CLI_FAKE_NODE_PREFIX.startswith(NODE_NAME_PREFIX),
            f'test_01 kills nodes named {ROS2CLI_FAKE_NODE_PREFIX!r}<cycle>, but the '
            f'installed ros2cli package now builds its own hidden node names from '
            f'{NODE_NAME_PREFIX!r} - these no longer match, so test_01 passing would no '
            'longer mean what this scenario claims it means',
        )


class TestNodeDeathBareNameCollision(unittest.TestCase):
    """Two nodes sharing a bare name in different namespaces; one exits.

    The fault names the one that exited and does not name the one still running - proven
    with assert_fault_describes_only so the claim holds for the WHOLE window a scenario
    watches it, not one lucky read taken before a second tick could have widened (or
    corrupted) the description.

    Required/forbidden are the two nodes' FULLY QUALIFIED names, not their discovery-layer
    App ids. The collision-avoidance rule that gives the two colliding nodes their
    namespace-prefixed ids (`ros2_runtime_introspection.cpp`) is re-evaluated on every
    discovery sweep from whichever bare names are CURRENTLY duplicated - it is not a
    property assigned once and kept. Once node_a departs, only one 'calibration' remains,
    the collision that justified prefixing it resolves, and node_b's own id reverts to the
    bare 'calibration' - confirmed live: GET /apps read `coll_a_calibration` and
    `coll_b_calibration` while both were up, then just `calibration` once node_a was gone.
    A forbidden needle keyed to an id that stops existing the moment the departure this
    scenario is about actually happens would not discriminate anything. The FQN carries no
    such collision-avoidance and stays valid for both nodes throughout.
    """

    def test_dead_one_named_alive_one_not(self, node_a, node_b):
        # Both ids, not the global gate: this scenario perturbs (kills) node_a and its
        # claim is about node_b too ("does not name the one still running"), so both must
        # have been read by the detector before node_a dies - see TestNodeDeathRaise's
        # identical gate for why the global gate alone would not prove that.
        for app_id in (COLLISION_APP_ID_A, COLLISION_APP_ID_B):
            self.assertTrue(
                wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=app_id),
                f'graph_watchdog never reported {app_id} armed - the discovery layer did '
                'not namespace-disambiguate the bare-name collision the way this scenario '
                'assumes, or one of the two fixtures never came up, or it did but was '
                'never read',
            )

        node_b_pid = node_b.process_details['pid']
        os.kill(node_a.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, COLLISION_APP_ID_A, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{COLLISION_APP_ID_A} never left GET /apps after SIGTERM')
        # node_b's OWN discovery-layer id is not stable across node_a's departure (see the
        # class docstring), so its continued survival is checked at the OS level instead -
        # the one identity that does not shift when the collision this scenario sets up
        # resolves.
        try:
            os.kill(node_b_pid, 0)
        except ProcessLookupError:
            self.fail(
                f'node_b (pid {node_b_pid}) is gone too - only node_a was meant to die, '
                'so the pair no longer discriminates')

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(fault, f'{FAULT_CODE} never raised after {COLLISION_APP_ID_A} exited')

        assert_fault_describes_only(
            self, PORT, FAULT_CODE,
            required=[f'{COLLISION_NAMESPACE_A}/{TARGET_NODE}'],
            forbidden=[f'{COLLISION_NAMESPACE_B}/{TARGET_NODE}'],
            duration=MUTUAL_NAMING_WINDOW_SEC)


class TestNodeDeathFastTickFloor(unittest.TestCase):
    """A very short tick_interval_ms, with the node continuously present, raises nothing.

    Narrower than this row's own name suggests, and deliberately so - see below. Absence
    alone cannot distinguish a detector that is correctly silent here from one that raises
    nothing at all - see TestNodeDeathDeactivatedNotDead's own note.

    What this scenario does NOT prove: that the documented miss_grace floor (config sweep
    C1's own concern) is what stands between a fast tick and a false raise. The row exists
    because a fast-ticking detector could read the SAME stale entity-cache generation many
    times before the cache refreshes, and - absent the floor - a naive per-tick counter
    would count each of those reads as an independent miss rather than one. This scenario
    never forces that condition: the node is present throughout and the cache is never
    caught between a real departure and a real return, so there is no stale generation to
    misread in the first place. A detector built WITHOUT the floor would pass this
    scenario just as easily as one built with it, as long as no such staleness happened to
    occur in the window - which, with nothing perturbing the graph, it never does.

    Investigated, not assumed, whether the precondition can be forced deterministically.
    Traced the real mechanism in `gateway_node.cpp`: the entity cache refreshes on a ROS
    graph event, coalesced to at most one refresh per `discovery.refresh_debounce_ms`
    (default 1000 ms) via `graph_check_timer_`/`decide_graph_refresh` - PLUS an
    independent, unconditional `backstop_timer_` on its own `refresh_interval_ms` cadence,
    which the shared `create_gateway_node()` factory pins to 1000 ms for every launch in
    this test suite. Both are overridable via `extra_gateway_params`, which in principle
    opens a multi-second window: kill the node, poll GET /apps until the departure is
    reflected (proving a refresh already consumed the debounce budget), then respawn it
    by hand (not launch's own `respawn=True`, whose `RESPAWN_DELAY_SEC` floor is
    calibrated to be slow elsewhere in this file and is the wrong direction here) fast
    enough to land inside the same now-widened window, and confirm the new process is
    genuinely alive via a DIRECT service call that bypasses the gateway entirely while
    GET /apps still reports it absent.

    Not implemented. Two reasons: (1) it is a real race, not a guaranteed sequence -
    "deterministic" has to mean reliable across CI hardware, and nothing pins how long DDS
    discovery takes to converge on the manually-respawned process relative to the widened
    window, only that it is likely to fit; (2) winning the race would require new
    subprocess-lifecycle code (resolving the demo executable's install path, building its
    `--ros-args` remapping by hand, coverage-env parity with every other fixture in this
    file, guaranteed cleanup on a failed assertion) whose own bugs could leak an orphaned
    process - exactly what this package's own test discipline warns leaks into the NEXT run
    as a false regression. Forcing the precondition would exercise a real detector rather
    than prove something trivially true either way, which is what makes the remaining cost
    purely about race reliability and fixture risk, not about whether the result would mean
    anything.
    """

    def test_fast_tick_alone_never_raises(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC),
            'graph_watchdog never reported an armed global state')
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 - the absence below would prove nothing')
        self.assertTrue(
            _poll_apps_present(PORT, 'calibration', timeout=PRESENCE_TIMEOUT_SEC),
            "'calibration' never appeared on GET /apps - there is no present node here "
            'whose non-death this scenario is about',
        )

        assert_fault_absent_throughout(self, PORT, FAULT_CODE, SUSTAINED_WINDOW_SEC)

        self.assertTrue(
            _poll_apps_present(PORT, 'calibration', timeout=5.0),
            "'calibration' is no longer present at the END of the silence window - the "
            'trigger (a present node, ticked over rapidly) did not survive it',
        )


class TestNodeDeathRestartLoopOccurrences(unittest.TestCase):
    """A node killed and restarted three times: occurrence_count reaches 3.

    Each cycle: kill, wait for the fault to CONFIRM (a fresh occurrence), wait for the node
    to come back, THEN explicitly acknowledge it via DELETE
    {SOURCE_ENTITY_PATH}/faults/{code} - the fault manager's own boundary between
    occurrences: `~/clear_fault` IS the acknowledge, not a deletion, and a FAILED event
    reactivating a CLEARED record is what the fault manager counts as a new occurrence
    (`fault_storage.cpp`'s own comment: "a new outage cycle, not a continuation of the one
    that just cleared"). A level-triggered heal on return would NOT do this - reconfirming
    from HEALED (rather than from CLEARED) does not bump occurrence_count at all
    (`fault_storage.cpp`: "a re-report on a still-active fault... the same continuous
    occurrence"), which is why this scenario acknowledges explicitly between cycles instead
    of waiting for an organic heal.

    Deliberately does NOT assert the "more than one recording" half of this row. The
    per-fault rosbag store enforces `fault_code` as UNIQUE (`sqlite_fault_storage.cpp`'s
    `store_rosbag_file_locked`: "INSERT OR REPLACE INTO rosbag_files ... (fault_code is
    UNIQUE)" - the SAME fault_code can hold at most one recording ROW, structurally, and a
    re-confirm deletes the previous bag file from disk). No config key in the fault manager
    lifts this cap today: recording more than one rosbag per fault_code needs the storage
    schema itself to change. Asserting a recording count here would either assert something
    trivially true for the wrong reason (no captures happen at all without a detector) or
    something structurally impossible to ever pass - neither is written.
    """

    def test_repeated_kills_reach_matching_occurrence_count(self, target_node):
        # app_id=TARGET_NODE: see TestNodeDeathRaise's identical gate for why the global
        # gate alone is not enough before a kill. Re-checked before every LATER kill too
        # (below), since a respawned instance needs its own read just as much as the first.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')

        pid = target_node.process_details['pid']
        for cycle in range(1, RESTART_LOOP_OCCURRENCES_TARGET + 1):
            os.kill(pid, signal.SIGTERM)
            self.assertTrue(
                _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
                f'cycle {cycle}: {TARGET_NODE} never left GET /apps after SIGTERM',
            )
            self.assertTrue(
                _poll_occurrence_count(PORT, FAULT_CODE, cycle, timeout=RAISE_TIMEOUT_SEC),
                f'cycle {cycle}: {FAULT_CODE} never reached occurrence_count={cycle}',
            )

            if cycle < RESTART_LOOP_OCCURRENCES_TARGET:
                # Wait for the RETURN before acknowledging - not merely a style choice:
                # clearing while the node is still absent leaves a window in which a live
                # detector's own continued FAILED reports (the SAME departure, still being
                # measured) would reactivate the just-cleared record too, one cycle early.
                # Waiting for the return first means every reactivation this loop measures
                # is attributable to the NEXT kill, not a race against this one's tail end.
                # app_id-scoped again, not just present: the respawned instance is a NEW
                # process the detector has not read yet, and the next iteration is about
                # to kill it - the same precondition the FIRST kill above needed.
                self.assertTrue(
                    wait_until_watchdog_armed(
                        PORT, timeout=DEPARTURE_TIMEOUT_SEC + RESPAWN_DELAY_SEC,
                        app_id=TARGET_NODE),
                    f'cycle {cycle}: {TARGET_NODE} never came back armed after the SIGTERM - '
                    'launch did not respawn it, or the respawned instance was never read, '
                    'so this is a single departure, not a loop',
                )
                self.assertTrue(
                    _clear_fault(PORT, SOURCE_ENTITY_PATH, FAULT_CODE),
                    f'cycle {cycle}: DELETE /{SOURCE_ENTITY_PATH}/faults/{FAULT_CODE} '
                    'did not acknowledge the fault - the next kill could not reactivate '
                    'it as a fresh occurrence',
                )
                # A fresh pid every cycle: read it back rather than assume launch's
                # respawn keeps the value this test already has.
                pid = target_node.process_details['pid']

        record = _fault_record(PORT, FAULT_CODE, timeout=DEPARTURE_TIMEOUT_SEC)
        if record is None:
            self.fail(f'{FAULT_CODE} vanished from the store entirely')
        self.assertEqual(
            record.get('occurrence_count'), RESTART_LOOP_OCCURRENCES_TARGET,
            f'{FAULT_CODE} ended the loop with occurrence_count='
            f'{record.get("occurrence_count")!r}, not {RESTART_LOOP_OCCURRENCES_TARGET}',
        )


class TestNodeDeathRestartRebaseline(unittest.TestCase):
    """A gateway restart with a death outstanding: the re-baseline boundary is pinned.

    RECORDS a boundary, the way TestLifecycleExpectationRestartDeparted does for the
    sibling detector - this test does not fix anything. Pin: the fault stays CONFIRMED
    across the restart, proven as a SUSTAINED claim (assert_fault_persists_throughout),
    not a single sample that could just mean "not yet re-evaluated".

    Reasoning, on file rather than assumed: node_death has no forced state transition - a
    violation's evidence closes only when the graph genuinely recovers, or an operator
    explicitly acknowledges it. A gateway restart is neither: the node is
    STILL gone, and the freshly-started detector process, having never observed it
    present in this lifetime, has nothing to report either way. Unlike
    lifecycle_expectation's require_active (an explicit, possibly-mistyped list that has
    to eventually give up on an entry that never matches anything, or a typo would block
    healing forever), node_death is zero-config: it has no static entries to protect
    against typos in the first place, so it has no equivalent reason to ever manufacture a
    clear for a node it has simply never seen.
    """

    def test_01_the_fault_raises_and_survives_the_node_leaving(self, target_node):
        # app_id=TARGET_NODE: see TestNodeDeathRaise's identical gate for why the global
        # gate alone is not enough before a kill. test_02 below gates globally instead -
        # by then TARGET_NODE is gone BY DESIGN, so an app_id gate on it could never
        # succeed; what test_02 checks is that the PLUGIN itself is back up post-restart,
        # not that this specific (permanently absent) node is.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=ARM_TIMEOUT_SEC, app_id=TARGET_NODE),
            f'graph_watchdog never reported {TARGET_NODE} armed')

        os.kill(target_node.process_details['pid'], signal.SIGTERM)
        self.assertTrue(
            _poll_apps_absent(PORT, TARGET_NODE, timeout=DEPARTURE_TIMEOUT_SEC),
            f'{TARGET_NODE} never left GET /apps after SIGTERM')

        fault = poll_faults(PORT, FAULT_CODE, timeout=RAISE_TIMEOUT_SEC)
        self.assertIsNotNone(fault, f'{FAULT_CODE} never raised for the departed {TARGET_NODE}')

        record = _fault_record(PORT, FAULT_CODE, timeout=DEPARTURE_TIMEOUT_SEC)
        if record is None:
            self.fail(f'{FAULT_CODE} vanished from the store')
        self.assertIsNone(
            record.get('last_passed'),
            f'{FAULT_CODE} was reported PASSED before the restart '
            f'(last_passed={record.get("last_passed")!r}) - there is nothing left here for '
            'the restart below to wrongly preserve or wrongly heal',
        )

    def test_02_a_gateway_restart_does_not_manufacture_a_clear(self, gateway_node, target_node):
        old_pid = gateway_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _wait_until_port_is_down(PORT, timeout=60.0),
            f'the gateway (pid {old_pid}) kept answering after SIGTERM - nothing restarted')
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=90.0),
            'the gateway never came back armed after the restart')
        self.assertNotEqual(
            gateway_node.process_details['pid'], old_pid,
            'the gateway process id did not change, so this test never restarted anything')
        # The armed gate is served by the plugin INSIDE the restarted gateway process and
        # proves nothing about its connection to the fault_manager - a SEPARATE process
        # that survived the restart, but whose services the freshly restarted gateway has
        # to rediscover from scratch. Without this, the persistence window below could
        # start against a /faults that has not reconnected yet and fail for that reason
        # instead of the boundary this test is actually pinning.
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=FAULTS_LIVE_TIMEOUT_SEC),
            'GET /faults never answered 200 after the restart - the restarted gateway '
            'never reconnected to the fault_manager, so the persistence window below '
            'would prove nothing',
        )
        self.assertFalse(
            _poll_apps_present(PORT, TARGET_NODE, timeout=5.0),
            f'{TARGET_NODE} is back in GET /apps - the boundary this test pins is about '
            'a restart with the node STILL gone, and it came back instead',
        )

        # The pin: sustained across a real window post-restart, not a single sample.
        assert_fault_persists_throughout(self, PORT, FAULT_CODE, SUSTAINED_WINDOW_SEC)

        record = _fault_record(PORT, FAULT_CODE, timeout=DEPARTURE_TIMEOUT_SEC)
        if record is None:
            self.fail(f'{FAULT_CODE} vanished from the store after the restart')
        self.assertIsNone(
            record.get('last_passed'),
            f'the restarted gateway reported {FAULT_CODE} PASSED for a node it can never '
            f'have observed in this process (last_passed={record.get("last_passed")!r}) - '
            'the boundary this test pins no longer holds',
        )


# Each CTest target launches this file with one scenario, so only that scenario's case may
# run. Removing the others from the module (rather than skipping them) means each run
# reports exactly one case, and a missing result is a real failure rather than an expected
# line of output - see test_lifecycle_expectation_e2e.test.py's identical rationale.
_SCENARIO_CASES = {
    'raise': 'TestNodeDeathRaise',
    'clear_on_return': 'TestNodeDeathClearOnReturn',
    'no_heal_standalone': 'TestNodeDeathNoHealStandalone',
    'deactivated_not_dead': 'TestNodeDeathDeactivatedNotDead',
    'manifest_never_online': 'TestNodeDeathManifestNeverOnline',
    'ros2cli_ignored': 'TestNodeDeathRos2cliIgnored',
    'bare_name_collision': 'TestNodeDeathBareNameCollision',
    'fast_tick_floor': 'TestNodeDeathFastTickFloor',
    'restart_loop_occurrences': 'TestNodeDeathRestartLoopOccurrences',
    'restart_rebaseline': 'TestNodeDeathRestartRebaseline',
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
