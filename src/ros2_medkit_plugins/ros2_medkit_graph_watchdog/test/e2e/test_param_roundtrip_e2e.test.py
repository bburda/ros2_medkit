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
"""What one parameter read really costs the node that serves it.

``param_drift`` charges its ``max_reads_per_tick`` budget in parameter-service ROUND TRIPS
and not in transport calls, because one call is several round trips at the watched node:
``kBaselineReadRoundTrips`` (2), ``kExpectReadRoundTrips`` (3) and
``kExpectNotFoundRoundTrips`` (2) in ``src/detectors/param_drift_detector.cpp``. Every other
test in this package proves the detector's arithmetic GIVEN those numbers, and the stand-in
transport they use counts CALLS, so not one of them can tell whether the TRANSPORT still costs
what those numbers say. Add a fourth service call inside ``Ros2ParameterTransport`` and the
whole suite stays green while the load bound this feature sells quietly becomes a lie.

This test measures the transport. It runs the REAL one - the gateway compiles the same
``ros2_parameter_transport.cpp`` the plugin does (see ``PARAMETER_TRANSPORT_SOURCES`` in
CMakeLists.txt) - against a node that answers the parameter services BY HAND and counts every
request it is asked to serve. The counts are what the node paid, not what anything claims it
paid.

What it does NOT do, and must not be relied on for, is check the detector's own literals. It
never reads them: ``param_drift`` is switched OFF here and the reads are driven over HTTP, so
setting ``kBaselineReadRoundTrips`` to 4 and ``kExpectReadRoundTrips`` to 6 with the transport
untouched leaves this file green. What catches a CHARGE that has drifted away from the
measurement is the timing suite in ``test/test_param_drift_integration.cpp``
(``ABaselineVisitIsChargedTwoRoundTripsAndNotTheColdStartPair``,
``TheBudgetIsSpentInRoundTripsNotCalls``, ``AnUndeclaredPinIsChargedLessThanOneThatResolves``),
which times the reader against a configured budget. The two halves are complementary and
neither is redundant: this file pins the cost, those pin the charge against it.

The gateway is launched here with a 5 s per-call timeout and the negative cache disabled, while
the detector builds its transport with 0.5 s and a 10 s cache. That difference does not affect
anything measured below: a round-trip count on the SUCCESS path is the number of requests the
transport issues, which is the same whatever the deadline on each of them is. The generous
settings exist only so a slow first DDS discovery cannot turn a success into a TIMEOUT, or lock
a probe out for a minute after one early attempt, and so make the first-contact measurements
repeatable.

Why the reads are driven over HTTP rather than by the detector. The constants describe two
functions, ``Ros2ParameterTransport::list_parameters`` and ``::get_parameter``, and
``GET /{entity}/configurations`` / ``GET /{entity}/configurations/{id}`` invoke exactly those,
once per backing node, synchronously, on demand (``ConfigurationManager`` delegates 1:1). That
is what makes a count attributable to ONE call. The detector reaches the same two functions
from a background round-robin sweep that never stops and exposes no visit boundary, so driving
it would only ever yield an increment pattern smeared over the sweep's pacing - a weaker claim
about the same code. ``param_drift`` is therefore switched OFF here, so the only parameter
traffic the probe node sees is the traffic this test asks for.

The four costs pinned below, and how they relate to what the detector charges:

* cold baseline visit = 4 - ``list_parameters`` primes the transport's defaults cache before
  its own read (list + get, then list + get). That cache has no TTL, so this is a one-off per
  node per gateway lifetime, and the detector deliberately does NOT charge it.
* warm baseline visit = 2 - matches ``kBaselineReadRoundTrips``.
* resolved ``expect`` pin = 3 - matches ``kExpectReadRoundTrips`` (name-scoped list, get,
  describe; describe is unconditional on the success path).
* unresolved ``expect`` pin = 1 - the name-scoped list comes back empty and the transport
  returns NOT_FOUND before the get. The detector charges 2 for this case ON PURPOSE: a second,
  rarer NOT_FOUND path costs two round trips and the error code cannot tell the two apart, so
  it charges the higher of them. A bound may overstate a cost, never understate one - so 1 here
  is the real cost and is ALLOWED to be below the charge. Do not "fix" either number to match
  the other.
* a listed name the get returns no value for = 2 - that second NOT_FOUND path, measured, so the
  charge of 2 is anchored from above as well as below.
* a DECLARED BUT UNSET parameter = 3, and a SUCCESS rather than a NOT_FOUND. rclcpp answers one
  with a ``PARAMETER_NOT_SET`` value, not with an absent one, so the transport gets a parameter
  back and pays for the describe like any other resolved pin. It is charged 3, which is right.
  This case is pinned separately because the two-round-trip path above was previously described
  as being it.
* FIRST resolved ``expect`` pin against an untouched node = 3, the same as any later one.
  ``get_parameter`` never primes the defaults cache, so the cold-contact pair is a property of
  ``list_parameters`` and of ``baseline: true`` mode, not of reading a node for the first time.
  Measured against a second probe that nothing has listed.
"""

import os
import sys
import threading
import time
import unittest

import launch_testing
from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType,
    ParameterValue,
    SetParametersResult,
)
from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    GetParameterTypes,
    ListParameters,
    SetParameters,
    SetParametersAtomically,
)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import create_watchdog_test_launch  # noqa: E402

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port  # noqa: E402

PORT = get_test_port()
API_BASE = f'http://127.0.0.1:{PORT}/api/v1'

PROBE_NODE = 'pd_rt_probe'
# A SECOND probe, identical but never listed. The cold-contact pair belongs to
# list_parameters (which primes the transport's defaults cache) and not to get_parameter
# (which does not), so proving that needs a node whose cache is still unprimed when the pinned
# read reaches it - which the first probe stops being the moment test_02 lists it.
PIN_PROBE_NODE = 'pd_rt_pinprobe'
# Two declared parameters, so the batched get after a full list carries more than one name and
# a regression that silently drops the batch cannot pass by returning a single value.
DECLARED_PARAMS = {'probe_gain': 2.5, 'probe_margin': 0.5}
# A name the probe never declares: the name-scoped list answers empty and the transport must
# stop there.
UNDECLARED_PARAM = 'pd_rt_no_such_parameter'
# A name the probe lists and then returns NO VALUE for. This is the second, rarer NOT_FOUND path
# and the whole justification for charging NOT_FOUND two round trips rather than one: the
# name-scoped list answers, so the transport goes on to the get, and only the get finds there is
# nothing there. The error code is the same NOT_FOUND as the one-round-trip case and the caller
# cannot tell which it took, so the charge takes the higher of the two.
#
# The GetParameters contract does not require one value per requested name, so a reply that simply
# omits it is well formed - and it is the only reply that reaches this branch. See NOT_SET_PARAM
# for the case this was previously described as, which behaves quite differently.
NO_VALUE_PARAM = 'pd_rt_no_value_returned'
# A name the probe lists and answers with a PARAMETER_NOT_SET value: a parameter that is DECLARED
# BUT UNSET, which is what rclcpp's own parameter service replies for one. That reply is a SUCCESS
# carrying a null value, not a NOT_FOUND: the transport gets a parameter back, goes on to the
# describe, and returns it. It therefore costs 3 and is charged 3 like any other resolved pin.
# Pinned here because the 2-round-trip NOT_FOUND above used to be documented as this case, which
# would have made the charge for a resolved-but-null read an understatement of what the node pays.
# Measured on the SECOND probe, whose full list nothing asserts on; adding either name to the
# first would change what test_02 and test_03 read back.
NOT_SET_PARAM = 'pd_rt_declared_but_unset'

# Per-request HTTP patience. Generous enough for a first request that races gateway startup, short
# enough that several of them still fit inside the ctest timeout with the retry deadlines below.
HTTP_TIMEOUT = 15

# The six services rclcpp's AsyncParametersClient requires before it reports ready. Only three
# of them are ever expected to carry traffic; the rest exist so wait_for_service() succeeds,
# and are counted anyway - a transport that started calling one of them would show up here
# rather than hide inside a total.
SERVICE_NAMES = (
    'list_parameters',
    'get_parameters',
    'describe_parameters',
    'get_parameter_types',
    'set_parameters',
    'set_parameters_atomically',
)


def _double(value):
    """Build a well-formed DOUBLE ``ParameterValue``."""
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))


class _RequestFailed:
    """Stand-in for a response that never arrived.

    ``_measure`` is called from retry loops that decide what to do next from
    ``response.status_code``, and the HTTP call it wraps is reachable in precisely the state
    those loops exist for: a read that degenerates and outlives the client's patience. Letting
    ``requests`` raise out of it aborts the retry with a traceback rather than retrying, and none
    of the diagnostics the loop was written to print ever run - the failure report is then a stack
    trace about a socket instead of a statement about round trips.
    """

    status_code = None

    def __init__(self, exc):
        self.text = f'{type(exc).__name__}: {exc}'

    def json(self):
        raise AssertionError(f'there is no response body to parse: {self.text}')


class ParameterRequestCounter(Node):
    """A node that serves the parameter services by hand and counts what it is asked for.

    Its own rclcpp/rclpy parameter services are DISABLED: their traffic is unobservable from
    the outside, and counting is the entire point. The hand-rolled replacements answer on the
    names rclpy would have used, so the transport's ``AsyncParametersClient`` finds them
    exactly as it finds a normal node's.

    Every reply is well formed. A malformed one would send the transport down an early-return
    path, and the test would then be counting failures instead of the cost of a successful
    read - a green run that proves nothing.
    """

    def __init__(self, node_name, listed_extras=()):
        super().__init__(node_name, start_parameter_services=False)
        self._lock = threading.Lock()
        self._counts = dict.fromkeys(SERVICE_NAMES, 0)
        # Names the list reports beyond DECLARED_PARAMS. NO_VALUE_PARAM is omitted from the get
        # reply entirely; NOT_SET_PARAM is answered with a PARAMETER_NOT_SET value. Those two
        # replies take the transport down different paths and cost the node different amounts.
        self._listed_extras = tuple(listed_extras)
        fqn = self.get_fully_qualified_name()
        self._services = [
            self.create_service(ListParameters, f'{fqn}/list_parameters', self._on_list),
            self.create_service(GetParameters, f'{fqn}/get_parameters', self._on_get),
            self.create_service(
                DescribeParameters, f'{fqn}/describe_parameters', self._on_describe),
            self.create_service(
                GetParameterTypes, f'{fqn}/get_parameter_types', self._on_types),
            self.create_service(SetParameters, f'{fqn}/set_parameters', self._on_set),
            self.create_service(
                SetParametersAtomically,
                f'{fqn}/set_parameters_atomically',
                self._on_set_atomically),
        ]

    def snapshot(self):
        """Per-service request counts as of now."""
        with self._lock:
            return dict(self._counts)

    def _bump(self, service):
        with self._lock:
            self._counts[service] += 1

    def _on_list(self, request, response):
        self._bump('list_parameters')
        names = sorted(list(DECLARED_PARAMS) + list(self._listed_extras))
        if request.prefixes:
            # rcl's prefix rule: a name matches when it IS the prefix or lives under it. The
            # transport's name-scoped probe passes the bare parameter name, so an undeclared
            # name matches nothing and the reply is legitimately empty.
            names = [
                name for name in names
                if any(name == p or name.startswith(f'{p}.') for p in request.prefixes)
            ]
        response.result.names = names
        return response

    def _on_get(self, request, response):
        self._bump('get_parameters')
        # NO_VALUE_PARAM is omitted from the reply rather than answered NOT_SET. The service
        # contract does not require one value per requested name, and an omitted value is the only
        # reply that reaches the transport's second NOT_FOUND branch - the one that costs two round
        # trips and that kExpectNotFoundRoundTrips is sized for.
        response.values = [
            _double(DECLARED_PARAMS[name]) if name in DECLARED_PARAMS
            else ParameterValue(type=ParameterType.PARAMETER_NOT_SET)
            for name in request.names
            if name != NO_VALUE_PARAM
        ]
        return response

    def _on_describe(self, request, response):
        self._bump('describe_parameters')
        response.descriptors = [
            ParameterDescriptor(name=name, type=ParameterType.PARAMETER_DOUBLE)
            for name in request.names
        ]
        return response

    def _on_types(self, request, response):
        self._bump('get_parameter_types')
        response.types = [
            ParameterType.PARAMETER_DOUBLE if name in DECLARED_PARAMS
            else ParameterType.PARAMETER_NOT_SET
            for name in request.names
        ]
        return response

    def _on_set(self, request, response):
        self._bump('set_parameters')
        response.results = [
            SetParametersResult(successful=False, reason='probe is read-only')
            for _ in request.parameters
        ]
        return response

    def _on_set_atomically(self, request, response):
        self._bump('set_parameters_atomically')
        response.result = SetParametersResult(successful=False, reason='probe is read-only')
        return response


def generate_test_description():
    detector_params = {
        # The detector whose constants this test pins is switched OFF so that every request the
        # probe counts is one this test asked for. Its background sweep would otherwise read the
        # probe on its own schedule and no count would be attributable to a single call. `off`
        # keeps the detector out of the plugin's tick list entirely (see
        # graph_watchdog_plugin.cpp), so it never even builds a transport.
        'plugins.graph_watchdog.detectors.param_drift.mode': 'off',
    }
    gateway_params = {
        # A generous per-call timeout keeps a slow first DDS discovery from turning a real
        # SUCCESS path into a TIMEOUT. Disabling the negative cache means an early attempt made
        # before the probe's services are discoverable cannot lock the node out for a minute
        # (the default TTL): it just fails, costs the probe nothing, and test_02 retries while
        # first contact is still ahead of it.
        'parameter_service_timeout_sec': 5.0,
        'parameter_service_negative_cache_sec': 0.0,
    }
    return create_watchdog_test_launch(
        detector_params=detector_params,
        extra_gateway_params=gateway_params,
        demo_nodes=None,
        port=PORT,
    )


class TestParameterRoundTripCost(unittest.TestCase):
    """One transport call, counted at the node that serves it."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._probe = ParameterRequestCounter(PROBE_NODE)
        cls._pin_probe = ParameterRequestCounter(
            PIN_PROBE_NODE, listed_extras=(NO_VALUE_PARAM, NOT_SET_PARAM))
        cls._executor = SingleThreadedExecutor()
        cls._executor.add_node(cls._probe)
        cls._executor.add_node(cls._pin_probe)
        cls._stop = threading.Event()
        cls._spin = threading.Thread(target=cls._spin_until_stopped, daemon=True)
        cls._spin.start()

    @classmethod
    def _spin_until_stopped(cls):
        while not cls._stop.is_set() and rclpy.ok():
            cls._executor.spin_once(timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls._stop.set()
        cls._spin.join(timeout=5.0)
        cls._executor.remove_node(cls._probe)
        cls._executor.remove_node(cls._pin_probe)
        cls._executor.shutdown()
        cls._probe.destroy_node()
        cls._pin_probe.destroy_node()
        rclpy.shutdown()

    # -- helpers ----------------------------------------------------------------------------

    def _measure(self, url, probe=None):
        """GET `url` and return the response together with what it cost `probe`.

        The probe's counters are bumped inside the service callback, which completes before
        the reply reaches the transport and therefore long before the HTTP response returns.
        The settle window after the response is not there to catch a laggard: it is there so
        that a request arriving from anywhere ELSE lands inside the delta and breaks the
        assertion, instead of silently shifting the next measurement's baseline.
        """
        probe = probe if probe is not None else self._probe
        before = probe.snapshot()
        try:
            response = requests.get(url, timeout=HTTP_TIMEOUT)
        except requests.exceptions.RequestException as exc:
            # A timed-out or dropped request is a legitimate outcome here, not a test error: see
            # _RequestFailed. The delta is still taken, because a request that failed at the
            # CLIENT may well have cost the probe something at the far end, and the retry loops
            # check exactly that before deciding first contact is still ahead of them.
            response = _RequestFailed(exc)
        time.sleep(1.0)
        after = probe.snapshot()
        delta = {name: after[name] - before[name] for name in SERVICE_NAMES}
        return response, delta

    @staticmethod
    def _total(counts):
        return sum(counts.values())

    def _assert_round_trips(self, delta, expected, what):
        self.assertEqual(
            self._total(delta), expected,
            f'{what} cost the node {self._total(delta)} parameter-service round trips, not '
            f'{expected}. Per service: {delta}. Either a round trip was added to or removed '
            f'from Ros2ParameterTransport, or the charge for this case in '
            f'param_drift_detector.cpp no longer matches what the node actually pays.')

    # -- tests ------------------------------------------------------------------------------

    def test_01_the_probes_are_discovered_and_nothing_reads_them_unasked(self):
        """Every later delta is only attributable if the probes are otherwise untouched."""
        wanted = {PROBE_NODE, PIN_PROBE_NODE}
        # The retry deadlines in this file are chosen so that ALL of them together, plus the
        # single-shot cases between them, expire well inside the ctest timeout. A test written to
        # diagnose a slow-discovery path is worthless if ctest kills it before it can print.
        deadline = time.monotonic() + 60.0
        seen = set()
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{API_BASE}/apps', timeout=5)
                if response.status_code == 200:
                    seen = {item.get('id') for item in response.json().get('items', [])}
                    if wanted <= seen:
                        break
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)

        self.assertTrue(
            wanted <= seen,
            f'the gateway never discovered {sorted(wanted - seen)} as apps, so no configurations '
            f'request can reach them. Apps seen: {sorted(seen)}')

        before = (self._probe.snapshot(), self._pin_probe.snapshot())
        time.sleep(3.0)
        after = (self._probe.snapshot(), self._pin_probe.snapshot())
        self.assertEqual(
            before, after,
            f'something is reading parameters from a probe on its own: {before} -> {after}. '
            f'Every round-trip count below would then include traffic this test did not ask for.')
        for name, counts in ((PROBE_NODE, after[0]), (PIN_PROBE_NODE, after[1])):
            self.assertEqual(
                self._total(counts), 0,
                f'{name} had already served {self._total(counts)} parameter requests before this '
                f'test made any: {counts}. The first-contact measurements below need to be the '
                f'FIRST reads ever made against these nodes.')

    def test_02_a_cold_baseline_visit_costs_four_round_trips(self):
        """First contact pays for priming the transport's defaults cache as well as the read."""
        url = f'{API_BASE}/apps/{PROBE_NODE}/configurations'
        deadline = time.monotonic() + 45.0
        while True:
            response, delta = self._measure(url)
            if response.status_code == 200:
                break
            # A failure BEFORE the probe served anything is a transport that has not discovered
            # the services yet: it fails inside wait_for_service, costs the node nothing, and
            # the next attempt is still first contact. A failure that DID cost something has
            # spent the cold visit, and the measurement below can no longer be made.
            self.assertEqual(
                self._total(delta), 0,
                f'the first configurations read failed with HTTP {response.status_code} after '
                f'costing the probe {self._total(delta)} round trips ({delta}), so first '
                f'contact is gone and the cold cost can no longer be measured')
            self.assertLess(
                time.monotonic(), deadline,
                f'GET {url} never succeeded; last status {response.status_code}: '
                f'{response.text[:400]}')
            time.sleep(1.0)

        names = {item.get('name') for item in response.json().get('items', [])}
        self.assertEqual(
            names, set(DECLARED_PARAMS),
            f'the read did not come back with the parameters the probe declares ({names}), so '
            f'the transport did not take its SUCCESS path and the count below is the price of a '
            f'failure, not of a read')

        self._assert_round_trips(
            delta, 4,
            'a COLD baseline visit (first list_parameters against the node: the defaults-cache '
            'priming list+get, then the second list+get the read itself makes)')
        # Per service, not just the total. The four is two list+get PAIRS and nothing else; a
        # change that swapped one of them for a describe or a get_parameter_types would leave the
        # total at four and mean something quite different about what the node pays.
        self.assertEqual(
            (delta['list_parameters'], delta['get_parameters']), (2, 2),
            f'a cold baseline visit is two list+get pairs - the defaults-cache priming one and '
            f'the read itself - and this was not: {delta}')

    def test_03_a_warm_baseline_visit_costs_two_round_trips(self):
        """The defaults cache has no TTL, so the cold pair is paid once per node, not per read."""
        url = f'{API_BASE}/apps/{PROBE_NODE}/configurations'
        response, delta = self._measure(url)
        self.assertEqual(response.status_code, 200, response.text[:400])
        names = {item.get('name') for item in response.json().get('items', [])}
        self.assertEqual(names, set(DECLARED_PARAMS), response.text[:400])

        self._assert_round_trips(
            delta, 2,
            'a WARM baseline visit (kBaselineReadRoundTrips: one list, then one batched get of '
            'every name it returned)')
        self.assertEqual(
            (delta['list_parameters'], delta['get_parameters']), (1, 1),
            f'a warm baseline visit is exactly one list and one batched get, and this was not - a '
            f'total of two says nothing about WHICH two: {delta}')

    def test_04_a_resolved_expect_pin_costs_three_round_trips(self):
        """describe_parameters is unconditional on the success path, so the pin pays for it."""
        pin = 'probe_gain'
        response, delta = self._measure(f'{API_BASE}/apps/{PROBE_NODE}/configurations/{pin}')
        self.assertEqual(response.status_code, 200, response.text[:400])
        self.assertEqual(
            response.json().get('data'), DECLARED_PARAMS[pin],
            f'the pinned read did not return the value the probe serves, so the transport did '
            f'not take its SUCCESS path: {response.text[:400]}')

        self._assert_round_trips(
            delta, 3,
            'a RESOLVED expect pin (kExpectReadRoundTrips: name-scoped list, get, describe)')
        self.assertEqual(
            delta['describe_parameters'], 1,
            f'the descriptor round trip is what makes a resolved pin cost three rather than '
            f'two, and it did not happen: {delta}')

    def test_05_an_unresolved_expect_pin_costs_one_round_trip(self):
        """Charged 2 by the detector on purpose; the real cost is 1 and is allowed to be lower.

        The transport answers NOT_FOUND from the name-scoped list alone and never reaches the
        get or the describe. ``kExpectNotFoundRoundTrips`` is 2 because a SECOND NOT_FOUND path
        - a parameter that is declared but unset - does cost two, and the error code cannot
        distinguish them, so the constant takes the higher of the two. Overstating a load bound
        is safe; understating it is not. This asserts the real cost, not the charge.
        """
        response, delta = self._measure(
            f'{API_BASE}/apps/{PROBE_NODE}/configurations/{UNDECLARED_PARAM}')
        self.assertEqual(
            response.status_code, 404,
            f'a parameter the probe never declares must come back 404, not '
            f'{response.status_code}: {response.text[:400]}')

        self._assert_round_trips(
            delta, 1,
            'an UNRESOLVED expect pin (the name-scoped list returns no names and the transport '
            'stops there)')
        self.assertEqual(
            delta['get_parameters'], 0,
            f'the transport read a value for a parameter its own name-scoped list said the node '
            f'does not have: {delta}')

    def test_06_a_pinned_read_has_no_cold_contact_surcharge(self):
        """The cold pair belongs to list_parameters, not to a read in general.

        ``get_parameter`` never calls ``cache_default_values``, so an ``expect``-only sweep
        (``baseline: false``) never primes that cache and never pays the extra pair - the first
        pinned read of a node costs the same 3 as every one after it. The second probe exists
        purely to have a node whose defaults cache is still unprimed at this point; the first
        one stopped being one the moment test_02 listed it.
        """
        pin = 'probe_gain'
        before = self._pin_probe.snapshot()
        self.assertEqual(
            self._total(before), 0,
            f'{PIN_PROBE_NODE} has already been read ({before}), so this is no longer a '
            f'first-contact measurement')

        url = f'{API_BASE}/apps/{PIN_PROBE_NODE}/configurations/{pin}'
        deadline = time.monotonic() + 45.0
        while True:
            response, delta = self._measure(url, probe=self._pin_probe)
            if response.status_code == 200:
                break
            self.assertEqual(
                self._total(delta), 0,
                f'the first pinned read of {PIN_PROBE_NODE} failed with HTTP '
                f'{response.status_code} after costing it {self._total(delta)} round trips '
                f'({delta}), so first contact is gone and the cost can no longer be measured')
            self.assertLess(
                time.monotonic(), deadline,
                f'GET {url} never succeeded; last status {response.status_code}: '
                f'{response.text[:400]}')
            time.sleep(1.0)

        self.assertEqual(
            response.json().get('data'), DECLARED_PARAMS[pin],
            f'the pinned read did not return the value the probe serves, so the transport did '
            f'not take its SUCCESS path: {response.text[:400]}')
        self._assert_round_trips(
            delta, 3,
            'the FIRST expect pin read of a node (same as any later one: get_parameter does not '
            'prime the defaults cache, so there is no cold-contact pair in expect-only mode)')

    def test_07_a_listed_name_with_no_value_costs_two_round_trips(self):
        """The rarer NOT_FOUND path, and the reason the charge for NOT_FOUND is 2 and not 1.

        test_05 measures the COMMON NOT_FOUND: the name-scoped list comes back empty and the
        transport stops there, for one round trip. This is the other one. The node lists the name,
        so the transport goes on to the get, and the get comes back with no value for it - two
        round trips, and the same NOT_FOUND error code as the one-round-trip case.
        ``kExpectNotFoundRoundTrips`` is 2 because that is the higher of the two and the transport
        cannot tell a caller which it took: a load bound may overstate a cost, never understate it.

        Without this case the charge was anchored from one side only. test_05 says the charge may
        legitimately exceed the real cost, which on its own is an argument for any number at all;
        it takes this measurement to say why the number is exactly two.
        """
        response, delta = self._measure(
            f'{API_BASE}/apps/{PIN_PROBE_NODE}/configurations/{NO_VALUE_PARAM}',
            probe=self._pin_probe)
        self.assertEqual(
            response.status_code, 404,
            f'a parameter the probe lists but returns no value for must come back 404, not '
            f'{response.status_code}: {response.text[:400]}')

        self._assert_round_trips(
            delta, 2,
            'a LISTED name the get returns no value for (the name-scoped list finds the name, the '
            'get finds no value, and the transport returns NOT_FOUND before the describe)')
        self.assertEqual(
            (delta['list_parameters'], delta['get_parameters'], delta['describe_parameters']),
            (1, 1, 0),
            f'the two round trips are the list and the get, and the describe must not happen - a '
            f'NOT_FOUND that reached the descriptor read would cost three, which is MORE than '
            f'kExpectNotFoundRoundTrips charges and would make the bound an understatement: '
            f'{delta}')

    def test_08_a_declared_but_unset_pin_is_a_success_costing_three(self):
        """A declared-but-unset parameter is NOT the two-round-trip NOT_FOUND path.

        This is worth its own case because it is what that path used to be documented as. A
        parameter a node has declared and not set is answered by rclcpp's parameter service with a
        ``PARAMETER_NOT_SET`` VALUE, not with an absent one, so the transport gets a parameter
        back, goes on to the unconditional describe, and returns SUCCESS carrying null. It costs
        the node three round trips, the same as any other resolved pin, and ``expect`` charges it
        three - which is right. Had the charge of 2 really been sized for this case it would have
        been an UNDERSTATEMENT of what the node pays, and the whole point of that constant is that
        it may overstate and must never understate.
        """
        response, delta = self._measure(
            f'{API_BASE}/apps/{PIN_PROBE_NODE}/configurations/{NOT_SET_PARAM}',
            probe=self._pin_probe)
        self.assertEqual(
            response.status_code, 200,
            f'a parameter the node declares but has not set is a value the node HAS (an unset '
            f'one), so the read succeeds: {response.status_code}, {response.text[:400]}')
        self.assertIsNone(
            response.json().get('data'),
            f'the unset parameter came back carrying a value: {response.text[:400]}')

        self._assert_round_trips(
            delta, 3,
            'a DECLARED BUT UNSET pin (list, get, describe - the same success path as any other '
            'resolved pin, which is why expect charges it kExpectReadRoundTrips and not '
            'kExpectNotFoundRoundTrips)')
        self.assertEqual(
            delta['describe_parameters'], 1,
            f'the descriptor round trip is what makes this a three rather than a two, and it did '
            f'not happen: {delta}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway/fault_manager stack exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
