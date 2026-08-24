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

"""
Integration tests: a manifest entity keeps its provenance under every layer policy.

``x-medkit.source`` is provenance, and the merge pipeline's orphan and
deduplication filters key their delete decision on it: a manifest-sourced
entity is protected, a runtime-sourced one is not. A field-group policy must
therefore never be able to rewrite it.

Three gateways run side by side on one ROS graph against the same entity
declarations, differing only in how the manifest layer's METADATA policy ends
up configured:

====================  ==============================================  ==============
Gateway               Configuration                                   Manifest layer
====================  ==============================================  ==============
default               shipped defaults                                authoritative
no-override           manifest ``config: allow_manifest_override:      fallback
                      false``
metadata-fallback     gateway param                                    fallback
                      ``...layers.manifest.metadata: fallback``
====================  ==============================================  ==============

The two demoting gateways are the two doors onto the same code path. All three
must serve the same answer.

**The app under test is deliberately named after its node.** ``temp_sensor``
binds to the node ``temp_sensor``, so the manifest layer and the runtime layer
contribute an entity under the *same* id and the per-field-group merge actually
runs. ``mp-pressure`` binds to ``pressure_sensor`` under a different id and is
the control: no merge, so it was never at risk.

**Why this test polls instead of checking once.** The failure it guards is not
a clean, stable deletion - the entity can come and go between discovery passes,
and the collection listing and the detail endpoint can contradict each other at
the same instant. A single sample passes even against the broken gateway. So
every sample is asserted, and the two views are compared against each other,
not just against the expectation.

**How the test knows discovery really kept running.** Elapsed wall time proves
nothing: a gateway whose refresh timer died after its first pass would still
sit there answering, and a sleep-based test would call that twelve passes. The
gateway does publish a real counter - ``x-medkit-entity-cache.generation`` -
but it is deliberately gated on the cache CONTENTS changing, so on a steady
graph it never moves however many passes run (measured: flat across eight
samples spanning ten passes). So the window is given something to notice: an
undeclared node joins part-way through, and sampling continues until every
gateway's generation has advanced past the value it had before. That advance is
the proof the pipeline was alive during the window; it cannot be produced by a
stopped timer.
"""

import os
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
)

# --------------------------------------------------------------------------
# Ports - one gateway per configuration
# --------------------------------------------------------------------------

DEFAULT_PORT = get_test_port(0)
NO_OVERRIDE_PORT = get_test_port(1)
METADATA_FALLBACK_PORT = get_test_port(2)


def _base_url(port):
    return f'http://localhost:{port}{API_BASE_PATH}'


DEFAULT_URL = _base_url(DEFAULT_PORT)
NO_OVERRIDE_URL = _base_url(NO_OVERRIDE_PORT)
METADATA_FALLBACK_URL = _base_url(METADATA_FALLBACK_PORT)

ALL_URLS = (DEFAULT_URL, NO_OVERRIDE_URL, METADATA_FALLBACK_URL)

# --------------------------------------------------------------------------
# Manifest fixture
# --------------------------------------------------------------------------

AREA_ID = 'mp-area'
COMPONENT_ID = 'mp-ecu'

# The app under test: its id is deliberately identical to the name of the node
# it binds to, which is what makes both discovery layers contribute an entity
# under the same id.
SELF_NAMED_APP_ID = 'temp_sensor'
SELF_NAMED_NODE = 'temp_sensor'
SELF_NAMED_FQN = '/powertrain/engine/temp_sensor'

# The control app: distinct id, so the two layers never meet on it.
DISTINCT_APP_ID = 'mp-pressure'
DISTINCT_NODE = 'pressure_sensor'
DISTINCT_FQN = '/chassis/brakes/pressure_sensor'

MANIFEST_APPS = {SELF_NAMED_APP_ID, DISTINCT_APP_ID}

# The node that joins mid-window purely so the gateways have a change to
# notice. It is declared nowhere, and the default policy (warn) keeps it
# visible, so every gateway's entity set changes and its cache generation
# advances - which is what proves the discovery loop was alive. Started by the
# test once every gateway is ready, so the change can never land before the
# baseline is read.
LATE_NODE_EXECUTABLE = 'demo_rpm_sensor'
LATE_NODE_NAME = 'rpm_sensor'
LATE_NODE_NAMESPACE = '/powertrain/engine'

# Every gateway here runs at the shared test default of 1000 ms
# (create_gateway_node sets refresh_interval_ms), so sampling a little slower
# than that keeps consecutive samples on different passes.
# Wall-clock BUDGETS are scaled by MEDKIT_TEST_TIME_SCALE. The sanitizer jobs
# multiply every ctest timeout, but a deadline asserted inside a test is
# invisible to that rewrite, so an ASan/TSan gateway blows it and the failure
# reads as a regression rather than as instrumentation cost. Unscaled
# elsewhere, so the budgets keep their falsifying edge on normal jobs.
#
# Poll INTERVALS and soak durations are deliberately NOT scaled: polling more
# slowly would only notice the same answer later, and stretching a soak would
# hold the gateways longer under exactly the instrumentation that made them
# expensive - which is load this file would be adding, not absorbing.
TIME_SCALE = get_time_scale()

# Sample granularity, tied to the gateway's 1000 ms refresh - an INTERVAL, so
# unscaled: stretching it would change how many passes the window spans.
SAMPLE_INTERVAL_SEC = 1.25
# Minimum samples per gateway. Sampling runs past this if the generation has
# not advanced yet, so the liveness proof is a condition and not a guess.
SAMPLE_COUNT = 12
SAMPLE_DEADLINE_SEC = 70.0 * TIME_SCALE

# Was a private 60.0; this is the shared discovery budget, scaled.
POLL_TIMEOUT_SEC = DISCOVERY_TIMEOUT * TIME_SCALE
POLL_INTERVAL_SEC = 0.5
# How long a single HTTP call has to come back.
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE
# How long the helper node has to exit when the test reaps it.
PROCESS_REAP_TIMEOUT_SEC = 10.0 * TIME_SCALE


def _manifest_yaml(allow_manifest_override):
    """Render the shared manifest with the given override flag."""
    flag = 'true' if allow_manifest_override else 'false'
    return f"""\
manifest_version: "1.0"
metadata:
  name: "Merge provenance test vehicle"
  version: "1.0.0"
config:
  unmanifested_nodes: "warn"
  allow_manifest_override: {flag}
areas:
  - id: {AREA_ID}
    name: "Merge Provenance Area"
components:
  - id: {COMPONENT_ID}
    name: "Merge Provenance ECU"
    area: {AREA_ID}
apps:
  - id: {SELF_NAMED_APP_ID}
    name: "Engine Temperature Sensor"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: {SELF_NAMED_NODE}
      namespace: /powertrain/engine
  - id: {DISTINCT_APP_ID}
    name: "Brake Pressure Sensor"
    is_located_on: {COMPONENT_ID}
    ros_binding:
      node_name: {DISTINCT_NODE}
      namespace: /chassis/brakes
"""


# Manifests this module wrote, so the post-shutdown phase can remove them
# instead of leaving a file per run behind in the temp directory.
_MANIFEST_PATHS = []


def _remove_manifests():
    """Delete every manifest this module wrote."""
    while _MANIFEST_PATHS:
        try:
            os.unlink(_MANIFEST_PATHS.pop())
        except OSError:
            pass


def _write_manifest(name, content):
    """Write manifest YAML to a temporary file, return its path."""
    fd, path = tempfile.mkstemp(
        suffix='.yaml', prefix=f'test_merge_provenance_{name}_',
    )
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    _MANIFEST_PATHS.append(path)
    return path


# --------------------------------------------------------------------------
# Sampling helpers
# --------------------------------------------------------------------------

def _sample(base_url, app_id):
    """Read the collection and the detail endpoint back to back.

    Returns a dict describing one instant: whether the app is listed, what the
    detail endpoint answers, and the provenance each view reports.
    """
    listing = requests.get(f'{base_url}/apps', timeout=HTTP_TIMEOUT_SEC)
    detail = requests.get(f'{base_url}/apps/{app_id}', timeout=HTTP_TIMEOUT_SEC)

    listing.raise_for_status()
    items = listing.json().get('items', [])
    listed = next((item for item in items if item.get('id') == app_id), None)

    detail_source = None
    if detail.status_code == 200:
        detail_source = detail.json().get('x-medkit', {}).get('source')

    return {
        'in_collection': listed is not None,
        'collection_source': (
            listed.get('x-medkit', {}).get('source') if listed else None
        ),
        'detail_status': detail.status_code,
        'detail_source': detail_source,
        'listed_ids': sorted(item.get('id') for item in items),
    }


def _poll(predicate, *, what, timeout=POLL_TIMEOUT_SEC):
    """Poll *predicate* until it returns a truthy value, else fail loudly."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
            if last:
                return last
        except requests.exceptions.RequestException as exc:
            last = exc
        time.sleep(POLL_INTERVAL_SEC)
    raise AssertionError(
        f'timed out after {timeout}s waiting for {what}; last={last!r}'
    )


def _bound_nodes(base_url):
    """Return the set of ROS node FQNs any app on *base_url* is bound to."""
    response = requests.get(f'{base_url}/apps', timeout=HTTP_TIMEOUT_SEC)
    response.raise_for_status()
    nodes = set()
    for app in response.json().get('items', []):
        node = app.get('x-medkit', {}).get('ros2', {}).get('node')
        if node:
            nodes.add(node)
    return nodes


def _wait_for_linking(base_url):
    """Block until *base_url* has linked the control app to its node.

    Proves the gateway has completed a discovery pass that saw the live ROS
    graph, so the samples that follow are not reading a pre-graph snapshot.
    The control app is used as the witness precisely because it is the app the
    defect cannot touch.
    """
    return _poll(
        lambda: DISTINCT_FQN in _bound_nodes(base_url),
        what=f'{base_url} to link {DISTINCT_FQN}',
    )


_LATE_NODE_PROCESS = None


def _start_late_node():
    """Put one undeclared node on the graph, once, and keep it running.

    Started from the test rather than the launch description so it cannot
    appear before the baseline sample has been recorded.
    """
    global _LATE_NODE_PROCESS
    if _LATE_NODE_PROCESS is not None:
        return _LATE_NODE_PROCESS
    executable = os.path.join(
        get_package_prefix('ros2_medkit_integration_tests'),
        'lib', 'ros2_medkit_integration_tests', LATE_NODE_EXECUTABLE,
    )
    _LATE_NODE_PROCESS = subprocess.Popen(  # noqa: S603
        [executable, '--ros-args',
         '-r', f'__ns:={LATE_NODE_NAMESPACE}',
         '-r', f'__node:={LATE_NODE_NAME}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    return _LATE_NODE_PROCESS


def _stop_late_node():
    """Reap the helper so it cannot outlive the test as an orphan."""
    global _LATE_NODE_PROCESS
    if _LATE_NODE_PROCESS is None:
        return
    _LATE_NODE_PROCESS.terminate()
    try:
        _LATE_NODE_PROCESS.wait(timeout=PROCESS_REAP_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        _LATE_NODE_PROCESS.kill()
        _LATE_NODE_PROCESS.wait(timeout=PROCESS_REAP_TIMEOUT_SEC)
    _LATE_NODE_PROCESS = None


def _generation(base_url):
    """Return the gateway's entity-cache generation counter.

    Advances only when a refresh actually changes the cache contents, so it is
    a true witness that the discovery pipeline ran AND saw something new - not
    a tick count that a stopped timer could fake.
    """
    health = requests.get(f'{base_url}/health', timeout=HTTP_TIMEOUT_SEC)
    health.raise_for_status()
    return health.json().get('x-medkit-entity-cache', {}).get('generation')


# All three gateways are sampled in one interleaved window rather than once per
# test class: it is the same wall-clock window for each of them, it keeps the
# file inside the feature-test budget, and it means the single late node lands
# inside every gateway's window instead of only the first one's.
_SAMPLES = None


def _collect_samples():
    """Sample every gateway until the window is long enough AND provably live.

    Returns ``(samples_by_url, baseline_generation_by_url)``.
    """
    global _SAMPLES
    if _SAMPLES is not None:
        return _SAMPLES

    for url in ALL_URLS:
        _wait_for_linking(url)

    samples = {url: [] for url in ALL_URLS}
    started = time.monotonic()

    # Sample 0 is taken before the change and its generation is the baseline.
    # Reading a generation that no sample is recorded against proves only what
    # the counter said at that instant; it leaves the first recorded sample free
    # to land on the far side of the change, and then no sample sits before it.
    # Taking the baseline from a sample in the window makes "a sample before the
    # change" a property of the window rather than of the scheduler.
    baseline = {}
    for url in ALL_URLS:
        sample = _sample(url, SELF_NAMED_APP_ID)
        sample['generation'] = _generation(url)
        sample['index'] = 0
        sample['at'] = round(time.monotonic() - started, 3)
        samples[url].append(sample)
        baseline[url] = sample['generation']

    # The change that drives the liveness proof is started HERE, after every
    # gateway has linked and its baseline sample has been taken - not on a
    # launch-time timer. A fixed timer races convergence: on a slow box the
    # node joins before the baseline is taken, no generation advance is ever
    # observed, and a correct implementation fails as an opaque timeout.
    _start_late_node()

    # The budget for the advance runs from the change, not from the first
    # baseline read: the reads above are HTTP round trips on an instrumented
    # gateway and charging them to the liveness proof shortens it by however
    # long they took.
    deadline = time.monotonic() + SAMPLE_DEADLINE_SEC
    # Sample 1 follows the change with no sleep in front of it. A merge that
    # drops or rewrites the app for one refresh and repairs itself on the next
    # is only visible to a read taken while it is wrong, so the window keeps an
    # observation at the moment the entity set changed.
    index = 1
    while True:
        if index > 1:
            time.sleep(SAMPLE_INTERVAL_SEC)
        for url in ALL_URLS:
            sample = _sample(url, SELF_NAMED_APP_ID)
            sample['generation'] = _generation(url)
            sample['index'] = index
            sample['at'] = round(time.monotonic() - started, 3)
            samples[url].append(sample)
        index += 1

        advanced = all(
            samples[url][-1]['generation'] > baseline[url] for url in ALL_URLS
        )
        if index >= SAMPLE_COUNT and advanced:
            break
        if time.monotonic() > deadline:
            latest = {u: samples[u][-1]['generation'] for u in ALL_URLS}
            raise AssertionError(
                f'after {SAMPLE_DEADLINE_SEC}s and {index} samples the cache '
                f'generation had not advanced on every gateway; '
                f'baseline={baseline}, latest={latest}. Either the late node '
                f'never joined or a discovery loop is dead.'
            )

    _SAMPLES = (samples, baseline)
    return _SAMPLES


# --------------------------------------------------------------------------
# Launch description
# --------------------------------------------------------------------------

def generate_test_description():
    permissive_manifest = _write_manifest('override', _manifest_yaml(True))
    restrictive_manifest = _write_manifest(
        'nooverride', _manifest_yaml(False),
    )

    def gateway(name, port, manifest_path, extra=None):
        params = {
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
        }
        if extra:
            params.update(extra)
        return create_gateway_node(port=port, name=name, extra_params=params)

    gateways = [
        gateway('gw_mp_default', DEFAULT_PORT, permissive_manifest),
        gateway('gw_mp_no_override', NO_OVERRIDE_PORT, restrictive_manifest),
        gateway(
            'gw_mp_metadata_fallback', METADATA_FALLBACK_PORT,
            permissive_manifest,
            {'discovery.merge_pipeline.layers.manifest.metadata': 'fallback'},
        ),
    ]

    demo_nodes = create_demo_nodes([SELF_NAMED_NODE, DISTINCT_NODE])

    return (
        LaunchDescription(
            gateways + demo_nodes + [launch_testing.actions.ReadyToTest()],
        ),
        {'gateway_node': gateways[0]},
    )


# --------------------------------------------------------------------------
# Shared assertions
# --------------------------------------------------------------------------

class ProvenanceHoldsMixin:
    """Assertions every gateway in this file must satisfy identically."""

    # Only the control app is required up front. The app under test is the
    # subject of the assertions, so waiting for it here would turn a clean
    # failure into a setUpClass timeout.
    REQUIRED_APPS = {DISTINCT_APP_ID}

    @classmethod
    def setUpClass(cls):
        """Wait for the graph, then take (or reuse) the shared sample window."""
        super().setUpClass()
        all_samples, baseline = _collect_samples()
        cls.samples = all_samples[cls.BASE_URL]
        cls.baseline_generation = baseline[cls.BASE_URL]

    def test_the_discovery_loop_ran_throughout_the_sampling_window(self):
        """Guards the instrument, on a counter rather than on a sleep.

        The cache generation advances only when a refresh changes the entity
        set. An undeclared node joins mid-window, so an advance here means the
        pipeline actually ran and saw it. A gateway whose refresh timer had
        stopped after its first pass would fail this and could not be rescued
        by waiting longer.
        """
        self.assertGreaterEqual(len(self.samples), SAMPLE_COUNT)
        self.assertGreater(
            self.samples[-1]['generation'], self.baseline_generation,
            'the entity-cache generation never advanced, so nothing proves a '
            'discovery pass ran during the window; generations seen: '
            f'{[s["generation"] for s in self.samples]}',
        )

    def test_self_named_manifest_app_is_served_on_every_pass(self):
        """The app is listed, answers 200, and stays manifest-sourced."""
        bad = [s for s in self.samples if not s['in_collection']
               or s['detail_status'] != 200]
        self.assertEqual(
            bad, [],
            f"'{SELF_NAMED_APP_ID}' must be served on every one of "
            f'{len(self.samples)} samples; failing samples: {bad}',
        )

        wrong_source = [
            s for s in self.samples
            if s['collection_source'] != 'manifest'
            or s['detail_source'] != 'manifest'
        ]
        self.assertEqual(
            wrong_source, [],
            'provenance is not content: a layer policy must not rewrite '
            f"'{SELF_NAMED_APP_ID}' away from manifest; failing samples: "
            f'{wrong_source}',
        )

    def test_the_app_survives_the_graph_change_too(self):
        """Change, not just steady state: a node joins and it still holds.

        Sample 0 is taken before the late node is started and carries the
        baseline generation, so the window always straddles the moment the
        entity set changed and the assertions above cover both sides of it.
        This names the claim so it is not read as a steady-state result.
        """
        changed_at = next(
            (s['index'] for s in self.samples
             if s['generation'] > self.baseline_generation),
            None,
        )
        self.assertIsNotNone(changed_at)
        self.assertTrue(
            any(s['index'] >= changed_at for s in self.samples)
            and any(s['index'] < changed_at for s in self.samples),
            f'the window must contain samples from before and after the '
            f'change; it changed at sample {changed_at} of {len(self.samples)}',
        )
        after = [s for s in self.samples if s['index'] >= changed_at]
        self.assertEqual(
            [s for s in after if not s['in_collection']
             or s['detail_status'] != 200], [],
            f'the app must survive the entity-set change; after={after}',
        )

    def test_collection_and_detail_never_disagree(self):
        """The two views of the same entity agree at every instant."""
        disagreements = [
            s for s in self.samples
            if s['in_collection'] != (s['detail_status'] == 200)
        ]
        self.assertEqual(
            disagreements, [],
            f'GET /apps and GET /apps/{SELF_NAMED_APP_ID} contradicted each '
            f'other on {len(disagreements)} of {len(self.samples)} samples: '
            f'{disagreements}',
        )

    def test_control_app_with_a_distinct_id_is_unaffected(self):
        """The app whose id differs from its node was never at risk."""
        sample = _sample(self.BASE_URL, DISTINCT_APP_ID)
        self.assertTrue(sample['in_collection'], sample)
        self.assertEqual(sample['detail_status'], 200, sample)
        self.assertEqual(sample['collection_source'], 'manifest', sample)

    def test_both_manifest_apps_are_present(self):
        """The manifest contributed both apps, whatever the layer policy."""
        listed = _poll(
            lambda: set(_sample(
                self.BASE_URL, SELF_NAMED_APP_ID)['listed_ids']) or None,
            what=f'{self.BASE_URL} to list any app',
        )
        self.assertEqual(
            MANIFEST_APPS - listed, set(),
            f'missing manifest apps: {sorted(MANIFEST_APPS - listed)}; '
            f'listed: {sorted(listed)}',
        )


# --------------------------------------------------------------------------
# Baseline: the shipped defaults
# --------------------------------------------------------------------------

class TestDefaultPolicies(ProvenanceHoldsMixin, GatewayTestCase):
    """Control gateway: nothing demotes the manifest layer."""

    BASE_URL = DEFAULT_URL


# --------------------------------------------------------------------------
# Door 1: the manifest's own `allow_manifest_override: false`
# --------------------------------------------------------------------------

class TestManifestOverrideDisabled(ProvenanceHoldsMixin, GatewayTestCase):
    """`allow_manifest_override: false` demotes METADATA to fallback."""

    BASE_URL = NO_OVERRIDE_URL


# --------------------------------------------------------------------------
# Door 2: the gateway parameter, reachable without any manifest flag
# --------------------------------------------------------------------------

class TestManifestMetadataFallback(ProvenanceHoldsMixin, GatewayTestCase):
    """`layers.manifest.metadata: fallback` demotes the same field group."""

    BASE_URL = METADATA_FALLBACK_URL


# --------------------------------------------------------------------------
# The three gateways must not merely each be self-consistent
# --------------------------------------------------------------------------

class TestAllConfigurationsAgree(GatewayTestCase):
    """Provenance is identical across all three configurations."""

    BASE_URL = DEFAULT_URL
    REQUIRED_APPS = {DISTINCT_APP_ID}

    def test_every_gateway_reports_the_same_provenance(self):
        """One entity, three layer policies, one answer."""
        urls = (DEFAULT_URL, NO_OVERRIDE_URL, METADATA_FALLBACK_URL)
        for url in urls:
            _wait_for_linking(url)

        observed = {url: _sample(url, SELF_NAMED_APP_ID) for url in urls}
        for url, sample in observed.items():
            self.assertTrue(
                sample['in_collection'],
                f'{url} does not list {SELF_NAMED_APP_ID}: {sample}',
            )
            self.assertEqual(
                sample['detail_status'], 200,
                f'{url} does not serve {SELF_NAMED_APP_ID}: {sample}',
            )
            self.assertEqual(
                sample['collection_source'], 'manifest',
                f'{url} reports the wrong provenance: {sample}',
            )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_helper_node_and_manifests_are_cleaned_up(self):
        """The test owns the late node and its manifests, so it reaps them."""
        _stop_late_node()
        _remove_manifests()

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
