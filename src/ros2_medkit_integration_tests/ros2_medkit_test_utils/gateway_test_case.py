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

"""Base test case for ros2_medkit gateway integration tests.

Provides health-check polling, discovery waiting, HTTP helpers, wait helpers,
and discovery assertion methods. Replaces ~400 lines of duplicated boilerplate
in every test file.

Subclasses override class constants to control discovery waiting:

    class TestMyFeature(GatewayTestCase):
        MIN_EXPECTED_APPS = 3
        REQUIRED_AREAS = {'powertrain', 'chassis'}
        REQUIRED_APPS = {'temp_sensor', 'actuator'}
        REQUIRED_FUNCTIONS = {'powertrain', 'chassis'}
"""

import time
import unittest
from urllib.parse import quote

import requests

from ros2_medkit_test_utils.constants import (
    ACTION_TIMEOUT,
    DEFAULT_BASE_URL,
    DISCOVERY_INTERVAL,
    DISCOVERY_TIMEOUT,
    FAULT_TIMEOUT,
    GATEWAY_STARTUP_INTERVAL,
    GATEWAY_STARTUP_TIMEOUT,
    ROSBAG_TIMEOUT,
    SNAPSHOT_TIMEOUT,
)


class GatewayTestCase(unittest.TestCase):
    """Base class for gateway integration tests.

    Class Attributes
    ----------------
    BASE_URL : str
        Full base URL including API path prefix.
    MIN_EXPECTED_APPS : int
        Minimum number of apps that must be discovered before tests run.
        Set to 0 to skip discovery waiting entirely.
    REQUIRED_AREAS : set of str
        Area IDs that must be discovered. Empty set skips area checking.
    REQUIRED_APPS : set of str
        App IDs that must be discovered. Empty set skips app checking.
    REQUIRED_FUNCTIONS : set of str
        Function IDs that must be discovered. Empty set skips function checking.

    """

    BASE_URL = DEFAULT_BASE_URL

    # Override in subclass for discovery waiting.
    # When all three are falsy, setUpClass skips discovery polling.
    MIN_EXPECTED_APPS = 0
    REQUIRED_AREAS: set = set()
    REQUIRED_APPS: set = set()
    REQUIRED_FUNCTIONS: set = set()
    # Map of entity endpoint -> operation ID that must be discovered.
    # Example: {'/apps/calibration': 'calibrate'}
    REQUIRED_OPERATIONS: dict = {}

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    @classmethod
    def setUpClass(cls):
        """Wait for gateway health, then optionally wait for discovery."""
        cls._wait_for_gateway_health()
        if (cls.MIN_EXPECTED_APPS > 0 or cls.REQUIRED_AREAS
                or cls.REQUIRED_APPS or cls.REQUIRED_FUNCTIONS):
            cls._wait_for_discovery()
        if cls.REQUIRED_OPERATIONS:
            cls._wait_for_operations()

    @classmethod
    def _wait_for_gateway_health(cls):
        """Poll GET /health until the gateway responds with 200.

        Uses ``time.monotonic()`` for a reliable, monotonic clock.

        Raises
        ------
        AssertionError
            If the gateway does not respond within
            ``GATEWAY_STARTUP_TIMEOUT`` seconds.

        """
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if response.status_code == 200:
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(GATEWAY_STARTUP_INTERVAL)
        raise AssertionError(
            f'Gateway not responding after {GATEWAY_STARTUP_TIMEOUT}s'
        )

    @classmethod
    def _wait_for_discovery(cls):
        """Poll /apps, /areas, /functions until required entities are discovered.

        Checks:
        1. Number of apps >= ``MIN_EXPECTED_APPS``
        2. All ``REQUIRED_APPS`` IDs are present
        3. All ``REQUIRED_AREAS`` IDs are present
        4. All ``REQUIRED_FUNCTIONS`` IDs are present

        Raises
        ------
        AssertionError
            If the deadline is reached without satisfying all conditions.

        """
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        while time.monotonic() < deadline:
            try:
                apps_response = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
                areas_response = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
                funcs_response = requests.get(f'{cls.BASE_URL}/functions', timeout=5)
                if (apps_response.status_code == 200
                        and areas_response.status_code == 200
                        and funcs_response.status_code == 200):
                    apps = apps_response.json().get('items', [])
                    areas = areas_response.json().get('items', [])
                    funcs = funcs_response.json().get('items', [])
                    app_ids = {a.get('id', '') for a in apps}
                    area_ids = {a.get('id', '') for a in areas}
                    func_ids = {f.get('id', '') for f in funcs}

                    missing_areas = cls.REQUIRED_AREAS - area_ids
                    missing_apps = cls.REQUIRED_APPS - app_ids
                    missing_funcs = cls.REQUIRED_FUNCTIONS - func_ids
                    apps_ok = len(apps) >= cls.MIN_EXPECTED_APPS and not missing_apps
                    areas_ok = not missing_areas
                    funcs_ok = not missing_funcs

                    if apps_ok and areas_ok and funcs_ok:
                        print(
                            f'Discovery complete: {len(apps)} apps, '
                            f'{len(areas)} areas, {len(funcs)} functions'
                        )
                        return

                    print(
                        f'  Waiting: {len(apps)}/{cls.MIN_EXPECTED_APPS} apps, '
                        f'{len(areas)} areas, {len(funcs)} functions. '
                        f'Missing areas: {missing_areas}, '
                        f'Missing apps: {missing_apps}, '
                        f'Missing functions: {missing_funcs}'
                    )
            except requests.exceptions.RequestException:
                # Connection errors expected while nodes are starting; retry.
                pass
            time.sleep(DISCOVERY_INTERVAL)

        discovered_apps = set()
        discovered_areas = set()
        discovered_funcs = set()
        try:
            r = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
            if r.status_code == 200:
                discovered_apps = {
                    a.get('id', '') for a in r.json().get('items', [])
                }
            r = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
            if r.status_code == 200:
                discovered_areas = {
                    a.get('id', '') for a in r.json().get('items', [])
                }
            r = requests.get(f'{cls.BASE_URL}/functions', timeout=5)
            if r.status_code == 200:
                discovered_funcs = {
                    f.get('id', '') for f in r.json().get('items', [])
                }
        except requests.exceptions.RequestException:
            pass
        raise AssertionError(
            f'Discovery incomplete after {DISCOVERY_TIMEOUT}s - '
            f'found {len(discovered_apps)} apps, need {cls.MIN_EXPECTED_APPS}. '
            f'Missing apps: {cls.REQUIRED_APPS - discovered_apps}, '
            f'Missing areas: {cls.REQUIRED_AREAS - discovered_areas}, '
            f'Missing functions: {cls.REQUIRED_FUNCTIONS - discovered_funcs}'
        )

    @classmethod
    def _wait_for_operations(cls):
        """Wait for all ``REQUIRED_OPERATIONS`` to be discovered.

        Polls each entity's operations endpoint until the specified
        operation ID appears. Runs after ``_wait_for_discovery()`` so
        that apps/areas are already visible.

        Raises
        ------
        AssertionError
            If any required operation is not found within
            ``DISCOVERY_TIMEOUT`` seconds.

        """
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        pending = dict(cls.REQUIRED_OPERATIONS)  # copy

        while pending and time.monotonic() < deadline:
            still_missing = {}
            for entity_endpoint, operation_id in pending.items():
                try:
                    response = requests.get(
                        f'{cls.BASE_URL}{entity_endpoint}/operations',
                        timeout=5,
                    )
                    if response.status_code == 200:
                        ops = response.json().get('items', [])
                        if any(op.get('id') == operation_id for op in ops):
                            continue  # found
                except requests.exceptions.RequestException:
                    pass
                still_missing[entity_endpoint] = operation_id
            pending = still_missing
            if pending:
                time.sleep(DISCOVERY_INTERVAL)

        if pending:
            raise AssertionError(
                f'Operations not discovered after {DISCOVERY_TIMEOUT}s: '
                f'{pending}'
            )
        print(f'Operations ready: {dict(cls.REQUIRED_OPERATIONS)}')

    # ------------------------------------------------------------------
    # HTTP helpers
    # ------------------------------------------------------------------

    def get_json(self, endpoint, *, timeout=10, expected_status=200):
        """Send GET and return parsed JSON.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL`` (e.g. ``'/apps'``).
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code. Assertion fails otherwise.

        Returns
        -------
        dict
            Parsed JSON response body.

        """
        response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=timeout)
        self.assertEqual(
            response.status_code,
            expected_status,
            f'GET {endpoint} returned {response.status_code}: {response.text}',
        )
        return response.json()

    def post_json(self, endpoint, data, *, timeout=10, expected_status=200):
        """Send POST with JSON body and return parsed JSON.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        data : dict
            JSON-serializable request body.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code. Assertion fails otherwise.

        Returns
        -------
        dict
            Parsed JSON response body.

        """
        response = requests.post(
            f'{self.BASE_URL}{endpoint}', json=data, timeout=timeout
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'POST {endpoint} returned {response.status_code}: {response.text}',
        )
        return response.json()

    def put_json(self, endpoint, data, *, timeout=10, expected_status=200):
        """Send PUT with JSON body and return parsed JSON.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        data : dict
            JSON-serializable request body.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code. Assertion fails otherwise.

        Returns
        -------
        dict
            Parsed JSON response body.

        """
        response = requests.put(
            f'{self.BASE_URL}{endpoint}', json=data, timeout=timeout
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'PUT {endpoint} returned {response.status_code}: {response.text}',
        )
        return response.json()

    def delete_request(self, endpoint, *, timeout=10, expected_status=204):
        """Send DELETE and return the raw ``Response``.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code (default 204 No Content).

        Returns
        -------
        requests.Response
            The raw response object.

        """
        response = requests.delete(
            f'{self.BASE_URL}{endpoint}', timeout=timeout
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'DELETE {endpoint} returned {response.status_code}: {response.text}',
        )
        return response

    def get_raw(self, endpoint, *, timeout=10, expected_status=200, **kwargs):
        """Send GET and return the raw ``Response`` (no JSON parsing).

        Useful for binary downloads, streaming, or inspecting headers.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code.
        **kwargs
            Extra keyword arguments forwarded to ``requests.get()``
            (e.g. ``stream=True``).

        Returns
        -------
        requests.Response
            The raw response object.

        """
        response = requests.get(
            f'{self.BASE_URL}{endpoint}', timeout=timeout, **kwargs
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'GET {endpoint} returned {response.status_code}: {response.text}',
        )
        return response

    def put_raw(self, endpoint, data, *, timeout=10, expected_status=200, **kwargs):
        """Send PUT with JSON body and return the raw ``Response`` (no JSON parsing).

        Useful when the caller needs to inspect headers or the response body
        without asserting it is valid JSON (e.g. testing error responses).

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        data : dict
            JSON-serializable request body.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code.
        **kwargs
            Extra keyword arguments forwarded to ``requests.put()``.

        Returns
        -------
        requests.Response
            The raw response object.

        """
        response = requests.put(
            f'{self.BASE_URL}{endpoint}', json=data, timeout=timeout, **kwargs
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'PUT {endpoint} returned {response.status_code}: {response.text}',
        )
        return response

    # ------------------------------------------------------------------
    # Waiters
    # ------------------------------------------------------------------

    def poll_endpoint(self, endpoint, *, timeout=10.0, interval=0.2):
        """Poll GET endpoint until it returns 200 and return parsed JSON.

        Useful for waiting on entities or resources that may take time to
        become available (e.g. after discovery, service-only nodes that
        can transiently disappear from the ROS 2 graph between cycles).

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL`` (e.g. ``'/apps/calibration/data'``).
        timeout : float
            Maximum time to wait in seconds.
        interval : float
            Sleep between retries in seconds.

        Returns
        -------
        dict
            Parsed JSON response body.

        Raises
        ------
        AssertionError
            If the deadline is reached without a 200 response.

        """
        return self.poll_endpoint_until(
            endpoint,
            condition=None,
            timeout=timeout,
            interval=interval,
        )

    def poll_endpoint_until(
        self,
        endpoint,
        condition=None,
        *,
        timeout=10.0,
        interval=0.2,
    ):
        """Poll GET endpoint until it returns 200 and a condition is met.

        When *condition* is ``None`` (the default), any 200 response succeeds
        and the full parsed JSON is returned -- equivalent to
        :meth:`poll_endpoint`.

        When *condition* is a callable, it receives the parsed JSON on each
        200 response.  If it returns a truthy value, that value is returned
        to the caller.  If it returns a falsy value, polling continues.
        This allows both boolean checks and value extraction::

            # Boolean -- wait for non-empty items list
            data = self.poll_endpoint_until(
                '/apps/lidar_sensor/faults',
                lambda d: d.get('items'),
            )

            # Extraction -- return the first fault item directly
            fault = self.poll_endpoint_until(
                '/apps/lidar_sensor/faults',
                lambda d: next(iter(d.get('items', [])), None),
            )

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        condition : callable or None
            ``condition(response_json) -> truthy_value``.  When ``None``,
            any 200 response is accepted.
        timeout : float
            Maximum time to wait in seconds.
        interval : float
            Sleep between retries in seconds.

        Returns
        -------
        object
            Parsed JSON (when *condition* is ``None``) or whatever
            *condition* returned.

        Raises
        ------
        AssertionError
            If the deadline is reached without satisfying the condition.

        """
        start_time = time.monotonic()
        last_error = None
        while time.monotonic() - start_time < timeout:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{endpoint}', timeout=2
                )
                if response.status_code == 200:
                    data = response.json()
                    if condition is None:
                        return data
                    result = condition(data)
                    if result:
                        return result
                    last_error = 'Condition not met'
                else:
                    last_error = f'Status {response.status_code}'
            except requests.exceptions.RequestException as e:
                last_error = str(e)
            time.sleep(interval)
        self.fail(
            f'GET {endpoint} not available after {timeout}s. '
            f'Last error: {last_error}'
        )

    def wait_for_fault(self, entity_endpoint, fault_code, *, max_wait=FAULT_TIMEOUT):
        """Poll until a specific fault appears on an entity.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL`` (e.g. ``'/apps/lidar_sensor'``).
        fault_code : str
            The ``fault_code`` value to wait for.
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        dict
            The fault object from the ``items`` list when found.

        Raises
        ------
        AssertionError
            If the fault is not found within *max_wait*.

        """
        start_time = time.monotonic()
        while time.monotonic() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{entity_endpoint}/faults', timeout=5
                )
                if response.status_code == 200:
                    data = response.json()
                    for fault in data.get('items', []):
                        if fault.get('fault_code') == fault_code:
                            return fault
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'Fault {fault_code} not found at {entity_endpoint}/faults '
            f'within {max_wait}s'
        )

    def wait_for_fault_detail(
        self,
        entity_endpoint,
        *,
        snapshot_types=None,
        max_wait=SNAPSHOT_TIMEOUT,
    ):
        """Wait for a fault to appear and return its full detail response.

        Polls the faults list until a fault with a ``fault_code`` appears,
        then retrieves the detail endpoint. If *snapshot_types* is given,
        continues polling the detail endpoint until all requested snapshot
        types are present in ``environment_data.snapshots`` (snapshots are
        captured asynchronously by the fault manager).

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL``
            (e.g. ``'/apps/lidar_sensor'``).
        snapshot_types : set or list of str, optional
            Snapshot ``type`` values that must be present before returning
            (e.g. ``{'freeze_frame', 'rosbag'}``). When ``None``, no
            snapshot check is performed.
        max_wait : float
            Maximum total time to wait in seconds (covers both fault
            appearance and snapshot capture).

        Returns
        -------
        dict
            The full fault detail JSON from
            ``GET {entity_endpoint}/faults/{fault_code}``.

        Raises
        ------
        AssertionError
            If the fault or required snapshots are not found within
            *max_wait*.

        """
        fault_code = self.poll_endpoint_until(
            f'{entity_endpoint}/faults',
            lambda d: next(
                (
                    item.get('fault_code')
                    for item in d.get('items', [])
                    if item.get('fault_code')
                ),
                None,
            ),
            timeout=max_wait,
            interval=1.0,
        )

        if not snapshot_types:
            return self.get_json(f'{entity_endpoint}/faults/{fault_code}')

        required = set(snapshot_types)

        def _find_fault_with_snapshots(listing_data):
            """Check all faults for required snapshot types.

            The rosbag ring buffer is shared across faults.  When multiple
            faults confirm simultaneously, only one may get the buffer data.
            Iterate over all faults to find the one that succeeded.
            """
            for item in listing_data.get('items', []):
                code = item.get('fault_code')
                if not code:
                    continue
                url = f'{self.BASE_URL}{entity_endpoint}/faults/{code}'
                try:
                    resp = requests.get(url, timeout=5)
                    if resp.status_code != 200:
                        continue
                    detail = resp.json()
                except (requests.RequestException, ValueError):
                    continue
                snapshots = detail.get('environment_data', {}).get(
                    'snapshots', []
                )
                found = {s.get('type') for s in snapshots}
                if required.issubset(found):
                    return detail
            return None

        return self.poll_endpoint_until(
            f'{entity_endpoint}/faults',
            _find_fault_with_snapshots,
            timeout=max_wait,
            interval=1.0,
        )

    def wait_for_fault_with_rosbag(self, entity_endpoint, *, max_wait=ROSBAG_TIMEOUT):
        """Poll until a rosbag snapshot is available for an entity.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL`` (e.g. ``'/apps/lidar_sensor'``).
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        str
            The rosbag ``id`` when found.

        Raises
        ------
        AssertionError
            If no rosbag is found within *max_wait*.

        """
        start_time = time.monotonic()
        while time.monotonic() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{entity_endpoint}/bulk-data/rosbags',
                    timeout=5,
                )
                if response.status_code == 200:
                    data = response.json()
                    items = data.get('items', [])
                    if len(items) > 0:
                        return items[0].get('id')
            except requests.exceptions.RequestException:
                pass
            time.sleep(1)
        raise AssertionError(
            f'No rosbag found at {entity_endpoint}/bulk-data/rosbags '
            f'within {max_wait}s'
        )

    def create_execution(self, entity_endpoint, operation_id, *, input_data=None):
        """POST to create an operation execution and return the response JSON.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL``
            (e.g. ``'/apps/long_calibration'``).
        operation_id : str
            The operation ID to execute.
        input_data : dict or None
            Input parameters for the execution. Defaults to ``{}``.

        Returns
        -------
        tuple[requests.Response, dict]
            ``(response, data)`` tuple. Callers should check
            ``response.status_code`` (202 for async, 200 for sync).

        """
        if input_data is None:
            input_data = {}
        url = f'{self.BASE_URL}{entity_endpoint}/operations/{operation_id}/executions'
        response = requests.post(url, json={'parameters': input_data}, timeout=15)
        self.assertIn(
            response.status_code, (200, 201, 202),
            f'POST {entity_endpoint}/operations/{operation_id}/executions '
            f'returned {response.status_code}: {response.text}',
        )
        data = response.json()
        return response, data

    def wait_for_execution_status(
        self, exec_endpoint, target_statuses, *, max_wait=ACTION_TIMEOUT
    ):
        """Poll execution status until it reaches one of the target statuses.

        Parameters
        ----------
        exec_endpoint : str
            Full execution path relative to ``BASE_URL``
            (e.g. ``'/apps/long_calibration/operations/long_calibration/
            executions/<id>'``).
        target_statuses : list of str
            SOVD status strings to wait for (e.g. ``['completed', 'failed']``).
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        dict
            The status response data when target status is reached.

        Raises
        ------
        AssertionError
            If no target status is reached within *max_wait*.

        """
        start_time = time.monotonic()
        last_status = None
        while time.monotonic() - start_time < max_wait:
            try:
                status_response = requests.get(
                    f'{self.BASE_URL}{exec_endpoint}', timeout=5
                )
                if status_response.status_code == 200:
                    data = status_response.json()
                    last_status = data.get('status')
                    if last_status in target_statuses:
                        return data
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'Execution did not reach status {target_statuses} within {max_wait}s. '
            f'Last status: {last_status}'
        )

    def wait_for_operation(self, entity_endpoint, operation_id, *, max_wait=15.0):
        """Wait for an operation to be discovered for an entity.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL``
            (e.g. ``'/apps/calibration'``).
        operation_id : str
            The operation ID to wait for.
        max_wait : float
            Maximum time to wait in seconds.

        Raises
        ------
        AssertionError
            If the operation is not found within *max_wait*.

        """
        start_time = time.monotonic()
        while time.monotonic() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{entity_endpoint}/operations', timeout=5
                )
                if response.status_code == 200:
                    ops = response.json().get('items', [])
                    if any(op.get('id') == operation_id for op in ops):
                        return
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'Operation {operation_id} not discovered at '
            f'{entity_endpoint}/operations within {max_wait}s'
        )

    def wait_for_operation_type_info(
        self, entity_endpoint, operation_id, required_keys, *, max_wait=15.0
    ):
        """Wait until an operation exposes a resolved ``type_info`` schema.

        An operation appears under ``/operations`` as soon as it is discovered
        by name, but its ``x-medkit.type_info`` is only populated once the
        gateway has resolved the underlying ROS interface type: while
        ``ros2.type`` is still empty, or the type-introspection lookup has not
        yet loaded the type, the handler omits ``type_info`` entirely. Polling
        the operation-name endpoint alone (``wait_for_operation``) races that
        second resolution step, so wait until ``type_info`` actually carries the
        expected schema keys (``goal``/``result``/``feedback`` for an action,
        ``request``/``response`` for a service) before asserting on it.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL`` (e.g. ``'/apps/long_calibration'``).
        operation_id : str
            The operation ID (matched against each item's ``id`` or ``name``).
        required_keys : iterable of str
            Schema keys that must all be present in ``type_info``.
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        dict
            The operation item whose ``type_info`` carries all *required_keys*.

        Raises
        ------
        AssertionError
            If no such operation is found within *max_wait*.

        """
        required = tuple(required_keys)

        def _resolved(data):
            for op in data.get('items', []):
                if op.get('id') != operation_id and op.get('name') != operation_id:
                    continue
                type_info = op.get('x-medkit', {}).get('type_info')
                if isinstance(type_info, dict) and all(
                    key in type_info for key in required
                ):
                    return op
            return None

        return self.poll_endpoint_until(
            f'{entity_endpoint}/operations',
            _resolved,
            timeout=max_wait,
            interval=0.5,
        )

    def wait_for_data_item(self, entity_endpoint, direction, *, timeout=10.0):
        """Poll entity data until an item with the given direction appears.

        Subscriber-only topics may take an extra discovery cycle to become
        visible (especially on CycloneDDS / Humble).  This helper retries
        instead of failing on the first attempt.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL``
            (e.g. ``'/apps/actuator'``).
        direction : str
            Topic direction to search for (``'publish'``, ``'subscribe'``,
            or ``'both'``).
        timeout : float
            Maximum time to wait in seconds.

        Returns
        -------
        dict
            The first data item matching the requested direction.

        Raises
        ------
        AssertionError
            If no matching item is found within *timeout*.

        """

        def _find_item(data):
            for item in data.get('items', []):
                x_medkit = item.get('x-medkit', {})
                ros2 = x_medkit.get('ros2', {})
                if ros2.get('direction') == direction:
                    return item
            return None

        return self.poll_endpoint_until(
            f'{entity_endpoint}/data',
            condition=_find_item,
            timeout=timeout,
        )

    # ------------------------------------------------------------------
    # Discovery assertions
    # ------------------------------------------------------------------

    def assert_entity_exists(self, entity_type, entity_id):
        """Assert that an entity of the given type and ID exists.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        entity_id : str
            The entity ID to look up.

        Returns
        -------
        dict
            The entity JSON data.

        """
        response = requests.get(
            f'{self.BASE_URL}/{entity_type}/{entity_id}', timeout=10
        )
        self.assertEqual(
            response.status_code,
            200,
            f'{entity_type}/{entity_id} not found: '
            f'{response.status_code} {response.text}',
        )
        return response.json()

    def assert_entity_not_found(self, entity_type, entity_id):
        """Assert that an entity returns 404.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        entity_id : str
            The entity ID to look up.

        """
        response = requests.get(
            f'{self.BASE_URL}/{entity_type}/{entity_id}', timeout=10
        )
        self.assertEqual(
            response.status_code,
            404,
            f'Expected 404 for {entity_type}/{entity_id}, '
            f'got {response.status_code}: {response.text}',
        )

    def assert_entity_has_capabilities(
        self, entity_type, entity_id, expected_capabilities
    ):
        """Assert that an entity advertises the given capability collections.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        entity_id : str
            The entity ID to inspect.
        expected_capabilities : list of str
            Collection names expected in the entity's ``capabilities`` field
            (e.g. ``['data', 'operations', 'configurations']``).

        """
        data = self.assert_entity_exists(entity_type, entity_id)
        capabilities = data.get('capabilities', [])
        for cap in expected_capabilities:
            self.assertIn(
                cap,
                capabilities,
                f'{entity_type}/{entity_id} missing capability {cap!r}. '
                f'Has: {capabilities}',
            )

    def assert_entity_list_contains(self, entity_type, expected_ids):
        """Assert that a collection listing contains the given IDs.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        expected_ids : set or list of str
            Entity IDs that must appear in the listing.

        """
        data = self.get_json(f'/{entity_type}')
        items = data.get('items', [])
        found_ids = {item.get('id', '') for item in items}
        missing = set(expected_ids) - found_ids
        self.assertFalse(
            missing,
            f'Missing {entity_type} IDs: {missing}. Found: {found_ids}',
        )

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def encode_topic_path(topic_path):
        """Encode a ROS topic path for use in URLs.

        Slashes are encoded as ``%2F`` for proper URL routing.

        Example::

            '/powertrain/engine/temperature' -> 'powertrain%2Fengine%2Ftemperature'

        Parameters
        ----------
        topic_path : str
            Full ROS topic path starting with ``'/'``.

        Returns
        -------
        str
            URL-encoded topic path without leading slash.

        Raises
        ------
        ValueError
            If *topic_path* doesn't start with ``'/'``.

        """
        if not topic_path.startswith('/'):
            raise ValueError(f"Topic path must start with '/': {topic_path}")
        # Remove leading slash and encode the rest
        topic_path = topic_path[1:]
        return quote(topic_path, safe='')

    def assert_declared_media_type(
        self, served_content_type, declared_content, *, where, exact,
    ):
        """Assert a served ``Content-Type`` is one the document declares.

        The Tier-2 half of the media-type contract: the OpenAPI ``content``
        keys for a response are only worth anything if the bytes on the wire
        actually arrive under one of them.

        Parameters
        ----------
        served_content_type : str
            Raw ``Content-Type`` response header. Any parameters
            (``; charset=...``, ``; boundary=...``) are stripped before
            matching, since OpenAPI ``content`` keys carry none.
        declared_content : dict
            The operation's ``responses[code]['content']`` mapping.
        where : str
            Human-readable location for the failure message.
        exact : bool
            ``True`` requires a named media-type key and explicitly rejects a
            match that only succeeds via the ``*/*`` catch-all - that is what
            keeps the derivable half of an open set from silently degrading
            into "the wildcard covers it". ``False`` accepts the catch-all and
            asserts it is present, which is the open half of the same set.

        """
        media_type = served_content_type.split(';')[0].strip().lower()
        declared = {k.lower() for k in declared_content}
        self.assertTrue(
            declared, f'{where}: response declares no content at all')
        if exact:
            self.assertIn(
                media_type, declared,
                f'{where}: served {media_type!r}, which the document does not name '
                f'(declares {sorted(declared)}). A wildcard does not count here - '
                'this type is derivable from the code and must be declared.',
            )
            return
        self.assertTrue(
            media_type in declared or '*/*' in declared,
            f'{where}: served {media_type!r}, not covered by {sorted(declared)}',
        )
