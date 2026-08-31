# ros2_medkit_integration_tests

Integration tests, demo nodes, and shared test utilities for ros2_medkit.

## Overview

This package contains everything needed to validate the ros2_medkit system
end-to-end:

- **Demo nodes** -- automotive-themed C++ nodes (sensors, actuators, services,
  actions) that produce realistic ROS 2 traffic for testing
- **Feature tests** -- focused tests that validate individual API features
  (data access, operations, faults, SSE, CORS, auth, etc.)
- **Scenario tests** -- end-to-end stories that exercise multi-step workflows
  (fault lifecycle, action execution, discovery modes, subscriptions, etc.)
- **Shared test library** (`ros2_medkit_test_utils`) -- base test case,
  launch helpers, and assertion utilities shared across all test files

## Package Structure

```
ros2_medkit_integration_tests/
  demo_nodes/           # C++ demo node sources
  launch/               # demo_nodes.launch.py
  ros2_medkit_test_utils/
    __init__.py
    constants.py        # DEFAULT_PORT, timeouts
    coverage.py         # GCOV_PREFIX helpers for CI
    gateway_test_case.py  # GatewayTestCase base class
    launch_helpers.py   # create_test_launch(), DEMO_NODE_REGISTRY
  test/
    features/           # Focused API feature tests
    scenarios/          # End-to-end workflow tests
```

## Running Tests

```bash
# Build everything
colcon build --symlink-install
source install/setup.bash

# All integration tests
colcon test --packages-select ros2_medkit_integration_tests
colcon test-result --verbose

# Single test
colcon test --packages-select ros2_medkit_integration_tests \
  --ctest-args -R test_data_read

# Feature tests only
colcon test --packages-select ros2_medkit_integration_tests \
  --ctest-args -L feature

# Scenario tests only
colcon test --packages-select ros2_medkit_integration_tests \
  --ctest-args -L scenario
```

## Demo Nodes

Launch all demo nodes for manual testing or the web UI:

```bash
ros2 launch ros2_medkit_integration_tests demo_nodes.launch.py
```

| Node | Namespace | Type | Description |
|------|-----------|------|-------------|
| `temp_sensor` | `/powertrain/engine` | Publisher | Engine temperature (2 Hz) |
| `rpm_sensor` | `/powertrain/engine` | Publisher | RPM readings (2 Hz) |
| `pressure_sensor` | `/chassis/brakes` | Publisher | Brake pressure (2 Hz) |
| `status_sensor` | `/body/door/front_left` | Publisher | Door open/closed (0.5 Hz) |
| `lidar_sensor` | `/perception/lidar` | Publisher + Fault reporter | LaserScan with fault detection |
| `actuator` | `/chassis/brakes` | Subscriber + Publisher | Brake actuator (command/feedback) |
| `controller` | `/body/lights` | Subscriber + Publisher | Light controller (command/status) |
| `calibration` | `/powertrain/engine` | Service | Trigger-based calibration |
| `long_calibration` | `/powertrain/engine` | Action | Fibonacci-based long-running action |
| `dual_calibration` | `/testrig/dual` | Services + Actions | `left/calibrate` and `right/calibrate`, `left/sweep` and `right/sweep` - one provider carrying each operation short name twice, so the ROS path is the only id that separates the copies |

### Crash reports

Every demo binary goes through `run_demo_node()` in
`include/ros2_medkit_integration_tests/demo_node_main.hpp`, which installs a
fatal-signal handler before anything else runs. A node killed by `SIGSEGV`,
`SIGBUS` or `SIGABRT` writes its stack to stderr, so launch_testing captures it
with the rest of the process output:

```
MEDKIT-CRASH signal=SIGSEGV
/path/to/demo_brake_pressure_sensor(+0x129cb) [0x5603199349cb]
/opt/ros/<distro>/lib/libfastdds.so.3(...) [0x...]
MEDKIT-CRASH end
```

Grep a CI log for `MEDKIT-CRASH` to find one. Frames from our own binaries come
back as `binary(+0xOFFSET)` because the release build has no exported symbols;
resolve them with `addr2line -e <binary> <offset>`. The process still dies from
the original signal, so the exit status a test asserts on is unchanged.

This exists because a node that dies during startup otherwise leaves nothing at
all: no output, and no core file, since a container cannot set the host's
`core_pattern`.

## Writing New Tests

### Feature Test Template

Feature tests validate a single API capability:

```python
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )

class TestMyFeature(GatewayTestCase):
    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_something(self):
        data = self.get_json('/apps')
        self.assertGreater(len(data['items']), 0)
```

### Scenario Test Template

Scenario tests exercise multi-step workflows with numbered test methods:

```python
class TestMyScenario(GatewayTestCase):
    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'lidar_sensor'}

    def test_01_first_step(self):
        """Step 1 description. @verifies REQ_XXX"""
        ...

    def test_02_second_step(self):
        """Step 2 depends on step 1."""
        ...
```

### Key GatewayTestCase Methods

- `get_json(path)` -- GET request, assert 200, return parsed JSON
- `delete_request(path, expected_status)` -- DELETE with status check
- `assert_entity_exists(entity_type, entity_id)` -- GET entity, assert exists
- `assert_entity_list_contains(entity_type, ids)` -- assert IDs in list
- `wait_for_fault(endpoint, fault_code)` -- poll until fault appears
- `wait_for_operation(endpoint, op_id)` -- poll until operation discovered
- `create_execution(endpoint, op_id, input_data)` -- POST to create execution
- `wait_for_execution_status(endpoint, statuses)` -- poll execution status

## Requirements Traceability

Tests are tagged with `# @verifies REQ_XXX` in docstrings. Run the
verification script to update the traceability matrix:

```bash
python scripts/generate_verification.py
```

## License

Apache-2.0
