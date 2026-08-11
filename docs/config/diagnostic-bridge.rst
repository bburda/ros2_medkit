Diagnostic Bridge Configuration
================================

The ``ros2_medkit_diagnostic_bridge`` node converts standard ROS 2 ``/diagnostics`` messages
to fault events, providing a migration path for existing diagnostic infrastructure.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The diagnostic bridge:

1. Subscribes to ``/diagnostics`` (or custom topic)
2. Maps ``DiagnosticStatus`` names to fault codes
3. Reports faults to the FaultManager via ``ReportFault`` service

**Status Mapping:**

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Diagnostic Level
     - Fault Event
     - Severity
   * - OK
     - PASSED
     - (healing event)
   * - WARN
     - FAILED
     - SEVERITY_WARN (1)
   * - ERROR
     - FAILED
     - SEVERITY_ERROR (2)
   * - STALE
     - FAILED
     - SEVERITY_ERROR (2)

Parameters
----------

.. code-block:: yaml

   diagnostic_bridge:
     ros__parameters:
       diagnostics_topic: "/diagnostics"   # Topic to subscribe to
       auto_generate_codes: true           # Auto-generate fault codes from names
       keyvalue_codes: ["fault_code"]      # Take the code from these key-value keys

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Parameter
     - Default
     - Description
   * - ``diagnostics_topic``
     - ``/diagnostics``
     - Topic to subscribe for ``DiagnosticArray`` messages.
   * - ``auto_generate_codes``
     - ``true``
     - Automatically generate fault codes from diagnostic names when no explicit
       mapping exists.
   * - ``keyvalue_codes``
     - ``[]``
     - Keys to look for inside the ``DiagnosticStatus`` key-value pairs. The
       first key in this list that the message carries supplies the fault code
       as its value, so a publisher can name its own code instead of relying on
       a mapping. Checked after ``name_to_code`` and before auto-generation.
       Empty strings in the list are ignored.

Custom Fault Code Mappings
--------------------------

Map specific diagnostic names to custom fault codes using the ``name_to_code`` parameter prefix:

.. code-block:: yaml

   diagnostic_bridge:
     ros__parameters:
       diagnostics_topic: "/diagnostics"
       auto_generate_codes: true
       name_to_code:
         motor_temp: "MOTOR_OVERHEAT"
         battery_level: "LOW_BATTERY"
         camera_driver: "CAMERA_FAILURE"

Or via command line:

.. code-block:: bash

   ros2 run ros2_medkit_diagnostic_bridge diagnostic_bridge \
     --ros-args \
     -p "name_to_code.motor_temp:=MOTOR_OVERHEAT" \
     -p "name_to_code.battery_level:=LOW_BATTERY"

Auto-Generated Fault Codes
~~~~~~~~~~~~~~~~~~~~~~~~~~

When ``auto_generate_codes: true`` and no explicit mapping exists, fault codes are
generated from the diagnostic name:

1. Convert to uppercase
2. Replace spaces, slashes, and dashes with underscores
3. Remove non-alphanumeric characters (except underscore)
4. Prepend ``DIAG_`` prefix

**Examples:**

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Diagnostic Name
     - Generated Fault Code
   * - ``motor_temp``
     - ``DIAG_MOTOR_TEMP``
   * - ``/camera/driver``
     - ``DIAG_CAMERA_DRIVER``
   * - ``Battery Level Monitor``
     - ``DIAG_BATTERY_LEVEL_MONITOR``

Launch File Configuration
-------------------------

Example launch file:

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='ros2_medkit_diagnostic_bridge',
               executable='diagnostic_bridge_node',
               name='diagnostic_bridge',
               parameters=[{
                   'diagnostics_topic': '/diagnostics',
                   'auto_generate_codes': True,
                   'name_to_code': {
                       'motor_temp': 'MOTOR_OVERHEAT',
                       'battery_level': 'LOW_BATTERY',
                   }
               }],
           ),
       ])

Integration with FaultManager
-----------------------------

The diagnostic bridge requires a running FaultManager to report faults. Ensure the
FaultManager is started before the bridge:

.. code-block:: bash

   # Terminal 1: Start FaultManager
   ros2 run ros2_medkit_fault_manager fault_manager_node

   # Terminal 2: Start Diagnostic Bridge
   ros2 run ros2_medkit_diagnostic_bridge diagnostic_bridge_node

Or use a combined launch file:

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='ros2_medkit_fault_manager',
               executable='fault_manager_node',
               name='fault_manager',
           ),
           Node(
               package='ros2_medkit_diagnostic_bridge',
               executable='diagnostic_bridge_node',
               name='diagnostic_bridge',
           ),
       ])

Migration Strategy
------------------

For transitioning from standard ROS 2 diagnostics to direct fault reporting:

1. **Phase 1**: Deploy diagnostic bridge alongside existing diagnostics infrastructure
2. **Phase 2**: Create explicit mappings for important diagnostics
3. **Phase 3**: Migrate critical nodes to direct FaultReporter usage
4. **Phase 4**: Disable auto_generate_codes, rely only on explicit mappings
5. **Phase 5**: Remove diagnostic bridge when all nodes use FaultReporter

.. seealso::

   :doc:`/design/ros2_medkit_fault_reporter/index` - Direct fault reporting with FaultReporter

See Also
--------

- :doc:`fault-manager` - FaultManager configuration
- :doc:`/design/ros2_medkit_fault_reporter/index` - Direct fault reporting guide
- :doc:`/api/messages` - ReportFault service definition
- :doc:`/design/ros2_medkit_diagnostic_bridge/index` - Bridge architecture
