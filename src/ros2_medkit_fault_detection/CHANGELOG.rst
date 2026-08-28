^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Initial release of the package: shared, protocol-agnostic fault-detection model for medkit
  gateway plugins. A single header-only evaluator maps a raw value read from any
  source (OPC UA, S7, Modbus, ADS, ...) into the set of faults it implies, using
  one of three composable detection modes: ``ThresholdRule`` (numeric
  above/below a setpoint), ``StatusWordRule`` (decode named bits of an integer
  status register, with optional source-width masking to drop sign-extended high
  bits), and ``EnumMapRule`` (map a fault-code register value to a fault code +
  text, with an optional catch-all for unmapped values)
  (`#481 <https://github.com/selfpatch/ros2_medkit/issues/481>`_).
* ``evaluate(value, rule)`` is a pure function with no ROS / protocol
  dependencies, so it is trivially unit-testable and safe to compile into a
  dlopen-loaded plugin MODULE. Undecidable input (a non-finite double, a string,
  a failed numeric conversion) yields an empty result so a transition tracker
  holds the prior state instead of clearing a standing fault - a bad read never
  masks a real alarm.
* ``FaultTransitionTracker`` layers stateful raise/clear edge detection on top,
  keyed by ``fault_code`` alone; consumers that share one tracker across many
  points must enforce global fault-code uniqueness at config-load time.
* Shipped as a header-only INTERFACE library (``cxx_std_17``); the ``OPC UA``
  plugin is the first consumer and migrates its threshold / status-bit / enum
  detection onto this module.
* Global fault-code uniqueness is enforced across an OPC UA node map, so two rules cannot claim the same code and leave which one raised it undefined (`#486 <https://github.com/selfpatch/ros2_medkit/pull/486>`_)
* A tracker holds its prior state on an undecidable read instead of clearing a standing fault, and an enum value with no mapping is labelled rather than dropped (`#486 <https://github.com/selfpatch/ros2_medkit/pull/486>`_)
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), and its tests take a DDS domain at run time from the shared allocator (`#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* Contributors: @mfaferek93, @bburda
