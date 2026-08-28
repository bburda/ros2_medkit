^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_opcua
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Config-less discovery. A read-only network scan finds the OPC UA server instead of requiring its endpoint up front (`#509 <https://github.com/selfpatch/ros2_medkit/pull/509>`_), ``auto_browse`` walks the address space recursively and builds the SOVD tree from it (`#510 <https://github.com/selfpatch/ros2_medkit/pull/510>`_), and identity, writability and fault triggers are read from the device itself rather than declared in a node map (`#544 <https://github.com/selfpatch/ros2_medkit/pull/544>`_)
* Native OPC UA Alarms and Conditions become medkit faults with no configuration (`#511 <https://github.com/selfpatch/ros2_medkit/pull/511>`_). Alarms are routed to distinct faults by message substring (`#506 <https://github.com/selfpatch/ros2_medkit/pull/506>`_), ``severity`` is accepted as an alias in ``event_alarms`` (`#501 <https://github.com/selfpatch/ros2_medkit/pull/501>`_), and events that are not conditions are dropped from the alarm path instead of being reported as alarms (`#552 <https://github.com/selfpatch/ros2_medkit/pull/552>`_)
* Asset identity is populated from the device nameplate, including ``order_code`` (`#492 <https://github.com/selfpatch/ros2_medkit/pull/492>`_, `#499 <https://github.com/selfpatch/ros2_medkit/pull/499>`_)
* ``PLC_COMMS_LOST`` is raised when the connection to the PLC drops, so a lost link is a reported fault rather than an absence of data (`#507 <https://github.com/selfpatch/ros2_medkit/pull/507>`_)
* Security and reconnect hardening: a secure connection profile, replay of alarm state after a reconnect, and correct handling of several simultaneous alarms (`#485 <https://github.com/selfpatch/ros2_medkit/pull/485>`_)
* Fixed a data race between the REST thread and the poll thread on the pending-report buffer that crashed the gateway with a SIGSEGV (`#519 <https://github.com/selfpatch/ros2_medkit/pull/519>`_, `#521 <https://github.com/selfpatch/ros2_medkit/issues/521>`_)
* Fault detection moved onto the shared ``ros2_medkit_fault_detection`` evaluator, and the node map enforces global fault-code uniqueness so two rules cannot claim the same code (`#486 <https://github.com/selfpatch/ros2_medkit/pull/486>`_)
* This package is still excluded from the rosdistro binary release pending vendoring of open62541pp (`#366 <https://github.com/selfpatch/ros2_medkit/issues/366>`_)
* Contributors: @bburda, @mfaferek93

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Native OPC-UA Part 9 ``AlarmConditionType`` event subscription. The plugin now subscribes to vendor-defined alarms (Siemens S7-1500 ``Program_Alarm`` / ProDiag, Beckhoff TF6100, CodeSys 3.5+, Rockwell via FactoryTalk Linx) and bridges each event into the SOVD fault lifecycle. Configured via a new top-level ``event_alarms:`` block in the node map YAML; mutually exclusive per entry with the existing threshold-based ``alarm`` form. (issue #386)
* New SOVD operations on entities that host alarm sources: ``acknowledge_fault`` invokes the inherited ``Acknowledge`` method on the live ``ConditionId`` (i=9111, EventId tracked per Part 9 §5.7.3); ``confirm_fault`` invokes ``Confirm`` (i=9113). Both accept an optional ``comment`` rendered as ``LocalizedText`` on the server.
* ``OpcuaClient`` gains ``add_event_monitored_item`` / ``remove_event_monitored_item`` / ``call_method`` and a generation counter that filters callbacks fired from defunct subscriptions after a reconnect. Heap-owned ``EventCallbackContext`` resolves the open62541pp / raw-C lifetime hazard.
* Header-only ``AlarmStateMachine`` mapping ``EnabledState x ShelvingState x ActiveState x AckedState x ConfirmedState x BranchId`` to SOVD ``CONFIRMED / HEALED / CLEARED / Suppressed``. Full transition table documented in ``design/index.rst``.
* ``ConditionRefresh`` (Server method i=3875) is invoked on subscribe and on every reconnect, with ``RefreshStartEvent`` / ``RefreshEndEvent`` bracketing tracked for diagnostics.
* New ``test_alarm_server`` fixture (open62541-based, full namespace 0 + alarms enabled) emits AlarmConditionType events on stdin commands; integration test ``run_alarm_tests.sh`` runs in CI alongside the existing OpenPLC threshold suite. The fixture builds by default via the workspace ``colcon build`` (gated on ``MEDKIT_OPCUA_BUILD_ALARM_SERVER`` which defaults to ON; ``ExternalProject_Add`` rebuilds open62541 with ``UA_NAMESPACE_ZERO=FULL`` and alarms ON, with a serial sub-build to dodge the upstream ``-j`` race on ``namespace0_generated.c``).
* New CTest wrapper ``test_alarm_server_smoke`` boots the fixture on an ephemeral port and runs the asyncua smoke test against it; skips with CTest exit 77 (treated as pass) when ``asyncua`` is not importable, so iterating on plugin code without the Python dependency does not fail the suite.
* Contributors: @mfaferek93, @bburda

0.4.0 (2026-04-11)
------------------
* Initial release
* ``OpcuaPlugin`` implementation of ``GatewayPlugin`` and ``IntrospectionProvider`` that bridges OPC-UA capable PLCs into the SOVD entity tree
* REST endpoints via the new ``get_routes()`` plugin API: ``x-plc-data``, ``x-plc-operations``, ``x-plc-status``
* Vendor capabilities registered per entity - only PLC-backed apps and the PLC runtime component advertise the ``x-plc-*`` endpoints
* Full OPC 10000-6 section 5.3.1.10 node identifier support (``i=`` numeric, ``s=`` string, ``g=`` GUID, ``b=`` opaque ByteString); example node maps for OpenPLC, Siemens S7-1500 TIA Portal, Beckhoff TwinCAT 3, Allen-Bradley via Kepware and KUKA KR C5
* ``NodeMap`` driven by YAML configuration - same binary serves any OPC-UA compliant server by changing the node map file
* Deterministic entity ordering in ``IntrospectionResult`` output (entries sorted by id)
* Threshold-based PLC alarm detection routed to SOVD faults via ``ros2_medkit_msgs`` services ``ReportFault`` / ``ClearFault``
* Optional bridging of numeric PLC values to ROS 2 ``std_msgs/Float32`` topics from ``set_context()``
* Type-aware writes with per-node range validation
* Robust connection-loss detection: all three OPC-UA client paths (``read_value``, ``read_values``, ``write_value``) mark the connection as dropped on terminal status codes so ``OpcuaPoller`` reconnect kicks in without stalling
* Polling mode (default) and OPC-UA subscription mode, backed by ``open62541pp`` v0.16.0
* Integration test suite against an OpenPLC IEC 61131-3 tank demo container
* Contributors: @mfaferek93
