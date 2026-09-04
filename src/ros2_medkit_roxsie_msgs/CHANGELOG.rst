^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_roxsie_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial interface definitions for the ROXSIE integration: ``Alarm.msg`` and ``AlarmList.msg`` for the cyclic picture of alarms in the PLC, ``DiagnosticsStatus.msg`` for the condition sent back into the PLC.
* ``AlarmList``: ``catalog_version`` and ``truncated`` added, and the open questions on alarm id scope, text tables, block size and acknowledgement closed with the Siemens answers of 2026-09-03.
