Linux Introspection Plugins
===========================

The ``ros2_medkit_linux_introspection`` package provides three plugins that enrich the
gateway with OS-level metadata for ROS 2 nodes. Each plugin implements the
``IntrospectionProvider`` interface and registers vendor-specific REST endpoints on
Apps and Components.

- **procfs** - reads ``/proc`` for process info (PID, RSS, CPU ticks, threads, exe path,
  cmdline). Works on any Linux system.
- **systemd** - maps ROS 2 nodes to systemd units via ``sd_pid_get_unit()``, then queries
  unit properties (ActiveState, SubState, NRestarts, WatchdogUSec) via sd-bus. Requires
  ``libsystemd``.
- **container** - detects containerization via cgroup path analysis. Supports Docker,
  podman, and containerd. Reads resource limits from the unified hierarchy
  (``memory.max``, ``cpu.max``) and from the legacy one (``memory.limit_in_bytes``,
  ``cpu.cfs_quota_us``, ``cpu.cfs_period_us``).

Each plugin maintains its own PID cache that maps ROS 2 node fully-qualified names to
Linux PIDs by scanning ``/proc``. The cache refreshes on each discovery cycle and on
demand when the TTL expires.

Requirements
------------

- **procfs**: Linux only (reads ``/proc`` filesystem). No extra dependencies.
- **systemd**: requires ``libsystemd-dev`` at build time, systemd at runtime. Skipped
  automatically if ``libsystemd`` is not found during the build.
- **container**: Linux only. Works on the unified cgroup hierarchy (v2, the default on
  Ubuntu 22.04+ and Fedora 31+), on the legacy hierarchy (v1), and on hybrid hosts that
  mount both. Both cgroup namespace modes (``--cgroupns=host`` and ``--cgroupns=private``)
  are handled.

Building
--------

The plugins build as part of the ros2_medkit colcon workspace:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select ros2_medkit_linux_introspection

Verify the ``.so`` files are installed:

.. code-block:: bash

   ls install/ros2_medkit_linux_introspection/lib/ros2_medkit_linux_introspection/
   # libprocfs_introspection.so
   # libsystemd_introspection.so  (only if libsystemd was found)
   # libcontainer_introspection.so

.. note::

   The systemd plugin is conditionally built. If ``libsystemd-dev`` is not installed,
   CMake prints a warning and skips it. Install with ``sudo apt install libsystemd-dev``
   on Ubuntu/Debian.

Configuration
-------------

Add plugins to ``gateway_params.yaml``. You can enable any combination - each plugin
is independent:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["procfs", "systemd", "container"]
       # Paths are relative to the colcon workspace root (where you run 'ros2 launch').
       # Use absolute paths if launching from a different directory.
       plugins.procfs.path: "install/ros2_medkit_linux_introspection/lib/ros2_medkit_linux_introspection/libprocfs_introspection.so"
       plugins.procfs.pid_cache_ttl_seconds: 10
       plugins.systemd.path: "install/ros2_medkit_linux_introspection/lib/ros2_medkit_linux_introspection/libsystemd_introspection.so"
       plugins.systemd.pid_cache_ttl_seconds: 10
       plugins.container.path: "install/ros2_medkit_linux_introspection/lib/ros2_medkit_linux_introspection/libcontainer_introspection.so"
       plugins.container.pid_cache_ttl_seconds: 10

Or enable just one plugin:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["procfs"]
       plugins.procfs.path: "/opt/ros2_medkit/lib/ros2_medkit_linux_introspection/libprocfs_introspection.so"

Configuration parameters (all plugins):

``pid_cache_ttl_seconds`` (int, default 10)
   TTL in seconds for the PID cache. The cache maps ROS 2 node FQNs to PIDs by scanning
   ``/proc``. Lower values give fresher data but increase ``/proc`` scan frequency.

``proc_root`` (string, default ``"/"``)
   Root path for ``/proc`` access. Primarily used for testing with synthetic ``/proc``
   trees. In production, leave at the default.

See :doc:`/tutorials/plugin-system` for general plugin configuration details.

API Reference
-------------

Each plugin registers vendor-specific endpoints on Apps (individual nodes) and Components
(aggregated across child nodes).

procfs Endpoints
~~~~~~~~~~~~~~~~

**GET /apps/{id}/x-medkit-procfs**

Returns process-level metrics for a single ROS 2 node:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/temp_sensor/x-medkit-procfs | jq

.. code-block:: json

   {
     "pid": 12345,
     "ppid": 1,
     "exe": "/opt/ros/jazzy/lib/demo_nodes_cpp/talker",
     "cmdline": "/opt/ros/jazzy/lib/demo_nodes_cpp/talker --ros-args ...",
     "rss_bytes": 15728640,
     "vm_size_bytes": 268435456,
     "threads": 4,
     "cpu_user_ticks": 1500,
     "cpu_system_ticks": 300,
     "uptime_seconds": 3600
   }

**GET /components/{id}/x-medkit-procfs**

Returns aggregated process info for all child Apps of a Component, deduplicated by PID.
Each entry includes a ``node_ids`` array listing the Apps that share the process:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/sensor_suite/x-medkit-procfs | jq

.. code-block:: json

   {
     "processes": [
       {
         "pid": 12345,
         "ppid": 1,
         "exe": "/opt/ros/jazzy/lib/sensor_pkg/sensor_node",
         "cmdline": "/opt/ros/jazzy/lib/sensor_pkg/sensor_node --ros-args ...",
         "rss_bytes": 15728640,
         "vm_size_bytes": 268435456,
         "threads": 4,
         "cpu_user_ticks": 1500,
         "cpu_system_ticks": 300,
         "uptime_seconds": 3600,
         "node_ids": ["temp_sensor", "rpm_sensor"]
       }
     ]
   }

systemd Endpoints
~~~~~~~~~~~~~~~~~

**GET /apps/{id}/x-medkit-systemd**

Returns the systemd unit managing the node's process:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/temp_sensor/x-medkit-systemd | jq

.. code-block:: json

   {
     "unit": "ros2-demo-temp-sensor.service",
     "unit_type": "service",
     "active_state": "active",
     "sub_state": "running",
     "restart_count": 0,
     "watchdog_usec": 0
   }

**GET /components/{id}/x-medkit-systemd**

Returns aggregated unit info for all child Apps, deduplicated by unit name:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/sensor_suite/x-medkit-systemd | jq

.. code-block:: json

   {
     "units": [
       {
         "unit": "ros2-demo.service",
         "unit_type": "service",
         "active_state": "active",
         "sub_state": "running",
         "restart_count": 0,
         "watchdog_usec": 0,
         "node_ids": ["temp_sensor", "rpm_sensor"]
       }
     ]
   }

container Endpoints
~~~~~~~~~~~~~~~~~~~

**GET /apps/{id}/x-medkit-container**

Returns container metadata for a node running inside a container:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/temp_sensor/x-medkit-container | jq

.. code-block:: json

   {
     "container_id": "a1b2c3d4e5f6...",
     "runtime": "docker",
     "memory_limit_bytes": 536870912,
     "memory_limit_state": "limited",
     "cpu_quota_us": 100000,
     "cpu_period_us": 100000,
     "cpu_quota_state": "limited"
   }

.. note::

   The ``memory_limit_bytes``, ``cpu_quota_us``, and ``cpu_period_us`` fields are only
   present when a limit was actually read. ``memory_limit_state`` and ``cpu_quota_state``
   are always present and say why a number is missing:

   - ``limited`` - a limit is in force, and the numeric field carries it
   - ``unlimited`` - the container may use the whole machine
   - ``unreadable`` - a limit file was found but its contents could not be parsed
   - ``unavailable`` - no limit file exists in any supported cgroup layout

   The distinction matters: an unreadable limit file reported as "no limit" would look
   exactly like an unconstrained container. The CPU limit is the quota together with its
   period, so ``cpu_quota_state`` covers both and ``cpu_period_us`` has no state of its
   own; the period is reported even when the quota is ``unlimited``.

**GET /components/{id}/x-medkit-container**

Returns aggregated container info for all child Apps, deduplicated by container ID:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/sensor_suite/x-medkit-container | jq

.. code-block:: json

   {
     "containers": [
       {
         "container_id": "a1b2c3d4e5f6...",
         "runtime": "docker",
         "memory_limit_bytes": 536870912,
         "memory_limit_state": "limited",
         "cpu_quota_us": 100000,
         "cpu_period_us": 100000,
         "cpu_quota_state": "limited",
         "node_ids": ["temp_sensor", "rpm_sensor"]
       }
     ]
   }

Error Responses
---------------

All endpoints return SOVD-compliant ``GenericError`` responses on failure. Entity
validation errors (404 for unknown entities) are handled automatically by
``validate_entity_for_route()``. Plugin-specific errors:

+-----+---------------------------------------+-----------------------------------------------+
| Code| Error ID                              | Description                                   |
+=====+=======================================+===============================================+
| 404 | ``x-medkit-pid-lookup-failed``        | PID not found for node. The node may not be   |
|     |                                       | running, or the PID cache has not refreshed.  |
+-----+---------------------------------------+-----------------------------------------------+
| 503 | ``x-medkit-proc-read-failed``         | Failed to read ``/proc/{pid}`` info. Process  |
|     |                                       | may have exited between PID lookup and read.  |
+-----+---------------------------------------+-----------------------------------------------+
| 404 | ``x-medkit-not-in-systemd-unit``      | Node's process is not managed by a systemd    |
|     |                                       | unit. It may have been started manually.      |
+-----+---------------------------------------+-----------------------------------------------+
| 503 | ``x-medkit-systemd-query-failed``     | Failed to query systemd properties via sd-bus.|
|     |                                       | Check D-Bus socket access.                    |
+-----+---------------------------------------+-----------------------------------------------+
| 404 | ``x-medkit-not-containerized``        | Node's process is not running inside a        |
|     |                                       | container (no container cgroup path detected).|
+-----+---------------------------------------+-----------------------------------------------+
| 503 | ``x-medkit-cgroup-read-failed``       | The cgroup of the process could not be        |
|     |                                       | determined at all. Check access to            |
|     |                                       | ``/proc/{pid}/cgroup``. A limit that could    |
|     |                                       | not be read is reported as a state on a 200,  |
|     |                                       | not as this error.                            |
+-----+---------------------------------------+-----------------------------------------------+

.. note::

   Component-level endpoints (``/components/{id}/x-medkit-*``) silently skip child Apps
   that cannot be resolved. They return partial results rather than failing entirely.

Composable Nodes
----------------

When multiple ROS 2 nodes share a process (composable nodes / component containers),
they share the same PID. The plugins handle this correctly:

- **procfs**: the Component endpoint deduplicates by PID. A single process entry
  includes all node IDs that share it in the ``node_ids`` array.
- **systemd**: the Component endpoint deduplicates by unit name. Composable nodes in
  the same process always map to the same systemd unit.
- **container**: the Component endpoint deduplicates by container ID. All nodes sharing
  a container appear in one entry.

App-level endpoints always return data for the single process hosting that node,
regardless of how many other nodes share the same process.

Introspection Metadata
----------------------

Plugin introspection data is accessed via the vendor extension endpoints registered by
each plugin (e.g., ``GET /apps/{id}/x-medkit-procfs``). The ``IntrospectionProvider``
interface enriches the discovery pipeline with capabilities and metadata fields, but the
detailed introspection data is served through the plugin's own HTTP routes rather than
embedded in standard discovery responses.

Troubleshooting
---------------

PID lookup failures
~~~~~~~~~~~~~~~~~~~

The PID cache refreshes when its TTL expires (default 10 seconds). If a node was just
started, the cache may not have picked it up yet. Causes:

- Node started after the last cache refresh. Wait for the next refresh cycle.
- Node name mismatch. The PID cache matches ROS 2 node FQNs (e.g., ``/sensors/temp``)
  against ``/proc/{pid}/cmdline`` entries. Ensure the node's ``--ros-args -r __node:=``
  and ``-r __ns:=`` match expectations.
- Node exited. The process may have crashed between the cache refresh and the REST
  request.

Composable nodes
~~~~~~~~~~~~~~~~

Composable nodes loaded via ``ros2 component load`` into a component container do not
have ``__node:=`` or ``__ns:=`` arguments in their ``/proc/{pid}/cmdline``. Node names
are set programmatically via ``rclcpp::NodeOptions`` rather than through command-line
arguments. As a result, the PID cache cannot resolve these nodes and they will appear
as unreachable in all introspection endpoints.

**Workaround**: Launch composable nodes via ``ros2 launch`` with explicit remapping
arguments (``--ros-args -r __node:=<name> -r __ns:=<namespace>``) instead of loading
them dynamically with ``ros2 component load``.

Permission errors (procfs)
~~~~~~~~~~~~~~~~~~~~~~~~~~

Most ``/proc/{pid}`` files are world-readable. However:

- ``/proc/{pid}/exe`` (symlink to executable) requires same-user access or
  ``CAP_SYS_PTRACE``. If the gateway runs as a different user, the ``exe`` field may
  be empty.
- In hardened environments with ``hidepid=2`` mount option on ``/proc``, only processes
  owned by the same user are visible. Run the gateway as root or in the same user
  namespace.

systemd bus access
~~~~~~~~~~~~~~~~~~

The systemd plugin uses ``sd_bus_open_system()`` to connect to the system bus, typically
via ``/run/dbus/system_bus_socket``. If the gateway runs in a container:

.. code-block:: bash

   # Mount the host's D-Bus socket into the container
   docker run -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket ...

   # Or run privileged (not recommended for production)
   docker run --privileged ...

Without system bus access, the systemd plugin will return 503 errors for all queries.

Container detection
~~~~~~~~~~~~~~~~~~~

The container plugin relies on cgroup path analysis. To see which hierarchy your
system mounts:

.. code-block:: bash

   mount | grep cgroup
   # unified (v2): cgroup2 on /sys/fs/cgroup type cgroup2 (...)
   # legacy (v1):  cgroup on /sys/fs/cgroup/memory type cgroup (...)

   # Or check a process's cgroup path
   cat /proc/self/cgroup
   # unified (v2): "0::/user.slice/..."
   # legacy (v1):  "12:memory:/docker/<id>" - one line per hierarchy

Both are supported, as is the hybrid layout that mounts the legacy controllers at the
top level and the unified hierarchy beside them at ``/sys/fs/cgroup/unified``.

The reported path is relative to the cgroup namespace of the container, so where the
limit files sit depends on the namespace mode: ``--cgroupns=private`` mounts the
container's own cgroup at the mount point, while ``--cgroupns=host`` exposes the whole
hierarchy and the reported path leads into it. The plugin looks in both places and does
not need to be told which mode is in use.

Supported container runtimes and their cgroup path patterns:

- **Docker**: ``/docker/<64-char-hex>``
- **podman**: ``/libpod-<64-char-hex>.scope``
- **containerd** (CRI): ``/cri-containerd-<64-char-hex>.scope``

If your runtime uses a different cgroup path format, the plugin will not detect the
container. The ``runtime`` field in the response indicates the detected runtime.

Under ``--cgroupns=private`` the reported cgroup path is the namespace root and carries
no container ID at all. The plugin then falls back to the markers a runtime leaves
behind, read through the inspected process's own root (``/proc/<pid>/root``) so that a
containerized gateway does not answer for processes outside it:

- ``/.dockerenv`` (Docker) or ``/run/.containerenv`` (Podman), which also name the runtime
- ``/run/systemd/container`` (systemd-nspawn and others)
- an overlay filesystem mounted as the process's root, but only together with a cgroup
  path of ``/``. An overlay root on its own proves nothing, because whole distributions
  boot that way

The limits are still reported, with ``container_id`` empty. A process that matches none of
these gets ``404 x-medkit-not-containerized``.

A runtime that leaves no marker and does not use an overlay root - containerd or CRI-O on
a btrfs or ZFS snapshotter, for example - is not detected under a private namespace, and
its apps are reported as non-containerized.

CPU limits and cpusets
~~~~~~~~~~~~~~~~~~~~~~

``cpu_quota_us`` and ``cpu_period_us`` describe the CFS bandwidth limit
(``--cpus``/``--cpu-quota``) and nothing else. A container pinned to a subset of the
machine with ``--cpuset-cpus`` has no quota at all, so it is reported as
``"cpu_quota_state": "unlimited"`` even though it cannot use every core. The cpuset is
visible only through ``sched_getaffinity()``; a caller that wants the effective CPU
budget has to combine the two.
