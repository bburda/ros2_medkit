ros2_medkit_linux_introspection
================================

This section contains design documentation for the ros2_medkit_linux_introspection package.

Overview
--------

The ``ros2_medkit_linux_introspection`` package provides three gateway plugins that
enrich discovered entities with Linux-specific runtime metadata. Each plugin
implements both the ``GatewayPlugin`` and ``IntrospectionProvider`` interfaces,
registering vendor extension endpoints (``x-medkit-*``) and feeding metadata into
the discovery merge pipeline.

Architecture
------------

The following diagram shows the three plugins and their shared infrastructure.

.. plantuml::
   :caption: Linux Introspection Plugin Architecture

   @startuml ros2_medkit_linux_introspection_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title Linux Introspection - Plugin Architecture

   package "ros2_medkit_gateway" {
       interface GatewayPlugin {
           +name(): string
           +configure(config): void
           +set_context(ctx): void
           +get_routes(): vector~PluginRoute~
       }

       interface IntrospectionProvider {
           +introspect(input): IntrospectionResult
       }

       class PluginContext {
           +register_capability()
           +validate_entity_for_route()
           +get_child_apps()
       }
   }

   package "ros2_medkit_linux_introspection" {

       class ProcfsPlugin {
           - pid_cache_: PidCache
           - proc_root_: string
           --
           + name(): "procfs_introspection"
           + introspect(): process info per app
           --
           - handle_app_request(): single process
           - handle_component_request(): aggregated
       }

       class SystemdPlugin {
           - pid_cache_: PidCache
           - proc_root_: string
           --
           + name(): "systemd_introspection"
           + introspect(): unit info per app
           --
           - handle_app_request(): single unit
           - handle_component_request(): aggregated
       }

       class ContainerPlugin {
           - pid_cache_: PidCache
           - proc_root_: string
           --
           + name(): "container_introspection"
           + introspect(): cgroup info per app
           --
           - handle_app_request(): single container
           - handle_component_request(): aggregated
       }

       class PidCache {
           - node_to_pid_: map
           - ttl_: duration
           - mutex_: shared_mutex
           --
           + lookup(fqn, root): optional<pid_t>
           + refresh(root): void
           + size(): size_t
       }

       class "proc_reader" as PR <<utility>> {
           + read_process_info(pid): ProcessInfo
           + find_pid_for_node(name, ns): pid_t
           + read_system_uptime(): double
       }

       class "cgroup_reader" as CR <<utility>> {
           + read_cgroup_info(pid): CgroupInfo
       }

       class "systemd_utils" as SU <<utility>> {
           + escape_unit_for_dbus(name): string
       }

       class IntrospectionConfig {
           + pid_cache: PidCache
           + proc_root: string
       }
   }

   ProcfsPlugin .up.|> GatewayPlugin
   ProcfsPlugin .up.|> IntrospectionProvider
   SystemdPlugin .up.|> GatewayPlugin
   SystemdPlugin .up.|> IntrospectionProvider
   ContainerPlugin .up.|> GatewayPlugin
   ContainerPlugin .up.|> IntrospectionProvider

   ProcfsPlugin *--> PidCache
   SystemdPlugin *--> PidCache
   ContainerPlugin *--> PidCache

   ProcfsPlugin --> PR : reads /proc
   ProcfsPlugin --> PluginContext
   SystemdPlugin --> PR : PID lookup
   SystemdPlugin --> SU : D-Bus queries
   SystemdPlugin --> PluginContext
   ContainerPlugin --> CR : reads cgroups
   ContainerPlugin --> PR : PID lookup
   ContainerPlugin --> PluginContext

   @enduml

Plugins
-------

1. **ProcfsPlugin** (``procfs_introspection``) - Process-level metrics from ``/proc``

   - Registers ``x-medkit-procfs`` capability on Apps and Components
   - Reads ``/proc/{pid}/stat``, ``/proc/{pid}/status``, ``/proc/{pid}/cmdline``
   - Exposes: PID, PPID, state, RSS, VM size, CPU ticks, thread count, command line, exe path
   - Component-level endpoint aggregates processes, deduplicating by PID (multiple nodes may share a process)

2. **SystemdPlugin** (``systemd_introspection``) - Systemd unit metadata via sd-bus

   - Registers ``x-medkit-systemd`` capability on Apps and Components
   - Maps PID to systemd unit via ``sd_pid_get_unit()``
   - Queries unit properties over D-Bus: ActiveState, SubState, NRestarts, WatchdogUSec
   - Component-level endpoint aggregates units, deduplicating by unit name
   - Uses RAII wrapper (``SdBusPtr``) for sd-bus connection lifetime

3. **ContainerPlugin** (``container_introspection``) - Container detection from cgroups

   - Registers ``x-medkit-container`` capability on Apps and Components
   - Reads ``/proc/{pid}/cgroup`` to extract container ID and runtime (Docker, containerd, Podman)
   - Handles the unified (v2), legacy (v1) and hybrid hierarchies, and looks for the limit
     files both at the bare mount point (``cgroupns=private``) and at the mount point joined
     with the reported path (``cgroupns=host``)
   - Reports each limit with a ``LimitState`` (``limited``, ``unlimited``, ``unreadable``,
     ``unavailable``) so a failed read cannot be mistaken for an unconstrained container
   - Recognises a container even when the cgroup namespace hides its ID, from
     ``/.dockerenv``, ``/run/.containerenv`` or an overlay root filesystem
   - Component-level endpoint aggregates containers, deduplicating by container ID
   - Returns 404 for entities not running in a container (not an error - just bare-metal)

PidCache
--------

All three plugins share the ``PidCache`` class for mapping ROS 2 node fully-qualified
names to Linux PIDs. The cache:

- Scans ``/proc`` for processes whose command line matches ROS 2 node naming patterns
- Uses TTL-based refresh (default 10 seconds, configurable via ``pid_cache_ttl_seconds``)
- Is thread-safe via ``std::shared_mutex`` (concurrent reads, exclusive writes)
- Each plugin instance owns its own ``PidCache`` (no cross-plugin sharing, avoids lock contention)

The ``proc_root`` configuration parameter (default ``/``) allows testing with a mock
``/proc`` filesystem without running as root.

Vendor Extension Endpoints
--------------------------

Each plugin registers REST endpoints following the SOVD vendor extension pattern:

.. code-block:: text

   GET /api/v1/apps/{entity_id}/x-medkit-procfs
   GET /api/v1/components/{entity_id}/x-medkit-procfs
   GET /api/v1/apps/{entity_id}/x-medkit-systemd
   GET /api/v1/components/{entity_id}/x-medkit-systemd
   GET /api/v1/apps/{entity_id}/x-medkit-container
   GET /api/v1/components/{entity_id}/x-medkit-container

All endpoints validate the entity via ``PluginContext::validate_entity_for_route()``
and return SOVD-compliant error responses on failure.

Design Decisions
----------------

Three Separate Plugins
~~~~~~~~~~~~~~~~~~~~~~

Each Linux subsystem (procfs, systemd, cgroups) is a separate plugin rather than
one monolithic "linux" plugin. This allows deployers to load only what they need -
for example, a containerized deployment may only want the container plugin, while
a systemd-managed robot would use the systemd plugin. Each plugin is an independent
shared library (``.so``) loaded at runtime.

IntrospectionProvider Dual Interface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each plugin implements both ``GatewayPlugin`` (for HTTP routes and capabilities) and
``IntrospectionProvider`` (for the discovery merge pipeline). The ``introspect()``
method feeds metadata into the entity cache during each discovery cycle, while the
HTTP routes serve on-demand queries for individual entities. The C export function
``get_introspection_provider()`` enables the plugin loader to obtain both interfaces
from a single ``.so`` file.

Configurable proc_root
~~~~~~~~~~~~~~~~~~~~~~

The ``proc_root`` parameter defaults to ``/`` but can be overridden to point at a
mock filesystem tree. This enables unit testing of ``/proc`` parsing logic without
root privileges or actual processes, and supports containerized gateway deployments
where the host ``/proc`` is mounted at a non-standard path.
