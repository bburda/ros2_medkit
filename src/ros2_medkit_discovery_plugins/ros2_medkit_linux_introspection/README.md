# ros2_medkit_linux_introspection

Gateway discovery plugin that exposes Linux process-level diagnostics via vendor extension endpoints.
Maps ROS 2 nodes to OS processes and reports CPU, memory, systemd unit status, and container context.

## Plugins

| Plugin | Endpoint | Data Source |
|--------|----------|-------------|
| `procfs_plugin` | `x-medkit-procfs` | `/proc/[pid]/{stat,status,cmdline}` - CPU time, memory, threads, FDs |
| `systemd_plugin` | `x-medkit-systemd` | D-Bus sd-bus API - unit state, resource usage |
| `container_plugin` | `x-medkit-container` | Cgroup hierarchy - container runtime, resource limits |

## Key Components

- **ProcReader** - Parses procfs files for process metrics with configurable proc root
- **CgroupReader** - Reads the unified (v2), legacy (v1) and hybrid cgroup hierarchies for
  container/service context, under both cgroup namespace modes. Each limit is reported
  together with the outcome of reading it, so "unlimited" and "could not be read" stay
  distinguishable. A container whose ID the cgroup namespace hides is recognised from the
  markers its runtime leaves behind
- **SystemdUtils** - Queries systemd via D-Bus for service metadata
- **PidCache** - TTL-based cache mapping ROS 2 node names to Linux PIDs

## Configuration

Configure via `gateway_params.yaml` plugin parameters:

```yaml
plugins: ["linux_introspection"]
plugins.linux_introspection.path: "/path/to/libros2_medkit_linux_introspection.so"
plugins.linux_introspection.pid_cache_ttl_sec: 30
plugins.linux_introspection.proc_root: "/proc"
```

## Documentation

- [Linux Introspection Tutorial](https://selfpatch.github.io/ros2_medkit/tutorials/linux-introspection.html)
- [Plugin System Guide](https://selfpatch.github.io/ros2_medkit/tutorials/plugin-system.html)

## License

Apache License 2.0
