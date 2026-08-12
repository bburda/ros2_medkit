Troubleshooting & FAQ
=====================

Common issues and solutions when using ros2_medkit.

.. contents:: Table of Contents
   :local:
   :depth: 2

Installation Issues
-------------------

**rosdep fails to find packages**

.. code-block:: bash

   ERROR: the following packages/stacks could not have their rosdep keys resolved

Solution: Initialize and update rosdep:

.. code-block:: bash

   sudo rosdep init  # Only needed once
   rosdep update

**Cannot find ros2_medkit packages**

.. code-block:: bash

   Package 'ros2_medkit_gateway' not found

Solution: Source the workspace:

.. code-block:: bash

   source ~/ros2_medkit_ws/install/setup.bash

Gateway Issues
--------------

**Gateway shows no areas or components**

Possible causes:

1. **No other ROS 2 nodes running** - Start some nodes first
2. **Different ROS_DOMAIN_ID** - Ensure all nodes use the same domain
3. **Discovery not complete** - Wait a few seconds and refresh

The gateway logs a one-time **Discovery summary** a few seconds after startup
with the peer node / topic / entity counts. When it sees no application nodes it
also logs a warning that echoes the active ``ROS_DOMAIN_ID``,
``RMW_IMPLEMENTATION`` and ``ROS_LOCALHOST_ONLY`` - check the gateway's console
or logs first, as it names the most likely cause.

Check with:

.. code-block:: bash

   ros2 node list
   ros2 topic list

**"Connection refused" when accessing API**

Possible causes:

1. **Gateway not running** - Check process is active
2. **Wrong port** - Default is 8080 (HTTP) or 8443 (HTTPS)
3. **Localhost binding** - Gateway binds to 127.0.0.1 by default

For network access, set host to 0.0.0.0:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py server_host:=0.0.0.0

**Topic data returns empty or timeout**

Possible causes:

1. **Topic has no publishers** - Check with ``ros2 topic info``
2. **Publisher is slow** - Increase timeout in config
3. **Topic type not supported** - Check for custom message types

For idle topics, the gateway returns metadata instantly. If a topic has
publishers but no data, check if messages are actually being published:

.. code-block:: bash

   ros2 topic hz /your/topic

**CORS errors in browser**

.. code-block:: text

   Access to fetch blocked by CORS policy

Solution: Configure CORS in gateway params:

.. code-block:: yaml

   cors:
     allowed_origins: ["http://localhost:5173"]
     allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]

Authentication Issues
---------------------

**"Invalid token" error**

Possible causes:

1. **Token expired** - Get a new token or use refresh token
2. **Wrong secret** - Ensure JWT secret matches
3. **Malformed header** - Format must be ``Authorization: Bearer <token>``

**"Insufficient permissions" error**

Check your role has the required permission:

- ``viewer`` - Read only
- ``operator`` - Read + operations + data publish
- ``configurator`` - Operator + configurations
- ``admin`` - Full access

**Token refresh fails**

Refresh tokens are single-use. After refreshing, use the new refresh token
from the response.

TLS/HTTPS Issues
----------------

**"SSL certificate problem"**

For development with self-signed certificates:

.. code-block:: bash

   curl -k https://localhost:8443/api/v1/health

For production, use certificates from a trusted CA.

**"Key does not match certificate"**

Regenerate both certificate and key together:

.. code-block:: bash

   ./scripts/generate_dev_certs.sh ./certs

Docker Issues
-------------

**Container can't discover ROS 2 nodes**

Ensure all containers use the same:

1. Docker network
2. ``ROS_DOMAIN_ID``
3. DDS configuration

For development, try ``network_mode: host``.

**Web UI can't connect to gateway in container**

1. Gateway must listen on ``0.0.0.0``, not ``127.0.0.1``
2. CORS must allow the UI origin
3. Port must be exposed in docker-compose

Fault Manager Issues
--------------------

**Faults API returns 503 or empty**

The Faults API requires ``ros2_medkit_fault_manager`` node:

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node

**Faults not appearing for my component**

Components must use ``ros2_medkit_fault_reporter`` to report faults.
The REST API is read-only for fault status.

.. _graph-provider-pending-troubleshooting:

Graph Provider Issues
---------------------

**Every edge shows "pending" / no metrics**

The ``x-medkit-graph`` metrics model has exactly one source of live data: a
``/diagnostics`` producer whose ``DiagnosticStatus.name`` matches the topic's
fully-qualified name byte-for-byte. With no such producer running, every
edge's ``metrics_status`` stays ``"pending"`` forever - it never transitions
to ``"active"`` or ``"error"`` on its own.

Possible causes:

1. **No ``/diagnostics`` producer is running** - nothing publishes
   ``DiagnosticStatus`` messages for the topic at all.
2. **The producer's monitored-topic list omits this topic** - many producers
   (including the reference ``greenwave_monitor``) resolve their monitored
   topics once at startup and never re-resolve; a topic added or renamed
   afterward is silently skipped forever.
3. **Name mismatch** - ``DiagnosticStatus.name`` must equal the ROS topic name
   exactly (leading ``/``, no aliasing, no shortened form). Any other string
   never matches, however close.
4. **Producer started before its publishers existed** - a producer that
   resolves its monitored topics once at startup (see cause 2) must be
   started strictly after the topics it monitors already have live
   publishers in the ROS graph.

Check what is actually being published:

.. code-block:: bash

   ros2 topic echo /diagnostics --once

If nothing appears, no producer is running - start one (see
:doc:`/tutorials/graph-provider` for the reference ``greenwave_monitor``
invocation). If a ``DiagnosticStatus`` for the topic IS present but the edge
still reads ``pending``, compare its ``name`` field character-for-character
against the topic's fully-qualified ROS name.

See :doc:`/tutorials/graph-provider` for the full producer contract, and
:doc:`/config/graph-provider` for the freshness/degradation thresholds that
govern the ``active`` / ``error`` states once data does arrive.

Performance Issues
------------------

**Slow response for component data**

Possible causes:

1. **Many topics** - Response time scales with topic count
2. **Slow publishers** - the per-topic timeout applies (declared default 1.0 s,
   2.0 s with the shipped ``gateway_params.yaml``)
3. **Low parallelism** - Increase ``data_provider.max_parallel_samples``

Optimize with:

.. code-block:: yaml

   data_provider:
     max_parallel_samples: 32      # Increase from default 8
   topic_sample_timeout_sec: 0.5   # Decrease from default 2.0

**High CPU usage**

Discovery is graph-event driven (polled every 100 ms), so a stable graph
already incurs near-zero refresh overhead. On a graph with frequent node
churn that does not need to be reflected immediately, raise the safety-backstop
interval above the default ``30000`` to reduce the worst-case forced refresh
rate:

.. code-block:: yaml

   refresh_interval_ms: 60000  # Raise from default 30s if recovery from a
                                # missed graph event can tolerate longer delay.

FAQ
---

**Q: Does ros2_medkit modify my ROS 2 nodes?**

No. ros2_medkit is read-only by default. It discovers existing nodes and
exposes them via REST API. Write operations (publishing, service calls,
parameter changes) only happen when explicitly requested via the API.

**Q: Can I use ros2_medkit with ROS 1?**

No. ros2_medkit requires ROS 2 (Jazzy, Humble, or Lyrical). For ROS 1 systems, consider using
the ``ros1_bridge`` and running ros2_medkit on the ROS 2 side.

**Q: Is ros2_medkit production-ready?**

ros2_medkit is currently suitable for development and testing. For production:

- Enable TLS encryption
- Configure JWT authentication
- Test thoroughly with your specific use case
- Monitor performance under expected load

**Q: How does ros2_medkit compare to rosbridge?**

.. list-table::
   :widths: 30 35 35
   :header-rows: 1

   * - Feature
     - ros2_medkit
     - rosbridge
   * - Protocol
     - REST/HTTP
     - WebSocket/JSON
   * - Discovery
     - Automatic entity hierarchy
     - Topic/service access
   * - Authentication
     - JWT with RBAC
     - None built-in
   * - Fault management
     - Yes
     - No
   * - SOVD compatible
     - Yes
     - No

**Q: Can I extend ros2_medkit with custom endpoints?**

Yes. The gateway plugin framework allows you to add custom REST endpoints via
``GatewayPlugin::get_routes()``. Create a shared library (``.so``) that
implements the ``GatewayPlugin`` base class and configure it in ``gateway_params.yaml``.
See :doc:`/config/server` for plugin configuration details.

**Q: How do I report a bug or request a feature?**

- **Bug reports**: https://github.com/selfpatch/ros2_medkit/issues
- **Feature requests**: https://github.com/selfpatch/ros2_medkit/discussions
- **Discord**: https://discord.gg/6CXPMApAyq

Getting Help
------------

If your issue isn't covered here:

1. **Search existing issues**: https://github.com/selfpatch/ros2_medkit/issues
2. **Ask on Discord**: https://discord.gg/6CXPMApAyq
3. **Open a new issue** with:
   - ros2_medkit version
   - ROS 2 distribution
   - Steps to reproduce
   - Expected vs actual behavior
   - Relevant logs
