Server Configuration
====================

This reference describes all server-related configuration options for the
ros2_medkit gateway.

Quick Start
-----------

The gateway can be configured via:

1. **Command line**: ``--ros-args -p server.port:=9000``
2. **Launch files**: ``parameters=[{'server.port': 9000}]``
3. **YAML file**: See ``src/ros2_medkit_gateway/config/gateway_params.yaml``

Out-of-range values
-------------------

Where a parameter documents a valid range, that range is enforced when the value
is read at startup, and an out-of-range value is never accepted in silence. What
happens next differs by parameter, in one of three ways.

**Clamped to the nearest endpoint.** The value is moved to the closest end of the
range and the gateway logs both values. This applies to the thread and pool knobs
(``server.executor_threads``, ``server.http_thread_pool_size``,
``server.keep_alive_timeout_sec``, ``sse.max_clients``,
``service_call_timeout_sec``, ``logs.buffer_size``, ``entity_cache.capacity``,
``aggregation.max_discovered_peers``, ``fault_triggers.poll_interval_ms``) and to
the ``subscription_executor.*`` and ``data_provider.*`` knobs:

.. code-block:: text

   [WARN] [rest_server]: server.http_thread_pool_size 0 clamped to 1

**Rejected, and the documented default is used instead.** Clamping makes no sense
for these, so the value is refused and the default takes its place. The warning
names the value that was refused. This applies to ``server.port``,
``refresh_interval_ms``, ``discovery.refresh_debounce_ms``,
``topic_sample_timeout_sec``, ``fault_manager.service_timeout_sec``,
``bulk_data.max_upload_size``, ``sse.max_subscriptions``,
``sse.max_duration_sec`` and ``triggers.max_triggers``:

.. code-block:: text

   [ERROR] [ros2_medkit_gateway]: Invalid port 70000. Must be between 1024-65535. Using default 8080.

**Refused at startup.** ``parameter_service_timeout_sec`` and
``parameter_service_negative_cache_sec`` are declared with a floating-point range
descriptor, so rclcpp itself rejects an out-of-range value and the gateway does
not start:

.. code-block:: text

   [ros2_medkit_gateway] Fatal exception in main: parameter
   'parameter_service_timeout_sec' could not be set: Parameter
   {parameter_service_timeout_sec} doesn't comply with floating point range.

``.nan`` is the one value whose handling depends on the ROS 2 distribution. The
range check compares with ``<`` and ``>``, and every comparison against NaN is
false, so on Jazzy rclcpp reads NaN as in-range and the gateway starts; on
Lyrical it is refused like any other bad value and the gateway does not start.
Where it does get through, the gateway catches it when it reads the value, warns
and uses the default, so a NaN never reaches the code that would treat it as a
duration.

In every case the point is the same. The config file and the running process must
not disagree with nothing to show it, because the symptom of a mis-set value (a
slow or jittery gateway, a limit that never fires) surfaces far from the cause.
If the gateway behaves as though a parameter were different from what your YAML
says, its startup log names the parameter.

Note the logger name in the message. Most of these come from
``ros2_medkit_gateway``; the three that the REST server reads for itself -
``server.http_thread_pool_size``, ``server.keep_alive_timeout_sec`` and
``sse.max_duration_sec`` - come from ``rest_server``.

Network Settings
----------------

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.host``
     - string
     - ``"127.0.0.1"``
     - Host to bind the REST server. Use ``"0.0.0.0"`` for Docker or network access.
   * - ``server.port``
     - int
     - ``8080``
     - Port for REST API. Valid range: 1024-65535.

Example:

.. code-block:: bash

   # Expose on all interfaces (Docker/network)
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p server.host:=0.0.0.0 \
       -p server.port:=8080

TLS/HTTPS Configuration
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.tls.enabled``
     - bool
     - ``false``
     - Enable HTTPS using OpenSSL.
   * - ``server.tls.cert_file``
     - string
     - ``""``
     - Path to PEM-encoded certificate file.
   * - ``server.tls.key_file``
     - string
     - ``""``
     - Path to PEM-encoded private key file. Restrict permissions (chmod 600).
   * - ``server.tls.ca_file``
     - string
     - ``""``
     - Path to CA certificate file (reserved for mutual TLS).
   * - ``server.tls.min_version``
     - string
     - ``"1.2"``
     - Minimum TLS version: ``"1.2"`` (compatible) or ``"1.3"`` (secure).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         tls:
           enabled: true
           cert_file: "/etc/ros2_medkit/certs/cert.pem"
           key_file: "/etc/ros2_medkit/certs/key.pem"
           min_version: "1.2"

See :doc:`/tutorials/https` for a complete HTTPS setup tutorial.

CORS Configuration
------------------

Cross-Origin Resource Sharing (CORS) settings for browser-based clients.

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``cors.allowed_origins``
     - list
     - ``[]``
     - List of allowed origins. Use ``["*"]`` for all (not recommended). CORS is
       off while this is empty, so the shipped ``gateway_params.yaml`` value of
       ``[""]`` also leaves it off: empty strings are stripped before the check.
   * - ``cors.allowed_methods``
     - list
     - ``["GET", "PUT", "POST", "DELETE", "OPTIONS"]``
     - Allowed HTTP methods.
   * - ``cors.allowed_headers``
     - list
     - ``["Content-Type", "Accept"]``
     - Allowed headers in requests.
   * - ``cors.allow_credentials``
     - bool
     - ``false``
     - Allow credentials (cookies, auth headers).
   * - ``cors.max_age_seconds``
     - int
     - ``86400``
     - Preflight response cache duration (24 hours).

Example for development with ros2_medkit_web_ui:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       cors:
         allowed_origins: ["http://localhost:5173"]
         allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]
         allowed_headers: ["Content-Type", "Accept", "Authorization"]
         allow_credentials: true

Data Access Settings
--------------------

.. list-table::
   :header-rows: 1
   :widths: 35 10 10 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``data_provider.max_parallel_samples``
     - int
     - ``8``
     - Max concurrent topic samples. Higher values use more resources. Excess
       topics are processed in follow-up chunks. Range: 1-256.
   * - ``topic_sample_timeout_sec``
     - float
     - ``1.0``
     - Timeout for sampling topics with active publishers. Range: 0.1-30.0.
       The declared default is ``1.0``, which is what a bare ``ros2 run`` uses.
       ``gateway_params.yaml`` sets ``2.0``, so a launch that loads it gets that
       instead.
   * - ``parameter_service_timeout_sec``
     - float
     - ``2.0``
     - Timeout for ROS 2 parameter service calls (configurations endpoint).
       Nodes without parameter service (e.g., micro-ROS bridges) block for this
       duration before returning SERVICE_UNAVAILABLE. Range: 0.1-10.0.
   * - ``parameter_service_negative_cache_sec``
     - float
     - ``60.0``
     - After a node's parameter service fails to respond, subsequent requests
       return immediately with SERVICE_UNAVAILABLE for this duration.
       Set to 0 to disable. Range: 0-3600.
   * - ``service_call_timeout_sec``
     - int
     - ``10``
     - How long the gateway waits for a **response** to an operation RPC: a
       ROS 2 service call (``POST .../executions`` on a service-backed
       operation) and each of the three action RPCs - send goal, get result
       and cancel. Values outside the range are clamped with a warning at
       startup. Range: 1-3600.

       It is not the whole wall-clock cost, because each RPC first waits for
       its service to be discovered and the three do that differently:

       .. list-table::
          :header-rows: 1
          :widths: 30 35 35

          * - RPC
            - Discovery wait
            - Worst case in total
          * - Service call
            - none (bounded by the response wait)
            - ``service_call_timeout_sec``
          * - Action send goal
            - up to ``service_call_timeout_sec``
            - ``2 x service_call_timeout_sec``
          * - Action get result
            - up to 2 s, fixed
            - ``service_call_timeout_sec + 2 s``
          * - Action cancel
            - up to 2 s, fixed
            - ``service_call_timeout_sec + 2 s``

       The last row is the *cancel budget* referenced by
       ``DELETE .../executions/{id}`` and ``PUT .../executions/{id}`` (see
       :doc:`../api/rest`): a cancel that gets no answer within the response
       wait is reported as ``504 not-responding`` unless the action's status
       stream already shows the goal cancelling. The 2 s discovery waits are
       fixed and do not shrink with this parameter, so lowering it to the
       minimum of 1 s does not make an undiscovered cancel or get-result
       answer in under 2 s - size client timeouts off the "worst case in
       total" column, not off the parameter alone.

.. note::

   The defaults listed above are the ones the code declares. Where the shipped
   ``src/ros2_medkit_gateway/config/gateway_params.yaml`` sets a different value
   the row says so - ``topic_sample_timeout_sec`` is the one that differs today.
   If
   ``topic_sample_timeout_sec`` is out of range, ``DataAccessManager`` and the
   topic transport both fall back to ``1.0``.

Subscription Pool and Executor
------------------------------

The topic data provider keeps a pool of live subscriptions, and all subscription
create and destroy calls are funnelled through one serial worker. These knobs
bound both. Every value is clamped to its range on read, with a warning (see
`Out-of-range values`_).

.. list-table::
   :header-rows: 1
   :widths: 38 8 10 44

   * - Parameter
     - Type
     - Default
     - Description
   * - ``data_provider.max_pool_size``
     - int
     - ``256``
     - Live per-topic subscriptions the pool keeps. At the cap, sampling a new
       topic evicts the least recently used entry. Range: 1-4096.
   * - ``data_provider.cold_wait_cap``
     - int
     - ``4``
     - Concurrent HTTP workers allowed to block waiting for the first message on
       a freshly subscribed topic. Further callers get 503 and back off instead
       of saturating the request pool. Range: 0-1024.
   * - ``data_provider.idle_safety_net_sec``
     - int
     - ``900``
     - A pool entry unsampled for this long is evicted, releasing its
       subscription. ``0`` disables idle eviction. Range: 0-86400.
   * - ``data_provider.idle_sweep_tick_sec``
     - int
     - ``60``
     - How often the idle sweep runs. ``0`` disables idle eviction.
       Range: 0-3600.
   * - ``subscription_executor.max_queue_depth``
     - int
     - ``256``
     - Tasks queued on the serial worker before new posts are rejected with
       backpressure. Bounds peak memory when handler threads create
       subscriptions faster than the worker services them. Range: 16-4096.
   * - ``subscription_executor.watchdog_threshold_ms``
     - int
     - ``5000``
     - A worker task running longer than this marks the executor degraded
       (``x-medkit-subscription-executor.degraded`` on ``/health``). The flag
       clears when the task completes. Range: 100-60000.
   * - ``subscription_executor.watchdog_tick_ms``
     - int
     - ``1000``
     - How often the watchdog checks. Finer tick detects degradation sooner at
       the cost of more wakeups. Range: 10-10000.
   * - ``subscription_executor.graph_poll_tick_ms``
     - int
     - ``100``
     - How often graph events are polled (publisher appear or disappear, type
       changes). Range: 10-10000.

Fault Manager Integration
-------------------------

Configure how the gateway connects to the fault manager services and event topic.

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``fault_manager.namespace``
     - string
     - ``""``
     - Optional namespace prefix for fault manager service and event topic resolution.
       Examples: ``""`` -> ``/fault_manager/list_faults``, ``"robot1"`` -> ``/robot1/fault_manager/list_faults``.
   * - ``fault_manager.service_timeout_sec``
     - float
     - ``5.0``
     - Timeout for fault manager service calls such as ``list_faults`` and
       ``get_snapshots``. Must be finite and greater than ``0``; anything else
       is refused with a warning and ``5.0`` is used. Zero or negative would
       make every fault call report "service not available" instead of waiting,
       and a non-finite value would make it wait forever, holding an HTTP worker
       for the life of the process.
   * - ``entity_freeze_frame.enabled``
     - bool
     - ``true``
     - Zero-config freeze-frames for plugin-backed entities: when a fault from a
       plugin-owned entity confirms, snapshot the entity's data values as served
       by its plugin's DataProvider and merge them into the fault detail's
       ``environment_data.snapshots``. Faults already confirmed at gateway start
       are caught up at startup; those frames read the values at start time (not
       confirm time) and are marked ``x-medkit.capture_origin: startup``. For an
       entity that reports its link down these are the plugin's last known
       values, marked ``connected: false`` in the snapshot's ``x-medkit`` block.
       Explicit snapshot config in the fault manager always wins when present.
       Only active when plugins are loaded.

When ``fault_manager.namespace`` is set, the gateway also subscribes to the matching
fault event topic (for example ``/robot1/fault_manager/events`` instead of the default
``/fault_manager/events``).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       fault_manager:
         namespace: "robot1"
         service_timeout_sec: 5.0

Logging Configuration
---------------------

Configure the in-memory log buffer that collects ``/rosout`` messages.

.. list-table::
   :header-rows: 1
   :widths: 25 10 15 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``logs.buffer_size``
     - int
     - ``200``
     - Maximum log entries retained per node. Valid range: 1-100000
       (values outside this range are clamped with a warning).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       logs:
         buffer_size: 500

A ``LogProvider`` plugin can replace the default ``/rosout`` backend.
See :doc:`/tutorials/plugin-system` for details.

Performance Tuning
------------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``refresh_interval_ms``
     - int
     - ``30000``
     - Safety-backstop refresh interval (ms). Primary discovery is graph-event
       driven (polled every 100 ms); this timer forces a full refresh only if
       a graph event is missed. Range: 100-60000 (0.1s-60s).
   * - ``discovery.refresh_debounce_ms``
     - int
     - ``1000``
     - Debounce for graph-event-driven refreshes (ms). Under graph churn the
       rclcpp graph event fires many times per second; each refresh runs the
       full discovery pipeline. Coalesces graph events so a refresh runs at most
       once per this interval, bounding per-event allocation. A pending change is
       serviced within this interval plus one 100 ms poll. Range: 0-60000
       (0 disables debouncing). A value outside the range is rejected with a
       warning and the default (1000) is used.
   * - ``entity_cache.capacity``
     - int
     - ``256``
     - Fixed capacity of the thread-safe entity cache object pool, reserved once
       at startup. Steady-state refresh cycles perform zero structural allocations
       in the cache layer as long as the live entity count stays within this
       value. Valid range: 16-1000000 (values outside this range are clamped with
       a warning). Raise it for graphs larger than the default; exceeding the
       reserved capacity is harmless but triggers a one-shot WARN log. Note that a
       refresh that fully turns the entity set over allocates the new slots before
       freeing the absent old ones, so the pool transiently holds up to ~2x the
       steady-state live count - size capacity to about twice the expected peak to
       keep full-turnover ticks allocation-free.

Lower values shorten the worst-case recovery window if a graph event is missed
but increase idle CPU. The default rarely fires on a stable graph because the
graph-event poll handles node up/down events directly.

How long a departed node keeps being listed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once a node has actually left the ROS graph, ``GET /apps`` stops listing it within
roughly one refresh: at most ``discovery.refresh_debounce_ms`` plus the 100 ms graph
poll, or ``refresh_interval_ms``, whichever comes first. Nothing is retained behind
that - every refresh rebuilds the entity set from a live read of the graph - so this
is the whole of the gateway's share, and it is the only part of a departure the
gateway can be held to.

Before that point nothing here is promised, and the difference is not small:

* A node that shuts down cleanly unregisters its DDS participant on the way out, and
  the graph drops it almost at once. Measured at 225 ms from the signal to the node
  no longer being listed, polling ``GET /apps`` every 200 ms.
* A node that dies without unregistering - killed outright, or stopped before its
  shutdown could finish - costs the DDS participant lease instead, because remote
  participants have nothing to go on but its silence. Measured at 20.07 s, 19.76 s
  and 19.99 s over three runs, with stock ``rmw_fastrtps_cpp`` and no participant
  profile override.
* How long a node takes to shut down is the node's own business. The gateway cannot
  tell a node that is slow to exit from one that is simply still running, and neither
  can this bound.

So a caller waiting for a specific node to disappear should watch for its absence
rather than assume a deadline. Anything that measures a departure should establish
that the process has actually exited first, and only then hold the gateway to the
figure above.

Thread Pools
------------

The gateway runs two thread pools. By default both are bounded to a small fixed
size instead of scaling with the host CPU count, so the gateway's thread
footprint is the same on a 4-core SBC and a 64-core server. A third knob,
``keep_alive_timeout_sec``, bounds how long the HTTP pool keeps a worker parked
on an idle keep-alive connection. Every value is clamped to a working minimum on
read, so a mis-set parameter can never break request serving - and the clamp is
reported, so it cannot quietly change what the gateway runs (see
`Out-of-range values`_).

.. list-table::
   :header-rows: 1
   :widths: 32 8 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.http_thread_pool_size``
     - int
     - ``6``
     - Worker threads in the HTTP request pool (cpp-httplib). Replaces the
       library default of ``max(8, cores - 1)``. Kept at or above
       ``sse.max_clients + data_provider.cold_wait_cap`` so SSE streams and cold
       ``/data`` waits cannot starve every worker (see note below); the gateway
       warns at startup if it is set below that sum. Clamped to ``[1, 1024]``.
   * - ``server.keep_alive_timeout_sec``
     - int
     - ``2``
     - How long (seconds) a request-pool worker stays parked on an idle
       keep-alive connection before freeing it. Replaces the cpp-httplib default
       of ``5``. A shorter value recovers workers from short-lived client
       connections faster (important with a small pool); a longer value favours
       connection reuse. Clamped to ``[1, 3600]``.
   * - ``server.executor_threads``
     - int
     - ``2``
     - Threads in the main rclcpp ``MultiThreadedExecutor``. Replaces rclcpp's
       default (host cores, minimum 2). With two or more threads,
       blocking-RPC responses (operation executions) are dispatched even
       while other gateway callbacks run (see note below). Clamped to
       ``[1, 256]``.

**HTTP pool, keep-alive, and SSE.** Several things hold an HTTP pool worker:
each active SSE stream (fault dashboard, cyclic subscriptions, trigger events -
see `SSE (Server-Sent Events)`_) holds one for its entire lifetime; the data
cold-wait path parks up to ``data_provider.cold_wait_cap`` workers for up to
``topic_sample_timeout_sec`` each; a bulk-data download holds one
for the whole transfer (uncounted by any cap); and on top of that each *recently
used* client connection keeps a worker parked for up to ``keep_alive_timeout_sec``
after its last request. The shipped pool default (``6``) covers the documented
worst case ``sse.max_clients (2) + data_provider.cold_wait_cap (4)``; the gateway
emits a startup warning if ``http_thread_pool_size`` is set below
``sse.max_clients + data_provider.cold_wait_cap``. The short ``keep_alive_timeout_sec``
default (``2`` s) stops a poller that opens several short-lived connections per
cycle (for example hitting ``/apps``, ``/areas`` and ``/functions`` every
iteration) from pinning the pool until the keep-alive timers expire. If you raise
``sse.max_clients``, raise ``cold_wait_cap``, or serve concurrent bulk-data
downloads, raise ``http_thread_pool_size`` to match (and keep
``keep_alive_timeout_sec`` short unless your clients benefit from long-lived
connection reuse).

**Executor threads.** The main executor delivers the gateway node's own
callbacks (timers, graph events, log and fault subscriptions) and the
service-response callbacks that complete operation/action RPC futures. The RPC
response callbacks - the generic service clients behind ``/operations``
executions and the per-action send_goal / get_result / cancel_goal client trio -
run in a shared *reentrant* callback group, so with two or more executor threads
a response is dispatched even while another gateway callback (for example a
discovery refresh pass) is running: the default of ``2`` buys real RPC-response
parallelism. Timers and the SSE-fault / trigger-fault / ``/rosout``
subscriptions stay in the node's default, *mutually-exclusive* group by design -
discovery refresh passes are serialized on purpose, and those subscriptions rely
on in-order delivery. Per-action ``/_action/status`` subscriptions use a
dedicated mutually-exclusive group of their own: ordered among themselves,
decoupled from the default group. A single executor thread remains safe (a
reentrant group does not *require* a second thread), and the blocking wait for
an RPC runs on the cpp-httplib pool thread (a separate server thread), never on
an executor thread, so it cannot deadlock the executor; the fault transport also
uses its own private executor. Raise this beyond ``2`` if many concurrent RPC
responses must be dispatched in parallel or the node's own callback load grows
(for example very frequent graph churn).

Example (more SSE clients needs a larger pool and matching ``sse.max_clients``):

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         http_thread_pool_size: 16   # workers for heavy SSE + request load
         keep_alive_timeout_sec: 5   # longer reuse for steady browser clients
         executor_threads: 4
       sse:
         max_clients: 10             # raised together with the HTTP pool

Bulk Data Storage
-----------------

Configure file-based bulk data storage for uploads (calibration files, firmware, etc.).
The ``rosbags`` category is always available via the Fault Manager and does not need
configuration.

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``bulk_data.storage_dir``
     - string
     - ``/tmp/ros2_medkit_bulk_data``
     - Base directory for uploaded files. Each entity gets a subdirectory.
   * - ``bulk_data.max_upload_size``
     - int
     - ``104857600``
     - Maximum upload file size in bytes (default: 100 MB). Set to ``0`` for unlimited. A negative value is refused with a warning and the default is used: it would otherwise wrap to a huge unsigned size, which is the same "no limit" a typo should never buy silently.
   * - ``bulk_data.categories``
     - string[]
     - ``[]``
     - Allowed bulk data categories for upload/download (e.g., ``calibration``, ``firmware``).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       bulk_data:
         storage_dir: "/var/ros2_medkit/bulk_data"
         max_upload_size: 52428800   # 50 MB
         categories: ["calibration", "firmware", "logs"]

.. note::

   The ``rosbags`` category is always available through the Fault Manager and cannot be
   used for uploads or deletes. It is automatically included in the category list.

.. note::

   The gateway uses native rclcpp APIs for all ROS 2 interactions - no ROS 2 CLI
   dependencies. Topic discovery, sampling, publishing, service calls, and
   action operations are implemented in pure C++ using ros2_medkit_serialization.

.. note::

   Parameters whose value is a credential - ``auth.jwt_secret``,
   ``auth.clients`` and ``aggregation.peer_auth_header`` - are reported by the
   configurations API as ``***`` rather than their value. The parameter is
   still listed, with its real type, so it stays discoverable; only the secret
   is withheld. Set them through the node's configuration, and read them back
   from there rather than over the API.

SSE (Server-Sent Events)
------------------------

Configure limits for SSE-based streaming (fault events and cyclic subscriptions).

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``sse.max_clients``
     - int
     - ``2``
     - Maximum number of concurrent SSE connections (fault stream, cyclic subscription streams, and trigger event streams combined). Each connection pins one ``server.http_thread_pool_size`` worker for its lifetime, so this defaults at or below that pool; raise both together for more concurrent streams. Range: 1-1024, clamped with a warning.
   * - ``sse.max_subscriptions``
     - int
     - ``100``
     - Maximum number of active cyclic subscriptions across all entities. Returns HTTP 503 when this limit is reached. Must be at least ``1``; anything lower is refused with a warning and ``100`` is used, because the cap is compared as an unsigned count and a negative would remove it.
   * - ``sse.max_duration_sec``
     - int
     - ``3600``
     - Maximum allowed subscription duration in seconds. Requests exceeding this are rejected with HTTP 400. Range: 1 to 2147483647 (the value becomes an ``int``); outside it the value is refused with a warning and ``3600`` is used.
   * - ``sse.keepalive_interval_sec``
     - int
     - ``30``
     - How often an idle stream writes a keepalive comment. This is also how long a client that has gone away stays counted against ``sse.max_clients``, because a closed connection is only noticed on the next write: a gateway whose clients come and go faster than this interval can refuse a stream for a slot nobody is using. Lower it where streams are short-lived, at the cost of more traffic on idle ones. Range: 1-3600, clamped with a warning.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       sse:
         max_clients: 2
         max_subscriptions: 100
         max_duration_sec: 3600
         keepalive_interval_sec: 30

Triggers
--------

Configure condition-based push notifications for resource changes.

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``triggers.enabled``
     - bool
     - ``true``
     - Enable/disable the trigger subsystem. When disabled, trigger endpoints
       return HTTP 501.
   * - ``triggers.max_triggers``
     - int
     - ``1000``
     - Maximum number of concurrent triggers across all entities. Returns
       HTTP 503 when this limit is reached. A value below ``1`` or past what an
       ``int`` holds is refused with a warning and ``1000`` is used, because the
       cap is compared as an unsigned count and a negative would remove it.
   * - ``triggers.on_restart_behavior``
     - string
     - ``"reset"``
     - Behavior on gateway restart for persistent triggers. ``"reset"`` clears
       all triggers on restart. ``"restore"`` reloads persistent triggers from
       the storage database.
   * - ``triggers.storage.path``
     - string
     - ``""``
     - Path to SQLite database for persistent trigger storage. When empty
       (default), triggers are stored in-memory only and lost on restart.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       triggers:
         enabled: true
         max_triggers: 1000
         on_restart_behavior: "restore"
         storage:
           path: "/var/lib/ros2_medkit/triggers.db"

Fault-trigger rules
^^^^^^^^^^^^^^^^^^^

A separate engine evaluates threshold rules over plugin-provided data values and
raises faults from them, without a rule file. It only starts when at least one
plugin is loaded; with no plugins these parameters do nothing.

.. list-table::
   :header-rows: 1
   :widths: 32 8 14 46

   * - Parameter
     - Type
     - Default
     - Description
   * - ``fault_triggers.enabled``
     - bool
     - ``true``
     - Master switch for the rule engine.
   * - ``fault_triggers.poll_interval_ms``
     - int
     - ``1000``
     - How often the rules are evaluated. Range: 50 to 2147483647 ms. The floor
       stops the loop busy-spinning; the ceiling is what an ``int`` holds, since
       the value becomes a timer period. Either end is clamped with a warning.
       Read only when the engine actually starts, so with no plugin loaded a
       mis-set value is neither applied nor reported - it does nothing either
       way.
   * - ``fault_triggers.storage.path``
     - string
     - ``""``
     - SQLite file the rules are persisted in. Empty keeps them in memory, so
       they are lost on restart.

Rate Limiting
-------------

Token-bucket-based rate limiting for API requests. Disabled by default.

.. list-table::
   :header-rows: 1
   :widths: 35 10 15 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``rate_limiting.enabled``
     - bool
     - ``false``
     - Enable rate limiting.
   * - ``rate_limiting.global_requests_per_minute``
     - int
     - ``600``
     - Maximum RPM across all clients combined.
   * - ``rate_limiting.client_requests_per_minute``
     - int
     - ``60``
     - Maximum RPM per client IP.
   * - ``rate_limiting.endpoint_limits``
     - string[]
     - ``[]``
     - Per-endpoint overrides as ``"pattern:rpm"`` strings.
       Pattern uses ``*`` as single-segment wildcard
       (e.g., ``"/api/v1/*/operations/*:10"``).
   * - ``rate_limiting.client_cleanup_interval_seconds``
     - int
     - ``300``
     - How often to scan and remove idle client tracking entries (seconds).
   * - ``rate_limiting.client_max_idle_seconds``
     - int
     - ``600``
     - Remove client entries idle longer than this (seconds).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       rate_limiting:
         enabled: true
         global_requests_per_minute: 600
         client_requests_per_minute: 60
         endpoint_limits: ["/api/v1/*/operations/*:10"]

See :doc:`/api/rest` for rate limiting response headers and 429 behavior.

Authentication
--------------

JWT-based authentication with Role-Based Access Control (RBAC). Disabled by
default for local development.

.. list-table::
   :header-rows: 1
   :widths: 35 10 25 30

   * - Parameter
     - Type
     - Default
     - Description
   * - ``auth.enabled``
     - bool
     - ``false``
     - Enable/disable JWT authentication.
   * - ``auth.jwt_secret``
     - string
     - ``""``
     - JWT signing secret. For HS256: the shared secret string. For RS256: path to the private key file (PEM format).
   * - ``auth.jwt_public_key``
     - string
     - ``""``
     - Path to public key file for RS256. Required for RS256, optional for HS256.
   * - ``auth.jwt_algorithm``
     - string
     - ``"HS256"``
     - JWT signing algorithm: ``"HS256"`` (symmetric) or ``"RS256"`` (asymmetric).
   * - ``auth.token_expiry_seconds``
     - int
     - ``3600``
     - Access token validity period in seconds (1 hour).
   * - ``auth.refresh_token_expiry_seconds``
     - int
     - ``86400``
     - Refresh token validity period in seconds (24 hours). Must be >= ``token_expiry_seconds``.
   * - ``auth.require_auth_for``
     - string
     - ``"write"``
     - When to require authentication: ``"none"`` (auth endpoints still available), ``"write"`` (POST/PUT/DELETE only), or ``"all"`` (every request).
   * - ``auth.issuer``
     - string
     - ``"ros2_medkit_gateway"``
     - JWT issuer claim (``iss`` field in tokens).
   * - ``auth.clients``
     - string[]
     - ``[]``
     - Pre-configured clients as ``"client_id:client_secret:role"`` strings.

.. note::

   RBAC roles and their permissions:

   - **viewer** - Read-only access (GET on areas, components, data, faults)
   - **operator** - Viewer + trigger operations, acknowledge faults, publish data
   - **configurator** - Operator + modify/reset configurations
   - **admin** - Full access including auth management

.. danger::

   The example below uses **placeholder secrets for illustration only**.
   In production, generate secrets with ``openssl rand -base64 32`` and
   never commit them to configuration files. Use environment variable
   substitution or a secrets manager.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       auth:
         enabled: true
         jwt_secret: "CHANGE-ME-use-openssl-rand-base64-32"
         jwt_algorithm: "HS256"
         require_auth_for: "write"
         token_expiry_seconds: 3600
         clients: ["admin:REPLACE_WITH_STRONG_SECRET:admin", "viewer:REPLACE_WITH_STRONG_SECRET:viewer"]

See :doc:`/tutorials/authentication` for a complete setup tutorial.

Plugin Framework
----------------

Extend the gateway with custom plugins loaded from shared libraries (``.so``).
Plugins can implement provider interfaces (e.g., ``UpdateProvider``, ``IntrospectionProvider``)
that are automatically detected and wired into the gateway's subsystem managers.

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``plugins``
     - string[]
     - ``[]``
     - List of plugin names to load. Each plugin requires a corresponding ``plugins.<name>.path`` parameter.
   * - ``plugins.<name>.path``
     - string
     - (required)
     - Absolute path to the plugin ``.so`` file. Must exist and have ``.so`` extension.

Plugin loading lifecycle:

1. Shared library is loaded via ``dlopen`` with ``RTLD_NOW | RTLD_LOCAL``
2. API version is checked (must match gateway headers)
3. ``create_plugin()`` factory is called to instantiate the plugin
4. Provider interfaces are queried via ``extern "C"`` functions
5. ``configure()`` is called with per-plugin config
6. ``set_context()`` passes the gateway context to the plugin
7. ``get_routes()`` returns custom REST endpoint definitions as ``vector<PluginRoute>``

Error isolation: if a plugin throws during any lifecycle call, it is disabled
without crashing the gateway. Other plugins continue to operate normally.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"

Software Updates
----------------

Configure the software updates system. Updates are disabled by default.
When enabled, a plugin implementing ``UpdateProvider`` is required to provide
the backend functionality (see `Plugin Framework`_ above).

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``updates.enabled``
     - bool
     - ``false``
     - Enable/disable software updates endpoints. When disabled, ``/updates`` routes are not registered.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_update_plugin"]
       plugins.my_update_plugin.path: "/opt/ros2_medkit/lib/libmy_update_plugin.so"
       updates:
         enabled: true

Complete Example
----------------

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         host: "0.0.0.0"
         port: 8080
         tls:
           enabled: false

       refresh_interval_ms: 5000
       topic_sample_timeout_sec: 3.0

       data_provider:
         max_parallel_samples: 30

       cors:
         allowed_origins: ["http://localhost:5173", "https://dashboard.example.com"]
         allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]
         allowed_headers: ["Content-Type", "Accept", "Authorization"]
         allow_credentials: true
         max_age_seconds: 86400

       bulk_data:
         storage_dir: "/var/ros2_medkit/bulk_data"
         max_upload_size: 104857600
         categories: ["calibration", "firmware"]

       sse:
         max_clients: 2
         max_subscriptions: 100
         max_duration_sec: 3600

       triggers:
         enabled: true
         max_triggers: 1000
         on_restart_behavior: "reset"

       logs:
         buffer_size: 200

       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"

       updates:
         enabled: true

       auth:
         enabled: true
         jwt_secret: "CHANGE-ME-use-openssl-rand-base64-32"
         jwt_algorithm: "HS256"
         require_auth_for: "write"
         clients: ["admin:REPLACE_WITH_STRONG_SECRET:admin"]

       rate_limiting:
         enabled: false

       scripts:
         scripts_dir: "/var/ros2_medkit/scripts"
         allow_uploads: true
         max_concurrent_executions: 5

API Documentation
-----------------

Configure the self-describing OpenAPI capability description endpoint.

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``docs.enabled``
     - bool
     - ``true``
     - Enable/disable ``/docs`` capability description endpoints. When disabled,
       all ``/docs`` endpoints return HTTP 501.

**Build option:** ``ENABLE_SWAGGER_UI``

Set ``-DENABLE_SWAGGER_UI=ON`` during CMake configure to embed Swagger UI assets
in the gateway binary. This provides an interactive API browser at
``/api/v1/swagger-ui``. Requires network access to download assets from unpkg.com
during configure. Disabled by default.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       docs:
         enabled: true

Scripts
-------

Diagnostic scripts: upload, manage, and execute scripts on entities. Set
``scripts.scripts_dir`` to a directory path to enable the feature. When left
empty, all script endpoints return HTTP 501.

.. list-table::
   :header-rows: 1
   :widths: 35 10 15 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``scripts.scripts_dir``
     - string
     - ``""``
     - Directory for storing uploaded scripts. Empty string disables the feature.
   * - ``scripts.allow_uploads``
     - bool
     - ``true``
     - Allow uploading scripts via HTTP. Set to ``false`` for hardened deployments that only use manifest-defined scripts.
   * - ``scripts.max_file_size_mb``
     - int
     - ``10``
     - Maximum uploaded script file size in megabytes.
   * - ``scripts.max_concurrent_executions``
     - int
     - ``5``
     - Maximum number of scripts executing concurrently.
   * - ``scripts.default_timeout_sec``
     - int
     - ``300``
     - Default timeout per execution in seconds (5 minutes).
   * - ``scripts.max_execution_history``
     - int
     - ``100``
     - Maximum number of completed executions to keep in memory. Oldest completed entries are evicted when this limit is exceeded.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       scripts:
         scripts_dir: "/var/ros2_medkit/scripts"
         allow_uploads: true
         max_file_size_mb: 10
         max_concurrent_executions: 5
         default_timeout_sec: 300

Locking
-------

SOVD resource locking (ISO 17978-3, Section 7.17). Clients acquire locks on
components and apps to prevent concurrent modification.

.. list-table::
   :header-rows: 1
   :widths: 40 15 15 30

   * - Parameter
     - Type
     - Default
     - Description
   * - ``locking.enabled``
     - bool
     - ``true``
     - Enable the LockManager and lock endpoints
   * - ``locking.default_max_expiration``
     - int
     - ``3600``
     - Maximum lock TTL in seconds
   * - ``locking.cleanup_interval``
     - int
     - ``30``
     - Seconds between expired lock cleanup sweeps
   * - ``locking.defaults.components.lock_required_scopes``
     - [string]
     - ``[""]``
     - Collections requiring a lock on components (empty = no requirement)
   * - ``locking.defaults.components.breakable``
     - bool
     - ``true``
     - Whether component locks can be broken
   * - ``locking.defaults.apps.lock_required_scopes``
     - [string]
     - ``[""]``
     - Collections requiring a lock on apps (empty = no requirement)
   * - ``locking.defaults.apps.breakable``
     - bool
     - ``true``
     - Whether app locks can be broken

Per-entity overrides are configured in the manifest ``lock:`` section.
See :doc:`manifest-schema` and :doc:`/api/locking`.

See Also
--------

- :doc:`/tutorials/authentication` - JWT authentication setup
- :doc:`/tutorials/https` - HTTPS configuration
- :doc:`discovery-options` - Discovery and entity mapping options
