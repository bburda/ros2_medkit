^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_gateway
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-08-27)
------------------
* Rosbag bulk-data is addressed by recording id instead of fault code, so a fault holding several recordings can expose each one. ``GET /{entity}/bulk-data/rosbags`` now emits one descriptor per recording rather than one per fault - a burst that shares a bag used to appear as several entries each reporting the full bag size - and the covered faults move into ``x-medkit.fault_codes`` (was the scalar ``x-medkit.fault_code``). Old URLs keep working: an id that is not a recording is resolved as a fault code and serves that fault's newest recording, which is what it returned before. Authorization is unchanged in effect - a download is allowed when any fault the recording covers is in the entity's source scope, which is exactly the set that could reach it previously (`#620 <https://github.com/selfpatch/ros2_medkit/issues/620>`_)
* Manual asset inventory: a manifest ``assets:`` list and a new ``discovery.inventory.csv_path`` parameter declare assets that no protocol layer can describe (or fully describe). Both paths recognize the canonical names ``id, manufacturer, model, serial, hardware_rev, firmware, endpoint, role, area`` plus the shared aliases (``serial_number``, ``hardware_revision`` / ``hw_rev``, ``firmware_version`` / ``fw``) and keep any other column / key as an extra; RFC-4180-style quoting is honored. Each asset becomes a Component with ``source = "inventory"`` and a structured asset identity carrying per-field provenance, appended to the base manifest on every load / reload and merged into the tree by id alongside protocol-discovered structure; ``area`` places the asset under an Area, without it the asset appears only in the flat component list. CSV rows never fail the load: rows without an ``id`` are skipped with a warning, for duplicate ids the first row wins, a row whose id is already a manifest component keeps the manifest definition (the row's identity is folded in as gap-fill), and an unknown ``area`` is dropped with a warning. The CSV is size-capped at 1 MiB before being read; a missing file is skipped with a warning (mirrors ``fragments_dir``), while an unreadable or malformed one fails the load / reload. Requires a manifest-backed discovery mode (``manifest_only`` / ``hybrid`` with ``discovery.manifest_path`` set); empty = disabled (default) (`#490 <https://github.com/selfpatch/ros2_medkit/issues/490>`_)
* **Breaking:** the lifecycle status ``operationId`` values were pluralized, so a generated client gets renamed methods for those operations (`#497 <https://github.com/selfpatch/ros2_medkit/pull/497>`_)
* Aggregation now separates the time budget for reading metadata from the budget for real work. One ``aggregation.timeout_ms`` was applied as both the connect and the read timeout for every call, so a synchronous service call on a peer got two seconds end to end while the peer's own budget for the same call was ten; a large resource fanned out to a peer could not finish inside it either. The write timeout, which was never set and stayed at the cpp-httplib default, now follows the configured budget, and the timeout values are validated and reported rather than silently clamped (`#638 <https://github.com/selfpatch/ros2_medkit/pull/638>`_, `#528 <https://github.com/selfpatch/ros2_medkit/issues/528>`_)
* A request that ran out of time is reported as a timeout. A peer that did not answer in the budget returns ``504`` with ``ERR_NOT_RESPONDING`` instead of ``502`` claiming the peer is unavailable - which it did while that peer was answering the same request - a fanned-out collection carries the per-peer failure reason instead of only a boolean, and an operation that exceeds its own service-call budget says so rather than returning a generic failure (`#638 <https://github.com/selfpatch/ros2_medkit/pull/638>`_, `#612 <https://github.com/selfpatch/ros2_medkit/issues/612>`_)
* ``GET /faults/stream`` on an aggregating gateway relays its peers' fault events. It previously returned ``200`` with an open stream that only ever sent keepalive comments, which is indistinguishable from a healthy system - and on a deployment where the aggregator is the only reachable port, it was the only fault stream available (`#638 <https://github.com/selfpatch/ros2_medkit/pull/638>`_, `#611 <https://github.com/selfpatch/ros2_medkit/issues/611>`_)
* One addressing model for an aggregating entity's resources, so the aggregator no longer refuses or 404s work its peers can serve (`#626 <https://github.com/selfpatch/ros2_medkit/pull/626>`_, `#613 <https://github.com/selfpatch/ros2_medkit/issues/613>`_)
* Nested ``plugins.<name>.*`` parameters are rebuilt into a nested object instead of a flat dotted key, so nested plugin configuration reaches the plugin again (`#518 <https://github.com/selfpatch/ros2_medkit/pull/518>`_, `#520 <https://github.com/selfpatch/ros2_medkit/issues/520>`_)
* Discovery configuration is read from the documented top-level ``config:`` key. It was only ever read from ``discovery.config``, so the documented form was dropped without a word and unmanifested nodes leaked into the tree in hybrid mode (`#609 <https://github.com/selfpatch/ros2_medkit/pull/609>`_, `#529 <https://github.com/selfpatch/ros2_medkit/issues/529>`_)
* Every startup parameter that is coerced or refused is reported. A clamped thread count or keep-alive timeout used to change the value and log nothing, leaving the configuration file and the running process in silent disagreement. Integer parameters are read as the int64 a ROS parameter holds and validated before narrowing, so a value past ``INT_MAX`` can no longer wrap back into the legal band and pass its own range check, and range checks are written so that NaN is refused rather than accepted (`#607 <https://github.com/selfpatch/ros2_medkit/pull/607>`_, `#603 <https://github.com/selfpatch/ros2_medkit/issues/603>`_)
* ``server.executor_threads`` is real rather than advisory, and a cancel that runs out of time is reported as a timeout (`#593 <https://github.com/selfpatch/ros2_medkit/pull/593>`_)
* An unresponsive parameter node no longer hangs the REST API (`#532 <https://github.com/selfpatch/ros2_medkit/pull/532>`_), per-node parameter caches are bounded with LRU eviction (`#534 <https://github.com/selfpatch/ros2_medkit/pull/534>`_), the transport's node is released before the context dies (`#567 <https://github.com/selfpatch/ros2_medkit/pull/567>`_), and the parameter error surface is consistent between list and get, with a non-404 error winning over NOT_FOUND across nodes (`#540 <https://github.com/selfpatch/ros2_medkit/pull/540>`_, `#543 <https://github.com/selfpatch/ros2_medkit/pull/543>`_)
* Faults from plugin-provided entities are visible in the fault list, in fault detail and in freeze-frame through fault-scope ownership (`#503 <https://github.com/selfpatch/ros2_medkit/pull/503>`_), an external Component owns its fault-manager faults (`#530 <https://github.com/selfpatch/ros2_medkit/pull/530>`_), plugin-provided entities own their bulk data and logs (`#560 <https://github.com/selfpatch/ros2_medkit/pull/560>`_), and the external flag survives the hybrid merge (`#522 <https://github.com/selfpatch/ros2_medkit/pull/522>`_)
* Zero-config freeze-frame: the faulting entity's own data is captured with no configuration (`#538 <https://github.com/selfpatch/ros2_medkit/pull/538>`_), a fault already raised at gateway startup gets one (`#563 <https://github.com/selfpatch/ros2_medkit/pull/563>`_), and so does an entity serving last known values (`#565 <https://github.com/selfpatch/ros2_medkit/pull/565>`_)
* Data triggers work on plugin-provided entities, resolving from declared topics, and fail loudly on a topic name that cannot be resolved instead of silently never firing (`#592 <https://github.com/selfpatch/ros2_medkit/pull/592>`_). Trigger subscriptions go through the shared subscription executor (`#549 <https://github.com/selfpatch/ros2_medkit/pull/549>`_)
* Asset identity model with per-field provenance and merge-by-identity (`#488 <https://github.com/selfpatch/ros2_medkit/pull/488>`_)
* Build, image and test: an arm64 multi-arch image gated on tag or dispatch (`#508 <https://github.com/selfpatch/ros2_medkit/pull/508>`_), the fault bridges and ``ros2_medkit_fault_detection`` bundled into the image (`#470 <https://github.com/selfpatch/ros2_medkit/pull/470>`_, `#494 <https://github.com/selfpatch/ros2_medkit/pull/494>`_), coverage instrumentation for every C++ package (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), and clang-tidy analysing a package's translation units in parallel, cutting PR feedback time from about 56 minutes to about 35 (`#588 <https://github.com/selfpatch/ros2_medkit/pull/588>`_, `#590 <https://github.com/selfpatch/ros2_medkit/pull/590>`_)
* Contributors: @bburda, @mfaferek93, @YueBit

0.6.0 (2026-06-22)
------------------
* SOVD entity status and lifecycle control endpoints: ``GET /apps/{id}/status`` and ``GET /components/{id}/status``, plus lifecycle control routes backed by a new ``LifecycleProvider`` plugin interface and plugin-manager routing. Control returns ``501 Not Implemented`` until a provider is registered; the routes are RBAC-gated, advertised via a ``status`` link on app and component detail, and declared under the OpenAPI ``Lifecycle`` tag (`#437 <https://github.com/selfpatch/ros2_medkit/pull/437>`_)
* Accurate app and component status: app status is read from the managed-node lifecycle state through a ``GetState``-backed reader, and a component reports ``notReady`` when all hosted apps are offline while staying ``ready`` as long as it is reachable (`#455 <https://github.com/selfpatch/ros2_medkit/pull/455>`_)
* Bounded the executor and HTTP server thread pools, sized to the cold-wait plus SSE budget, with misconfiguration guards and a bounded keep-alive timeout (`#457 <https://github.com/selfpatch/ros2_medkit/pull/457>`_)
* Startup discovery summary logged at boot, with an empty-graph warning when no entities are discovered (`#438 <https://github.com/selfpatch/ros2_medkit/pull/438>`_)
* Bounded the unsupported-message-type cache and exposed its size; unknown message types now warn once instead of on every sample (`#450 <https://github.com/selfpatch/ros2_medkit/pull/450>`_)
* OpenAPI query parameters are derived from a typed query contract, tightening the query-parameter schema and its regression gate (`#417 <https://github.com/selfpatch/ros2_medkit/pull/417>`_)
* ``PluginContext`` can aggregate peer faults across daisy-chained gateways through the SOVD service interface (`#419 <https://github.com/selfpatch/ros2_medkit/pull/419>`_)
* Single-command bringup of the local medkit stack via ``bringup.launch.py`` and ``bringup_params.yaml`` (`#439 <https://github.com/selfpatch/ros2_medkit/pull/439>`_)
* Docker image enables CORS for the documented web UI path and uses explicit web UI origins instead of wildcard CORS (`#452 <https://github.com/selfpatch/ros2_medkit/pull/452>`_)
* Docker image bundles the CycloneDDS RMW as an opt-in alternative implementation (`#451 <https://github.com/selfpatch/ros2_medkit/pull/451>`_)
* Native gateway launch enables web UI CORS by default and honors the CORS settings from a supplied ``config_file`` (`#461 <https://github.com/selfpatch/ros2_medkit/pull/461>`_)
* Incremental, embedded-hardened entity cache: ``ThreadSafeEntityCache`` stores entities in a fixed-capacity ``SlotStore`` object pool indexed by open-addressed flat hash maps, and ``update_all`` reconciles the discovery output by id (add / remove / change only) so steady-state refresh does zero structural allocations. Capacity is reserved via ``entity_cache.capacity`` (default 256) and cache stats are exposed on ``/health`` as ``x-medkit-entity-cache``. Discovery refreshes are debounced via ``discovery.refresh_debounce_ms`` (default 1000) and operation ``type_info`` schemas resolve lazily, cutting gateway CPU under graph churn roughly 4x. Entity detail ``operations[]`` no longer embeds schemas eagerly; the schemas remain on the ``/operations`` resource (`#462 <https://github.com/selfpatch/ros2_medkit/pull/462>`_)
* Contributors: @bburda, @mfaferek93

0.5.0 (2026-06-08)
------------------

**Breaking Changes:**

* Typed router refactor. ``HandlerContext`` no longer carries
  ``send_json`` / ``send_error`` / ``send_plugin_error`` / ``send_dto`` /
  ``parse_body``: handlers return ``http::Result<TResponse>`` and the
  framework owns response writing through ``RouteRegistry``. The raw
  ``void(httplib::Request, httplib::Response)`` ``RouteRegistry`` lambda
  overloads are removed - call sites must use the typed
  ``reg.get<T>`` / ``reg.post<TBody, T>`` / ``reg.del<T>`` overloads, the
  multi-shape ``reg.post_alternates<TBody, TAlt...>`` /
  ``reg.del_alternates<TAlt...>``, or one of the named escape hatches
  (``reg.sse`` / ``reg.binary_download`` / ``reg.multipart_upload<T>`` /
  ``reg.static_asset`` / ``reg.docs_endpoint`` / ``reg.docs_subtree``).
  ``static_assert(dto::has_dto_shape_v<T>)`` gates every typed overload, so
  non-DTO return types fail at compile time. The plugin ABI is unaffected:
  ``PluginResponse`` keeps its ``send_json`` / ``send_error`` surface and
  now routes through the same internal ``http::detail::write_json_body``
  primitive as the framework, so plugin wire format is unchanged
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* Provider ABI typed. ``FaultProvider``, ``DataProvider``,
  ``OperationProvider``, and ``UpdateProvider::get_update`` return typed
  DTO envelopes (``FaultListResult`` / ``FaultDetailResult`` /
  ``FaultClearResult`` / the matching ``Data*Result`` and
  ``Operation*Result`` shapes / ``UpdateStatusResult``) instead of raw
  ``tl::expected<nlohmann::json, ErrorInfo>``. The wire bytes are
  byte-identical because each envelope wraps an opaque ``content`` object
  emitted verbatim by ``JsonWriter``; commercial and out-of-tree plugins
  must wrap their existing JSON in the matching envelope type
  (mechanical: ``Result.content = std::move(json_payload)``). The plugin
  ABI itself (``PluginRoute`` shape, ``PluginResponse`` ctor, plugin api
  version) is locked by ``test_plugin_abi_conformance`` and is unchanged
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``SchemaWriter`` emits optional DTO fields as
  ``anyOf: [<inner>, {type: "null"}]`` (the OpenAPI 3.1 idiom) instead of
  ``nullable: true``. Generated clients see ``T | null`` for every optional
  field rather than ``T | undefined``. Wire format is unchanged - the
  gateway still omits absent optional fields, and ``JsonReader`` continues
  to accept absent fields; the schema change only opts the published spec
  into round-tripping a literal ``null`` value cleanly for clients that
  prefer to send one. As part of this, a handful of fields that were
  previously emitted as an explicit JSON ``null`` when absent are now omitted
  entirely (consistent with the optional-omission policy): the script
  execution fields ``progress`` / ``started_at`` / ``completed_at`` /
  ``parameters`` / ``error`` (``GET .../scripts/{id}/executions/{eid}``) and
  the script ``parameters_schema`` field (``GET .../scripts/{id}``). Clients
  that tested ``field === null`` or relied on the key always being present must
  treat an absent key the same as ``null``
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* Synchronous operation-execution service-call failures
  (``POST /api/v1/{entity-path}/operations/{id}/executions`` when the underlying
  ROS 2 service call fails) now return the standard SOVD ``GenericError`` envelope
  (``{"error_code": "vendor-error", "vendor_code":
  "x-medkit-ros2-service-unavailable", "message": "Service call failed", ...}``,
  HTTP status 500 unchanged) instead of the previous bespoke nested
  ``{"error": {"code", "message", "details"}}`` object. This aligns the one
  remaining non-standard error path with every other gateway error; clients that
  parsed ``error.code`` / ``error.details`` for this specific failure must read
  ``vendor_code`` / ``parameters`` instead
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``GET /api/v1/{entity-path}/data`` now publishes the opaque ``DataListResult``
  schema (``{type: object, additionalProperties: true, x-medkit-opaque: true}``),
  matching how ``GET .../faults`` already publishes ``FaultListResult``. The wire
  payload is unchanged for runtime (ROS 2) entities - it is still
  ``{"items": [...], "x-medkit": {...}}`` built from the typed
  ``Collection<DataItem, DataListXMedkit>`` - but for plugin-owned entities the
  provider's free-form per-item shape now passes through verbatim instead of
  being re-parsed into ``Collection<DataItem>``. This fixes a regression in which
  plugin per-item fields (the OPC-UA plugin's ``value`` / ``unit`` / ``data_type``
  / ``writable``) were silently dropped by the typed re-parse. Clients that
  generated a typed ``DataItem`` model from the previous spec for this route now
  see an opaque object instead
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* Entity responses (areas, components, apps, functions - list items and detail)
  now always carry a top-level ``type`` discriminator (an enum of
  ``area`` / ``component`` / ``app`` / ``function``). Previously list items had no
  ``type`` and detail responses exposed it only inside ``x-medkit.entityType``.
  Additive and tolerant-client-safe; consumers keying on the entity kind can now
  read the top-level ``type``
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``ros2_medkit_msgs/srv/ClearFault`` request gains a ``bool skip_correlation_auto_clear`` field (see the per-entity fault scope entry below for the in-tree motivation). Adding a request field changes the service type hash, so out-of-tree callers that invoke the service directly (for example ``ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault ...`` as documented in the ``ros2_medkit_fault_manager`` README) must rebuild against the new ``ros2_medkit_msgs`` to keep talking to ``fault_manager``. The in-tree gateway client and server are updated together (`#395 <https://github.com/selfpatch/ros2_medkit/issues/395>`_)
* Per-entity fault routes are now correctly scoped to the entity's hosted apps. ``GET /api/v1/{entity-path}/faults/{fault_code}``, ``DELETE /api/v1/{entity-path}/faults/{fault_code}``, ``GET /api/v1/{entity-path}/faults``, and ``DELETE /api/v1/{entity-path}/faults`` previously fell back to a prefix match against the entity's ``namespace_path``; when that was empty (host-derived / synthetic components, manifest components without a ``namespace`` field, Areas, Functions, and Apps with a wildcard ``ros_binding.namespace_pattern``) the scope filter was silently disabled and the routes exposed - and on ``DELETE``, cleared - faults reported by apps that belonged to entirely different entities. All four handlers now resolve the addressed entity to its hosted-app FQN set (via the new ``HandlerContext::resolve_entity_source_fqns`` helper) and apply a strict all-sources scope check: a fault counts as in scope only when **every** entry in its ``reporting_sources`` is owned by the entity (exact FQN match, or strict path-child via ``<fqn>/...``). Per-fault routes return ``404 Resource Not Found`` for any fault that fails the check; collection routes return an empty ``items`` array. The underlying ``GetFault.srv`` contract is unchanged; ``ClearFault.srv`` gains a new ``skip_correlation_auto_clear`` request flag so per-entity DELETE can opt out of cascade-clearing correlated symptom fault codes that may live in other entities. Per-entity collection responses no longer include the global ``muted_count`` / ``cluster_count`` / ``muted_faults`` / ``clusters`` correlation metadata; those remain on the global ``GET /api/v1/faults`` route. Behavior changes visible to clients: (a) faults reported by apps outside the addressed entity are no longer returned or cleared via that entity's route, (b) **mixed-source** faults that include at least one out-of-entity reporter are likewise rejected with ``404`` on per-fault routes and excluded from per-entity collection responses (use the global ``GET /api/v1/faults`` to see them), (c) per-entity DELETE no longer cascade-clears correlated symptoms outside the entity (`#395 <https://github.com/selfpatch/ros2_medkit/issues/395>`_)
* ``GET /api/v1/updates/{id}/status`` no longer returns ``404`` for a registered-but-idle package; ``POST /api/v1/updates`` now seeds a ``pending`` status, so the endpoint returns ``200 {"status": "pending"}`` immediately after registration. ``404`` is reserved for packages that are not registered. Clients that used ``404`` as a signal for "registered but nothing started yet" must adapt (`#378 <https://github.com/selfpatch/ros2_medkit/issues/378>`_)

**Features:**

* Typed ``fan_out_collection<T>`` aggregating helper replaces raw-JSON ``merge_peer_items`` on the typed collection routes (data, operations, config, logs). Peer items are decoded via ``dto::JsonReader<T>``; items that fail validation are removed from the merged ``items`` array, recorded in ``x-medkit.peer_dropped_items`` with the JsonReader error plus a best-effort ``source_id``, and logged at ``WARN``. Items that parse successfully are re-serialized through the local ``dto::JsonWriter<T>``, so any peer-supplied fields outside the local DTO schema are dropped from the merged response (the previous raw passthrough preserved unknown peer fields verbatim). Previously, malformed peer items silently disappeared into the merged response; fleet operators can now detect inter-gateway schema drift directly on the wire (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``Collection<T, XMedkitT>`` is now a 2-parameter template. Domain list endpoints (faults, config, logs) reference their richer per-domain collection x-medkit struct (``FaultListXMedkit``, ``ConfigListXMedkit``, ``LogListXMedkit``) directly in the published schema instead of the generic ``XMedkitCollection``, so generated clients see aggregation counts, peer provenance, and ``peer_dropped_items`` from the schema. The data list builds the same typed ``Collection<DataItem, DataListXMedkit>`` internally (so the wire still carries those fields) but publishes the opaque ``DataListResult`` envelope, because plugin-owned data entities can return vendor per-item fields the typed item schema cannot describe (see the data-list breaking-change entry above) (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* New ``opaque_object("key", &T::field)`` DTO field descriptor in ``dto/contract.hpp``. Binds a ``nlohmann::json`` member as a typed "any JSON object" field: ``JsonWriter`` emits it verbatim, ``JsonReader`` rejects scalars / arrays / null, ``SchemaWriter`` emits ``{type: object, additionalProperties: true, x-medkit-opaque: true}``. Used for fields whose runtime shape is decided by an upstream component the gateway cannot introspect (live ROS message payloads, plugin-defined fault envelopes, action results) (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``GET /api/v1/faults/stream`` event payloads now carry an optional ``x-medkit`` SOVD payload-extension object with ``entity_type`` and ``entity_id`` fields. When the gateway can resolve the fault's first reporting source back to a SOVD entity (via the manifest-mode linking index, or a runtime-mode last-segment match against an existing App), consumers can hit ``/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}`` directly instead of HEAD-probing every entity. Resolution is snapshotted at event arrival, so a discovery refresh between enqueue and stream-out cannot retroactively change the entity reported to consumers. The ``x-medkit`` object is omitted entirely when no entity can be resolved, so existing SSE consumers ignore the addition (`#380 <https://github.com/selfpatch/ros2_medkit/issues/380>`_)
* Plugin API version bumped to v7. Adds ``PluginContext::notify_entities_changed(EntityChangeScope)`` lifecycle hook for plugins that mutate the entity surface at runtime; default no-op keeps v6 source code compiling unchanged against v7 headers. Binary compatibility is not provided: the plugin loader uses a strict equality check on ``plugin_api_version()``, so out-of-tree plugins must be recompiled (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* New ``discovery.manifest.fragments_dir`` parameter: gateway scans the directory for ``*.yaml`` / ``*.yml`` fragment files on every manifest load / reload and merges apps, components, and functions on top of the base manifest. Fragments are forbidden from declaring top-level ``areas``, ``metadata``, ``discovery``, ``scripts``, ``capabilities``, or ``lock_overrides`` - those stay in the base manifest. Presence of any forbidden key (including empty-valued ones like ``areas: []``) is reported as a ``FRAGMENT_FORBIDDEN_FIELD`` validation error that fails the load / reload. Unknown top-level keys (typos such as ``app:`` vs ``apps:``) are ignored with a warning log. Files merged in alphabetical order for deterministic duplicate-id errors (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* Fragment files are size-capped at 1 MiB (``ManifestParser::kMaxFragmentBytes``) before being read into memory, and any symlink resolving outside the canonical ``fragments_dir`` is skipped with a warning, so misconfigurations or symlink-based escapes cannot hand arbitrary bytes to the YAML parser (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* All-or-nothing fragment semantics: a single malformed or forbidden fragment fails the entire load / reload and keeps the previously-loaded manifest active (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* ``ManifestParser::parse_fragment_file`` convenience entrypoint that injects a synthetic ``manifest_version`` header when the fragment omits one
* See ``design/plugin_entity_notifications.rst`` for the lifecycle, merge-rule, and plugin-side write-contract walkthrough
* New ``GET /api/v1/apps/{app_id}/belongs-to`` discovery endpoint returning the areas and components an app belongs to; the ``belongs-to`` URI is advertised on ``GET /apps/{app_id}`` (`#196 <https://github.com/selfpatch/ros2_medkit/issues/196>`_)
* Pool-backed ``TopicDataProvider`` for live topic data: a shared subscription pool owned by a single-writer executor node, with LRU and idle eviction and publisher-QoS matching, replacing per-request subscriptions. Pool and executor health are surfaced as the ``x-medkit-subscription-executor`` vendor-extension stats on ``GET /api/v1/health``, read atomically so ``/health`` never blocks under load (`#384 <https://github.com/selfpatch/ros2_medkit/issues/384>`_)
* ``GET /api/v1/updates/{id}/status`` exposes the update lifecycle ``phase`` under the response ``x-medkit`` object
* ``gateway.launch.py`` and ``gateway_https.launch.py`` accept a ``config_file`` launch argument pointing at an external parameter YAML. Parameters present in the file override the matching gateway defaults; parameters the file omits keep their launch defaults instead of being reset (`#408 <https://github.com/selfpatch/ros2_medkit/pull/408>`_)
* Plugin-facing headers are httplib-free across the ``.so`` boundary: the handler-result vocabulary (``Result``, ``NoContent``, ``Forwarded``, ``ValidatorResult``, ``ResponseAttachments``) moved to a new leaf header ``http/handler_result.hpp`` so provider and DTO interfaces no longer transitively include ``<httplib.h>``. Out-of-tree plugins built against the installed gateway (build-farm / Docker topology, where the vendored httplib is not on the include path) compile again; no ABI, wire, or behaviour change. A pre-push gate and CI scan keep the plugin-facing headers httplib-free (`#411 <https://github.com/selfpatch/ros2_medkit/pull/411>`_)
* Contributors: @bburda, @eclipse0922, @evTessellate, @mfaferek93

0.4.0 (2026-03-20)
------------------

**Breaking Changes:**

* ``GET /version-info`` response key renamed from ``sovd_info`` to ``items`` for SOVD alignment (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``GET /`` root endpoint restructured: ``endpoints`` is now a flat string array, added ``capabilities`` object, ``api_base`` field, and ``name``/``version`` top-level fields (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Default rosbag storage format changed from ``sqlite3`` to ``mcap`` (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Plugin API version bumped to v4 - added ``ScriptProvider``, locking API, and extended ``PluginContext`` with entity snapshot, fault listing, and sampler registration
* ``GraphProviderPlugin`` extracted to separate ``ros2_medkit_graph_provider`` package

**Features:**

*Discovery & Merge Pipeline:*

* Layered merge pipeline for hybrid discovery with per-layer, per-field-group merge policies (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Gap-fill configuration: control heuristic entity creation with ``allow_heuristic_*`` options and namespace filtering (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Plugin layer: ``IntrospectionProvider`` now wired into discovery pipeline via ``PluginLayer`` (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``/health`` endpoint includes merge pipeline diagnostics (layers, conflicts, gap-fill stats) (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Entity detail responses now include ``logs``, ``bulk-data``, ``cyclic-subscriptions`` URIs (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Entity capabilities fix: areas and functions now report correct resource collections (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``discovery.manifest.enabled`` / ``discovery.runtime.enabled`` parameters for hybrid mode
* ``NewEntities.functions`` - plugins can now produce Function entities
* ``GET /apps/{id}/is-located-on`` endpoint for reverse host lookup (app to component)
* Beacon discovery plugin system - push-based entity enrichment via ROS 2 topic
* ``x-medkit-topic-beacon`` and ``x-medkit-param-beacon`` vendor extension REST endpoints
* Linux introspection plugins: procfs, systemd, and container plugins via ``x-medkit-*`` vendor endpoints (`#263 <https://github.com/selfpatch/ros2_medkit/pull/263>`_)

*Locking:*

* SOVD-compliant resource locking: acquire, release, extend with session tracking and expiration
* Lock enforcement on all mutating handlers (PUT, POST, DELETE)
* Per-entity lock configuration via manifest YAML with ``required_scopes``
* Lock API exposed to plugins via ``PluginContext``
* Automatic cyclic subscription cleanup on lock expiry
* ``LOCKS`` capability in entity descriptions

*Scripts:*

* SOVD script execution endpoints: CRUD for scripts and executions with subprocess execution
* ``ScriptProvider`` plugin interface for custom script backends
* ``DefaultScriptProvider`` with manifest + filesystem CRUD, argument passing, and timeout
* Manifest-defined scripts: ``ManifestParser`` populates ``ScriptsConfig.entries`` from manifest YAML
* ``allow_uploads`` config toggle for hardened deployments
* RBAC integration for script operations

*Logging:*

* ``LogProvider`` plugin interface for custom log backends (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``LogManager`` with ``/rosout`` ring buffer and plugin delegation
* ``/logs`` and ``/logs/configuration`` endpoints
* ``LOGS`` capability in discovery responses
* Configurable log buffer size via parameters
* Area and function log endpoints with namespace aggregation (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)

*Triggers:*

* Condition-based triggers with CRUD endpoints, SSE event streaming, and hierarchy matching
* ``TriggerManager`` with ``ConditionEvaluator`` interface and 4 built-in evaluators (OnChange, OnChangeTo, EnterRange, LeaveRange)
* ``ResourceChangeNotifier`` for async dispatch from FaultManager, UpdateManager, and OperationManager
* ``TriggerTopicSubscriber`` for data trigger ROS 2 topic subscriptions
* Persistent trigger storage via SQLite with restore-on-restart support
* ``TriggerTransportProvider`` plugin interface for custom trigger delivery

*OpenAPI & Documentation:*

* ``RouteRegistry`` as single source of truth for routes and OpenAPI metadata
* ``OpenApiSpecBuilder`` for full OpenAPI 3.1.0 document assembly with ``SchemaBuilder`` and ``PathBuilder``
* Compile-time Swagger UI embedding (``ENABLE_SWAGGER_UI``)
* Named component schemas with ``$ref``, clean ``operationId`` values, endpoint descriptions, ``GenericError`` schema refs, ``info.contact``, Spectral-clean output, multipart upload schemas, static spec caching
* SOVD compliance documentation with resource collection support matrix (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)

*Other:*

* Multi-collection cyclic subscription support (data, faults, logs, configurations, update-status)
* Generation-based caching for capability responses via ``CapabilityGenerator``
* ``PluginContext::get_child_apps()`` for Component-level aggregation
* Sub-resource RBAC patterns for all collections
* Auto-populate gateway version from ``package.xml`` via CMake
* Namespaced fault manager integration - ``FaultManagerPaths`` resolves service/topic names for custom namespaces
* Grouped ``fault_manager.*`` parameter namespace for cleaner configuration

**Build:**

* Extracted shared cmake modules into ``ros2_medkit_cmake`` package (`#294 <https://github.com/selfpatch/ros2_medkit/pull/294>`_)
* Auto-detect ccache for faster incremental rebuilds
* Precompiled headers for gateway package
* Centralized clang-tidy configuration (opt-in locally, mandatory in CI)

**Tests:**

* Unit tests for DiscoveryHandlers, OperationHandlers, ScriptHandlers, LockHandlers, LockManager, ScriptManager, DefaultScriptProvider
* Comprehensive integration tests for locking, scripts, graph provider plugin, beacon plugins, OpenAPI/docs, logging, namespaced fault manager
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------

**Features:**

* Gateway plugin framework with dynamic C++ plugin loading (`#237 <https://github.com/selfpatch/ros2_medkit/pull/237>`_)
* Software updates plugin with 8 SOVD-compliant endpoints (`#237 <https://github.com/selfpatch/ros2_medkit/pull/237>`_, `#231 <https://github.com/selfpatch/ros2_medkit/pull/231>`_)
* SSE-based periodic data subscriptions for real-time streaming without polling (`#223 <https://github.com/selfpatch/ros2_medkit/pull/223>`_)
* Global ``DELETE /api/v1/faults`` endpoint (`#228 <https://github.com/selfpatch/ros2_medkit/pull/228>`_)
* Return HEALED/PREPASSED faults via status filter (`#218 <https://github.com/selfpatch/ros2_medkit/pull/218>`_)
* Bulk data upload and delete endpoints (`#216 <https://github.com/selfpatch/ros2_medkit/pull/216>`_)
* Token-bucket rate limiting middleware, configurable per-endpoint (`#220 <https://github.com/selfpatch/ros2_medkit/pull/220>`_)
* Reduce lock contention in ConfigurationManager (`#194 <https://github.com/selfpatch/ros2_medkit/pull/194>`_)
* Cache component topic map to avoid per-request graph rebuild (`#212 <https://github.com/selfpatch/ros2_medkit/pull/212>`_)
* Require cpp-httplib >= 0.14 in pkg-config check (`#230 <https://github.com/selfpatch/ros2_medkit/pull/230>`_)
* Add missing ``ament_index_cpp`` dependency to ``package.xml`` (`#191 <https://github.com/selfpatch/ros2_medkit/pull/191>`_)
* Unit tests for HealthHandlers, DataHandlers, and AuthHandlers (`#232 <https://github.com/selfpatch/ros2_medkit/pull/232>`_, `#234 <https://github.com/selfpatch/ros2_medkit/pull/234>`_, `#233 <https://github.com/selfpatch/ros2_medkit/pull/233>`_)
* Standardize include guards to ``#pragma once`` (`#192 <https://github.com/selfpatch/ros2_medkit/pull/192>`_)
* Use ``foreach`` loop for CMake coverage flags (`#193 <https://github.com/selfpatch/ros2_medkit/pull/193>`_)
* Migrate ``ament_target_dependencies`` to compat shim for Rolling (`#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda, @eclipse0922, @mfaferek93

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
* HTTP REST gateway for ros2_medkit diagnostics system
* SOVD-compatible entity discovery with four entity types:

  * Areas, Components, Apps, Functions
  * HATEOAS links and capabilities in all responses
  * Relationship endpoints (subareas, subcomponents, related-apps, hosts)

* Three discovery modes:

  * Runtime-only: automatic ROS 2 graph introspection
  * Manifest-only: YAML manifest with validation (11 rules)
  * Hybrid: manifest as source of truth + runtime linking

* REST API endpoints:

  * Fault management: GET/POST/DELETE /api/v1/faults
  * Data access: topic sampling via GenericSubscription
  * Operations: service calls and action goals via GenericClient
  * Configuration: parameter get/set via ROS 2 parameter API
  * Snapshots: GET /api/v1/faults/{code}/snapshots
  * Rosbag: GET /api/v1/faults/{code}/snapshots/bag

* Server-Sent Events (SSE) at /api/v1/faults/stream:

  * Multi-client support with thread-safe event queue
  * Keepalive, Last-Event-ID reconnection, configurable max_clients

* JWT-based authentication with configurable policies
* HTTPS/TLS support via OpenSSL and cpp-httplib
* Native C++ ROS 2 serialization via ros2_medkit_serialization (no CLI dependencies)
* Contributors: Bartosz Burda, Michal Faferek
