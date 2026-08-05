Status and Lifecycle Endpoints
==============================

This document describes the design of the SOVD status/lifecycle endpoints
in ros2_medkit_gateway. It covers the six REST routes, the status read
semantics for each entity type, the ``LifecycleProvider`` plugin seam,
error mapping, and SOVD requirement coverage.

.. contents:: Table of Contents
   :local:
   :depth: 3

Overview
--------

SOVD ISO 17978-3 defines a lifecycle model for software entities. A client
reads the current lifecycle state via a GET and drives transitions via PUT
sub-resources. The gateway implements these routes for **Apps** and
**Components** only (Areas and Functions do not carry lifecycle state in the
current SOVD profile).

The six routes are:

+-------------------------------------------+--------+--------------------------------------+
| Path                                      | Method | Description                          |
+===========================================+========+======================================+
| ``/{entity}/{id}/status``                 | GET    | Read current lifecycle status        |
+-------------------------------------------+--------+--------------------------------------+
| ``/{entity}/{id}/status/start``           | PUT    | Request transition to started state  |
+-------------------------------------------+--------+--------------------------------------+
| ``/{entity}/{id}/status/restart``         | PUT    | Request controlled restart           |
+-------------------------------------------+--------+--------------------------------------+
| ``/{entity}/{id}/status/force-restart``   | PUT    | Request forced restart               |
+-------------------------------------------+--------+--------------------------------------+
| ``/{entity}/{id}/status/shutdown``        | PUT    | Request controlled shutdown          |
+-------------------------------------------+--------+--------------------------------------+
| ``/{entity}/{id}/status/force-shutdown``  | PUT    | Request forced shutdown              |
+-------------------------------------------+--------+--------------------------------------+

where ``{entity}`` is either ``apps`` or ``components``.

Route Registration
------------------

The lifecycle routes are registered **outside** the four-entity-type loop in
``rest_server.cpp::setup_routes()``, covering only ``apps`` and
``components``:

.. code-block:: cpp

   for (const auto & et_lc :
        std::vector<std::pair<const char *, const char *>>{
            {"apps", "app"}, {"components", "component"}}) {
     const std::string base_lc = "/" + et_lc.first + "/{" + et_lc.second + "_id}";

     // GET and PUT use different HTTP methods so neither can shadow the
     // other; registration order within the loop is arbitrary.
     for (const auto & action :
          {"start", "restart", "force-restart", "shutdown", "force-shutdown"}) {
       reg.put<http::Accepted<http::NoContent>>(base_lc + "/status/" + action, ...);
     }
     reg.get<dto::LifecycleStatusResponse>(base_lc + "/status", ...);
   }

The GET ``/status`` route and the PUT ``/status/{action}`` routes use
different HTTP methods, so neither can shadow the other. cpp-httplib keeps a
separate handler list per method, so registration order does not matter.

Status Read Semantics
---------------------

``GET /{entity}/{id}/status`` returns a ``LifecycleStatusResponse`` DTO:

.. code-block:: text

   {
     "status": "ready" | "notReady",
     "start":          "/api/v1/{entity}/{id}/status/start",    // optional
     "restart":        "/api/v1/{entity}/{id}/status/restart",  // optional
     "force-restart":  "/api/v1/{entity}/{id}/status/force-restart", // optional
     "shutdown":       "/api/v1/{entity}/{id}/status/shutdown",       // optional
     "force-shutdown": "/api/v1/{entity}/{id}/status/force-shutdown"  // optional
   }

The optional transition URI fields are only present when the registered
``LifecycleProvider`` signals support for that transition. Each field value
is the absolute path the client should call to trigger the transition.

**App status:** for a managed lifecycle node (one exposing ``get_state`` and ``change_state``),
the status is read from the node's lifecycle state via ``lifecycle_msgs/srv/GetState``: the
``active`` state is ``"ready"``; any other state, or an unreachable/timed-out read, is
``"notReady"``. This subsumes liveness, since a dead node's ``get_state`` service is gone. For a
plain node with no lifecycle services, the status falls back to ``App::is_online`` (presence in
the ROS 2 graph), which is the best signal available for an unmanaged node. A node that is not
online short-circuits to ``"notReady"`` without a GetState read (an offline node cannot be
``active``), which also avoids a blocking read against a crashed managed node whose services still
linger in the cache. The GetState read runs on a private node and executor (spun inline), so it
never blocks or races the gateway executor; it is, however, serialized by an internal mutex, so a
reachable-but-slow managed node holds that mutex across its spin and delays other concurrent
``/status`` reads for up to the (short) read timeout.

**Component status:** the synthetic host component (the one carrying ``host_metadata``,
populated by ``HostInfoProvider``) is ``"ready"`` while the gateway is serving the request -
its substrate is the gateway host, which is reachable by definition. Any other local
component derives readiness from its hosted Apps: a component that hosts Apps is
``"notReady"`` when every one of them is offline (its subsystem is down), and ``"ready"`` when
at least one is online. A component with no hosted Apps is a pure grouping and stays
``"ready"`` (the local substrate is reachable; a single down App is that App's own
``"notReady"``, not enough to mark the component down). Remote components are reached through
aggregation forwarding, so peer reachability drives their status.

When a ``LifecycleProvider`` is registered for the entity the provider's
response takes precedence over the cache-derived default. The handler also
fills absolute transition URIs from the provider-supplied optional fields.

LifecycleProvider Interface
---------------------------

``LifecycleProvider`` is a per-entity plugin interface defined in
``include/ros2_medkit_gateway/core/providers/lifecycle_provider.hpp``.
It follows the same typed-DTO style as ``OperationProvider``.

.. plantuml::
   :caption: LifecycleProvider and handler relationships

   @startuml lifecycle_provider

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   package "core/providers/" {
       class LifecycleProvider <<interface>> {
           + get_status(entity_id): expected<LifecycleStatusResponse, LifecycleProviderErrorInfo>
           + request_transition(entity_id, transition): expected<monostate, LifecycleProviderErrorInfo>
       }
   }

   package "core/http/handlers/" {
       class LifecycleHandlers {
           + handle_get_status(req): Result<LifecycleStatusResponse>
           + handle_transition(req, transition): Result<pair<Accepted<NoContent>, ResponseAttachments>>
           - ctx_: HandlerContext
           - plugin_mgr_: PluginManager*
       }
   }

   package "core/plugins/" {
       class PluginManager {
           + get_lifecycle_provider_for_entity(entity_id): LifecycleProvider*
       }
   }

   LifecycleHandlers --> PluginManager : queries for provider
   LifecycleHandlers --> LifecycleProvider : calls get_status / request_transition
   PluginManager ..> LifecycleProvider : returns registered instance

   @enduml

A substrate plugin (ROS 2 lifecycle node manager, process/container
supervisor, host-level service manager) registers a ``LifecycleProvider``
implementation via ``PluginManager``. The handler looks up the provider per
entity at request time via ``plugin_mgr_->get_lifecycle_provider_for_entity(entity_id)``.

**Fallback behavior when no provider is registered:**

- ``GET /status`` returns a cache-derived ``LifecycleStatusResponse`` with
  no transition URI fields (no actuation capability advertised).
- ``PUT /status/{action}`` returns ``501 Not Implemented`` with error code
  ``not-implemented``.

Control remains ``501`` for all five PUT transitions until a plugin
registers a provider for the entity.

Error Mapping
-------------

``LifecycleProviderErrorInfo`` carries a typed error code, a message string,
and an optional HTTP status hint. The handler maps it to a gateway
``ErrorInfo`` via the file-local ``to_error_info`` helper:

+-----------------------------------+---------+----------------------------------+
| ``LifecycleProviderError``        | HTTP    | SOVD error code                  |
+===================================+=========+==================================+
| ``AccessDenied``                  | 403     | ``insufficient-access-rights``   |
+-----------------------------------+---------+----------------------------------+
| ``PreconditionFailed``            | 409     | ``precondition-not-fulfilled``   |
+-----------------------------------+---------+----------------------------------+
| ``Unsupported``                   | 501     | ``not-implemented``              |
+-----------------------------------+---------+----------------------------------+
| ``EntityNotFound``,               | *hint*  | ``x-medkit-plugin-error``        |
| ``TransportError``, ``Internal``  |         |                                  |
+-----------------------------------+---------+----------------------------------+

The HTTP status from ``LifecycleProviderErrorInfo::http_status`` is clamped
to the range 400-599. Plugin exceptions are caught; unknown exceptions map
to ``500 / x-medkit-plugin-error``.

There are two distinct 403 paths. The auth middleware returns 403 before the
handler runs when the client's RBAC role is insufficient (for example a viewer
calling a PUT transition). Separately, the provider error table's
``AccessDenied`` row maps to 403 when a registered ``LifecycleProvider`` itself
refuses the operation for a substrate-level reason, unrelated to the gateway
RBAC check.

Transition Flow
---------------

.. plantuml::
   :caption: PUT lifecycle transition - full request flow

   @startuml lifecycle_transition_flow

   participant Client
   participant RouteRegistry as reg
   participant LifecycleHandlers as handler
   participant PluginManager as pm
   participant LifecycleProvider as provider

   Client -> reg : PUT /api/v1/apps/{id}/status/restart
   reg -> handler : handle_transition(req, "restart")
   handler -> handler : validate_entity_for_route(req, entity_id)

   alt entity not found
       handler --> reg : 404 GenericError
       reg --> Client : 404
   end

   handler -> pm : get_lifecycle_provider_for_entity(entity_id)

   alt no provider registered
       handler --> reg : 501 not-implemented
       reg --> Client : 501 Not Implemented
   end

   pm --> handler : LifecycleProvider*
   handler -> provider : request_transition(entity_id, "restart")

   alt provider error
       provider --> handler : unexpected(LifecycleProviderErrorInfo)
       handler -> handler : to_error_info(e) -> ErrorInfo
       handler --> reg : ErrorInfo (403 / 409 / 501 / 5xx)
       reg --> Client : error response
   end

   provider --> handler : expected<monostate, ...> (success)
   handler -> handler : build ResponseAttachments (Location header only)
   handler --> reg : pair<Accepted<NoContent>, ResponseAttachments>
   reg --> Client : 202 Accepted\nLocation: /api/v1/apps/{id}/status

   @enduml

The transition is **async**: the PUT returns 202 with a ``Location`` header
pointing to ``GET /{entity}/{id}/status``. The client polls that endpoint to
observe the state change. The provider is responsible for initiating the
substrate-level operation (e.g., calling a ROS 2 lifecycle service or
sending a signal to a process manager); it returns immediately on acceptance.

The 202 is declared by the handler's return type
(``http::Accepted<http::NoContent>``), not set at runtime, so the OpenAPI
document and the wire status come from one place and cannot disagree. The
``ResponseAttachments`` companion carries only the ``Location`` header. See
:doc:`dto_contract` for the status-wrapper contract.

SOVD Requirement Coverage
--------------------------

+----------------+--------+-------------------------------------+
| Requirement    | Status | Notes                               |
+================+========+=====================================+
| REQ_INTEROP_076| verified| GET /status implemented; default   |
|                |        | status derived from entity cache    |
|                |        | when no provider is registered.     |
+----------------+--------+-------------------------------------+
| REQ_INTEROP_077| open   | Route registered; returns 501 until |
|                |        | a LifecycleProvider registers.      |
+----------------+--------+-------------------------------------+
| REQ_INTEROP_078| open   | Same as REQ_INTEROP_077.            |
+----------------+--------+-------------------------------------+
| REQ_INTEROP_079| open   | Same as REQ_INTEROP_077.            |
+----------------+--------+-------------------------------------+
| REQ_INTEROP_080| open   | Same as REQ_INTEROP_077.            |
+----------------+--------+-------------------------------------+
| REQ_INTEROP_081| open   | Same as REQ_INTEROP_077.            |
+----------------+--------+-------------------------------------+

REQ_INTEROP_076 is verified by the lifecycle integration tests. Requirements
077-081 remain open because they require actuation at the substrate level
(process/container/ROS 2 lifecycle node). They will be marked verified when a
plugin implementing ``LifecycleProvider::request_transition`` ships.

Key Files
---------

``include/ros2_medkit_gateway/dto/lifecycle.hpp``
    ``LifecycleStatusResponse`` DTO with ``dto_fields`` and ``dto_name``
    specializations. Wire keys ``force-restart`` and ``force-shutdown`` use
    hyphen as required by the SOVD spec.

``include/ros2_medkit_gateway/core/providers/lifecycle_provider.hpp``
    ``LifecycleProvider`` pure virtual interface and
    ``LifecycleProviderErrorInfo`` error struct.

``include/ros2_medkit_gateway/core/http/handlers/lifecycle_handlers.hpp``
    ``LifecycleHandlers`` class declaration.

``src/http/handlers/lifecycle_handlers.cpp``
    Handler implementations for GET status and PUT transition, including
    the ``to_error_info`` mapping helper.

``src/http/rest_server.cpp``
    Route registration (search for ``// === Lifecycle``).
