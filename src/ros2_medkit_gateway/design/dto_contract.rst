DTO Contract Layer
==================

This document describes the typed DTO contract layer of the
ros2_medkit_gateway. It covers the problem it solves, the architecture of the
contract primitives, the three code-generation visitors, the OpenAPI schema
registry, the typed router that consumes the contract, the named escape
hatches for non-DTO routes, the typed-only Provider ABI, and the workflow for
adding new endpoints.

.. contents:: Table of Contents
   :local:
   :depth: 3

Overview
--------

Before this layer existed, a handler in the gateway had three independent
artefacts that described the same wire payload:

1. Hand-written ``nlohmann::json`` construction in the handler body.
2. A ``SchemaBuilder::*_schema()`` factory that produced the matching OpenAPI
   JSON Schema object.
3. An ``XMedkit`` fluent builder that assembled the ``x-medkit`` vendor
   extension block.

These three artefacts had no mechanical relationship. A field added to the
handler body had to be separately added to the schema factory and, if it
appeared in the ``x-medkit`` block, also to the fluent builder. Because the
compiler had no way to enforce the relationship, schemas and wire payloads
drifted silently. The OpenAPI spec served at ``/api/v1/docs`` described a
different shape than what the endpoint actually returned.

The DTO contract layer resolves this by making the C++ struct the single
source of truth. The same descriptor tuple that defines the struct is used
by three template visitors to produce the wire JSON, the OpenAPI schema, and
the request-body parser. Adding a field to the struct and its descriptor
automatically updates all three outputs.

Architecture
------------

The contract is implemented entirely as header-only templates in
``include/ros2_medkit_gateway/dto/``. No virtual dispatch, no runtime type
erasure, and no separate code-generation step are needed.

.. plantuml::
   :caption: DTO Contract Layer - Component Relationships

   @startuml dto_contract_overview

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   package "dto/" {
       class "contract.hpp" as contract {
           Field<Class, Member>
           dto_fields<T> constexpr tuple
           dto_name<T> string_view
           is_dto_v<T> bool
           for_each_field<T>(visitor)
       }

       class JsonWriter<T> {
           + write(obj: T): json
       }

       class SchemaWriter<T> {
           + schema(): json
       }

       class JsonReader<T> {
           + read(j: json): expected<T, vector<FieldError>>
       }

       class "registry.hpp" as registry {
           AllDtos tuple
           collect_component_schemas(): json
       }
   }

   package "openapi/" {
       class RouteRegistry {
           + get<T>(path, handler)
           + post<TBody,T>(path, handler)
           + del<T>(path, handler)
           + post_alternates<TBody, TAlt...>(path, handler)
           + del_alternates<TAlt...>(path, handler)
           + sse(path, factory)
           + binary_download(path, handler)
           + multipart_upload<T>(path, handler)
           + static_asset(path, handler)
           + docs_endpoint(path, handler)
           + docs_subtree(regex, handler)
       }

       class OpenApiSpecBuilder {
           + build(): json
       }
   }

   JsonWriter .up.|> contract : folds over dto_fields
   SchemaWriter .up.|> contract : folds over dto_fields
   JsonReader .up.|> contract : folds over dto_fields

   RouteRegistry --> JsonWriter : serializes Result<T>
   RouteRegistry --> JsonReader : parses TBody on POST/PUT/PATCH
   OpenApiSpecBuilder --> registry : collect_component_schemas
   OpenApiSpecBuilder --> RouteRegistry : to_openapi_paths
   registry --> SchemaWriter : per DTO in AllDtos

   @enduml

Field Descriptor (``Field<C, M>``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each field in a DTO is described by a ``Field<Class, Member>`` aggregate
defined in ``contract.hpp``:

.. code-block:: cpp

   template <class Class, class Member>
   struct Field {
     std::string_view key;          // JSON wire key
     Member Class::*ptr;            // pointer-to-member
     Presence presence;             // kRequired or kOptional
     std::string_view description;  // OpenAPI property description
     const std::string_view * enum_values;  // allowed string values (or nullptr)
     std::size_t enum_count;
   };

Fields are never constructed directly. The ``field()`` and ``field_enum()``
factory functions deduce the class and member types from the pointer-to-member
argument:

.. code-block:: cpp

   // Required string field
   field("fault_code", &FaultListItem::fault_code)

   // Optional field (presence deduced from std::optional<> member type)
   field("description", &FaultListItem::description)

   // Enum-constrained field with inline constexpr string_view array
   field_enum("status", &FaultStatus::aggregated_status, kFaultAggregatedStatusValues)

``dto_fields<T>`` - the Descriptor Tuple
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The descriptor tuple for a type ``T`` is a ``constexpr`` specialization of
the variable template ``dto_fields<T>``:

.. code-block:: cpp

   template <>
   inline constexpr auto dto_fields<FaultListItem> = std::make_tuple(
       field("fault_code", &FaultListItem::fault_code),
       field("severity",   &FaultListItem::severity),
       field("description",&FaultListItem::description),
       field("status",     &FaultListItem::status));

The primary template is a sentinel pointer (``detail::not_a_dto<T>*``) so
the type is identifiable for ``is_dto_v`` checks without forcing
instantiation of ``not_a_dto<T>`` at every probe. The ``is_dto_v<T>`` trait
returns ``true`` only when a specialization exists, which gates all three
visitors at compile time.

**Placement rule:** every ``dto_fields<X>`` and ``dto_name<X>``
specialization must appear in the same header as the struct declaration. A
translation unit that instantiates a visitor before seeing the specialization
silently binds the sentinel, producing a latent ODR-adjacent bug.

``dto_name<T>`` - Schema Registry Key
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each DTO names itself in ``components/schemas`` via a ``constexpr string_view``
specialization:

.. code-block:: cpp

   template <>
   inline constexpr std::string_view dto_name<FaultListItem> = "FaultListItem";

The name is used by ``SchemaWriter`` when emitting ``$ref`` cross-references
and by ``collect_component_schemas()`` when populating the OpenAPI registry.

The Three Visitors
~~~~~~~~~~~~~~~~~~

All three visitors fold over ``dto_fields<T>`` using ``for_each_field<T>()``,
which calls ``std::apply`` over the constexpr tuple. The fold is entirely at
compile time; no runtime reflection is involved.

**JsonWriter** (``json_writer.hpp``)

Serializes a DTO instance to a ``nlohmann::json`` object. Optional members
that have no value are omitted from the output. Nested DTO members are
recursively serialized. ``std::vector`` members become JSON arrays.
``std::variant`` members are serialized as the active alternative.
``nlohmann::json`` members pass through unchanged.

**SchemaWriter** (``schema_writer.hpp``)

Generates the OpenAPI 3.1 ``components/schemas`` entry for a type. Each
field maps to a JSON Schema property. Required fields are listed in the
``required`` array. Nested DTO types become ``$ref`` entries pointing to
the named schema. Optional wrapper types are unwrapped before schema
generation. Enum-constrained string fields include an ``enum`` array.

**JsonReader** (``json_reader.hpp``)

Parses and validates a ``nlohmann::json`` object into a DTO instance.
Collects all field-level errors rather than short-circuiting on the first
failure, returning ``tl::expected<T, std::vector<FieldError>>`` on
completion. Required fields missing or null produce a ``FieldError``.
Unknown extra fields in the input are silently ignored (lenient parsing).
Enum-constrained string fields are validated against the allowed set after
decoding.

.. plantuml::
   :caption: Request Lifecycle through the Typed Router

   @startuml dto_request_lifecycle

   participant Client
   participant RouteRegistry as reg
   participant JsonReader
   participant Handler
   participant JsonWriter

   == Request body parsing ==

   Client -> reg : POST /api/v1/.../executions\n{...JSON body...}
   reg -> JsonReader : read(body_json) [TBody = ExecutionUpdateRequest]
   JsonReader -> JsonReader : fold over dto_fields<ExecutionUpdateRequest>
   JsonReader --> reg : expected<ExecutionUpdateRequest, vector<FieldError>>
   alt validation failed
       reg --> Client : 400 GenericError (field errors collected)
   else validation ok
       reg -> Handler : handler(TypedRequest, ExecutionUpdateRequest)
   end

   == Response serialization ==

   Handler -> Handler : build OperationExecution dto
   Handler --> reg : Result<OperationExecution> (success branch)
   reg -> JsonWriter : write(dto)
   JsonWriter -> JsonWriter : fold over dto_fields<OperationExecution>
   JsonWriter --> reg : nlohmann::json object
   reg --> Client : 200 OK + JSON response

   @enduml

AllDtos Registry (``registry.hpp``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``AllDtos`` is a single ``std::tuple`` listing every named DTO type:

.. code-block:: cpp

   using AllDtos = std::tuple<
       GenericError,
       AreaListItem, AreaDetail,
       FaultListItem, FaultDetail, FaultStatus,
       OperationItem, OperationExecution,
       // ... all other domain DTOs ...
   >;

The free function ``collect_component_schemas()`` iterates ``AllDtos`` at
compile time (via ``std::index_sequence``) and calls
``SchemaWriter<T>::schema()`` for each type. ``SchemaBuilder::component_schemas()``
(in ``src/openapi/schema_builder.cpp``) returns exactly this map: every entry in
``components/schemas`` is generated from ``AllDtos``, with no hand-written
survivors merged in. No runtime loop over a dynamic registry is required.

The hand-written schema factories that remain on ``SchemaBuilder`` -
``from_ros_msg`` / ``from_ros_srv_request`` / ``from_ros_srv_response`` (for
dynamic ROS 2 payloads whose field names are not known at compile time) and
``binary_schema`` / ``generic_object_schema`` - are no longer part of the
``components/schemas`` map. They are called by the path builder
(``src/openapi/path_builder.cpp``) to emit *inline* operation schemas for the
per-topic / per-service / per-action routes, whose request and response shape is
derived from the live ROS 2 type rather than from a named DTO.

The ``Collection<T>`` template is a generic DTO for paginated list responses
(``{"items": [...]}``). It is specialized for each element type in
``AllDtos`` and given a name like ``"FaultList"`` via a ``dto_name``
specialization.

Typed Router
------------

Every built-in route is registered through ``RouteRegistry`` using a typed
overload that names its DTOs in the template parameter list. The framework
owns request decoding, response writing, and status-code dispatch; handlers
never touch ``httplib::Response``.

Handler Signatures
~~~~~~~~~~~~~~~~~~

A typed handler returns ``http::Result<TResponse>`` (which is
``tl::expected<TResponse, ErrorInfo>``) and receives a ``TypedRequest``
plus, on POST / PUT / PATCH overloads, an already-parsed ``TBody``:

.. code-block:: cpp

   // GET /entity/{id}/resource -> 200 + JSON body
   reg.get<dto::MyResponse>(
         "/entity/{id}/resource",
         [this](http::TypedRequest req) -> http::Result<dto::MyResponse> {
           // ... build dto::MyResponse or return tl::make_unexpected(err) ...
         })
      .tag("MyTag")
      .summary("...")
      .operation_id("getMyResource");

   // POST /entity/{id}/resource with parsed body -> 200 + JSON body
   reg.post<dto::MyCreateRequest, dto::MyResponse>(
         "/entity/{id}/resource",
         [this](http::TypedRequest req, dto::MyCreateRequest body)
             -> http::Result<dto::MyResponse> {
           // ... return result ...
         });

Success Status Lives in the Return Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A handler that completes with something other than 200 says so in its return
type, not at runtime. ``http::Created<T>`` declares 201 and ``http::Accepted<T>``
declares 202; both are transparent wrappers whose payload is ``T``:

.. code-block:: cpp

   reg.post<dto::TriggerCreateRequest, http::Created<dto::Trigger>>(
         "/{entity}/triggers",
         [](http::TypedRequest, dto::TriggerCreateRequest)
             -> http::Result<http::Created<dto::Trigger>> {
           dto::Trigger t;
           return http::Created<dto::Trigger>{std::move(t)};
         });

The registry reads ``http::dto_alternate_status<TResponse>`` for the status and
``http::status_payload_t<TResponse>`` for everything else - the schema ``$ref``,
the ``has_dto_shape_v`` assertion and the body writer. An unwrapped ``TResponse``
is its own payload, so a plain DTO still means 200 and ``http::NoContent`` still
means 204. ``http::Accepted<http::NoContent>`` is the shape for an accepted
asynchronous transition that sends no body (202, empty).

This is what keeps the document honest: the declared status and the status on
the wire come from one type, so they cannot disagree. Writing the status at the
call site instead - ``.response(201, ...)`` beside a handler that returns 200 -
is what previously let 45 operations advertise a success status their handler
could never emit.

``ResponseAttachments`` remains the channel for everything that is *not* the
status: extra headers on the success response, and the rare runtime status
override. The pair-returning overloads carry it alongside the (possibly
wrapped) response:

.. code-block:: cpp

   reg.post<dto::Req, http::Created<dto::Resp>>(
         "/...",
         [](http::TypedRequest, dto::Req)
             -> http::Result<std::pair<http::Created<dto::Resp>, http::ResponseAttachments>> {
           dto::Resp r;
           http::ResponseAttachments att;
           att.with_header("Location", "/resources/123");
           return std::make_pair(http::Created<dto::Resp>{std::move(r)}, std::move(att));
         });

When the attachments carry no ``status_override``, the framework falls back to
``dto_alternate_status<TResponse>`` - never to a literal 200/204 - so wrapping a
paired response is enough to move both the wire status and the declared one.

The framework writes the response body via
``JsonWriter<status_payload_t<TResponse>>``, applies the attachments, and renders
any error branch via the route's configured ``ErrorRenderer``
(``kSovdGenericError`` by default; the ``/auth/*`` routes opt into
``kOAuth2Error`` to emit the RFC 6749 wire shape).

Type-System Guarantees
~~~~~~~~~~~~~~~~~~~~~~

Each typed overload carries a ``static_assert(dto::has_dto_shape_v<T>)``
gate, so any non-DTO type passed as ``TResponse`` or ``TBody`` rejects at
compile time with a contract-aware diagnostic. ``has_dto_shape_v<T>`` is true
when either ``is_dto_v<T>`` (a regular field-walking DTO) or
``is_opaque_dto_v<T>`` (a hand-written opaque DTO envelope) is true; the
``NoContent`` marker is the third accepted shape and triggers an empty-body
branch in ``write_success_body``. The gate is applied to
``status_payload_t<TResponse>``, so ``Created<T>`` / ``Accepted<T>`` are
accepted exactly when ``T`` is - the wrappers deliberately have no
``dto_fields`` / ``dto_name`` specialization of their own.

The OpenAPI status **and** schema slot for every typed route is wired
automatically from ``TResponse`` and ``TBody`` (and from the alternates in
``post_alternates<TBody, TAlt...>`` / ``del_alternates<TAlt...>``). The
registry calls
``RouteEntry::response<status_payload_t<TResponse>>(dto_alternate_status<TResponse>::value, ...)``
/ ``RouteEntry::request_body<TB>("")`` so neither the status nor the schema can
drift from the handler: the same C++ type names all three.

Hand-attached ``.response(...)`` calls are therefore reserved for statuses the
*framework cannot see* - error statuses beyond the blanket 400/404/500 set, and
the ``request_body(...)`` annotation the rare body-less typed ``post`` / ``put``
overloads need when they parse non-JSON bodies (form-urlencoded auth). Never
hand-attach a 2xx: it restates something the return type already decides, and
if the two disagree the route publishes both.

To author the prose a generated client shows for a success response, use
``.success_description("Trigger created")``. It rewrites the description of the
already-derived 2xx and touches neither the status nor the schema. Without it
the framework publishes a status-appropriate default ("Created", "Accepted",
"No content", "Successful response").

Routes whose handler genuinely returns a ``std::variant`` - the
``post_alternates`` / ``del_alternates`` helpers - legitimately declare more
than one 2xx. Those helpers call ``RouteEntry::mark_alternates()`` themselves,
which publishes the ``x-medkit-alternates: true`` operation extension, so the
document contract test can tell a real variant from a route that declares a
status it cannot return. Nothing else may set that marker.

Two further ``RouteEntry`` knobs shape the published response set:

- ``errors({409, 423})`` - declare error statuses this route can emit beyond
  the blanket set; each is rendered as a ``GenericError`` response ``$ref``.
  Statuses below 400 are ignored and reported by ``validate_completeness()``,
  because a success status belongs in the return type, not here.
- ``only_status(code, desc)`` - this route has exactly one outcome. Clears every
  other response and suppresses the blanket 400/404/500 injection. The
  auth 401/403 refs stay when authentication is enabled: they come from the
  middleware ahead of the handler and are reachable on every route.
  A ``code >= 400`` is published with the ``GenericError`` schema attached,
  because that is what the handler puts on the wire; publishing it bare would
  describe a bodyless response a generated client then receives JSON into.
  ``only_status`` is not sticky with respect to ``errors()``: a call placed
  *after* it re-declares those statuses, so state the single outcome last. It
  *is* safe with respect to ``gated_on()`` in either order - a live gate is a
  second reachable outcome, so ``only_status`` re-declares the gate's status
  rather than dropping it.
- ``gated_on(available, unavailable)`` - the route's backing feature can be
  absent. ``available`` is re-evaluated per request (a manager can appear after
  registration), and when it is false the framework answers with
  ``unavailable`` rendered through this route's ``ErrorRenderer``. The call
  also declares ``unavailable.http_status`` via ``errors()``, which is the
  point: a gate written as an inline ``if (!handlers_) return
  tl::unexpected(...)`` inside the handler lambda is invisible to the document
  generator, so the published operation omitted the 501 it answers with in
  practice.

  The guard runs *inside* the typed wrapper, at the same place the inline
  ``if`` used to sit - after the request body has been parsed. A malformed
  payload sent to a gated-off route therefore still answers 400, not the gate's
  status.

  Feature gates the registration cannot see - a handler that answers 501
  because its own backend is unconfigured, e.g. ``LockHandlers`` without a lock
  manager - are declared with plain ``errors({501})`` until a handler-level
  seam exists.

Escape Hatches
--------------

Not every payload can be expressed as a typed DTO. Two orthogonal categories
of escape hatches exist: in-body dynamic payloads (``opaque_object`` fields
inside a DTO, see below) and dedicated non-DTO route helpers.

Named Route Escape Hatches
~~~~~~~~~~~~~~~~~~~~~~~~~~

The typed ``RouteRegistry`` exposes a closed set of named escape hatches for
routes whose wire shape is not JSON-and-only-JSON. Each helper produces a
typed handler with a purpose-built response type, so even non-DTO routes
remain compile-time-checked at their boundary.

- ``reg.sse(path, factory)`` - registers a Server-Sent Events route. The
  factory returns a ``Result<http::SseStream>`` whose ``next_event``
  callback the framework drives via cpp-httplib's chunked content provider.
  Used by the fault SSE stream and by cyclic-subscription event streams.
- ``reg.binary_download(path, handler)`` - registers a range-aware binary
  download. The handler returns a ``Result<http::BinaryResponse>`` carrying
  ``provider``, ``content_type``, ``filename``, ``supports_ranges``, and
  ``total_size``; the framework wires ``provider`` into cpp-httplib's
  range-aware content-provider machinery so partial-content fetches work
  without manual ``Content-Range`` plumbing.
- ``reg.multipart_upload<TResponse>(path, handler)`` - registers a
  ``multipart/form-data`` upload. The handler receives ``http::MultipartBody``
  (already parsed by cpp-httplib) and returns
  ``Result<std::pair<TResponse, http::ResponseAttachments>>``. Uploads declare
  201 through ``TResponse`` (``http::Created<dto::BulkDataDescriptor>``) and use
  the attachments only for the ``Location`` header. Used by bulk-data POST/PUT.
- ``reg.static_asset(path, handler)`` - serves bytes already in memory
  (Swagger UI bundles, embedded HTML/JS/CSS) as
  ``Result<http::StaticAsset>`` carrying ``bytes``, ``content_type``, and
  per-response ``headers`` (``Cache-Control``, ``ETag``).
- ``reg.docs_endpoint(path, handler)`` - registers the OpenAPI JSON endpoint
  at the given path. The handler returns ``Result<nlohmann::json>``; this is
  the only built-in route allowed to use raw ``nlohmann::json`` as
  ``TResponse``, because the body is the spec itself.
- ``reg.docs_subtree(regex, handler)`` - catch-all for the Swagger UI subtree
  (asset paths without a fixed shape). Hidden from the OpenAPI output so it
  does not pollute the generated spec.
- ``reg.post_alternates<TBody, TAlt...>(path, handler)`` /
  ``reg.del_alternates<TAlt...>(path, handler)`` - register multi-shape
  responses. The active variant alternative is dispatched to its
  ``dto_alternate_status<T>::value`` (default 200; specialize per type, for
  example ``NoContent`` -> 204, ``Created<T>`` -> 201, ``Accepted<T>`` -> 202).
  The published spec lists every alternative under its own status code, and the
  wire status is picked by the active alternative at call time. Both helpers
  call ``mark_alternates()``, so these are the only operations allowed to carry
  more than one 2xx.

Plugin-Owned Routes (``PluginContext::register_route()``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Routes contributed by ``GatewayPlugin`` subclasses bypass the typed router
entirely and run a ``void(PluginRequest, PluginResponse)`` handler that the
plugin owns. ``PluginResponse`` is a thin shim over a cpp-httplib response
whose ``send_json`` / ``send_error`` methods route through the same internal
``http::detail::write_json_body`` primitive used by the typed router, so
plugin responses remain wire-format-identical to built-in responses (same
SOVD ``GenericError`` shape, same ``Content-Type`` handling). The plugin
ABI is locked by ``test_plugin_abi_conformance``; nothing here changes for
out-of-tree plugins.

Opaque Object Policy
--------------------

Some DTO fields carry an entire JSON object whose internal shape is decided
at runtime by an upstream component the gateway cannot introspect at compile
time. The ``opaque_object("key", &T::field)`` descriptor in
``dto/contract.hpp`` binds such a field to a ``nlohmann::json`` member:

- **JsonWriter** writes the member as-is (no introspection, no schema check).
- **JsonReader** accepts any JSON object value, rejects scalars / arrays /
  null with a ``FieldError``; an absent field leaves the member at its
  default (empty object).
- **SchemaWriter** emits
  ``{type: object, additionalProperties: true, x-medkit-opaque: true}`` and
  marks the field required (opaque fields are not wrapped in ``std::optional``).

Use ``opaque_object`` for fields whose runtime shape depends on context, not
for fields the gateway could describe but chose not to. Concrete cases in
the codebase:

- **Live ROS 2 message payloads** - topic samples returned by data handlers
  carry whatever fields the actual message type declares at runtime.
- **Plugin-defined fault envelopes** - ``FaultListResult`` /
  ``FaultDetailResult`` / ``FaultClearResult`` returned by the typed
  ``FaultProvider`` ABI wrap a ``content`` opaque object so UDS, OPC-UA, and
  vendor backends can each emit their own per-item shape.
- **Action results / service responses** - ``Operation*`` execution
  payloads whose field set is determined by the ROS 2 service / action type
  bound to the operation, not by the gateway.
- **OpenAPI spec body** - the ``/docs`` endpoint returns the spec itself,
  declared via ``reg.docs_endpoint(path, Result<json>(...))`` (a route-level
  escape hatch, not an opaque DTO field).

Fields backed by ``std::optional<nlohmann::json>`` rather than
``opaque_object`` (notably ``extended_data_records`` / ``snapshots`` on
``FaultEnvironmentData``) follow the same rule: pass the JSON through
verbatim because the fault reporter plugin owns the shape. The opaque DTO
marker (``is_opaque_dto_v<T> = true``) plays the analogous role at the
envelope level: it tells the framework "this whole DTO has a hand-written
JsonWriter / JsonReader / SchemaWriter trio because its shape is opaque",
which is what the typed Provider envelopes use.

Provider ABI: Typed-Only Policy
-------------------------------

Per-entity provider interfaces (``FaultProvider``, ``DataProvider``,
``OperationProvider``) and the singleton ``UpdateProvider`` all return typed
DTOs. None of them return raw ``tl::expected<nlohmann::json, ErrorInfo>``
any more.

The typed envelopes - ``FaultListResult``, ``FaultDetailResult``,
``FaultClearResult``, the corresponding ``Data*Result`` and ``Operation*Result``
shapes, and ``UpdateProvider::get_update``'s typed return - wrap an opaque
``content`` payload so the wire bytes are byte-identical to the pre-typed
ABI: ``JsonWriter`` emits the ``content`` object verbatim, ``SchemaWriter``
publishes ``x-medkit-opaque: true``, and JsonReader accepts any JSON object
on round-trip. This keeps backend-specific shapes (UDS DTC records, OPC-UA
alarm metadata, vendor extensions) flowing through unchanged while pinning
the envelope itself to a single typed contract.

Commercial plugins (UDS, OPC-UA, Uptane OTA, ...) implement the typed
interface directly. Out-of-tree plugins that previously returned raw
``nlohmann::json`` must wrap their response in the matching envelope type;
the conversion is mechanical (``Result.content = std::move(json_payload)``).

Header Purity: No httplib Across the Plugin Boundary
----------------------------------------------------

Plugin-facing public headers - the provider interfaces
(``core/providers/*.hpp``), the plugin base headers a ``GatewayPlugin``
subclass includes (``core/plugins/gateway_plugin.hpp``,
``plugin_context.hpp``, ``plugin_http_types.hpp``,
``plugins/ros_plugin_context.hpp``), and every DTO header they pull in -
MUST NOT depend on ``<httplib.h>``. cpp-httplib is a gateway-internal
implementation detail. Across the ``.so`` boundary plugins exchange only
``nlohmann::json``, typed ``dto::`` structs, ``tl::expected``, and the opaque
``PluginRequest`` / ``PluginResponse`` shim. Because no httplib type ever
crosses that boundary, the gateway and its plugins do not need to share an
httplib version: a plugin built against the installed gateway (the ROS
build-farm / Docker topology, where the gateway's vendored httplib is not on
the include path) still compiles.

The httplib-free handler-result vocabulary - ``Result``, ``NoContent``,
``Forwarded``, ``ValidatorResult``, ``ResponseAttachments`` - lives in
``http/handler_result.hpp``. Only ``http/typed_router.hpp`` (which owns
``TypedRequest`` and the raw-response escape hatch) and the handler-internal
headers downstream of it touch ``<httplib.h>``; ``typed_router.hpp``
re-exports the ``handler_result.hpp`` vocabulary so existing includers keep
working without pulling httplib transitively.

The invariant is enforced by the ``gateway_plugin_header_purity`` ctest
(``scripts/check_headers_httplib_free.sh``, ``linter`` label), which runs a
preprocessor-only scan (``g++ -M -MG``) over the plugin-facing surface and
fails on any transitive ``httplib.h`` dependency, and by the pre-push hook of
the same name. The build-farm topology (installed gateway, no vendored
httplib on the include path) is reproduced locally by
``scripts/check_isolated_build.sh``.

Fan-Out Observability
---------------------

Aggregating collection routes call ``http::fan_out_collection<T>(agg, req)``
to query peer gateways and merge their ``items`` arrays. The helper returns
a typed ``FanOutResult<T>``:

.. code-block:: cpp

   template <class T>
   struct FanOutResult {
     std::vector<T> items;              // parsed peer items
     bool partial{false};               // at least one peer failed
     std::vector<std::string> failed_peers;
     std::vector<dto::DroppedItem> dropped_items;  // items that failed JsonReader<T>
   };

Each peer item is decoded via ``dto::JsonReader<T>``. Items that fail
validation are removed from ``items`` and recorded in ``dropped_items`` with
the JsonReader error message plus a best-effort ``source_id`` extracted from
the item's ``id`` / ``name`` / ``fault_id`` / ``data_id`` / ``operation_id``
field. A ``WARN`` log fires for each drop, naming ``dto_name<T>`` and the
reason. The peer URL on each ``DroppedItem`` is left empty in this commit
because ``AggregationManager::fan_out_get`` coalesces all peer responses
into a single merged array without per-item provenance; per-peer attribution
is left for a future enrichment of the aggregation manager.

Handlers surface drops on the wire via the ``peer_dropped_items`` field on
every collection-level x-medkit DTO (``XMedkitCollection``,
``FaultListXMedkit``, ``FaultListAggXMedkit``, ``DataListXMedkit``,
``LogListXMedkit``, ...). Previously, malformed peer items disappeared
silently into the merged ``items`` array; now they show up in
``x-medkit.peer_dropped_items`` so clients (and fleet operators) can detect
drift between heterogeneous gateways. The legacy ``merge_peer_items`` helper
(raw-JSON mutation) is still in use on routes whose merged items are
dynamic-shaped (the fault aggregation routes, ``GET /health``), where the
items are not addressable by a single ``T`` for ``JsonReader<T>``; typed
collection routes (data, operations, config, logs) call
``fan_out_collection<T>`` directly.

OpenAPI Generation Pipeline
---------------------------

The published ``openapi.json`` is assembled mechanically from two sources:

- ``components/schemas`` is exactly ``collect_component_schemas<AllDtos>()`` -
  one entry per DTO listed in ``dto/registry.hpp``, with no hand-written
  survivors merged in.
- ``paths`` is ``RouteRegistry::to_openapi_paths()``: every typed route
  contributes a path item with ``$ref`` entries auto-derived from its
  ``TResponse`` / ``TBody`` template parameters plus any tags, summary,
  description, ``operation_id``, parameter, or extra-status metadata pinned
  on the route via the fluent ``RouteEntry`` builder. The per-topic /
  per-service / per-action routes for genuinely dynamic ROS 2 payloads carry an
  *inline* schema built by ``SchemaBuilder``'s ``from_ros_msg`` /
  ``from_ros_srv_request`` / ``from_ros_srv_response`` / ``binary_schema`` /
  ``generic_object_schema`` factories (these feed path operations, not
  ``components/schemas``).

``OpenApiSpecBuilder::build()`` then assembles ``info`` / ``servers`` /
``tags`` / ``security`` around those two compiled blocks. There are no
hand-written ``paths`` items in the published spec, and no hand-written schema
blocks in ``components/schemas``. Adding a route or a DTO field updates the
spec on the next process start with no schema-side edit.

Optional fields are now emitted as ``anyOf: [<inner>, {type: "null"}]``
(OpenAPI 3.1 idiom) so generated clients see ``T | null`` rather than
``T | undefined``. That matches the wire reality of the gateway: optional
fields are either present-with-value or absent, never explicit ``null``;
but the schema also accepts ``null`` so clients that prefer to emit a
nullable value on the wire round-trip cleanly through ``JsonReader``.

Adding a New DTO
----------------

Follow these four steps when introducing a new typed payload:

1. **Define the struct and its descriptor** in the appropriate domain header
   under ``include/ros2_medkit_gateway/dto/``. Add a ``dto_fields<T>``
   specialization and a ``dto_name<T>`` specialization in the same header.

   .. code-block:: cpp

      // In dto/my_domain.hpp
      struct MyResponse {
        std::string id;
        std::optional<std::string> label;
        int64_t count{0};
      };

      template <>
      inline constexpr auto dto_fields<MyResponse> = std::make_tuple(
          field("id",    &MyResponse::id),
          field("label", &MyResponse::label),
          field("count", &MyResponse::count));

      template <>
      inline constexpr std::string_view dto_name<MyResponse> = "MyResponse";

2. **Register in AllDtos** by adding ``MyResponse`` to the tuple in
   ``include/ros2_medkit_gateway/dto/registry.hpp``. Also add the include
   for ``dto/my_domain.hpp`` at the top of ``registry.hpp``.

3. **Use in the handler.** Handlers never touch ``httplib::Response`` - they
   return ``http::Result<TResponse>`` and the framework writes the body.
   Entity validation is also typed: ``validate_entity_for_route`` returns
   ``http::ValidatorResult<EntityInfo>``; the helper ``flatten_validator_error``
   collapses the local-error and ``Forwarded`` branches into a single
   ``ErrorInfo`` (the ``Forwarded`` branch becomes the framework-internal
   sentinel that the RouteRegistry wrapper recognises and skips error
   rendering for):

   .. code-block:: cpp

      // GET handler - typed response
      http::Result<dto::MyResponse> MyHandlers::handle_get(http::TypedRequest req) {
        auto entity = ctx_.validate_entity_for_route(req, req.path_param(0));
        if (!entity) {
          return tl::make_unexpected(flatten_validator_error(entity.error()));
        }

        dto::MyResponse resp;
        resp.id    = entity->id;
        resp.label = "example";
        resp.count = 42;
        return resp;
      }

      // POST handler - typed request body (parsed by the framework before
      // the handler runs; the handler receives an already-validated TBody).
      http::Result<dto::MyResponse> MyHandlers::handle_post(
          http::TypedRequest req, dto::MyCreateRequest body) {
        // use body.field_name directly
        dto::MyResponse resp;
        // ... build response ...
        return resp;
      }

4. **Register the route via the typed RouteRegistry.** Because ``MyResponse``
   is now in ``AllDtos``, ``collect_component_schemas()`` automatically
   includes its schema in the ``/docs`` response, and the typed overload
   wires the ``$ref`` into the path item:

   - For **built-in gateway routes**, register in
     ``rest_server.cpp::setup_routes()`` via ``reg.get<T>`` /
     ``reg.post<TBody, T>`` / etc. The framework derives the
     ``response<T>(200, "")`` and ``request_body<TBody>("")`` slots from the
     template parameters; the call site only adds tags, summary, extra
     status codes, and ``operation_id``:

     .. code-block:: cpp

        reg.get<dto::MyResponse>(
               "/my-entity/{id}/my-resource",
               [this](http::TypedRequest req) -> http::Result<dto::MyResponse> {
                 /* handler */
               })
            .tag("MyTag")
            .summary("Get my resource")
            .operation_id("getMyResource")
            .response(404, "Resource not found");  // extra non-200 status

   - For **plugin-contributed routes**, use the ``RouteDescriptionBuilder``
     API in ``core/openapi/route_descriptions.hpp``. Plugin routes do not go
     through the typed registry (see Plugin-Owned Routes above), so the
     schema wire-up is explicit.

Adding a New Endpoint (Full Checklist)
--------------------------------------

A new endpoint with a typed payload follows the standard gateway handler
checklist plus the DTO steps above:

1. Define DTO struct + ``dto_fields`` + ``dto_name`` in a domain header.
2. Add to ``AllDtos`` in ``registry.hpp``.
3. Implement handler in ``src/http/handlers/`` as a typed function returning
   ``http::Result<TResponse>``.
4. Register route in ``rest_server.cpp::setup_routes()`` via
   ``reg.get<T>`` / ``reg.post<TBody, T>`` / ``reg.del<T>`` / the matching
   alternates or escape-hatch helper. Use the dual-path pattern for entity
   types that share the same route shape.
5. Update ``handle_root`` endpoint list in ``health_handlers.cpp`` to mirror
   the new route.
6. Add URI field to entity detail response if the new route is a resource
   collection.
7. Write a unit test using ``JsonWriter<T>::write()`` and
   ``JsonReader<T>::read()`` directly - no HTTP server needed.
8. Write an integration test that calls the live endpoint.

Collection<T, XMedkitT> Parametrisation
---------------------------------------

The generic ``Collection<T, XMedkitT>`` list wrapper is parameterised over
both the item type and the collection-level ``x-medkit`` shape. Entity list
endpoints (areas, components, apps, functions) use the default
``XMedkitCollection`` x-medkit; the domain collection endpoints specialise
``XMedkitT`` to their richer per-domain shape (``FaultListXMedkit``,
``FaultListAggXMedkit``, ``ConfigListXMedkit``, ``DataListXMedkit``,
``LogListXMedkit``). For the config and log list routes the published schema
references the actual collection x-medkit struct directly, so generated clients
see the exact aggregation, peer-provenance, and ``peer_dropped_items`` fields
that appear on the wire.

The fault and data list routes are the exception: they publish the opaque
``FaultListResult`` / ``DataListResult`` envelopes rather than the typed
``Collection<...>`` schema, because plugin-owned entities can return
vendor-specific per-item shapes that the typed item schema cannot describe.
The data list handler still *builds* a typed
``Collection<DataItem, DataListXMedkit>`` for runtime (ROS 2) entities and
serializes it into the envelope (so the wire shape - including
``peer_dropped_items`` - is unchanged), but the plugin branch passes the
provider's free-form payload through verbatim. See "Opaque Object Policy" and
the "Provider ABI" section above.

Key Files
---------

``include/ros2_medkit_gateway/dto/contract.hpp``
    ``Field``, ``dto_fields``, ``dto_name``, ``is_dto_v``,
    ``for_each_field`` - the contract primitives.

``include/ros2_medkit_gateway/dto/json_writer.hpp``
    ``JsonWriter<T>`` - struct to JSON serialization.

``include/ros2_medkit_gateway/dto/schema_writer.hpp``
    ``SchemaWriter<T>`` and ``schema_of<U>`` - type to OpenAPI schema.

``include/ros2_medkit_gateway/dto/json_reader.hpp``
    ``JsonReader<T>`` and ``FieldError`` - JSON to struct with validation.

``include/ros2_medkit_gateway/dto/registry.hpp``
    ``AllDtos`` tuple and ``collect_component_schemas()``.

``src/openapi/route_registry.hpp``
    ``RouteRegistry`` typed overloads (``get<T>`` / ``post<TBody, T>`` /
    ``del<T>`` / alternates) and named escape hatches (``sse`` /
    ``binary_download`` / ``multipart_upload<T>`` / ``static_asset`` /
    ``docs_endpoint`` / ``docs_subtree``), plus the wrapper-closure
    template implementations.

``include/ros2_medkit_gateway/http/response_types.hpp``
    ``SseStream``, ``BinaryResponse``, ``MultipartBody``, ``StaticAsset`` -
    the typed response shapes consumed by the named escape hatches.

``include/ros2_medkit_gateway/http/handlers/handler_context.hpp``
    ``HandlerContext::validate_entity_for_route`` and the typed validator
    surface (``ValidatorResult<T>``, ``flatten_validator_error``).

Domain headers
    ``dto/errors.hpp``, ``dto/entities.hpp``, ``dto/faults.hpp``,
    ``dto/operations.hpp``, ``dto/config.hpp``, ``dto/locks.hpp``,
    ``dto/triggers.hpp``, ``dto/logs.hpp``, ``dto/scripts.hpp``,
    ``dto/updates.hpp``, ``dto/auth.hpp``, ``dto/health.hpp``,
    ``dto/bulkdata.hpp``, ``dto/cyclic_subscriptions.hpp``,
    ``dto/data.hpp``, ``dto/x_medkit.hpp`` - per-domain struct definitions
    with co-located ``dto_fields`` and ``dto_name`` specializations.
    ``dto/errors.hpp`` holds ``GenericError``, the error response DTO used
    by every endpoint.

``dto/enums.hpp``
    Enum-vocabulary header. Contains the ``constexpr string_view`` arrays
    (``kFaultSeverityLabelValues``, ``kOperationExecutionStatusValues``, etc.)
    referenced by ``field_enum()`` descriptors in the domain headers. Does
    not define any DTO structs.
