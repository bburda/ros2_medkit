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

The same argument governs everything the document says about a *route*, not
just about a payload, and it does not always reach the same strength.
:doc:`openapi_derivation` states the rule that produced the mechanisms below -
if a fact can be derived from the handler it must not be declared separately,
and where it cannot the declaration lives at a seam that also does the work -
and records which enforcement tier each mechanism actually reaches. Read it
first if the question is "what stops this from going stale?" rather than "how
do I use it?".

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
           + binary_download(path, handler, media_types)
           + multipart_upload<T>(path, handler)
           + static_asset(path, handler)
           + docs_endpoint(path, handler)
           + docs_subtree(openapi_path, regex, handler)
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
     FieldConstraints constraints;  // minimum / maximum / maxLength / pattern / format
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

Schema Constraints (``FieldConstraints``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A field can publish JSON Schema keywords its C++ type does not imply. Every
member of ``FieldConstraints`` is unset by default, so a ``field()`` call that
names none emits what it always did. C++17 has no designated initialisers, so
a call site spells the unset members out:

.. code-block:: cpp

   field("max_entries", &LogConfiguration::max_entries,
         "Ceiling on how many entries one GET answers with...",
         FieldConstraints{/*minimum=*/1.0, /*maximum=*/10000.0, {}, {}, {}})

**Only declare a bound the handler enforces unconditionally.** A ceiling the
gateway reads from a ROS parameter is deployment configuration, and publishing
it as ``maximum`` makes the document wrong on any deployment that changed it -
those belong in the ``description``. ``AcquireLockRequest.lock_expiration`` is
the worked example: ``minimum: 1`` is a constraint because ``LockManager``
always rejects a non-positive expiration, while the 3600 s ceiling is
``locking.default_max_expiration`` and is described rather than declared.

Numeric width is derived, not declared: ``schema_of`` emits ``format: int64``
for a **signed** 64-bit integral. Signedness is part of the test because
``sizeof(U) == 8`` alone also matches ``uint64_t`` and ``std::size_t``, whose
upper half the format excludes. Set ``FieldConstraints::format`` only for a
string format the type cannot imply, such as ``date-time``.

Constraints and ``enum`` are attached through one lambda in
``derived_object_schema``, so their placement cannot drift: an optional member
renders as ``{anyOf: [<inner>, {type: null}]}`` and every validation keyword
goes on the non-null branch ``anyOf[0]``, because one written at the property
level would apply to the null branch too and reject the ``null`` that
``anyOf`` advertises. Required members have no ``anyOf`` and take the keyword
at the top level. The ``description`` is the exception and stays at the
property level either way: it describes the field, not one branch of its
schema.

**An enum is not always the right way to publish a vocabulary.** Where the
handler answers an unrecognised value with an error richer than "value not in
allowed set" - ``ExecutionUpdateRequest.capability`` names
``supported_capabilities`` for the backend in front of the caller, and
``LogConfiguration.severity_filter``'s 400 lists the accepted severities - a
schema-level ``enum`` makes ``JsonReader`` reject the request first and
replaces that error with a generic body-validation failure. Both fields
therefore publish their vocabulary as prose. Use ``field_enum`` where the
parser rejecting the value is the whole of the validation.

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
``generic_object_schema`` - are no longer part of the
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

.. _success-status-from-the-return-type:

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
           att.with_location(api_path("/resources/123"));
           return std::make_pair(http::Created<dto::Resp>{std::move(r)}, std::move(att));
         });

When the attachments carry no ``status_override``, the framework falls back to
``dto_alternate_status<TResponse>`` - never to a literal 200/204 - so wrapping a
paired response is enough to move both the wire status and the declared one.

``with_location(uri)`` is the typed form of the ``Location`` attachment, and it
is not merely sugar. The registry declares a ``Location`` response header on
*every* derived 201 and 202, because ``Created<T>`` / ``Accepted<T>`` already
told it the status - so a handler behind one of those return types that does
not call ``with_location`` publishes a header it never sends.

The obligation is enforced where it can be. The non-attachments overloads -
``get`` / ``post`` / ``put`` / ``patch`` / ``del<TResponse>`` and the
non-attachments ``post_alternates`` / ``del_alternates`` - give a handler no
channel for a header at all, so they ``static_assert`` against a ``TResponse``
(or a variant alternate) whose status is 201 or 202. A route that would
advertise ``Location`` and be structurally unable to send it does not compile.
What the type system cannot see is a *pair-returning* handler that simply
forgets the call; that half is covered by the document contract test, which
asserts every declared 201/202 carries the header and that a real 201 puts it
on the wire. ``uri`` is the
absolute, API-prefixed path form every ``href`` in the document uses: build it
with ``api_path(...)``, or pass ``req.path() + "/" + id`` when the new resource
is a child of the request path (``TypedRequest::path()`` is already prefixed).
Writing the ``"/api/v1/"`` literal by hand is what let three spellings of the
same URI accumulate across the handlers.

The framework writes the response body via
``JsonWriter<status_payload_t<TResponse>>``, applies the attachments, and renders
any error branch via the route's configured ``ErrorRenderer``
(``kSovdGenericError`` by default; the ``/auth/*`` routes opt into
``kOAuth2Error`` to emit the RFC 6749 wire shape).

The document reads the same field. ``to_openapi_paths()`` selects between the
``GenericError`` and ``OAuth2Error`` component responses from
``route.error_renderer_``, so the declared error body and the rendered one come
from one fact rather than from a hand-written list that can go stale. Three
statuses deliberately bypass that selection, because they are not produced by
the route's renderer at all:

- **416** is written by cpp-httplib before routing, with no body, and
  ``RESTServer::setup_global_error_handlers`` fills it with a ``GenericError``.
  That happens on the ``/auth/*`` routes too, so 416 is declared as a
  ``GenericError`` everywhere.
- **401 / 403** come from ``AuthMiddleware``, also ahead of any handler. Both
  serialise ``AuthErrorResponse::to_json()`` = ``{error, error_description}``,
  which is the RFC 6749 shape - so the shared ``Unauthorized`` and ``Forbidden``
  component responses carry the ``OAuth2Error`` schema on every route, not just
  the auth ones. They keep their own components because only there can their
  ``WWW-Authenticate`` header live: no handler return type produces it.

``RateLimited`` (429) stays a ``GenericError``: the limiter emits the SOVD
shape, unlike the two auth statuses beside it.

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

Its sibling ``.success_schema<T>()`` rewrites the *schema* of the already-derived
2xx, again leaving the status and the description alone. It exists for the few
routes whose C++ return type is a pass-through envelope
(``FaultListResult`` and friends) purely so a backend's JSON survives
byte-for-byte, while the wire shape on that particular route is nonetheless
fixed. ``GET /faults`` is the shipped example: it returns ``FaultListResult`` so
the FaultManager's items and the peers' merged items are never re-parsed, and it
publishes ``FaultList`` because - unlike the per-entity list - it has no
plugin-delegation branch and can only ever answer with that shape. Using it on a
route that *does* delegate to a plugin would promise fields no plugin sends.
A call that finds no single declared 2xx is dropped and reported by
``validate_completeness()``.

Routes whose handler genuinely returns a ``std::variant`` - the
``post_alternates`` / ``del_alternates`` helpers - legitimately declare more
than one 2xx. Those helpers call ``RouteEntry::mark_alternates()`` themselves,
which publishes the ``x-medkit-alternates: true`` operation extension, so the
document contract test can tell a real variant from a route that declares a
status it cannot return. Nothing else may set that marker.

There is exactly one other way a second 2xx is reachable, and it carries its
own marker rather than reusing that one. ``reg.binary_download`` handlers never
assign a status, so cpp-httplib answers 200 or **206 Partial Content**
depending on whether the request carried a ``Range`` at all - it also fills in
``Content-Range``. Note "at all", not "a satisfiable one": a ``Range`` that
parses but asks for bytes past the end of the file still yields 206, and one
that does not parse is rejected with 416 before routing, so by the time this
decision is made every surviving ``Range`` is a parseable one. The helper therefore declares both statuses and
calls ``RouteEntry::mark_partial_content()``, publishing
``x-medkit-partial-content: true``. Two markers, not one: there the handler
chooses between variant members, here the handler returns one thing and the
HTTP layer decides how to frame it. A single marker covering both would let the
contract test wave through a route that declares a status it can never return.
Nothing outside ``binary_download`` may set it.

Media types
~~~~~~~~~~~

A response declared through the JSON overloads is published under
``application/json``, with the content entry omitted entirely when the schema
is empty - that is what keeps a 204 body-less rather than giving it a
``schema: null``.

Non-JSON bodies use the four-argument overload
``response(status, desc, schema, content_types)``. Each media type becomes its
own ``content`` entry holding an **empty** Media Type Object. The missing
schema is the declaration, not an omission, for two independent reasons:

- ``{"type": "string", "format": "binary"}`` is an OpenAPI 3.0 idiom. 3.1
  aligned with JSON Schema 2020-12, where ``format: binary`` carries no
  meaning and ``type: string`` actively misdescribes raw bytes.
- The SSE families emit three different frame shapes, so any single schema
  would be wrong for two of them.

The ``schema`` argument exists only for signature symmetry and must be empty; a
caller that passes one has it dropped rather than attached to a media type it
may not describe, and the miscall is reported by ``validate_completeness()``.

Both completeness gates - ``validate_completeness()`` and the served-document
check in ``test_health::test_docs_spec_completeness`` - treat a 2xx carrying a
non-JSON media type as complete without a schema. That replaced an older rule
that exempted a route when its *summary* contained "SSE" or "stream", so the
exemption now follows what a route declares rather than what it is named.

**Open media-type sets.** Where the served type is not enumerable the
declaration says so, by listing the derivable types *and* ``*/*``. The
bulk-data download is the case in the tree:
``BulkDataHandlers::download_media_types()`` names the three types
``get_rosbag_mimetype()`` can return, then ``*/*`` for the store-backed
categories, which serve back whatever media type the uploading client put on
its multipart part. Declaring only the concrete types would under-declare the
route; declaring only ``*/*`` would throw away the half that is derivable.

That declaration is checked against a run, not a review. The integration suite
downloads real artifacts and asserts the served ``Content-Type`` against the
document, distinguishing the two halves: a rosbag type must match a **named**
content key (matching only via ``*/*`` fails), while a client-supplied type may
match via the catch-all, which the test also asserts is present. Both
directions have been shown to fail on an injected defect.

Further ``RouteEntry`` knobs shape the published operation:

- ``errors({409, 423})`` - declare error statuses this route can emit beyond
  the blanket set; each is rendered as a ``GenericError`` response ``$ref``.
  Statuses below 400 are ignored and reported by ``validate_completeness()``,
  because a success status belongs in the return type, not here. This is the
  one knob whose completeness is checked against a *run* rather than a review -
  see `Emitted-status recorder`_ below.
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
- ``response_header(status, {name, description, schema})`` - declare a header
  this route sets on an **already declared** status. A header is a property of
  a response, so a call aimed at a status no response declares is dropped and
  reported by ``validate_completeness()`` rather than minting a
  description-less response object for a status the handler cannot return
  (a release build compiles ``assert`` out, so a precondition check there would
  be no check at all). Re-declaring the same header name on the same status
  replaces it, which is how a route overrides the framework's automatic
  ``Location`` prose. Most routes never call it: ``Location`` comes from the
  status, and the ``sse`` / ``binary_download`` helpers declare their own
  framework-owned headers (``Cache-Control`` and ``X-Accel-Buffering``;
  ``Content-Disposition`` and ``Accept-Ranges``) next to the code that sets
  them. Declared headers carry no ``required`` flag - OpenAPI response headers
  are optional by definition, which matches headers the gateway sets
  conditionally.
- ``description(text)`` - the behaviour a caller has to know that the field
  names and the summary do not carry: what an omitted filter defaults to, what
  a 202 does and does not promise, what the answer is silently truncated to.
  Required on every operation, and gated:
  ``test_openapi_contract.test.py::test_every_operation_has_a_description``
  fails on any operation that ships with only a ``summary``.
- ``body_example(json)`` - publish a working request body a caller can copy out
  of the document, emitted as
  ``requestBody.content[<primary media type>].examples.default.value``. A
  ``$ref`` names the fields and their types; it does not say that
  ``trigger_condition`` needs a ``condition_type`` key, or that ``interval`` is
  a word rather than a number. Examples live here and nowhere else:
  ``dto_fields<T>`` is ``inline constexpr`` and could hold only a string
  literal per property, and putting one on the schema as well would be a second
  source for one concept. Attaches to the primary body only - the extra
  encodings ``accepts()`` adds are the same payload in another wire format,
  where a JSON example would not parse. Whether the route has a body is read at
  emission, not at the call, so the fluent chain can order the two either way;
  a route with no body at all drops the example and ``validate_completeness()``
  reports it rather than minting a body the route does not take.
  ``test_openapi_contract.test.py::test_non_trivial_request_bodies_carry_an_example``
  pins the App-entity variant of each route family that carries one.
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
- ``lock_guarded()`` - this route takes part in entity locking. One call
  publishes all three halves of that contract: the ``X-Client-Id`` request
  header the handler reads, the 409 it answers when the entity's collection is
  locked by a different client, and an ``x-medkit-lock-guarded: true`` operation
  extension. The header is declared **optional** on purpose - a caller that
  sends none is an anonymous client, which succeeds while nothing is locked and
  is refused once something is; declaring it required would describe a gateway
  that rejects the header-less request outright, which is not what happens.

  Unlike everything else in this section, **this one is declared and not
  derived, and its test only checks half of it.** The header read that decides
  the 409 lives in ``HandlerContext::validate_lock_access``, which 12 handlers
  across 6 files call, and the document is built from the live route table when
  ``/docs`` is served rather than captured at registration time - so a
  registration cannot see through that call, and no accessor on
  ``TypedRequest`` changes that.
  ``test_openapi_contract.test.py::test_lock_guarded_set_matches_the_handlers``
  pins the marked set against ``EXPECTED_LOCK_GUARDED``, a hand-maintained
  literal committed next to the test. That catches the **document** drifting
  away from the list: dropping a ``.lock_guarded()`` from a registration turns
  the suite red. It does **not** catch the list drifting away from the
  handlers - a new route that calls ``validate_lock_access`` and forgets both
  the decorator and the list entry passes every gate. Adding a lock check to a
  handler means editing that list by hand.

  Two companion tests keep the marker from degenerating into a label:
  ``test_lock_guarded_routes_declare_the_contract`` asserts every marked
  operation really publishes the header and the 409, and
  ``test_lock_guarded_route_answers_the_409_it_declares`` drives a real locked
  write through the gateway so the declared status is one the wire returns.

  ``DELETE /faults`` is the deliberate near-miss: it reads ``X-Client-Id`` like
  every lock-guarded write but never answers 409 - it *skips* faults on
  entities locked by somebody else and still answers 204. Nothing on the
  response says which ones were skipped; ``X-Medkit-Local-Only`` on that 204 is
  about aggregated peers, not locks. It therefore declares the header with its
  own prose via ``header_param`` and does not call ``lock_guarded()``; marking
  it would publish a status it cannot return.
- ``fan_out_aware()`` - this route reads the ``X-Medkit-No-Fan-Out`` request
  header, i.e. a client can ask it to answer from this gateway alone instead of
  merging aggregated peers. Carried by the routes whose handlers go through
  ``fan_out_collection`` or ``merge_peer_items``. The declared schema is a bare
  string, not a boolean: the gateway tests ``has_header`` and never reads the
  value, so ``X-Medkit-No-Fan-Out: false`` still suppresses fan-out and a
  boolean schema would promise a generated client the opposite. Also
  hand-applied, with the same caveat as ``lock_guarded()`` above.

Every self-check named above (``errors()`` handed a sub-400 status,
``response_header()`` aimed at an undeclared status, a ``lock_guarded()`` marker
whose 409 a later ``only_status()`` cleared, a route with no tag or no
success schema) reports through ``RouteRegistry::validate_completeness()``, and
``RESTServer::report_route_metadata_issues()`` calls it once at start-up and
logs what it finds. That call is what makes "reported" mean something: before
it existed the issues were collected and discarded outside the unit tests, so
the guarantee was words only.

The report is logged, never fatal. Every issue it can raise is a defect in the
*document*, and a gateway that refused to serve traffic because one route is
missing a summary would trade a documentation bug for an outage. Its job is to
cover the route set a given configuration actually assembled - which feature
gates and plugins make impossible to enumerate in a test - while the OpenAPI
contract suite gates the shape of the document itself.

The report is not merely logged. It emits a summary line unconditionally -
including for a clean route set, because a line that only appears on failure
cannot be asserted on - and
``test_openapi_contract.test.py::test_shipped_route_set_declares_complete_metadata``
waits for that line with a zero error count, on the fixture that turns every
optional feature gate on. That is what makes it a gate rather than a diagnostic
nobody reads.

One consequence worth stating: the request-body check reads the *registration*,
not the HTTP method - and only the **attachments** body-less ``put<TResponse>``
is exempt. That overload is the fire-and-forget state-machine kick
(``/updates/{id}/prepare``, the lifecycle transitions), which genuinely takes no
payload, and it records that on the entry. The plain body-less ``put`` and the
body-less ``post`` are both **not** exempt: their callers read the body by hand
(``PUT /{entity}/data/{data_id}`` parses free-form JSON so plugin-owned entities
can send shapes ``DataWriteRequest`` does not describe; ``/auth/*`` parses
form-urlencoded), so a missing ``.request_body(...)`` there is a real gap the
check must keep reporting.

A fourth status never reaches a handler either, and it is the only one gated on
nothing: **416**. cpp-httplib parses the ``Range`` header in
``Server::process_request``, before routing, and rejects an unparseable one
outright - on any path, including paths that do not exist. It is therefore
declared on every operation, next to the blanket 400/404/500, rather than on
the six download routes where sending a ``Range`` is *useful*. Those six carry
the ``Range`` request *parameter*; the status itself is universal.

It is declared as a ``GenericError`` ``$ref`` like the other error statuses,
and getting there needs both halves of the picture: cpp-httplib writes 416 with
an empty body, and ``RESTServer::setup_global_error_handlers`` then fills any
body-less error response with a ``GenericError``. Reading only the vendored
header suggests a body-less response and would have published one - the wire
assertion in
``test_openapi_contract.test.py::test_range_rejection_is_answered_on_a_route_that_declares_it``
is what settles it, deliberately against ``/health`` rather than a download so
the universality is the thing being proven.

Unlike the limiter's 429 there is no configuration knob to gate it on, and
unlike a handler status the emitted-status recorder cannot observe it - no
handler runs. It is therefore a framework-level constant verified by
``RouteRegistryTest.EveryDocumentedRouteDeclaresTheFrameworkAnsweredRangeRejection``
plus that wire test, not by a recorded run. It also sits outside
``only_status()``, for the same reason 401/403 do: that knob constrains what
the *handler* can return.

Three further statuses never reach a handler: the auth middleware answers 401
and 403, and the rate limiter answers 429, both ahead of routing. No return
type can describe them and no ``RouteEntry`` can carry their headers, so they
are declared once as the shared component responses ``Unauthorized`` (carrying
``WWW-Authenticate``), ``Forbidden`` and ``RateLimited`` (carrying
``Retry-After`` and the ``X-RateLimit-*`` trio) in ``OpenApiSpecBuilder``.
Routes reference them - 401/403 when ``set_auth_enabled(true)``, 429 when
``set_rate_limit_enabled(true)`` - so the document mentions a middleware status
exactly when that middleware is live.

The 429 gate covers the **rate limiter's** 429 and nothing else. A handler can
answer 429 for a reason of its own - the script manager's concurrent-execution
limit is the one that exists today - and that one is reachable whether or not
``rate_limiting.enabled`` is set, so it is declared on its route with
``errors({429})`` like any other handler status. Reading the two as one status
makes the coverage rule below unsatisfiable: with the limiter off, the document
would have to both omit 429 (no limiter) and declare it (the execution-start
route answers it).

Where a route declares a status the middleware also owns, the route wins:
``add_response_ref`` is first-wins and the ``errors()`` loop runs first. So the
execution-start route publishes its own ``GenericError`` 429 rather than the
``RateLimited`` component, and loses that component's ``Retry-After`` and
``X-RateLimit-*`` headers; a lifecycle route that declares 403 shadows
``Forbidden`` the same way. OpenAPI allows one response object per status, so
one description has to lose, and the route-specific one is the more useful.
The body shape is unaffected - all three components reference the same
``GenericError`` schema - so what is lost is the header list and the prose.
Pinned by ``RouteRegistryTest.RouteDeclaredStatusWinsOverTheMiddlewareComponent``
so the precedence is a decision on record rather than an accident of statement
order.

A fourth gate, ``set_aggregation_enabled(bool)``, works the same way for peer
federation. When an entity turns out to belong to a peer, the request is
proxied from inside ``validate_entity_for_route``, and the statuses the gateway
itself writes there are 502 (peer unknown, unreachable, or its response over the
size cap) and 503 (this gateway is shutting down and refuses to forward). They
are declared on entity-scoped routes only - the entity id has to come from the
path for the lookup to happen at all - and only when aggregation is on, because
``aggregation.enabled`` defaults false and the ``AggregationManager`` is only
constructed when it is set, so with it off no entity can be remote. The gate
reads the manager pointer rather than the parameter, so it cannot drift from the
branch it describes. Unlike the middleware refs it also respects
``only_status``: the forward happens *inside* the handler, so a route that
declares itself single-outcome genuinely cannot reach it.

What is **not** declared there is the status a healthy peer returns, which is
copied through verbatim. No finite ``errors({...})`` describes "whatever the
peer said", and choosing what the document should promise is an
aggregation-contract question rather than a documentation one.

Path parameters are synthesised, not declared
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``to_openapi_paths()`` walks each route's ``{param}`` templates and emits a
parameter object for every one the registration did not declare by hand, taking
its prose - and, where there is one, its length constraint - from a table in
that function. Almost every route relies on this: the entity-scoped routes are
registered from loops over the four entity types, and only the fault-trigger
routes and one ``app_id`` call ``path_param()`` explicitly.

That is why the constraint lives in the table rather than at the call sites.
``{fault_code}`` and ``{config_id}`` appear on several registrations each, none
of which describes them today; a per-registration ``path_param(name, desc,
schema)`` would have to be repeated on each and could be omitted on the next
route added, whereas the table applies to every route carrying the template and
there is nothing to forget.

That convenience carries a precondition, and it is the table's one weakness:
being keyed by parameter *name*, it cannot distinguish a route whose handler
checks from one whose handler does not. A ``maxLength`` therefore goes in only
where **every** handler behind the template rejects an over-long value
unconditionally - 256 for ``fault_code``, 512 for ``config_id`` (256-character
entity id, ``:``, 256-character parameter name).

The first version of this table did not hold that precondition:
``ConfigHandlers::delete_configuration`` was the one verb of the three that
read ``config_id`` without measuring it, while GET and PUT both rejected, so
four routes published a bound nothing enforced. The check was added rather than
the ``maxLength`` removed - GET and PUT checking and DELETE not was a real
inconsistency in the handler family.

Both rows are now driven on every verb they publish to, so the precondition is
tested rather than assumed:
``test_configuration_api.test.py::test_06b_every_verb_rejects_an_oversized_config_id``
covers ``config_id`` on GET, PUT and DELETE, and
``test_faults_api.test.py::test_both_verbs_reject_an_oversized_fault_code``
covers ``fault_code`` on GET and DELETE. A new row needs its own, or the bound
it publishes rests on a reading of the handlers rather than on a run.

One ordering difference the tests deliberately avoid depending on:
``FaultHandlers::clear_fault`` measures the code *after*
``validate_lock_access``, where the configuration handlers measure before it.
The published bound still holds - an over-long value is always rejected - but
under a competing lock the fault route answers 409 rather than 400.

``config_id`` is the one whose prose carries a contract rather than a
restatement of its name. On an entity that aggregates several ROS 2 nodes the
identifier is the ``app_id:param_name`` form the configurations list returns as
each item's ``id``, and a write of a bare parameter name is refused as
ambiguous; on a single-node entity it is the bare name and a colon is part of
it. The earlier description - "the ROS 2 parameter name" - named exactly the
form the write path rejects.

.. _emitted-status-recorder:

Emitted-status recorder
-----------------------

Everything above is a *declaration*. ``errors()``, ``response_header()``,
``lock_guarded()`` - each is something a person typed next to a registration,
and each can fall behind the handler it describes without any test noticing. A
new ``make_error(503, ...)`` in a handler nobody re-reads is invisible to every
check in this document.

The recorder is the one mechanism that notices, and it maintains no list.
``include/ros2_medkit_gateway/http/detail/status_recorder.hpp`` compiles - in
test builds only, gated on ``MEDKIT_STATUS_RECORDER``, which
``CMakeLists.txt`` sets exactly when ``BUILD_TESTING`` is on - two observers:

- ``StatusRecordingScope``, installed by ``RouteRegistry::register_all`` around
  every mounted handler, which records ``(method, OpenAPI templated path,
  status)`` for the status that actually reached ``httplib::Response``. It is
  installed at the mounting point because that is the only place that knows
  both the route's identity and everything the route can answer. This is the
  authoritative half: it sees the status the client receives, including one no
  ``make_error()`` built (a peer-forwarded status, a raw ``res.status`` write,
  the entity-not-found 404 that comes from ``validate_entity_for_route``).
- a call in ``make_error()`` that records the ``file:line`` of each error
  construction, which is what lets a run report *how much* of the ~281-site
  error surface it exercised rather than implying it saw all of it.

The two halves are deliberately not joined. The scope carries the route
identity as its own member, so nothing travels out-of-band, and
``make_error()``'s hook is route-agnostic - it contributes to a site set, not to
the ``(route, status)`` set the assertion reads. The consequence worth stating:
``make_error()`` touches no thread-local storage at all, which matters because
it is an ``inline`` header function whose out-of-line copy lands in
``gateway_ros2``, linked into six MODULE targets. Route-attributing the sites
would need an ambient carrier, and that carrier would have to be a
namespace-scope ``extern thread_local`` (the ``tl_forward_response`` pattern),
never a function-local ``static thread_local``, which compiles to initial-exec
TLS a shared object cannot relocate. Not needing it is the stronger position,
and the wire-status set is strictly more accurate than route-attributed
construction sites would be.

``test_openapi_error_coverage.test.py`` drives the whole documented surface into
its error branches - every parameterised operation called with an absent id,
with a malformed id, and (where a trailing absent id makes the call safe) with a
real leading entity - then asserts **declared is a superset of observed**. The
sweep is derived from the served document, so a route added tomorrow is swept
tomorrow, and its companion assertions stop the rule passing vacuously: the only
operations allowed to go unreached are state-changing verbs on parameterless
paths, and at least one observed status must be outside the blanket set.

What the recorder cannot see has to be declared by hand, and that is the whole
list:

- anything answered ahead of routing - the rate limiter's 429, the auth
  middleware's 401/403, the CORS reject, the OPTIONS pre-flight;
- anything cpp-httplib answers itself - 404/405 for an unrouted request, 413
  over ``set_payload_max_length``, 416 for an **unparseable** ``Range`` (an
  unsatisfiable-but-parseable one yields 206, not 416);
- routes mounted straight onto the server rather than through the registry -
  the Swagger UI subtree in ``-DENABLE_SWAGGER_UI=ON`` builds, the status
  recorder's own coverage endpoint, and anything a plugin mounts through
  ``PluginManager::register_routes``. The two ``/docs`` routes are **not** in
  this group: they are ``docs_endpoint`` / ``docs_subtree`` registrations, so
  ``register_all`` mounts them and the recorder wraps them like any other
  route. A plugin-served operation is excluded from the coverage assertion by
  its ``x-medkit-plugin-served`` marker rather than by a list;
- statuses on branches no test run drives - a provider that reports
  ``AccessDenied``, a fault store that cannot be read, an update already in
  flight. These are the ``errors({...})`` calls in ``rest_server.cpp`` that
  carry a "the recorder cannot reach it" comment.

Fourteen ``make_error`` sites pass a **computed** status rather than a literal,
and they split into two classes that get opposite treatment:

- **Seven are first-party with a finite range**, and are declared. Four go
  through ``classify_parameter_error``, whose ``ParameterErrorCode`` switch can
  only produce ``{400, 403, 404, 500, 503}``; three go through ``LockError``,
  where ``extend`` and ``release`` can only produce ``{400, 403, 404}``. Neither
  set is copied by hand. The parameter routes declare
  ``handlers::parameter_error_statuses()``, which runs the classifier over every
  enumerator listed in ``kAllParameterErrorCodes`` - so a new enumerator mapping
  to a new status widens the declaration with no edit at the registration, once
  it has been added to that hand-written array. ``-Werror=switch-enum`` alone
  did not force that: it demands a ``case``, and a build with the cases added
  and the array left short compiled clean while the registrations silently
  dropped a status. The enum ends in a ``COUNT`` sentinel and the array's size
  is ``static_assert``-ed against it, which is what makes the array's
  completeness a compiler check rather than a convention. The lock claim is behavioural
  rather than textual, so it is pinned behaviourally:
  ``LockManagerTest.extend_and_release_answer_only_400_403_404`` drives every
  reachable failure path of both verbs and asserts the exact status set,
  including that 409 is **not** among them - only ``acquire`` conflicts.
- **Seven are plugin-clamped and stay undeclared**: ``make_plugin_error`` in the
  data, fault, lifecycle and operation handlers passes a provider-supplied
  status clamped only to 400-599. A plugin can answer any of ~200 statuses, so
  no finite ``errors({...})`` describes it - the same reason the peer
  pass-through above is not declared. Left undeclared deliberately, not
  overlooked: this is the boundary where "declare what the gateway can emit"
  stops being a finite question, and both sides of it are named here so the
  next reader does not have to re-derive which half is which.

A shipped gateway compiles none of it: the ``Dockerfile`` builds with
``-DBUILD_TESTING=OFF``, so ``make_error()`` is byte-identical to what it was
and ``register_all`` mounts the handler directly.

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
  Used by the fault SSE stream and by cyclic-subscription event streams. The
  helper declares ``text/event-stream`` on the 200 from the same string it
  hands cpp-httplib, and declares **no** frame schema: the three SSE families
  put different shapes in ``data:``, so one schema *here* would be wrong for
  two of them. Each registration names its own with
  ``.success_schema<dto::XxxEventFrame>()``, which replaces the schema of the
  already-declared 200 and leaves its ``Cache-Control`` /
  ``X-Accel-Buffering`` headers standing - a second ``response(200, ...)``
  would replace the whole response object and drop them.

  The three families and the code each schema has to agree with:

  .. list-table::
     :header-rows: 1
     :widths: 30 30 40

     * - DTO
       - Built by
       - Shape
     * - ``TriggerEventFrame``
       - ``TriggerManager``
       - ``{timestamp, payload}``; no error branch, because a frame exists only
         when the condition fired
     * - ``SubscriptionEventFrame``
       - ``SubscriptionTransportProvider::make_sse_stream``
       - ``{timestamp, payload | error}``; a failed sample reports and the
         stream stays open
     * - ``FaultStreamEvent``
       - ``SSEFaultHandler::format_sse_event``
       - ``{event_type, fault, timestamp, x-medkit?}``; no ``payload`` key at
         all, and ``timestamp`` is epoch seconds where the other two send an
         ISO 8601 string

  A schema against ``text/event-stream`` is legitimate because a ``data:``
  field *is* a JSON document. ``response()`` decides that per media type
  (``media_type_carries_a_json_document``) rather than allowing it wholesale,
  so a binary download still cannot acquire one - see ``binary_download``
  below. Neither schema covers the non-JSON lines a stream also emits:
  ``:keepalive`` comments, and the ``id:`` / ``event:`` fields the fault stream
  sets. The fault stream declares the matching ``Last-Event-ID`` request
  header, without which its frame ids are a number clients can see and cannot
  use.
- ``reg.binary_download(path, handler, media_types)`` - registers a range-aware
  binary download. The handler returns a ``Result<http::BinaryResponse>``
  carrying ``provider``, ``content_type``, ``filename``, ``supports_ranges``,
  and ``total_size``; the framework wires ``provider`` into cpp-httplib's
  range-aware content-provider machinery so partial-content fetches work
  without manual ``Content-Range`` plumbing. The helper owns the whole header
  and status story for these routes: it sends ``Content-Disposition`` when the
  response names a file and ``Accept-Ranges: bytes`` when the provider is
  range-capable (cpp-httplib only sets the latter for ``HEAD``), and it
  declares 200, 206, those headers, the ``Range`` request parameter and
  ``multipart/byteranges`` on the 206 - see ``mark_partial_content()`` above.
  ``media_types`` is required rather than defaulted so a new download route
  cannot inherit another route's answer, and it must cover every value the
  handler can put in ``BinaryResponse::content_type``.
- ``reg.multipart_upload<TResponse>(path, handler)`` - registers a
  ``multipart/form-data`` upload. The handler receives ``http::MultipartBody``
  (already parsed by cpp-httplib) and returns
  ``Result<std::pair<TResponse, http::ResponseAttachments>>``. Uploads declare
  201 through ``TResponse`` (``http::Created<dto::BulkDataDescriptor>``) and use
  the attachments only for the ``Location`` header. Used by bulk-data POST/PUT.

  The *request* half has to be declared at the call site with
  ``.multipart_body(desc, parts)``: the helper cannot see which parts a handler
  looks up in ``MultipartBody.parts``, so without it the body is the
  ``{"type": "object", "additionalProperties": true}`` placeholder the helper
  installs - a body no generated client can build. Each ``MultipartPart`` names
  the part, says whether the handler rejects the request without it, and carries
  either a schema (a textual part such as ``metadata``) or an empty schema plus
  a ``content_type`` (a binary part such as ``file``). The empty schema is
  deliberate: OpenAPI 3.1 describes a binary part through
  ``encoding.<part>.contentType``, having dropped ``format: binary`` with the
  rest of the pre-JSON-Schema-2020-12 vocabulary.
- ``reg.static_asset(path, handler)`` - serves bytes already in memory
  (Swagger UI bundles, embedded HTML/JS/CSS) as
  ``Result<http::StaticAsset>`` carrying ``bytes``, ``content_type``, and
  per-response ``headers`` (``Cache-Control``, ``ETag``).
- ``reg.docs_endpoint(path, handler)`` - registers the OpenAPI JSON endpoint
  at the given path. The handler returns ``Result<nlohmann::json>``; this is
  the only built-in route allowed to use raw ``nlohmann::json`` as
  ``TResponse``, because the body is the spec itself.
- ``reg.docs_subtree(openapi_path, regex, handler)`` - a route whose URI is a
  cpp-httplib regex the OpenAPI path grammar cannot express. Its one caller
  outside the unit tests is the ``<entity-path>/docs`` sub-document, mounted on
  ``(.+)/docs$`` because the prefix it captures is a whole entity or resource
  path. ``openapi_path``
  is what the document publishes and ``regex`` is what cpp-httplib matches;
  they are separate arguments so the regex never reaches the document as a
  path key.
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
``FaultEnvironmentData``, and ``_links`` on the four ``*Detail`` DTOs) follow
the same rule: pass the JSON through verbatim because something other than the
DTO layer owns the shape. ``_links`` is the case where the reason is a type
rather than a plugin: its values are a union - a path string for every relation
except ``depends-on``, which is an array of paths - and ``SchemaWriter`` walks
``dto_fields``, so it has no descriptor for a map whose values differ in type.
The field carries a description naming each relation and its value shape, so a
generated client reads prose instead of an unexplained ``{}``. The opaque DTO
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
ABI: ``JsonWriter`` emits the ``content`` object verbatim and JsonReader
accepts any JSON object on round-trip. This keeps backend-specific shapes
(UDS DTC records, OPC-UA alarm metadata, vendor extensions) flowing through
unchanged while pinning the envelope itself to a single typed contract.

What their ``SchemaWriter`` publishes is *not* uniformly
``{type: object, x-medkit-opaque: true}``. An envelope on a route that also
serves ROS 2-backed entities has a known shape for exactly those entities, and
publishing the bare object schema told a client nothing about either branch. So
those envelopes publish an ``anyOf``: the in-tree DTO first, the opaque
plugin branch last, with a schema-level ``description`` naming
``x-medkit.source`` as the way a client tells the two apart before calling.
``FaultListResult`` (``FaultList`` | ``FaultListAggregated`` | plugin),
``FaultDetailResult`` (``FaultDetail`` | plugin) and ``DataListResult``
(``DataList`` | plugin) are the three. Envelopes with no in-tree named shape -
``FaultClearResult``, ``DataValue``, ``OperationExecutionResult`` - stay a plain
opaque object and carry a ``description`` saying who decides the shape and where
a client discovers it. Every named schema needs *some* prose for that reason;
``test_openapi_contract`` fails a content-free schema that carries none.

``UpdateDetail`` and ``UpdateRegisterRequest`` show the third variant: their
``SchemaWriter`` publishes real ``properties`` while ``JsonWriter`` and
``JsonReader`` stay pass-through. Nothing round-trips through the descriptor,
so the vendor extensions a backend stores (Uptane TUF metadata, component
lists) survive untouched, and ``additionalProperties: true`` keeps them legal.
The two are typed from different sources, because the standard treats them
differently: ``UpdateDetail`` gets SOVD's attribute table (ISO 17978-3
section 7.18), since SOVD fixes the response shape; ``UpdateRegisterRequest``
declares only ``id`` - the sole field ``post_update`` validates - because SOVD
leaves the register *request* manufacturer-specific. Transplanting the response
table onto the request would document a validation the gateway does not
perform.

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
  survivors merged in. Membership of ``AllDtos`` is therefore a publishing
  decision, not a bookkeeping one: a DTO that exists only to type a plugin ABI
  (``DataWriteResult``, the return type of ``DataProvider::write_data``, which
  no route answers with) or that a route stopped returning
  (``Collection<ScriptMetadata>``, superseded by ``ScriptList``) must be left
  out, or every generated client materialises a type it can never receive.
- ``paths`` is ``RouteRegistry::to_openapi_paths()``: every typed route
  contributes a path item with ``$ref`` entries auto-derived from its
  ``TResponse`` / ``TBody`` template parameters plus any tags, summary,
  description, ``operation_id``, parameter, or extra-status metadata pinned
  on the route via the fluent ``RouteEntry`` builder. The per-topic /
  per-service / per-action routes for genuinely dynamic ROS 2 payloads carry an
  *inline* schema built by ``SchemaBuilder``'s ``from_ros_msg`` /
  ``from_ros_srv_request`` / ``from_ros_srv_response`` /
  ``generic_object_schema`` factories (these feed path operations, not
  ``components/schemas``).

``OpenApiSpecBuilder::build()`` then assembles ``info`` / ``servers`` /
``tags`` / ``security`` around those two compiled blocks. There are no
hand-written ``paths`` items in the published spec, and no hand-written schema
blocks in ``components/schemas``. Adding a route or a DTO field updates the
spec on the next process start with no schema-side edit.

Because the two blocks are compiled independently, they can drift apart: a DTO
can sit in ``AllDtos`` while no route's ``$ref`` chain reaches it.
``openapi::unreachable_schemas(document)``
(``core/openapi/document_checks.hpp``) closes that gap. It walks every ``$ref``
an operation makes, follows the chain through ``components/responses`` and
through each reached schema, and returns the names nothing arrives at.
``CapabilityGenerator::generate_root()`` runs it over the assembled document and
logs a warning naming the orphans;
``test_openapi_contract::test_no_unreachable_schemas`` is what turns a suite red.
It is deliberately a free function over the finished document rather than a rule
in ``RouteRegistry::validate_completeness()``: the registry sees routes and has
no visibility of either component block, so only the assembled document can
answer the question. Its unit tests are ``test_schema_reachability``, which
links ``gateway_core`` - the function touches no ROS type.

``openapi::referenced_schemas(subtree, pool)`` in the same header is the inverse
walk, and it exists for the scoped ``<entity-path>/docs`` documents. Those are a
projection of the root document's ``paths`` (``paths_under()`` filters by path
segment, ``strip_entity_path_parameter()`` removes the ``in: path`` parameter a
substituted id answered), so their operations ``$ref`` named schemas while the
document carries only what ``OpenApiSpecBuilder`` always emits. Shipping the
whole ``AllDtos`` pool on every entity page is the alternative;
``referenced_schemas`` ships the transitive closure the slice actually reaches.
``CapabilityGeneratorTest.SubDocumentCarriesTheSchemasItReferences`` walks every
``$ref`` in five scoped documents and fails on one that resolves to nothing.

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
5. Declare the route's weakest permitted caller with
   ``.requires_role(UserRole::...)`` - see "Route Authorization" below. This is
   not optional: ``validate_completeness()`` reports a route without it as an
   error, because authorization fails closed and the route would answer 403 for
   every role below ADMIN.
6. Nothing to do for the root endpoint list: ``get_root`` derives it from the
   registry, so registering the route advertises it. Routes a plugin mounts
   itself are derived too, from ``RouteDescriptions::endpoints()``. Swagger UI
   is the only hand-written entry left. That the list and the document agree is
   checked by
   ``test_openapi_contract.test.py::test_the_root_list_and_the_document_agree``.
7. Add URI field to entity detail response if the new route is a resource
   collection.
8. Write a unit test using ``JsonWriter<T>::write()`` and
   ``JsonReader<T>::read()`` directly - no HTTP server needed.
9. Write an integration test that calls the live endpoint.

Route Authorization
-------------------

A route's RBAC rule is a property of the route, so it is declared where the
route is registered:

.. code-block:: cpp

   reg.put<dto::ConfigurationWriteRequest, dto::ConfigurationReadValue>(...)
       .tag("Configuration")
       .requires_role(UserRole::CONFIGURATOR)
       ...

That one call feeds two consumers, which is the whole reason it lives on the
registration rather than in a table beside it:

- ``RouteRegistry::route_permissions(api_prefix)`` turns it into the
  ``"<METHOD>:<pattern>"`` entries ``AuthManager::check_authorization``
  matches against. ``RESTServer::setup_routes()`` merges those into the manager
  before the server starts listening, together with
  ``AuthConfig::residual_route_permissions()`` for the routes the registry never
  sees (plugin routes, Swagger UI, the test-build status recorder).
- ``to_openapi_paths()`` publishes it as
  ``security: [{bearerAuth: [<role>]}]`` on the operation - the same shape a
  plugin's ``OperationDesc::requires_role`` emits.
  ``CapabilityGenerator::generate_impl()`` strips every per-operation
  requirement again when ``auth.enabled`` is false, once, over the assembled
  document.

Two translations happen inside ``route_permissions()``:

- The pattern is derived from the route's **cpp-httplib regex**, not from its
  OpenAPI path. ``([^/]+)`` becomes ``*`` and ``(.+)`` becomes ``**``, which is
  what keeps the slash-spanning parameters (``{data_id}``, ``{config_id}``) and
  the ``<entity-path>/docs`` catch-all reachable. Deriving from ``{param}``
  alone would make all three single-segment.
- Roles are expanded upward. ``AuthConfig`` has no inheritance -
  ``check_authorization`` looks up exactly one role's set - so a route
  declaring ``OPERATOR`` is written into OPERATOR, CONFIGURATOR and ADMIN.

``public_route()`` is the only alternative to ``requires_role()``, and it is
legitimate only where the middleware exempts the path before the table is
consulted at all - today ``/auth/*``, which both ``AllAuthRequirementPolicy``
and ``WriteOnlyAuthRequirementPolicy`` let through by prefix. It emits
``security: []`` and contributes no permission entry.

The declaration is required on ``hidden()`` routes too. Hidden removes a route
from the document, not from the router: the request still arrives and still
meets the permission table.
``test_rbac_contract.test.py`` is the end-to-end check that the published role
and the enforced role are the same role.

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
