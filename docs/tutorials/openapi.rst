OpenAPI / Swagger
=================

The gateway self-describes its REST API using the
`OpenAPI 3.1.0 <https://spec.openapis.org/oas/v3.1.0>`_ specification.
Every level of the URL hierarchy exposes a ``/docs`` endpoint that returns
a context-scoped OpenAPI spec - so tools and developers always have an
accurate, up-to-date description of available operations.

Accessing the Spec
------------------

Append ``/docs`` to any valid API path:

.. code-block:: bash

   # Full gateway spec (all endpoints)
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/docs | jq .

   # Spec scoped to the components collection
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/docs

   # Spec for a specific component and its resource collections
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/my_sensor/docs

   # Spec for one resource collection (e.g. data)
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/my_sensor/data/docs

Entity-level specs reflect the actual capabilities of each entity at
runtime. Plugin-registered vendor routes also appear when the requested
path matches a plugin route prefix.

Swagger UI
----------

For an interactive API browser, build the gateway with Swagger UI support:

.. code-block:: bash

   colcon build --cmake-args -DENABLE_SWAGGER_UI=ON

Then open ``http://localhost:8080/api/v1/swagger-ui`` in your browser.
The UI assets are embedded in the binary - no CDN dependency at runtime.

.. note::

   The CMake configure step downloads assets from unpkg.com, so network
   access is required during the build.

Using with External Tools
-------------------------

The spec returned by ``/docs`` is standard OpenAPI and works with any
compatible tooling:

- **Postman** - Import ``http://localhost:8080/api/v1/docs`` as a URL to
  auto-generate a request collection.
- **Client generators** - Feed the spec to
  `openapi-generator <https://openapi-generator.tech/>`_ to produce typed
  clients in Python, TypeScript, Go, and many other languages.
- **Documentation** - Render with `Redoc <https://redocly.com/redoc>`_ or
  any OpenAPI-compatible doc renderer.

Configuration
-------------

The ``/docs`` endpoints are enabled by default. To disable them:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       docs:
         enabled: false

When disabled, all ``/docs`` endpoints return HTTP 501.

See :doc:`/config/server` for the full parameter reference.

How Schemas Are Generated
--------------------------

The ``components/schemas`` object in every ``/docs`` response is generated
automatically from the DTO registry. Most response and request types are
declared as a plain C++ struct with a ``constexpr dto_fields<T>`` descriptor
tuple; the ``SchemaWriter<T>`` visitor folds over this tuple at compile time to
produce the OpenAPI JSON Schema entry. The ``AllDtos`` registry in
``dto/registry.hpp`` lists every named type so that
``collect_component_schemas()`` populates the entire ``components/schemas`` map
with no hand-written schema factories.

For a field-walking DTO the same descriptor also drives serialization
(``JsonWriter<T>``) and request-body validation (``JsonReader<T>``), so the wire
shape and the published schema are always derived from the same source.

A few envelope types whose payload shape is decided at runtime by a plugin or by
a live ROS 2 type - the ``Data*Result`` / ``Fault*Result`` /
``OperationExecutionResult`` *opaque* DTOs - have no ``dto_fields``. They carry a
hand-written ``JsonWriter`` / ``JsonReader`` / ``SchemaWriter`` trio instead, are
still listed in ``AllDtos``, and publish an opaque object schema
(``{type: object, additionalProperties: true, x-medkit-opaque: true}``). Inside
an ordinary DTO, a free-form member typed as a bare ``nlohmann::json`` (for
example the fault environment records) appears as an unconstrained object
(``{}``). The per-topic / per-service / per-action routes for live ROS 2 data do
not use ``components/schemas`` at all: their request and response bodies carry an
inline schema derived from the ROS 2 type on the route itself.

For the full design of the DTO contract layer, see
:doc:`/design/ros2_medkit_gateway/dto_contract`.

See Also
--------

- :doc:`/api/rest` - Complete REST API reference
- :doc:`/config/server` - Server configuration options
