Resource Locking
================

This tutorial shows how to use SOVD resource locking (ISO 17978-3, Section 7.17)
to prevent concurrent modification of entity resources by multiple clients.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

When multiple clients interact with the same entity - for example, two
diagnostic tools both trying to reconfigure a motor controller - their changes
can conflict. Resource locking solves this by granting a client exclusive access
to an entity's resource collections (data, configurations, operations, etc.)
for a bounded time period.

Key concepts:

- **Client identification**: Each client generates a UUID and sends it via the
  ``X-Client-Id`` header on every request
- **Scoped locks**: A lock can protect specific collections (e.g., only
  ``configurations``) or all collections when no scopes are specified
- **Parent propagation**: A lock on a component also protects its child apps -
  the gateway walks up the entity hierarchy when checking access
- **Lock breaking**: A client can forcefully replace an existing lock by setting
  ``break_lock: true`` (unless the entity is configured as non-breakable)
- **Automatic expiry**: Locks expire after a TTL and are cleaned up periodically

Locking is supported on **components** and **apps** only. Areas cannot be locked
directly.

Quick Example
-------------

The examples below use ``curl`` against a gateway running on ``localhost:8080``.
Pick a client ID (any unique string) and use it consistently.

.. code-block:: bash

   CLIENT_ID="my-diagnostic-tool-$(uuidgen)"

**1. Acquire a lock**

Lock the ``configurations`` and ``operations`` collections on a component for
5 minutes:

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/components/motor_controller/locks \
     -H "Content-Type: application/json" \
     -H "X-Client-Id: $CLIENT_ID" \
     -d '{
       "lock_expiration": 300,
       "scopes": ["configurations", "operations"]
     }'

Response (``201 Created``):

.. code-block:: json

   {
     "id": "lock_1",
     "owned": true,
     "scopes": ["configurations", "operations"],
     "lock_expiration": "2026-03-21T15:05:00Z"
   }

Save the ``id`` value - you need it to extend or release the lock.

**2. Perform work while holding the lock**

Other clients that try to modify ``configurations`` or ``operations`` on
``motor_controller`` (or any of its child apps) will receive a ``409 Conflict``
response until the lock is released or expires.

.. code-block:: bash

   # This succeeds because we hold the lock
   curl -X PUT http://localhost:8080/api/v1/components/motor_controller/configurations/max_speed \
     -H "Content-Type: application/json" \
     -H "X-Client-Id: $CLIENT_ID" \
     -d '{"data": 1500}'

**3. Extend the lock**

If the work takes longer than expected, extend the lock before it expires:

.. code-block:: bash

   curl -X PUT http://localhost:8080/api/v1/components/motor_controller/locks/lock_1 \
     -H "Content-Type: application/json" \
     -H "X-Client-Id: $CLIENT_ID" \
     -d '{"lock_expiration": 600}'

Response: ``204 No Content``

The lock now expires 600 seconds from the time of the extend request (not from
the original acquisition time).

**4. Release the lock**

When done, release the lock so other clients can proceed:

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/components/motor_controller/locks/lock_1 \
     -H "X-Client-Id: $CLIENT_ID"

Response: ``204 No Content``

**5. List locks (optional)**

Check what locks exist on an entity:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/motor_controller/locks \
     -H "X-Client-Id: $CLIENT_ID"

The ``owned`` field in each lock item indicates whether the requesting client
holds that lock.

Lock Enforcement
----------------

By default, locking is **opt-in** - clients can acquire locks, but the gateway
does not *require* them. To make locking mandatory for certain resource
collections, configure ``lock_required_scopes``.

When required scopes are set, any mutating request to a listed collection is
rejected with ``409 Conflict`` unless the requesting client holds a valid lock
on the entity.

**Example**: Require a lock before modifying configurations or operations on
any component:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       locking:
         enabled: true
         defaults:
           components:
             lock_required_scopes: [configurations, operations]
           apps:
             lock_required_scopes: [configurations]

With this configuration, a ``PUT`` to
``/components/motor_controller/configurations/max_speed`` without first
acquiring a lock returns:

.. code-block:: json

   {
     "error_code": "invalid-request",
     "message": "Lock required for 'configurations' on entity 'motor_controller'"
   }

The two-phase access check works as follows:

1. **Lock-required check** - If ``lock_required_scopes`` includes the target
   collection, the client must hold a valid (non-expired) lock on the entity.
   If not, access is denied immediately.
2. **Lock-conflict check** - If another client holds a lock covering the target
   collection, access is denied. This check walks up the parent chain
   (app -> component -> area) so a component lock also protects child apps.

Per-Entity Configuration
------------------------

The manifest ``lock:`` section lets you override lock behavior for individual
components or apps. This is useful when certain entities have stricter
requirements than the global defaults.

.. code-block:: yaml

   # manifest.yaml
   components:
     - id: safety_controller
       name: Safety Controller
       lock:
         required_scopes: [configurations, operations, data]
         breakable: false
         max_expiration: 7200

     - id: telemetry
       name: Telemetry
       lock:
         breakable: true

   apps:
     - id: motor_driver
       name: Motor Driver
       component: safety_controller
       lock:
         required_scopes: [configurations]
         breakable: false

The three manifest lock fields are:

- ``required_scopes`` - Collections that require a lock before mutation
  (overrides the type-level ``lock_required_scopes`` default)
- ``breakable`` - Whether other clients can use ``break_lock: true`` to replace
  an existing lock on this entity (default: ``true``)
- ``max_expiration`` - Maximum lock TTL in seconds for this entity
  (``0`` = use the global ``default_max_expiration``)

Configuration is resolved with the following priority:

1. Per-entity manifest override (``lock:`` section on the entity)
2. Per-type default (``locking.defaults.components`` or ``locking.defaults.apps``)
3. Global default (``locking.default_max_expiration``)

Lock Expiry
-----------

Every lock has a TTL set by ``lock_expiration`` at acquisition time. The maximum
allowed value is capped by ``default_max_expiration`` (global) or
``max_expiration`` (per-entity override).

A background timer runs every ``cleanup_interval`` seconds (default: 30) and
removes all expired locks. When a lock expires, the gateway also cleans up
associated temporary resources:

- **Cyclic subscriptions**: If the expired lock's scopes include
  ``cyclic-subscriptions`` (or the lock had no scopes, meaning all collections),
  any cyclic subscriptions for that entity are removed. This prevents orphaned
  subscriptions from accumulating after a client disconnects without cleaning up.

The cleanup timer logs each expiration:

.. code-block:: text

   [INFO] Lock lock_3 expired on entity motor_controller
   [INFO] Removed subscription sub_42 on lock expiry

To avoid lock expiry during long operations, clients should periodically extend
their locks using the ``PUT`` endpoint.

Plugin Integration
------------------

Gateway plugins receive a ``PluginContext`` reference that provides lock-aware
methods. Plugins should check locks before performing mutating operations on
entity resources.

**Checking lock access:**

.. code-block:: cpp

   // In a plugin provider method
   auto result = context.check_lock(entity_id, client_id, "configurations");
   if (!result.allowed) {
     return tl::make_unexpected("Blocked by lock: " + result.denied_reason);
   }

``check_lock`` delegates to ``LockManager::check_access`` and performs the same
two-phase check (lock-required + lock-conflict) used by the built-in handlers.
If locking is disabled on the gateway, ``check_lock`` always returns
``allowed = true``.

**Acquiring a lock from a plugin:**

.. code-block:: cpp

   auto lock_result = context.acquire_lock(entity_id, client_id, {"configurations"}, 300);
   if (!lock_result) {
     // lock_result.error() contains LockError with code, message, status_code
     return tl::make_unexpected(lock_result.error().message);
   }
   auto lock_info = lock_result.value();
   // lock_info.lock_id, lock_info.expires_at, etc.

Configuration Reference
-----------------------

All locking parameters are documented in the server configuration reference:

- :doc:`/config/server` - ``Locking`` section for gateway parameters
- :doc:`/api/locking` - Full REST API endpoint reference with request/response schemas

.. list-table::
   :header-rows: 1
   :widths: 40 15 45

   * - Parameter
     - Default
     - Description
   * - ``locking.enabled``
     - ``true``
     - Enable the lock manager and lock endpoints
   * - ``locking.default_max_expiration``
     - ``3600``
     - Maximum lock TTL in seconds
   * - ``locking.cleanup_interval``
     - ``30``
     - Seconds between expired lock cleanup sweeps
   * - ``locking.defaults.components.lock_required_scopes``
     - ``[]``
     - Collections requiring a lock on components (empty = no requirement)
   * - ``locking.defaults.components.breakable``
     - ``true``
     - Whether component locks can be broken
   * - ``locking.defaults.apps.lock_required_scopes``
     - ``[]``
     - Collections requiring a lock on apps (empty = no requirement)
   * - ``locking.defaults.apps.breakable``
     - ``true``
     - Whether app locks can be broken

Valid lock scopes: ``data``, ``operations``, ``configurations``, ``faults``,
``bulk-data``, ``modes``, ``scripts``, ``logs``, ``cyclic-subscriptions``.

See Also
--------

- :doc:`/api/locking` - Locking REST API reference
- :doc:`/config/server` - Server configuration (Locking section)
- :doc:`/tutorials/authentication` - JWT authentication (complementary to locking)
- :doc:`/tutorials/manifest-discovery` - Manifest YAML for per-entity lock config
- :doc:`/tutorials/plugin-system` - Writing gateway plugins
