Locking API
===========

SOVD-compliant resource locking for components and apps (ISO 17978-3, Section 7.17).
Locks prevent concurrent modification of entity resources by multiple clients.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Locking provides mutual exclusion for entity resource collections. A client acquires a lock
on a component or app, specifying which resource collections (scopes) are protected. Other
clients are blocked from modifying locked collections until the lock is released or expires.

Key concepts:

- **Client identification**: Clients use the ``X-Client-Id`` header (client-generated UUID)
- **Scoped locks**: Locks can protect specific collections (e.g., ``configurations``, ``operations``)
  or all collections when no scopes are specified
- **Parent propagation**: A lock on a component also protects its child apps (lazy check)
- **Lock breaking**: Privileged clients can break existing locks with ``break_lock: true``
- **Automatic expiry**: Locks expire after the specified TTL and are cleaned up periodically

Configuration
-------------

Enable locking in ``gateway_params.yaml``:

.. code-block:: yaml

   locking:
     enabled: true
     default_max_expiration: 3600
     cleanup_interval: 30
     defaults:
       components:
         lock_required_scopes: [configurations, operations]
         breakable: true
       apps:
         lock_required_scopes: [configurations]
         breakable: true

Per-entity overrides in the manifest:

.. code-block:: yaml

   components:
     - id: safety_controller
       name: Safety Controller
       lock:
         required_scopes: [configurations, operations, data]
         breakable: false
         max_expiration: 7200

Endpoints
---------

Acquire Lock
~~~~~~~~~~~~

``POST /api/v1/{components|apps}/{entity_id}/locks``

Acquire a lock on an entity. Requires ``X-Client-Id`` header.

**Request Body:**

.. code-block:: json

   {
     "lock_expiration": 300,
     "scopes": ["configurations", "operations"],
     "break_lock": false
   }

- ``lock_expiration`` (required): Lock TTL in seconds
- ``scopes`` (optional): Resource collections to lock. If omitted, all collections are locked.
  Valid scopes: ``data``, ``operations``, ``configurations``, ``faults``, ``bulk-data``,
  ``modes``, ``scripts``, ``logs``, ``cyclic-subscriptions``
- ``break_lock`` (optional, default ``false``): If true, replaces any existing lock

**Response (201 Created):**

.. code-block:: json

   {
     "id": "lock_1",
     "owned": true,
     "scopes": ["configurations", "operations"],
     "lock_expiration": "2026-03-18T21:30:00Z"
   }

List Locks
~~~~~~~~~~

``GET /api/v1/{components|apps}/{entity_id}/locks``

List locks on an entity. ``X-Client-Id`` header is optional (determines ``owned`` field).

**Response (200 OK):**

.. code-block:: json

   {
     "items": [
       {
         "id": "lock_1",
         "owned": true,
         "scopes": ["configurations"],
         "lock_expiration": "2026-03-18T21:30:00Z"
       }
     ]
   }

Get Lock Details
~~~~~~~~~~~~~~~~

``GET /api/v1/{components|apps}/{entity_id}/locks/{lock_id}``

Get details of a specific lock. Returns 404 if not found.

Extend Lock
~~~~~~~~~~~

``PUT /api/v1/{components|apps}/{entity_id}/locks/{lock_id}``

Extend a lock's expiration. Requires ``X-Client-Id`` header (must be lock owner).

**Request Body:**

.. code-block:: json

   {
     "lock_expiration": 600
   }

**Response:** 204 No Content

Release Lock
~~~~~~~~~~~~

``DELETE /api/v1/{components|apps}/{entity_id}/locks/{lock_id}``

Release a lock. Requires ``X-Client-Id`` header (must be lock owner).

**Response:** 204 No Content

.. _locking-blocked-operations:

Which Operations a Lock Blocks
------------------------------

The five endpoints above manage locks. A lock only means something because
*other* endpoints honour it: every write below reads the caller's
``X-Client-Id`` and answers ``409`` when the entity's collection is held by a
different client. Sending no ``X-Client-Id`` makes the caller anonymous - the
write succeeds while nothing is locked and is refused once something is - so
the header is optional on these routes, not required.

Each row applies to all four entity types (``areas``, ``components``, ``apps``,
``functions``) except bulk-data, which only exists for ``components`` and
``apps``.

.. list-table::
   :header-rows: 1
   :widths: 45 20 35

   * - Endpoint
     - Lock scope
     - Operation IDs
   * - ``PUT /{entity}/data/{data_id}``
     - ``data``
     - ``put{Area,Component,App,Function}DataItem``
   * - ``POST /{entity}/operations/{id}/executions``
     - ``operations``
     - ``execute{...}Operation``
   * - ``PUT /{entity}/operations/{id}/executions/{exec_id}``
     - ``operations``
     - ``update{...}Execution``
   * - ``DELETE /{entity}/operations/{id}/executions/{exec_id}``
     - ``operations``
     - ``cancel{...}Execution``
   * - ``PUT /{entity}/configurations/{config_id}``
     - ``configurations``
     - ``set{...}Configuration``
   * - ``DELETE /{entity}/configurations/{config_id}``
     - ``configurations``
     - ``delete{...}Configuration``
   * - ``DELETE /{entity}/configurations``
     - ``configurations``
     - ``deleteAll{...}Configurations``
   * - ``DELETE /{entity}/faults/{fault_code}``
     - ``faults``
     - ``clear{...}Fault``
   * - ``DELETE /{entity}/faults``
     - ``faults``
     - ``clearAll{...}Faults``
   * - ``PUT /{entity}/logs/configuration``
     - ``logs``
     - ``set{...}LogConfiguration``
   * - ``POST /{entity}/bulk-data/{category_id}``
     - ``bulk-data``
     - ``upload{Component,App}BulkData``
   * - ``DELETE /{entity}/bulk-data/{category_id}/{file_id}``
     - ``bulk-data``
     - ``delete{Component,App}BulkData``

Every one of these operations carries ``x-medkit-lock-guarded: true`` in the
generated OpenAPI document, alongside the ``X-Client-Id`` parameter and the
``409`` response, so a generated client can select the lock-participating
surface without pattern-matching on paths.

The marker is applied per route at registration time, not inferred from the
handler. It is pinned by
``test_openapi_contract.test.py::test_lock_guarded_set_matches_the_handlers``
against a hand-maintained list, which catches the document losing a marker but
cannot catch a *new* lock-checking handler that was never added to the list.
Adding a lock check to a handler means updating that list too.

The One Exception: Global Fault Clear
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``DELETE /api/v1/faults`` reads ``X-Client-Id`` like the writes above but never
answers ``409``. It walks every fault, **skips** the ones whose reporting
entity is locked by another client, clears the rest, and answers ``204``.
Nothing on the response says which faults were skipped - the
``X-Medkit-Local-Only: true`` header that 204 also carries is set
unconditionally and reports that aggregated *peers* were not cleared, not that
a lock intervened. A caller who needs to know re-reads the entity's faults to
see what survived. The operation declares ``X-Client-Id`` but carries no
``x-medkit-lock-guarded`` marker, because it cannot return the ``409`` the
marker implies.

Error Responses
---------------

.. list-table::
   :header-rows: 1

   * - Status
     - Error Code
     - Condition
   * - 400
     - ``invalid-parameter``
     - Missing ``X-Client-Id``, invalid ``lock_expiration``, unknown scope
   * - 403
     - ``forbidden``
     - Trying to extend/release a lock owned by another client
   * - 404
     - ``entity-not-found``
     - Entity does not exist
   * - 404
     - ``resource-not-found``
     - Lock ID not found on this entity
   * - 409
     - ``invalid-request``
     - Entity already locked by another client (without ``break_lock``)
   * - 409
     - ``lock-broken``
     - Mutating operation blocked by another client's lock
   * - 501
     - ``not-implemented``
     - Locking is disabled on this gateway
