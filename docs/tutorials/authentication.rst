Configuring Authentication
==========================

This tutorial shows how to enable JWT-based authentication with
Role-Based Access Control (RBAC) in ros2_medkit_gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

By default, the gateway runs without authentication for easy development.
For production deployments, you should enable authentication to:

- Control who can access the API
- Limit write operations to authorized users
- Audit API access

Authentication Modes
--------------------

The gateway supports three authentication modes via the ``require_auth_for`` parameter:

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Mode
     - Description
   * - ``none``
     - No authentication required. Auth endpoints still available for token management.
   * - ``write``
     - Authentication required for POST, PUT, DELETE operations. GET is open. **(Recommended for development)**
   * - ``all``
     - All endpoints require authentication. **(Recommended for production)**

Roles and Permissions
---------------------

Roles are cumulative: each one may do everything the row above it may do, plus
its own column. There is no inheritance in the stored table - the gateway
expands each route's declared role upward when it builds the table - but the
effect for a caller is the ladder below.

.. list-table::
   :widths: 18 14 14 14 20 20
   :header-rows: 1

   * - Role
     - Read (GET)
     - Data (PUT)
     - Operations, faults, locks, subscriptions, triggers, bulk-data,
       script runs, start/restart
     - Configurations, log configuration, script upload/delete, updates,
       shutdown
     - Plugin-served routes
   * - ``viewer``
     - ✅
     - ❌
     - ❌
     - ❌
     - ❌
   * - ``operator``
     - ✅
     - ✅
     - ✅
     - ❌
     - ❌
   * - ``configurator``
     - ✅
     - ✅
     - ✅
     - ✅
     - ❌
   * - ``admin``
     - ✅
     - ✅
     - ✅
     - ✅
     - ✅

Clearing faults is ``operator``, not ``admin`` - it is a runtime action, not a
change to how the system is configured. Tearing an entity down
(``PUT /{entity}/status/shutdown`` and ``force-shutdown``) is ``configurator``,
while bringing one up or restarting it is ``operator``.

Where the table comes from
~~~~~~~~~~~~~~~~~~~~~~~~~~

Each route declares its own weakest caller at the point it is registered, and
that single declaration produces both halves of the contract:

* the permission entries ``AuthManager::check_authorization`` matches against,
  and
* the ``security`` requirement the served OpenAPI document publishes for that
  operation - ``GET /api/v1/docs`` names the role each endpoint needs.

Enforcement fails closed: a path no entry matches is refused, so an endpoint's
published role is the role the gateway actually demands rather than a separate
claim about it.

Two consequences worth knowing:

* **Plugin-served routes are ``admin``-only.** A plugin mounts its routes
  directly on the HTTP server, outside the route registry, so no per-route
  declaration describes them. They are covered only by the admin wildcards, and
  the document publishes ``admin`` for them.
* **The document reflects this deployment, not the product.** With
  ``auth.enabled`` false the gateway serves every endpoint unauthenticated and
  the document publishes no roles at all. With it true, ``require_auth_for``
  still decides how much is checked - under ``write`` a GET is served without a
  token even though its operation names the role the table would grant.

Basic Setup
-----------

1. **Create a configuration file** (``auth_params.yaml``):

   .. code-block:: yaml

      ros2_medkit_gateway:
        ros__parameters:
          auth:
            enabled: true
            jwt_secret: "CHANGE-ME-use-openssl-rand-base64-32"
            jwt_algorithm: "HS256"
            token_expiry_seconds: 3600
            refresh_token_expiry_seconds: 86400
            require_auth_for: "write"
            issuer: "ros2_medkit_gateway"
            clients:
              - "admin:REPLACE_WITH_STRONG_SECRET:admin"
              - "operator:REPLACE_WITH_STRONG_SECRET:operator"
              - "viewer:REPLACE_WITH_STRONG_SECRET:viewer"

   .. danger::

      **Never use example credentials in production.** Generate strong secrets with
      ``openssl rand -base64 32`` and keep them out of version control.

2. **Launch with authentication:**

   .. code-block:: bash

      ros2 launch ros2_medkit_gateway gateway.launch.py \
        extra_params_file:=auth_params.yaml

Using Authentication
--------------------

**Step 1: Get an access token**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/auth/authorize \
     -H "Content-Type: application/json" \
     -d '{
       "grant_type": "client_credentials",
       "client_id": "admin",
       "client_secret": "YOUR_ADMIN_SECRET"
     }'

Response:

.. code-block:: json

   {
     "access_token": "eyJhbGciOiJIUzI1NiIs...",
     "token_type": "Bearer",
     "expires_in": 3600,
     "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
     "scope": "admin"
   }

**Step 2: Use the token**

.. code-block:: bash

   TOKEN="eyJhbGciOiJIUzI1NiIs..."

   # Protected endpoint (POST requires auth in "write" mode)
   curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate/executions \
     -H "Authorization: Bearer $TOKEN" \
     -H "Content-Type: application/json" \
     -d '{}'

**Step 3: Refresh the token**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/auth/token \
     -H "Content-Type: application/json" \
     -d '{
       "grant_type": "refresh_token",
       "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g..."
     }'

**Step 4: Revoke a token**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/auth/revoke \
     -H "Content-Type: application/json" \
     -d '{"token": "dGhpcyBpcyBhIHJlZnJlc2g..."}'

Production Recommendations
--------------------------

1. **Use a strong, random secret:**

   .. code-block:: bash

      # Generate a secure secret
      openssl rand -base64 32

2. **Store secrets securely:**

   - Use environment variables or secret management systems
   - Never commit secrets to version control

3. **Use short token expiry:**

   - Access tokens: 15-60 minutes
   - Refresh tokens: 8-24 hours

4. **Consider RS256 for distributed systems:**

   RS256 uses asymmetric keys, allowing verification without sharing the signing key.

   .. code-block:: yaml

      auth:
        enabled: true
        jwt_algorithm: "RS256"
        jwt_secret: "/path/to/private_key.pem"
        jwt_public_key: "/path/to/public_key.pem"

5. **Always use HTTPS in production:**

   See :doc:`https` for TLS configuration.

Troubleshooting
---------------

**"Invalid token" error**

- Check that the token hasn't expired
- Verify the ``Authorization`` header format: ``Bearer <token>``
- Ensure the JWT secret matches between token generation and verification

**"Insufficient permissions" error**

- Check that your client has the required role
- Verify the operation matches the role's permissions

**Token refresh fails**

- Refresh tokens are single-use; get a new one after refresh
- Check that the refresh token hasn't been revoked

See Also
--------

- :doc:`https` - Enable TLS/HTTPS
- `JWT.io <https://jwt.io/>`_ - Debug and verify JWT tokens
