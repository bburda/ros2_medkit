Configuring HTTPS/TLS
=====================

This tutorial shows how to enable TLS (Transport Layer Security) for
encrypted HTTPS communication with the gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

By default, the gateway uses plain HTTP. For production deployments,
you should enable TLS to:

- Encrypt all traffic between clients and the gateway
- Prevent eavesdropping on sensitive data
- Authenticate the server to clients

Quick Start (Development)
-------------------------

For development and testing, use the provided launch file that
auto-generates self-signed certificates:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway_https.launch.py

The gateway will start on ``https://localhost:8443``.

Test with curl (skip certificate verification for self-signed):

.. code-block:: bash

   curl -k https://localhost:8443/api/v1/health

Generating Development Certificates
-----------------------------------

The package includes a helper script:

.. code-block:: bash

   cd ~/ros2_medkit_ws/src/ros2_medkit/src/ros2_medkit_gateway
   ./scripts/generate_dev_certs.sh ./certs

This creates:

- ``ca.crt`` - Certificate Authority certificate
- ``server.crt`` - Server certificate
- ``server.key`` - Server private key
- ``client.crt`` - Client certificate (for mutual TLS)
- ``client.key`` - Client private key

Manual Configuration
--------------------

1. **Create a configuration file** (``tls_params.yaml``):

   .. code-block:: yaml

      ros2_medkit_gateway:
        ros__parameters:
          server:
            host: "0.0.0.0"
            port: 8443
            tls:
              enabled: true
              cert_file: "/path/to/server.crt"
              key_file: "/path/to/server.key"
              min_version: "1.2"

2. **Launch with TLS:**

   .. code-block:: bash

      ros2 launch ros2_medkit_gateway gateway.launch.py \
        extra_params_file:=tls_params.yaml

Configuration Options
---------------------

.. list-table::
   :widths: 30 15 55
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``server.tls.enabled``
     - ``false``
     - Enable/disable TLS
   * - ``server.tls.cert_file``
     - (required)
     - Path to PEM-encoded certificate
   * - ``server.tls.key_file``
     - (required)
     - Path to PEM-encoded private key
   * - ``server.tls.ca_file``
     - ``""``
     - CA that signs client certificates. Setting it turns on mutual TLS and
       makes a client certificate **required**; leave empty for server-only TLS
   * - ``server.tls.min_version``
     - ``"1.2"``
     - Minimum TLS version: ``"1.2"`` or ``"1.3"``. Enforced on the server's own
       SSL context, so it is the floor regardless of what the local OpenSSL
       policy would otherwise allow. Any other value is rejected at startup

Defaults
--------

TLS is **on** in the shipped ``gateway_params.yaml``, and ``cert_file`` and
``key_file`` are empty. A gateway with TLS enabled and no certificate refuses
to start rather than fall back to plaintext, so a first run has to supply one
of the two:

.. code-block:: bash

   # a certificate, for a real deployment or a self-signed pair for a first run
   ros2 launch ros2_medkit_gateway gateway.launch.py \
     cert_file:=/path/to/cert.pem key_file:=/path/to/key.pem

   # or no TLS at all, only on a host nothing else can reach
   ros2 launch ros2_medkit_gateway gateway.launch.py tls_enabled:=false

For a first run on a developer machine, ``scripts/generate_dev_certs.sh``
writes a self-signed certificate and key. Browsers and ``curl`` will refuse it
until you pass the CA explicitly, which is the correct behaviour for a
certificate nothing has vouched for, not a problem to work around in
production.

Mutual TLS
----------

Set ``ca_file`` to the CA that signs your client certificates and the gateway
requires one from **every** client:

.. code-block:: yaml

   server:
     tls:
       enabled: true
       cert_file: "/etc/ros2_medkit/certs/server.pem"
       key_file: "/etc/ros2_medkit/certs/server-key.pem"
       ca_file: "/etc/ros2_medkit/certs/client-ca.pem"

This is all or nothing per gateway. A client that presents no certificate is
rejected during the handshake, before any request is read, and there is no
"verify it only if offered" setting. A client whose certificate is signed by
any other CA is rejected the same way.

.. code-block:: bash

   # without a client certificate: no response, the handshake never completes
   curl --cacert ca.pem https://localhost:8443/api/v1/areas

   # with one signed by ca_file
   curl --cacert ca.pem --cert client.pem --key client-key.pem \
     https://localhost:8443/api/v1/areas

Mutual TLS is transport-level and sits alongside token authentication rather
than replacing it. SOVD authenticates with bearer tokens, so leave ``ca_file``
empty unless every client on that network can be issued a certificate.

Using with curl
---------------

**With CA verification (recommended):**

.. code-block:: bash

   curl --cacert ./certs/ca.crt https://localhost:8443/api/v1/areas

**Skip verification (development only):**

.. code-block:: bash

   curl -k https://localhost:8443/api/v1/areas

Using with Postman
------------------

**Option A: Disable SSL verification (development)**

1. Go to Settings → Settings
2. Find "SSL certificate verification"
3. Toggle OFF

**Option B: Add CA certificate (recommended)**

1. Go to Settings → Certificates
2. Under "CA Certificates", click "Select File"
3. Select your ``ca.crt`` file

Production Certificates
-----------------------

For production, obtain certificates from a trusted Certificate Authority:

**Using Let's Encrypt (free):**

1. Install certbot:

   .. code-block:: bash

      sudo apt install certbot

2. Obtain certificates:

   .. code-block:: bash

      sudo certbot certonly --standalone -d your-domain.com

3. Configure the gateway:

   .. code-block:: yaml

      server:
        tls:
          enabled: true
          cert_file: "/etc/letsencrypt/live/your-domain.com/fullchain.pem"
          key_file: "/etc/letsencrypt/live/your-domain.com/privkey.pem"

Security Best Practices
-----------------------

1. **Protect private keys:**

   .. code-block:: bash

      chmod 600 server.key

2. **Use TLS 1.3 when possible:**

   .. code-block:: yaml

      server:
        tls:
          min_version: "1.3"

3. **Never use self-signed certificates in production**

4. **Rotate certificates before expiry**

5. **Combine with authentication:**

   See :doc:`authentication` for JWT configuration.

Troubleshooting
---------------

**"SSL certificate problem" with curl**

- Use ``-k`` flag for self-signed certs, or
- Provide CA certificate with ``--cacert``

**"Key does not match certificate"**

- Regenerate both certificate and key together

**Connection refused**

- Check the port (8443 for HTTPS, not 8080)
- Verify ``tls.enabled`` is ``true``

See Also
--------

- :doc:`authentication` - JWT authentication
- `Let's Encrypt <https://letsencrypt.org/>`_ - Free TLS certificates
