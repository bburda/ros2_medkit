Diagnostic Scripts
==================

This tutorial covers SOVD diagnostic scripts - uploading, managing, and
executing scripts on entities through the gateway REST API.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Diagnostic scripts (SOVD ISO 17978-3, Section 7.15) let you run diagnostic
routines on individual entities. A script is a file - shell or Python - that
the gateway executes as a subprocess when triggered via the REST API. The
gateway tracks each execution's lifecycle and exposes stdout, stderr, and
exit status through a polling endpoint.

Scripts are available on **Components** and **Apps** entity types.

Typical use cases:

- Run a sensor self-test on a specific component
- Collect extended diagnostics that go beyond the standard data endpoints
- Execute a calibration routine on a hardware driver
- Trigger a cleanup or recovery procedure on a misbehaving node

The feature is **disabled by default**. Set ``scripts.scripts_dir`` to a
directory path to enable it. When disabled, all script endpoints return
HTTP 501.

Quick Example
-------------

1. **Enable scripts** in your gateway configuration:

   .. code-block:: yaml

      ros2_medkit_gateway:
        ros__parameters:
          scripts:
            scripts_dir: "/var/ros2_medkit/scripts"

2. **Upload a script** via ``multipart/form-data``:

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/main-computer/scripts \
        -F "file=@check_disk.sh" \
        -F 'metadata={"name": "Disk Check", "description": "Check disk usage and health"}'

   Response (``201 Created``):

   .. code-block:: json

      {"id": "script_1717123456_0", "name": "Disk Check"}

3. **Execute the script**:

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/main-computer/scripts/script_1717123456_0/executions \
        -H "Content-Type: application/json" \
        -d '{"execution_type": "now"}'

   Response (``202 Accepted``):

   .. code-block:: json

      {
        "id": "exec_1717123500_0",
        "status": "running",
        "progress": null,
        "started_at": "2026-01-15T10:25:00Z",
        "completed_at": null,
        "parameters": null,
        "error": null
      }

4. **Poll for completion**:

   .. code-block:: bash

      curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components/main-computer/scripts/script_1717123456_0/executions/exec_1717123500_0

   Response when finished:

   .. code-block:: json

      {
        "id": "exec_1717123500_0",
        "status": "completed",
        "progress": null,
        "started_at": "2026-01-15T10:25:00Z",
        "completed_at": "2026-01-15T10:25:03Z",
        "parameters": {
          "stdout": "Filesystem      Size  Used Avail Use%\n/dev/sda1        50G   32G   18G  64%\n",
          "stderr": "",
          "exit_code": 0
        },
        "error": null
      }

5. **Clean up** the execution record and script when done:

   .. code-block:: bash

      # Delete the execution record
      curl -X DELETE http://localhost:8080/api/v1/components/main-computer/scripts/script_1717123456_0/executions/exec_1717123500_0

      # Delete the uploaded script
      curl -X DELETE http://localhost:8080/api/v1/components/main-computer/scripts/script_1717123456_0

Script Formats
--------------

The gateway detects the script format from the uploaded filename extension and
selects the appropriate interpreter:

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Extension
     - Interpreter
     - Notes
   * - ``.sh``
     - ``sh``
     - POSIX shell. Safest choice for maximum portability.
   * - ``.bash``
     - ``bash``
     - Bash-specific features (arrays, ``[[ ]]``, etc.)
   * - ``.py``
     - ``python3``
     - Python 3. Useful for structured output or complex logic.

Scripts are executed as subprocesses with their own process group. The gateway
captures both stdout and stderr and includes them in the execution result along
with the exit code.

Manifest-Defined Scripts
------------------------

In addition to uploading scripts at runtime, you can pre-deploy scripts by
populating the ``ScriptsConfig.entries`` vector programmatically. This is
the approach used by **ScriptProvider plugins** - they can register a fixed
set of managed scripts that are always available, cannot be deleted through
the API, and appear with ``"managed": true`` in listing responses.

Each manifest entry supports these fields:

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Field
     - Type
     - Description
   * - ``id``
     - string
     - Unique script identifier
   * - ``name``
     - string
     - Human-readable display name
   * - ``description``
     - string
     - What the script does
   * - ``path``
     - string
     - Absolute filesystem path to the script file
   * - ``format``
     - string
     - Interpreter selection: ``sh``, ``bash``, or ``python``
   * - ``timeout_sec``
     - int
     - Per-script timeout override (default: 300)
   * - ``entity_filter``
     - [string]
     - Glob patterns restricting which entities see this script (e.g., ``["components/*"]``). Empty list means all entities.
   * - ``env``
     - map
     - Extra environment variables passed to the subprocess
   * - ``args``
     - array
     - Argument definitions (name, type, flag) for parameterized execution
   * - ``parameters_schema``
     - JSON
     - JSON Schema describing accepted input parameters

Managed scripts are validated at startup - the gateway logs a warning if a
script's ``path`` does not point to an existing regular file.

**Example: plugin providing a managed script**

A ScriptProvider plugin can return pre-defined scripts in its ``list_scripts``
implementation. See :doc:`plugin-system` for the full plugin tutorial.

Execution Lifecycle
-------------------

Every script execution transitions through a defined set of states:

.. code-block:: text

   POST .../executions {"execution_type": "now"}
         |
         v
     +----------+
     | prepared |  (initial, before subprocess starts)
     +----------+
         |
         v
     +---------+
     | running |  (subprocess active)
     +---------+
         |
         +----------------------------+----------------------------+
         |                            |                            |
         v                            v                            v
   +-----------+               +--------+               +------------+
   | completed |               | failed |               | terminated |
   +-----------+               +--------+               +------------+
   (exit code 0)          (non-zero exit or           (stopped by user
                            internal error)            or timeout)

Starting an Execution
~~~~~~~~~~~~~~~~~~~~~

``POST /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions``

Required body field:

- ``execution_type`` (string) - currently ``"now"`` is the supported type

Optional body fields:

- ``parameters`` (object) - input parameters passed to the script
- ``proximity_response`` (string) - proof-of-proximity token for scripts that require physical access

The response is ``202 Accepted`` with a ``Location`` header pointing to the
execution status URL.

Polling Status
~~~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``

Returns the current ``ExecutionInfo`` with:

- ``status`` - one of ``prepared``, ``running``, ``completed``, ``failed``, ``terminated``
- ``progress`` - optional integer (0-100) if the script reports progress
- ``started_at`` / ``completed_at`` - ISO 8601 timestamps
- ``parameters`` - output data including ``stdout``, ``stderr``, and ``exit_code``
- ``error`` - error details if the execution failed

Controlling a Running Execution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``PUT /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``

Send a control action to stop a running script:

.. code-block:: json

   {"action": "stop"}

Supported actions:

- ``stop`` - sends SIGTERM to the subprocess, allowing graceful shutdown
- ``forced_termination`` - sends SIGKILL for immediate termination

Deleting an Execution Record
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``DELETE /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``

Removes a completed, failed, or terminated execution record. Returns ``204 No
Content``. Returns ``409`` if the execution is still running.

.. note::

   The gateway automatically evicts the oldest completed execution records
   when the count exceeds ``scripts.max_execution_history`` (default: 100).

Timeouts
~~~~~~~~

Each execution is subject to a timeout. When the timeout expires, the gateway
terminates the subprocess and sets the execution status to ``terminated``.
Manifest-defined scripts can override the timeout per-script via
``timeout_sec``. Uploaded scripts use the global
``scripts.default_timeout_sec`` (default: 300 seconds).

Concurrency
~~~~~~~~~~~

The gateway enforces a maximum number of concurrent executions across all
entities and scripts. If the limit is reached, new execution requests return
HTTP 429 with error code ``x-medkit-script-concurrency-limit``. The default
limit is 5 (configurable via ``scripts.max_concurrent_executions``).

Security
--------

Disabling Uploads
~~~~~~~~~~~~~~~~~

For hardened deployments that should only run pre-deployed scripts, disable
runtime uploads:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       scripts:
         scripts_dir: "/var/ros2_medkit/scripts"
         allow_uploads: false

When ``allow_uploads`` is ``false``, ``POST .../scripts`` returns HTTP 400.
Pre-deployed manifest-defined scripts remain available for listing and
execution.

RBAC Roles
~~~~~~~~~~

When :doc:`authentication` is enabled (``auth.enabled: true``),
script endpoints are protected by role-based access control:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Role
     - Script Permissions
   * - ``viewer``
     - Read-only: list scripts, get script details, get execution status
   * - ``operator``
     - Viewer permissions plus: start executions, control (terminate) executions, delete execution records
   * - ``configurator``
     - Operator permissions plus: upload scripts, delete scripts
   * - ``admin``
     - All permissions (inherits from configurator)

This means:

- A **viewer** can inspect what scripts are available and check execution
  results, but cannot run or modify anything.
- An **operator** can run diagnostic scripts and stop them, but cannot upload
  new scripts or delete existing ones.
- A **configurator** (or **admin**) has full control over the script library.

File Size Limits
~~~~~~~~~~~~~~~~

Uploaded scripts are limited to ``scripts.max_file_size_mb`` (default: 10 MB).
Uploads exceeding the limit return HTTP 413.

ScriptProvider Plugins
----------------------

The built-in ``DefaultScriptProvider`` stores uploaded scripts on the
filesystem and executes them as POSIX subprocesses. For alternative backends -
such as storing scripts in a database, fetching them from a remote service, or
running them in a sandboxed container runtime - you can implement a
``ScriptProvider`` plugin.

A plugin ScriptProvider replaces the built-in backend entirely. It must
implement all 8 interface methods (list, get, upload, delete, start execution,
get execution, control execution, delete execution). The ``ScriptManager``
wraps all calls with null-safety and exception isolation, so a plugin crash
will not take down the gateway.

See :doc:`plugin-system` for the full plugin development tutorial, including a
ScriptProvider skeleton.

Configuration Reference
-----------------------

All scripts configuration parameters are documented in the server
configuration reference:

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
     - Allow uploading scripts via HTTP.
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
     - Maximum completed executions to keep in memory.

For the full server configuration reference, see :doc:`/config/server`.
