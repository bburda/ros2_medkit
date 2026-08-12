Manifest Schema Reference
=========================

This document describes the complete YAML schema for SOVD system manifests.

.. contents:: Table of Contents
   :local:
   :depth: 2

Top-Level Structure
-------------------

A manifest file has the following top-level structure:

.. code-block:: yaml

   manifest_version: "1.0"    # Required - manifest schema version

   metadata:              # Optional - document metadata
     name: string
     version: string
     description: string

   config:                # Optional - discovery behavior settings
                          # (deprecated alias for this key: "discovery:")
     unmanifested_nodes: string
     inherit_runtime_resources: boolean
     allow_manifest_override: boolean

   areas: []              # Optional - area definitions
   components: []         # Optional - component definitions
   assets: []             # Optional - manual asset inventory entries
   apps: []               # Optional - app definitions
   functions: []          # Optional - function definitions
   scripts: []             # Optional - pre-defined script entries

.. note::

   **Unknown top-level keys are ignored, and the gateway says so.** Any
   top-level key outside the set the parser reads
   (``manifest_version``, ``metadata``, ``config``, ``discovery``, ``areas``,
   ``components``, ``assets``, ``apps``, ``functions``, ``scripts``,
   ``capabilities``) is skipped, and the gateway logs a warning naming the key
   and listing the ones it does know. If a whole block of your manifest seems
   to have no effect, that log line is the first place to look - a misspelled
   or misplaced top-level key is the usual cause.

manifest_version (Required)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The manifest schema version. Currently must be ``"1.0"``.

.. code-block:: yaml

   manifest_version: "1.0"

metadata (Optional)
~~~~~~~~~~~~~~~~~~~

Document metadata for identification and documentation.

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Field
     - Type
     - Description
   * - ``name``
     - string
     - System/robot name (e.g., "turtlebot3-nav2")
   * - ``version``
     - string
     - Manifest version (e.g., "1.0.0")
   * - ``description``
     - string
     - Human-readable description

.. code-block:: yaml

   metadata:
     name: "my-robot"
     version: "2.0.0"
     description: "Mobile robot with Nav2 navigation stack"

config (Optional)
~~~~~~~~~~~~~~~~~

Discovery behavior configuration.

Every setting in this block describes how the manifest is combined with what
runtime discovery finds, so none of them does anything in ``manifest_only``
(which parses the block and then runs without the merge pipeline that reads
it) or in ``runtime_only`` (which has no manifest at all).

Within ``hybrid`` mode they differ in what else they need.
``unmanifested_nodes`` and ``inherit_runtime_resources`` are applied by the
runtime linker, which only exists when the runtime layer is enabled.
``allow_manifest_override`` is applied to the manifest layer's own policies,
so it takes effect whenever the manifest layer is built - though with no
runtime layer to merge against there is nothing for it to change.

.. note::

   ``config:`` is the canonical top-level key. ``discovery:`` is accepted as a
   **deprecated alias** for the same block: it is read, the gateway logs a
   deprecation warning naming it, and it will be removed in a future release.
   A manifest that declares both keys uses ``config:`` and ignores
   ``discovery:``; both facts are logged.

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Field
     - Type
     - Description
   * - ``unmanifested_nodes``
     - string
     - Policy for running ROS nodes that the manifest does not declare
       (default: ``warn``)
   * - ``inherit_runtime_resources``
     - boolean
     - Copy the bound node's topics, services and actions onto the linked
       manifest app (default: true)
   * - ``allow_manifest_override``
     - boolean
     - Let manifest values outrank runtime values in the merge (default:
       true). ``false`` is a blanket demotion of the manifest layer - see
       :ref:`manifest-allow-manifest-override` for what it actually changes.

.. _manifest-unmanifested-nodes:

unmanifested_nodes
^^^^^^^^^^^^^^^^^^

A node is *unmanifested* (an *orphan*) when it is present in the ROS graph and
no manifest app binds to it.

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Value
     - Behaviour
   * - ``ignore``
     - Hide them. Every app that did not come from the manifest, the manual
       asset inventory or a plugin is dropped from the entity tree, and
       heuristic Components and Areas sitting in a namespace taken from one of
       the orphan node FQNs are dropped with them. Note the reach: this
       suppresses *every* non-manifest app, not only the ones that were
       classified as orphans.
   * - ``warn``
     - Default. The nodes stay in the entity tree; the gateway logs one
       warning per orphan node.
   * - ``error``
     - The nodes stay in the entity tree, exactly as under ``warn``. The
       gateway logs one error naming the orphan count, and reports the nodes
       on ``GET /api/v1/health`` in the ``warnings`` array under the code
       ``unmanifested_nodes``, with the node fully-qualified names in
       ``ros_node_fqns`` (``entity_ids`` is empty - a node name is not an
       addressable entity id). **It does not fail startup.** The gateway keeps serving
       and ``status`` stays ``"healthy"`` - see :doc:`/api/warning_codes`.
   * - ``include_as_orphan``
     - The same entity tree as ``warn``. The logging differs in shape as
       well as level: ``warn`` emits one warning line per orphan node,
       ``include_as_orphan`` emits a single info line giving the count. No
       entity is tagged ``source: "orphan"`` - nothing in discovery ever
       assigns that value.

The four values are lower-case. ``Warn`` is not ``warn``: it is an
unrecognised value and is treated as one.

An unrecognised value raises validation warning ``R014``, and the parsed
policy falls back to ``warn``. Because ``discovery.manifest_strict_validation``
defaults to ``true`` and turns validation warnings into a load failure, a typo
here rejects the manifest by default; with strict validation off the manifest
loads, runs on ``warn``, and the gateway logs the rejected value along with
the list of valid ones.

.. _manifest-r015:

Malformed structure: ``R015``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``R014`` is about a value the parser understands the *shape* of but not the
*content* of. ``R015`` is the other half: the key is there but its YAML kind
is wrong. It is raised when

- the ``config:`` block (or the deprecated ``discovery:`` alias) is not a
  mapping - a scalar or a sequence; or
- one of the settings inside it is not the kind it must be:
  ``unmanifested_nodes`` not a string, ``inherit_runtime_resources`` or
  ``allow_manifest_override`` not a boolean.

A key that carries no value at all is YAML null and means "not set". That is
never an error: ``unmanifested_nodes:`` with nothing after it loads silently
and the default applies.

.. important::

   **Advisory in this release.** ``R014`` and ``R015`` are reported on the log
   channel only: they do **not** fail the load, whatever
   ``discovery.manifest_strict_validation`` is set to. The manifest loads, the
   offending block or setting is ignored, and its documented default applies.

   This is a deliberate one-release grace period. The settings in this block
   were parsed but never read before, so a manifest carrying
   ``unmanifested_nodes: Warn`` or ``inherit_runtime_resources: 0`` loaded
   without complaint; making them validation warnings immediately would refuse
   that same file on upgrade under the shipped strict default, taking hybrid
   discovery down to ``runtime_only`` for a deployment nobody edited.

   **They become validation warnings in the next release**, at which point the
   strictness dial does apply and a strict gateway rejects a manifest carrying
   either. Fix them now while they are only noisy: grep your gateway log for
   ``[R014]`` and ``[R015]``.

Either way the malformed part costs only itself: the rest of the manifest is
parsed normally rather than the whole file being abandoned.

.. _manifest-inherit-runtime-resources:

inherit_runtime_resources
^^^^^^^^^^^^^^^^^^^^^^^^^

When a manifest app is linked to a running node, the default (``true``) copies
that node's topics, services and actions onto the app, so its ``/data`` and
``/operations`` collections describe what the node really exposes.

With ``false``, the link itself is unchanged - the app still reports
``is_online`` and still names the node it is bound to - but the copy does not
happen, so the app exposes only the resources the manifest declares for it.
An app that declares none therefore serves empty ``/data`` and
``/operations`` collections while still being reported as online.

This holds however the app is named. An app whose ``id`` equals the bound
node's name is one entity by id with the runtime layer's view of that node, so
the merge would otherwise fold the node's live topics and services in before
linking runs; the declared collections are restored after the merge precisely
so the flag means the same thing in both spellings.

.. _manifest-allow-manifest-override:

allow_manifest_override
^^^^^^^^^^^^^^^^^^^^^^^

**What it means.** With the shipped manifest and runtime layers, this flag
controls exactly one thing: whether the manifest claims *exclusivity* over the
hierarchy. ``true`` (the default) means a declared parent-child relationship is
the whole truth and runtime-discovered members are dropped. ``false`` means the
manifest stops claiming exclusivity, so runtime-discovered hosts and children
join the union instead of being discarded.

Against that layer pair it changes nothing else. (A discovery plugin is a
third layer and widens this - see **Mechanism** below.) In particular it does
**not** change
an entity's provenance: ``x-medkit.source`` keeps naming the layer that
declared the entity under every layer policy, because provenance is what the
gateway keys its delete decisions on and is never negotiated between layers.

If you need per-field-group control, do not reach for this flag - it is a
blunt switch. Use ``discovery.merge_pipeline.layers.manifest.<group>`` in the
gateway parameters, which sets one group at a time; the field groups and the
policy values are listed under
:doc:`Merge Policies </config/discovery-options>`. That page documents the
parameters themselves and does not discuss this flag.

**Mechanism.** The flag is shorthand for the per-field-group merge policies of
the manifest layer: ``false`` demotes all five groups - identity, hierarchy,
live_data, status, metadata - to ``fallback``, so the manifest layer only fills
gaps the runtime layer leaves. Four of those five demotions are inert against
the shipped runtime layer, which is why the visible result is narrower than the
mechanism suggests:

- **hierarchy** - the one that shows. Collection fields (``depends_on``, a
  Function's ``hosted_by`` list) become the union of the manifest's and the
  runtime layer's, rather than the manifest's alone. The scalar hierarchy
  fields are unchanged: an App's ``is_located_on``, a Component's ``area``,
  ``parent_component_id``, ``fqn`` and ``namespace``, and an Area's
  ``namespace`` and ``parent_area``.
- **identity**, **live_data**, **status** - no change. The runtime layer is
  already ``fallback`` for identity and already ``authoritative`` for live data
  and status, and a scalar merge behaves identically whether the manifest layer
  outranks the runtime layer or ties with it.
- **metadata** - no change, because ``source`` is the only metadata field the
  runtime layer contributes and provenance is exempt from the policy. This
  demotion becomes visible only when a discovery plugin contributes metadata of
  its own: a ``variant`` or an external-entity classification from the plugin
  layer then wins over the manifest's.

**Precedence.** The blanket demotion is applied *before* the per-group gateway
parameters, so an explicitly configured
``discovery.merge_pipeline.layers.manifest.<group>`` wins over
``allow_manifest_override: false`` for that one group; the other four stay
demoted. Pinning ``hierarchy`` back to ``authoritative`` therefore restores
manifest exclusivity while leaving the rest of the switch in force.

The flag also only matters for an entity that both layers contribute under the
same id. An entity that only the manifest declares is unaffected.

.. code-block:: yaml

   config:
     unmanifested_nodes: warn
     inherit_runtime_resources: true
     allow_manifest_override: true

Areas
-----

Areas represent logical or physical groupings (subsystems, locations, etc.).
In runtime-only mode, areas are derived from ROS 2 namespaces.

.. note::

   The ``areas:`` section is optional. For simple robots without subsystem hierarchy,
   you can omit areas entirely and use components as the top-level entities. See
   :ref:`manifest-flat-entity-tree` below.

Schema
~~~~~~

.. code-block:: yaml

   areas:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       namespace: string       # Optional - ROS 2 namespace path
       category: string        # Optional - classification
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key
       parent_area_id: string  # Optional - parent area reference
       subareas: []            # Optional - nested area definitions

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 15 10 55

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier (alphanumeric, hyphens allowed)
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``namespace``
     - string
     - No
     - ROS 2 namespace path (e.g., "/perception")
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering and grouping
   * - ``translation_id``
     - string
     - No
     - Internationalization key
   * - ``parent_area_id``
     - string
     - No
     - Parent area ID (for flat hierarchy definition)
   * - ``subareas``
     - [Area]
     - No
     - Nested area definitions

Example
~~~~~~~

.. code-block:: yaml

   areas:
     - id: perception
       name: "Perception Subsystem"
       category: "sensor-processing"
       description: "Sensor data acquisition and processing"
       tags:
         - sensors
         - realtime
       subareas:
         - id: lidar-processing
           name: "LiDAR Processing"
           description: "Point cloud processing pipeline"

         - id: camera-processing
           name: "Camera Processing"
           description: "Image processing pipeline"

     - id: navigation
       name: "Navigation Subsystem"
       category: "motion-planning"

Components
----------

Components represent hardware or virtual entities (ECUs, sensors, controllers).
In runtime-only mode, synthetic components are created per namespace to group Apps (nodes).
In manifest mode, components are explicitly defined and Apps are linked to them.

Schema
~~~~~~

.. code-block:: yaml

   components:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       type: string            # Optional - component type
       category: string        # Optional - classification
       area: string            # Optional - reference to area.id
       namespace: string       # Optional - ROS 2 namespace
       fqn: string             # Optional - fully qualified name
       variant: string         # Optional - hardware variant
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key
       parent_component_id: string  # Optional - parent component
       depends_on: [string]    # Optional - component IDs this depends on
       subcomponents: []       # Optional - nested definitions
       external: boolean       # Optional - non-ROS external asset (default: false)

       identity:               # Optional - asset-identity nameplate
         manufacturer: string        # Vendor / manufacturer name
         model: string               # Product designation / order code
         serial_number: string       # Unit serial number
         hardware_revision: string   # Hardware revision
         firmware_version: string    # Firmware version
         software_version: string    # Software/application version
         network_endpoint: string    # e.g. "opc.tcp://plc.local:4840"
         role: string                # Functional role (e.g. "plc", "drive")
         extra:                      # Optional - vendor-specific extras
           <key>: string             # Free-form string map (rack/slot, MAC, asset tag, ...)

       lock:                   # Optional - per-entity lock configuration
         required_scopes: [string]  # Collections requiring a lock before mutation
         breakable: boolean         # Whether locks can be broken (default: true)
         max_expiration: integer    # Max lock TTL in seconds (0 = global default)

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 15 10 53

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``type``
     - string
     - No
     - Component type (sensor, actuator, controller, etc.)
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``area``
     - string
     - No
     - Parent area ID
   * - ``namespace``
     - string
     - No
     - ROS 2 namespace path
   * - ``fqn``
     - string
     - No
     - Fully qualified name (namespace + id)
   * - ``variant``
     - string
     - No
     - Hardware variant identifier
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering
   * - ``translation_id``
     - string
     - No
     - Internationalization key
   * - ``parent_component_id``
     - string
     - No
     - Parent component ID
   * - ``depends_on``
     - [string]
     - No
     - List of component IDs this component depends on
   * - ``subcomponents``
     - [Component]
     - No
     - Nested component definitions
   * - ``external``
     - boolean
     - No
     - True if the component is a non-ROS external asset (PLC, fieldbus device,
       any asset a protocol plugin bridges into SOVD). Tri-state: **omitting**
       ``external:`` leaves it unset, so in hybrid mode it does not clear an
       ``external`` classification contributed by another discovery layer (e.g. a
       protocol plugin). An **explicit** value is authoritative and resolves by
       normal layer priority. An external component with no bound child apps owns
       its fault_manager faults under its own entity id, so a Function or Area
       hosting it rolls up its faults without a synthetic child app (#516).
   * - ``identity``
     - object
     - No
     - Asset-identity nameplate of the asset behind the component. All keys
       are optional strings (``manufacturer``, ``model``, ``serial_number``,
       ``hardware_revision``, ``firmware_version``, ``software_version``,
       ``network_endpoint``, ``role``) plus an extensible ``extra`` string map
       for vendor-specific keys not modeled up front. Each populated field is
       recorded with provenance ``manifest``; protocol plugins (e.g. OPC UA
       device-info) fill in or override fields per the identity merge
       precedence. A live protocol read outranks the manifest only over an
       authenticated session (e.g. an OPC UA secured channel with certificate
       validation); an unauthenticated read only fills fields the manifest
       left empty. Exposed over REST as ``x-medkit.identity``.

Common Component Types
~~~~~~~~~~~~~~~~~~~~~~

- ``sensor`` - Sensors (LiDAR, camera, IMU)
- ``actuator`` - Actuators (motors, grippers)
- ``controller`` - Controllers (main computer, ECU)
- ``accelerator`` - Compute accelerators (GPU, TPU)
- ``communication`` - Communication interfaces
- ``power`` - Power management

Example
~~~~~~~

.. code-block:: yaml

   components:
     - id: main-computer
       name: "Main Computer"
       type: "controller"
       area: control
       description: "Raspberry Pi 4 running ROS 2"
       variant: "rpi4-8gb"
       subcomponents:
         - id: gpu-unit
           name: "GPU Processing Unit"
           type: "accelerator"

     - id: lidar-sensor
       name: "LiDAR Sensor"
       type: "sensor"
       area: perception
       description: "360° laser range finder"
       tags:
         - safety-critical
         - realtime

     - id: imu-sensor
       name: "IMU Sensor"
       type: "sensor"
       area: perception

     # External device component (not a ROS node); owns its faults by entity id
     - id: line-plc
       name: "Line PLC"
       external: true

Assets
------

Assets declare manually inventoried equipment that no protocol layer can
describe (or fully describe): unnetworked devices, third-party hardware, spare
nameplate data. Each asset becomes a Component with ``source: "inventory"`` and
a structured asset identity carrying per-field provenance ``"inventory"``, and
merges into the entity tree by ``id`` alongside protocol-discovered structure.
In the identity merge, ``inventory`` ranks below ``manifest`` and live protocol
reads but above runtime guesses.

Schema
~~~~~~

.. code-block:: yaml

   assets:
     - id: string                 # Required - stable asset id (merge key)
       manufacturer: string       # Optional - vendor / OEM
       model: string              # Optional - model / order code
       serial: string             # Optional - serial number
       hardware_rev: string       # Optional - hardware revision
       firmware: string           # Optional - firmware / software version
       endpoint: string           # Optional - network endpoint (URL / host:port)
       role: string               # Optional - functional role
       area: string               # Optional - Area id placing the asset in the tree
       namespace: string          # Optional - operator-declared placement (sets fqn)
       name: string               # Optional - display name (default: "<manufacturer> <model>")
       description: string        # Optional - detailed description
       variant: string            # Optional - hardware variant identifier
       type: string               # Optional - component type
       translation_id: string     # Optional - internationalization key
       parent_component_id: string # Optional - parent component ID
       depends_on: [string]       # Optional - component IDs this asset depends on
       tags: [string]             # Optional - tags for filtering
       external: boolean          # Optional - non-ROS external asset (default: false)
       any_other_key: string      # Kept verbatim as an identity extra

Fields
~~~~~~

The identity keys accept the same aliases as the CSV import:
``serial_number`` for ``serial``, ``hardware_revision`` / ``hw_rev`` for
``hardware_rev``, and ``firmware_version`` / ``fw`` for ``firmware``. Aliased
keys land on the typed identity fields, not in the extras. Any scalar key not
listed above is preserved as an identity extra.

``external`` classifies the asset as a non-ROS device, the same tri-state field
as on a ``components:`` entry. An external asset with no bound child apps owns
its fault scope by its own entity id. It is a recognized key, so it classifies
the Component instead of being kept as an identity extra.

Placement is optional: without ``area`` (and ``namespace``) the asset is
reachable at ``/components/{id}`` and in the flat component list, but does not
appear under any Area. An ``area`` must reference an area defined in the
manifest, otherwise validation fails (rule R006).

Example
~~~~~~~

.. code-block:: yaml

   areas:
     - id: cell-3
       name: "Cell 3"

   assets:
     - id: hyd-pump-2
       manufacturer: Grundfos
       model: CR-5
       serial_number: "GP-2214-0087"
       area: cell-3
       role: pump
       rack: R2          # kept as identity extra "rack"

CSV Inventory Import
~~~~~~~~~~~~~~~~~~~~

The same asset entries can be bulk-imported from a CSV file via the
``discovery.inventory.csv_path`` gateway parameter (requires a manifest-backed
discovery mode: ``manifest_only``, or ``hybrid`` with
``discovery.manifest_path`` set; empty = disabled). The CSV is re-read on every
manifest load / reload and appended to the merged manifest before validation.

- **Columns**: the header row is matched case-insensitively after trimming.
  Canonical columns are ``id`` (required), ``manufacturer``, ``model``,
  ``serial``, ``hardware_rev``, ``firmware``, ``endpoint``, ``role`` and
  ``area``, with the same aliases as the ``assets:`` list. Any other column is
  kept as an identity extra keyed by its original header.
- **Quoting**: RFC-4180-style; double-quoted fields may contain commas,
  newlines and escaped quotes (``""``). Unquoted fields are whitespace-trimmed;
  a UTF-8 BOM (Excel "CSV UTF-8" export) is stripped.
- **Size cap**: the file is rejected before reading if it exceeds 1 MiB.
- **Row policy**: rows without an ``id`` are skipped with a warning; for
  duplicate ids within the CSV the first row wins. A row whose id is already a
  manifest component keeps the manifest definition and folds the row's identity
  in as gap-fill; a row whose id collides with any other manifest entity is
  skipped, and an unknown ``area`` value is dropped (asset kept, placement-less).
  None of these fail the load. A missing file is skipped with a warning; an
  unreadable or malformed file (e.g. no ``id`` column) fails the load.

Apps
----

Apps represent software applications, typically mapping 1:1 to ROS 2 nodes.
Apps exist only in manifest and hybrid modes.

Schema
~~~~~~

.. code-block:: yaml

   apps:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       category: string        # Optional - classification
       is_located_on: string   # Optional - component ID
       depends_on: [string]    # Optional - app IDs this app depends on
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key
       external: boolean       # Optional - not a ROS node (default: false)

       ros_binding:            # Required for hybrid mode linking
         node_name: string     # Required - ROS node name
         namespace: string     # Optional - namespace (default: /)
         topic_namespace: string  # Optional - match by topic prefix

       lock:                   # Optional - per-entity lock configuration
         required_scopes: [string]  # Collections requiring a lock before mutation
         breakable: boolean         # Whether locks can be broken (default: true)
         max_expiration: integer    # Max lock TTL in seconds (0 = global default)

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 15 10 55

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``is_located_on``
     - string
     - No
     - Component ID where app runs
   * - ``depends_on``
     - [string]
     - No
     - List of app IDs this app depends on
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering
   * - ``translation_id``
     - string
     - No
     - Internationalization key
   * - ``external``
     - boolean
     - No
     - True if not a ROS node (treated as not external when omitted). The
       classification is tri-state internally: **omitting** ``external:`` leaves
       it unset, so in hybrid mode it does not clear an ``external``
       classification contributed by another discovery layer (e.g. a protocol
       plugin) - the app keeps its bare-id fault scope. An **explicit** value is
       authoritative and resolves by normal layer priority, so an authoritative
       manifest ``external: false`` overrides a plugin's ``external: true``.
       Combining ``external: true`` with a ``ros_binding`` is contradictory and
       raises validation warning R013 (the binding is ignored for linking and
       fault scoping - the app is scoped by its entity id).

ros_binding Fields
~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 15 10 53

   * - Field
     - Type
     - Required
     - Description
   * - ``node_name``
     - string
     - Yes*
     - ROS 2 node name to bind to
   * - ``namespace``
     - string
     - No
     - Namespace ("*" for wildcard)
   * - ``topic_namespace``
     - string
     - Yes*
     - Alternative: match by topic prefix

\* Either ``node_name`` or ``topic_namespace`` is required.

**Matching behavior:**

1. **Name and namespace match** (default): ``node_name`` must match exactly.
   ``namespace`` uses path-segment-boundary matching: ``/nav`` matches ``/nav``
   and ``/nav/sub`` but NOT ``/navigation``.
2. **Wildcard namespace**: Set ``namespace: "*"`` to match node in any namespace
3. **Topic namespace**: Match nodes by their published topic prefix

Example
~~~~~~~

.. code-block:: yaml

   apps:
     # Match by exact node name and namespace
     - id: lidar-driver
       name: "LiDAR Driver"
       is_located_on: lidar-sensor
       ros_binding:
         node_name: velodyne_driver
         namespace: /sensors

     # Match node in any namespace
     - id: camera-driver
       name: "Camera Driver"
       ros_binding:
         node_name: usb_cam
         namespace: "*"

     # Match by topic namespace
     - id: perception-pipeline
       name: "Perception Pipeline"
       ros_binding:
         topic_namespace: /perception

     # App with dependencies
     - id: slam-node
       name: "SLAM Node"
       category: "localization"
       is_located_on: main-computer
       depends_on:
         - lidar-driver
         - imu-driver
       ros_binding:
         node_name: slam_toolbox
         namespace: /mapping

     # External app (not a ROS node)
     - id: cloud-connector
       name: "Cloud Connector"
       external: true
       description: "External cloud service integration"

Functions
---------

Functions represent high-level capabilities spanning multiple apps.
Functions are always manifest-defined and aggregate data from their host apps.

Schema
~~~~~~

.. code-block:: yaml

   functions:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       category: string        # Optional - classification
       hosted_by: [string]     # Required - list of app IDs
       depends_on: [string]    # Optional - function IDs
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 15 10 55

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``hosted_by``
     - [string]
     - Yes
     - List of app IDs that implement this function
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``depends_on``
     - [string]
     - No
     - List of function IDs this function depends on
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering
   * - ``translation_id``
     - string
     - No
     - Internationalization key

Function Capabilities
~~~~~~~~~~~~~~~~~~~~~

Functions aggregate capabilities from their host apps:

- **Data**: Combined topics from all host apps
- **Operations**: Combined services/actions from all host apps
- **Faults**: Faults from all host apps

Example
~~~~~~~

.. code-block:: yaml

   functions:
     - id: autonomous-navigation
       name: "Autonomous Navigation"
       category: "mobility"
       description: "Complete autonomous navigation capability"
       tags:
         - safety-critical
         - autonomous
       hosted_by:
         - amcl-node
         - planner-server
         - controller-server
         - bt-navigator

     - id: localization
       name: "Localization"
       category: "state-estimation"
       hosted_by:
         - amcl-node
         - map-server

     - id: perception
       name: "Environment Perception"
       category: "sensing"
       hosted_by:
         - lidar-driver
         - camera-driver
         - point-cloud-processor

Scripts
-------

Scripts define pre-deployed diagnostic scripts that are available on entities.
Scripts defined in the manifest are ``managed`` - they cannot be deleted via the REST API.

.. note::

   Manifest scripts are only loaded when ``scripts.scripts_dir`` is configured in the
   gateway parameters. Without it, the ``scripts:`` block is parsed but scripts are
   not exposed via the REST API.

Schema
~~~~~~

.. code-block:: yaml

   scripts:
     - id: string              # Required - unique identifier
       name: string            # Optional - human-readable name (defaults to id)
       description: string     # Optional - detailed description
       path: string            # Required - filesystem path to script file
       format: string          # Required - execution format (bash, python, sh)
       timeout_sec: integer    # Optional - execution timeout (default: 300)
       entity_filter: [string] # Optional - glob patterns for entity matching
       env:                    # Optional - environment variables
         KEY: "value"
       args:                   # Optional - argument definitions
         - name: string
           type: string
           flag: string
       parameters_schema:      # Optional - JSON Schema for parameters

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 15 10 53

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique script identifier
   * - ``name``
     - string
     - No
     - Human-readable name (defaults to id)
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``path``
     - string
     - Yes
     - Filesystem path to the script file
   * - ``format``
     - string
     - Yes
     - Execution format: ``bash``, ``python``, ``sh``
   * - ``timeout_sec``
     - integer
     - No
     - Max execution time in seconds (default: 300)
   * - ``entity_filter``
     - [string]
     - No
     - Glob patterns for entity matching (e.g., ``components/*``, ``apps/*``). Empty means all entities.
   * - ``env``
     - map
     - No
     - Environment variables passed to the script
   * - ``args``
     - [object]
     - No
     - Argument definitions with ``name``, ``type``, ``flag`` fields
   * - ``parameters_schema``
     - object
     - No
     - JSON Schema for execution parameters validation. Nested objects and arrays are fully supported.

Example
~~~~~~~

.. code-block:: yaml

   scripts:
     - id: run-diagnostics
       name: "Run Diagnostics"
       description: "Check health of all sensors"
       path: "/opt/scripts/run-diagnostics.sh"
       format: "bash"
       timeout_sec: 30
       entity_filter:
         - "components/*"
       env:
         GATEWAY_URL: "http://localhost:8080"

     - id: calibrate-sensor
       name: "Calibrate Sensor"
       path: "/opt/scripts/calibrate.py"
       format: "python"
       timeout_sec: 60
       args:
         - name: threshold
           type: float
           flag: "--threshold"

.. seealso::

   See the Scripts section in :doc:`/api/rest` for API endpoints.

.. _manifest-flat-entity-tree:

Flat Entity Tree
----------------

For simple robots where the entire system is a single unit, you can omit the
``areas:`` section and use a flat component tree instead. The top-level component
represents the robot itself, with subcomponents for hardware modules:

.. code-block:: yaml

   manifest_version: "1.0"

   metadata:
     name: "flat-turtlebot"
     version: "1.0.0"
     description: "TurtleBot3 without area hierarchy"

   # No areas section - components are top-level entities

   components:
     - id: turtlebot3
       name: "TurtleBot3 Burger"
       type: "mobile-robot"

     - id: raspberry-pi
       name: "Raspberry Pi 4"
       type: "controller"
       parent_component_id: turtlebot3

     - id: lds-sensor
       name: "LDS-02 LiDAR"
       type: "sensor"
       parent_component_id: turtlebot3

   apps:
     - id: lidar-driver
       name: "LiDAR Driver"
       is_located_on: lds-sensor
       ros_binding:
         node_name: ld08_driver
         namespace: /

For manifest-based discovery (``manifest_only`` or ``hybrid``), simply omit the
``areas:`` section as shown above - no additional configuration is needed.
In runtime-only discovery, Areas are never created - they come from manifest
only. A complete example is available at
``config/examples/flat_robot_manifest.yaml`` in the gateway package.

Complete Example
----------------

Here's a complete manifest for a TurtleBot3 robot:

.. code-block:: yaml

   manifest_version: "1.0"

   metadata:
     name: "turtlebot3-nav2"
     version: "2.0.0"
     description: "TurtleBot3 with Nav2 navigation stack"

   config:
     unmanifested_nodes: warn
     inherit_runtime_resources: true

   areas:
     - id: perception
       name: "Perception"
       category: "sensor-processing"

     - id: navigation
       name: "Navigation"
       category: "motion-planning"

     - id: control
       name: "Control"
       category: "motion-control"

   components:
     - id: lidar-sensor
       name: "LiDAR Sensor"
       type: "sensor"
       area: perception

     - id: main-computer
       name: "Main Computer"
       type: "controller"
       area: control

   apps:
     - id: lidar-driver
       name: "LiDAR Driver"
       is_located_on: lidar-sensor
       ros_binding:
         node_name: ld08_driver

     - id: amcl-node
       name: "AMCL Localization"
       category: "localization"
       is_located_on: main-computer
       ros_binding:
         node_name: amcl

     - id: planner-server
       name: "Planner Server"
       category: "navigation"
       is_located_on: main-computer
       depends_on:
         - amcl-node
       ros_binding:
         node_name: planner_server

   functions:
     - id: autonomous-navigation
       name: "Autonomous Navigation"
       category: "mobility"
       hosted_by:
         - amcl-node
         - planner-server

   scripts:
     - id: run-diagnostics
       name: "Run Diagnostics"
       path: "/opt/scripts/diagnostics.sh"
       format: "bash"
       timeout_sec: 30
       entity_filter:
         - "components/*"

Validation
----------

Manifests are validated during loading. The validator checks:

**Required fields:**

- ``manifest_version`` must be present and equal to "1.0"
- All entities must have ``id`` and ``name``
- Apps with ``ros_binding`` must have ``node_name`` or ``topic_namespace``
- Functions must have at least one entry in ``hosted_by``
- Scripts must have ``id``, ``path``, and ``format``
- ``format`` must be one of: ``bash``, ``python``, ``sh``

**References:**

- ``area`` references must point to valid area IDs
- ``is_located_on`` must point to valid component IDs
- ``depends_on`` must point to valid app/function IDs
- ``hosted_by`` must point to valid app IDs

**Uniqueness:**

- All entity IDs must be unique within their type
- IDs must be unique across all entity types (areas, components, apps, functions, scripts)

**Format:**

- IDs should contain only alphanumeric characters and hyphens
- IDs should not start with numbers

Validation errors are reported with the path to the invalid field:

.. code-block:: text

   Validation error at apps[2].ros_binding: 'node_name' or 'topic_namespace' required
   Validation error at functions[0].hosted_by[1]: App 'unknown-app' not found

.. seealso::

   - :doc:`/tutorials/manifest-discovery` - User guide for manifest-based discovery
   - :doc:`/tutorials/migration-to-manifest` - Migration guide
