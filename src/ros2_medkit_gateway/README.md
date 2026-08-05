# ros2_medkit_gateway

HTTP gateway node for the ros2_medkit diagnostics system.

## Overview

The ROS 2 Medkit Gateway exposes ROS 2 system information and data through a RESTful HTTP API. It automatically discovers nodes in the ROS 2 system, organizes them into a SOVD-aligned entity hierarchy (Areas, Components, Apps, Functions), and provides endpoints to query and interact with them.

**Key Features:**
- **Auto-discovery**: Automatically detects ROS 2 nodes and topics
- **SOVD entity model**: Areas, Components (host-level), Apps (ROS 2 nodes), and Functions (namespace-based logical grouping)
- **REST API**: Standard HTTP/JSON interface
- **Incremental entity cache**: Discovery refresh diffs add/remove/change and performs zero structural allocations in the cache layer at steady state (object-pool backed, fixed capacity reserved at init via `entity_cache.capacity`)
- **Real-time updates**: Configurable cache refresh for up-to-date system state
- **Bulk Data Management**: Upload, download, list, and delete bulk data files (calibration, firmware, etc.)
- **Resource Locking**: SOVD-compliant entity locking with scoped access control, lock breaking, and automatic expiry

## Endpoints

All endpoints are prefixed with `/api/v1` for API versioning.

### Discovery Endpoints

- `GET /api/v1/health` - Health check endpoint (returns healthy status)
- `GET /api/v1/` - Gateway status and version information
- `GET /api/v1/version-info` - SOVD version info (supported SOVD versions and base URIs)
- `GET /api/v1/areas` - List all discovered areas (powertrain, chassis, body, root)
- `GET /api/v1/areas/{area_id}` - Get area capabilities
- `GET /api/v1/areas/{area_id}/subareas` - List sub-areas within an area
- `GET /api/v1/areas/{area_id}/contains` - List components contained in an area
- `GET /api/v1/components` - List all discovered components across all areas
- `GET /api/v1/components/{component_id}` - Get component capabilities
- `GET /api/v1/components/{component_id}/subcomponents` - List sub-components
- `GET /api/v1/components/{component_id}/hosts` - List apps hosted on a component
- `GET /api/v1/components/{component_id}/depends-on` - List component dependencies
- `GET /api/v1/areas/{area_id}/components` - List components within a specific area
- `GET /api/v1/apps` - List all discovered apps
- `GET /api/v1/apps/{app_id}` - Get app capabilities
- `GET /api/v1/apps/{app_id}/is-located-on` - Get the component hosting this app
- `GET /api/v1/functions` - List all discovered functions
- `GET /api/v1/functions/{function_id}` - Get function capabilities
- `GET /api/v1/functions/{function_id}/hosts` - List apps grouped by this function

### Component Data Endpoints

- `GET /api/v1/components/{component_id}/data` - Read all topic data from a component
- `GET /api/v1/components/{component_id}/data/{topic_name}` - Read specific topic data from a component
- `PUT /api/v1/components/{component_id}/data/{topic_name}` - Publish data to a topic

### Operations Endpoints (Services & Actions)

- `GET /api/v1/components/{component_id}/operations` - List all services and actions for a component
- `GET /api/v1/components/{component_id}/operations/{operation_id}` - Get operation details
- `POST /api/v1/components/{component_id}/operations/{operation_id}/executions` - Execute operation (call service or send action goal)
- `GET /api/v1/components/{component_id}/operations/{operation_id}/executions` - List all executions for an operation
- `GET /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}` - Get execution status
- `DELETE /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}` - Cancel action execution

### Configurations Endpoints (ROS 2 Parameters)

- `GET /api/v1/components/{component_id}/configurations` - List all parameters for a component
- `GET /api/v1/components/{component_id}/configurations/{param}` - Get parameter value
- `PUT /api/v1/components/{component_id}/configurations/{param}` - Set parameter value
- `DELETE /api/v1/components/{component_id}/configurations/{param}` - Reset parameter to default value
- `DELETE /api/v1/components/{component_id}/configurations` - Reset all parameters to default values

### Bulk Data Endpoints

- `GET /api/v1/{entity}/{id}/bulk-data` - List bulk-data categories (rosbags + configured)
- `GET /api/v1/{entity}/{id}/bulk-data/{category}` - List bulk-data items in a category
- `GET /api/v1/{entity}/{id}/bulk-data/{category}/{item_id}` - Download a bulk-data file
- `POST /api/v1/{entity}/{id}/bulk-data/{category}` - Upload bulk data (components/apps only)
- `DELETE /api/v1/{entity}/{id}/bulk-data/{category}/{item_id}` - Delete bulk data (components/apps only)

### Logging Endpoints

- `GET /api/v1/components/{component_id}/logs` - Query recent log entries for a component (all its nodes, prefix match)
- `GET /api/v1/apps/{app_id}/logs` - Query recent log entries for a specific app node (exact match)
- `GET /api/v1/components/{component_id}/logs/configuration` - Get log configuration for a component
- `GET /api/v1/apps/{app_id}/logs/configuration` - Get log configuration for an app
- `PUT /api/v1/components/{component_id}/logs/configuration` - Update log configuration for a component
- `PUT /api/v1/apps/{app_id}/logs/configuration` - Update log configuration for an app

### Locking Endpoints

- `POST /api/v1/{components|apps}/{id}/locks` - Acquire a lock on an entity
- `GET /api/v1/{components|apps}/{id}/locks` - List active locks on an entity
- `GET /api/v1/{components|apps}/{id}/locks/{lock_id}` - Get lock details
- `PUT /api/v1/{components|apps}/{id}/locks/{lock_id}` - Extend lock expiration
- `DELETE /api/v1/{components|apps}/{id}/locks/{lock_id}` - Release a lock

### Trigger Endpoints

- `POST /api/v1/{entity}/{id}/triggers` - Create a trigger with conditions
- `GET /api/v1/{entity}/{id}/triggers` - List active triggers
- `GET /api/v1/{entity}/{id}/triggers/{trigger_id}` - Get trigger details
- `PUT /api/v1/{entity}/{id}/triggers/{trigger_id}` - Update trigger conditions
- `DELETE /api/v1/{entity}/{id}/triggers/{trigger_id}` - Delete a trigger
- `GET /api/v1/{entity}/{id}/triggers/{trigger_id}/events` - SSE stream of trigger events

### Fault-Trigger Endpoints (threshold rules)

- `GET /api/v1/apps/{app_id}/fault-triggers` - List threshold rules for an app
- `POST /api/v1/apps/{app_id}/fault-triggers` - Create a threshold rule (fires/clears a SOVD fault on value cross)
- `DELETE /api/v1/apps/{app_id}/fault-triggers/{id}` - Delete a rule (clears any fault it asserts)

Two separate collections by design: `/triggers` is the SOVD notification-trigger
CRUD (condition subscriptions delivering SSE events), while `/fault-triggers`
holds server-side threshold rules that raise and clear faults. They share no
schema or store, so the threshold engine is mounted as a sibling instead of
overloading the SOVD `/triggers` contract.

### Scripts Endpoints

- `GET /api/v1/{entity}/{id}/scripts` - List available diagnostic scripts
- `POST /api/v1/{entity}/{id}/scripts` - Upload a new script
- `GET /api/v1/{entity}/{id}/scripts/{script_id}` - Get script details
- `DELETE /api/v1/{entity}/{id}/scripts/{script_id}` - Delete a script
- `POST /api/v1/{entity}/{id}/scripts/{script_id}/executions` - Start script execution
- `GET /api/v1/{entity}/{id}/scripts/{script_id}/executions/{eid}` - Get execution status and output
- `PUT /api/v1/{entity}/{id}/scripts/{script_id}/executions/{eid}` - Send stdin or control signals
- `DELETE /api/v1/{entity}/{id}/scripts/{script_id}/executions/{eid}` - Cancel execution

### API Documentation (OpenAPI)

- `GET /api/v1/docs` - Full OpenAPI 3.1.0 specification
- `GET /api/v1/{entity_type}/{id}/docs` - Entity-scoped OpenAPI spec
- `GET /api/v1/swagger-ui` - Interactive Swagger UI (requires build with `-DENABLE_SWAGGER_UI=ON`)

### Status and Lifecycle Endpoints

- `GET /api/v1/apps/{app_id}/status` - Read app lifecycle status (`ready` or `notReady`)
- `PUT /api/v1/apps/{app_id}/status/start` - Request app start transition
- `PUT /api/v1/apps/{app_id}/status/restart` - Request controlled app restart
- `PUT /api/v1/apps/{app_id}/status/force-restart` - Request forced app restart
- `PUT /api/v1/apps/{app_id}/status/shutdown` - Request controlled app shutdown
- `PUT /api/v1/apps/{app_id}/status/force-shutdown` - Request forced app shutdown
- `GET /api/v1/components/{component_id}/status` - Read component lifecycle status
- `PUT /api/v1/components/{component_id}/status/start` - Request component start transition
- `PUT /api/v1/components/{component_id}/status/restart` - Request controlled component restart
- `PUT /api/v1/components/{component_id}/status/force-restart` - Request forced component restart
- `PUT /api/v1/components/{component_id}/status/shutdown` - Request controlled component shutdown
- `PUT /api/v1/components/{component_id}/status/force-shutdown` - Request forced component shutdown

`GET /status` always returns a response. For apps, the status is read from the ROS 2 lifecycle `GetState` service when the node is a managed lifecycle node (`active` -> `ready`; any other state, or an unreachable/timed-out read, -> `notReady`); plain (unmanaged) nodes fall back to graph presence (`is_online`). For components, the host component is `ready` while the gateway is reachable; any other local component is `notReady` when it hosts apps and all of them are offline, and `ready` otherwise (including when it hosts no apps). `PUT /status/{action}` returns `501 Not Implemented` until a substrate plugin registers a `LifecycleProvider` for the entity. Accepted transitions return `202` with a `Location` header pointing back to `GET /status`.

### Vendor Extension Endpoints

- `GET /api/v1/{entity}/{id}/x-medkit-topic-beacon` - Topic beacon metadata
- `GET /api/v1/{entity}/{id}/x-medkit-param-beacon` - Parameter beacon metadata
- `GET /api/v1/{entity}/{id}/x-medkit-procfs` - Process info (procfs plugin)
- `GET /api/v1/{entity}/{id}/x-medkit-systemd` - Systemd unit status
- `GET /api/v1/{entity}/{id}/x-medkit-container` - Container runtime info
- `GET /api/v1/{entity}/{id}/x-medkit-graph` - ROS 2 graph details

### API Reference

#### GET /api/v1/areas

Lists all discovered areas in the system.

**Example:**
```bash
curl http://localhost:8080/api/v1/areas
```

**Response:**
```json
{
  "items": [
    {
      "id": "powertrain",
      "name": "powertrain",
      "href": "/api/v1/areas/powertrain"
    },
    {
      "id": "chassis",
      "name": "chassis",
      "href": "/api/v1/areas/chassis"
    }
  ]
}
```

#### GET /api/v1/components

Lists all discovered components across all areas.

**Example:**
```bash
curl http://localhost:8080/api/v1/components
```

**Response:**
```json
{
  "items": [
    {
      "id": "my_hostname",
      "name": "my_hostname",
      "href": "/api/v1/components/my_hostname"
    }
  ]
}
```

**Response Fields:**
- `id` - Component identifier (hostname in runtime mode, or manifest-defined)
- `name` - Human-readable name
- `href` - URI to the component capabilities endpoint

#### GET /api/v1/areas/{area_id}/components

Lists all components within a specific area.

**Example (Success):**
```bash
curl http://localhost:8080/api/v1/areas/powertrain/components
```

**Response (200 OK):**
```json
{
  "items": [
    {
      "id": "temp_sensor",
      "name": "temp_sensor",
      "href": "/api/v1/components/temp_sensor"
    },
    {
      "id": "rpm_sensor",
      "name": "rpm_sensor",
      "href": "/api/v1/components/rpm_sensor"
    }
  ]
}
```

**Example (Error - Area Not Found):**
```bash
curl http://localhost:8080/api/v1/areas/nonexistent/components
```

**Response (404 Not Found):**
```json
{
  "error": "Area not found",
  "area_id": "nonexistent"
}
```

**URL Parameters:**
- `area_id` - Area identifier (e.g., `powertrain`, `chassis`, `body`)

**Use Cases:**
- Filter components by domain (only show powertrain components)
- Hierarchical navigation (select area → view its components)
- Area-specific health checks

### Component Data Read Endpoints

#### GET /api/v1/components/{component_id}/data

Read all topic data from a specific component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data
```

**Response (200 OK):**
```json
{
  "items": [
    {
      "topic": "/powertrain/engine/temperature",
      "timestamp": 1732377600000000000,
      "data": {
        "temperature": 85.5,
        "variance": 0.0
      }
    }
  ]
}
```

**Example (Error - Component Not Found):**
```bash
curl http://localhost:8080/api/v1/components/nonexistent/data
```

**Response (404 Not Found):**
```json
{
  "error": "Component not found",
  "component_id": "nonexistent"
}
```

**URL Parameters:**
- `component_id` - Component identifier (e.g., `temp_sensor`, `rpm_sensor`)

**Response Fields:**
- `topic` - Full topic path
- `timestamp` - Unix timestamp (nanoseconds since epoch) when data was sampled
- `data` - Topic message data as JSON object

**Behavior:**
- Returns array of all topics under the component's namespace
- Each topic is sampled once using native rclcpp GenericSubscription
- Empty array `[]` returned if component has no topics
- Configurable timeout per topic (default: 2 seconds)

**Use Cases:**
- Remote diagnostics - Read all sensor values from a component
- System monitoring - Get current state of all component topics
- Data logging - Periodic sampling of component data

**Performance Considerations:**
- Topic sampling uses **parallel execution** with configurable concurrency
- Default: up to 8 topics sampled in parallel (configurable via `data_provider.max_parallel_samples`)
- Response time scales with batch count: `ceil(topics / batch_size) × timeout`
- 3-second timeout per topic to accommodate slow-publishing topics

#### GET /api/v1/components/{component_id}/data/{topic_name}

Read data from a specific topic within a component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data/temperature
```

**Response (200 OK):**
```json
{
  "topic": "/powertrain/engine/temperature",
  "timestamp": 1732377600000000000,
  "data": {
    "temperature": 85.5,
    "variance": 0.0
  }
}
```

**Example (Error - Topic Not Found):**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data/nonexistent
```

**Response (404 Not Found):**
```json
{
  "error": "Topic not found or not publishing",
  "component_id": "temp_sensor",
  "topic_name": "nonexistent"
}
```

**Example (Error - Component Not Found):**
```bash
curl http://localhost:8080/api/v1/components/nonexistent/data/temperature
```

**Response (404 Not Found):**
```json
{
  "error": "Component not found",
  "component_id": "nonexistent"
}
```

**Example (Error - Invalid Topic Name):**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data/invalid-name
```

**Response (400 Bad Request):**
```json
{
  "error": "Invalid topic name",
  "details": "Entity ID contains invalid character: '-'. Only alphanumeric and underscore are allowed",
  "topic_name": "invalid-name"
}
```

**URL Parameters:**
- `component_id` - Component identifier (e.g., `temp_sensor`, `rpm_sensor`)
- `topic_name` - Topic name within the component (e.g., `temperature`, `rpm`)

**Response Fields:**
- `topic` - Full topic path (e.g., `/powertrain/engine/temperature`)
- `timestamp` - Unix timestamp (nanoseconds since epoch) when data was sampled
- `data` - Topic message data as JSON object

**Validation:**
- Both `component_id` and `topic_name` follow ROS 2 naming conventions
- Allowed characters: alphanumeric (a-z, A-Z, 0-9), underscore (_)
- Hyphens, special characters, and escape sequences are rejected

**Use Cases:**
- Read specific sensor value (e.g., just temperature, not all engine data)
- Lower latency than reading all component data
- Targeted monitoring of specific metrics

### Operations Endpoints

#### GET /api/v1/components/{component_id}/operations

List all operations (services and actions) available for a component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/calibration/operations
```

**Response (200 OK):**
```json
{
  "items": [
    {
      "name": "calibrate",
      "path": "/powertrain/engine/calibrate",
      "type": "std_srvs/srv/Trigger",
      "kind": "service",
      "type_info": {
        "schema": "...",
        "default_value": "..."
      }
    }
  ]
}
```

#### POST /api/v1/components/{component_id}/operations/{operation_id}/executions

Execute an operation (call a service or send an action goal).

**Example (Service Call):**
```bash
curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate/executions \
  -H "Content-Type: application/json" \
  -d '{"parameters": {}}'
```

**Response (200 OK - Service):**
```json
{
  "parameters": {
    "success": true,
    "message": "Calibration triggered"
  }
}
```

**Example (Action Goal):**
```bash
curl -X POST http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration/executions \
  -H "Content-Type: application/json" \
  -d '{"parameters": {"order": 10}}'
```

**Response (202 Accepted - Action):**
```json
{
  "id": "abc123def456...",
  "status": "running"
}
```

**Headers:** `Location: /api/v1/components/long_calibration/operations/long_calibration/executions/abc123def456...`

#### GET /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}

Get the status of an action execution.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration/executions/abc123def456
```

**Response (200 OK):**
```json
{
  "status": "running",
  "capability": "execute",
  "parameters": {
    "sequence": [0, 1, 1, 2, 3]
  },
  "x-medkit": {
    "goal_id": "abc123def456...",
    "ros2_status": "executing",
    "ros2": {
      "action": "/powertrain/engine/long_calibration",
      "type": "example_interfaces/action/Fibonacci"
    }
  }
}
```

**Status Values:** `running`, `completed`, `failed`

#### DELETE /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}

Cancel a running action execution.

**Example:**
```bash
curl -X DELETE http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration/executions/abc123def456
```

**Response:** `204 No Content` (empty body) - the cancellation is underway. Also
returned when the cancel response was lost but the action's status stream
already shows the goal cancelling.

**Other outcomes:**

| Status | Meaning |
|--------|---------|
| `400` | The action server answered and refused (`x-medkit-ros2-action-rejected`, `return_code` 1-3) |
| `404` | No such execution (`resource-not-found`) - including one evicted while the request was in flight |
| `500` | The cancel could not be delivered or parsed (`x-medkit-ros2-action-unavailable`) |
| `503` | The cancel service is gone - the action server died (`x-medkit-ros2-action-unavailable`) |
| `504` | No answer within the cancel budget and the status stream does not show the goal cancelling, so the outcome is unknown (`not-responding`) |

The cancel budget is `service_call_timeout_sec` (default 10 s) plus up to 2 s of
cancel-service discovery. On `504`, poll the execution resource above and read
`x-medkit.ros2_status` - the SOVD `status` field renders both `CANCELED` and
`ABORTED` as `failed`, so it cannot tell you whether the cancellation took
effect.

#### PUT /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}

Send a control command to a running execution. ROS 2 actions implement the SOVD
`stop` capability, which maps to action cancel and shares the outcome table
above (with `202 Accepted` in place of `204`, plus `409` when `execute` is
requested on an execution that is still running).

```bash
curl -X PUT -H 'Content-Type: application/json' -d '{"capability": "stop"}' \
  http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration/executions/abc123def456
```

### Authentication Endpoints

#### POST /api/v1/auth/authorize

Authenticate using OAuth2 client credentials flow. Returns access and refresh tokens.

**Example (JSON):**
```bash
curl -X POST http://localhost:8080/api/v1/auth/authorize \
  -H "Content-Type: application/json" \
  -d '{
    "grant_type": "client_credentials",
    "client_id": "my_client",
    "client_secret": "my_secret"
  }'
```

**Example (Form URL-encoded):**
```bash
curl -X POST http://localhost:8080/api/v1/auth/authorize \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d 'grant_type=client_credentials&client_id=my_client&client_secret=my_secret'
```

**Response (200 OK):**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2ggdG9rZW4...",
  "scope": "admin"
}
```

**Response (401 Unauthorized - Invalid Credentials):**
```json
{
  "error": "invalid_client",
  "error_description": "Invalid client credentials"
}
```

#### POST /api/v1/auth/token

Refresh an access token using a refresh token.

**Example:**
```bash
curl -X POST http://localhost:8080/api/v1/auth/token \
  -H "Content-Type: application/json" \
  -d '{
    "grant_type": "refresh_token",
    "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2ggdG9rZW4..."
  }'
```

**Response (200 OK):**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2ggdG9rZW4...",
  "scope": "admin"
}
```

**Note:** The `refresh_token` in the response is the same token that was sent in the request (not a new token). Refresh token rotation is not implemented.

#### POST /api/v1/auth/revoke

Revoke a refresh token to prevent further use.

**Example:**
```bash
curl -X POST http://localhost:8080/api/v1/auth/revoke \
  -H "Content-Type: application/json" \
  -d '{"token": "dGhpcyBpcyBhIHJlZnJlc2ggdG9rZW4..."}'
```

**Response (200 OK):**
```json
{
  "status": "revoked"
}
```

### Configurations Endpoints

#### GET /api/v1/components/{component_id}/configurations

List all parameters for a component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/configurations
```

**Response (200 OK):**
```json
{
  "component_id": "temp_sensor",
  "node_name": "/powertrain/engine/temp_sensor",
  "parameters": [
    {
      "name": "publish_rate",
      "value": 2.0,
      "type": "double"
    },
    {
      "name": "min_temp",
      "value": -40.0,
      "type": "double"
    }
  ]
}
```

#### GET /api/v1/components/{component_id}/configurations/{param}

Get a specific parameter value.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate
```

**Response (200 OK):**
```json
{
  "component_id": "temp_sensor",
  "parameter": {
    "name": "publish_rate",
    "value": 2.0,
    "type": "double"
  }
}
```

#### PUT /api/v1/components/{component_id}/configurations/{param}

Set a parameter value.

**Example:**
```bash
curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate \
  -H "Content-Type: application/json" \
  -d '{"value": 5.0}'
```

**Response (200 OK):**
```json
{
  "status": "success",
  "component_id": "temp_sensor",
  "parameter": {
    "name": "publish_rate",
    "value": 5.0,
    "type": "double"
  }
}
```

#### DELETE /api/v1/components/{component_id}/configurations/{param}

Reset a parameter to its default (initial) value.

**Example:**
```bash
curl -X DELETE http://localhost:8080/api/v1/components/temp_sensor/configurations/min_temp
```

**Response (200 OK):**
```json
{
  "name": "min_temp",
  "value": -40.0,
  "type": "double",
  "reset_to_default": true
}
```

#### DELETE /api/v1/components/{component_id}/configurations

Reset all parameters to their default (initial) values.

**Example:**
```bash
curl -X DELETE http://localhost:8080/api/v1/components/temp_sensor/configurations
```

**Response (200 OK):**
```json
{
  "node_name": "/powertrain/engine/temp_sensor",
  "reset_count": 5,
  "failed_count": 0
}
```

**Response (207 Multi-Status - Partial Success):**
```json
{
  "node_name": "/powertrain/engine/temp_sensor",
  "reset_count": 3,
  "failed_count": 2,
  "failed_parameters": ["read_only_param1", "read_only_param2"]
}
```

### Faults Endpoints

Faults represent errors or warnings reported by system components. The gateway provides access to faults stored in `ros2_medkit_fault_manager`.

- `GET /api/v1/faults` - List all faults across the system (convenience API for dashboards)
- `GET /api/v1/faults/stream` - Real-time fault event stream via Server-Sent Events (SSE)
- `GET /api/v1/faults/{fault_code}/snapshots` - Get topic snapshots captured when fault was confirmed
- `GET /api/v1/faults/{fault_code}/snapshots/bag` - Download rosbag file for fault (if rosbag capture enabled)
- `GET /api/v1/components/{component_id}/faults` - List faults for a specific component
- `GET /api/v1/components/{component_id}/faults/{fault_code}` - Get a specific fault
- `GET /api/v1/components/{component_id}/faults/{fault_code}/snapshots` - Get snapshots for a component's fault
- `DELETE /api/v1/components/{component_id}/faults/{fault_code}` - Clear a fault

#### GET /api/v1/faults

List all faults across the system. This is a convenience API for dashboards and monitoring tools that need a complete system health view without iterating over individual components.

**Query Parameters:**
- `status` - Filter by fault status: `pending`, `confirmed`, `cleared`, `all` (default: `pending` + `confirmed`)

**Example:**
```bash
curl http://localhost:8080/api/v1/faults
curl http://localhost:8080/api/v1/faults?status=all
```

**Response (200 OK):**
```json
{
  "faults": [
    {
      "fault_code": "NAVIGATION_PATH_BLOCKED",
      "severity": 2,
      "severity_label": "ERROR",
      "description": "Navigation failed - cannot find valid path to goal",
      "source_id": "/nav2_fault_reporter",
      "status": "pending",
      "first_occurred": 1735830000,
      "last_occurred": 1735830000,
      "occurrence_count": 1
    }
  ],
  "count": 1
}
```

**Response (400 Bad Request - Invalid Status):**
```json
{
  "error": "Invalid status parameter",
  "details": "Valid values: pending, confirmed, cleared, all",
  "parameter": "status",
  "value": "invalid"
}
```

#### GET /api/v1/faults/stream

Real-time fault event stream using Server-Sent Events (SSE). Clients receive instant notifications when faults are confirmed, updated, or cleared.

**Features:**
- **Real-time notifications**: Events pushed instantly when fault state changes
- **Automatic reconnection**: Supports `Last-Event-ID` header for seamless reconnection
- **Keepalive**: Sends `:keepalive` comment every 30 seconds to prevent timeouts
- **Event buffer**: Buffers up to 100 recent events for reconnecting clients. Under overflow the buffer evicts entries every live client has already received, then `fault_updated` entries superseded by a newer event for the same fault code - a lagging client still converges on the current state of every fault and loses no status transition. Anything beyond that (a superseded transition, or an event with no newer sibling) is genuinely lost for lagging clients: those losses are counted and logged as drops, and an affected client should refetch `GET /api/v1/faults` to resynchronize
- **Entity context (SOVD payload extension)**: When the gateway can resolve the fault's first reporting source back to an entity, the payload carries an `x-medkit` object with `entity_type` and `entity_id` fields so consumers can hit `/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}` directly without enumerating entities
- **Correlation payload**: when a root-cause event auto-clears correlated symptom faults, the payload carries their codes in `auto_cleared_codes` (omitted when empty); those symptoms get no event of their own

**Event Types:**
- `fault_confirmed` - Fault transitioned to CONFIRMED status
- `fault_updated` - Fault data changed (occurrence_count, sources, etc.)
- `fault_cleared` - Fault ended: cleared via ClearFault service, or auto-healed by PASSED events (check `fault.status` for `CLEARED` vs `HEALED`)

**Example:**
```bash
curl -N http://localhost:8080/api/v1/faults/stream
```

**Response (SSE Stream):**
```
:keepalive

id: 1
event: fault_confirmed
data: {"event_type":"fault_confirmed","fault":{"fault_code":"MOTOR_OVERHEAT",...},"timestamp":1735830000.123,"x-medkit":{"entity_type":"apps","entity_id":"motor_controller"}}

id: 2
event: fault_cleared
data: {"event_type":"fault_cleared","fault":{"fault_code":"MOTOR_OVERHEAT",...},"timestamp":1735830060.456,"x-medkit":{"entity_type":"apps","entity_id":"motor_controller"}}
```

**SOVD payload extension `x-medkit.entity_*`** (non-standard, SOVD-compatible): Resolution is best-effort and snapshotted at event arrival. Manifest and hybrid discovery use the linking result; runtime-only discovery falls back to the FQN's last segment and only emits the fields when an App with that ID exists in the cache. When no entity can be resolved (no reporting sources, orphan source, etc.), the entire `x-medkit` object is omitted and the consumer must fall back to discovery.

**Response (503 Service Unavailable - Client Limit Reached):**
```json
{
  "error": "Maximum number of SSE clients reached. Please try again later."
}
```

**Reconnection with Last-Event-ID:**
```bash
curl -N -H "Last-Event-ID: 5" http://localhost:8080/api/v1/faults/stream
```

This replays any buffered events with ID > 5, then continues streaming new events. Replayed `id` sequences may contain holes (e.g. 41, 43, 44) where buffer eviction removed superseded events; ids are monotonically increasing but never guaranteed contiguous. A `Last-Event-ID` beyond the newest issued id is treated as "caught up": the client receives only events published after it connected.

#### GET /api/v1/components/{component_id}/faults

List all faults for a specific component.

**Query Parameters:**
- `status` - Filter by fault status: `pending`, `confirmed`, `cleared`, `all` (default: `pending` + `confirmed`)

**Example:**
```bash
curl http://localhost:8080/api/v1/components/nav2_controller/faults
```

**Response (200 OK):**
```json
{
  "component_id": "nav2_controller",
  "source_id": "/nav2_controller",
  "faults": [...],
  "count": 1
}
```

#### GET /api/v1/faults/{fault_code}/snapshots

Get topic snapshots captured when a fault transitioned to CONFIRMED status. Snapshots provide system state at the moment of fault confirmation for debugging purposes.

**Query Parameters:**
- `topic` - (optional) Filter by specific topic name

**Example:**
```bash
curl http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots
curl http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots?topic=/joint_states
```

**Response (200 OK):**
```json
{
  "fault_code": "MOTOR_OVERHEAT",
  "captured_at": 1735830000.123,
  "topics": {
    "/joint_states": {
      "message_type": "sensor_msgs/msg/JointState",
      "data": {"name": ["joint1"], "position": [1.57]}
    },
    "/cmd_vel": {
      "message_type": "geometry_msgs/msg/Twist",
      "data": {"linear": {"x": 0.5}, "angular": {"z": 0.1}}
    }
  }
}
```

**Response (200 OK - No snapshots):**
```json
{
  "fault_code": "MOTOR_OVERHEAT",
  "topics": {}
}
```

**Response (404 Not Found):**
```json
{
  "error": "Fault not found",
  "fault_code": "NONEXISTENT_FAULT"
}
```

#### GET /api/v1/components/{component_id}/faults/{fault_code}/snapshots

Get topic snapshots for a specific component's fault. Same as the system-wide endpoint but scoped to a component.

**Query Parameters:**
- `topic` - (optional) Filter by specific topic name

**Example:**
```bash
curl http://localhost:8080/api/v1/components/motor_controller/faults/MOTOR_OVERHEAT/snapshots
```

**Response (200 OK):**
```json
{
  "component_id": "motor_controller",
  "fault_code": "MOTOR_OVERHEAT",
  "captured_at": 1735830000.123,
  "topics": {
    "/motor/temperature": {
      "message_type": "sensor_msgs/msg/Temperature",
      "data": {"temperature": 85.5, "variance": 0.1}
    }
  }
}
```

**Snapshot Configuration:**

Snapshots are configured via FaultManager parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `snapshots.enabled` | bool | `true` | Enable/disable snapshot capture |
| `snapshots.background_capture` | bool | `false` | Use background subscriptions (caches latest message) vs on-demand capture |
| `snapshots.timeout_sec` | double | `1.0` | Timeout waiting for topic message (on-demand mode) |
| `snapshots.max_message_size` | int | `65536` | Maximum message size in bytes (larger messages skipped) |
| `snapshots.default_topics` | string[] | `[]` | Topics to capture for all faults |
| `snapshots.config_file` | string | `""` | Path to YAML config file for `fault_specific` and `patterns` |
| `snapshots.recapture_cooldown_sec` | double | `60.0` | Min seconds between captures for the same fault code. |
| `snapshots.max_per_fault` | int | `10` | Max snapshots retained per fault. |
| `snapshots.capture_pool_size` | int | `2` | Max concurrent capture threads under a fault storm (>= 1). Parallelizes snapshot capture only; rosbag stays single-writer (one fault at a time). |
| `snapshots.capture_queue_depth` | int | `16` | Max pending captures before the full-queue policy applies (>= 1). |
| `snapshots.capture_queue_full_policy` | string | `reject_newest` | Policy when the queue is full: `reject_newest` or `drop_oldest`. |

**Topic Resolution Priority:**
1. `fault_specific` - Exact match for fault code (configured via YAML config file)
2. `patterns` - Regex pattern match (configured via YAML config file)
3. `default_topics` - Fallback for all faults

**Example YAML config file** (`snapshots.yaml`):
```yaml
fault_specific:
  MOTOR_OVERHEAT:
    - /joint_states
    - /motor/temperature
patterns:
  "MOTOR_.*":
    - /joint_states
    - /cmd_vel
default_topics:
  - /diagnostics
```

#### GET /api/v1/faults/{fault_code}/snapshots/bag

Download the rosbag file associated with a fault. This endpoint is only available when rosbag capture is enabled in FaultManager.

Rosbag capture provides "black box" style recording - a ring buffer continuously records configured topics, and when a fault is confirmed, the buffer is flushed to a bag file. This allows capturing system state both **before and after** fault confirmation.

**Example:**
```bash
# Download rosbag archive
curl -O -J http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots/bag

# Or save with custom filename
curl http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots/bag -o motor_fault.tar.gz
```

**Response (200 OK):**
- For directory-based bags (default rosbag2 format): compressed tar.gz archive containing the full bag directory with metadata.yaml and all storage segments
- Content-Type: `application/gzip`
- Content-Disposition: `attachment; filename="fault_MOTOR_OVERHEAT_20260124_153045.tar.gz"`

The archive can be extracted and played directly with `ros2 bag play`.

**Response (404 Not Found - Fault or rosbag not found):**
```json
{
  "error": "Rosbag not found",
  "fault_code": "MOTOR_OVERHEAT",
  "details": "No rosbag file associated with this fault"
}
```

**Response (404 Not Found - Rosbag file deleted):**
```json
{
  "error": "Rosbag file not found",
  "fault_code": "MOTOR_OVERHEAT",
  "details": "File was deleted or moved"
}
```

**Rosbag Configuration:**

Rosbag capture is configured via FaultManager parameters. See `config/snapshots.yaml` for full configuration options.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `snapshots.rosbag.enabled` | bool | `false` | Enable/disable rosbag capture |
| `snapshots.rosbag.duration_sec` | double | `5.0` | Ring buffer duration (seconds before fault) |
| `snapshots.rosbag.duration_after_sec` | double | `1.0` | Recording duration after fault confirmed |
| `snapshots.rosbag.topics` | string | `"entity"` | Topic selection: `"entity"` (default; faulting node's topics + `/tf`), `"config"`, `"all"`, or `"explicit"` |
| `snapshots.rosbag.exclude_sensor_topics` | bool | `true` | Auto-exclude image/points/depth/compressed in broad modes (`include_topics` re-adds) |
| `snapshots.rosbag.qos_match` | bool | `true` | Match each topic's publisher QoS for faithful capture |
| `snapshots.rosbag.format` | string | `"sqlite3"` | Bag format: `"sqlite3"` or `"mcap"` |
| `snapshots.rosbag.auto_cleanup` | bool | `true` | Delete bag when fault is cleared |
| `snapshots.rosbag.max_buffer_mb` | int | `256` | Ring-buffer RAM cap (oldest messages drop past it) |
| `snapshots.rosbag.max_bag_size_mb` | int | `50` | Max size per bag file |
| `snapshots.rosbag.max_total_storage_mb` | int | `500` | Total storage limit |

**Enable rosbag capture:**
```bash
ros2 run ros2_medkit_fault_manager fault_manager_node \
  --ros-args -p snapshots.rosbag.enabled:=true \
             -p snapshots.rosbag.duration_sec:=5.0
```

**Playback downloaded rosbag:**
```bash
# Extract the downloaded archive
tar -xzf fault_MOTOR_OVERHEAT_20260124_153045.tar.gz

# Play back the bag
ros2 bag play fault_MOTOR_OVERHEAT_1735830000/

# Inspect bag contents
ros2 bag info fault_MOTOR_OVERHEAT_1735830000/
```

**Differences from JSON Snapshots:**

| Feature | JSON Snapshots | Rosbag Capture |
|---------|----------------|----------------|
| Data format | JSON (human-readable) | Binary (native ROS 2) |
| Time coverage | Point-in-time (at confirmation) | Time window (before + after) |
| Message fidelity | Converted to JSON | Original serialization |
| Playback | N/A | `ros2 bag play` |
| Query via REST | Yes (structured JSON) | Download only |
| Default | Enabled | Disabled |

### Logging Endpoints

The gateway collects `/rosout` messages and exposes them via REST. Each node's log entries are stored in a per-node ring buffer (default: 200 entries, configurable via `logs.buffer_size` in `gateway_params.yaml`).

**Query parameters for GET /logs:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `severity` | string | Minimum severity filter: `debug`, `info`, `warning`, `error`, `fatal` |
| `context`  | string | Substring filter applied to the logger name |

#### GET /api/v1/components/{component_id}/logs

Returns recent log entries for all nodes under the component's namespace (prefix match). Results are capped at `max_entries` (default: 100, configurable per entity via `PUT /logs/configuration`).

```bash
curl http://localhost:8080/api/v1/components/temp_sensor/logs
curl "http://localhost:8080/api/v1/components/temp_sensor/logs?severity=warning"
```

**Response:**

```json
{
  "items": [
    {
      "id": "log_42",
      "timestamp": "2026-03-03T12:00:00.000000000Z",
      "severity": "warning",
      "message": "Temperature exceeded threshold",
      "context": {
        "node": "powertrain/engine/temp_sensor",
        "function": "publish_temperature",
        "file": "temp_sensor_node.cpp",
        "line": 87
      }
    }
  ]
}
```

#### GET /api/v1/apps/{app_id}/logs

Same as above but for a single app node (exact logger name match).

```bash
curl http://localhost:8080/api/v1/apps/temp_sensor/logs
curl "http://localhost:8080/api/v1/apps/temp_sensor/logs?severity=error&context=engine"
```

#### GET /api/v1/{entity}/{id}/logs/configuration

Returns the current log configuration for the entity.

```bash
curl http://localhost:8080/api/v1/components/temp_sensor/logs/configuration
```

**Response:**

```json
{
  "severity_filter": "debug",
  "max_entries": 100
}
```

#### PUT /api/v1/{entity}/{id}/logs/configuration

Updates the log configuration. Both fields are optional; omitted fields are unchanged.

```bash
curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/logs/configuration \
  -H "Content-Type: application/json" \
  -d '{"severity_filter": "warning", "max_entries": 50}'
```

**Request body:**

| Field | Type | Description |
|-------|------|-------------|
| `severity_filter` | string | Minimum severity stored/returned: `debug`, `info`, `warning`, `error`, `fatal` |
| `max_entries` | integer > 0 | Maximum entries returned by GET /logs for this entity |

**Response:** `204 No Content`.

## Quick Start

### Build

```bash
# From your ROS 2 workspace
colcon build --packages-select ros2_medkit_gateway

# Source the workspace
source install/setup.bash
```

### Run

**Start the gateway:**
```bash
ros2 run ros2_medkit_gateway gateway_node
```

**Or use the launch file:**
```bash
ros2 launch ros2_medkit_gateway gateway.launch.py
```

**Start with HTTPS (auto-generates development certificates):**
```bash
ros2 launch ros2_medkit_gateway gateway_https.launch.py
```

**Start demo nodes:**
```bash
ros2 launch ros2_medkit_integration_tests demo_nodes.launch.py
```

**Test the API:**
```bash
# HTTP
curl http://localhost:8080/api/v1/areas

# HTTPS (skip certificate verification for self-signed)
curl -k https://localhost:8443/api/v1/areas
```

## Configuration

The gateway can be configured via parameters in `config/gateway_params.yaml` or through command-line arguments.

### Parameters

#### Server Configuration

| Parameter                    | Type   | Default     | Description                                                                            |
| ---------------------------- | ------ | ----------- | -------------------------------------------------------------------------------------- |
| `server.host`                | string | `127.0.0.1` | Host to bind the REST server (`127.0.0.1` for localhost, `0.0.0.0` for all interfaces) |
| `server.port`                | int    | `8080`      | Port for the REST API (range: 1024-65535)                                              |
| `refresh_interval_ms`        | int    | `30000`     | Safety-backstop refresh interval in milliseconds. Primary refresh is graph-event driven; this controls only the periodic forced refresh (range: 100-60000) |
| `data_provider.max_parallel_samples` | int | `8` | Max concurrent topic samples when fetching data (range: 1-256)                                |
| `topic_sample_timeout_sec`   | float  | `2.0`       | Timeout for sampling topics with active publishers (range: 0.1-30.0)                   |
| `fault_manager.namespace`    | string | `""`        | Optional namespace prefix for fault manager services and events (for example `robot1`)  |
| `fault_manager.service_timeout_sec`  | float  | `5.0`       | Timeout for fault manager service calls such as `list_faults` and `get_snapshots`      |

These defaults reflect the recommended values from `config/gateway_params.yaml`.
If the gateway is run without those parameters, the `DataAccessManager` fallback
default is `topic_sample_timeout_sec=1.0`; the parallel-sample concurrency
falls back to the `Ros2TopicDataProvider` default of 8.

#### SSE (Server-Sent Events) Configuration

| Parameter        | Type | Default | Description                                                                |
| ---------------- | ---- | ------- | -------------------------------------------------------------------------- |
| `sse.max_clients` | int  | `10`    | Maximum concurrent SSE connections. Prevents resource exhaustion attacks. Returns 503 when limit reached. |

#### Native Sampling

The gateway always uses native rclcpp APIs for topic discovery and sampling. This provides significant UX improvements:

- **Instant metadata for idle topics**: Topics without publishers return immediately with metadata (type, schema) instead of waiting for a 3-second timeout
- **Faster discovery**: Uses `node->get_topic_names_and_types()` instead of `ros2 topic list`
- **Reduced external dependencies**: Native APIs for discovery and idle topics. GNU `timeout` is still required for sampling topics with active publishers (install `coreutils` in containers if needed)
- **Better responsiveness**: Robots waiting for commands (many idle topics) respond instantly

CLI is only used for publishing (`ros2 topic pub`), as native publishing requires compile-time type knowledge.

#### CORS Configuration

Cross-Origin Resource Sharing (CORS) settings for browser-based clients. CORS is **enabled automatically** when `allowed_origins` is not empty.

| Parameter                | Type     | Default                      | Description                                                                         |
| ------------------------ | -------- | ---------------------------- | ----------------------------------------------------------------------------------- |
| `cors.allowed_origins`   | string[] | `[]`                         | List of allowed origins (e.g., `["http://localhost:5173"]`). Empty = CORS disabled. |
| `cors.allowed_methods`   | string[] | `["GET", "PUT", "OPTIONS"]`  | HTTP methods allowed for CORS requests                                              |
| `cors.allowed_headers`   | string[] | `["Content-Type", "Accept"]` | Headers allowed in CORS requests                                                    |
| `cors.allow_credentials` | bool     | `false`                      | Allow credentials (cookies, auth headers). Cannot be `true` with wildcard origin.   |
| `cors.max_age_seconds`   | int      | `86400`                      | How long browsers cache preflight response (24 hours default)                       |

#### Authentication Configuration

JWT-based authentication with Role-Based Access Control (RBAC). Authentication is **disabled by default** for backward compatibility.

| Parameter                           | Type     | Default               | Description                                                                 |
| ----------------------------------- | -------- | --------------------- | --------------------------------------------------------------------------- |
| `auth.enabled`                      | bool     | `false`               | Enable/disable authentication. Set to `true` to require auth.              |
| `auth.jwt_secret`                   | string   | (required if enabled) | Secret key for HS256 signing. Must be at least 32 characters.              |
| `auth.jwt_algorithm`                | string   | `HS256`               | JWT signing algorithm: `HS256` (symmetric) or `RS256` (asymmetric).        |
| `auth.token_expiry_seconds`         | int      | `3600`                | Access token lifetime in seconds (range: 60-86400).                        |
| `auth.refresh_token_expiry_seconds` | int      | `86400`               | Refresh token lifetime in seconds (range: 300-604800).                     |
| `auth.require_auth_for`             | string   | `write`               | Auth requirement: `none`, `write` (POST/PUT/DELETE only), or `all`.        |
| `auth.issuer`                       | string   | `ros2_medkit_gateway` | JWT issuer claim for token validation.                                     |
| `auth.clients`                      | string[] | `[]`                  | Client credentials in format `client_id:client_secret:role`.               |

#### TLS/HTTPS Configuration

TLS (Transport Layer Security) enables encrypted HTTPS communication. TLS is **disabled by default** for backward compatibility.

| Parameter                    | Type   | Default | Description                                                                 |
| ---------------------------- | ------ | ------- | --------------------------------------------------------------------------- |
| `server.tls.enabled`         | bool   | `false` | Enable/disable TLS. When enabled, server uses HTTPS instead of HTTP.       |
| `server.tls.cert_file`       | string | (required if enabled) | Path to PEM-encoded certificate file.                         |
| `server.tls.key_file`        | string | (required if enabled) | Path to PEM-encoded private key file.                         |
| `server.tls.ca_file`         | string | `""`    | Optional CA certificate (reserved for future mutual TLS support).          |
| `server.tls.min_version`     | string | `"1.2"` | Minimum TLS version: `"1.2"` (compatible) or `"1.3"` (more secure).        |

> **Note:** Mutual TLS (client certificate verification) is planned for a future release.

**Roles and Permissions:**

| Role         | Read (GET) | Operations (POST) | Configurations (PUT/DELETE) | Faults (DELETE) |
| ------------ | ---------- | ----------------- | --------------------------- | --------------- |
| `viewer`     | ✅          | ❌                 | ❌                           | ❌               |
| `operator`   | ✅          | ✅                 | ❌                           | ❌               |
| `configurator` | ✅        | ✅                 | ✅                           | ❌               |
| `admin`      | ✅          | ✅                 | ✅                           | ✅               |

### Configuration Examples

**Change port via command line:**
```bash
ros2 run ros2_medkit_gateway gateway_node --ros-args -p server.port:=9090
```

**Change host via launch file:**
```bash
ros2 launch ros2_medkit_gateway gateway.launch.py server_host:=0.0.0.0
```

**Enable CORS for local development:**
```yaml
cors:
  allowed_origins: ["http://localhost:5173", "http://localhost:3000"]
  allowed_methods: ["GET", "PUT", "OPTIONS"]
  allowed_headers: ["Content-Type", "Accept"]
  allow_credentials: false
  max_age_seconds: 86400
```

**Enable CORS for production (specific domain):**
```yaml
cors:
  allowed_origins: ["https://dashboard.example.com"]
  allowed_methods: ["GET", "OPTIONS"]
  allowed_headers: ["Content-Type", "Accept", "Authorization"]
  allow_credentials: true
  max_age_seconds: 86400
```

> ⚠️ **Security Note:** Using `["*"]` as `allowed_origins` is not recommended for production. When `allow_credentials` is `true`, wildcard origins will cause the application to fail to start with an exception.

**Use config from another package (`gateway.launch.py`):**

```bash
ros2 launch ros2_medkit_gateway gateway.launch.py \
  config_file:=<your_package>/config/gateway_params.yaml
```

> **Parameter priority:** launch args > `config_file` > packaged defaults. `config_file` overrides only the keys it defines; `server_host`, `server_port`, and `refresh_interval_ms` are launch-arg only and always take precedence.

| Launch Argument       | Default                               | Description                                                                                          |
| --------------------- | ------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `config_file`         | packaged `config/gateway_params.yaml` | Path to a YAML parameter file applied on top of the packaged defaults.                               |
| `server_host`         | `127.0.0.1`                           | Host to bind the REST server (`127.0.0.1` or `0.0.0.0`). Launch-arg only.                           |
| `server_port`         | `8080`                                | Port for the REST API. Launch-arg only.                                                              |
| `refresh_interval_ms` | `30000`                               | Safety-backstop refresh interval in ms (graph events drive the primary refresh). Launch-arg only.    |

### Authentication Configuration Examples

**Enable authentication with write-only protection (recommended for development):**
```yaml
auth:
  enabled: true
  jwt_secret: "your_secret_key_at_least_32_chars_long"
  jwt_algorithm: "HS256"
  token_expiry_seconds: 3600
  refresh_token_expiry_seconds: 86400
  require_auth_for: "write"   # GET requests work without auth
  issuer: "ros2_medkit_gateway"
  clients:
    - "admin:admin_secret:admin"
    - "operator:operator_secret:operator"
    - "viewer:viewer_secret:viewer"
```

**Full protection (all endpoints require authentication):**
```yaml
auth:
  enabled: true
  jwt_secret: "your_production_secret_minimum_32_chars"
  jwt_algorithm: "HS256"
  token_expiry_seconds: 1800    # 30 minutes for tighter security
  refresh_token_expiry_seconds: 43200  # 12 hours
  require_auth_for: "all"      # All endpoints require valid token
  issuer: "production_gateway"
  clients:
    - "dashboard:dashboard_secret_key:admin"
    - "monitoring:monitoring_secret:viewer"
```

**Usage with curl:**
```bash
# 1. Get access token
TOKEN=$(curl -s -X POST http://localhost:8080/api/v1/auth/authorize \
  -H "Content-Type: application/json" \
  -d '{"grant_type":"client_credentials","client_id":"admin","client_secret":"admin_secret"}' \
  | jq -r '.access_token')

# 2. Use token for protected endpoints
curl http://localhost:8080/api/v1/components/temp_sensor/data \
  -H "Authorization: Bearer $TOKEN"

# 3. Set a parameter (requires operator+ role)
curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/rate \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"value": 5.0}'
```

> ⚠️ **Security Notes:**
> - Store `jwt_secret` securely and never commit it to version control
> - Use environment variables or secure secret management in production
> - RS256 algorithm requires additional setup with public/private key files
> - Client secrets should be generated using cryptographically secure random strings

### TLS/HTTPS Configuration Examples

**Generate development certificates:**

The package includes a helper script to generate self-signed certificates for development:

```bash
# Generate certificates in a certs/ directory
./scripts/generate_dev_certs.sh ./certs

# This creates:
# ./certs/ca.crt           - CA certificate
# ./certs/server.crt       - Server certificate
# ./certs/server.key       - Server private key (chmod 600)
# ./certs/client.crt       - Client certificate (for future mutual TLS)
# ./certs/client.key       - Client private key
```

**Basic TLS (HTTPS only):**
```yaml
server:
  host: "0.0.0.0"
  port: 8443
  tls:
    enabled: true
    cert_file: "/path/to/server.crt"
    key_file: "/path/to/server.key"
    min_version: "1.2"
```

**TLS with minimum TLS 1.3 (recommended for new deployments):**
```yaml
server:
  host: "0.0.0.0"
  port: 8443
  tls:
    enabled: true
    cert_file: "/path/to/server.crt"
    key_file: "/path/to/server.key"
    min_version: "1.3"
```

<!-- TODO: Add Mutual TLS example when implemented
**Mutual TLS (client certificate verification):**
```yaml
server:
  host: "0.0.0.0"
  port: 8443
  tls:
    enabled: true
    cert_file: "/path/to/server.crt"
    key_file: "/path/to/server.key"
    ca_file: "/path/to/ca.crt"
    min_version: "1.2"
    mutual_tls: true
```
-->

**Quick start with HTTPS (auto-generates development certificates):**
```bash
# Start gateway with HTTPS - certificates are generated automatically
ros2 launch ros2_medkit_gateway gateway_https.launch.py

# Custom port
ros2 launch ros2_medkit_gateway gateway_https.launch.py server_port:=9443

# Persist certificates to custom directory
ros2 launch ros2_medkit_gateway gateway_https.launch.py cert_dir:=/home/user/certs

# Use TLS 1.3 only
ros2 launch ros2_medkit_gateway gateway_https.launch.py min_tls_version:=1.3
```

**Use config from another package (`gateway_https.launch.py`):**

```bash
ros2 launch ros2_medkit_gateway gateway_https.launch.py \
  config_file:=<your_package>/config/gateway_params.yaml
```

> **Parameter priority:** launch args > `config_file` > packaged defaults. `config_file` overrides only the keys it defines; `cert_dir`, `server_host`, `server_port`, `min_tls_version`, and `refresh_interval_ms` are launch-arg only and always take precedence.

| Launch Argument       | Default                               | Description                                                                              |
| --------------------- | ------------------------------------- | ---------------------------------------------------------------------------------------- |
| `config_file`         | packaged `config/gateway_params.yaml` | Path to a YAML parameter file applied on top of the packaged defaults.                   |
| `cert_dir`            | `/tmp/ros2_medkit_certs`              | Directory for auto-generated certificates. Launch-arg only.                              |
| `server_host`         | `127.0.0.1`                           | Host to bind HTTPS server. Launch-arg only.                                              |
| `server_port`         | `8443`                                | Port for HTTPS API. Launch-arg only.                                                     |
| `min_tls_version`     | `1.2`                                 | Minimum TLS version (`1.2` or `1.3`). Launch-arg only.                                  |
| `refresh_interval_ms` | `30000`                               | Safety-backstop refresh interval (graph events drive primary refresh). Launch-arg only.  |

**Usage with curl (self-signed certs):**
```bash
# Basic HTTPS (skip verification for self-signed)
curl -k https://localhost:8443/api/v1

# HTTPS with CA verification
curl --cacert ./certs/ca.crt https://localhost:8443/api/v1

# TODO: Mutual TLS example - coming in future release
# curl --cacert ./certs/ca.crt \
#      --cert ./certs/client.crt \
#      --key ./certs/client.key \
#      https://localhost:8443/api/v1/areas
```

> ⚠️ **Security Notes:**
> - **Never use self-signed certificates in production** - obtain certificates from a trusted CA
> - Protect private key files with restricted permissions: `chmod 600 server.key`
> - Use TLS 1.3 minimum version when all clients support it
> - Consider using Let's Encrypt for automated certificate management
> - Store certificates outside the source tree and never commit private keys

## Architecture

### Build Layers

The package compiles into two layered static libraries with a strict dependency direction:

- **`gateway_core`** - middleware-neutral business logic. Sources live under `src/core/` and headers under `include/ros2_medkit_gateway/core/`. Links only header-only and C-level externals (cpp-httplib, nlohmann/json, yaml-cpp, tl::expected, jwt-cpp, OpenSSL, SQLite, dl). Carries no rclcpp / rcl_interfaces / message-package dependency. Hosts the neutral HTTP request handlers, JWT authentication, fault model (debounce, storage, cache, correlation), peer aggregation, manifest parsing, the entity cache, the neutral managers (lock, bulk-data, subscription, script, update, plugin), and every provider interface contract.
- **`gateway_ros2`** - ROS adapter layer. Compiles the remaining sources under `src/` and links `gateway_core` publicly via `medkit_target_dependencies`. Hosts `GatewayNode` (the `rclcpp::Node` entry point), the ROS-coupled managers (data access, operation, configuration, fault facade, log, trigger), runtime discovery, the native topic sampler, and the ROS-specific provider default implementations.

The `gateway_node` executable and existing test targets link `gateway_ros2`, so they transitively get both layers from a single dependency. The neutral contract is enforced by two CTest checks:

- `gateway_core_purity` (linter label) - greps `core/` for any ROS-package include and fails on any match.
- `test_gateway_core_smoke` (unit label) - compiles a translation unit that includes a sampling of `core/` headers and links exclusively against `gateway_core` + GTest with no `ament_target_dependencies`. Build failure indicates a transitive ROS coupling that the grep guard might miss.

A third purity gate keeps cpp-httplib out of the plugin ABI. Plugin-facing public headers - the provider interfaces, the `GatewayPlugin` base headers, and the DTOs they include - must not depend on `<httplib.h>`. cpp-httplib is a gateway-internal detail; plugins exchange `nlohmann::json`, typed `dto::` structs, `tl::expected`, and the opaque `PluginRequest`/`PluginResponse` shim across the `.so` boundary, so the gateway and its plugins need not share an httplib version. The httplib-free handler-result vocabulary (`Result`, `NoContent`, `Forwarded`, `ValidatorResult`, `ResponseAttachments`) lives in `http/handler_result.hpp`; only `http/typed_router.hpp` and the handler-internal headers touch httplib. The invariant is enforced by:

- `gateway_plugin_header_purity` (linter label) - runs `scripts/check_headers_httplib_free.sh`, a preprocessor-only scan (`g++ -M -MG`) over the plugin-facing surface that fails on any transitive `httplib.h` dependency. Also wired into the pre-push hook. The build-farm topology (installed gateway, no vendored httplib on the include path) is reproduced locally by `scripts/check_isolated_build.sh`.

### Components

- **Gateway Node**: Main ROS 2 node that runs the REST server
- **Discovery Manager**: Discovers and caches ROS 2 nodes, organizing them into areas and components
- **REST Server**: HTTP server using cpp-httplib
- **Entity Cache**: In-memory cache of discovered areas and components, updated periodically

### Topic-Based Discovery

In addition to standard ROS 2 node discovery, the gateway supports **topic-based discovery** for systems that publish topics without creating discoverable nodes (e.g., NVIDIA Isaac Sim, hardware bridges).

**How it works:**
1. Gateway scans all topics in the ROS 2 graph
2. Extracts unique namespace prefixes (e.g., `/carter1/odom` → `carter1`)
3. Creates virtual "components" for namespaces that have topics but no nodes
4. These components have `"source": "topic"` to distinguish them from node-based components

**Example:** Isaac Sim publishes topics like `/carter1/odom`, `/carter1/cmd_vel`, `/carter2/imu` without creating ROS 2 nodes. The gateway discovers:
- Component `carter1` with topics: `/carter1/odom`, `/carter1/cmd_vel`
- Component `carter2` with topics: `/carter2/imu`

**System topic filtering:** The following topics are filtered out during discovery:
- `/parameter_events`, `/rosout`, `/clock`
- Note: `/tf` and `/tf_static` are NOT filtered (useful for diagnostics)

### Entity Organization

In runtime discovery mode, the gateway maps the ROS 2 graph to the SOVD entity model:

- **Component**: A single host-derived Component is created from `HostInfoProvider` (hostname, OS, architecture). All Apps belong to this Component.
- **App**: Each discovered ROS 2 node becomes an App entity.
- **Function**: The first namespace segment creates a Function entity that groups all Apps under that namespace (e.g., `/powertrain/engine/temp_sensor` and `/powertrain/engine/rpm_sensor` both belong to Function `powertrain`).
- **Area**: Areas are only created from manifest definitions. They are never auto-generated in runtime mode. Use hybrid or manifest-only mode to organize entities into Areas.

```
Runtime mode mapping:
/powertrain/engine/temp_sensor  -> Component: <hostname>, Function: powertrain, App: temp_sensor
/chassis/brakes/pressure_sensor -> Component: <hostname>, Function: chassis, App: pressure_sensor
/body/lights/controller         -> Component: <hostname>, Function: body, App: controller
/standalone_node                -> Component: <hostname>, App: standalone_node
```

### Asset Identity (nameplate) and merge-by-identity

Each Component can carry an **asset identity**: the stable nameplate of the physical
or logical asset it represents (an ECU, PLC, drive, sensor), separate from its runtime
ROS 2 footprint. The model lives in `core/discovery/models/asset_identity.hpp`.

Typed fields (queryable, AAS-mappable) plus a generic `extra` key-value map for
vendor-specific attributes we do not model up front:

| Field | JSON key | AAS Nameplate (IDTA 02006) |
|-------|----------|----------------------------|
| `manufacturer` | `manufacturer` | ManufacturerName |
| `model` (order code) | `model` | ManufacturerProductDesignation |
| `serial_number` | `serialNumber` | SerialNumber |
| `hardware_revision` | `hardwareRevision` | HardwareVersion |
| `firmware_version` | `firmwareVersion` | FirmwareVersion |
| `software_version` | `softwareVersion` | SoftwareVersion |
| `network_endpoint` | `networkEndpoint` | Asset Interface Description (AID) endpoint |
| `role` | `role` | functional role (BoM/usage context) |
| `extra` | `extra` | additional Nameplate properties |

The whole block maps to an Asset Administration Shell later without rework: the asset
itself is the shell (`Component.id` as localId), this block is the **Nameplate** submodel,
the Component hierarchy is the **BoM** submodel, and `network_endpoint` is an **AID** entry.

Identity is emitted under `x-medkit.identity` only when populated, so identity-less
components keep their existing JSON unchanged.

**Why typed fields and a map?** Typed fields are the common, queryable, standard-mappable
attributes; the `extra` map is the escape hatch for vendor keys (rack/slot, MAC, asset tag)
that do not warrant a schema change. Both merge with the same rules.

**Merge-by-identity with provenance.** A single asset is usually described by several
sources (a hand-authored manifest, a live protocol device-info read, runtime discovery).
`merge_identity()` (`core/discovery/identity_merge.hpp`) combines them field by field and
records **per-field provenance** (which source set each field) under `_provenance`.

- **Precedence** is config-driven (`IdentityMergeConfig::source_precedence`, highest first)
  and is deliberately **decoupled from the structural `MergePolicy`**: a manifest can be the
  authoritative *structure* source while a live protocol read is the authoritative *identity*
  source. Authority is ranked on each contributing entity's canonical `Component.source` tag
  ("manifest", "inventory", "plugin", "runtime", "node", "config", or a protocol-class tag a
  provider sets such as "opcua"), **not** the free-form discovery-layer name. Default order:
  protocol device-info (`opcua`, `s7`, `ethernet_ip`, `modbus`, `ads`, `profinet`, and the
  generic `plugin`) > `manifest` > `inventory` (CSV import / manifest `assets:` list) >
  `config` > runtime sources. A higher-authority source overrides a field; lower-authority
  sources only fill gaps; unknown sources rank lowest. Empty values never overwrite.

Components are correlated for merging by `Component.id`; identity is merged whenever two
sources contribute the same Component id (in the discovery pipeline, and gap-filled across
peer aggregation).

The pipeline calls `merge_identity` inside the `IDENTITY` field group for Components, seeding
provenance with the base (highest-priority) layer's source tag. Configure via
`MergePipeline::set_identity_merge_config()`; precedence entries must equal the canonical
`Component.source` values you expect at runtime.

## Demo Nodes

The package includes demo automotive nodes for testing:

| Node              | Namespace               | Description               |
| ----------------- | ----------------------- | ------------------------- |
| `temp_sensor`     | `/powertrain/engine`    | Engine temperature sensor |
| `rpm_sensor`      | `/powertrain/engine`    | Engine RPM sensor         |
| `pressure_sensor` | `/chassis/brakes`       | Brake pressure sensor     |
| `status_sensor`   | `/body/door/front_left` | Door status sensor        |
| `actuator`        | `/chassis/brakes`       | Brake actuator            |
| `controller`      | `/body/lights`          | Light controller          |
| `calibration`     | `/powertrain/engine`    | Calibration service       |

**Launch all demo nodes:**
```bash
ros2 launch ros2_medkit_integration_tests demo_nodes.launch.py
```

## URL Encoding

Topic names in URLs use standard percent-encoding (`%2F`) for the forward slash character:

| ROS 2 Topic Path                | URL Encoding                                |
| ------------------------------- | ------------------------------------------- |
| `/powertrain/engine/temperature`| `powertrain%2Fengine%2Ftemperature`         |
| `/chassis/brakes/command`       | `chassis%2Fbrakes%2Fcommand`                |
| `/body/door/front_left/status`  | `body%2Fdoor%2Ffront_left%2Fstatus`         |

**Example:**
- Topic: `/powertrain/engine/temperature`
- URL: `/api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature`

> **Note:** Component IDs do not require encoding (they are simple names like `temp_sensor`).
> The leading slash is stripped before encoding.

## Manual API Testing with Postman

We provide a Postman collection for easy API testing:

1. **Import collection:** `postman/collections/ros2-medkit-gateway.postman_collection.json`
2. **Import environment:** `postman/environments/local.postman_environment.json`
3. **Activate environment:** Select "ROS 2 Medkit Gateway - Local" in Postman
4. **Start gateway:** `ros2 launch ros2_medkit_gateway gateway.launch.py`
5. **Start demo nodes:** `ros2 launch ros2_medkit_integration_tests demo_nodes.launch.py`
6. **Test:** Send requests from Postman!

See [postman/README.md](postman/README.md) for detailed instructions.

## Multi-Instance Aggregation

Federate multiple gateway instances into a single unified API. A primary gateway merges entities from peer gateways and transparently forwards requests for remote entities.

**Quick Start:**

```yaml
# gateway_params.yaml
ros2_medkit_gateway:
  ros__parameters:
    aggregation:
      enabled: true
      peer_urls: ["http://localhost:8081"]
      peer_names: ["subsystem_b"]
```

```bash
# Start primary gateway with aggregation
ros2 run ros2_medkit_gateway gateway_node --ros-args \
    --params-file gateway_params.yaml

# Query merged entity tree
curl http://localhost:8080/api/v1/components | jq
```

**Key features:**
- **Static peers**: List known gateways in config
- **mDNS auto-discovery**: Zero-configuration peer discovery on local network
- **Type-aware merging**: Areas, Functions, and Components merge by ID; Apps get peer-name prefixes on collision
- **Transparent forwarding**: Requests for remote entities forwarded to owning peer
- **Graceful degradation**: Unhealthy peers excluded, partial results clearly marked

See the [documentation](https://selfpatch.github.io/ros2_medkit/) for detailed configuration reference and tutorials.

## Testing

### Run Integration Tests

```bash
# Build with tests
colcon build --packages-select ros2_medkit_gateway

# Run tests
colcon test --packages-select ros2_medkit_gateway --event-handlers console_direct+

# View test results
colcon test-result --verbose
```

### Subscription Architecture Regression Gate

All rcl subscription create/destroy calls must go through
`Ros2SubscriptionSlot::create_typed` / `create_generic` (defined in
`ros2_common/`). Calling `node->create_subscription<T>()`,
`create_generic_subscription()`, or `create_callback_group()` anywhere
else triggers the race from issue #375 and SIGSEGVs on Rolling/Lyrical under
concurrent HTTP load.

The regression gate is `scripts/check_no_naked_subscriptions.sh`. It
runs in CI (`quality.yml`) and as a pre-commit hook. If your PR
adds a naked subscription call the gate fails with the file:line,
the class to use instead, and a pointer to
[ros2_subscription_architecture.rst](design/ros2_subscription_architecture.rst)
for the full rationale and example.

Five legacy call sites are currently on the allowlist
(`sse_fault_handler.cpp`, `trigger_fault_subscriber.cpp`,
`trigger_topic_subscriber.cpp`, `operation_manager.cpp`,
`log_manager.cpp`) pending migration to the slot pattern.

## SOVD Compliance

| Requirement     | Status   | Notes                                                                  |
|-----------------|----------|------------------------------------------------------------------------|
| REQ_INTEROP_076 | verified | `GET /status` - returns entity lifecycle status from cache or provider |
| REQ_INTEROP_077 | open (route registered, returns 501) | `PUT /status/start` route registered; returns 501 until a `LifecycleProvider` plugin registers for the entity |
| REQ_INTEROP_078 | open (route registered, returns 501) | `PUT /status/restart` - same as above                                  |
| REQ_INTEROP_079 | open (route registered, returns 501) | `PUT /status/force-restart` - same as above                            |
| REQ_INTEROP_080 | open (route registered, returns 501) | `PUT /status/shutdown` - same as above                                 |
| REQ_INTEROP_081 | open (route registered, returns 501) | `PUT /status/force-shutdown` - same as above                           |

Requirements 077-081 will be fully verified when a substrate plugin (ROS 2 lifecycle node manager, process supervisor, or container manager) implements the `LifecycleProvider` interface and registers it via the plugin manager.

## License

Apache-2.0
