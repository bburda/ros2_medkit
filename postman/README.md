# ROS 2 Medkit Gateway Postman Collection

This folder contains Postman collections for testing the ROS 2 Medkit Gateway REST API.

## API Base Path

All endpoints are prefixed with `/api/v1` for API versioning.

## Collection Contents

**Collection:** `collections/ros2-medkit-gateway.postman_collection.json`

### Authentication Endpoints (JWT/OAuth2)
- ✅ POST `/api/v1/auth/authorize` - Authenticate with client credentials (get access + refresh tokens)
- ✅ POST `/api/v1/auth/token` - Refresh access token using refresh token
- ✅ POST `/api/v1/auth/revoke` - Revoke a refresh token (invalidates all derived access tokens)

### Discovery Endpoints
- ✅ GET `/api/v1/` - Server capabilities and entry points
- ✅ GET `/api/v1/version-info` - Gateway status and version
- ✅ GET `/api/v1/areas` - List all areas
- ✅ GET `/api/v1/areas/{area_id}` - Get area capabilities
- ✅ GET `/api/v1/areas/{area_id}/contains` - List components contained in area
- ✅ GET `/api/v1/components` - List all components with operations and type schemas
- ✅ GET `/api/v1/components/{component_id}` - Get component capabilities
- ✅ GET `/api/v1/components/{component_id}/hosts` - List apps hosted on component (SOVD 7.6.2.4)
- ✅ GET `/api/v1/components/{component_id}/depends-on` - List component dependencies
- ✅ GET `/api/v1/areas/{area_id}/components` - List components in specific area

### Component Data Endpoints
- ✅ GET `/api/v1/components/{component_id}/data` - Read all topic data from a component
- ✅ GET `/api/v1/components/{component_id}/data/{topic_path}` - Read specific topic data
- ✅ PUT `/api/v1/components/{component_id}/data/{topic_path}` - Publish to a topic

### Operations Endpoints (Services & Actions)
- ✅ GET `/api/v1/components/{component_id}/operations` - List all operations (services & actions) with schema info
- ✅ GET `/api/v1/components/{component_id}/operations/{operation_id}` - Get operation details
- ✅ POST `/api/v1/components/{component_id}/operations/{operation_id}/executions` - Execute operation
- ✅ GET `/api/v1/components/{component_id}/operations/{operation_id}/executions` - List executions
- ✅ GET `/api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}` - Get execution status
- ✅ DELETE `/api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}` - Cancel execution

### Configurations Endpoints (ROS 2 Parameters)
- ✅ GET `/api/v1/components/{component_id}/configurations` - List all parameters
- ✅ GET `/api/v1/components/{component_id}/configurations/{param}` - Get parameter value
- ✅ PUT `/api/v1/components/{component_id}/configurations/{param}` - Set parameter value
- ✅ DELETE `/api/v1/components/{component_id}/configurations/{param}` - Reset parameter to default value
- ✅ DELETE `/api/v1/components/{component_id}/configurations` - Reset all parameters to default values

### Faults Endpoints (Fault Management)
- ✅ GET `/api/v1/faults` - List all faults across the system
- ✅ DELETE `/api/v1/faults` - Clear all faults system-wide (extension)
- ✅ GET `/api/v1/components/{component_id}/faults` - List all faults for a component
- ✅ GET `/api/v1/components/{component_id}/faults/{fault_code}` - Get specific fault details
- ✅ DELETE `/api/v1/components/{component_id}/faults/{fault_code}` - Clear a fault

## Quick Start

### 1. Import Collection

**In Postman Desktop:**
1. Click **Import** button (top-left)
2. Select: `postman/collections/ros2-medkit-gateway.postman_collection.json`
3. Click **Import**

### 2. Import Environment

1. Click **Environments** icon (left sidebar)
2. Click **Import**
3. Select: `postman/environments/local.postman_environment.json`
4. Activate environment: Select **"ROS 2 Medkit Gateway - Local"** from dropdown (top-right)

### 3. Start Gateway & Demo Nodes

```bash
# Terminal 1 - Demo Nodes (sensors, actuators, services, actions)
ros2 launch ros2_medkit_integration_tests demo_nodes.launch.py

# Terminal 2 - Fault Manager (required for Faults API and LIDAR Fault Workflow)
ros2 run ros2_medkit_fault_manager fault_manager_node

# Terminal 3 - Gateway
ros2 launch ros2_medkit_gateway gateway.launch.py
```

### 4. Test Endpoints

**Authentication (when auth is enabled):**
1. Expand **"Authentication"** folder
2. Set `client_id` and `client_secret` in environment (default: admin/admin_secret)
3. Click **"POST Authenticate (Client Credentials)"** → **Send**
4. Tokens are automatically saved to environment variables
5. Use `{{access_token}}` in Authorization header for protected endpoints

> **Note:** Auth endpoints are always accessible. By default (`require_auth_for: all`), every request needs a token, GET included.

**Discovery:**
1. Expand **"Discovery"** folder
2. Click **"GET Server Capabilities"** → **Send**
3. Click **"GET List Components"** → **Send** (shows all components with operations)

**Component Data:**
1. Expand **"Component Data"** folder
2. Click **"GET Component Data (All Topics)"** → **Send**
3. Click **"PUT Publish Brake Command"** → **Send** (publishes 50.0 bar to brake actuator)

**Operations:**
1. Expand **"Operations" → "Sync Operations (Services)"**
2. Click **"POST Call Calibrate Service"** → **Send** (calls std_srvs/srv/Trigger)

1. Expand **"Operations" → "Async Operations (Actions)"**
2. Click **"POST Send Action Goal (Long Calibration)"** → **Send**
3. Copy the `goal_id` from response
4. Click **"GET Action Status (Latest)"** → **Send** (shows executing/succeeded)

**Configurations:**
1. Expand **"Configurations"** folder
2. Click **"GET List Component Configurations"** → **Send**
3. Click **"PUT Set Configuration (publish_rate)"** → **Send** (changes temp_sensor rate)

**Faults:**
1. Expand **"Faults"** folder
2. Click **"GET List Component Faults"** → **Send** (shows faults for component)
3. Click **"GET Specific Fault"** → **Send** (gets fault details by code)
4. Click **"DELETE Clear Fault"** → **Send** (clears a fault)

> **Note:** Faults API requires `ros2_medkit_fault_manager` node (see Terminal 2 in startup instructions).
> Faults are reported by ROS 2 nodes via the ReportFault service, not via REST API.

**LIDAR Fault Workflow (Complete Example):**

The collection includes a complete fault management workflow example using the demo LIDAR sensor.
The sensor starts with intentionally invalid parameters that generate faults:
- `LIDAR_RANGE_INVALID` (ERROR): min_range > max_range
- `LIDAR_FREQ_UNSUPPORTED` (WARN): scan_frequency > 20.0 Hz
- `LIDAR_CALIBRATION_REQUIRED` (INFO): sensor not calibrated

1. Expand **"LIDAR Fault Workflow"** folder
2. Execute requests 1-8 in order:
   - **Step 1:** Check initial faults (3 faults expected)
   - **Step 2:** View invalid parameter values
   - **Steps 3a-3c:** Fix parameters via Configurations API
   - **Steps 4a-4b:** Clear parameter-related faults
   - **Step 5:** Run LIDAR calibration service
   - **Step 6:** Clear calibration fault
   - **Step 7:** Verify all faults cleared
   - **Step 8:** View complete fault history

## Authentication

The gateway supports JWT-based authentication with Role-Based Access Control (RBAC).

### Roles

| Role | Permissions |
|------|-------------|
| `viewer` | Read-only access (GET on all endpoints) |
| `operator` | Viewer + trigger operations, clear faults, publish data |
| `configurator` | Operator + modify/reset configurations |
| `admin` | Full access including auth management |

### Auth Configuration

Authentication is configured via ROS 2 parameters:

```yaml
auth:
  enabled: true
  jwt_secret: "your-secret-key"
  jwt_algorithm: "HS256"  # or "RS256" for asymmetric
  token_expiry_seconds: 3600
  refresh_token_expiry_seconds: 86400
  require_auth_for: "write"  # "none", "write", or "all"
  clients:
    - "admin:admin_secret:admin"
    - "operator:operator_secret:operator"
    - "viewer:viewer_secret:viewer"
```

### Using Tokens in Postman

For protected endpoints, add Authorization header:
```
Authorization: Bearer {{access_token}}
```

The collection's auth requests automatically save tokens to environment variables.

## URL Encoding for Topics

Topic paths use standard percent-encoding (`%2F` for `/`):

| Topic Path | URL Encoding |
|-----------|--------------|
| `/powertrain/engine/temperature` | `powertrain%2Fengine%2Ftemperature` |
| `/chassis/brakes/command` | `chassis%2Fbrakes%2Fcommand` |

Example: `GET /api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature`

## Environment Variables

The collection includes two environments:

### HTTP Environment (`local.postman_environment.json`)
- `base_url`: `http://localhost:8080/api/v1` (default HTTP gateway)

### TLS/HTTPS Environment (`local-tls.postman_environment.json`)
- `base_url`: `https://localhost:8443/api/v1` (HTTPS gateway with TLS)

**Common variables (both environments):**
- `client_id`: Client ID for authentication (default: `admin`)
- `client_secret`: Client secret for authentication (default: `admin_secret`)
- `access_token`: JWT access token (auto-populated after authentication)
- `refresh_token`: JWT refresh token (auto-populated after authentication)
- `goal_id`: Used for action status queries (set manually after sending a goal)

## TLS/HTTPS Setup

To test the gateway with TLS enabled:

### 1. Generate Development Certificates

```bash
cd ros2_medkit/src/ros2_medkit_gateway
./scripts/generate_dev_certs.sh ./certs
```

### 2. Start Gateway with TLS

```bash
ros2 launch ros2_medkit_gateway gateway.launch.py \
  server.tls.enabled:=true \
  server.tls.cert_file:=$(pwd)/certs/server.crt \
  server.tls.key_file:=$(pwd)/certs/server.key \
  server.port:=8443
```

### 3. Configure Postman for Self-Signed Certificates

**Option A: Disable SSL Verification (Development Only)**
1. Go to **Settings** (gear icon, top-right)
2. Click **Settings**
3. Scroll to **SSL certificate verification**
4. Toggle **OFF**

**Option B: Add CA Certificate (Recommended)**
1. Go to **Settings** → **Certificates**
2. Under **CA Certificates**, click **Select File**
3. Select `certs/ca.crt` from your generated certificates
4. Requests to `localhost:8443` will now verify against your CA

### 4. Use TLS Environment

1. Import `environments/local-tls.postman_environment.json`
2. Select **"ROS 2 Medkit Gateway - Local TLS"** from environment dropdown
3. All requests will now use `https://localhost:8443/api/v1`

### Mutual TLS (Client Certificates)

If the gateway is configured with `mutual_tls: true`:

1. Go to **Settings** → **Certificates**
2. Click **Add Certificate**
3. Enter:
   - **Host**: `localhost:8443`
   - **CRT file**: `certs/client.crt`
   - **KEY file**: `certs/client.key`
4. Click **Add**

---

**Happy Testing! 🚀**
