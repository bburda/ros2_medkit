# ros2_medkit

[![CI](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/selfpatch/ros2_medkit/branch/main/graph/badge.svg)](https://codecov.io/gh/selfpatch/ros2_medkit)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2 Jazzy | Humble | Lyrical](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Humble%20%7C%20Lyrical-blue)](https://docs.ros.org/en/jazzy/)
[![Discord](https://img.shields.io/badge/Discord-Join%20Us-7289DA?logo=discord&logoColor=white)](https://discord.gg/6CXPMApAyq)

<p align="center">
  <img src="docs/_static/images/medkit_hero.gif" alt="A Nav2 NavigateToPose goal aborts: stock ROS 2 diagnostics leave you a log line lost in the noise, while ros2_medkit turns it into one structured fault over REST" width="100%">
</p>

<p align="center">
  <b>A structured diagnostic model for ROS 2 robots, over REST.</b><br>
  Drop it next to the stack you already run - no code changes - and a failure that ROS 2
  diagnostics leaves as a log line becomes one structured fault: a fault code on a SOVD entity
  tree, with lifecycle, history, and the state captured at the moment it happened.
</p>

## Drop it into the stack you already run

### Nav2: a navigation goal that quietly aborts

You run Nav2. A `NavigateToPose` goal aborts - the planner gives up, the robot is wedged in a
corner - and the only trace is a line buried in a log you would have to SSH in to read.

Start ros2_medkit next to it (no changes to Nav2). The aborted goal becomes a fault:

```bash
curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/bt_navigator/faults
# → ACTION_NAVIGATE_TO_POSE_ABORTED  severity=ERROR  source=/bt_navigator  status=CONFIRMED
#   + a black-box rosbag of the seconds around the failure
```

It works because an aborted action goal is often the *only* failure signal Nav2 emits, and
ros2_medkit's action bridge turns that into a structured fault on the `bt_navigator` entity -
no instrumentation, no callbacks added to Nav2.

> [!TIP]
> Nothing was added to Nav2. The bridge reads the action status your stack already publishes,
> so the same trick works for **any** action server - MoveIt, Nav2, or your own nodes.

<details>
<summary><b>↳ Another example: MoveIt - a motion that fails to execute</b></summary>

<br>

You run MoveIt. A `MoveGroup` goal aborts - no valid plan, or the controller rejects the
trajectory. Same story: the same action bridge surfaces the aborted move as a fault on the
`move_group` entity, with the freeze-frame of what the arm was doing, without touching MoveIt.

```bash
curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/move_group/faults
# → the aborted MoveGroup goal, as a structured fault with its snapshot
```

</details>

**Two ways to feed it:**

- **Native, for code you own** - report faults directly with the
  [`FaultReporter`](https://github.com/selfpatch/ros2_medkit/tree/main/src/ros2_medkit_fault_reporter)
  client. This is the richest path (your own codes, severities and context) and the canonical way
  for new code; see the
  [integration tutorial](https://selfpatch.github.io/ros2_medkit/tutorials/integration.html).
- **Drop-in bridges, for the stack you will not rewrite** - most real robots run huge existing
  projects nobody is going to retrofit with diagnostics. Point the bridges at what they already
  emit and you get structured faults (and states) in minutes, zero code changes:
  [`/diagnostics`](https://github.com/selfpatch/ros2_medkit/tree/main/src/ros2_medkit_diagnostic_bridge),
  [`/rosout` logs](https://github.com/selfpatch/ros2_medkit/tree/main/src/ros2_medkit_log_bridge),
  [aborted actions](https://github.com/selfpatch/ros2_medkit/tree/main/src/ros2_medkit_action_status_bridge).

So you get a remote, queryable, time-traveled fault instead of a log line - whether or not you
touch the node's code.

## Run it in 5 minutes

**One command. No code changes, no config.** Point a container at your already-running robot - it
ships the gateway, the fault manager and the drop-in bridges, and speaks whatever DDS your stack
speaks (Fast DDS and CycloneDDS are both baked in):

```bash
docker run --rm --network host --ipc host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  ghcr.io/selfpatch/ros2_medkit-jazzy:latest \
  ros2 launch ros2_medkit_gateway bringup.launch.py
# → REST API live at http://localhost:8080/api/v1/
```

Swap `jazzy` for `humble`/`lyrical`; the two `-e` flags forward your shell's `ROS_DOMAIN_ID` and
`RMW_IMPLEMENTATION` (falling back to domain `0` / Fast DDS if unset) so the container joins the
same DDS graph as your robot. The packages are heading into the ROS index, so once your distro
picks them up it is a plain apt install too:

```bash
sudo apt install ros-jazzy-ros2-medkit-gateway   # or ros-humble- / ros-lyrical-
ros2 launch ros2_medkit_gateway bringup.launch.py
```

> [!TIP]
> That is the whole setup. It auto-discovers every node, topic, service and action and starts
> emitting structured faults over REST - no instrumentation, no changes to your stack. Prefer
> source or Pixi? See the [installation docs](https://selfpatch.github.io/ros2_medkit/installation.html).

> [!NOTE]
> The log and action-status bridges are on by default; the `/diagnostics` bridge is opt-in. If your
> stack publishes `/diagnostics` via `diagnostic_updater`, turn it on with
> `ros2 launch ros2_medkit_gateway bringup.launch.py enable_diagnostic_bridge:=true`.

**Then just curl the REST API** (no UI, no CORS):

```bash
# Your whole graph as a SOVD entity tree, instantly:
curl localhost:8080/api/v1/apps

# The headline - structured faults (code, severity, source node, lifecycle, history):
curl localhost:8080/api/v1/faults

# Watch faults arrive live (Server-Sent Events):
curl -N localhost:8080/api/v1/faults/stream

# One fault's full context (freeze-frame snapshots + black-box rosbag reference):
curl localhost:8080/api/v1/apps/bt_navigator/faults/ACTION_NAVIGATE_TO_POSE_ABORTED

# Download the black-box rosbag captured around that fault:
curl -O -J localhost:8080/api/v1/apps/bt_navigator/bulk-data/rosbags/ACTION_NAVIGATE_TO_POSE_ABORTED
```

Full API: **Swagger UI at http://localhost:8080/api/v1/docs** · [Postman collection](postman/) ·
[API reference](https://selfpatch.github.io/ros2_medkit/).

**What you get the moment you attach** - no instrumentation, no code changes (right after a
Nav2 `NavigateToPose` goal aborts):

```jsonc
// GET /api/v1/faults
{
  "items": [
    { "fault_code": "ACTION_NAVIGATE_TO_POSE_ABORTED",
      "severity_label": "ERROR", "status": "CONFIRMED",
      "reporting_sources": ["/bt_navigator"] }
  ],
  "x-medkit": { "count": 1 }
}
```

<details>
<summary><b>Prefer a dashboard? Run the web UI alongside it (optional)</b></summary>

<br>

```bash
docker run -p 3000:80 ghcr.io/selfpatch/ros2_medkit_web_ui:latest
# open http://localhost:3000 -> Connect -> http://localhost:8080
```

The browser calls the gateway from a different origin, so the gateway must allow that origin via
CORS (the prebuilt gateway Docker image enables it; for a native bringup set
`cors.allowed_origins`). See the [web UI tutorial](https://selfpatch.github.io/ros2_medkit/tutorials/web-ui.html).

</details>

For a guided walkthrough, see the
[Getting Started tutorial](https://selfpatch.github.io/ros2_medkit/getting_started.html).

## vs. standard ROS 2 diagnostics

`diagnostic_updater` + `/diagnostics` + `diagnostic_aggregator` + `rqt_robot_monitor` report
current node health to a desktop GUI. ros2_medkit turns that into a queryable, remote,
time-traveled, actionable fault.

> [!NOTE]
> It **consumes `/diagnostics` too**, so it is additive, not a rip-and-replace. Keep your
> existing `diagnostic_updater` publishers; medkit reads them and gives you the rest.

| | 🔴 ROS 2 diagnostics | 🟢 ros2_medkit |
|---|---|---|
| Access | `/diagnostics` topic + rqt GUI (local desktop) | SOVD REST API (remote; any tool / dashboard / agent) |
| Instrumentation | required (`diagnostic_updater` in node code) | works without it - drop-in bridges (`/rosout`, action status, `/diagnostics` passthrough) |
| State | live OK / WARN / ERROR / STALE | fault lifecycle (debounce, confirm, heal) |
| Moment of failure | none | freeze-frame snapshot + black-box rosbag |
| History | none (live only) | persisted, queryable |
| Model | key/value pairs | structured fault codes + SOVD entity model |
| Scope | ROS only | ROS today; PLC / ECU on one API |
| Agent access | none | MCP adapter |

## "I already have Foxglove. Why do I need this?"

Foxglove, Rerun and PlotJuggler are how you *look* at robot data - plots, 3D, logs, live or
from a bag - and they're great at it. ros2_medkit does the job they leave undone: it turns a
failure into a **structured fault**.

> Every mature machine industry already draws this line: in a car, an oscilloscope is not a
> scan tool. UDS / DTC diagnostics sit *above* the signal layer, as structured, queryable
> state. Robotics just hasn't drawn it yet.

| Common pushback | Why a structured fault still wins |
|---|---|
| *"I'll just set an alert."* | A threshold alert fires on a raw signal and is gone - no entity, no lifecycle, no history, and it means something different on every robot. A fault is confirmed, persisted, entity-attributed state - and, being SOVD, it means the same thing across robots and vendors. **Alerting pages a human; diagnostics gives a machine an answer.** |
| *"I'll just watch the fleet dashboard."* | Visualization is O(humans) - one operator, one timeline, and nobody scrubs 400 of them. A structured fault is O(1): the same code and lifecycle aggregate across the fleet, so *"which 12 of 400 robots have a confirmed navigation fault right now?"* is a query, not a person. |
| *"I can already see it fail."* | Observability is read-only by design. A structured fault is the precondition for doing something about it - gating an OTA on robot health, triggering remediation. **You can't safely patch what you haven't first diagnosed as a fault.** |

So it is not a replacement for your observability stack - it is the **diagnosis layer
underneath it**. medkit flags and persists the fault (readable by dashboards, fleet managers
and agents, not just people) and captures the black-box rosbag you then open in Foxglove.

## Beyond faults

Faults are the wedge. Once medkit is in, the same REST surface is a full SOVD diagnostic
gateway - your whole robot as an entity tree, live data, service and action calls, bulk data,
software updates and JWT/RBAC auth. The fault gets you in the door;
[the docs](https://selfpatch.github.io/ros2_medkit/) have the rest.

## Documentation & community

- 📖 [Documentation](https://selfpatch.github.io/ros2_medkit/) · 🗺️ [Roadmap](https://selfpatch.github.io/ros2_medkit/roadmap.html) · 🧩 [Postman collection](postman/)
- 💬 [Discord](https://discord.gg/6CXPMApAyq) · 🐛 [Issues](https://github.com/selfpatch/ros2_medkit/issues) · 💡 [Discussions](https://github.com/selfpatch/ros2_medkit/discussions)
- 🤝 Contributing: see [CONTRIBUTING.md](CONTRIBUTING.md) and [good first issues](https://github.com/selfpatch/ros2_medkit/labels/good%20first%20issue)
- 🔒 Security: responsible disclosure in [SECURITY.md](SECURITY.md)

## License

Apache License 2.0 - see [LICENSE](LICENSE).

---

<p align="center">
  Made with ❤️ by the <a href="https://github.com/selfpatch">selfpatch</a> community ·
  <a href="https://discord.gg/6CXPMApAyq">Join us on Discord</a>
</p>
