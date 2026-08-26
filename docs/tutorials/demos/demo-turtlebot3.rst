TurtleBot3 Demo
===============

This tutorial shows ros2_medkit integration with a TurtleBot3 simulation
running Gazebo and Nav2 navigation stack.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

This Docker-based demo showcases ros2_medkit with TurtleBot3 Waffle:

- **TurtleBot3 Simulation** — Gazebo world with TurtleBot3 Waffle robot
- **Nav2 Navigation** — Full navigation stack with SLAM
- **ros2_medkit Gateway** — REST API exposing robot capabilities
- **Web UI** — Visual entity browser

**Key Features:**

- Complete navigation system with goal sending
- Sensor data access (LiDAR, odometry, camera)
- Fault injection scenarios (localization, navigation failures)
- Docker-based — no local ROS 2 installation needed

Prerequisites
-------------

- Docker and Docker Compose installed
- Git (to clone the demo repository)

Starting the Demo
-----------------

Clone the demo repository and run the startup script:

.. code-block:: bash

   git clone https://github.com/selfpatch/selfpatch_demos.git
   cd selfpatch_demos/demos/turtlebot3_integration

   # Start the demo (daemon mode)
   ./run-demo.sh

.. figure:: /_static/images/17_turtlebot_run_demo_terminal.png
   :alt: Demo startup terminal output
   :align: center
   :width: 600px

   Terminal showing demo services starting up.

The script will build and start Docker containers with:

- Gazebo simulation with TurtleBot3 Waffle
- Nav2 navigation stack
- ros2_medkit gateway (REST API on port 8080)
- ros2_medkit_web_ui (Web interface on port 3000)

**Startup Options:**

.. code-block:: bash

   ./run-demo.sh --attached    # Run in foreground with logs
   ./run-demo.sh --update      # Pull latest images
   ./run-demo.sh --no-cache    # Build without cache

Exploring the System
--------------------

Open the web UI at http://localhost:3000 and connect to the gateway at
http://localhost:8080 and api/v1

.. figure:: /_static/images/16_turtlebot_gazebo.png
   :alt: TurtleBot3 in Gazebo
   :align: center
   :width: 600px

   TurtleBot3 Waffle in Gazebo simulation.

The demo uses a **manifest-based configuration** to organize entities into logical areas:

- **robot** — TurtleBot3 hardware and drivers
- **navigation** — Nav2 navigation stack
- **diagnostics** — ros2_medkit gateway and fault management
- **bridge** — Legacy diagnostics bridge

Querying via API:

.. code-block:: bash

   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/areas | jq

.. figure:: /_static/images/13_curl_areas_turtlebot3.png
   :alt: Areas response
   :align: center
   :width: 600px

   Areas discovered from TurtleBot3 system.

Interactive API exploration:

.. code-block:: bash

   # Run the interactive check script
   ./check-entities.sh

Reading Sensor Data
-------------------

Navigate to an app in the web UI and explore the **data** folder to see published topics.

Query data via REST API:

.. code-block:: bash

   # List all apps
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps | jq

   # Get specific topic from AMCL localization
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/amcl/data | jq

   # Get specific topic from controller server
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/controller-server/data | jq

.. figure:: /_static/images/06_topic_data_view.png
   :alt: Topic data view
   :align: center
   :width: 600px

   Viewing topic data in the web UI.

Sending Navigation Goals
-------------------------

Use the provided script to send navigation goals:

.. code-block:: bash

   # Send a navigation goal
   ./send-nav-goal.sh x y yaw

This sends a goal to Nav2 and monitors its progress through the gateway API.

You can also interact with the navigation stack via API:

.. code-block:: bash

   # List operations on BT Navigator
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/bt-navigator/operations | jq

   # List operations on Controller Server
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/controller-server/operations | jq

Managing Parameters
-------------------

Click on the **configurations** folder in the web UI to see ROS 2 parameters.

View and modify parameters:

.. code-block:: bash

   # List all configurations for AMCL
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/amcl/configurations | jq

   # Get specific parameter
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps/amcl/configurations/use_sim_time | jq

   # Change parameter value
   curl -X PUT http://localhost:8080/api/v1/apps/amcl/configurations/use_sim_time \
     -H "Content-Type: application/json" \
     -d '{"value": false}'

Fault Injection
---------------

The demo includes fault injection scripts to test diagnostic capabilities:

**Available fault injection scripts:**

.. code-block:: bash

   # Inject localization failure
   ./inject-localization-failure.sh

   # Inject navigation failure
   ./inject-nav-failure.sh

   # Restore normal operation
   ./restore-normal.sh

**View active faults:**

.. code-block:: bash

   # Check faults script
   ./check-faults.sh

   # Or query via API
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/faults

.. figure:: /_static/images/18_faults_injected_dashboard.png
   :alt: Faults dashboard
   :align: center
   :width: 600px

   Dashboard showing active faults.

.. figure:: /_static/images/19_faults_injected_app_view.png
   :alt: Faults in app view
   :align: center
   :width: 600px

   Fault details in the entity view.

Stopping the Demo
-----------------

To stop all services:

.. code-block:: bash

   ./stop-demo.sh

See Also
--------

- :doc:`demo-sensor` — Built-in sensor diagnostics demo
- :doc:`/tutorials/docker` — Docker deployment guide
- `selfpatch_demos <https://github.com/selfpatch/selfpatch_demos>`_ — Integration demos repository
