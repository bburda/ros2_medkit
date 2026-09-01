Installation
============

This guide covers installation of ros2_medkit on Ubuntu 24.04 with ROS 2 Jazzy,
Ubuntu 22.04 with ROS 2 Humble, or Ubuntu 26.04 with ROS 2 Lyrical.

System Requirements
-------------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Requirement
     - Version
   * - Operating System
     - Ubuntu 26.04 LTS (Resolute), Ubuntu 24.04 LTS (Noble), or Ubuntu 22.04 LTS (Jammy)
   * - ROS 2 Distribution
     - Jazzy, Humble, or Lyrical
   * - C++ Compiler
     - GCC 11+ (C++17 support required)
   * - CMake
     - 3.22+
   * - Python
     - 3.10+ (Humble) / 3.12+ (Jazzy) / 3.14+ (Lyrical)

Prerequisites
-------------

**ROS 2 Jazzy, Humble, or Lyrical** must be installed and sourced. Follow the official installation guide
for your distribution:

- Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Lyrical: https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html

.. note::

   On Ubuntu 22.04 (Humble), the system ``libcpp-httplib-dev`` package provides
   cpp-httplib 0.10.x which is too old. ros2_medkit requires >= 0.14 but ships a
   vendored copy as a fallback - no manual installation is needed. The build system
   automatically uses the vendored header when the system package is insufficient.

Installation from Binary Packages
---------------------------------

ros2_medkit is in the ROS index, so most packages install with ``apt``:

.. code-block:: bash

   sudo apt install ros-jazzy-ros2-medkit-gateway   # or ros-humble- / ros-lyrical-
   ros2 launch ros2_medkit_gateway bringup.launch.py

``ros2_medkit_opcua`` is the one package not distributed this way; it is waiting
on open62541pp being vendored and has to be built from source.

Installation from Source
------------------------

Build from source to get every package, to run against an unreleased version, or
to develop against the tree.

1. **Create a workspace**

   .. code-block:: bash

      mkdir -p ~/ros2_medkit_ws/src
      cd ~/ros2_medkit_ws/src

2. **Clone the repository**

   Clone directly into the ``src`` directory:

   .. code-block:: bash

      git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git .

   This places packages (``ros2_medkit_gateway``, ``ros2_medkit_fault_manager``, etc.) directly under ``~/ros2_medkit_ws/src/``.

   If you already cloned without submodules, initialize them:

   .. code-block:: bash

      cd ~/ros2_medkit_ws/src
      git submodule update --init --recursive

3. **Install dependencies**

   .. code-block:: bash

      cd ~/ros2_medkit_ws
      rosdep update
      rosdep install --from-paths src --ignore-src -r -y

4. **Build the workspace**

   .. code-block:: bash

      cd ~/ros2_medkit_ws
      colcon build --symlink-install

5. **Source the workspace**

   .. code-block:: bash

      source install/setup.bash

   Add this to your ``~/.bashrc`` for persistence:

   .. code-block:: bash

      echo "source ~/ros2_medkit_ws/install/setup.bash" >> ~/.bashrc

Verifying Installation
----------------------

After installation, verify that everything works:

.. code-block:: bash

   # Check that the gateway node is available
   ros2 pkg list | grep ros2_medkit

   # Launch the gateway
   ros2 launch ros2_medkit_gateway gateway.launch.py &

   # Test the API
   curl http://localhost:8080/api/v1/health

   # Stop the gateway
   pkill -f gateway_node

You should see output like:

.. code-block:: json

   {"status": "healthy", "timestamp": "..."}

Docker Installation
-------------------

Pre-built Docker images are published to GitHub Container Registry for all supported
ROS 2 distributions:

.. code-block:: bash

   # Jazzy (recommended)
   docker run -p 8080:8080 ghcr.io/selfpatch/ros2_medkit-jazzy:latest

   # Humble
   docker run -p 8080:8080 ghcr.io/selfpatch/ros2_medkit-humble:latest

   # Lyrical
   docker run -p 8080:8080 ghcr.io/selfpatch/ros2_medkit-lyrical:latest

The gateway will be available at http://localhost:8080/api/v1/health.

To use a custom configuration, mount a params file:

.. code-block:: bash

   docker run -p 8080:8080 \
     -v ./my_params.yaml:/etc/ros2_medkit/params.yaml \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

See the :doc:`tutorials/docker` for detailed Docker usage instructions including
Docker Compose examples and plugin configuration.

Development Installation
------------------------

For development with code coverage and debug symbols:

.. code-block:: bash

   cd ~/ros2_medkit_ws
   colcon build --symlink-install \
     --cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON

Troubleshooting
---------------

**rosdep fails to find packages**

   Make sure rosdep is initialized and updated:

   .. code-block:: bash

      sudo rosdep init  # Only needed once
      rosdep update

**Build fails with C++17 errors**

   Ensure you have GCC 13 or newer:

   .. code-block:: bash

      gcc --version  # Should show 13.x or higher

**Build fails on Humble with** ``Could not find cpp-httplib >= 0.14``

   This should not happen with current builds - a vendored copy of cpp-httplib 0.14.3
   is included as an automatic fallback. If you see this error, ensure ``ros2_medkit_cmake``
   is built before the gateway (``colcon build`` handles this automatically).

**Cannot find ros2_medkit packages after build**

   Make sure you source the workspace:

   .. code-block:: bash

      source ~/ros2_medkit_ws/install/setup.bash

.. _experimental-pixi:

Experimental: Pixi
------------------

.. warning::

   Pixi support is **experimental** and not the official build path. The standard
   ROS 2 toolchain (rosdep + colcon) remains the primary and recommended method.
   This feature is under evaluation - feedback is welcome on
   `GitHub issue #265 <https://github.com/selfpatch/ros2_medkit/issues/265>`_.

`Pixi <https://pixi.sh>`_ provides a reproducible, lockfile-based development environment
using packages from `RoboStack <https://robostack.github.io>`_ (conda-forge). It does not
require a system-wide ROS 2 installation - Pixi manages all dependencies in an isolated
environment.

Currently supported on **Linux x86_64** only. macOS and Windows support are tracked
separately (`#267 <https://github.com/selfpatch/ros2_medkit/issues/267>`_,
`#268 <https://github.com/selfpatch/ros2_medkit/issues/268>`_).

**Install Pixi:**

.. code-block:: bash

   curl -fsSL https://pixi.sh/install.sh | bash

**Clone and build:**

.. code-block:: bash

   git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git
   cd ros2_medkit
   pixi install -e jazzy    # or: pixi install -e humble
   pixi run -e jazzy build
   pixi run -e jazzy test
   pixi run -e jazzy smoke

**Environments:**

Two environments are available, matching the supported ROS 2 distributions:

- ``jazzy`` - ROS 2 Jazzy Jalisco (recommended)
- ``humble`` - ROS 2 Humble Hawksbill

Use ``-e <env>`` with all pixi commands, e.g. ``pixi run -e humble build``.

**Available tasks:**

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Task
     - Description
   * - ``pixi run -e <env> prep``
     - Install cpp-httplib into Pixi prefix (runs automatically before build)
   * - ``pixi run -e <env> build``
     - Build all packages with colcon
   * - ``pixi run -e <env> test``
     - Run unit tests (excludes linters and integration tests)
   * - ``pixi run -e <env> test-integ``
     - Run integration tests
   * - ``pixi run -e <env> test-results``
     - Show detailed test results
   * - ``pixi run -e <env> start``
     - Launch the gateway
   * - ``pixi run -e <env> smoke``
     - Smoke test: launch gateway and check health endpoint

**Known limitations:**

- **Linux x86_64 only** - macOS fails due to yaml-cpp build issues with Apple Clang
  (`#267 <https://github.com/selfpatch/ros2_medkit/issues/267>`_);
  Windows fails due to MSVC build issues
  (`#268 <https://github.com/selfpatch/ros2_medkit/issues/268>`_)
- **cpp-httplib built from source** - not available on conda-forge; ``pixi run prep`` clones
  and builds v0.14.3 from GitHub
- **RoboStack package coverage** - not all ROS 2 packages are available on RoboStack;
  ``pixi install`` will report any missing packages
- **No integration tests in CI** - only unit tests and smoke test run in CI;
  ``test-integ`` is available for local use
- **Lockfile size** - ``pixi.lock`` can be large; collapsed in GitHub diffs via
  ``.gitattributes``

Next Steps
----------

- :doc:`getting_started` - Quick tutorial to get started
- :doc:`tutorials/index` - Step-by-step tutorials
- :doc:`design/index` - Architecture documentation
