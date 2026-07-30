Development Container
=====================

ros2_medkit includes a ready-to-use VS Code development container (devcontainer) that
provides a complete, reproducible development environment.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The devcontainer provides:

- **Ubuntu 24.04** base image
- **ROS 2 Jazzy** desktop-full installation
- **C++17** toolchain (GCC 13)
- **Development tools**: CMake, colcon, rosdep, clang-format, clang-tidy
- **Documentation tools**: Doxygen, Sphinx, PlantUML
- **Code coverage**: lcov
- **VS Code extensions**: C++, Python, CMake, GitHub Copilot, GitLens

Prerequisites
-------------

1. **VS Code** with the `Dev Containers extension <https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers>`_
2. **Docker** (Docker Desktop on Windows/macOS, Docker Engine on Linux)
3. **Git** with SSH key configured (optional, for pushing to remotes)

Quick Start
-----------

1. **Clone the repository with submodules:**

   .. code-block:: bash

      git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git
      cd ros2_medkit

   If you already cloned without submodules:

   .. code-block:: bash

      git submodule update --init --recursive

2. **Open in VS Code:**

   .. code-block:: bash

      code .

3. **Reopen in Container:**

   When VS Code opens, click "Reopen in Container" in the notification popup, or:

   - Press ``F1`` → "Dev Containers: Reopen in Container"

4. **Wait for container build:**

   The first build takes 5-10 minutes. Subsequent opens use the cached image.

5. **Start developing:**

   Once the container starts, the terminal will have ROS 2 sourced automatically.

Container Configuration
-----------------------

devcontainer.json
~~~~~~~~~~~~~~~~~

The ``.devcontainer/devcontainer.json`` configures:

**Build Settings:**

- Uses custom ``Dockerfile`` for ROS 2 Jazzy setup
- Passes ``USERNAME`` from host for consistent file permissions
- Network mode: ``host`` for ROS 2 discovery

**Workspace Mounting:**

- Workspace mounted at ``/home/<username>/workspace``
- SSH keys mounted read-only for git operations
- Git config mounted for commit identity

**VS Code Extensions:**

Pre-installed extensions for C++, Python, CMake, YAML, and documentation development.

**Post-Create Script:**

Runs ``setup-env.sh`` to initialize rosdep and source ROS 2 environment.

Dockerfile
~~~~~~~~~~

The ``.devcontainer/Dockerfile`` installs:

**System Packages:**

- ``ros-jazzy-desktop-full`` - Complete ROS 2 installation
- ``python3-colcon-common-extensions`` - Build system
- ``doxygen``, ``graphviz``, ``plantuml`` - Documentation
- ``clang-format``, ``clang-tidy`` - Code formatting/linting
- ``lcov`` - Coverage reports
- ``hadolint`` - Dockerfile linting

**Python Packages:**

- ``sphinx``, ``sphinx-rtd-theme``, ``sphinx-needs`` - Documentation
- ``black``, ``flake8-*`` - Python linting
- ``pytest``, ``pytest-repeat``, ``pytest-rerunfailures`` - Testing

Development Workflow
--------------------

Building the Project
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd ~/workspace
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install

Running Tests
~~~~~~~~~~~~~

.. code-block:: bash

   # All tests
   colcon test && colcon test-result --verbose

   # Unit tests only (exclude integration tests package)
   colcon test --packages-skip ros2_medkit_integration_tests

   # Linters only
   colcon test --ctest-args -L linters

Building Documentation
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd ~/workspace/docs
   pip install -e .
   sphinx-build -b html . _build/html

Generating Coverage
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd ~/workspace
   WS="${PWD}"

   colcon build --packages-skip ros2_medkit_opcua \
     --cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
   colcon test --packages-skip ros2_medkit_opcua --ctest-args -LE linter

   lcov --capture --directory build --output-file coverage.raw.info \
     --ignore-errors mismatch,negative,empty,gcov
   lcov --extract coverage.raw.info "${WS}/src/*/src/*" \
     "${WS}/src/*/include/*" \
     --output-file coverage.extracted.info --ignore-errors unused,empty
   lcov --remove coverage.extracted.info '*/vendored/*' \
     --output-file coverage.info --ignore-errors unused,empty
   genhtml coverage.info --output-directory coverage_html --ignore-errors source

   # Assert no package silently dropped out of the report
   ./scripts/check_coverage_packages.sh coverage.info --skip ros2_medkit_opcua

Open ``coverage_html/index.html`` in a browser. This mirrors the measurement
pipeline of the CI coverage job, so the number is comparable to the published
one; ``.github/workflows/ci.yml`` is the authoritative version. See
``CONTRIBUTING.md`` for why the last step exists and why ``ros2_medkit_opcua``
is skipped.

Customization
-------------

Adding Extensions
~~~~~~~~~~~~~~~~~

Edit the ``customizations.vscode.extensions`` array in ``devcontainer.json``:

.. code-block:: json

   "extensions": [
     "ms-vscode.cpptools",
     "your.extension-id"
   ]

Changing Shell
~~~~~~~~~~~~~~

The container uses ``zsh`` with Oh My Zsh by default. To use bash, modify the
``features`` section in ``devcontainer.json``:

.. code-block:: json

   "features": {
     "ghcr.io/devcontainers/features/common-utils:2": {
       "installZsh": false,
       "configureZshAsDefaultShell": false
     }
   }

Persistent Storage
~~~~~~~~~~~~~~~~~~

Add volume mounts for data that should persist across container rebuilds:

.. code-block:: json

   "mounts": [
     "source=ros2-medkit-build-cache,target=/home/${localEnv:USER}/workspace/build,type=volume"
   ]

Troubleshooting
---------------

Container fails to start
~~~~~~~~~~~~~~~~~~~~~~~~

**SSH key issues:**

If you don't have SSH keys, remove or comment out the SSH mount:

.. code-block:: json

   // "source=${localEnv:HOME}/.ssh,target=/home/${localEnv:USER}/.ssh,..."

**Docker socket permissions:**

On Linux, ensure your user is in the ``docker`` group:

.. code-block:: bash

   sudo usermod -aG docker $USER
   # Log out and back in

ROS 2 commands not found
~~~~~~~~~~~~~~~~~~~~~~~~

The ROS 2 environment should be sourced automatically. If not:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash

Rebuild the container if setup-env.sh didn't run:

- Press ``F1`` → "Dev Containers: Rebuild Container"

Slow file I/O on macOS/Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For better performance with large workspaces, consider using a named volume
for the build directory instead of bind mounting.

See Also
--------

- :doc:`../installation` - Manual installation instructions
- :doc:`docker` - Production Docker deployment
- `VS Code Dev Containers docs <https://code.visualstudio.com/docs/devcontainers/containers>`_
