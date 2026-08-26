.. ros2_medkit documentation master file, created by
   sphinx-quickstart on Sun Nov 23 17:20:30 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

ros2_medkit documentation
=========================

**Modern, SOVD-compatible diagnostics for ROS 2 robots.**

ros2_medkit provides a REST API gateway that exposes your ROS 2 system for
external tools, web interfaces, and remote diagnostics. It automatically
discovers nodes, organizes them into a hierarchical entity tree, and provides
endpoints for data access, operations, configurations, and fault management.

Quick Links
-----------

.. grid:: 2

   .. grid-item-card:: 🚀 Getting Started
      :link: getting_started
      :link-type: doc

      New to ros2_medkit? Start here for a hands-on tutorial.

   .. grid-item-card:: 📦 Installation
      :link: installation
      :link-type: doc

      System requirements and installation instructions.

   .. grid-item-card:: 📖 Tutorials
      :link: tutorials/index
      :link-type: doc

      Step-by-step guides for common use cases.

   .. grid-item-card:: 🔧 Troubleshooting
      :link: troubleshooting
      :link-type: doc

      Solutions for common issues and FAQ.

Quick Reference
---------------

Common commands for quick access:

.. code-block:: bash

   # Check gateway health
   curl http://localhost:8080/api/v1/health

.. note::

   The gateway ships closed: every route below needs a credential, and
   ``GET /api/v1/health`` is the only one that does not. See
   :doc:`getting_started` for how to obtain ``$TOKEN``.


   # List all areas
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/areas

   # List all components
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/components

   # List all apps
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/apps

   # List all functions
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/functions

   # Get data from an entity (area, component, app, or function)
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/{entity-type}/{entity-id}/data

   # List operations for  an entity (area, component, app, or function)
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/{entity-type}/{entity-id}/operations

   # Get configurations (parameters)
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/{entity-type}/{entity-id}/configurations

   # List faults
   curl -H "Authorization: Bearer $TOKEN" http://localhost:8080/api/v1/faults

Community
---------

- 💬 **Discord**: `Join our server <https://discord.gg/6CXPMApAyq>`_ for discussions and help
- 🐛 **Issues**: `Report bugs <https://github.com/selfpatch/ros2_medkit/issues>`_
- 💡 **Discussions**: `GitHub Discussions <https://github.com/selfpatch/ros2_medkit/discussions>`_

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   introduction
   installation
   getting_started
   tutorials/index
   troubleshooting

.. toctree::
   :maxdepth: 2
   :caption: Architecture

   design/index
   roadmap

.. toctree::
   :maxdepth: 2
   :caption: Configuration

   config/index

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/index

.. toctree::
   :maxdepth: 4
   :caption: Standard Compliance

   requirements/index

.. toctree::
   :maxdepth: 1
   :caption: Reference

   glossary
   changelog
