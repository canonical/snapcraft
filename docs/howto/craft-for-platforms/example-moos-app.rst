.. _example-moos-app:

Example MOOS app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using MOOS. We'll work
through the aspects unique to MOOS-based apps by examining an existing
recipe.


Example test-moos recipe
------------------------

The following code comprises the recipe for the `MOOS core project
<https://github.com/themoos/core-moos>`_ itself.

.. collapse:: MOOS recipe

  .. code:: yaml

    name: test-moos
    version: '0.1'
    summary: MOOS Example
    description: |
      This example includes MOOSDB, the main communication mechanism for all MOOS
      apps.

    base: core18
    confinement: devmode

    parts:
      test-moos:
        source: https://github.com/themoos/core-moos/archive/v10.4.0.tar.gz
        plugin: cmake
        build-packages: [g++]

    apps:
      test-moos:
        command: bin/MOOSDB


Add a part written for MOOS
---------------------------

.. code:: yaml

  parts:
    test-moos:
      source: https://github.com/themoos/core-moos/archive/v10.4.0.tar.gz
      plugin: cmake
      build-packages: [g++]

MOOS parts are written in C++, so they require the :ref:`CMake plugin
<craft_parts_cmake_plugin>`.

To add a MOOS part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: cmake``.
#. For ``build-packages``, add ``g++``.
