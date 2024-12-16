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

  .. literalinclude:: ../code/craft-for-platforms/example-moos-recipe.yaml
    :language: yaml
    :lines: 2-


Add a part written for MOOS
---------------------------

.. literalinclude:: ../code/craft-for-platforms/example-moos-recipe.yaml
  :language: yaml
  :start-at: parts:
  :end-at: build-packages: [g++]

MOOS parts are written in C++, so they require the :ref:`CMake plugin
<craft_parts_cmake_plugin>`.

To add a MOOS part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: cmake``.
#. For ``build-packages``, add ``g++``.
