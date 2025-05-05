.. _reference:

Reference
=========

These pages provide details about the features and processes in Snapcraft.


Commands
--------

Snapcraft is operated from the command line, with a command for each function.

:ref:`reference-commands`


Project file
------------

The main object inside a snap project is a configurable project file. Read on for a
complete reference of the structure and contents of this file.

:ref:`Project file references <reference-project-file>`


Bases and architectures
-----------------------

Snaps are built using existing technologies and must be configured to build for specific
CPU architectures.

- :ref:`reference-bases`
- :ref:`reference-architectures`


Parts
-----

Software is brought into a snap through definitions of parts, and must be configured for the software's language and build systems.

- :ref:`reference-parts-and-steps`
- :ref:`reference-part-environment-variables`
- :ref:`reference-plugins`
- :ref:`reference-extensions`


Processes
---------

Snapcraft has defined processes underlying its basic operation.

- :ref:`reference-snap-build-process`
- :ref:`reference-snap-publishing-process`


.. toctree::
    :hidden:

    commands
    build-environment-options
    project-file/index
    bases
    architectures
    parts/index
    plugins
    package-repositories
    extensions/index
    external-package-information
    components
    linters
    layouts
    advanced-grammar
    hooks
    channels
    snapshots
    metrics
    processes/index
    system-requirements
