.. _reference:

Reference
=========

References describe the structure and function of the individual components in Snapcraft.


Commands
--------

Snapcraft is operated from the command line, with a command for each function.

- :ref:`reference-commands`
- :ref:`reference-build-environment-options`


Project file
------------

The main object inside a snap project is a configurable project file. Read on for a
complete reference of the structure and contents of this file.

- :ref:`snapcraft.yaml reference <reference-snapcraft-yaml>`
- :ref:`reference-extensions`


Bases and architectures
-----------------------

Snaps are built using existing technologies and must be configured to build for specific
CPU architectures.

- :ref:`reference-bases`
- :ref:`reference-platforms`


Parts
-----

Software is brought into a snap through definitions of parts, and must be configured for
the software's language and build systems.

- :ref:`reference-parts-and-steps`
- :ref:`reference-part-environment-variables`
- :ref:`reference-plugins`


Requirements and support
------------------------

Snapcraft is actively supported on Linux systems, and Canonical provides long term
support (LTS) for it.

- :ref:`reference-system-requirements`
- :ref:`reference-support-schedule`


.. toctree::
    :hidden:

    snapcraft-yaml
    commands
    build-environment-options
    bases
    platforms
    parts-steps
    part-environment-variables
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
    system-requirements
    support-schedule
