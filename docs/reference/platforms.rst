.. meta::
    :description: Reference for the platforms and architectures keys in a snapcraft.yaml file. Includes the structure, project variables, and supported architectures.

.. _reference-platforms:

Platforms
=========

Project file
------------

The :ref:`platforms <reference-snapcraft-yaml-platform-keys>` and :ref:`architectures
<reference-snapcraft-yaml-architectures>` keys in a project file define where snaps are
built and where they run.

The keys are base-dependent:

* ``platforms`` is used for core24 and higher.
* ``architectures`` is used for core22.

The :ref:`platforms how-to <how-to-select-platforms>` provides examples of how to use
these keys.

core24 and higher
~~~~~~~~~~~~~~~~~

Structure
^^^^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
        <platform 1>:
            build-on: [<arch 1>, <arch 2>]
            build-for: [<arch 1>]
        <platform 2>:
            build-on: [<arch 3>]
            build-for: [<arch 4>]

``platform``
^^^^^^^^^^^^

A name for the technologies that the snap builds on and runs on. If the platform name is
a valid Debian architecture, ``build-on`` and ``build-for`` can be omitted.

In most cases, snaps are distributed with variants for every compatible CPU
architecture, so the platform name is often the same as the value of the ``build-for``
key.

The platform name can also describe a specific application or use. For example, a
platform named ``amd64-debug`` could include binaries built with debug flags enabled.


``build-on``
^^^^^^^^^^^^

An optional list of architectures where the snap can be built.

If the platform name is a valid architecture and ``build-for`` is not defined,
``build-on`` can be omitted and will default to the platform name.

``build-for``
^^^^^^^^^^^^^

An optional single-element list containing the architecture where the snap runs.

If the platform name is a valid architecture, ``build-for`` defaults to the platform
name.

``build-for: [all]`` is a special value that denotes an architecture-independent snap.
When using ``all``, no other ``build-on`` and ``build-for`` pairs can be defined.
:ref:`how-to-arch-build-for-all` describes how to use the ``all`` key.

.. _reference-architectures-core22:

core22
~~~~~~

Structure
^^^^^^^^^

The ``architectures`` key defines a list of ``build-on`` and ``build-for`` pairs:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [<arch 1>, <arch 2>]
        build-for: [<arch 1>]
      - build-on: [<arch 3>]
        build-for: [<arch 4>]

``build-on``
^^^^^^^^^^^^

A list of architectures where the snap can be built.

.. _reference-build-for:

``build-for``
^^^^^^^^^^^^^

An optional single-element list containing the architecture where the snap runs. If not
defined, ``build-for`` defaults to the value of ``build-on``.

The same architecture can't appear in multiple ``build-for`` entries.

``build-for: [all]`` is a special value that denotes an architecture-independent snap.
When using ``all``, no other ``build-on`` and ``build-for`` pairs can be defined.
:ref:`how-to-arch-build-for-all` describes how to use the ``all`` key.

.. _architectures-project-variables:

Project variables
-----------------

Snapcraft provides the following platform and architecture environment variables for use
during the part lifecycle and execution of user-defined scriptlets:

core24 and higher
~~~~~~~~~~~~~~~~~

+----------------------------------+-------------------------------------------+
| Project variable                 | Description                               |
+==================================+===========================================+
| ``CRAFT_PLATFORM``               | The name of the platform.                 |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_BUILD_FOR``         | The architecture the snap is built for.   |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_BUILD_ON``          | The architecture the snap is built on.    |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET_BUILD_FOR`` | The architecture triplet for the target   |
|                                  | platform.                                 |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET_BUILD_ON``  | The architecture triplet for the build    |
|                                  | platform.                                 |
+----------------------------------+-------------------------------------------+

core22
~~~~~~

+----------------------------------+-------------------------------------------+
| Project variable                 | Description                               |
+==================================+===========================================+
| ``CRAFT_ARCH_BUILD_FOR``         | The architecture the snap is built for.   |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_BUILD_ON``          | The architecture the snap is built on.    |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET_BUILD_FOR`` | The architecture triplet for the target   |
|                                  | platform.                                 |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET_BUILD_ON``  | The architecture triplet for the build    |
|                                  | platform.                                 |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET``           | Deprecated. Use                           |
|                                  | ``CRAFT_ARCH_TRIPLET_BUILD_FOR``.         |
+----------------------------------+-------------------------------------------+
| ``CRAFT_TARGET_ARCH``            | Deprecated. Use                           |
|                                  | ``CRAFT_ARCH_BUILD_FOR``.                 |
+----------------------------------+-------------------------------------------+

.. _reference_platforms_environment_variables:

Environment variables and command line arguments
------------------------------------------------

The platform can be defined at build time with environment variables and command-line
arguments passed to the ``snapcraft`` command.

Command-line arguments take priority over environment variables. Environment variables
prefixed with ``SNAPCRAFT`` take priority over those prefixed with ``CRAFT``.

core24 and higher
~~~~~~~~~~~~~~~~~

Command line arguments
^^^^^^^^^^^^^^^^^^^^^^^^
* ``--platform=<platform>``
* ``--build-for=<arch>``

Environment variables
^^^^^^^^^^^^^^^^^^^^^
* ``SNAPCRAFT_PLATFORM=<platform>``
* ``CRAFT_PLATFORM=<platform>``
* ``SNAPCRAFT_BUILD_FOR=<arch>``
* ``CRAFT_BUILD_FOR=<arch>``

core22
~~~~~~

Command line arguments
^^^^^^^^^^^^^^^^^^^^^^^^
* ``--build-for=<arch>``

Environment variables
^^^^^^^^^^^^^^^^^^^^^
* ``SNAPCRAFT_BUILD_FOR=<arch>``

.. _supported-architectures:

Supported CPU architectures

Local builds
~~~~~~~~~~~~

Snapcraft supports building for these architectures locally:

* amd64
* arm64
* armhf
* i386
* powerpc
* ppc64el
* riscv64
* s390x

.. _supported-architectures-launchpad:

Launchpad builds
~~~~~~~~~~~~~~~~

Launchpad supports remote building for these architectures:

* amd64
* arm64
* armhf
* ppc64el
* riscv64
* s390x
