Architectures
=============

snapcraft.yaml
--------------

The :doc:`architectures how-to</howto/architectures>` provides examples of how
to use the ``architectures`` keyword.

core22
^^^^^^

The root keyword ``architectures`` defines a list of ``build-on->build-for``
pairs with the following syntax:

.. code-block:: yaml

  architectures:
    - build-on: [<arch 1>, <arch 2>]
      build-for: [<arch 1>]
    - build-on: [<arch 3>]
      build-for: [<arch 4>]
      ...

See :ref:`here<supported-architectures>` for a list of supported architectures.

Data types
""""""""""

If the value is a single item, it can be written as a single element list or a
scalar. For example:

.. code-block:: yaml

  architectures:
    - build-on: amd64
      build-for: arm64

is equivalent to:

.. code-block:: yaml

  architectures:
    - build-on: [amd64]
      build-for: [arm64]

.. note::

  Using a list is the recommended syntax. Scalars are not recommended.

``build-on``
""""""""""""

The ``build-on`` field is required and can contain multiple architectures.

.. _reference-build-for:

``build-for``
"""""""""""""

The ``build-for`` field is optional and should be a single element list. If it
is not defined, the ``build-for`` will be set the ``build-on`` for each build
in the :ref:`build plan<build-plans>`.

``build-for: [all]`` is a special keyword to denote an architecture-independent
snap. If the ``all`` keyword is used, no other ``build-on/build-for`` pairs can
be defined. See :ref:`this page<how-to-arch-build-for-all>` to learn how to
use the ``all`` keyword.

``all`` cannot be used for ``build-on``.

The same architecture cannot be defined in multiple ``build-for`` entries.

Shorthand format
""""""""""""""""

As an alternative to the explicit format described above, a shorthand format
can be used for simple ``build-on/build-for`` pairs. The following shorthand:

.. code-block:: yaml

  architectures: [amd64, arm64]

is equivalent to:

.. code-block:: yaml

  architectures:
    - build-on: [amd64]
      build-for: [amd64]
    - build-on: [arm64]
      build-for: [arm64]

The explicit and shorthand format cannot be mixed.

core20
^^^^^^

The above syntax and rules for ``core22`` apply for ``core20`` except that
``run-on`` is used in place of ``build-for``. Additionally, ``core20`` supports
multiple architectures in the ``run-on`` field, which will create
multi-architecture snaps.

Project variables
-----------------

core22
^^^^^^

+----------------------------------+-------------------------------------------+
| Project variable                 | Description                               |
+==================================+===========================================+
| ``CRAFT_ARCH_BUILD_FOR``         | The architecture of the platform the snap |
|                                  | is built for.                             |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_BUILD_ON``          | The architecture of the platform the snap |
|                                  | is built on.                              |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET_BUILD_FOR`` | The architecture triplet of the platform  |
|                                  | the snap is built for.                    |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET_BUILD_ON``  | The architecture triplet of the platform  |
|                                  | the snap is built on.                     |
+----------------------------------+-------------------------------------------+
| ``CRAFT_ARCH_TRIPLET``           | deprecated: use                           |
|                                  | ``CRAFT_ARCH_TRIPLET_BUILD_FOR`` instead  |
+----------------------------------+-------------------------------------------+
| ``CRAFT_TARGET_ARCH``            | deprecated: use ``CRAFT_ARCH_BUILD_FOR``  |
|                                  | instead                                   |
+----------------------------------+-------------------------------------------+

core20
^^^^^^

``core20`` snaps use the terminology ``run-on`` in the ``architectures`` field
in ``snapcraft.yaml``. The project variables and documentation use the
preferred term ``build-for``.

+--------------------------------------+---------------------------------------+
| Project variable                     | Description                           |
+======================================+=======================================+
| ``SNAPCRAFT_ARCH_BUILD_FOR``         | The architecture of the platform the  |
|                                      | snap is built for.                    |
+--------------------------------------+---------------------------------------+
| ``SNAPCRAFT_ARCH_BUILD_ON``          | The architecture of the platform the  |
|                                      | snap is built on.                     |
+--------------------------------------+---------------------------------------+
| ``SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR`` | The architecture triplet of the       |
|                                      | platform the snap is built for.       |
+--------------------------------------+---------------------------------------+
| ``SNAPCRAFT_ARCH_TRIPLET_BUILD_ON``  | The architecture triplet of the       |
|                                      | platform the snap is built on.        |
+--------------------------------------+---------------------------------------+
| ``SNAPCRAFT_ARCH_TRIPLET``           | The architecture triplet specified by |
|                                      | ``--target-arch``. If a target arch   |
|                                      | is not provided, then the             |
|                                      | architecture of the build-on platform |
|                                      | is used.                              |
+--------------------------------------+---------------------------------------+
| ``SNAPCRAFT_TARGET_ARCH``            | The architecture specified by         |
|                                      | ``--target-arch``. If a target arch   |
|                                      | is not provided, then the             |
|                                      | architecture of the build-on platform |
|                                      | is used.                              |
+--------------------------------------+---------------------------------------+

``SNAPCRAFT_ARCH_BUILD_FOR`` and ``SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR`` are not
available in any of the following scenarios:

* when the build-for architecture is not in the list of supported architectures
* when building a multi-architecture snap

.. _supported-architectures:

Supported architectures
-----------------------

Supported locally
^^^^^^^^^^^^^^^^^

The following architectures can be used when building a snap locally.

* amd64
* arm64
* armhf
* i386
* powerpc
* ppc64el
* riscv64
* s390x

.. _supported-architectures-launchpad:

Supported by Launchpad
^^^^^^^^^^^^^^^^^^^^^^

The following architectures are supported by Launchpad for remote building.

* amd64
* arm64
* armhf
* ppc64el
* s390x

Environment variables and command line arguments
------------------------------------------------

core22
^^^^^^

The command line argument ``--build-for=<arch>`` or environment variable
``SNAPCRAFT_BUILD_FOR=<arch>`` is used to build a single snap package for an
architecture. It selects a single ``build-on->build-for`` pair from the build
plan. It does not allow building a snap not defined by the build plan.

The command line argument takes priority over the environment variable.

See :ref:`build-plans` for more information on how build plans are created.

core20
^^^^^^

``--target-arch=<arch>`` is used for cross-compiling in ``core20``. It adds
repositories for the target arch, which are used for ``stage-packages``. The
target arch does not need to be listed in the ``snapcraft.yaml``
``architectures`` keyword.

The ``--target-arch`` argument can only be used in destructive mode and with
``--enable-experimental-target-arch`` or the environment variable
``SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH``.

The full usage is
``snapcraft --destructive-mode --enable-experimental-target-arch --target-arch=<arch>``.
