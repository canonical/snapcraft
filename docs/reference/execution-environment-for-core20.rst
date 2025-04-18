.. _execution-environment-for-core20:

Execution environment for core20
================================

.. important::

    The following environment variables will no longer be supported upon the release of
    core26. For instruction on changing to a newer base, see :ref:`how-to-change-bases`.

When building a part with core20, the following Snapcraft environment variables are
set:

.. list-table::

    * - ``SNAPCRAFT_ARCH_BUILD_FOR``
      - The deb-style architecture of the platform the snap is built for.
    * - ``SNAPCRAFT_ARCH_BUILD_ON``
      - The deb-style architecture of the platform the snap is built on.
    * - ``SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR``
      - The deb-style architecture triplet of the platform the snap is built for.
    * - ``SNAPCRAFT_ARCH_TRIPLET_BUILD_ON``
      - The deb-style architecture triplet of the platform the snap is built on.
    * - ``SNAPCRAFT_ARCH_TRIPLET``
      - The deb-style architecture triplet specified by ``--target-arch``. If a target
        architecture is not provided, then the architecture of the build-on platform is
        used.
    * - ``SNAPCRAFT_PARALLEL_BUILD_COUNT``
      - The preferred number of jobs to build the project with.
    * - ``SNAPCRAFT_PROJECT_NAME``
      - The Snapcraft project name set by the project file's ``name`` key.
    * - ``SNAPCRAFT_PROJECT_VERSION``
      - The Snapcraft project version set by the project file's ``version`` key.
    * - ``SNAPCRAFT_PROJECT_GRADE``
      - The Snapcraft project grade set in the project file.
    * - ``SNAPCRAFT_TARGET_ARCH``
      - The deb-style architecture specified by ``--target-arch``. If a target
        architecture is not provided, then the architecture of the build-on platform is
        used.
