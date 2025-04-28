.. _reference-part-environment-variables:

Part environment variables
==========================

When building a part to construct a snap, Snapcraft creates the following sets of
environment variables that can optionally be used by a part's build mechanism:


Configuration
-------------

For all environment variables related to system architectures, see
:ref:`architectures-project-variables`.

In addition to architecture-related variables, the following environment variables are
also set:


core22 and core24
~~~~~~~~~~~~~~~~~

.. list-table::

    * - ``CRAFT_PARALLEL_BUILD_COUNT``

        ``SNAPCRAFT_PARALLEL_BUILD_COUNT``
      - The preferred number of jobs to build the project with.
    * - ``CRAFT_PROJECT_NAME``

        ``SNAPCRAFT_PROJECT_NAME``
      - The Snapcraft project name set by the project file's ``name`` key.
    * - ``SNAPCRAFT_PROJECT_VERSION``
      - The Snapcraft project version set by the project file's ``version`` key.
    * - ``SNAPCRAFT_PROJECT_GRADE``
      - The Snapcraft project grade set in the project file.
    * - ``CRAFT_PART_NAME``
      - The part currently being processed, as set by the part's name in the project
        file.
    * - ``CRAFT_STEP_NAME``
      - The step currently being executed.


core20
~~~~~~

.. list-table::

    * - ``SNAPCRAFT_PARALLEL_BUILD_COUNT``
      - The preferred number of jobs to build the project with.
    * - ``SNAPCRAFT_PROJECT_NAME``
      - The Snapcraft project name set by the project file's ``name`` key.
    * - ``SNAPCRAFT_PROJECT_VERSION``
      - The Snapcraft project version set by the project file's ``version`` key.
    * - ``SNAPCRAFT_PROJECT_GRADE``
      - The Snapcraft project grade set in the project file.


Directories
-----------

See :ref:`explanation-parts-lifecycle` for details on which directories are created when
building a part.


core22 and core24
~~~~~~~~~~~~~~~~~

Snapcraft exposes the following directory-related environment variables when building with
core22 and core24:

.. list-table::

    * - ``CRAFT_PART_SRC``

        ``SNAPCRAFT_PART_SRC``
      - The absolute path to where a part's sources are pulled. It's also the part's
        working directory for the pull step.
    * - ``CRAFT_PART_SRC_WORK``

        ``SNAPCRAFT_PART_SRC_WORK``
      - The absolute path to the part source subdirectory, if any. Defaults to the part
        source directory
    * - ``CRAFT_PART_BUILD``

        ``SNAPCRAFT_PART_BUILD``
      - The absolute path to the sources used for the part's build step. It is also the
        working directory of the build step.
    * - ``CRAFT_PART_BUILD_WORK``

        ``SNAPCRAFT_PART_BUILD_WORK``
      - The absolute path to the part build subdirectory in case of out-of-tree builds.
        Defaults to the part source directory.
    * - ``CRAFT_PART_INSTALL``

        ``SNAPCRAFT_PART_INSTALL``
      - The absolute path to the results of the part's build step. It also contains the
        staged packages of that part.
    * - ``CRAFT_PRIME``

        ``SNAPCRAFT_PRIME``
      - The absolute path to where files are primed.
    * - ``CRAFT_PROJECT_DIR``

        ``SNAPCRAFT_PROJECT_DIR``
      - The absolute path to the root of the Snapcraft project.
    * - ``CRAFT_STAGE``

        ``SNAPCRAFT_STAGE``
      - The absolute path to where files are staged.

For more details on the Craft Parts step execution environment, see
:ref:`parts-step-execution-environment`.


core20
~~~~~~

Snapcraft exposes the following directory-related environment variables when building a
part with core20:

.. list-table::

    * - ``SNAPCRAFT_PART_SRC``
      - The absolute path to where a part's sources are pulled. It's also the part's
        working directory for the pull step.
    * - ``SNAPCRAFT_PART_BUILD``
      - The absolute path to the sources used for the part's build step. It is also the
        working directory of the build step.
    * - ``SNAPCRAFT_PART_INSTALL``
      - The absolute path to the results of the part's build step. It also contains the
        staged packages of that part.
    * - ``SNAPCRAFT_PRIME``
      - The absolute path to where files are primed.
    * - ``SNAPCRAFT_PROJECT_DIR``
      - The absolute path to the root of the Snapcraft project.
    * - ``SNAPCRAFT_STAGE``
      - The absolute path to where files are staged.
