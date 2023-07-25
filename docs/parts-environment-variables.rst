.. 12271.md

.. _parts-environment-variables:

Parts environment variables
===========================

When building a part to construct a snap, :ref:`Snapcraft <snapcraft-overview>` creates the following sets of environment variables that can optionally be used by a part’s build mechanism:

-  `Locating directories <parts-environment-variables-locating-directories_>`__

   -  `core \| core18 \| core20 <parts-environment-variables-locating-directories-core18-core20_>`__
   -  `core22 <parts-environment-variables-locating-directories-core22_>`__

-  `Snapcraft configuration <parts-environment-variables-snapcraft-configuration_>`__

   -  `core \| core18 \| core20 <parts-environment-variables-snapcraft-configuration-core18-core-20_>`__
   -  `core22 <parts-environment-variables-snapcraft-configuration-core22_>`__

-  `Build flags <parts-environment-variables-build-flags_>`__
-  `Plugin variables <parts-environment-variables-plugin-variables_>`__

Environment variables can be accessed via the :ref:`override- <snapcraft-parts-metadata>` keywords with shell commands and :ref:`Scriptlets <override-build-steps>`, or more generally within your project’s build infrastructure.

See :ref:`Adding parts <adding-parts>` for a general overview of what parts are and how to use them, and for more details on how parts are built within the *snapcraft* environment, including build stages and the directories they use, see :ref:`Parts lifecycle <parts-lifecycle>`.

   ⓘ For the various environment variables available to running snap applications, see :ref:`Environment variables <environment-variables>`.


.. _parts-environment-variables-locating-directories:

Locating directories
~~~~~~~~~~~~~~~~~~~~

See :ref:`Parts lifecycle <parts-lifecycle>` and :ref:`Parts directories <parts-lifecycle-parts-directories>` for details on which directories are created and used when building a part.


.. _parts-environment-variables-locating-directories-core18-core20:

core \| core18 \| core20
^^^^^^^^^^^^^^^^^^^^^^^^

Snapcraft exposes the following directory related environment variables when building a part with bases ``core``, ``core18``, or ``core20``. These can help when moving or locating files:

.. list-table::
   :header-rows: 0

   * - ``SNAPCRAFT_PART_SRC``
     - absolute path to where a part’s sources are pulled. It’s also the part’s working directory for the *pull* step
   * - ``SNAPCRAFT_PART_BUILD``
     - absolute path to the sources used for the part’s *build* step. It is also the working directory of the *build* step.
   * - ``SNAPCRAFT_PART_INSTALL``
     - absolute path to the results of the part’s *build* step. It also contains the staged packages of that part.
   * - ``SNAPCRAFT_PRIME``
     - absolute path to where files are primed
   * - ``SNAPCRAFT_PROJECT_DIR``
     - absolute path to the root of the snapcraft project
   * - ``SNAPCRAFT_STAGE``
     - absolute path to where files are staged


.. _parts-environment-variables-locating-directories-core22:

core22
^^^^^^

Snapcraft exposes the following directory related environment variables when build with base ``core22``. These can help when moving or locating files:

+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_PART_SRC`` ``SNAPCRAFT_PART_SRC``               | absolute path to where a part’s sources are pulled. It’s also the part’s working directory for the *pull* step       |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_PART_SRC_WORK`` ``SNAPCRAFT_PART_SRC``          | absolute path to the part source subdirectory, if any. Defaults to the part source directory.                        |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_PART_BUILD`` ``SNAPCRAFT_PART_BUILD``           | absolute path to the sources used for the part’s *build* step. It is also the working directory of the *build* step. |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_PART_BUILD_WORK`` ``SNAPCRAFT_PART_BUILD_WORK`` | absolute path to the part build subdirectory in case of out-of-tree builds. Defaults to the part source directory.   |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_PART_INSTALL`` ``SNAPCRAFT_PART_INSTALL``       | absolute path to the results of the part’s *build* step. It also contains the staged packages of that part.          |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_PRIME`` ``SNAPCRAFT_PRIME``                     | absolute path to where files are primed                                                                              |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_PROJECT_DIR`` ``SNAPCRAFT_PROJECT_DIR``         | absolute path to the root of the snapcraft project                                                                   |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_STAGE`` ``SNAPCRAFT_STAGE``                     | absolute path to where files are staged                                                                              |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+
| ``CRAFT_OVERLAY``                                       | absolute path the part’s layer directory during the ``OVERLAY`` step if overlays are enabled.                        |
+---------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------+


.. _parts-environment-variables-snapcraft-configuration:

Snapcraft configuration
~~~~~~~~~~~~~~~~~~~~~~~


.. _parts-environment-variables-snapcraft-configuration-core18-core-20:

core \| core18 \| core20
^^^^^^^^^^^^^^^^^^^^^^^^

When building a part with bases ``core``, ``core18``, or ``core20``, the following *snapcraft* environment variables are set:

+------------------------------------+-------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_ARCH_TRIPLET``         | the architecture triplet used for the selected base                                       |
+------------------------------------+-------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PARALLEL_BUILD_COUNT`` | the preferred number of jobs the project is to be built with                              |
+------------------------------------+-------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PROJECT_NAME``         | the snapcraft project name as set by ``name`` in :file:`snapcraft.yaml`                   |
+------------------------------------+-------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PROJECT_VERSION``      | the snapcraft project version as set by :file:`snapcraft.yaml`                            |
+------------------------------------+-------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PROJECT_GRADE``        | the snapcraft project grade as set in :file:`snapcraft.yaml`                              |
+------------------------------------+-------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_TARGET_ARCH``          | deb-style architecture that snap is being built for, e.g. “amd64”, “armhf”, “arm64”, etc. |
+------------------------------------+-------------------------------------------------------------------------------------------+


.. _parts-environment-variables-snapcraft-configuration-core22:

core22
^^^^^^

When building a part with base ``core22``, the following *snapcraft* environment variables are set:

+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``CRAFT_ARCH_TRIPLET`` ``SNAPCRAFT_ARCH_TRIPLET``                 | the architecture triplet used for the selected base                                                    |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``CRAFT_PARALLEL_BUILD_COUNT`` ``SNAPCRAFT_PARALLEL_BUILD_COUNT`` | the preferred number of jobs the project is to be built with                                           |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``CRAFT_PROJECT_NAME`` ``SNAPCRAFT_PROJECT_NAME``                 | the snapcraft project name as set by ``name`` in :file:`snapcraft.yaml`                                |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PROJECT_VERSION``                                     | the snapcraft project version as set by :file:`snapcraft.yaml`                                         |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PROJECT_GRADE``                                       | the snapcraft project grade as set in :file:`snapcraft.yaml`                                           |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``CRAFT_TARGET_ARCH`` ``SNAPCRAFT_TARGET_ARCH``                   | deb-style architecture that snap is being built for, e.g. “amd64”, “armhf”, “arm64”, etc.              |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``CRAFT_PART_NAME``                                               | the part currently being processed, as set by the part’s name in :file:`snapcraft.yaml`                |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+
| ``CRAFT_STEP_NAME``                                               | the step currently being executed (i.e. ``PRIME``)                                                     |
+-------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------+


.. _parts-environment-variables-build-flags:

Build flags
~~~~~~~~~~~

The following specific *build flags* are also set:

+-----------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``CFLAGS``                        | empty unless ``after`` is used in the part and headers are staged in the common include paths for which they will be included (i.e.; paths added with ``-I``) |
+-----------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``CPPFLAGS``                      | same behavior as CFLAGS                                                                                                                                       |
+-----------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``CXXFLAGS``                      | same behavior as CFLAGS                                                                                                                                       |
+-----------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``LDFLAGS``                       | empty unless ``after`` is used in the part and headers are staged in the common library paths (i.e.; paths added with ``-L``)                                 |
+-----------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``PKG_CONFIG_PATH``               | empty unless ``after`` is used in the part and .pc files are staged in the common pkgconfig paths                                                             |
+-----------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------+


.. _parts-environment-variables-plugin-variables:

Plugin variables
~~~~~~~~~~~~~~~~

A part’s :ref:`plugin <snapcraft-plugins>` can add its own set of environment variables, or expand on the above *build* related flags.

The ``build-environment`` keyword can be used to either override the default environment variables or define new ones. Here is a basic example:

.. code:: yaml

   parts:
     hello-part:
       source: gnu-hello.tar.gz
       plugin: autotools
       build-environment:
       - CFLAGS: "$CFLAGS -O3"  # add -O3 to the existing flags
       - LDFLAGS: "-L$SNAPCRAFT_STAGE/non-standard/lib"

The above example will override default flags and search for libraries in a non-standard path.

For a complete list of environment variables, see :ref:`Environment variables exposed by Snapcraft <environment-variables-that-snapcraft-exposes>`.
