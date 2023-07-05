.. 8621.md

.. _the-cmake-plugin:

The cmake plugin
================

The ``cmake`` plugin is useful for building `CMake <https://cmake.org/>`__-based parts. This plugin uses the common plugin keywords as well as those for :ref:`sources <snapcraft-parts-metadata-source>`. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

This plugin also supports options from the :ref:`make <the-make-plugin>` plugin.

A *cmake* project will typically include a *CMakeLists.txt* file to drive the build, and the plugin requires that *CMakeLists.txt* exists within the root of the source tree.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-cmake-plugin-core22_>`__
-  `base: core20 <the-cmake-plugin-core20_>`__
-  `base: core18 \| core <the-cmake-plugin-core18_>`__

For a simple example, see :ref:`MOOS applications <moos-applications>`, or search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+cmake%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-cmake-plugin-core22:

base: core22
~~~~~~~~~~~~

-  **cmake-generator** (string, default: *Unix Makefiles*) Determine what native build system is to be used. Can be either ``Ninja`` or ``Unix Makefiles`` (default).
-  **cmake-parameters** (list of strings) Configure flags to pass to the build using the common *cmake* semantics.

Note that Snapcraft does not specify cmake parameters by default. A common parameter for parts using the cmake plugin is ``CMAKE_INSTALL_PREFIX=/usr`` - this prevents installation to the default directory of ``/usr/local``. For example:

::

   parts:
     hello:
       plugin: cmake
       cmake-parameters:
         - -DCMAKE_INSTALL_PREFIX=/usr
       ....

Requires Snapcraft version *7.0+*.


.. _the-cmake-plugin-core20:

base: core20
~~~~~~~~~~~~

-  **cmake-generator** (string, default: *Unix Makefiles*) Determine what native build system is to be used. Can be either ``Ninja`` or ``Unix Makefiles`` (default).
-  **cmake-parameters** (list of strings) Configure flags to pass to the build using the common *cmake* semantics.

Note that Snapcraft does not specify cmake parameters by default. A common parameter for parts using the cmake plugin is ``CMAKE_INSTALL_PREFIX=/usr`` - this prevents installation to the default directory of ``/usr/local``. For example:

::

   parts:
     hello:
       plugin: cmake
       cmake-parameters:
         - -DCMAKE_INSTALL_PREFIX=/usr
       ....

Requires Snapcraft version *4.0+*.


.. _the-cmake-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

-  **configflags** (list of strings) Configure flags to pass to the build using the common *cmake* semantics.

Requires Snapcraft version *3.0+*.
