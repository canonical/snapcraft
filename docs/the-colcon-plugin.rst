.. 11895.md

.. _the-colcon-plugin:

The colcon plugin
=================

The ``colcon`` plugin is useful when building `ROS 2 <http://www.ros.org/>`__ parts that use `colcon <https://colcon.readthedocs.io/en/released/>`__.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-colcon-plugin-core22_>`__
-  `base: core20 <the-colcon-plugin-core20_>`__
-  `base: core18 \| core <the-colcon-plugin-core18_>`__

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.


.. _the-colcon-plugin-core22:

base: core22
~~~~~~~~~~~~

For core22, this plugin is designed to work with the :ref:`ROS 2 Humble extension <the-ros-2-humble-extension>`. If not using this extension, it is required to set the ``ROS_DISTRO`` environment variable to ``humble`` using ``build-environment``.

This plugin enables the following plugin-specific keywords on core22:

-  **colcon-ament-cmake-args** (list of strings) Arguments to pass to *ament_cmake* packages. Note that any arguments here that match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-catkin-cmake-args** (list of strings) Arguments to pass to catkin packages. Note that any arguments here which match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-cmake-args** (list of strings) Arguments to pass to cmake projects. Note that any arguments here which match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-packages** (list of strings) List of colcon packages to build. If not specified, all packages in the workspace will be built. If set to an empty list (``[]``), no packages will be built, which could be useful if you only want ROS debs in the snap.
-  **colcon-packages-ignore** (list of strings) List of packages for colcon to ignore.


.. _the-colcon-plugin-core20:

base: core20
~~~~~~~~~~~~

For core20, this plugin is designed to work with the :ref:`ROS 2 Foxy extension <the-ros2-foxy-extension>`. If not using this extension, it is required to set the ``ROS_DISTRO`` environment variable to ``foxy`` using ``build-environment``.

This plugin enables the following plugin-specific keywords on core20:

-  **colcon-ament-cmake-args** (list of strings) Arguments to pass to *ament_cmake* packages. Note that any arguments here that match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-catkin-cmake-args** (list of strings) Arguments to pass to catkin packages. Note that any arguments here which match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-cmake-args** (list of strings) Arguments to pass to cmake projects. Note that any arguments here which match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-packages** (list of strings) List of colcon packages to build. If not specified, all packages in the workspace will be built. If set to an empty list (``[]``), no packages will be built, which could be useful if you only want ROS debs in the snap.
-  **colcon-packages-ignore** (list of strings) List of packages for colcon to ignore.


.. _the-colcon-plugin-core18:

base: core18
~~~~~~~~~~~~

This plugin enables the following plugin-specific keywords on core18:

-  **colcon-packages** (list of strings) List of colcon packages to build. If not specified, all packages in the workspace will be built. If set to an empty list (``[]``), no packages will be built, which could be useful if you only want ROS debs in the snap.
-  **colcon-source-space** (string) The source space containing colcon packages (defaults to ``src``).
-  **colcon-rosdistro** (string) The ROS distro to use. Available options are bouncy and crystal (defaults to crystal), both of which are only compatible with core18 as the base.
-  **colcon-cmake-args** (list of strings) Arguments to pass to cmake projects. Note that any arguments here which match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-catkin-cmake-args** (list of strings) Arguments to pass to catkin packages. Note that any arguments here which match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.
-  **colcon-ament-cmake-args** (list of strings) Arguments to pass to ament_cmake packages. Note that any arguments here which match colcon arguments need to be prefixed with a space. This can be done by quoting each argument with a leading space.

Related Information
-------------------

See the :ref:`catkin plugin <the-catkin-plugin>` for building ROS 1 parts.

For a simple example, see :ref:`ROS 2 applications <ros-2-deployment-with-snaps>`, or search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+colcon%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
