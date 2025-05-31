.. _reference-colcon-plugin:

Colcon plugin
=============

The ``colcon`` plugin is useful when building `ROS 2 <http://www.ros.org/>`_ parts
that use the `colcon <https://colcon.readthedocs.io/en/released/>`_ build tool.

Keywords
--------

In addition to the common :ref:`plugin <part-properties-plugin>` and
:ref:`sources <part-properties-sources>` keywords, this plugin provides the following
plugin-specific keywords:

colcon-ament-cmake-args
~~~~~~~~~~~~~~~~~~~~~~~
**Type:** list of strings
**Default:** []

Arguments to pass to ament_cmake packages. Note that any arguments here that match
colcon arguments need to be prefixed with a space. This can be done by quoting each
argument with a leading space.

colcon-catkin-cmake-args
~~~~~~~~~~~~~~~~~~~~~~~~
**Type:** list of strings
**Default:** []

Arguments to pass to catkin packages. Note that any arguments here which match colcon
arguments need to be prefixed with a space. This can be done by quoting each argument
with a leading space.

colcon-cmake-args
~~~~~~~~~~~~~~~~~
**Type:** list of strings
**Default:** []

Arguments to pass to cmake projects. Note that any arguments here which match colcon
arguments need to be prefixed with a space. This can be done by quoting each argument
with a leading space.

colcon-packages
~~~~~~~~~~~~~~~
**Type:** list of strings
**Default:** []

List of colcon packages to build. If not specified, all packages in the workspace will
be built. If set to an empty list (``[]``), no packages will be built, which could
be useful if you only want Debian packages in the snap.

colcon-packages-ignore
~~~~~~~~~~~~~~~~~~~~~~
**Type:** list of strings
**Default:** []

List of packages for colcon to ignore.

.. _colcon-ros-build-snaps-option:

colcon-ros-build-snaps
~~~~~~~~~~~~~~~~~~~~~~
**Type:** list of strings
**Default:** []

List of ROS 2 snaps that contain ROS 2 workspaces. This is set by the
:ref:`reference-ros-2-content-extensions` and shouldn't be set by the user.

Environment variables
---------------------

This plugin sets the following environment variables in the build environment:

AMENT_PYTHON_EXECUTABLE
~~~~~~~~~~~~~~~~~~~~~~~
**Default value**: ``/usr/bin/python3``

COLCON_PYTHON_EXECUTABLE
~~~~~~~~~~~~~~~~~~~~~~~~
**Default value**: ``/usr/bin/python3``

ROS_PYTHON_VERSION
~~~~~~~~~~~~~~~~~~
**Default value**: ``3``

Dependencies
------------

This plugin installs ``python3-rosdep``, ``python3-colcon-common-extensions``, and
any ROS 2 packages required to build the part.

On core22, this plugin also installs ``rospack-tools``, ``python3-rosinstall``, and
``python3-wstool``.

On other cores, this plugin also installs ``ros-<ros-distro>-ros2pkg`` where
``<ros-distro>`` is the ROS 2 distro release name.

This plugin installs all build snaps in the
:ref:`colcon-ros-build-snaps-option` option, which is configured by the ROS 2 Content
extension.

How it works
------------

The colcon plugin is designed to work with the ROS 2 extensions.

If a ROS 2 extension isn't used, the ``ROS_DISTRO`` environment variable must be set to
the ROS 2 distro release name in the part's ``build-environment`` key.

The extension or ``ROS_DISTRO`` environment variable used depends on the snap's
:ref:`base <reference-bases>`. For a list of which extensions can be used for a base,
see the :ref:`reference-ros-2-extensions` and :ref:`reference-ros-2-content-extensions`
reference pages.

During the build step the plugin performs the following actions:

* Call ``rosdep init`` and ``rosdep update`` to initialize the rosdep database.
* Source ROS workspaces present in any build snaps, stage snaps, and on the system.
* Install ROS 2 build packages required by the part.
* Call ``colcon build`` with any colcon-specific keywords set in the part.
* Stage runtime packages required by the part.

Example
-------

See :ref:`how-to-craft-an-ros-2-app` for an example of how to create a snap
for a ROS 2 app.
