.. _reference-catkin-tools-plugin:

Catkin Tools plugin
===================

The Catkin Tools plugin builds `ROS 1 <https://www.ros.org/>`_ parts using the `catkin
CLI tool <https://catkin-tools.readthedocs.io/en/latest/>`_.

In Snapcraft 8, this plugin depends on the :ref:`reference-catkin-plugin` for
configuration. Both these plugins can be used in combination with the
:ref:`reference-ros-1-extension` to build core20 snaps.


Keys
----

This plugin uses the same keys as the Catkin plugin, and provides the following unique
keys for core20 snaps.


catkin-tools-packages
~~~~~~~~~~~~~~~~~~~~~

**Type**: list of strings

The list of catkin packages to build. If not set, all packages in the catkin workspace
are built. If unset or set to an empty list, no packages are built, and the snap will
only contain Debian packages for ROS 1.


catkin-tools-cmake-args
~~~~~~~~~~~~~~~~~~~~~~~

**Type**: list of strings

The arguments to pass to CMake.


Dependencies
------------

For core20 snaps, this plugin installs ``python3-catkin-tools`` and any other ROS 1
packages declared in the part.

If the project isn't using the ROS 1 extension, the part must set the ``ROS_DISTRO``
environment variable like so:

.. code-block:: yaml
    :caption: snapcraft.yaml

    build-environment:
      - ROS_DISTRO: "noetic"


How it works
------------

The Catkin plugin is designed to work alongside the :ref:`reference-ros-1-extension`.

During the build step, the plugin performs the following actions:

#. Source `catkin workspaces <http://wiki.ros.org/catkin/workspaces>`_ in any declared
   build snaps, stage snaps, and on the host system.
#. Install any missing project dependencies with rosdep.
#. Initialize the workspace to use catkin tools.
#. Overwrite the default catkin build, so that builds aren't affected by profile
   changes.
#. Configure the project for snap builds.
#. Run ``catkin build``, passing the arguments in the ``catkin-tools-cmake-args`` key to
   CMake.


Example
-------

The test suite in Snapcraft has a `catkin-tools-noetic-hello snap`_ built with the
Catkin Tools plugin.

.. _catkin-tools-noetic-hello snap: https://github.com/canonical/snapcraft/blob/2e9e72ab8a6531dbd1c576688255eae5da048bf8/tests/spread/plugins/v2/snaps/catkin-tools-noetic-hello/snap/snapcraft.yaml
