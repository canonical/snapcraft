.. 30809.md

.. _the-ros-2-humble-extension:

The ROS 2 Humble extension
==========================

This extension helps you snap ROS 2 applications for the `Humble Hawksbill <https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html>`__ distribution.

   ℹ This extension requires Snapcraft 7.3+.

How to use it
-------------

Add ``extensions: [ ros2-humble ]`` to the application definition in your ``snapcraft.yaml`` file. See the :ref:`ROS 2 deployment <ros-2-deployment-with-snaps-core22>` page for a complete tutorial on how to use this extension.

.. code:: yaml

       ...
   apps:
     colcon-ros2-humble-rlcpp-hello:
       command: opt/ros/humble/bin/ros2 run colcon_ros2_rlcpp_hello colcon_ros2_rlcpp_hello
       extensions: [ros2-humble]
       ...

What it does for the build-time environment
-------------------------------------------

-  Adds `ROS 2 APT package repository <http://repo.ros2.org/ubuntu/main>`__ build-time configuration for the snap, installing the necessary GPG key.
-  Adds the following ``build-packages``:

   -  ros-humble-ros-environment
   -  ros-humble-ros-workspace
   -  ros-humble-ament-index-cpp
   -  ros-humble-ament-index-python

-  Defines the ``build-environment``:

   -  ``ROS_VERSION = 2``
   -  ``ROS_DISTRO = humble``

What it does for the run-time environment
-----------------------------------------

-  Initializes runtime environment required by ROS 2 before launching the application, similar to sourcing the typical ROS 2 ``local_setup.bash``.

Further reading
---------------

-  For an example of this extension being used, see the *core22* section of the :ref:`ROS 2 deployment <ros-2-deployment-with-snaps-core22>` page.
-  For a complete picture of what this extension does, add it to your app definition and run ``snapcraft expand-extensions``.

..

   ℹ Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap. See :ref:`Snapcraft extensions <snapcraft-extensions>` for further details.
