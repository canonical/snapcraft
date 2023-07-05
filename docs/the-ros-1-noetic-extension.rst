.. 20070.md

.. _the-ros-1-noetic-extension:

The ROS 1 Noetic Extension
==========================

This extension helps you snap ROS1 applications for the `Noetic Ninjemys <https://wiki.ros.org/noetic>`__ distribution.

   ℹ This extension requires Snapcraft 7.3+.

How to use it
-------------

Add ``extensions: [ ros1-noetic ]`` to the application definition in your ``snapcraft.yaml`` file. See for a complete tutorial on how to use this extension.

.. code:: yaml

   apps:
     catkin-ros1-run:
       command: opt/ros/noetic/bin/roslaunch listener talk_and_listen.launch
       plugs: [network, network-bind]
       extensions: [ros1-noetic]
       ...

What it does for the build-time environment
-------------------------------------------

-  Adds `ROS APT package repository <http://packages.ros.org/ros/ubuntu>`__ build-time configuration for the snap, installing the necessary GPG key.
-  Adds ``ros-noetic-catkin`` to ``build-packages``.

What it does for the run-time environment
-----------------------------------------

-  Initializes runtime environment required by ROS before launching the application, similar to sourcing the typical ROS ``setup.sh`` or ``local_setup.sh``.

Further reading
---------------

-  For a usage example of this extension, please visit: `ros-applications <https://snapcraft.io/docs/ros-applications>`__ (core20 part).
-  For a complete picture of what this extension does, add it to your app definition and run ``snapcraft expand-extensions``.

..

   ℹ Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap. See :ref:`Snapcraft extensions <snapcraft-extensions>` for further details.
