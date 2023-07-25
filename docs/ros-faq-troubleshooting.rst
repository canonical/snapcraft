.. 29208.md

.. _ros-faq-troubleshooting:

ROS FAQ & Troubleshooting
=========================

This page reference `ROS and ROS 2 snap <https://snapcraft.io/docs/robotics>`__ common questions and troubleshooting:

Frequently Asked Questions
==========================

If you cannot find an answer to your question here, feel free to ask it on `the snapcraft forum <https://forum.snapcraft.io/>`__.

-  I cannot snap my application. What should I check?

   -  Snapcraft uses the familiar ROS tools (``rosdep``/``catkin``/``colcon`` etc). Which means that your application must follow the ROS directives for proper packaging, such as declaring all the necessary dependencies in the *package.xml* files or the install rules in your *CMakeFile.txt*. Make sure that these are in good order before attempting to create a snap.

-  Which base should I use, (``core18``, ``core20`` or ``core22)``?

   -  You should use the base that corresponds to your ROS version. That is,

      -  ``core18`` for ROS Melodic and ROS 2 Dashing
      -  ``core20`` for ROS Noetic and ROS 2 Foxy.
      -  ``core22`` for ROS 2 Humble.

-  For ROS 1, do I have to expose ``roscore`` from my snap?

   -  Exposing a ``roslaunch`` command from your snap will automatically launch a ``roscore`` if needed. The only reason to expose explicitly the roscore would be if you plan to start the ``roscore`` explicitly from your snap.

-  Where should my :file:`snapcraft.yaml` file live?

   -  Within the package:

      -  In ``core20`` the *snap/* directory should be located at the root of the package (next to your *package.xml* file)
      -  In ``core18`` the snap/ directory should be located either one folder behind your package root or at the root of your workspace

   -  Outside the package:

      -  Using a ``rosinstall`` file to download the sources.
      -  Using a single git repository holding the sources.

-  Can my snap save data on the host?

   -  The snap define some `environment variables <https://snapcraft.io/docs/environment-variables>`__ pointing to different locations that a snap can write to depending on the use case of your data.
   -  You can save data that are common across revisions of a snap. These directories **won’t be backed-up** and restored across revisions:

      -  ``$SNAP_COMMON``, typical value: ``/var/snap/hello-world/common``. Owned by ``root``
      -  ``$SNAP_USER_COMMON``, typical value: ``/home/$USER/snap/hello-world/common``. Owned by ``$USER``

   -  You can save data for a revision of a snap. This directory **is backed up** and restored across revisions:

      -  ``$SNAP_DATA``, typical value: ``/var/snap/hello-world/27``. Owned by ``root``
      -  ``$SNAP_USER_DATA``, typical value: ``/home/$USER/snap/hello-world/27``. Owned by ``$USER``

   -  Additionally, with the `home interface <https://snapcraft.io/docs/home-interface>`__, your snap could access the real ``$HOME`` of the user by accessing ``$SNAP_REAL_HOME``. # Troubleshooting

-  The command(s) ``rosrun`` and/or ``roslaunch`` are not available in my snap.

   -  If this happens, it means that your ROS project does not define a runtime dependency on either ``rosrun`` nor ``roslaunch`` anywhere. You can fix this by declaring the dependency in the appropriate ROS *package.xml* file. Another option is to list either (or both) ROS packages as ``stage-packages`` in your :file:`snapcraft.yaml` file. The ROS packages for ``rosrun`` and ``roslaunch`` are respectively:

      -  ``ros-${ROS-DISTRO}-rosbash``
      -  ``ros-${ROS-DISTRO}-roslaunch``.

-  With ``core18`` Catkin plugin creates an external link that prevents the security checks to pass.

   -  Please see: `Catkin generating an external link <https://forum.snapcraft.io/t/23269>`__.

-  Missing ``lapack`` and/or ``blas``.

   -  Paths to the libraries ``lapack`` and ``blas`` are not included in the library path by default. Thus, it must be extended manually in your app.

      .. code:: yaml

         environment:
           "LD_LIBRARY_PATH": "$LD_LIBRARY_PATH:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/blas:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/lapack"

-  Warning: *“This part is missing libraries that cannot be satisfied with any available stage-packages known to snapcraft”*.

   -  Some libraries are build-time only dependencies, but are still reported as run-time dependencies by :command:`snapcraft`. This warning is a false positive and will be fixed soon in snapcraft. For instance, when snapping ``ros2-demo`` you might encounter:

      .. code:: bash

         This part is missing libraries that cannot be satisfied with any available stage-packages known to snapcraft:
         # false-positive, none of the following are necessary at run-time
         libnddsc.so
         libnddscore.so
         libnddscpp.so
         librosidl_typesupport_connext_c.so
         librosidl_typesupport_connext_cpp.so
         librticonnextmsgcpp.so

-  | Strictly confined ROS 2 snaps shows an access error regarding shared memory.
   | If you see something similar to:

   ::

      [RTPS_TRANSPORT_SHM Error] Failed to create segment 86bb3c83d0835208: Permission denied -> Function compute_per_allocation_extra_size
      [RTPS_MSG_OUT Error] Permission denied -> Function init

-  At runtime the snap might show an error similar to :

   ::

      [rospack] Unable to create temporary cache file /home/USER/.ros/.rospack_cache.VyyWPF: Permission denied

   -  By default rospack and roslog write to the ``$HOME/.ros``. When strictly confined a snap who doesn’t have the `home interface <https://snapcraft.io/docs/home-interface>`__ cannot access the host ``$HOME``. Also, even with the `home plug <https://snapcraft.io/docs/home-interface>`__ the snap cannot access to hidden directories (.directories) for security reasons (like .ssh).

   -  To solve that we can write ROS logs in the ``$SNAP_USER_DATA`` environment variable. We can do so by defining the ROS environment variable ``ROS_HOME``. We can do so by adding to a snap app in the ``snapcraft.yaml``:

      ::

                 [...]
                 apps:
                   myapp:
                     environment:
                       ROS_HOME: $SNAP_USER_DATA/ros
                     command: [...]

   -  The data will also be available from the host in: ``~/snap/YOUR_SNAP_NAME/current/ros``

   ROS 2 communication library is trying to use the shared memory mechanism. Don’t worry, even if you see this error, the messages are going to be transmitted (just not through shared memory). If you want to use the shared memory of ROS 2 within snap, visit: :ref:`ros-2-shared-memory-in-snaps`

-  Calling :command:`snapcraft` give the following error:

   .. code::

      Failed to install GPG key: unable to establish connection to key server ‘keyserver.ubuntu.com’

      Recommended resolution: Verify any configured GPG keys.

      Detailed information: GPG key ID: C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 GPG key server: keyserver.ubuntu.com

   -  If the problem is persistent it’s most probably a DNS issue.

      -  To verify if it’s a DNS issue, if the following command succeeds it’s most probably a DNS issue:

         ``sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654``

      -  We can also verify that the port ``11371`` is not blocked or occupied.
