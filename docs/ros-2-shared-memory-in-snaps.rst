.. 31214.md

.. _ros-2-shared-memory-in-snaps:

ROS 2 shared memory in snaps
============================

Strictly confined ROS 2 snaps shows an access error regarding shared memory. If you see something similar to:

::

   [RTPS_TRANSPORT_SHM Error] Failed to create segment 86bb3c83d0835208: Permission denied -> Function compute_per_allocation_extra_size
   [RTPS_MSG_OUT Error] Permission denied -> Function init

ROS 2 communication library is trying to use the shared memory mechanism. But don’t worry, even if you see this error, the messages are going to be transmitted (just not through shared memory).

Here, we present solutions to make the error message disappear by activating snap shared memory interface or simply disabling ROS 2 shared memory.

If you want a detail explanation about ROS 2 shared memory and why It doesn’t work right away in snaps please visit the blog: `how-to-use-ros-2-shared-memory-in-snaps <https://canonical.com/blog/how-to-use-ros-2-shared-memory-in-snaps>`__.

Note that everything discussed hereafter is exemplified on `GitHub <https://github.com/ubuntu-robotics/ros-snaps-examples/tree/main/shared_memory_foxy_core20>`__.

Public shared memory interface for ROS 2
----------------------------------------

Snap is providing an interface called `shared-memory <https://snapcraft.io/docs/shared-memory-interface>`__. This interface allows different snaps to have access (read and/or write) to a specified path. This is meant to share resources in ``/dev/shm`` across snaps. To do so, we have to declare both a :ref:`slot as well as a plug <interface-management>`. Before running the snap, we will have to manually connect theseplug, possibly connecting several other snaps to the same slot.

The slot is the one defining the shared memory paths to be accessed. With ROS 2, we actually need access to everything within the ``/dev/shm`` directory (due to the semaphore temporary files). An example of the slot and plug is as follows:

::

   slots:
     shmem-slot:
       interface: shared-memory
       write: ['*'] # paths are relative to /dev/shm
       private: false
   plugs:
     shmem-plug:
       interface: shared-memory
       shared-memory: shmem-slot
       private: false

Then both, the plug and the slot are added to the apps:

::

   apps:
     my-ros-2-app:
       [...]
       plugs: [network, network-bind, shmem-plug]
       slots: [shmem-slot]

Once the snap is built and installed, we can connect the shared memory with the command

::

   sudo snap connect my_snap_name:shmem-plug my_snap_name:shmem-slot

Additional snaps will simply have to create their own plug and connect it to the very same ``slot``.

This solution has an important overhead, since one must define slots and plugs, possibly across multiple snaps. But it is the de-facto way to enable the use of the shared memory feature in strictly confined snaps. Note that ``snapd 2.56.2`` (or above) is necessary.

You can find a complete example of a ROS 2 snap using the public shared memory interface on `GitHub <https://github.com/ubuntu-robotics/ros-snaps-examples/tree/main/shared_memory_foxy_core20/public-shared-memory>`__.

Private shared memory interface
-------------------------------

Snap is providing another interface called `private shared memory <https://snapcraft.io/blog/private-shared-memory-support-for-snaps>`__ that vastly simplifies shared memory support with snap. The `private shared memory <https://snapcraft.io/blog/private-shared-memory-support-for-snaps>`__ is a subset of the `shared memory interface <https://snapcraft.io/docs/shared-memory-interface>`__. Without modifying your software, all calls to ``/dev/shm`` are going to be bound to ``/dev/shm/snap.SNAP_NAME`` automatically. It can be activated by simply adding the plug to the :file:`snapcraft.yaml` file:

::

   plugs:
     shared-memory:
       private: true

In a ROS 2 context, adding the `private shared memory <https://snapcraft.io/blog/private-shared-memory-support-for-snaps>`__ allows the shared memory to work within a given snap, hence benefiting from better performance.

Everything is fine within the snap, but the application is mute to the outside. ROS 2 applications running on the host, but outside the snap, will not be able to see the topics from the snap; let alone to subscribe to them.

This being said, it is important to note that if you access these topics published from the snap from another computer, it will work seamlessly as the data are shared via UDP.

You can find a complete example of a ROS 2 snap using the private shared memory interface on `GitHub <https://github.com/ubuntu-robotics/ros-snaps-examples/tree/main/shared_memory_foxy_core20/private-shared-memory>`__.

Disabling shared memory for ROS 2
---------------------------------

FastDDS offers two options to totally disable the shared memory feature; either at compile time or at run time. We are detailing both options hereafter.

At compile-time
~~~~~~~~~~~~~~~

FastDDS offers an option to compile without the shared memory feature by simply specifying a CMake variable: ``-DSHM_TRANSPORT_DEFAULT=OFF``. With this, no shared memory nor any associated files – ciao the error message. Adding the following FastDDS part to your :file:`snapcraft.yaml` file will disable FastDDS at compile time:

::

     fastdds:
       plugin: colcon
       source: https://github.com/eProsima/Fast-DDS.git
       source-tag: "v2.1.1" # Use tag according to your ROS version
       colcon-cmake-args:
               - -DCMAKE_BUILD_TYPE=Release
               - -DSHM_TRANSPORT_DEFAULT=OFF
       build-packages:
               - libasio-dev
               - ros-foxy-fastcdr # Replace by used ROS 2 release
               - nlohmann-json3-dev
               - ros-foxy-tinyxml2-vendor  # Replace by used ROS 2 release
       stage-packages:
               - ros-foxy-fastcdr  # Replace by used ROS 2 release
               - ros-foxy-tinyxml2-vendor  # Replace by used ROS 2 release

Of course, the main drawback of this approach is that we have to recompile FastDDS with every snap.

You can find a complete example of a ROS 2 snap using the FastDDS with shared memory disable at compile time on `GitHub <https://github.com/ubuntu-robotics/ros-snaps-examples/tree/main/shared_memory_foxy_core20/disable-shared-memory-compile-time>`__.

Disabling shared memory at run-time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

FastDDS also allows for providing a `configuration XML file <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`__ at runtime in order to customize several aspects of the middleware. Such as, forcing the transport to use UDPv4. The XML profile is passed through an `environment variable <https://fast-dds.docs.eprosima.com/en/latest/fastdds/env_vars/env_vars.html#fastrtps-default-profiles-file>`__: Under your ``snap/local`` directory, create the file ``fastdds_no_shared_memory.xml`` with the following content:

::

   <?xml version="1.0" encoding="UTF-8" ?>
       <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
           <transport_descriptors>
               <transport_descriptor>
                   <transport_id>CustomUdpTransport</transport_id>
                   <type>UDPv4</type>
               </transport_descriptor>
           </transport_descriptors>

           <participant profile_name="participant_profile" is_default_profile="true">
               <rtps>
                   <userTransports>
                       <transport_id>CustomUdpTransport</transport_id>
                   </userTransports>

                   <useBuiltinTransports>false</useBuiltinTransports>
               </rtps>
           </participant>
       </profiles>

And then, you can add the proper ``part`` to place your profile and set the environment variable to your app in your :file:`snapcraft.yaml` file:

::

   parts:
     [...]
     config:
       plugin: dump
       source: snap/local/
       organize:
         'fastdds_no_shared_memory.xml': usr/share/
   apps:
     my-ros-2-app:
       [...]
       environment:
           FASTRTPS_DEFAULT_PROFILES_FILE: ${SNAP}/usr/share/fastdds_no_shared_memory.xml

This is much easier to set up and to change in subsequent releases of a snap.

You can find a complete example of a ROS 2 snap using the FastDDS with shared memory disable at run time on `GitHub <https://github.com/ubuntu-robotics/ros-snaps-examples/tree/main/shared_memory_foxy_core20/disable-shared-memory-run-time>`__.
