.. 35431.md

.. _ros-architectures-with-snaps:

ROS architectures with snaps
============================

This documentation is meant to present the different snap architectures that developers can adopt for their ROS applications.

There are two main approaches that can be taken: the monolithic approach and the multi-snap approach.

To represent and image the different architectures we will use an example of a robot meant to clean and patrol. This robot consists of three application. The main one being the Brain app. The Brain app is responsible for all the basic features of a robot, controlling the robot, navigating, etc. The Patrol app is simply responsible for patrolling and sending the rights commands to the Brain app. Similarly, the Clean app is meant to send the right cleaning command and logic to the Brain app. The same logic could be used for another robot with the Brain app being a reusable and generic application.

Monolithic architecture
-----------------------

The monolithic snap for a ROS application is a common approach that involves shipping the complete software stack and applications in one snap. **This approach is recommended for first-time snaps developers.**

The monolithic snap architecture includes all binaries, libraries, configurations, and dependencies in one snap. This means that only one snapcraft.yaml is required to snap all applications. The ``snapcraft.yaml`` can still contain multiple `parts <https://snapcraft.io/docs/snapcraft-parts-metadata>`__ in case some dependencies of the ROS workspace must be built.

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/optimized/2X/c/c75996c9400e2b531c8ba5f2b58470ac277da481_2_690x353.png
   :alt: image|690x353


As you can see in the picture, the monolithic snap called “my-monolithic-snap” contains all the system and ROS dependencies as well as the configurations and everything needed at run-time. Additionally, this single monolithic snap is exposing three applications. The three applications are communicating using ROS (Could be the network, shared memory, etc.).

Complexity
~~~~~~~~~~

A monolithic architecture is a common approach since snaps can bundle all dependencies and applications in a single snap.

This approach require writing only one ``snapcraft.yaml`` to snap all our applications and stack.

The ``snapcraft.yaml`` can still contain multiple parts in case some dependencies of the ROS workspace must be built.

Stability
~~~~~~~~~

All the binaries, libraries, configurations, etc live inside the same snap. Since snaps are immutable, having everything in one snap makes an application robust. Additionally, when the snap is being tested, users or devices will always get the same behaviour.

Size
~~~~

Having everything in one snap avoids duplicating files or binaries across snaps. This is beneficial for devices with limited storage space.

Deployment
~~~~~~~~~~

In case there is a change in the snap, rebuilding the whole snap is necessary (which can take time for complex robotics applications). Due to changes in libraries (Ubuntu libraries or even ROS ones), the final snap will probably differ a lot from the original one. Even with delta updates, updating the snap will most probably be in the order of magnitude of the snap size.

Reusability
~~~~~~~~~~~

Having two robotics applications using the same “Brain” (navigation stack, hardware controllers, etc) will require building and distributing two completely different snaps. This means that there is very little chance that such a monolithic snap will be deployed on another robot.

You can find a complete implementation of the TurtleBot3 as a monolithic snap\ `on GitHub <https://github.com/canonical/turtlebot3c-snap>`__.

Pros of monolithic snaps
^^^^^^^^^^^^^^^^^^^^^^^^

-  Easy to set up and maintain

-  Self-sufficient snaps

-  Robust

-  Space efficient

Cons of monolithic snaps
^^^^^^^^^^^^^^^^^^^^^^^^

-  Updates can be heavy to deploy on devices

-  Reduced reusability

-  Require the release of a completely new snap if the application is slightly changed

Multi snaps architecture
------------------------

ROS is a modular system that allows for the exchange of data between applications via the network. With the right network interfaces, two applications can exchange data via ROS without being in the same snap, enabling a complete robot software stack to be deployed over multiple snaps.

The composite of these snaps will then be the complete software stack of the robot.

In the case of our mobile robot application, the snap architecture will look like:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/optimized/2X/6/6155ac886e96220b7d6184dbb3b5a2e2b1ff56f8_2_690x303.png
   :alt: image|690x303


As you can see in the picture, the “Brain” snap called “my-brain-snap” contains everything for the Brain app. Meaning that “my-brain-snap” is carrying its own ROS installation as well as dependencies. Additionally, the “my-patrol-snap” and “my-clean-snap” also carry their ROS installation and specific dependencies. The three apps are exposing their own applications working together communicating with each other with their ROS communication (topics, services, etc) through the host. Each snap is an application, you shouldn’t snap just a library or a ROS package. The content of the snap should make sense as an application.

.. _complexity-1:

Complexity
~~~~~~~~~~

Deploying multiple snaps means multiple snapcraft.yaml files to define, build, and maintain, making the multi-snap architecture more complex.

ROS 2 DDS default implementation ``FastDDS`` can use shared memory to exchange faster when two `DomainParticipants <https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/domain/domainparticipant.html>`__ are on the same host. You can enable shared memory across multi snaps with the `addition of an extra interface <https://snapcraft.io/docs/ros2-shared-memory-in-snaps>`__.

.. _stability-1:

Stability
~~~~~~~~~

Multi-snap architecture will require additional testing to ensure that the multiple snaps are working well together, especially when updating the snaps.

In case some snaps will no longer be compatible with each other, :term:`channels` could be used to clarify the compatibility between snaps.

Moreover, Ubuntu Core’s `validation set <https://ubuntu.com/core/docs/reference/assertions/validation-set>`__ prevents incompatible software installation on a device.

.. _size-1:

Size
~~~~

Deploying robot software via multiple snaps is going to take more space on the disk. Since snaps bundle all their dependencies, splitting a robot software stack in multiple snaps will most probably mean shipping different snaps containing some common libraries (e.g. ROS base libraries).

.. _deployment-1:

Deployment
~~~~~~~~~~

When updating a multi snap architecture you don’t have to redeploy and update all the other snaps that didn’t change. This reduces bandwidth constrains.

Finally, if a set of snaps is needed to run an application, You can pair multiple snaps together for deployment via a `private Snap Store <https://ubuntu.com/core/docs/dedicated-snap-stores>`__ or by creating a custom `Ubuntu Core image <https://ubuntu.com/core/docs/build-an-image>`__.

.. _reusability-1:

Reusability
~~~~~~~~~~~

Developers could keep the benefits of ROS modularity and be able to reuse one Brain snap for all the robots while deploying an “application” snap to enable a certain function on the robot. Through this multiple applications could be developed, and they could all work along the Brain snap bringing the basic functionality of a robot.

The developed applications relying on the Brain of a certain robot could be reused on another robot as long as the interface is standardized (same topic names, units, etc).

Pros of multi snaps
^^^^^^^^^^^^^^^^^^^

-  Brings reusability for the snaps

-  Allows modularity in applications

-  Reduces update bandwidth cost in case of an update

Cons of multi snaps
^^^^^^^^^^^^^^^^^^^

-  Less space efficient

-  Potential snaps incompatibility

-  Harder to maintain

-  Might require additional `interfaces <https://snapcraft.io/docs/supported-interfaces>`__

-  Need coordination between the releases of the snaps

Conclusion
----------

While the monolithic approach is relatively easy to set up and maintain, it does come with some downsides. For one thing, it requires heavy updates, which can be time-consuming and potentially disruptive. Additionally, it can be less reusable, as developers may need to duplicate code across different applications.

On the other hand, the multi-snap approach offers greater reusability and modularity, which can be a significant advantage in certain contexts. However, it does come with some trade-offs as well. For instance, it may be less space-efficient and harder to maintain than the monolithic approach. Furthermore, it may require additional interfaces and coordination between snap releases in order to function properly.
