.. 34319.md

.. _docker-to-snap:

Docker to Snap
==============

Docker has greatly facilitated robotics software development by providing a way to package applications and their dependencies into portable containers. However, due to its cloud-oriented design, Docker poses some difficulties for developers when it comes to deploying software on a robotic device.

In this document, we are going to see when and how to migrate a ROS application currently deployed with Docker to a Snap.

When to migrate
===============

The software lifecycle for a robotics application typically consists of four stages: development, testing, deployment, and maintenance.

When transitioning from development and testing to deployment and maintenance, Docker’s limitations in embedded devices become apparent. Docker lacks dedicated high-level interfaces for accessing low-level hardware, a robust update system, state transactionality and are not integrated in terms of network. All of these require the user to implement workarounds that can be challenging and expose your application to security issues. This is where developers start their migration to snaps.

Snaps offer a better solution for the deployment and maintenance of the software lifecycle.

They provide robust security features to ensure the safety and integrity of software applications and are cryptographically-signed. Additionally, automatic over-the-air updates ensure that snaps are always up-to-date, while delta-binary downloads minimize bandwidth costs. Snaps also provide atomic install and removal functionality which ensures that the overall system is never in a broken state.

From Docker to Snap
===================

Docker images are built using Dockerfiles, which are essentially scripts containing a series of commands executed in sequence to define the image of your Docker container. Snaps are built based on a recipe declared in a `YAML file <https://snapcraft.io/docs/snapcraft-schema>`__. This similarity will prove convenient while converting from Docker to Snap as you will see hereafter.

A generic snapcraft.yaml file is defined by four main blocks:

-  snap’s metadata
-  build environment
-  parts definition
-  apps definition

More detailed information about creating a snapcraft yaml file can be found in `Creating snapcraft.yaml <https://snapcraft.io/docs/creating-snapcraft-yaml>`__.

Snap metadata
-------------

This is the data that describes your snap, including your snap’s name, version, icon location and summary.

For help completing these details, see `global metadata <https://snapcraft.io/docs/adding-global-metadata>`__.

Build environment
-----------------

Docker allows users to create images based on existing parent images that provide the required environment for the application. This is useful to avoid having to set up the same core libraries every time. Similarly, snaps provide `base snaps <https://snapcraft.io/docs/base-snaps>`__.

Both base snaps and parent Docker images serve as the foundation for building our application. For example, for a ROS Noetic application, in Docker you would include the Ubuntu 20.04 Lts image with the following command:

::

   FROM ubuntu:20.04

In a similar way, in snapcraft you will select the core20 base for the Snap as follows:

::

   base: core20

In Docker there are all sorts of images, even ones with ROS already installed. However, base snaps have the goal of guaranteeing a minimal, stable, maintained and secure environment . ROS is thus installed on top of this in the parts as we will see hereafter.

Visit the :ref:`base snaps documentation <base-snaps>` for more information.

Building the application
------------------------

Once the build environment is defined you can proceed to building your application.

When writing a Dockerfile, you think of the bash commands that would run on the host to install an application and all of its dependencies correctly. For a generic ROS application these commands would install ROS, install the project dependencies using rosdep, compile your ROS package and source it.

Building ROS
~~~~~~~~~~~~

In Dockerfile, ROS can be made available in two ways. Either by writing the bash commands required to install ROS as outlined in the `ROS documentation <http://wiki.ros.org/noetic/Installation/Ubuntu>`__ or by using a Docker parent image with ROS preinstalled. In both cases, ROS installation requires the following steps:

-  adding the ros package repositories
-  setting up the GPG keys
-  installing the ROS debian package.

In Snaps, adding the required package repository and setting up the keys is done by using the Snapcraft `package repository keyword <https://snapcraft.io/docs/package-repositories>`__. For example:

::

   package-repositories:
   - components: [main] # Apt repository components to enable
     formats: [deb] # List of deb types to enable
     key-id: C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 # 40 character GPG key identifier
     key-server: keyserver.ubuntu.com # Key server to fetch key
     suites: [focal] # Repository suites to enable
     type: apt # Specifies type of package-repository
     url: http://repo.ros2.org/ubuntu/main # Repository URL

The ROS debian packages are installed by defining a Snap `part <https://snapcraft.io/docs/snapcraft-parts-metadata>`__. Snap parts are recipes to build a piece of software and are driven via `plugins <https://snapcraft.io/docs/snapcraft-plugins>`__. When defining a part, you can search the `snapcraft plugins <https://snapcraft.io/docs/supported-plugins>`__ page to select the required plugin. If we solely want to install some ROS packages, no source code involved, we can use the nil plugin as follows:

::

   parts:
     ros2-humble-extension:
       plugin: nil

To add the packages that are needed to build our ROS project you can use the build-packages keyword, which allows for the installation of build-time dependencies. You can learn more about `Build and staging dependencies <https://snapcraft.io/docs/build-and-staging-dependencies>`__ in the documentation.

For a ROS project, the bare minimum packages required are those setting up a ROS workspace, hence they are added to the list of build-packages as follows:

::

   parts:
     ros2-humble-extension:
     plugin: nil # Plugin for parts with no source to import
     build-packages:
       - ros-humble-ros-environment
       - ros-humble-ros-workspace
       - ros-humble-ament-index-cpp
       - ros-humble-ament-index-python

It’s important to emphasize that build-packages are only used for building and won’t be packaged in the final Snap.

To ease sourcing the ROS workspace for an application, Snapcraft provides a `script <https://github.com/snapcore/snapcraft/blob/main/extensions/ros2/launch>`__ to do so. You can pull the script from the Snapcraft source and install it. Following the example above the ros2-humble-extension will look like this:

::

   parts:
     ros2-humble-extension:
     plugin: nil # Plugin for parts with no source to import
     build-packages:
       - ros-humble-ros-environment
       - ros-humble-ros-workspace
       - ros-humble-ament-index-cpp
       - ros-humble-ament-index-python
     source: $SNAPCRAFT_EXTENSIONS_DIR/ros2
     override-build: install -D -m 0755 launch ${SNAPCRAFT_PART_INSTALL}/snap/command-chain/ros2-launch # Install the ros2-launch script responsible of sourcing your ROS environment

Read more about `overriding the build step <https://snapcraft.io/docs/overrides>`__ in the documentation.

This is the process to set up ROS in a snap, and the process is the same for every ROS distribution.

Building your ROS package
~~~~~~~~~~~~~~~~~~~~~~~~~

After setting up ROS, you can proceed building the ROS package. In a Dockerfile the application’s source code is copied into the image filesystem, its dependencies installed and the package compiled. Something along the line of:

.. code:: bash

   COPY ./my-ros-application ./src/my-ros-application

   RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
       rosdep update --rosdistro $ROS_DISTRO && \
       rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
       cd /ros_ws && \
       catkin_make

In Snap, this is all handled automatically with the catkin plugin. Add it to your part as follows:

::

   parts:
     my-ros-application-part:
       plugin: catkin

Then you need to provide the plugin with information on the source it has to build.

The source can be a folder on your host or the link to a git repository. You can learn more about parts, how to set up a specific branch or a subfolder in the `Snapcraft parts metadata <https://snapcraft.io/docs/snapcraft-parts-metadata>`__ documentation. Let’s add an example source in the previous example:

::

   parts:
     my-ros-application-part:
       plugin: catkin
       source: https://github.com/my-company/my-ros-application.git
       source-branch: testing-branch

The plugin will take care of installing the dependencies defined in the package.xml file.

Remember to add any dependency that is required by the application and is not included in rosdep or installing instructions via the build-packages and stage-packages keywords.

By including these keywords in your part, you can ensure that all necessary packages and dependencies are properly installed.

Running the application
-----------------------

When deploying a ROS application in either Docker or Snapcraft, you can identify three main components that must be defined:

-  command; launch file or node to be run
-  enabling access to the necessary host resources (such as cameras, GPIO pins, network connections, and drivers), defining the launch file or rosnode to run
-  sourcing ROS and the workspace

Command
~~~~~~~

Docker offers various means of defining a command at container runtime (e.g. you can define a docker_compose.yaml file or use the CMD keyword in the Dockerfile). Also the entrypoint allows to specify bash commands that should be run when the container is started.

Snaps effectively allows you to define and isolate the pieces of your application that you want to expose to the rest of the system via the `apps <https://snapcraft.io/docs/snapcraft-app-and-service-metadata>`__ tag.

After having identified the command that launch your application you can add it with the command keyword as follows:

::

   apps:
     my-awesome-ros-app:
       command: opt/ros/noetic/bin/roslaunch my-awesome-ros-app app.launch

Access to host resources
~~~~~~~~~~~~~~~~~~~~~~~~

In order to allow access to host resources via Docker it is necessary to define port and volumes in the Dockerfile or when running the container. For example, Docker uses a virtual network to manage communication between containers. In order for the network to use the host’s network namespace, you have to run your container with –network=host option. Other examples can include using the –device flag to access a usb port, or various flags required for X11 forwarding to render GUI applications.

By default snap applications are confined and are not allowed to access any of the host resources. `Interfaces and plugs <https://snapcraft.io/docs/interface-management>`__ allow the user to define the resources on the host that the application will have access to. You can have a look at the list of `supported interfaces <https://snapcraft.io/docs/supported-interfaces>`__.

For a generic ROS application that communicates with other ROS components via topics, you will need the the “network” plug to grant the Snap access to the host’s network, and also the “network-bind” plug, which provides the Snap with the ability to bind to a specific IP address and port as required for ROS communication.For instance, if you needed X11 forwarding for a GUI you would use the `x11 interface <https://snapcraft.io/docs/x11-interface>`__ and so on. By adding the plugs, my app would look like this:

::

   apps:
     my-awesome-ros-app:
       command: opt/ros/noetic/bin/roslaunch my-awesome-ros-app app.launch
         plugs: [network, network-bind, x11]

Sourcing
~~~~~~~~

In Docker, sourcing of the ROS workspace is usually handled in the Dockerfile in the following way:

.. code:: bash

   RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
       echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

Sourcing can also be done using the Docker entrypoint script.

In Snapcraft this is achieved using the script installed earlier and executing it before launching our application through the command-chain tag as shown below:

::

   apps:
     my-awesome-ros-app:
       command: opt/ros/noetic/bin/roslaunch my-awesome-ros-app app.launch
       plugs: [network, network-bind]
       command-chain: [snap/command-chain/ros1-launch]

The command-chain keyword contains a list of commands to be executed prior to the app command. Because of it, we are relieved from the concern of having to source the ROS environment before launching our ROS application.

ROS extensions
--------------

To further simplify the deployment of snaps for robotic applications, ros-extensions have been implemented. ROS extensions automatically set up a fair share of what we’ve just detailed here, such as adding the ROS apt package repository, building ROS and defining the build environment. This means that developers can focus on building their application without worrying about the underlying ROS integration to Snapcraft.

The ros-extensions currently available are:

-  `ros1-noetic-extension <https://snapcraft.io/docs/ros1-extension>`__
-  `ros2-foxy-extension <https://snapcraft.io/docs/ros2-extension>`__
-  `ros2-humble-extension <https://snapcraft.io/docs/ros2-humble-extension>`__

You can read more about `snap extensions <https://snapcraft.io/docs/snapcraft-extensions>`__ in the official documentation.

Learn more
==========

In this post you have seen how to Snap an application by gathering information from its Dockerfile. To learn more about the core snap concepts look at the next resources:

-  `base snaps <https://snapcraft.io/docs/base-snaps>`__
-  `parts <https://snapcraft.io/docs/adding-parts>`__
-  `apps <https://snapcraft.io/docs/snapcraft-app-and-service-metadata>`__
-  `extensions <https://snapcraft.io/docs/snapcraft-extensions>`__
-  `plugs and interfaces <https://snapcraft.io/docs/interface-management>`__
-  `How to deploy robotic applications with snaps <https://snapcraft.io/docs/robotics>`__
