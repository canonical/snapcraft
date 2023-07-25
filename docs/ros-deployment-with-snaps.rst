.. 7822.md

.. _ros-deployment-with-snaps:

ROS deployment with snaps
=========================

`ROS <https://www.ros.org/>`__ is distributed via Open Robotics’ own Debian archive, along with many community-supported tools. It’s possible to get your own application into their archive as well, but it requires that the application is open-source.

You’re also left with the question of how to update ROS and your application on a robotic platform that has already been shipped. With *snapcraft* it’s just one command to bundle a specific ROS version along with your application into a snap that works anywhere and can be automatically updated.

Why are snaps good for ROS projects?
------------------------------------

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** Bundle the exact versions of the tools you need, including ROS, along with all of your app’s dependencies, be they modules or system libraries.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision with data preserved.

Build a snap in 20 minutes
~~~~~~~~~~~~~~~~~~~~~~~~~~

Ready to get started? By the end of this guide, you’ll understand how to make a snap of your app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

   ℹ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

There are currently two supported bases for ROS, core18 and core20.

`core18 <https://snapcraft.io/core18>`__ is based on\ `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.

`core20 <https://snapcraft.io/core20>`__ is based on\ `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`__.

Let us explore the differences between core18 and core20.

.. _ros-deployment-with-snaps-core18:

core18
~~~~~~

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for an example project, `ros-talker-listener <https://github.com/snapcraft-docs/ros-talker-listener>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: ros-talker-listener
   version: '0.1'
   summary: ROS Talker/Listener Example
   description: |
     This example launches a ROS talker and listener.

   confinement: devmode
   base: core18

   parts:
     ros-tutorials:
       plugin: catkin
       source: https://github.com/ros/ros_tutorials.git
       source-branch: melodic-devel
       source-space: roscpp_tutorials/

   apps:
     ros-talker-listener:
       command: roslaunch roscpp_tutorials talker_listener.launch

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/ros-talker-listener

Metadata
~~~~~~~~

The ``snapcraft.yaml`` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: ros-talker-listener
   version: '0.1'
   summary: ROS Talker/Listener Example
   description: |
     This example launches a ROS talker and listener.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

Versions carry no semantic meaning in snaps and this version is arbitrary. It’s also possible to write a script to calculate the version, or to take a tag or commit from a git repository.

The ``summary`` can not exceed 78 characters. You can use a pipe symbol ‘\|’ in the ``description`` key to declare a multi-line description.

For more information about top level metadata, see, `top-level-metadata <https://snapcraft.io/docs/snapcraft-top-level-metadata>`__. #### Base

The base keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core18

`core18 <https://snapcraft.io/core18>`__ is the current standard base for snap building and is based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__. It is therefore the base for ROS Melodic and ROS2 Dashing snaps.

Security model
~~~~~~~~~~~~~~

To get started, we won’t confine this application. Unconfined applications, specified with ``devmode``, can only be released to the hidden “edge” :term:`channel`.

.. code:: yaml

   confinement: devmode

For more information about security model, see, `choosing-a-security-model <https://snapcraft.io/docs/choosing-a-security-model>`__.

Parts
~~~~~

Parts define how to build your app and can be anything: programs, libraries, or other assets needed to create and run your application. Their source can be local directories, remote git repositories, or tarballs. In this example, we have a single part: ros-tutorials.

Snapcraft relies on well known and well established ROS1 tools such as, in this example, ``catkin``.

**Note:** Often, ROS developers rely on the *devel* space of their ``catkin`` workspace. As a result, it’s easy to forget the importance of complete install rules, i.e. rules for installing every component of the package necessary to run, or every component necessary to use a given library.

The Catkin packages you’re building must have install rules, or else snapcraft won’t know which components to place into the snap. Make sure you install binaries, libraries, header files, launch files, etc.

.. code:: yaml

   parts:
     ros-tutorials:
       plugin: catkin
       source: https://github.com/ros/ros_tutorials.git
       source-branch: melodic-devel
       source-space: roscpp_tutorials/

For more details on catkin-specific metadata, see `The catkin plugin <https://snapcraft.io/docs/catkin-plugin>`__ and for more information about general parts metadata, see, `parts-metadata <https://snapcraft.io/docs/snapcraft-parts-metadata>`__.

Apps
~~~~

Apps are the commands and services exposed to end users. Each entry under ``apps`` is the command name that should be exposed to the end users.

The ``command`` specifies the full path to the binary to be run.

.. code:: yaml

   apps:
     ros-talker-listener:
       command: roslaunch roscpp_tutorials talker_listener.launch

In snap, an application is usually prefixed by the snap name so that the application ``my-app`` from the snap ``my-snap`` can be executed calling ``my-snap.my-app``. However, if both the snap and the app are called the same, as is the case in our ROS example, the execution command collapses to avoid the tediousness of writing twice the same words. The command ``ros-talker-listener.ros-talker-listener`` simply becomes ``ros-talker-listener``.

Building the snap
~~~~~~~~~~~~~~~~~

After you have created the :file:`snapcraft.yaml` file, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped ros-talker-listener_0.1_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install ros-talker-listener_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ ros-talker-listener

   ... logging to /home/user/snap/ros-talker-listener/x1/.ros/log/40e8a1a6-9f0b-11ec-9d79-ef345aa758b5/roslaunch-computer-1812506.log
   Checking log directory for disk usage. This may take a while.
   Press Ctrl-C to interrupt
   Done checking log file disk usage. Usage is <1GB.

   started roslaunch server http://computer:40283/

   SUMMARY
   ========

   PARAMETERS
   * /rosdistro: noetic
   * /rosversion: 1.15.14

   NODES
    /
      listener (roscpp_tutorials/listener)
      talker (roscpp_tutorials/talker)
   auto-starting new master
   process[master]: started with pid [1812557]
   ROS_MASTER_URI=http://localhost:11311

   setting /run_id to 40e8a1a6-9f0b-11ec-9d79-ef345aa758b5
   process[rosout-1]: started with pid [1812567]
   started core service [/rosout]
   process[listener-2]: started with pid [1812570]
   process[talker-3]: started with pid [1812571]
   [ INFO] [1646763123.183650984]: hello world 0
   [ INFO] [1646763123.484887322]: I heard: [hello world 0]
   ...

Removing the snap is simple too:

::

   $ sudo snap remove ros-talker-listener

Once done developing your snap, you can easily clean up the build environment:

.. code:: bash

   $ snapcraft clean


----------

.. _ros-deployment-with-snaps-core20:

core20
~~~~~~

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire snapcraft.yaml file for an example project,\ `ros-talker-listener-core20 <https://github.com/snapcraft-docs/ros-talker-listener-core20>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: ros-talker-listener
   version: '0.1'
   summary: ROS Talker/Listener Example
   description: |
    This example launches a ROS talker and listener.

   confinement: devmode
   base: core20

   parts:
    ros-tutorials:
      plugin: catkin
      source: https://github.com/ros/ros_tutorials.git
      source-branch: noetic-devel
      catkin-packages: [roscpp_tutorials]
      stage-packages:
          - ros-noetic-roslaunch

   apps:
    ros-talker-listener:
      command: opt/ros/noetic/bin/roslaunch roscpp_tutorials talker_listener.launch
      extensions: [ros1-noetic]

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/ros-talker-listener-core20


Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the `presentation of your app in the Snap Store <https://snapcraft.io/plotjuggler>`__.

.. code:: yaml

   name: ros-talker-listener
   version: '0.1'
   summary: ROS Talker/Listener Example
   description: |
    This example launches a ROS talker and listener.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

Versions carry no semantic meaning in snaps and this version is arbitrary. It’s also possible to write a script to calculate the version, or to take a tag or commit from a git repository.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the ``description`` key to declare a multi-line description.

For more information about top level metadata, see, `top-level-metadata <https://snapcraft.io/docs/snapcraft-top-level-metadata>`__. #### Base

The base keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core20

`core20 <https://snapcraft.io/core20>`__ is the current standard base for snap building and is based on\ `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`__. It is therefore the base for ROS Noetic and ROS 2 Foxy snaps.


Security model
~~~~~~~~~~~~~~

To get started, we won’t confine this application. Unconfined applications, specified with ``devmode``, can only be released to the “edge” :term:`channel`.

.. code:: yaml

   confinement: devmode

For more information about security model, see, `choosing-a-security-model <https://snapcraft.io/docs/choosing-a-security-model>`__.


Parts
~~~~~

Parts define how to build your app and can be anything: programs, libraries, or other assets needed to create and run your application. Their source can be local directories, remote git repositories, or tarballs. In this example, we have a single part: ros-tutorials.

Snapcraft relies on well known and well established ROS tools such as, in this example, ``catkin``.

**Note:** Often, ROS developers rely on the *devel* space of their ``catkin`` workspace. As a result, it’s easy to forget the importance of complete install rules, i.e. rules for installing every component of the package necessary to run, or every component necessary to use a given library. The ``catkin`` packages you’re building must have install rules, or else :command:`snapcraft` won’t know which components to place into the snap. Make sure you install binaries, libraries, header files, launch files, etc.

.. code:: yaml

   parts:
    ros-tutorials:
      plugin: catkin
      source: https://github.com/ros/ros_tutorials.git
      source-branch: noetic-devel
      catkin-packages: [roscpp_tutorials]
      stage-packages:
          - ros-noetic-roslaunch

For more details on catkin-specific metadata, see\ `The catkin plugin <https://snapcraft.io/docs/catkin-plugin>`__ and for more information about general parts metadata, see, `parts-metadata <https://snapcraft.io/docs/snapcraft-parts-metadata>`__.


Apps
~~~~

Apps are the commands and services exposed to end users. Each entry under ``apps`` is the command name that should be exposed to the end users.

The command specifies the path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
    ros-talker-listener:
      command: opt/ros/noetic/bin/roslaunch roscpp_tutorials talker_listener.launch
      extensions: [ros1-noetic]

In snap, an application is usually prefixed by the snap name so that the application ``my-app`` from the snap ``my-snap`` can be executed calling ``my-snap.my-app``. However, if both the snap and the app are called the same, as is the case in our ROS example, the execution command collapses to avoid the tediousness of writing twice the same words. The command ``ros-talker-listener.ros-talker-listener`` simply becomes ``ros-talker-listener``.


Building the snap
~~~~~~~~~~~~~~~~~

After you have created the :file:`snapcraft.yaml` file, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft --enable-experimental-extensions
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped ros-talker-listener_0.1_amd64.snap

The resulting snap can be immediately installed. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. Furthermore, using the ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install ros-talker-listener_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ ros-talker-listener

   ... logging to /home/user/snap/ros-talker-listener/x1/.ros/log/40e8a1a6-9f0b-11ec-9d79-ef345aa758b5/roslaunch-computer-1812506.log
   Checking log directory for disk usage. This may take a while.
   Press Ctrl-C to interrupt
   Done checking log file disk usage. Usage is <1GB.

   started roslaunch server http://computer:40283/

   SUMMARY
   ========

   PARAMETERS
   * /rosdistro: noetic
   * /rosversion: 1.15.14

   NODES
    /
      listener (roscpp_tutorials/listener)
      talker (roscpp_tutorials/talker)
   auto-starting new master
   process[master]: started with pid [1812557]
   ROS_MASTER_URI=http://localhost:11311

   setting /run_id to 40e8a1a6-9f0b-11ec-9d79-ef345aa758b5
   process[rosout-1]: started with pid [1812567]
   started core service [/rosout]
   process[listener-2]: started with pid [1812570]
   process[talker-3]: started with pid [1812571]
   [ INFO] [1646763123.183650984]: hello world 0
   [ INFO] [1646763123.484887322]: I heard: [hello world 0]
   ...

Removing the snap is simple too:

.. code:: bash

   $ sudo snap remove ros-talker-listener

Once done developing your snap, you can easily clean up the build environment:

.. code:: bash

   $ snapcraft clean --enable-experimental-extensions



Publishing your snap
--------------------

To share your snaps you need to publish them in the Snap Store. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads and control publishing.

You’ll need to choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the :command:`snapcraft` command is authenticated using the email address attached to your Snap Store account:

.. code:: bash

   $ snapcraft login

Reserve a name for your snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

.. code:: bash

   $ snapcraft register myrossnap

Be sure to update the ``name:`` in your ``snapcraft.yaml`` to match this registered name, then run :command:`snapcraft` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   $ snapcraft upload --release=edge myrossnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and enable automatic builds (see :ref:`Build from GitHub <build-from-github>`) so that any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first ROS snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
