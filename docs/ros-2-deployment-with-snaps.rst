.. 7823.md

.. _ros-2-deployment-with-snaps:

ROS 2 deployment with snaps
===========================

`ROS 2 <https://index.ros.org/doc/ros2/>`__ is distributed via Open Robotics’ own Debian archive, along with many community-supported tools. It’s possible to get your own application into their archive as well, but it requires that the application is open-source.

You’re also left with the question of how to update ROS 2 and your application on a robotic platform that has already been shipped. With *snapcraft* it’s just one command to bundle a specific ROS 2 version along with your application (proprietary or open-source) into a snap that works anywhere and can be automatically updated.

Why are snaps good for ROS 2 projects?
--------------------------------------

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** Bundle the exact versions of the tools you need, including ROS 2, along with all of your app’s dependencies, be they modules or system libraries.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision with data preserved.

Build a snap in 20 minutes
~~~~~~~~~~~~~~~~~~~~~~~~~~

Ready to get started? By the end of this guide, you’ll understand how to make a snap of your app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

   ℹ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

There are currently three supported bases for ROS 2, core18, core20 and core22.

`core18 <https://snapcraft.io/core18>`__ is based on\ `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.

`core20 <https://snapcraft.io/core20>`__ is based on\ `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`__.

`core22 <https://snapcraft.io/core22>`__ is based on\ `Ubuntu 22.04 LTS <http://releases.ubuntu.com/22.04/>`__.

Let us explore the differences between the different bases.


.. _ros-2-deployment-with-snaps-core18:

core18
~~~~~~

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for an example project, `ros2-talker-listener <https://github.com/snapcraft-docs/ros2-talker-listener>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: ros2-talker-listener
   version: '0.1'
   summary: ROS 2 Talker/Listener Example
   description: |
     This example launches a ROS 2 talker and listener.

   base: core18
   confinement: devmode

   parts:
     ros-demos:
       plugin: colcon
       source: https://github.com/ros2/demos.git
       source-branch: dashing
       colcon-rosdistro: dashing
       colcon-source-space: demo_nodes_cpp
       stage-packages: [ros-dashing-ros2launch]

   apps:
     ros2-talker-listener:
       command: opt/ros/dashing/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/ros2-talker-listener

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the `presentation of your app in the Snap Store <https://snapcraft.io/plotjuggler>`__.

.. code:: yaml

   name: ros2-talker-listener
   version: '0.1'
   summary: ROS 2 Talker/Listener Example
   description: |
     This example launches a ROS 2 talker and listener.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

Versions carry no semantic meaning in snaps and this version is arbitrary. It’s also possible to write a script to calculate the version, or to take a tag or commit from a git repository.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the ``description`` key to declare a multi-line description.

For more information about top level metadata, see, :ref:`top-level-metadata <snapcraft-top-level-metadata>`.

Base
~~~~

The base keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core18

`core18 <https://snapcraft.io/core18>`__ is based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__. It is therefore the base for ROS Melodic and ROS 2 Dashing snaps.

Security model
~~~~~~~~~~~~~~

To get started we won’t confine this application. Unconfined applications, specified with ``devmode``, can only be released to the hidden “edge” :term:`channel`.

.. code:: yaml

   confinement: devmode

For more information about security model, see, :ref:`choosing-a-security-model <choosing-a-security-model>`.

Parts
~~~~~

Parts define how to build your app and can be anything: programs, libraries, or other assets needed to create and run your application. Their source can be local directories, remote git repositories, or tarballs. In this example, we have a single part: ros-demos.

Snapcraft relies on well known and well established ROS tools such as, in this example, ``colcon``.

**Note:** It is easy to forget the importance of complete install rules, i.e. rules for installing every component of the package necessary to run, or every component necessary to use a given library.

The packages you’re building must have install rules, or else :command:`snapcraft` won’t know which components to place into the snap. Make sure you install binaries, libraries, header files, launch files, etc.

.. code:: yaml

   parts:
     ros-demos:
       plugin: colcon
       source: https://github.com/ros2/demos.git
       source-branch: dashing
       colcon-rosdistro: dashing
       colcon-source-space: demo_nodes_cpp
       stage-packages: [ros-dashing-ros2launch]

For more details on colcon-specific metadata, see :ref:`the colcon plugin documentation <the-colcon-plugin>`. For more information about general parts metadata, see :ref:`parts-metadata <snapcraft-parts-metadata>`.

Apps
~~~~

Apps are the commands and services exposed to end users. Each entry under ``apps`` is the command name that should be exposed to the end users.

The ``command`` specifies the full path to the binary to be run.

.. code:: yaml

   apps:
     ros2-talker-listener:
       command: opt/ros/dashing/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py

In snap, an application is usually prefixed by the snap name so that the application ``my-app`` from the snap ``my-snap`` can be executed calling ``my-snap.my-app``. However, if both the snap and the app are called the same, as is the case in our ROS 2 example, the execution command collapses to avoid the tediousness of writing twice the same words. The command ``ros2-talker-listener.ros2-talker-listener`` simply becomes ``ros2-talker-listener``.

Building the snap
~~~~~~~~~~~~~~~~~

After you have created the :file:`snapcraft.yaml` file, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped ros2-talker-listener_0.1_amd64.snap

.. warning::

   The warnings regarding missing libraries that you might see when building your snap are false positive. These libraries are build time dependencies only. [/note] The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install ros2-talker-listener_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ ros2-talker-listener
   [INFO] [launch]: All log files can be found below /home/user/snap/ros2-talker-listener/x1/.ros/log/2022-03-09-15-33-33-276616-computer-1876564
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [talker-1]: process started with pid [1876618]
   [INFO] [listener-2]: process started with pid [1876620]
   [talker-1] [INFO] [1646836414.794632135] [talker]: Publishing: 'Hello World: 1'
   [listener-2] [INFO] [1646836414.795643603] [listener]: I heard: [Hello World: 1]
   [talker-1] [INFO] [1646836415.794321203] [talker]: Publishing: 'Hello World: 2'
   [listener-2] [INFO] [1646836415.795037146] [listener]: I heard: [Hello World: 2]
   [...]

Removing the snap is simple too:

::

   $ sudo snap remove ros2-talker-listener

Once done developing your snap, you can easily clean up the build environment:

.. code:: bash

   $ snapcraft clean


----------


.. _ros-2-deployment-with-snaps-core20:

core20
~~~~~~

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for an example project, `ros2-talker-listener-core20 <https://github.com/snapcraft-docs/ros2-talker-listener-core20>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: ros2-talker-listener
   version: '0.1'
   summary: ROS 2 Talker/Listener Example
   description: |
    This example launches a ROS 2 talker and listener.

   confinement: devmode
   base: core20

   parts:
    ros-demos:
      plugin: colcon
      source: https://github.com/ros2/demos.git
      source-branch: foxy
      source-subdir: demo_nodes_cpp
      stage-packages: [ros-foxy-ros2launch]

   apps:
    ros2-talker-listener:
      command: opt/ros/foxy/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
      extensions: [ros2-foxy]

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/ros2-talker-listener-core20


Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be adapted from the GitHub description or project ``README.md`` file. This data is used in the presentation of your app in the Snap Store (see e.g. `the PlotJuggler front page <https://snapcraft.io/plotjuggler>`__).

.. code:: yaml

   name: ros2-talker-listener
   version: '0.1'
   summary: ROS 2 Talker/Listener Example
   description: |
    This example launches a ROS 2 talker and listener.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

This is a declarative version of the packaged software and is not linked to the version of the snap itself. It’s also possible to write a script to calculate the version, or to take a tag or commit from a git repository.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the ``description`` key to declare a multi-line description.

For more information about top level metadata, see, :ref:`top-level-metadata <snapcraft-top-level-metadata>`.


Base
~~~~

The ``base`` keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core20

`core20 <https://snapcraft.io/core20>`__ is the current standard base for snap building and is based on\ `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`__. It is therefore the base for ROS Noetic and ROS 2 Foxy snaps.


Security model
~~~~~~~~~~~~~~

To get started, we won’t confine this application. Unconfined applications, specified with ``devmode``, can only be released to the “edge” :term:`channel`.

.. code:: yaml

   confinement: devmode

For more information about security model, see, :ref:`choosing-a-security-model <choosing-a-security-model>`.


Parts
~~~~~

Parts define how to build your app and can be anything: programs, libraries, or other assets needed to create and run your application. Their source can be local directories, remote git repositories, or tarballs. In this example, we have a single part: ros-demos.

Snapcraft relies on well known and well established ROS tools such as, in this example, ``colcon``.

**Note:** It is easy to forget the importance of complete install rules, i.e. rules for installing every component of the package necessary to run, or every component necessary to use a given library.

The packages you’re building must have install rules, or else snapcraft won’t know which components to place into the snap. Make sure you install binaries, libraries, header files, launch files, etc.

.. code:: yaml

   parts:
    ros-demos:
      plugin: colcon
      source: https://github.com/ros2/demos.git
      source-branch: foxy
      source-subdir: demo_nodes_cpp
      stage-packages: [ros-foxy-ros2launch]

For more details on colcon-specific metadata, see :ref:`the colcon plugin documentation <the-colcon-plugin-core20>`. For more information about general parts metadata, see :ref:`parts-metadata <snapcraft-parts-metadata>`.


Apps
~~~~

Apps are the commands and services exposed to end users. Each entry under apps is the command name that should be exposed to the end users.

The command specifies the path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
    ros2-talker-listener:
      command: opt/ros/foxy/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
      extensions: [ros2-foxy]

For more details about the ros2-foxy extension, see :ref:`ros2-foxy extension <the-ros2-foxy-extension>`.

In snap, an application is usually prefixed by the snap name so that the application ``my-app`` from the snap ``my-snap`` can be executed calling ``my-snap.my-app``. However, if both the snap and the app are called the same, as is the case in our ROS 2 example, the execution command collapses to avoid the tediousness of writing twice the same words. The command ``ros2-talker-listener.ros2-talker-listener`` simply becomes ``ros2-talker-listener``.


Building the snap
~~~~~~~~~~~~~~~~~

After you have created the :file:`snapcraft.yaml` file, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped ros2-talker-listener_0.1_amd64.snap

[note type=“caution” status=“Warning”] The warnings regarding missing libraries that you might see when building your snap are false positive. These libraries are build time dependencies only. [/note] The resulting snap can be immediately installed. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. Furthermore, the ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install ros2-talker-listener_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ ros2-talker-listener
   [INFO] [launch]: All log files can be found below /home/user/snap/ros2-talker-listener/x1/.ros/log/2022-03-09-15-33-33-276616-computer-1876564
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [talker-1]: process started with pid [1876618]
   [INFO] [listener-2]: process started with pid [1876620]
   [talker-1] [INFO] [1646836414.794632135] [talker]: Publishing: 'Hello World: 1'
   [listener-2] [INFO] [1646836414.795643603] [listener]: I heard: [Hello World: 1]
   [talker-1] [INFO] [1646836415.794321203] [talker]: Publishing: 'Hello World: 2'
   [listener-2] [INFO] [1646836415.795037146] [listener]: I heard: [Hello World: 2]
   [...]

Removing the snap is simple too:

.. code:: bash

   $ sudo snap remove ros2-talker-listener

Once done developing your snap, you can easily clean up the build environment:

.. code:: bash

   $ snapcraft clean


.. _ros-2-deployment-with-snaps-core22:

core22
~~~~~~

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for an example project, `ros2-talker-listener-core22 <https://github.com/snapcraft-docs/ros2-talker-listener-core22>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: ros2-talker-listener
   version: '0.1'
   summary: ROS 2 Talker/Listener Example
   description: |
    This example launches a ROS 2 talker and listener.

   confinement: devmode
   base: core22

   parts:
    ros-demos:
      plugin: colcon
      source: https://github.com/ros2/demos.git
      source-branch: humble
      source-subdir: demo_nodes_cpp
      stage-packages: [ros-humble-ros2launch]

   apps:
    ros2-talker-listener:
      command: opt/ros/humble/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
      extensions: [ros2-humble]

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/ros2-talker-listener-core22

.. _metadata-2:

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store (see e.g. `PlotJuggler front page <https://snapcraft.io/plotjuggler>`__).

.. code:: yaml

   name: ros2-talker-listener
   version: '0.1'
   summary: ROS 2 Talker/Listener Example
   description: |
    This example launches a ROS 2 talker and listener.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

This is a declarative version of the packaged software and is not linked to the version of the snap itself. It’s also possible to write a script to calculate the version, or to take a tag or commit from a git repository.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the ``description`` key to declare a multi-line description.

For more information about top level metadata, see, :ref:`top-level-metadata <snapcraft-top-level-metadata>`.

.. _base-2:

Base
~~~~

The ``base`` keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core22

`core22 <https://snapcraft.io/core22>`__ is the current standard base for snap building and is based on\ `Ubuntu 22.04 LTS <http://releases.ubuntu.com/22.04/>`__. It is therefore the base for ROS 2 Humble snaps.

.. _security-model-2:

Security model
~~~~~~~~~~~~~~

To get started, we won’t confine this application. Unconfined applications, specified with ``devmode``, can only be released to the “edge” :term:`channel`.

.. code:: yaml

   confinement: devmode

For more information about security model, see, :ref:`choosing-a-security-model <choosing-a-security-model>`.

.. _parts-2:

Parts
~~~~~

Parts define how to build your app and can be anything: programs, libraries, or other assets needed to create and run your application. Their source can be local directories, remote git repositories, or tarballs. In this example, we have a single part: ‘ros-demos’.

Snapcraft relies on well known and well established ROS tools such as, in this example, ``colcon``.

**Note:** It is easy to forget the importance of complete install rules, i.e. rules for installing every component of the package necessary to run, or every component necessary to use a given library.

The packages you’re building must have install rules, or else snapcraft won’t know which components to place into the snap. Make sure you install binaries, libraries, header files, launch files, etc.

.. code:: yaml

   parts:
    ros-demos:
      plugin: colcon
      source: https://github.com/ros2/demos.git
      source-branch: humble
      colcon-packages: [demo_nodes_cpp]
      stage-packages: [ros-humble-ros2launch]

For more details on colcon-specific metadata, see :ref:`the colcon plugin <the-colcon-plugin-core22>` documentation. For more information about general parts metadata, see, :ref:`parts-metadata <snapcraft-parts-metadata>`.

.. _apps-2:

Apps
~~~~

Apps are the commands and services exposed to end users. Each entry under apps is the command name that should be exposed to the end users.

The command specifies the path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
    ros2-talker-listener:
      command: opt/ros/humble/bin/ros2 launch demo_nodes_cpp talker_listener.launch.py
      extensions: [ros2-humble]

For more details about the ‘ros2-humble’ extension, have a look at its :ref:`documentation <the-ros-2-humble-extension>`.

In snap, an application is usually prefixed by the snap name so that the application ``my-app`` from the snap ``my-snap`` can be executed calling ``my-snap.my-app``. However, if both the snap and the app are called the same, as is the case in our ROS 2 example, the execution command collapses to avoid the tediousness of writing twice the same words. The command ``ros2-talker-listener.ros2-talker-listener`` simply becomes ``ros2-talker-listener``.

.. _building-the-snap-2:

Building the snap
~~~~~~~~~~~~~~~~~

After you have created the :file:`snapcraft.yaml`, you can build the snap by simply executing the :command:`snapcraft` command in the project directory.

.. code:: bash

   $ snapcraft
   Launching instance...
   [...]
   Created snap package ros2-talker-listener_0.1_amd64.snap

.. warning::

   Some versions of Snapcraft will not prompt you to install LXD if it is not already installed, causing Snapcraft to hang indefinitely.

   To install and initialise LXD, run the following:
   
   .. code::
   
      sudo snap install lxd && sudo lxd init --auto

The resulting snap can be immediately installed. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. Furthermore, the ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install ros2-talker-listener_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ ros2-talker-listener
   [INFO] [launch]: All log files can be found below /home/user/snap/ros2-talker-listener/x1/ros/log/2022-07-08-14-47-49-370040-host-26782
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [talker-1]: process started with pid [27671]
   [INFO] [listener-2]: process started with pid [27674]
   [talker-1] [INFO] [1657306071.096406021] [talker]: Publishing: 'Hello World: 1'
   [listener-2] [INFO] [1657306071.096756965] [listener]: I heard: [Hello World: 1]
   [talker-1] [INFO] [1657306072.096312107] [talker]: Publishing: 'Hello World: 2'
   [listener-2] [INFO] [1657306072.096541441] [listener]: I heard: [Hello World: 2]
   [...]

Removing the snap is simple too:

.. code:: bash

   $ sudo snap remove ros2-talker-listener

Once done developing your snap, you can easily clean up the build environment:

.. code:: bash

   $ snapcraft clean



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

Be sure to update the ``name:`` in your :file:`snapcraft.yaml` file to match this registered name, then run :command:`snapcraft` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   $ snapcraft upload --release=edge myrossnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first ROS snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
