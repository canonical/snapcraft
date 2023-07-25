.. 13753.md

.. _qt5-and-kde-frameworks-applications:

Qt5 and KDE Frameworks Applications
===================================

Why are snaps good for Qt5 and KDE Frameworks applications?
-----------------------------------------------------------

Snapcraft bundles necessary libraries required by the application, and can configure the environment for confinement of applications for end user peace of mind. Developers can ensure their application is delivered pre-packaged with libraries which will not be replaced or superseded by a distribution vendor.

Here are some snap advantages that will benefit many Qt applications:

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Ubuntu Software Center, the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the latest version of Qt5 and KDE Frameworks, along with all of your app’s dependencies, be they binaries or system libraries.
-  **You control the release schedule** You decide when a new version of your application is released without having to wait for distributions to catch up.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision.

Build a snap in 20 minutes
~~~~~~~~~~~~~~~~~~~~~~~~~~

Typically this guide will take around 20 minutes and will result in a working Qt5 application in a snap. Once complete, you’ll understand how to package Qt5 applications as snaps and deliver them to millions of Linux users. After making the snap available in the store, you’ll get access to installation metrics and tools to directly manage the delivery of updates to Linux users.

   ⓘ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for KCalc. Don’t worry, we’ll break this down.

KCalc Snap
~~~~~~~~~~

Snaps are defined in a single yaml file placed in the root of your project. The Kcalc example shows the entire snapcraft.yaml for an existing project. We’ll break this down.

.. code:: yaml

   name: kcalc
   version: '19.08.0'
   grade: stable
   adopt-info: kcalc

   confinement: strict
   base: core18

   apps:
     kcalc:
       common-id: org.kde.kcalc.desktop
       command: kcalc
       extensions:
         - kde-neon
       plugs:
         - kde-frameworks-5-plug
         - home
         - opengl
         - network
         - network-bind
         - pulseaudio

   slots:
     session-dbus-interface:
       interface: dbus
       name: org.kde.kcalc.desktop
       bus: session

   parts:
     kcalc:
       parse-info:
         - usr/share/metainfo/org.kde.kcalc.appdata.xml
       build-snaps:
         - kde-frameworks-5-core18-sdk
         - kde-frameworks-5-core18
       plugin: cmake
       build-packages:
         - libmpfr-dev
         - libgmp-dev
         - kdoctools-dev
       stage-packages:
         - libmpfr6
         - libgmp10
       source: https://download.kde.org/stable/applications/19.08.0/src/kcalc-19.08.0.tar.xz
       configflags:
         - "-DKDE_INSTALL_USE_QT_SYS_PATHS=ON"
         - "-DCMAKE_INSTALL_PREFIX=/usr"
         - "-DCMAKE_BUILD_TYPE=Release"
         - "-DENABLE_TESTING=OFF"
         - "-DBUILD_TESTING=OFF"
         - "-DKDE_SKIP_TEST_SETTINGS=ON"



Metadata
^^^^^^^^

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: kcalc
   version: '19.08.0'
   grade: stable
   adopt-info: kcalc

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

The ``version`` is a “human readable” version string. It contains no semantic meaning, its purpose is to inform users of which version of the application they are installing.

You can also fill in the ``title``, ``summary`` and ``description``. However, KCalc already has this metadata defined using an `AppStream <https://www.freedesktop.org/wiki/Distributions/AppStream/>`__ metadata file ``org.kde.kcalc.appdata.xml``, so we don’t want to duplicate this data. We use ``adopt-info`` to tell Snapcraft to get the metadata from the part itself. More on this later.

Base
^^^^

The base keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core18

`core18 <https://snapcraft.io/core18>`__ is the current standard base for snap building and is based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.

Security model
^^^^^^^^^^^^^^

To get started, we won’t :ref:`confine <snap-confinement>` this application. Unconfined applications, specified with ``devmode``, can only be released to the hidden “edge” channel where you and other developers can install them. After you get the snap working in ``devmode`` confinement, you can switch to strict mode and figure out which interfaces (plugs) the snap uses.

.. code:: yaml

   confinement: devmode

Parts
^^^^^

Parts define how to build your app. Parts can be anything: programs, libraries, or other assets needed to create and run your application. In this case we have two: the KCalc source release tarball and a number of runtime dependencies of KCalc. In other cases these can point to local directories, remote git repositories or other revision control systems.

Before building the part, the build dependencies listed as ``build-packages`` and ``build-snaps`` are installed. :ref:`The CMake plugin <the-cmake-plugin>` then uses ``cmake`` to build the part. The ``kde-frameworks-5-core18-sdk`` snap contains most build dependencies to build Qt5 and KDE applications. However, this snap also requires some tools from the ``kde-frameworks-5-core18`` runtime itself.

.. code:: yaml

   parts:
     kcalc:
       parse-info:
         - usr/share/metainfo/org.kde.kcalc.appdata.xml
       plugin: cmake
       build-snaps:
         - kde-frameworks-5-core18-sdk
         - kde-frameworks-5-core18
       build-packages:
         - libmpfr-dev
         - libgmp-dev
         - kdoctools-dev
       stage-packages:
         - libmpfr6
         - libgmp10
       source: https://download.kde.org/stable/applications/19.08.0/src/kcalc-19.08.0.tar.xz
       configflags:
         - "-DKDE_INSTALL_USE_QT_SYS_PATHS=ON"
         - "-DCMAKE_INSTALL_PREFIX=/usr"
         - "-DCMAKE_BUILD_TYPE=Release"
         - "-DENABLE_TESTING=OFF"
         - "-DBUILD_TESTING=OFF"
         - "-DKDE_SKIP_TEST_SETTINGS=ON"

``stage-packages`` are the packages required by KCalc to run, and mirror the same packages required by the binary on a standard distribution installation.

``parse-info`` points to the AppStream metadata file. Since we used ``adopt-info: kcalc`` in the metadata, the AppStream file of the ``kcalc`` part will be used to fill in the ``title``, ``summary`` and ``description`` of this snap. See :ref:`Using AppStream metadata <meta-appstream>` for more information.

Apps
^^^^

Apps are the commands and services exposed to end users. If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``KCalc.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

If you don’t want your command prefixed you can request an alias for it on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__. These are set up automatically when your snap is installed from the Snap Store.

.. code:: yaml

   apps:
     kcalc:
       common-id: org.kde.kcalc.desktop
       command: kcalc
       extensions:
         - kde-neon
       plugs:
         - home
         - opengl
         - network
         - network-bind
         - pulseaudio

You can see we use the :ref:`kde-neon extension <the-kde-neon-extension>`. This extension will make Qt5 and KDE libraries available to the snap at run time and it will configure the run time environment of the application so that all desktop functionality is correctly initialised.

The ``common-id`` field is used to link the AppStream metadata to this application. As a result, we don’t need to :ref:`manually specify the desktop entry file <desktop-files-for-menu-integration>` because it’s already defined in AppStream. See :ref:`Using AppStream metadata <meta-appstream>` for more information.

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/galgalesh/kcalc.git

After you have created the :file:`snapcraft.yaml` file, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped kcalc_19.08.0_amd64.snap

.. note::

   The extension used in this example currently only works on amd64 systems. Other architectures like arm are not supported.

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install kcalc_19.08.0_amd64.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ snap run kcalc

Removing the snap is simple too:

.. code:: bash

   $  sudo snap remove kcalc

You can clean up the build environment with the following command:

.. code:: bash

   $ snapcraft clean

By default, when you make a change to snapcraft.yaml, snapcraft only builds the parts that have changed. Cleaning a build, however, forces your snap to be rebuilt in a clean environment and will take longer.

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

   $ snapcraft register mysnap

Be sure to update the ``name:`` in your :file:`snapcraft.yaml` file to match this registered name, then run :command:`snapcraft` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   $ snapcraft upload --release=edge mysnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first Go snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
