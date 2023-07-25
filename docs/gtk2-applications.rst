.. 13508.md

.. _gtk2-applications:

GTK2 Applications
=================

Why are snaps good for GTK+ 2 applications?
-------------------------------------------

Snapcraft bundles necessary libraries required by the application, and can configure the environment for confinement of applications for end user peace of mind. Developers can ensure their application is delivered pre-packaged with libraries which will not be replaced or superseded by a distribution vendor.

Here are some snap advantages that will benefit many GTK+ 2 applications:

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Ubuntu Software Center, the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the exact version of whatever is required, along with all of your app’s dependencies, be they binaries or system libraries.
-  **You control the release schedule** You decide when a new version of your application is released without having to wait for distributions to catch up.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision.

Build a snap in 20 minutes
--------------------------

Typically this guide will take around 20 minutes and will result in a working GTK application in a snap. Once complete, you’ll understand how to package GTK+ 2 applications as snaps and deliver them to millions of Linux users. After making the snap available in the store, you’ll get access to installation metrics and tools to directly manage the delivery of updates to Linux users.

   ⓘ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows `the entire snapcraft.yaml file for the Arduino IDE <https://github.com/galgalesh/arduino-example/blob/master/snap/snapcraft.yaml>`__. Don’t worry, we’ll break this down.

Arduino IDE Snap
~~~~~~~~~~~~~~~~

Snaps are defined in a single yaml file placed in the root of your project. The Arduino IDE example shows the entire snapcraft.yaml for an existing project. We’ll break this down.

.. code:: yaml

   name: arduino
   title: Arduino IDE
   version: 1.8.12
   summary: Write code and upload it to your Arduino-compatible board.
   description: |
     Arduino is an open-source physical computing platform based on a simple I/O board and a development environment that implements the Processing/Wiring language. Arduino can be used to develop stand-alone interactive objects or can be connected to software on your computer (e.g. Flash, Processing and MaxMSP). The boards can be assembled by hand or purchased preassembled at https://arduino.cc

     **Setup**

     In order to upload code an arduino board over USB, you need to add your user to the `dailout` group and connect the snap to the `raw-usb` socket. Open a terminal window, run the following commands an *reboot* your computer.

     > `sudo usermod -a -G dialout $USER`

     > `sudo snap connect arduino:raw-usb`

     Now restart your computer and you're good to go!
   license: GPL-2.0
   icon: snap/gui/arduino.png
   grade: stable

   base: core18
   confinement: strict

   architectures:
     - build-on: amd64
     - build-on: i386
     - build-on: armhf
     - build-on: arm64

   apps:
     arduino:
       command: desktop-launch $SNAP/arduino-snap
       environment:
         # Fallback to XWayland if running in a Wayland session.
         DISABLE_WAYLAND: 1
       plugs:
         - x11
         - unity7
         - home
         - network
         - serial-port
         - raw-usb
     builder:
       command: arduino-builder

   parts:
     upstream:
       source:
         - on amd64: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linux64.tar.xz
         - on i386: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linux32.tar.xz
         - on armhf: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linuxarm.tar.xz
         - on arm64: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linuxaarch64.tar.xz
       source-type: tar
       plugin: dump
       prime:
         - -install.sh
         - -uninstall.sh
         - -java/lib/ext/jfxrt.jar
         - -java/lib/jfxswt.jar
         - -java/lib/*/libjfxwebkit.so
         - -java/lib/*/libjfxmedia.so
       stage-packages:
         - libxtst6
         - libcanberra-gtk-module
       after:
         - desktop-gtk2
     launcher:
       plugin: dump
       source: scripts
       prime:
         - arduino-snap

     # This part installs the `desktop-launch` script which initialises desktop
     # features such as fonts, themes and the XDG environment. It also installs
     # the GTK2 runtime libraries.
     #
     # It is copied straight from the snapcraft desktop helpers project. Please
     # periodically check the source for updates and copy the changes.
     #    https://github.com/ubuntu/snapcraft-desktop-helpers/blob/master/snapcraft.yaml
     #
     desktop-gtk2:
       source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
       source-subdir: gtk
       plugin: make
       make-parameters: ["FLAVOR=gtk2"]
       build-packages:
         - build-essential
         - libgtk2.0-dev
       stage-packages:
         - libxkbcommon0  # XKB_CONFIG_ROOT
         - ttf-ubuntu-font-family
         - dmz-cursor-theme
         - light-themes
         - adwaita-icon-theme
         - gnome-themes-standard
         - shared-mime-info
         - libgtk2.0-0
         - libgdk-pixbuf2.0-0
         - libglib2.0-bin
         - libgtk2.0-bin
         - unity-gtk2-module
         - locales-all
         - libappindicator1
         - xdg-user-dirs
         - ibus-gtk
         - libibus-1.0-5

   plugs:
     gtk-2-engines:
       interface: content
       target: $SNAP/lib/gtk-2.0
       default-provider: gtk2-common-themes
     gtk-2-themes:
       interface: content
       target: $SNAP/data-dir/themes
       default-provider: gtk-common-themes
     icon-themes:
       interface: content
       target: $SNAP/data-dir/icons
       default-provider: gtk-common-themes
     sound-themes:
       interface: content
       target: $SNAP/data-dir/sounds
       default-provider: gtk-common-themes



Metadata
--------

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: arduino
   title: Arduino IDE
   version: 1.8.12
   summary: Write code and upload it to your Arduino-compatible board.
   description: |
     Arduino is an open-source physical computing platform based on a simple I/O board and a development environment that implements the Processing/Wiring language. Arduino can be used to develop stand-alone interactive objects or can be connected to software on your computer (e.g. Flash, Processing and MaxMSP). The boards can be assembled by hand or purchased preassembled at https://arduino.cc

     **Setup**

     In order to upload code an arduino board over USB, you need to add your user to the `dailout` group and connect the snap to the `raw-usb` socket. Open a terminal window, run the following commands an *reboot* your computer.

     > `sudo usermod -a -G dialout $USER`

     > `sudo snap connect arduino:raw-usb`

     Now restart your computer and you're good to go!
   license: GPL-2.0
   icon: snap/gui/arduino.png

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the description key to declare a multi-line description.

The ``version`` parameter is an arbitrary string containing the *user-facing* version number.

Base
----

The ``base`` keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core18

`core18 <https://snapcraft.io/core18>`__ is the current standard base for snap building and is based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.

Security model
--------------

To get started, we won’t :ref:`confine <snap-confinement>` this application. Unconfined applications, specified with ``devmode``, can only be released to the hidden “edge” channel where you and other developers can install them. After you get the snap working in ``devmode`` confinement, you can switch to strict mode and figure out which interfaces (plugs) the snap uses.

.. code:: yaml

   confinement: devmode

Apps
----

Apps are the commands and services exposed to end users. If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``arduino.builder``, for example). This is to avoid conflicting with apps defined by other installed snaps.

If you don’t want your command prefixed you can request an alias for it on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__. These are set up automatically when your snap is installed from the Snap Store.

We declare two applications. The ``arduino`` command starts the IDE GUI and the ``builder`` command starts a CLI application.

.. code:: yaml

   apps:
     arduino:
       command: desktop-launch $SNAP/arduino-snap
       environment:
         # Fallback to XWayland if running in a Wayland session.
         DISABLE_WAYLAND: 1
       plugs:
         - x11
         - unity7
         - home
         - network
         - serial-port
         - raw-usb
     builder:
       command: arduino-builder

The ``desktop-launch`` script initializes the environment for GTK+ 2 applications. This script is provided by the ``desktop-gtk2`` part defined below.

Since GTK+ 2 does not support wayland, it’s best to set ``DISABLE_WAYLAND: 1`` to fallback to XWayland when running in a wayland session.

The GUI application uses a number of ``plugs`` to create a window and use desktop features. It also uses ``raw-usb`` and ``serial-port`` for access to Arduino boards.

Parts
-----

Parts define how to build your app. Parts can be anything: programs, libraries, or other assets needed to create and run your application. In this case we have three: the Arduino release tarball, a launcher script and the ``desktop-gtk2`` helper part.

The ``desktop-gtk2`` part is copied from the `Snapcraft Desktop Helpers <https://github.com/ubuntu/snapcraft-desktop-helpers>`__ repository and contains the GTK+ 2 runtime libraries and the ``desktop-launch`` script to configure the environment for GTK+ 2.

.. code:: yaml

   parts:
     upstream:
       source:
         - on amd64: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linux64.tar.xz
         - on i386: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linux32.tar.xz
         - on armhf: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linuxarm.tar.xz
         - on arm64: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linuxaarch64.tar.xz
       source-type: tar
       plugin: dump
       prime:
         - -install.sh
         - -uninstall.sh
         - -java/lib/ext/jfxrt.jar
         - -java/lib/jfxswt.jar
         - -java/lib/*/libjfxwebkit.so
         - -java/lib/*/libjfxmedia.so
       stage-packages:
         - libxtst6
         - libcanberra-gtk-module
       after:
         - desktop-gtk2
     launcher:
       plugin: dump
       source: scripts
       prime:
         - arduino-snap

     # This part installs the `desktop-launch` script which initialises desktop
     # features such as fonts, themes and the XDG environment. It also installs
     # the GTK2 runtime libraries.
     #
     # It is copied straight from the snapcraft desktop helpers project. Please
     # periodically check the source for updates and copy the changes.
     #    https://github.com/ubuntu/snapcraft-desktop-helpers/blob/master/snapcraft.yaml
     #
     desktop-gtk2:
       source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
       source-subdir: gtk
       plugin: make
       make-parameters: ["FLAVOR=gtk2"]
       build-packages:
         - build-essential
         - libgtk2.0-dev
       stage-packages:
         - libxkbcommon0  # XKB_CONFIG_ROOT
         - ttf-ubuntu-font-family
         - dmz-cursor-theme
         - light-themes
         - adwaita-icon-theme
         - gnome-themes-standard
         - shared-mime-info
         - libgtk2.0-0
         - libgdk-pixbuf2.0-0
         - libglib2.0-bin
         - libgtk2.0-bin
         - unity-gtk2-module
         - locales-all
         - libappindicator1
         - xdg-user-dirs
         - ibus-gtk
         - libibus-1.0-5

Before building the part, the dependencies listed as ``build-packages`` are installed. ``stage-packages`` are the packages required by the Arduino IDE to run, and mirror the same packages required by the binary on a standard distribution installation.

Plugs
-----

This snap connects to a number of desktop-specific content snaps in order to access common themes. These are provided by content snaps so that snaps don’t need to include every theme in the snap itself.

.. code:: yaml

   plugs:
     gtk-2-engines:
       interface: content
       target: $SNAP/lib/gtk-2.0
       default-provider: gtk2-common-themes
     gtk-2-themes:
       interface: content
       target: $SNAP/data-dir/themes
       default-provider: gtk-common-themes
     icon-themes:
       interface: content
       target: $SNAP/data-dir/icons
       default-provider: gtk-common-themes
     sound-themes:
       interface: content
       target: $SNAP/data-dir/sounds
       default-provider: gtk-common-themes

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/galgalesh/arduino-example.git

After you’ve created the :file:`snapcraft.yaml` file, you can build the snap by simply executing the *snapcraft* command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped arduino_1.8.12_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install arduino_1.8.12_amd64.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ arduino

Removing the snap is simple too:

.. code:: bash

   $ sudo snap remove arduino

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

Congratulations! You’ve just built and published your first GTK+ 2 snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
