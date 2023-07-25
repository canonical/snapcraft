.. 7817.md

.. _c-c-applications:

C/C++ applications
==================

Snapcraft builds on top of tools like autotools, make, and cmake to create snaps for people to install on Linux.

Why are snaps good for C/C++ projects?
--------------------------------------

Snapcraft bundles necessary libraries required by the application, and can configure the environment for confinement of applications for end user peace of mind. Developers can ensure their application is delivered pre-packaged with libraries which will not be replaced or superseded by a distribution vendor.

Here are some snap advantages that will benefit many C/C++ projects:

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the exact version of whatever is required, along with all of your app’s dependencies, be they binaries or system libraries.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision.

Build a snap in 20 minutes
~~~~~~~~~~~~~~~~~~~~~~~~~~

Typically this guide will take around 20 minutes and will result in a working C++ application in a snap. Once complete, you’ll understand how to package C/C++ applications as snaps and deliver them to millions of Linux users. After making the snap available in the store, you’ll get access to installation metrics and tools to directly manage the delivery of updates to Linux users.

   ℹ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined by a single YAML file placed either in the root folder of your project (old style), or in a top-level ``snap/`` directory (new style). The following example shows the entire :file:`snapcraft.yaml` file for an existing project, `DOSBox <https://github.com/snapcraft-docs/dosbox>`__. Don’t worry, we’ll break this down.

DOSBox Snap
~~~~~~~~~~~

Snaps are defined in a single yaml file placed in the root of your project. The DOSBox example shows the entire snapcraft.yaml for an existing project. We’ll break this down.

.. code:: yaml

   name: dosbox
   version: "0.74-svn"
   summary: DOS emulator
   description: |
     DOSBox is a x86 emulator with Tandy/Hercules/
     CGA/EGA/VGA/SVGA graphics sound and DOS. It's
     been designed to run old DOS games under
     platforms that don't support it.

   base: core18
   confinement: devmode

   parts:
     dosbox:
       plugin: autotools
       source-type: tar
       source: http://source.dosbox.com/dosboxsvn.tgz
       build-packages:
         - g++
         - make
         - libsdl1.2-dev
         - libpng-dev
         - libsdl-net1.2-dev
         - libsdl-sound1.2-dev
         - libasound2-dev
       stage-packages:
         - freeglut3
         - libsdl-sound1.2
         - libsdl-net1.2
         - libxcursor1
         - libxi6
         - libxinerama1
         - libxrandr2
         - libxrender1
         - libopenal1
         - libsndio6.1
         - libspeex1
         - libvorbisfile3
         - libwayland-client0
         - libwayland-cursor0
         - libwayland-egl1-mesa
         - libxkbcommon0
         - libglu1-mesa
         - libasound2
         - libasyncns0
         - libbsd0
         - libcaca0
         - libdbus-1-3
         - libflac8
         - libgcc1
         - libgcrypt20
         - libgl1
         - libglvnd0
         - libglx0
         - libgpg-error0
         - liblz4-1
         - liblzma5
         - libncursesw5
         - libogg0
         - libpng16-16
         - libpulse0
         - libsdl1.2debian
         - libslang2
         - libsndfile1
         - libstdc++6
         - libsystemd0
         - libtinfo5
         - libvorbis0a
         - libvorbisenc2
         - libwrap0
         - libx11-6
         - libxau6
         - libxcb1
         - libxdmcp6
         - libxext6
         - zlib1g

   apps:
     dosbox:
       command: dosbox
       environment:
         "LD_LIBRARY_PATH": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pulseaudio"
         "DISABLE_WAYLAND": "1"



Metadata
^^^^^^^^

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: dosbox
   version: "0.74-svn"
   summary: DOS emulator
   description: |
     DOSBox is a x86 emulator with Tandy/Hercules/
     CGA/EGA/VGA/SVGA graphics sound and DOS. It's
     been designed to run old DOS games under
     platforms that don't support it.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

By specifying ``git`` for the version, the current git tag or commit will be used as the version string. Versions carry no semantic meaning in snaps.

The ``summary`` can not exceed 79 characters. You can use the pipe symbol ‘\|’ in the ``description`` key to declare a multi-line description.

Base
^^^^

The base keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core18

`core18 <https://snapcraft.io/core18>`__ is one of the supported standard bases for snap building and is based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.

Security model
^^^^^^^^^^^^^^

To get started, we won’t :ref:`confine <snap-confinement>` this application. Unconfined applications, specified with ``devmode``, can only be released to the hidden “edge” channel where you and other developers can install them.

.. code:: yaml

   confinement: devmode

Parts
^^^^^

Parts define how to build your app. Parts can be anything: programs, libraries, or other assets needed to create and run your application. In this case we have one: the DOSBox source release tarball. In other cases these can point to local directories, remote git repositories or other revision control systems.

Before building the part, the dependencies listed as ``build-packages`` are installed. The autotools plugin uses the standard tools, ``configure`` and ``make`` to build the part.

.. code:: yaml

   parts:
     dosbox:
       plugin: autotools
       source-type: tar
       source: http://source.dosbox.com/dosboxsvn.tgz
       build-packages:
         - g++
         - make
         - libsdl1.2-dev
         - libpng-dev
         - libsdl-net1.2-dev
         - libsdl-sound1.2-dev
         - libasound2-dev

There’s also a large ``stage-packages`` section.

.. code:: yaml

       stage-packages:
         - freeglut3
         - libsdl-sound1.2
         - libsdl-net1.2
   [...]
         - libxext6
         - zlib1g

These are the packages required by DOSBox to run, and mirror the same packages required by the binary on a standard distribution installation.

For more details on autotools-specific metadata, see :ref:`The autotools plugin <the-autotools-plugin>`.

Apps
^^^^

Apps are the commands and services exposed to end users. If your command name matches the snap name, users will be able run the command directly; that is, if the installed snap application has the name ``dosbox.dosbox``, you can run it as merely ``dosbox``. If the names differ, then apps must be prefixed with the snap name (``dosbox.somethingelse``, for example). This is to avoid conflicting with apps defined by other installed snaps.

If you don’t want your command prefixed, you can request an alias for it on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__. These are set up automatically when your snap is installed from the Snap Store.

.. code:: yaml

   apps:
     dosbox:
       command: bin/dosbox
       environment:
         "LD_LIBRARY_PATH": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pulseaudio"
         "DISABLE_WAYLAND": "1"

You can see we also make two adjustments to the run time environment for DOSBox, the first to work around a PulseAudio issue and the second to disable Wayland. These kinds of requirements are usually figured out through trial and error after an initial build.

If your application is intended to run as a service you simply add the line ``daemon: simple`` after the command keyword. This will automatically keep the service running on install, update, and reboot.

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/dosbox

After you’ve created the :file:`snapcraft.yaml` file, you can build the snap by simply executing the *snapcraft* command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped dosbox_0.74-svn_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $  sudo snap install dosbox_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ dosbox

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/optimized/2X/5/5e4a99e71254372ac1c2da5b758fe488029b9d0a_2_690x495.png
   :alt: Screenshot_20190613_152721|690x495


Removing the snap is simple too:

.. code:: bash

   $  sudo snap remove dosbox

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

Congratulations! You’ve just built and published your first C/C++ snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
