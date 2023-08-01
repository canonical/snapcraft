.. 7817.md

.. _c-c-applications:

C/C++ applications
==================

Snapcraft can be used to package and distribute Python applications in a
way that enables convenient installation by users.

The process of creating a snap for a C or C++ application builds on standard
tools like :command:`autotools`, :command:`make`, making it possible to adapt
or integrate an application's existing packaging into the snap building process.


Getting started
---------------

Snaps are defined in a single :file:`snapcraft.yaml` file placed in a
:file:`snap` folder at the root of your project. This YAML file describes
the application, its dependencies and how it should be built.

The following example shows the entire :file:`snapcraft.yaml` file for an
existing project, `DOSBox`_:

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

This extensive description is broken down in the following sections.

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of
human-readable metadata, which is often already available in the project's
own packaging metadata or project :file:`README.md` file. This data is used in
the presentation of the application in the Snap Store.


.. code:: yaml

   name: dosbox
   version: "0.74-svn"
   summary: DOS emulator
   description: |
     DOSBox is a x86 emulator with Tandy/Hercules/
     CGA/EGA/VGA/SVGA graphics sound and DOS. It's
     been designed to run old DOS games under
     platforms that don't support it.

The ``name`` must be unique in the Snap Store. Valid snap names consist of
lower-case alphanumeric characters and hyphens. They cannot be all numbers and
they also cannot start or end with a hyphen.

By specifying ``git`` for the version, the current git tag or commit will be
used as the version string. Versions carry no semantic meaning in snaps.

The ``summary`` can not exceed 79 characters. You can use a chevron '>' in the
``description`` key to declare a multi-line description.

Base
~~~~

The base keyword declares which :term:`base snap` to use with the project.
A base snap is a special kind of snap that provides a run-time environment
alongside a minimal set of libraries that are common to most applications.

.. code:: yaml

   base: core18

In this example, `core18`_ is used as the base for snap building, and is based
on `Ubuntu 18.04 LTS`_. See :ref:`Base snaps <base-snaps>` for more details.

Security model
~~~~~~~~~~~~~~

Snaps are containerised to ensure more predictable application behaviour and
greater security. The general level of access a snap has to the user's system
depends on its level of confinement.

The next section of the :file:`snapcraft.yaml` file describes the level of
:term:`confinement` applied to the running application:

.. code:: yaml

   confinement: devmode

It is best to start creating a snap with a confinement level that provides
warnings for confinement issues instead of strictly applying confinement.
This is done by specifying the ``devmode`` (developer mode) confinement value.
When a snap is in devmode, runtime confinement violations will be allowed but
reported. These can be reviewed by running :command:`journalctl -xe`.

Because devmode is only intended for development, snaps must be set to strict
confinement before they can be published as "stable" in the Snap Store.
Once an application is working well in devmode, you can review confinement
violations, add appropriate interfaces, and switch to strict confinement.

The above example will also work if you change the confinement from ``devmode``
to ``strict``, as you would before a release.

Parts
~~~~~

Parts define what sources are needed to build your application. Parts can be
anything: programs, libraries, or other needed assets, but for this example,
we only need to use one part for the DOSBox source release tarball:

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

The ``plugin`` keyword is used to select a language or technology-specific
plugin that knows how to perform the build steps for the project. In this
example, the :ref:`autotools plugin <the-autotools-plugin>` is used to
automate the build, using the standard :command:`configure` and :command:`make`
tools to build the part.

Before building the part, the packages listed in the ``build-packages`` section
need to be installed in the build environment. These are the tools and libraries
that are used during the build process.

There is also a large ``stage-packages`` section:

.. code:: yaml

       stage-packages:
         - freeglut3
         - libsdl-sound1.2
         - libsdl-net1.2
   [...]
         - libxext6
         - zlib1g

These are packages containing libraries are resources that DOSBox needs to run.
They are very similar to those that would be listed as run-time dependencies on
a standard distribution package.

For more details on autotools-specific metadata, see :ref:`the-autotools-plugin`.

Apps
~~~~

Apps are the commands and services that the snap provides to users. Each key
under ``apps`` is the name of a command or service that should be made
available on users' systems.

.. code:: yaml

   apps:
     dosbox:
       command: bin/dosbox
       environment:
         "LD_LIBRARY_PATH": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pulseaudio"
         "DISABLE_WAYLAND": "1"

The ``command`` specifies the path to the binary to be run. This is resolved
relative to the root of the snap contents.

If the command name matches the name of the snap specified in the top-level
``name`` keyword (see `Metadata`_ above), the binary file will be given the
same name as the snap, as in this example.
If the names differ, the binary file name will be prefixed with the snap name
to avoid naming conflicts between installed snaps. An example of this would be
``dosbox.some-command``.

We also make two adjustments to the run-time environment for DOSBox: the first
to work around a PulseAudio issue and the second to disable Wayland. These kinds
of requirements are usually figured out through trial and error after an
initial build.

If your application is intended to run as a service you simply add the line
``daemon: simple`` after the command keyword. This will automatically keep the
service running on install, update, and reboot.

Building the snap
-----------------

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/dosbox

After you have created the :file:`snapcraft.yaml` file (which already exists
in the above repository), you can build the snap by simply executing the
:command:`snapcraft` command in the project directory:

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

.. |execname| replace:: dosbox
.. include:: common/removing-cleaning-snap.rst

.. Potentially just refer the reader to another tutorial.
.. include:: common/publishing-snap.rst

Congratulations! You've just built and published your first C/C++ snap.
For a more in-depth overview of the snap building process, see
:ref:`creating-a-snap`.

.. _`DOSBox`: https://github.com/snapcraft-docs/dosbox
