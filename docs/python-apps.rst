.. 6741.md

.. _python-apps:

Python apps
===========

Snapcraft can be used to package and distribute Python applications in a
way that enables convenient installation by users.

The process of creating a snap for a Python application builds on standard
Python packaging tools, making it possible to adapt or integrate an
application's existing packaging into the snap building process.


Getting started
---------------

Snaps are defined in a single :file:`snapcraft.yaml` file placed in a
:file:`snap` folder at the root of your project. This YAML file describes
the application, its dependencies and how it should be built.

The following example shows an entire :file:`snapcraft.yaml` file based on the
snap of an existing project, `yt-dlp`_:

.. code:: yaml

   name: yt-dlp
   summary: A fork of youtube-dl with additional features and patches
   description: |
         Download and play videos on your local system. Runs from the command
         line and with all the features and patches of youtube-dlc in addition
         to the latest youtube-dl.
   version: git
   grade: stable
   confinement: devmode
   base: core20
   architectures:
     - build-on: [arm64, armhf, amd64]

   apps:
     yt-dlp:
       command: bin/yt-dlp
       plugs: [home, network, network-bind, removable-media]

   parts:
     yt-dlp:
       plugin: python
       source: https://github.com/yt-dlp/yt-dlp.git

There will be minor differences to the latest version of the project, such
as the version definition and confinement level, but these can be changed
after the snap is working. We'll break this file down into its components in
the following sections.

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of
human-readable metadata, which is often already available in the project's
own packaging metadata or :file:`README.md` file. This data is used in the
presentation of the application in the Snap Store.

.. code:: yaml

   name: yt-dlp
   summary: A fork of youtube-dl with additional features and patches
   description: |
         Download and play videos on your local system. Runs from the command
         line and with all the features and patches of youtube-dlc in addition
         to the latest youtube-dl.
   version: git

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

   base: core20

In this example, `core20`_ is used as the base for snap building, and is based
on `Ubuntu 20.04 LTS`_. See :ref:`base-snaps` for more details.

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
we only need to use one part for the *yt-dlp* source code:

.. code:: yaml

   parts:
     yt-dlp:
       plugin: python
       source: https://github.com/yt-dlp/yt-dlp.git

The ``plugin`` keyword is used to select a language or technology-specific
plugin that knows how to perform the build steps for the project.
In this example, the :ref:`python plugin <the-python-plugin>` is used to
automate the build of this Python-based project.

The ``source`` keyword points to the source code of the Python project, which
can be a local directory or remote Git repository. In this case, it refers to
the main project repository.

Apps
~~~~

Apps are the commands and services that the snap provides to users. Each key
under ``apps`` is the name of a command or service that should be made
available on users' systems.

.. code:: yaml

   apps:
     yt-dlp:
       command: bin/yt-dlp
       plugs: [home, network, network-bind, removable-media]

The ``command`` specifies the path to the binary to be run. This is resolved
relative to the root of the snap contents.

If the command name matches the name of the snap specified in the top-level
``name`` keyword (see `Metadata`_ above), the binary file will be given the
same name as the snap, as in this example.
If the names differ, the binary file name will be prefixed with the snap name
to avoid naming conflicts between installed snaps. An example of this would be
``yt-dlp.some-command``.

The confinement of the snap, which was defined in the `Security model`_ section
above, can be changed through a set of :term:`interfaces`. In this example,
the ``plugs`` keyword is used to specify the interfaces that the snap needs
to access.

Building the snap
-----------------

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/degville/snap-yt-dlp.git

After you have created the :file:`snapcraft.yaml` file (which already exists
in the above repository), you can build the snap by simply executing the
:command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Launching a container.
   Waiting for container to be ready
   [...]
   Staging yt-dlp
   + snapcraftctl stage
   Priming yt-dlp
   + snapcraftctl prime
   Determining the version from the project repo (version: git).
   The version has been set to '0+git.9e6dc74-dirty'
   Snapping |
   Snapped yt-dlp_0+git.9e6dc74-dirty_multi.snap

The resulting snap can be installed locally. This requires the ``--dangerous``
flag because the snap is not signed by the Snap Store. The ``--devmode`` flag
acknowledges that you are installing an unconfined application:

.. code:: bash

   sudo snap install yt-dlp_0+git.*_multi.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   yt-dlp -h

.. |execname| replace:: yt-dlp
.. include:: common/removing-cleaning-snap.rst

.. Potentially just refer the reader to another tutorial.
.. include:: common/publishing-snap.rst

Congratulations! You've just built and published your first Python snap.
For a more in-depth overview of the snap building process, see
:ref:`creating-a-snap`.

.. _`yt-dlp`: https://snapcraft.io/yt-dlp
