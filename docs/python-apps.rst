.. 6741.md

.. _python-apps:

Python apps
===========

Snapcraft can be used to package and distribute Python applications in a
way that enables convenient installation by users.

The process of creating a snap for a Python application builds on standard
Python packaging tools, making it possible to adapt or integrate an
application's existing packaging into the snap building process.

Snaps are defined in a single :file:`snapcraft.yaml` file placed in a
:file:`snap` folder at the root of your project. This YAML file describes
the application, its dependencies and how it should be built.


Getting started
---------------

The following example shows an entire :file:`snapcraft.yaml` file based on the
snap of an existing project, `yt-dlp`_. There will be minor differences to the
latest version of the project, such as the version definition and confinement
level, but these can be changed after the snap is working. Don't worry, we'll
break this down in the following sections.

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

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of
human-readable metadata, which is often already available in the project's
own packaging metadata or project README.md. This data is used in the
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

The base keyword declares which :term:`base snap` to use with your project.
A base snap is a special kind of snap that provides a run-time environment
alongside a minimal set of libraries that are common to most applications.

.. code:: yaml

   base: core20

In this example, `core20`_ is the current standard base for snap building and
is based on `Ubuntu 20.04 LTS`_. See :ref:`Base snaps <base-snaps>` for more
details.

Security model
~~~~~~~~~~~~~~

Snaps are containerised to ensure more predictable application behaviour and
greater security. Unlike other container systems, the shape of this confinement
can be changed through a set of interfaces. These are declarations that tell
the system to give permission for specific tasks, such as accessing a webcam
or binding to a network port.

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
we only need to use one part: the *yt-dlp* source code.

.. code:: yaml

   parts:
     yt-dlp:
       plugin: python
       source: https://github.com/yt-dlp/yt-dlp.git

The Python plugin can be used to build Python-based parts that are normally
built using setuptools. It can also be used to build packages published on the
Python Package Index (PyPI).

Additionally, the plugin can install dependencies required for the build,
either from a :file:`requirements.txt` file, or using :command:`pip` directly.

This example project uses neither of these features, but they can be added to your own project with the ``python-requirement`` and ``python-packages`` keywords, described in the :ref:`Python plugin documentation <the-python-plugin>`.

The ``source`` keyword points to the source code of the Python project, which
can be a local directory or remote Git repository. Note that **your Python project should be using setuptools** and you should be able to run ``python setup.py bdist_wheel`` without errors. If either of these are not true, please consult the `setuptools documentation <https://setuptools.readthedocs.io/en/latest/>`__.

If you need additional packages, the ``stage-packages`` keyword simply lists any package dependencies needed to run your app. A corresponding ``build-packages`` keyword can also be used to specify packages only needed during the build phase.

For more details on Python-specific metadata, see :ref:`The Python plugin <the-python-plugin>`.

Apps
~~~~

Apps are the commands you want to expose to users and any background services your application provides. Each key under ``apps`` is the command name that should be made available on users' systems.

The ``command`` specifies the path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
     yt-dlp:
       command: bin/yt-dlp
       plugs: [home, network, network-bind, removable-media]

If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``yt-dlp.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

You can request an alias on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__ if your command name and snap name do not match but you don't want your command prefixed. These aliases are set up automatically when your snap is installed from the Snap Store.

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/degville/snap-yt-dlp.git

After you've created the :file:`snapcraft.yaml` (which already exists in the above repository), you can build the snap by simply executing the *snapcraft* command in the project directory:

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

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   sudo snap install yt-dlp_0+git.*_multi.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   yt-dlp -h

Removing the snap is simple too:

.. code:: bash

   sudo snap remove yt-dlp

You can also clean up the build environment, although this will slow down the next initial build:

.. code:: bash

   snapcraft clean

By default, when you make a change to :file:`snapcraft.yaml`, snapcraft only builds the parts that have changed. Cleaning a build, however, forces your snap to be rebuilt in a clean environment and will take longer.

Publishing your snap
--------------------

To share your snaps you need to publish them in the Snap Store. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads and control publishing.

You'll need to choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the :command:`snapcraft` command is authenticated using the email address attached to your Snap Store account:

.. code:: bash

   snapcraft login

Reserve a name for your snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

.. code:: bash

   snapcraft register mypythonsnap

Be sure to update the ``name:`` in your :file:`snapcraft.yaml` file to match this registered name, then run :command:`snapcraft` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   snapcraft upload --release=edge mypythonsnap_*.snap

If you're happy with the result, you can commit the :file:`snapcraft.yaml` to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You've just built and published your first Python snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.

.. _`yt-dlp`: https://snapcraft.io/yt-dlp
