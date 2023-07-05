.. 6741.md

.. _python-apps:

Python apps
===========

Python has several options that can assist when packaging and distributing applications. There’s `Setuptools <https://setuptools.readthedocs.io/en/latest/>`__, and the `Python Package Index <https://pypi.org/>`__ (*pip*), and `Virtualenv <https://virtualenv.pypa.io/en/latest/>`__ allows developers to isolate an application and its dependencies from the rest of the system.

But *pip* and *Virtualenv* are not user-oriented tools. Nor do they offer a solution for notifying users of available updates.

Snaps address these gaps, while building upon the work you’ve already done to teach Python how to package your app. By the end of this guide, you’ll understand how to make a snap of your Python app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

.. note::


          For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows an entire *snapcraft.yaml* file based on the snap of an existing project, `yt-dlp <https://snapcraft.io/yt-dlp>`__. There are minor differences, such as the version definition and confinement level, but these can be easily changed after the snap is working. Don’t worry, we’ll break this down.

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

The ``snapcraft.yaml`` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: yt-dlp
   summary: A fork of youtube-dl with additional features and patches
   description: |
         Download and play videos on your local system. Runs from the command
         line and with all the features and patches of youtube-dlc in addition
         to the latest youtube-dl.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

By specifying ``git`` for the version, the current git tag or commit will be used as the version string. Versions carry no semantic meaning in snaps.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the ``description`` key to declare a multi-line description.

Base
~~~~

The base keyword declares which *base snap* to use with your project. A base snap is a special kind of snap that provides a run-time environment alongside a minimal set of libraries that are common to most applications:

.. code:: yaml

   base: core20

As used above, `core20 <https://snapcraft.io/core20>`__ is the current standard base for snap building and is based on `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`__. See :ref:`Base snaps <base-snaps>` for more details.

Security model
~~~~~~~~~~~~~~

The next section describes the level of confinement applied to your app.

.. code:: yaml

   confinement: devmode

Snaps are containerised to ensure more predictable application behaviour and greater security. Unlike other container systems, the shape of this confinement can be changed through a set of interfaces. These are declarations that tell the system to give permission for a specific task, such as accessing a webcam or binding to a network port.

It’s best to start a snap with the confinement in warning mode, rather than strictly applied. This is indicated through the ``devmode`` keyword. When a snap is in devmode, runtime confinement violations will be allowed but reported. These can be reviewed by running ``journalctl -xe``.

Because devmode is only intended for development, snaps must be set to strict confinement before they can be published as “stable” in the Snap Store. Once an app is working well in devmode, you can review confinement violations, add appropriate interfaces, and switch to strict confinement.

The above example will also work if you change *devmode* to *strict*, as you would before a release.

Parts
~~~~~

Parts define what sources are needed to assemble your app. Parts can be anything: programs, libraries, or other needed assets, but for now, we’re only going to use one part: the *yt-dlp* source code.

.. code:: yaml

   parts:
     yt-dlp:
       plugin: python
       source: https://github.com/yt-dlp/yt-dlp.git

The Python plugin can be used by either Python 2 or Python 3 based parts using a setup.py script for building the project, or using a package published to PyPI, and optionally any of the following:

-  a requirements.txt file used to import Python modules
-  packages installed directly from pip

Our example project uses neither of the above but they can be added to your own project with the ``python-requirement`` and ``python-packages`` keywords, as described in our :ref:`Python plugin documentation <the-python-plugin>`.

The ``source`` keyword points to the root of your Python project and can be a local directory or remote Git repository. Note that **your Python project should be using setuptools** and you should be able to run ``python setup.py bdist_wheel`` without errors. If either of these are not true, please consult the `setuptools documentation <https://setuptools.readthedocs.io/en/latest/>`__.

If you need additional packages, the ``stage-packages`` keyword simply lists any package dependencies needed to run your app. A corresponding ``build-packages`` keyword can also be used to specify packages only needed during the build phase.

For more details on Python-specific metadata, see :ref:`The Python plugin <the-python-plugin>`.

Apps
~~~~

Apps are the commands you want to expose to users and any background services your application provides. Each key under ``apps`` is the command name that should be made available on users’ systems.

The ``command`` specifies the path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
     yt-dlp:
       command: bin/yt-dlp
       plugs: [home, network, network-bind, removable-media]

If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``yt-dlp.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

You can request an alias on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__ if your command name and snap name do not match but you don’t want your command prefixed. These aliases are set up automatically when your snap is installed from the Snap Store.

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/degville/snap-yt-dlp.git

After you’ve created the *snapcraft.yaml* (which already exists in the above repository), you can build the snap by simply executing the *snapcraft* command in the project directory:

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

By default, when you make a change to snapcraft.yaml, snapcraft only builds the parts that have changed. Cleaning a build, however, forces your snap to be rebuilt in a clean environment and will take longer.

Publishing your snap
--------------------

To share your snaps you need to publish them in the Snap Store. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads and control publishing.

You’ll need to choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the ``snapcraft`` command is authenticated using the email address attached to your Snap Store account:

.. code:: bash

   snapcraft login

Reserve a name for your snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

.. code:: bash

   snapcraft register mypythonsnap

Be sure to update the ``name:`` in your ``snapcraft.yaml`` to match this registered name, then run ``snapcraft`` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   snapcraft upload --release=edge mypythonsnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first Python snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
