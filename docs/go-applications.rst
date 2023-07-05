.. 7818.md

.. _go-applications:

Go applications
===============

Go makes it easy to create a zip of your app that runs across Linux, without dependencies. However, end user discovery and update management remain a challenge. Snaps fill this gap, letting you distribute a Go app in an app store experience for end users.

Why are snaps good for Go projects?
-----------------------------------

Installing Go applications often consists of downloading pre-built binaries (or running ``go get``). When distributed this way, getting updates is an exercise left to the reader. With snapcraft it’s just one command to produce a bundle that works anywhere and can be automatically updated.

Here are some snap advantages that will benefit many Go projects:

-  **Snaps are easy to discover and install** Install with ``snap install mygoapp``, regardless of distribution.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Extremely simple daemon creation** A single snap can provide multiple applications and services.
-  **Deliver assets with your snap** Include images and static web content inside the package.

Build a snap in 20 minutes
~~~~~~~~~~~~~~~~~~~~~~~~~~

Ready to get started? By the end of this guide, you’ll understand how to make a snap of your Go app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

   ℹ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire *snapcraft.yaml* file for an existing project, `Woke <https://github.com/degville/woke-snap>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: woke
   summary: Detect non-inclusive language in your source code
   description: |
         Creating an inclusive work environment is imperative to a healthy,
         supportive, and productive culture, and an environment where everyone
         feels welcome and included. woke is a text file analysis tool that finds
         places within your source code that contain non-inclusive language and
         suggests replacing them with more inclusive alternatives.
   version: git
   grade: stable
   base: core20

   confinement: devmode

   apps:
     woke:
       command: bin/woke
       plugs:
         - home
   parts:
     woke:
       plugin: go
       source-type: git
       source: https://github.com/get-woke/woke

Metadata
~~~~~~~~

The ``snapcraft.yaml`` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: woke
   summary: Detect non-inclusive language in your source code
   description: |
         Creating an inclusive work environment is imperative to a healthy,
         supportive, and productive culture, and an environment where everyone
         feels welcome and included. woke is a text file analysis tool that finds
         places within your source code that contain non-inclusive language and
         suggests replacing them with more inclusive alternatives.
   version: git

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

By specifying ``git`` for the version, the current git tag or commit will be used as the version string. Versions carry no semantic meaning in snaps.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the ``description`` key to declare a multi-line description.

Base
~~~~

The base keyword declares which *base snap* to use with your project. A base snap is a special kind of snap that provides a run-time environment alongside a minimal set of libraries that are common to most applications:

.. code:: yaml

   base: core20

As used above, `core20 <https://snapcraft.io/core20>`__ is the current standard base for snap building and is based on `Ubuntu 20.04 LTS <http://releases.ubuntu.com/20.04/>`__.

See :ref:`Base snaps <base-snaps>` for more details.

Security model
~~~~~~~~~~~~~~

The next section describes the level of confinement applied to your app.

.. code:: yaml

   confinement: devmode

Snaps are containerised to ensure more predictable application behaviour and greater security. Unlike other container systems, the shape of this confinement can be changed through a set of interfaces. These are declarations that tell the system to give permission for a specific task, such as accessing a webcam or binding to a network port.

It’s best to start a snap with the confinement in warning mode, rather than strictly applied. This is indicated through the ``devmode`` keyword. When a snap is in devmode, runtime confinement violations will be allowed but reported. These can be reviewed by running ``journalctl -xe``.

Because devmode is only intended for development, snaps must be set to strict confinement before they can be published as “stable” in the Snap Store. Once an app is working well in devmode, you can review confinement violations, add appropriate interfaces, and switch to strict confinement.

Parts
~~~~~

Parts define what sources are needed to assemble your app. Parts can be anything: programs, libraries, or other needed assets. In this case we have one: the *woke* source code. In other cases, these can point to local directories, remote git repositories, or tarballs.

The Go plugin will build using the version of Go on the system running snapcraft.

.. code:: yaml

   parts:
     woke:
       plugin: go
       source-type: git
       source: https://github.com/get-woke/woke

For more details on Go-specific metadata, see :ref:`The go plugin <the-go-plugin>`.

Apps
~~~~

Apps are the commands and services exposed to end users. If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``woke.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

If you don’t want your command prefixed you can request an alias for it on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__. These are set up automatically when your snap is installed from the Snap Store.

.. code:: yaml

   apps:
     woke:
       command: bin/woke
       plugs:
         - home

If your application is intended to run as a service you simply add the line ``daemon: simple`` after the command keyword. This will automatically keep the service running on install, update, and reboot.

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/degville/woke-snap

After you’ve created the *snapcraft.yaml*, you can build the snap by simply executing the *snapcraft* command in the project directory:

.. code:: bash

   $ snapcraft
   Launching a container.
   Waiting for container to be ready
   [...]
   Pulling woke
   + snapcraftctl pull
   Cloning into '/root/parts/woke/src'...
   remote: Enumerating objects: 2723, done.
   remote: Counting objects: 100% (939/939), done.
   remote: Compressing objects: 100% (401/401), done.
   remote: Total 2723 (delta 697), reused 635 (delta 522), pack-reused 1784
   Receiving objects: 100% (2723/2723), 22.33 MiB | 2.88 MiB/s, done.
   Resolving deltas: 100% (1574/1574), done.
   Building woke
   + snapcraftctl build
   + go mod download
   + go install -p 8 -ldflags -linkmode=external ./...
   Staging woke
   + snapcraftctl stage
   Priming woke
   + snapcraftctl prime
   Determining the version from the project repo (version: git).
   The version has been set to '0+git.f23bb0a-dirty'
   Snapping |
   Snapped woke_0+git.f23bb0a-dirty_multi.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install woke_*.snap --devmode --dangerous

You can then try it out:

::

   $ woke -h

Removing the snap is simple too:

::

   $ sudo snap remove woke

Publishing your snap
--------------------

To share your snaps you need to publish them in the Snap Store. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads and control publishing.

You’ll need to choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the ``snapcraft`` command is authenticated using the email address attached to your Snap Store account:

::

   $ snapcraft login

Reserve a name for your snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

::

   $ snapcraft register mygosnap

Be sure to update the ``name:`` in your ``snapcraft.yaml`` to match this registered name, then run ``snapcraft`` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

::

   $ snapcraft upload --release=edge mygosnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first Go snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
