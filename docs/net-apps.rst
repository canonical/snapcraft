.. 34730.md

.. _net-apps:

.NET apps
=========

The .NET toolchain makes it easy for developers to build and publish applications. However, end user discovery and update management remain a challenge. Snap helps fill this gap by letting developers distribute applications and its updates in an app store experience for end users.

Why are snaps good for .NET projects?
-------------------------------------

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the exact version of .NET required, along with all of your app’s dependencies, be they .NET DLLs or system libraries.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous version.

Build a snap in 20 minutes
--------------------------

Ready to get started? By the end of this guide, you’ll understand how to make a snap of your .NET app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

Snapcraft overview: For a brief overview of the snap creation process, including how to install snapcraft and how it’s used, see `Snapcraft overview <https://snapcraft.io/docs/snapcraft-overview>`__. For a more comprehensive breakdown of the steps involved, take a look at `Creating a snap <https://snapcraft.io/docs/creating-a-snap>`__.

.. note::


          For a brief overview of the snap creation process, including how to install snapcraft and how it’s used, see `Snapcraft overview <https://snapcraft.io/docs/snapcraft-overview>`__. For a more comprehensive breakdown of the steps involved, take a look at `Creating a snap <https://snapcraft.io/docs/creating-a-snap>`__.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. A complete reference of the available options for this file can be found in the `Snapcraft.yaml reference <https://snapcraft.io/docs/snapcraft-yaml-reference>`__. The following example shows the entire snapcraft.yaml file for an existing project, `whatime <https://github.com/snapcraft-docs/whatime>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: whatime
   version: '1.0.0'
   grade: devel
   summary: Get the current time in various cities around the world
   description: |
     Whatime is able to get you the current time in several different cities
     around the world.

   base: core22

   confinement: devmode

   parts:
     whatime:
       plugin: dotnet
       dotnet-build-configuration: Release
       dotnet-self-contained-runtime-identifier: linux-x64
       source: .
       build-packages:
         - dotnet-sdk-6.0
       stage-packages:
         - libicu70

   apps:
     whatime:
       command: Whatime

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description of the project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: whatime
   version: '1.0.0'
   grade: devel
   summary: Get the current time in various cities around the world
   description: |
     Whatime is able to get you the current time in several different cities
     around the world.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

The ``version`` is a user facing version to be displayed. You can also specify ``git`` for the version, which will use the current git tag or commit as the version string. Versions carry no semantic meaning in snaps.

The ``grade`` is an optional key that defines the quality grade of the snap. If can be either ``devel``, which means it’s a development version of the snap, so not to be published to the ``stable`` or ``candidate`` channels, or ``stable``, which can be released to all channels.

The ``summary`` can not exceed 79 characters. You can use the pipe symbol ‘\|’ in the description key to declare a multi-line description.

Base
~~~~

The base keyword declares which *base snap* to use with your project. A base snap is a special kind of snap that provides a run-time environment alongside a minimal set of libraries that are common to most applications.

.. code:: yaml

   base: core22

As used above, ``core22`` is based on `Ubuntu 22.04 LTS <https://releases.ubuntu.com/22.04/>`__. See `Base snaps <https://snapcraft.io/docs/base-snaps>`__ for more details.

Security model
~~~~~~~~~~~~~~

The next section describes the level of confinement applied to your app.

.. code:: yaml

   confinement: devmode

Snaps are containerized to ensure more predictable application behaviour and greater security. Unlike other container systems, the shape of this confinement can be changed through a set of interfaces. These are declarations that tell the system to give permission for a specific task, such as accessing a webcam or binding to a network port.

It’s best to start a snap with the confinement in warning mode, rather than strictly applied. This is indicated through the ``devmode`` keyword. When a snap is in devmode, runtime confinement violations will be allowed but reported. These can be reviewed by running ``journalctl -xe``.

Because devmode is only intended for development, snaps must be set to strict confinement before they can be published as “stable” in the Snap Store. Once an app is working well in devmode, you can review confinement violations, add appropriate interfaces, and switch to strict confinement.

The above example will also work if you change ``devmode`` to ``strict``, as you would before a release.

Parts
~~~~~

Parts define what sources are needed to assemble your app. Parts can be anything: programs, libraries, or other needed assets, but for now, we’re only going to use one part: the *whatime* source code in the local directory. In other cases, these can point to remote git repositories or tarballs.

.. code:: yaml

   parts:
     whatime:
       plugin: dotnet
       dotnet-build-configuration: Release
       dotnet-self-contained-runtime-identifier: linux-x64
       source: .
       build-packages:
         - dotnet-sdk-6.0
       stage-packages:
         - libicu70

The ``dotnet`` plugin can be used in .NET projects to install dependencies via the `NuGet <https://www.nuget.org/>`__ package manager, compile the application, and publish it. Optionally, the following parameters can also be defined:

-  ``dotnet-build-configuration``: The .NET build configuration to use. Defaults to Release.
-  ``dotnet-self-contained-runtime-identifier``: Optional parameter to specify the runtime identifier to use when building a self-contained application. Setting this parameter will automatically trigger a self-contained build (with the ``--self-contained`` flag on the ``dotnet publish`` command).

For single file publishing, the output of the deployment should contain the executable file along with all the necessary DLLs and dependencies necessary to run it. If you want to deploy your application as a single-file executable, you should set the following parameter inside the ``.csproj`` file of your project:

.. code:: xml

   <PropertyGroup>
   …
   <PublishSingleFile>true</PublishSingleFile>
   </PropertyGroup>

By doing this, the parameter ``dotnet-self-contained-runtime-identifier`` becomes required, since a runtime identifier is required to build a single-file .NET executable.

The ``source`` should point to the root of your .NET project. In our case, it points to the local directory as it’s where the source tree is located. It can also point to a URL if the source code is available as a compressed archive or a revision control repository.

You should also use ``build-packages`` to list any required package during build time. We are listing the dotnet-sdk-6.0 package as it installs the .NET 6 SDK, which is required by the plugin and does not get installed automatically.

The ``stage-packages`` directive is used to list any required packages during run time. In our case, the libicu70 is a required system package to run the .NET application.

Apps
~~~~

Apps are the commands you want to expose to users and any background services your application provides. Each key under ``apps`` is the command name that should be made available on users’ systems.

The ``command`` specifies the full path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
   whatime:
   command: Whatime

If your command name matches the snap ``name``, users will be able to run the command directly. If the names differ, then apps are prefixed with the snap name (``whatime.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

You can request an alias on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__ if your command name and snap name do not match but you don’t want your command prefixed. These aliases are set up automatically when your snap is installed from the Snap Store.

Building the snap
-----------------

You can download the example repository with the following command

::

   $ git clone https://github.com/mateusrodrigues/whatime.git

After you’ve created the :file:`snapcraft.yaml` file, which already exists in the above repository, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

::

   $ snapcraft
   Launching instance…
   Executed: pull whatime
   Executed: overlay whatime
   Executed: build whatime
   Executed: stage whatime
   Executed: prime whatime
   Executed parts lifecycle
   Generated snap metadata
   Created snap package whatime_1.0.0_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

::

   $ sudo snap install whatime_1.0.0_amd64.snap --devmode --dangerous

You can then try it out:

::

   $ whatime --help

Removing the snap is simple too:

::

   $ sudo snap remove whatime

You can also clean up the build environment, although this will slow down the next initial build:

::

   $ snapcraft clean

By default, when you make a change to snapcraft.yaml, snapcraft only builds the parts that have changed. Cleaning a build, however, forces your snap to be rebuilt in a clean environment and will take longer.

Publishing your snap
--------------------

To share your snaps, you need to publish them in the Snap Store. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads, and control publishing.

You’ll need to choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the :command:`snapcraft` command is authenticated using the email address attached to your Snap Store account.

::

   $ snapcraft login

Reserve a name for your snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

::

   $ snapcraft register mydotnetapp

Be sure to update the ``name:`` in your :file:`snapcraft.yaml` file to match this registered name, then run :command:`snapcraft` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

::

   $ snapcraft upload --release=edge mydotnetapp_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io/>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first .NET snap. For a more in-depth overview of the snap building process, see `Creating a snap <https://snapcraft.io/docs/creating-a-snap>`__.
