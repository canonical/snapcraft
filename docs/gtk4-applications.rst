.. 32266.md

.. _gtk4-applications:

GTK4 applications
=================

Why are snaps good for GTK4 applications?
-----------------------------------------

Snapcraft bundles necessary libraries required by the application, and can configure the environment for confinement of applications for end user peace of mind. Developers can ensure their application is delivered pre-packaged with libraries which will not be replaced or superseded by a distribution vendor.

Here are some snap advantages that will benefit many GTK4 applications:

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Ubuntu Software Center, the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the exact version of whatever is required, along with all of your app’s dependencies, be they binaries or system libraries.
-  **You control the release schedule** You decide when a new version of your application is released without having to wait for distributions to catch up.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision.

Build a snap in 20 minutes
~~~~~~~~~~~~~~~~~~~~~~~~~~

Typically this guide will take around 20 minutes and will result in a working GTK4 application in a snap. Once complete, you’ll understand how to package cutting edge GTK4 and GNOME 4 applications as snaps and deliver them to millions of Linux users. After making the snap available in the store, you’ll get access to installation metrics and tools to directly manage the delivery of updates to Linux users.

   ⓘ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows `the entire snapcraft.yaml file for GNOME Text Editor <https://github.com/ubuntu/gnome-text-editor/blob/stable/snap/snapcraft.yaml>`__. Don’t worry, we’ll break this down.

Gnome-Text-Editor Snap
~~~~~~~~~~~~~~~~~~~~~~

Snaps are defined in a single yaml file placed in the root of your project. The Gnome-Text-Editor example shows the entire snapcraft.yaml for an existing project. We’ll break this down.

.. code:: yaml

   name: gnome-text-editor
   grade: stable
   adopt-info: gnome-text-editor
   license: GPL-3.0+

   base: core22
   confinement: strict

   apps:
     gnome-text-editor:
       extensions: [gnome]
       command: usr/bin/gnome-text-editor
       desktop: usr/share/applications/org.gnome.TextEditor.desktop
       common-id: org.gnome.TextEditor.desktop
       plugs:
         - gsettings
         - cups

   parts:
     gnome-text-editor:
       source: https://gitlab.gnome.org/GNOME/gnome-text-editor
       source-tag: '42.2'
       source-type: git
       plugin: meson
       meson-parameters:
         - --prefix=/usr
         - --buildtype=release
       parse-info: [usr/share/metainfo/org.gnome.TextEditor.appdata.xml]

   slots:
     gnome-text-editor:
       interface: dbus
       bus: session
       name: org.gnome.TextEditor



Metadata
^^^^^^^^

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: gnome-text-editor
   grade: stable
   adopt-info: gnome-text-editor
   license: GPL-3.0+

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

You can also fill in ``title``, ``version``, ``summary``, ``description`` and ``icon``. However, Text Editor already has this metadata defined using an `AppStream <https://www.freedesktop.org/wiki/Distributions/AppStream/>`__ metadata file ``org.gnome.TextEditor.appdata.xml``, so we don’t want to duplicate this data. We instead use :ref:`adopt-info <using-external-metadata>` to tell Snapcraft to get the metadata from the ``gnome-text-editor`` part further on in the yaml. More on this later.

Base
^^^^

The :ref:`base <base-snaps>` keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered, and specified, when building a snap.

.. code:: yaml

   base: core22

`core22 <https://snapcraft.io/core22>`__ is latest standard base for snap building and is built on `Ubuntu 22.04 LTS <http://releases.ubuntu.com/22.04/>`__.

Security model
^^^^^^^^^^^^^^

We’re going to use *strict* :ref:`confinement <snap-confinement>` for Text Editor. Strictly confined snaps run in complete isolation, up to a minimal access level that’s deemed always safe.

.. code:: yaml

   confinement: strict

Unconfined applications, specified with ``devmode``, are useful while you build a working snap. Devmode snaps cannot be released to the stable channel, do not appear in search results, and do not automatically refresh. But after you get the snap working in ``devmode`` confinement, you can switch to strict mode and figure out which interfaces (plugs) the snap uses.

Apps
^^^^

Apps are the commands and services exposed to end users. If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``gnome-text-editor.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

If you don’t want your command prefixed you can request an alias for it on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__. These are set up automatically when your snap is installed from the Snap Store.

.. code:: yaml

   apps:
     gnome-text-editor:
       extensions: [gnome]
       command: usr/bin/gnome-text-editor
       desktop: usr/share/applications/org.gnome.TextEditor.desktop
       common-id: org.gnome.TextEditor.desktop
       plugs:
         - gsettings
         - cups

This application uses the :ref:`gnome extension <the-gnome-extension>`. This will make GTK4 and GNOME libraries available to the snap at runtime. It will also configure the runtime environment of the application so that all desktop functionality is correctly initialised.

The ``common-id`` property is used to link this application to the AppStream metadata specified further down below. This will cause this ``app`` to use the ``.desktop`` launcher specified in the AppStream file.

Snaps use interfaces to access resources outside of their confinement and an interface consists of a connection between a slot and a plug. The slot is the provider of the interface while the plug is the consumer. With the ``plugs:`` section, Text Editor is requesting access to the :ref:`gsettings <the-gsettings-interface>` and :ref:`cups <the-cups-interface>` interfaces to enable access to GNOME’s configuration and any configured printers.

Parts
^^^^^

Parts define how to build your app. Parts can be anything: programs, libraries, or other assets needed to create and run your application. In this case, we’re only using one to define the GitLab repository containing the GNOME Text Editor source code and how it’s to be built. In other cases these can point to local directories, local archives, other remote git repositories and other revision control systems.

:ref:`The Meson plugin <the-meson-plugin>` is used to run ``meson``, ``ninja build`` and ``ninja install`` to build the part, and we pass a couple of options to set the install location within the snap, and for which release we wish to build:

.. code:: yaml

   parts:
     gnome-text-editor:
       source: https://gitlab.gnome.org/GNOME/gnome-text-editor
       source-tag: '42.2'
       source-type: git
       plugin: meson
       meson-parameters:
         - --prefix=/usr
         - --buildtype=release
       parse-info: [usr/share/metainfo/org.gnome.TextEditor.appdata.xml]

``parse-info`` points to the AppStream metadata file. Since we used ``adopt-info: gnome-text-editor`` in the top-level metadata, the AppStream file of the ``gnome-text-editor`` part will be used to fill in the ``summary``, ``description`` and ``icon`` of this snap. See :ref:`Using AppStream metadata <meta-appstream>` for more information.

Slots
~~~~~

Many GTK3 and GTK4 applications require access to DBus in order to run correctly. However, snap blocks this access by default so you need to explicitly define that this application is allowed access to dbus.

.. code:: yaml

   slots:
     gnome-text-editor:
       interface: dbus
       bus: session
       name: org.gnome.TextEditor

Building the snap
~~~~~~~~~~~~~~~~~

To build the snap, create a new directory and run ``snapcraft init`` inside it. This will create a template snapcraft.yaml inside a snap directory:

.. code:: bash

   $ mkdir gnome-text-editor
   $ cd gnome-text-editor
   $ snapcraft init
   Created snap/snapcraft.yaml.
   Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more information about the snapcraft.yaml format.

Replace the contents of **snap/snapcraft.yaml** with our example above. You can now build the snap by running the *snapcraft* command:

.. code:: bash

   $ snapcraft
   Launching instance...
   Executed: pull gnome-text-editor
   Executed: pull gnome/sdk
   Executed: overlay gnome-text-editor
   Executed: overlay gnome/sdk
   Executed: build gnome-text-editor
   Executed: build gnome/sdk
   Executed: stage gnome-text-editor
   Executed: stage gnome/sdk
   Executed: prime gnome-text-editor
   Executed: prime gnome/sdk
   Executed parts lifecycle
   Generated snap metadata
   Created snap package gnome-text-editor_42.1_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. If we’d built the snap with *devmode* confinement, we’d also have to add the ``--devmode`` flag:

.. code:: bash

   $  sudo snap install ./gnome-text-editor*.snap --dangerous
   gnome-text-editor 42.1 installed

You can then try it out:

.. code:: bash

   $ gnome-text-editor

Removing the snap is simple too:

.. code:: bash

   $  sudo snap remove gnome-text-editor

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

Congratulations! You’ve just built and published your first GTK 4 snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
