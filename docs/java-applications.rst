.. 7819.md

.. _java-applications:

Java applications
=================

Distributing a Java application for Linux while reaching the widest possible audience is complicated. Typically, the user has to make sure the JRE/SDK version and their environment are configured correctly. When a Linux distribution changes the delivered JRE, this can be problematic for applications.

Snaps solve these problems and ensure the correct JRE is shipped alongside the application at all times.

Why are snaps good for Java projects?
-------------------------------------

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the exact version of whatever is required, along with all of your app’s dependencies, be they Java modules or system libraries.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision.

Build a snap in 20 minutes
--------------------------

Ready to get started? By the end of this guide, you’ll understand how to make a snap of your Java app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

   ℹ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for an existing project, `Freeplane <https://github.com/snapcraft-docs/freeplane>`__. Don’t worry, we’ll break this down.

Using a few lines of yaml and the snapcraft tool, a Java application, it’s dependencies and the correct JRE can be packaged as a snap. We’ll break this down.

.. code:: yaml

   name: freeplane
   title: Freeplane
   version: '1.8.1'
   summary: A free tool to structure and organise your information with mind mapping
   description: |
     Freeplane is a free and open source software application that supports
     thinking, sharing information and getting things done at work, in school
     and at home. The core of the software is tools for mind mapping (also known
     as concept mapping or information mapping) and using mapped information.

     Occupying the middle ground between an editor and a diagramming tool,
     Freeplane allows the user to add content as quickly and naturally as they
     would in a text editor, yet producing structured content that can be
     manipulated as easily as a diagram.

     Features include ordering ideas in nodes and freely positionable nodes,
     connecting nodes, automatic/conditional styles, scripting, add-ons, LaTeX,
     search/filtering, different export features, printing, password protection
     of nodes/maps and more.

   base: core18
   confinement: strict

   apps:
     freeplane:
       extensions:
         - gnome-3-28
       command: freeplane-$SNAPCRAFT_PROJECT_VERSION/freeplane.sh
       environment:
         JAVA_HOME: $SNAP/usr/lib/jvm/java-11-openjdk-amd64
         PATH: $JAVA_HOME/jre/bin:$PATH
       plugs:
         - home
         - network
         - cups-control

   parts:
     freeplane:
       plugin: gradle
       source: https://github.com/freeplane/freeplane.git
       source-tag: release-$SNAPCRAFT_PROJECT_VERSION
       gradle-version: '5.1.1'
       gradle-output-dir: BIN
       gradle-options: [binZip, -xtest, -xcreateGitTag]
       override-build: |
         snapcraftctl build
         unzip -o DIST/freeplane_bin-*.zip -d $SNAPCRAFT_PART_INSTALL/
       build-packages:
         - unzip



Metadata
--------

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: freeplane
   title: Freeplane
   version: '1.8.1'
   summary: A free tool to structure and organise your information with mind mapping
   description: |
     Freeplane is a free and open source software application that supports
     thinking, sharing information and getting things done at work, in school
     and at home. The core of the software is tools for mind mapping (also known
     as concept mapping or information mapping) and using mapped information.

     Occupying the middle ground between an editor and a diagramming tool,
     Freeplane allows the user to add content as quickly and naturally as they
     would in a text editor, yet producing structured content that can be
     manipulated as easily as a diagram.

     Features include ordering ideas in nodes and freely positionable nodes,
     connecting nodes, automatic/conditional styles, scripting, add-ons, LaTeX,
     search/filtering, different export features, printing, password protection
     of nodes/maps and more.

Base
----

The base keyword declares which *base snap* to use with your project. A base snap is a special kind of snap that provides a run-time environment alongside a minimal set of libraries that are common to most applications:

.. code:: yaml

   base: core18

As used above, `core18 <https://snapcraft.io/core18>`__ is the current standard base for snap building and is based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.

See :ref:`Base snaps <base-snaps>` for more details.

Security model
--------------

The next section describes the level of confinement applied to your app.

.. code:: yaml

   confinement: devmode

Snaps are containerised to ensure more predictable application behaviour and greater security. Unlike other container systems, the shape of this confinement can be changed through a set of interfaces. These are declarations that tell the system to give permission for a specific task, such as accessing a webcam or binding to a network port.

It’s best to start a snap with the confinement in warning mode, rather than strictly applied. This is indicated through the ``devmode`` keyword. When a snap is in devmode, runtime confinement violations will be allowed but reported. These can be reviewed by running ``journalctl -xe``.

Because devmode is only intended for development, snaps must be set to strict confinement before they can be published as “stable” in the Snap Store. Once an app is working well in devmode, you can review confinement violations, add appropriate interfaces, and switch to strict confinement.

Apps
----

Apps are the commands and services exposed to end users. If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``freeplane.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

If you don’t want your command prefixed you can request an alias for it on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__. These are set up automatically when your snap is installed from the Snap Store.

.. code:: yaml

   apps:
     freeplane:
       extensions:
         - gnome-3-28
       command: freeplane-$SNAPCRAFT_PROJECT_VERSION/freeplane.sh
       environment:
         JAVA_HOME: $SNAP/usr/lib/jvm/java-11-openjdk-amd64
         PATH: $JAVA_HOME/jre/bin:$PATH
       plugs:
         - home
         - network
         - cups-control

Since Freeplane is a desktop application, we use the :ref:`gnome-3-28 extension <the-gnome-3-28-extension>` to configure and setup the desktop integration and permissions for the snap. Although Freeplane is a Java Swing application which doesn’t need access to GTK or GNOME, the GNOME extension is still useful because it sets up many toolkit-independent libraries and functionality such as mouse cursor themes, locales and the XDG runtime environment.

Parts
-----

Parts define how to build your app. Parts can be anything: programs, libraries, or other assets needed to create and run your application. In this case we have only one: the Freeplane source. In other cases these can point to local directories, remote git repositories or other revision control systems.

The gradle plugin can build the application using standard parameters. In this case, however, the default build logic of the gradle plugin is not sufficient. While gradle by default build the ``jar`` target, Freeplane has a ``binZip`` target which build a handy zip file. We use ``gradle-options`` to specify that we want to build the ``binZip`` target and use an :ref:`override-build scriptlet to add additional logic <override-build-steps-overriding-the-build-step>` to the build step to extract the zip in the directory which will later get added to the final snap. See the :ref:`parts lifecycle docs <parts-lifecycle-parts-directories>` for more information on these directories. Since we use the ``unzip`` command in the build script, we specify it in ``build-packages`` so it is installed before the build script runs. Finally, we use the ``gradle-output-dir`` key to point the snapcraft plugin to the location of the built ``jar`` files for Freeplane.

.. code:: yaml

   parts:
     freeplane:
       plugin: gradle
       source: https://github.com/freeplane/freeplane.git
       source-tag: release-$SNAPCRAFT_PROJECT_VERSION
       gradle-version: '5.1.1'
       gradle-output-dir: BIN
       gradle-options: [binZip, -xtest, -xcreateGitTag]
       override-build: |
         snapcraftctl build
         unzip -o DIST/freeplane_bin-*.zip -d $SNAPCRAFT_PART_INSTALL/
       build-packages:
         - unzip

For more details on Gradle-specific metadata, see :ref:`The Gradle plugin <the-gradle-plugin>`.

Building the snap
-----------------

You can download the example repository with the following command:

::

   $ git clone https://github.com/galgalesh/freeplane-1

After you’ve created the snapcraft.yaml, you can build the snap by simply executing the snapcraft command in the project directory:

.. code:: bash

   $ snapcraft

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install freeplane_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ freeplane

Removing the snap is simple too:

.. code:: bash

   $ sudo snap remove freeplane

Publishing your snap
--------------------

To share your snaps you need to publish them in the Snap Store. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads and control publishing.

You’ll need to choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the :command:`snapcraft` command is authenticated using the email address attached to your Snap Store account:

.. code:: bash

   $ snapcraft login

Reserve a name for your snap
----------------------------

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

.. code:: bash

   $ snapcraft register myjavasnap

Be sure to update the ``name:`` in your :file:`snapcraft.yaml` file to match this registered name, then run :command:`snapcraft` again.

Upload your snap
----------------

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   $ snapcraft upload --release=edge myjavasnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first Java snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
