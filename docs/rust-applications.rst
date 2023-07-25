.. 7826.md

.. _rust-applications:

Rust applications
=================

You can distribute your Rust apps to many different Linux distributions using a MUSL-enabled version of Rust, with all the dependencies satisfied. However, end user discovery and update management remain a challenge.

Snaps fill this gap, letting you distribute a Rust app in an app store experience for end users.

Why are snaps good for Rust projects?
-------------------------------------

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the exact versions of the tools you need, along with all of your app’s dependencies, be they modules or system libraries.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision with data preserved.

Build a snap in 20 minutes
~~~~~~~~~~~~~~~~~~~~~~~~~~

Ready to get started? By the end of this guide, you’ll understand how to make a snap of your app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

   ℹ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for an existing project, `XSV <https://github.com/snapcraft-docs/xsv>`__, leveraging the existing ``Cargo.toml`` to satisfy runtime requirements. Don’t worry, we’ll break this down.

.. code:: yaml

   name: test-xsv
   version: git
   summary: A fast CSV command line toolkit written in Rust
   description: |
     xsv is a command line program for indexing,
     slicing, analysing, splitting and joining CSV
     files. Commands should be simple, fast and
     composable:
     - Simple tasks should be easy.
     - Performance trade offs should be exposed
       in the CLI interface.
     - Composition should not come at the
       expense of performance.

   base: core18
   confinement: devmode


   parts:
     test-xsv:
       plugin: rust
       source: .

   apps:
     test-xsv:
       command: bin/xsv

Metadata
~~~~~~~~

The ``snapcraft.yaml`` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: test-xsv
   version: git
   summary: A fast CSV command line toolkit written in Rust
   description: |
     xsv is a command line program for indexing,
     slicing, analysing, splitting and joining CSV
     files. Commands should be simple, fast and
     composable:
     - Simple tasks should be easy.
     - Performance trade offs should be exposed
       in the CLI interface.
     - Composition should not come at the
       expense of performance.

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers and they also cannot start or end with a hyphen.

By specifying ``git`` for the version, the current git tag or commit will be used as the version string. Versions carry no semantic meaning in snaps.

The ``summary`` can not exceed 79 characters. You can use a chevron ‘>’ in the ``description`` key to declare a multi-line description.

Base
^^^^

The base keyword defines a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they must be considered, and specified, when building a snap.

.. code:: yaml

   base: core18

`core18 <https://snapcraft.io/core18>`__ is the current standard base for snap building and is based on `Ubuntu 18.04 LTS <http://releases.ubuntu.com/18.04/>`__.

Security model
~~~~~~~~~~~~~~

To get started we won’t confine this application. Unconfined applications, specified with ``devmode``, can only be released to the hidden “edge” channel where you and other developers can install them.

.. code:: yaml

   confinement: devmode

Parts
~~~~~

Parts define what sources are needed to assemble your app. Parts can be anything: programs, libraries, or other needed assets, but for now, we’re only going to use one part: the *xsv* source code.

This example will also bundle the current stable release of Rust in the snap using Rustup, and you can define the exact version of Rust with the optional ``- rust-revision:`` keyword, should you have specific requirements. Dependencies from your ``Cargo.toml`` will also be bundled.

.. code:: yaml

   parts:
     test-xsv:
       plugin: rust
       source: .

For more details on Rust-specific metadata, see :ref:`The Rust plugin <the-rust-plugin>`.

Apps
~~~~

Apps are the commands and services exposed to end users. If your Apps are the commands you want to expose to users and any background services your application provides. Each key under ``apps`` is the command name that should be made available on users’ systems.

The ``command`` specifies the full path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
     test-xsv:
       command: bin/xsv

If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``test-xsv.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

If your application is intended to run as a service you must add the line ``daemon: simple`` after the command keyword. This will automatically keep the service running on install, update and reboot.

You can request an alias on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__ if your command name and snap name do not match but you don’t want your command prefixed. These aliases are set up automatically when your snap is installed from the Snap Store.

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/xsv

After you have created the :file:`snapcraft.yaml` file, you can build the snap by executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Using 'snapcraft.yaml': Project assets will be searched for from the 'snap' directory.
   Launching a VM.
   [...]
   Snapped test-xsv_0.13.0+git1.0728f0c-dirty_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install test-xsv_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ test-xsv

Removing the snap is simple too:

.. code:: bash

   $ sudo snap remove test-xsv

You can also clean up the build environment, although this will slow down the next initial build:

.. code:: bash

   $ snapcraft clean

By default, when you make a change to snapcraft.yaml, snapcraft only builds the parts that have changed. Cleaning a build, however, forces your snap to be rebuilt in a clean environment and will take longer.

Publishing your snap
--------------------

To share your snaps you must publish them in the Snap Store. First, create an account on `the dashboard <https://dashboard.snapcraft.io/dev/account/>`__. Here you can customise how your snaps are presented, review your uploads and control publishing.

You must choose a unique “developer namespace” as part of the account creation process. This name will be visible by users and associated with your published snaps.

Make sure the :command:`snapcraft` command is authenticated using the email address attached to your Snap Store account:

.. code:: bash

   $ snapcraft login

Reserve a name for your snap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can publish your own version of a snap, provided you do so under a name you have rights to. You can register a name on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/register-snap/>`__, or by running the following command:

.. code:: bash

   $ snapcraft register myrustsnap

Be sure to update the ``name:`` in your ``snapcraft.yaml`` to match this registered name, then run :command:`snapcraft` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   $ snapcraft upload --release=edge myrustsnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first Rust snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
