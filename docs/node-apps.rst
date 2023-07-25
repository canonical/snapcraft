.. 6747.md

.. _node-apps:

Node apps
=========

*Node.js* has the ``package.json`` format and *npm* or *yarn* to assist in packaging and distributing applications to other developers. But these are not user-oriented tools. Nor do they offer a solution for notifying users of available updates.

Snaps address these gaps while building upon the work you’ve already done to teach Node how to package your app.

Why are snaps good for Node.js projects?
----------------------------------------

-  **Snaps are easy to discover and install** Millions of users can browse and install snaps graphically in the Snap Store or from the command-line.
-  **Snaps install and run the same across Linux** They bundle the exact version of whatever is required, along with all of your app’s dependencies, be they modules or system libraries.
-  **Snaps automatically update to the latest version** Four times a day, users’ systems will check for new versions and upgrade in the background.
-  **Upgrades are not disruptive** Because upgrades are not in-place, users can keep your app open as it’s upgraded in the background.
-  **Upgrades are safe** If your app fails to upgrade, users automatically roll back to the previous revision.

Build a snap in 20 minutes
--------------------------

Ready to get started? By the end of this guide, you’ll understand how to make a snap of your Node.js app that can be published in the `Snap Store <https://snapcraft.io/store>`__, showcasing it to millions of Linux users.

   ℹ For a brief overview of the snap creation process, including how to install *snapcraft* and how it’s used, see :ref:`Snapcraft overview <snapcraft-overview>`. For a more comprehensive breakdown of the steps involved, take a look at :ref:`Creating a snap <creating-a-snap>`.

Getting started
---------------

Snaps are defined in a single YAML file placed in the root folder of your project. The following example shows the entire :file:`snapcraft.yaml` file for an existing project, `Wethr <https://github.com/snapcraft-docs/wethr>`__. Don’t worry, we’ll break this down.

.. code:: yaml

   name: wethr
   version: git
   summary: Command line weather tool.
   description: |
     Get current weather:-
       $ wethr
     Get current weather in metric units
       $ wethr --metric
     Get current weather in imperial units
       $ wethr --imperial

   confinement: strict
   base: core20

   apps:
     wethr:
       command: bin/wethr

   parts:
     wethr:
       plugin: npm
       npm-node-version: 14.16.1
       source: .

Metadata
~~~~~~~~

The :file:`snapcraft.yaml` file starts with a small amount of human-readable metadata, which usually can be lifted from the GitHub description or project README.md. This data is used in the presentation of your app in the Snap Store.

.. code:: yaml

   name: wethr
   version: git
   summary: Command line weather tool.
   description: |
     Get current weather:-
       $ wethr
     Get current weather in metric units
       $ wethr --metric
     Get current weather in imperial units
       $ wethr --imperial

The ``name`` must be unique in the Snap Store. Valid snap names consist of lower-case alphanumeric characters and hyphens. They cannot be all numbers. They also cannot start or end with a hyphen.

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

   confinement: strict

Snaps are containerised to ensure more predictable application behaviour and greater security. Unlike other container systems, the shape of this confinement can be changed through a set of interfaces. These are declarations that tell the system to give permission for a specific task, such as accessing a webcam or binding to a network port.

It’s best to start a snap with the confinement in warning mode, rather than strictly applied. This is indicated through the ``devmode`` keyword. When a snap is in devmode, runtime confinement violations will be allowed but reported. These can be reviewed by running ``journalctl -xe``.

Because devmode is only intended for development, snaps must be set to strict confinement before they can be published as “stable” in the Snap Store. Once an app is working well in devmode, you can review confinement violations, add appropriate interfaces, and switch to strict confinement.

Parts
~~~~~

Parts define what sources are needed to assemble your app. Parts can be anything: programs, libraries, or other needed assets. We’ll deal with libraries and other assets later, so for now we just have one part: the wethr source code.

.. code:: yaml

   parts:
     wethr:
       plugin: npm
       npm-node-version: 14.16.1
       source: .

The :ref:`npm plugin <the-npm-plugin>` builds upon the work you’ve already done to describe your app’s dependencies in your package.json. It will automatically include these in your snap.

The plugin also needs to know which version of Node to bundle. This is specified with the ``npm-node-version`` keyword.

Apps
~~~~

Apps are the commands you want to expose to users and any background services your application provides. Each key under ``apps`` is the command name that should be made available on users’ systems.

The ``command`` specifies the full path to the binary to be run. This is resolved relative to the root of your snap contents.

.. code:: yaml

   apps:
     wethr:
       command: bin/wethr

If your command name matches the snap ``name``, users will be able run the command directly. If the names differ, then apps are prefixed with the snap ``name`` (``wethr.command-name``, for example). This is to avoid conflicting with apps defined by other installed snaps.

You can request an alias on the `Snapcraft forum <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__ if your command name and snap name do not match but you don’t want your command prefixed. These aliases are set up automatically when your snap is installed from the Snap Store.

Building the snap
~~~~~~~~~~~~~~~~~

You can download the example repository with the following command:

.. code:: bash

   $ git clone https://github.com/snapcraft-docs/wethr

After you have created the :file:`snapcraft.yaml` file, or used the one provided, you can build the snap by simply executing the :command:`snapcraft` command in the project directory:

.. code:: bash

   $ snapcraft
   Launching a container.
   Waiting for container to be ready
   Waiting for network to be ready...
   snapd is not logged in, snap install commands will use sudo
   snap "core20" has no updates available
   Updating pull step for wethr (source changed)
   + snapcraftctl pull
   Updating build step for wethr ('pull' step changed)
   + snapcraftctl build
   + '[' '!' -f /root/parts/wethr/install/bin/node ']'
   + curl -s https://nodejs.org/dist/v14.16.1/node-v14.16.1-linux-x64.tar.gz
   + tar xzf - -C /root/parts/wethr/install/ --strip-components=1
   ++ npm pack .
   ++ tail -1
   npm notice
   npm notice 📦  wethr@1.5.2
   npm notice === Tarball Contents ===
   npm notice 1.1kB  LICENSE
   npm notice 29.1kB demo.gif
   npm notice 853B   get-emoji.js
   npm notice 1.7kB  test/get-emoji.js
   npm notice 1.6kB  wethr.js
   npm notice 602B   package.json
   npm notice 298B   README.md
   npm notice 30.7kB wethr-1.5.2.tgz
   npm notice 385B   snapcraft.yaml
   npm notice === Tarball Details ===
   npm notice name:          wethr
   npm notice version:       1.5.2
   npm notice filename:      wethr-1.5.2.tgz
   npm notice package size:  61.6 kB
   npm notice unpacked size: 66.3 kB
   npm notice shasum:        eb4c7ddc744ed0c6d2260f53fc14886683e6a239
   npm notice integrity:     sha512-cMLaJp36D45YW[...]gagZhUY0uLgSA==
   npm notice total files:   9
   npm notice
   + npm install -g --prefix /root/parts/wethr/install wethr-1.5.2.tgz
   /root/parts/wethr/install/bin/wethr -> /root/parts/wethr/install/lib/node_modules/wethr/wethr.js
   + wethr@1.5.2
   added 47 packages from 34 contributors in 2.555s
   Cleaning later steps and re-staging wethr ('build' step changed)
   + snapcraftctl stage
   Priming wethr
   + snapcraftctl prime
   'grade' property not specified: defaulting to 'stable'.
   Determining the version from the project repo (version: git).
   The version has been set to 'v1.4.0+git11.0cf85b4'
   Snapping |
   Snapped wethr_v1.4.0+git11.0cf85b4_amd64.snap

The resulting snap can be installed locally. This requires the ``--dangerous`` flag because the snap is not signed by the Snap Store. The ``--devmode`` flag acknowledges that you are installing an unconfined application:

.. code:: bash

   $ sudo snap install wethr_*.snap --devmode --dangerous

You can then try it out:

.. code:: bash

   $ wethr
   London, GB: 17.04C 🌧

Removing the snap is simple too:

.. code:: bash

   $ sudo snap remove wethr

You can also clean up the build environment, although this will slow down the next initial build:

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

   $ snapcraft register mynodesnap

Be sure to update the ``name:`` in your :file:`snapcraft.yaml` to match this registered name, then run :command:`snapcraft` again.

Upload your snap
~~~~~~~~~~~~~~~~

Use snapcraft to push the snap to the Snap Store.

.. code:: bash

   $ snapcraft upload --release=edge mynodesnap_*.snap

If you’re happy with the result, you can commit the snapcraft.yaml to your GitHub repo and `turn on automatic builds <https://build.snapcraft.io>`__ so any further commits automatically get released to edge, without requiring you to manually build locally.

Congratulations! You’ve just built and published your first Node snap. For a more in-depth overview of the snap building process, see :ref:`Creating a snap <creating-a-snap>`.
