.. 23455.md

.. _migrating-between-bases:

Migrating between bases
=======================

A *base* snap is a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications. They’re transparent to users, but they need to be considered and specified when building a snap.

See :ref:`Base snaps <base-snaps>` for details on how use and specify them.

Each base snap is built from a :ref:`corresponding Ubuntu LTS <base-snaps-supported>` release and migrating a snap from one base to the next gives the snap access to newer packages, extended support, and the latest :ref:`Snapcraft <snapcraft-overview>` features, including :ref:`plugins <supported-plugins>` and :ref:`extensions <snapcraft-extensions>`.

The complexity of the migration process is directly linked to both dependencies in the snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` and the base snap versions being migrated between.

At its simplest, migrating from one base snap to another requires only that the *base* keyword is updated:

.. code:: diff

   - base: core18
   + base: core20

But further changes will most likely be needed, and what these are will depend on the original base and the packages that are bundled alongside the application. The most common required changes are described below:

-  `No base, or old bases <migrating-between-bases-oldbase_>`__

-  `Package names <migrating-between-bases-names_>`__

-  `Architectures <migrating-between-bases-arch_>`__

-  `Environment variables <migrating-between-bases-environment_>`__

-  `Remote parts and extensions <migrating-between-bases-remote_>`__

-  `Audio interfaces <migrating-between-bases-audio_>`__

-  `Version scripts <migrating-between-bases-version_>`__

-  Plugins

   -  `name changes <migrating-between-bases-names_>`__: nodejs to npm
   -  `modified syntax <migrating-between-bases-syntax_>`__: npm, autotools, go,

-  `Application definitions <migrating-between-bases-definitions_>`__

   -  `paths <migrating-between-bases-paths_>`__
   -  `command-chain <migrating-between-bases-command-chain_>`__

-  .. rubric:: `Migrated snap examples <migrating-between-bases-examples_>`__
      :name: migrated-snap-examples


.. _migrating-between-bases-oldbase:

Updating from no or old bases
-----------------------------

Migrating a snap from having no base, or ``base: core``, to ``core18`` or ``core20``, for example, is a more involved process than going from ``core18`` to ``core20``.

This is because when building a snap with an old base, Snapcraft will operate in compatibility mode.

Compatibility mode is essentially a prior (2.43-era) version of Snapcraft, and will lose the functionality of newer releases. See :ref:`Features incompatible with bases <release-notes-snapcraft-3-0-base-exceptions>` for details.


.. _migrating-between-bases-names:

Package names
-------------

The ``build-packages`` and ``stage-packages`` sections in a snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` specify which packages need to be incorporated during the build and stage parts of the :ref:`Parts lifecycle <parts-lifecycle>`, and described in :ref:`Build and staging dependencies <build-and-staging-dependencies>`.

When no base or *core* is specified, packages from the Ubuntu 16.04 LTS archive are used at build and stage time. The ``core18`` base will use packages from the Ubuntu 18.04 LTS archive, whereas the ``core20`` base will consume packages from the Ubuntu 20.04 LTS archive, and package names can change between releases.

Package name example: `Irssi <https://github.com/snapcrafters/irssi/pull/9>`__

.. code:: diff

       stage-packages:
   -      - libperl5.22
   +      - libperl5.26

In the above example, the name of the Perl library package changed due to a version bump. The best way to resolve these issues is to first build your snap on the destination base system, either via *snapcraft* or a virtual machine/LXD container, and update each unresolved package in turn with the new equivalents.


.. _migrating-between-bases-arch:

Architectures
-------------

The *architectures* keyword defines a set of both build and run architectures:

.. code:: yaml

   architectures:
     - build-on: amd64
       run-on: amd64

Snaps that produce i386 builds are supportable for the lifetime of Ubuntu 16.04 LTS or Ubuntu 18.04 LTS when using the core or core18 snaps as the base, but ``base: core20`` does not support the i386 architecture.

Publishers who want to move to ‘base: core20’ must drop builds for the i386 architecture since it isn’t unavailable. Supported ``core20`` architectures are listed below:

.. code:: yaml

   architectures:
     - build-on: amd64
     - build-on: arm64
     - build-on: armhf
     - build-on: ppc64el
     - build-on: s390x

For potential approaches to maintain an i386 build of a snap, see `How best to handle i386 when moving to core20 <https://forum.snapcraft.io/t/17680>`_.


.. _migrating-between-bases-environment:

Environment variables
---------------------

Environment variables are often used in snaps to ensure binaries are able to find loadable modules or libraries which reside inside the snap at runtime. Sometimes this results in path names which require updates due to directory name changes between versions.

Environment variables example: `Irssi <https://github.com/snapcrafters/irssi/pull/9>`__

.. code:: diff

       environment:
   -        PERL5LIB:  "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl-base/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl5/5.22/:$SNAP/usr/share/perl5/:$SNAP/usr/share/perl/5.22.1/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.22/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.22.1/"
   +        PERL5LIB:  "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl-base/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl5/5.26/:$SNAP/usr/share/perl5/:$SNAP/usr/share/perl/5.26.1/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.26/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.26.1/"

When a package name changes or is updated, it’s worth checking to make sure no environment variables are dependent on a path related to an older name, as with the above path.


.. _migrating-between-bases-remote:

Remote parts and Extensions
---------------------------

In some snaps :ref:`remote parts <remote-reusable-parts>` may have been used to share configuration across multiple snaps and to reduce the local :file:`snapcraft.yaml` complexity.

These parts are defined elsewhere, and would be incorporated at build time. This functionality is deprecated, so remote parts should be pasted directly into the :file:`snapcraft.yaml` file or referenced from their source repository.

Example of pasted remote part: `Mr Rescue <https://github.com/snapcrafters/mrrescue/pull/6>`__

.. code:: diff

    parts:
      mrrescue:
   -    after:
   -      - desktop-glib-only
   +    desktop-glib-only:
   +      build-packages:
   +        - libglib2.0-dev
   +      plugin: make
   +      source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
   +      source-subdir: glib-only
   +      stage-packages:
   +        - libglib2.0-bin

Alternatively for some desktop applications it may be appropriate to switch to using an extension, which simplifies the :file:`snapcraft.yaml` file further. This is covered in :ref:`Snapcraft Extensions <snapcraft-extensions>`.

Example migration to an Extension: `Xonotic <https://github.com/snapcrafters/xonotic/pull/6>`__

.. code:: diff

    parts:
      xonotic:
   -    after:
   -      - desktop-glib-only
    apps:
      xonotic:
   -    command: desktop-launch $SNAP/Xonotic/xonotic-linux-sdl.sh
   +    extensions: [gnome-3-34]
   +    command: Xonotic/xonotic-linux-sdl.sh

In the above example, we remove the reference to a remote part ``desktop-glib-only`` and instead use the ``extensions`` section to use the ``gnome-3-34`` extension, which replaces the functionality of the remote part.

Extension naming
----------------

Not all extensions work on all bases. For example, on ``core18`` , use the ``gnome-3-34`` extension and on ``core20`` use ``gnome-3-38``. See :ref:`Supported extensions <supported-extensions>` for further details.

Example showing ``core20``-only Gnome extension: `Dwarf Fortress <https://github.com/ultraviolet-1986/df/pull/3>`__

.. code:: diff

    parts:
      tarball:
   -     after: [desktop-gtk3]
    apps:
      dwarffortress:
   -    command: desktop-launch $SNAP/wrapper.sh
   +    extensions: [gnome-3-38]
   +    command: wrapper.sh


.. _migrating-between-bases-audio:

Audio interfaces
----------------

For applications which play or record audio, the :ref:`interface <interface-management>` names have changed. Previously the :ref:`pulseaudio <the-pulseaudio-interface>` interface was used for both playback and recording of audio. This has been replaced by :ref:`audio-playback <the-audio-playback-interface>` and :ref:`audio-record <the-audio-record-interface>`:

Example audio interface update: `Xonotic <https://github.com/snapcrafters/xonotic/pull/6>`__

.. code:: diff

    apps:
      xonotic:
        plugs:
   -      pulseaudio
   +      audio-playback

Note that to ensure privacy, ``audio-playback`` is automatically connected but ``audio-record`` is *not*.

Application publishers who believe ``audio-record`` *should* be automatically connected on install (such as for an audio recording application) should start a thread in the `store-requests <https://forum.snapcraft.io/c/store-requests/19>`__ category on the Snapcraft forum asking for it.


.. _migrating-between-bases-version:

Version scripts
---------------

The top level ``version-script`` option has been :ref:`deprecated <deprecation-notice-10>` in favour of ``adopt-info``. This requires that you specify ``adopt-info`` with a reference to the part in which the version data (and some other metadata) may be set.

Within the ``parts`` section, use ``snapcraftctl set-version`` to define the snapcraft project version number used at build time.

Example replacing *version-script* with *adopt-info*: `Cointop <https://github.com/miguelmota/cointop/pull/94>`__

.. code:: diff

   -version-script: git -C parts/cointop/build rev-parse --short HEAD
   +adopt-info: cointop
    parts:
      cointop:
   +    override-pull: |
   +      snapcraftctl pull
   +      snapcraftctl set-version $(git rev-parse --short HEAD)

See :ref:`Using external metadata <using-external-metadata>` for further details.


.. _#migrating-between-bases-name:

Plugin name changes
-------------------

The following plugin names have changed across Snapcraft releases:

nodejs / npm
------------

The ``nodejs`` plugin is now ``npm``.

e.g. `wethr <https://github.com/snapcrafters/wethr/commit/678ac026fb03d42925eb585f376245ee073747ad>`__

.. code:: diff

    parts:
      wethr:
   -    plugin: nodejs
   +    plugin: npm


.. _migrating-between-bases-syntax:

Plugin syntax
-------------

Plugin changes can be queried with the ``snapcraft help <plugin name> --base <base name>`` command:

.. code:: bash

   $ snapcraft help npm --base core20
   Displaying help for the 'npm' plugin for 'core20'.
   [...]

You can also list plugins for a specific base with ``snapcraft list-plugins --base <base name>``:

.. code:: bash

   $ snapcraft list-plugins --base core20
   Displaying plugins available for 'core20'
   autotools  catkin  catkin-tools  cmake  colcon  dump  go  make
   meson nil  npm  python  qmake  rust

The following plugins have changed their syntax across Snapcraft releases.

npm
---

The :ref:`npm plugin <the-npm-plugin>` uses ``npm-node-version`` instead of ``node-engine`` to specify the version of upstream npm to be used at build time.

Example npm plugin syntax change: `wethr <https://github.com/snapcrafters/wethr/commit/678ac026fb03d42925eb585f376245ee073747ad>`__

.. code:: diff

    parts:
      wethr:
   -    node-engine: "10.14.1"
   +    npm-node-version: "10.14.1"

autotools
---------

The :ref:`Autotools plugin <the-autotools-plugin>` has migrated options from ``configflags`` to ``autotools-configure-parameters``.

Example Autotools plugin syntax changes: `Inadyn <https://github.com/snapcrafters/inadyn/commit/ba4f114eb07a3295e40798869c9cf7ce476e8037>`__

.. code:: diff

    parts:
      libconfuse:
       plugin: autotools
   -    configflags: ['--prefix=/usr', '--disable-examples', '--disable-static']
   +    autotools-configure-parameters: ['--prefix=/usr', '--disable-examples', '--disable-static']

go
--

The :ref:`go plugin <the-go-plugin>` no longer requires the ``go-importpath`` to be specified. A ``go-channel`` should be specified.

Example Go plugin syntax changes: `slack-term <https://github.com/snapcrafters/slack-term/commit/bca6333f64297a1c117b8fc9560eb92b427e0ea7>`__

.. code:: diff

    parts:
      slack-term:
        plugin: go
   -      go-importpath: github.com/erroneousboat/slack-term
   +      go-channel: latest/stable


.. _migrating-between-bases-definitions:

Application definitions
-----------------------


.. _migrating-between-bases-paths:

Paths
~~~~~

Snapcraft now requires explicit paths to be specified for binaries listed in the ``apps`` stanza:

Example update adding explicit paths: `wethr <https://github.com/snapcrafters/wethr/commit/678ac026fb03d42925eb585f376245ee073747ad>`__

.. code:: diff

    apps:
      wethr:
   -    command: wethr
   +    command: bin/wethr


.. _migrating-between-bases-command-chain:

command-chain
~~~~~~~~~~~~~

Rather than specify ``command`` followed by a long list of space-separated executables, they can now be listed with the :ref:`command-chain <snapcraft-app-and-service-metadata-command-chain>` option:

Example of command being replaced by command-chain: `Atom <https://github.com/snapcrafters/atom/pull/64>`__

.. code:: diff

    apps:
      atom:
   -    command: bin/launcher ${SNAP}/usr/share/atom/atom
   +    command-chain:
   +      - bin/launcher
   +    command: usr/share/atom/atom


.. _migrating-between-bases-examples:

Examples summary
----------------

-  `Atom <https://github.com/snapcrafters/atom/pull/64>`__
-  `Cointop <https://github.com/miguelmota/cointop/pull/94>`__
-  `ddgr <https://github.com/snapcrafters/ddgr/pull/3>`__
-  `Duck Marines <https://github.com/snapcrafters/duckmarines/pull/5>`__
-  `Dwarf Fortress <https://github.com/ultraviolet-1986/df/pull/3>`__
-  `Irssi <https://github.com/snapcrafters/irssi/pull/9>`__
-  `Mr Rescue <https://github.com/snapcrafters/mrrescue/pull/6>`__
-  `slack-term <https://github.com/snapcrafters/slack-term/commit/bca6333f64297a1c117b8fc9560eb92b427e0ea7>`__
-  `wethr <https://github.com/snapcrafters/wethr/commit/678ac026fb03d42925eb585f376245ee073747ad>`__
-  `Xonotic <https://github.com/snapcrafters/xonotic/pull/6>`__
