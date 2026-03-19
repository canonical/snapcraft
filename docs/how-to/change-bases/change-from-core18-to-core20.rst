.. _how-to-change-from-core18-to-core20:

Change from core18 to core20
============================

core20 support in Snapcraft introduces new features and changes that necessitate some
tweaks to your project when transitioning from core18.


Package names
-------------

The ``build-packages`` and ``stage-packages`` keys in a snap's project file specify
which packages need to be incorporated during the build and stage parts of the
:ref:`parts lifecycle <explanation-parts-lifecycle>` and described in build and staging
dependencies.

When no base or core is specified, packages from the Ubuntu 16.04 LTS archive are used
at build and stage time. The core18 base will use packages from the Ubuntu 18.04 LTS
archive, whereas the core20 base will consume packages from the Ubuntu 20.04 LTS
archive. This is important to note, as package names can change between releases. For
example, the `Irssi snap's core migration
<https://github.com/snapcrafters/irssi/pull/9/files>`_ includes:

.. code-block:: diff
    :caption: snapcraft.yaml of Irssi

      stage-packages:
    -   - libperl5.22
    +   - libperl5.26

In the above example, the name of the Perl library package changed due to a version
bump. The best way to resolve these issues is to first build your snap on the
destination base system, either via Snapcraft or a virtual machine/LXD container, before
updating each unresolved package with the new version.


Architectures
-------------

The ``architectures`` keyword defines a set of both build and run architectures:

.. code-block:: yaml

    architectures:
      - build-on: amd64
        run-on: amd64

Snaps that produce i386 builds are supportable for the lifetime of Ubuntu 16.04 LTS or
Ubuntu 18.04 LTS when using the core or core18 snaps as the base, but core20 does not
support the i386 architecture.

Publishers who want to move to core20 must drop builds for the i386 architecture since
it isn't supported. Supported core20 architectures are listed below:

.. code-block:: yaml

    architectures:
      - build-on: amd64
      - build-on: arm64
      - build-on: armhf
      - build-on: ppc64el
      - build-on: s390x


Environment variables
---------------------

Environment variables are often used in snaps to ensure binaries are able to find
loadable modules or libraries which reside inside the snap at runtime. Sometimes this
results in path names which require updates due to directory name changes between
versions.

.. code-block:: diff
    :caption: snapcraft.yaml of Irssi

      environment:
    -   PERL5LIB:  "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl-base/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl5/5.22/:$SNAP/usr/share/perl5/:$SNAP/usr/share/perl/5.22.1/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.22/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.22.1/"
    +   PERL5LIB:  "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl-base/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl5/5.26/:$SNAP/usr/share/perl5/:$SNAP/usr/share/perl/5.26.1/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.26/:$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/perl/5.26.1/"

When a package name changes, it's worth checking to make sure no environment variables
are dependent on a path related to an older name, as with the above path.


Remote parts and extensions
---------------------------

In some snaps, remote parts may have been used to share configuration across multiple
snaps and to reduce the local project file's complexity.

These parts are defined elsewhere, and would be incorporated at build time. This
functionality is deprecated, so remote parts should be pasted directly into the
project file or referenced from their source repository. For example, the `Mr. Rescue
snap's core migration <https://github.com/snapcrafters/mrrescue/pull/6>`_ includes:

.. code-block:: diff
    :caption: snapcraft.yaml of Mr. Rescue

      parts:
        mrrescue:
    -     after:
    -       - desktop-glib-only
    +     desktop-glib-only:
    +       build-packages:
    +         - libglib2.0-dev
    +       plugin: make
    +       source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
    +       source-subdir: glib-only
    +       stage-packages:
    +         - libglib2.0-bin

Alternatively, for some desktop applications, it may be appropriate to use an extension,
which further simplifies the snap's project file. For example, the `Xonotic snap's core
migration <https://github.com/snapcrafters/xonotic/pull/6/files>`_ includes:

.. code-block:: diff
    :caption: snapcraft.yaml of Xonotic

      parts:
        xonotic:
    -     after:
    -       - desktop-glib-only
      apps:
        xonotic:
    -     command: desktop-launch $SNAP/Xonotic/xonotic-linux-sdl.sh
    +     extensions: [gnome-3-34]
    +     command: Xonotic/xonotic-linux-sdl.sh

In the above example, we remove the reference to a remote part ``desktop-glib-only``
and instead use the ``extensions`` key to add the ``gnome-3-34`` extension, which
replaces the functionality of the remote part.


Extension naming
~~~~~~~~~~~~~~~~

Not all extensions work on all bases. For example, core18 requires the ``gnome-3-34``
extension and core20 requires ``gnome-3-38``. For example, the `Dwarf Fortress snap's
core migration <https://github.com/ultraviolet-1986/df/pull/3/files>`_ includes:

.. code-block:: diff
    :caption: snapcraft.yaml of Dwarf Fortress

      parts:
        tarball:
    -     after: [desktop-gtk3]
      apps:
        dwarffortress:
    -     command: desktop-launch $SNAP/wrapper.sh
    +     extensions: [gnome-3-38]
    +     command: wrapper.sh


Audio interfaces
----------------

For applications which play or record audio, the interface names have changed.
Previously the `pulseaudio <https://snapcraft.io/docs/pulseaudio-interface>`_ interface
was used for both playback and recording of audio. This has been replaced by
`audio-playback <https://snapcraft.io/docs/audio-playback-interface>`_ and
`audio-record <https://snapcraft.io/docs/audio-record-interface>`_:

.. code-block:: diff
    :caption: snapcraft.yaml of Xonotic

      apps:
        xonotic:
          plugs:
    -       pulseaudio
    +       audio-playback

Note that to ensure privacy, ``audio-playback`` is automatically connected but
``audio-record`` isn't.

Application publishers who believe ``audio-record`` should be automatically connected on
install (such as for an audio recording application) should start a thread in the
`store-requests <https://forum.snapcraft.io/c/store-requests>`_ category on the
Snapcraft forum asking for it.


Version scripts
---------------

The top-level ``version-script`` key has been deprecated in favor of ``adopt-info``.
This requires you to specify ``adopt-info`` with a reference to the part in which the
version data (and some other metadata) may be set.

Within the parts section, use ``snapcraftctl set-version`` to define the Snapcraft
project version number used at build time. For example, the `Cointop snap's core
migration <https://github.com/cointop-sh/cointop/pull/94/files>`_ includes:

.. code-block:: diff
    :caption: snapcraft.yaml of Cointop

    - version-script: git -C parts/cointop/build rev-parse --short HEAD
    + adopt-info: cointop
      parts:
        cointop:
    +     override-pull: |
    +       snapcraftctl pull
    +       snapcraftctl set-version $(git rev-parse --short HEAD)

See :ref:`how-to-manage-data-compatibility` for more details.


Plugin name changes
-------------------

The following plugin names have changed across Snapcraft releases:


nodejs / npm
~~~~~~~~~~~~

The ``nodejs`` plugin is now ``npm``.

.. code-block:: diff

      parts:
        <part-name>:
    -     plugin: nodejs
    +     plugin: npm


Plugin syntax
-------------

Plugins can now be queried with the ``snapcraft help <plugin name> --base <base name>``
command:

.. terminal::
    :input: snapcraft help npm --base core20

    Displaying help for the 'npm' plugin for 'core20'.
    [...]

You can also list plugins for a specific base with ``snapcraft list plugins --base <base
name>``:

.. terminal::
    :input: snapcraft plugins --base core20

    Displaying plugins available for 'core20'
    autotools  catkin  catkin-tools  cmake  colcon  dump  go  make
    meson nil  npm  python  qmake  rust

The following plugins have changed their syntax across Snapcraft releases:


npm
~~~

The :ref:`craft_parts_npm_plugin` uses ``npm-node-version`` instead of
``node-engine`` to specify the upstream version of NPM to use at build time. The `wethr
snap's core migration
<https://github.com/snapcrafters/wethr/commit/678ac026fb03d42925eb585f376245ee073747ad>`_
includes an example of this syntax change:

.. code-block:: diff
    :caption: snapcraft.yaml of wethr

        parts:
          wethr:
      -     node-engine: "10.14.1"
      +     npm-node-version: "10.14.1"


autotools
~~~~~~~~~

The :ref:`craft_parts_autotools_plugin` has migrated options from ``configflags`` to
``autotools-configure-parameters``. The `Inadyn snap's core migration
<https://github.com/snapcrafters/inadyn/commit/ba4f114eb07a3295e40798869c9cf7ce476e8037>`_
includes an example of this syntax change:

.. code-block:: diff
    :caption: snapcraft.yaml of Inadyn

      parts:
        libconfuse:
          plugin: autotools
    -       configflags: ['--prefix=/usr', '--disable-examples', '--disable-static']
    +       autotools-configure-parameters: ['--prefix=/usr', '--disable-examples', '--disable-static']


go
~~

The :ref:`craft_parts_go_plugin` no longer requires the ``go-importpath`` to be
specified. The ``go-channel`` should now be specified. The `slack-term snap's core
migration
<https://github.com/snapcrafters/slack-term/commit/bca6333f64297a1c117b8fc9560eb92b427e0ea7>`_
includes an example of this syntax change:

.. code-block:: diff
    :caption: snapcraft.yaml of slack-term

      parts:
        slack-term:
          plugin: go
    -       go-importpath: github.com/erroneousboat/slack-term
    +       go-channel: latest/stable


Application definitions
-----------------------


Paths
~~~~~

Snapcraft now requires explicit paths to be specified for binaries listed in the
``apps`` key:

.. code-block:: diff
    :caption: snapcraft.yaml of wethr

      apps:
        wethr:
    -     command: wethr
    +     command: bin/wethr


command-chain
~~~~~~~~~~~~~

Instead of defining a space-separated list of executables after the ``command`` key,
they can now be listed with the ``command-chain`` key. For example, the `Atom snap's
core migration <https://github.com/snapcrafters/atom/pull/64/files>`_ includes:

.. code-block:: diff
    :caption: snapcraft.yaml of Atom

      apps:
        atom:
    -     command: bin/launcher ${SNAP}/usr/share/atom/atom
    +     command-chain:
    +       - bin/launcher
    +     command: usr/share/atom/atom
