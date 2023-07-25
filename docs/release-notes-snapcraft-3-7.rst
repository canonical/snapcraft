.. 12509.md

.. _release-notes-snapcraft-3-7:

Release notes: Snapcraft 3.7
============================

These are the release notes for `Snapcraft 3.7 <https://github.com/snapcore/snapcraft/releases/tag/3.7>`__, a significant update to :ref:`Snapcraft 3 <snapcraft-overview>`.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

   ℹ Many of the improvements in this release are thanks to the great work done in collaboration with the attendees of the `2019 Snapcraft Summit <https://snapcraft.io/blog/snapcraft-summit-montreal>`__ that took place in Montreal.

New *core* features
-------------------

Extended build options
~~~~~~~~~~~~~~~~~~~~~~

**build-base**

Prior to the release of Snapcraft 3.7, using the ``base`` keyword within :file:`snapcraft.yaml` to specify a base type for a snap did not take into account the *creation* of bases. Instead, the ``name`` keyword was arbitrarily used to determine the build environment:

.. code:: yaml

   name: core18
   type: base
   # base: is not set elsewhere

The above example uses ``name`` to specify the creation of an Ubuntu 18.04 (core18) based build environment.

This fails if a base has yet to be bootstrapped, or is otherwise unavailable. For example, the following will currently generate a \`launch failed: Unable to find an image matching “core20” error:

.. code:: yaml

   name: core20
   type: base
   # base: is not set elsewhere

In cases like the above, where the base has not yet been bootstrapped, ``build-base`` can be used to explicitly define the base to use for the build environment.

To solve the above issue, for example, use the following:

.. code:: yaml

   name: core20
   type: base
   build-base: core18
   # base: is not set elsewhere

**snapd**

``snapd`` is a new value for *base type*. Its inclusion will help the *snapd* team when using Snapcraft to build their own snap.

Extended metadata
~~~~~~~~~~~~~~~~~

The :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` schema has been extended to support new app properties added to :ref:`snap.yaml <the-snap-format>`, alongside better error handling and schema checks.

-  It’s now possible to specify the *type* string for the following existing options with an unspecified type:

   -  ``stop-command``
   -  ``reload-command``

-  Daemon options:

   -  New daemon type: ``dbus``
   -  ``bus-name`` (for use with ``daemon: dbus``). Uses `regex pattern <https://github.com/snapcore/snapcraft/pull/2627#issuecomment-515550633>`__ found in snapd
   -  ``restart-delay``
   -  ``start-timeout``
   -  ``timer``
   -  ``watchdog-timeout``
   -  daemon dependencies can be specified as well as existing options (``stop-timeout``, ``restart-condition``)

-  ``on-watchdog`` for ``restart-condition``
-  ``autostart`` for apps installing autostart desktop files
-  `regex patterns <https://github.com/snapcore/snapcraft/pull/2627#issuecomment-515550633>`__ added to ``stop-timeout`` (to match introduced timeouts).

Faster LXD build iterations
~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using :ref:`Snapcraft with LXD <build-providers>` and :ref:`iterating over a build <iterating-over-a-build>`, a significant reduction in network overhead has resulted in much faster build times.

This is thanks to *snapd 2.39* supporting API snap retrieval, and is used to avoid a root requirement when adding snaps to the build environment. It means snap don’t need to be re-downloaded as frequently.

Improved missing file experience
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After the :ref:`prime stage <parts-lifecycle>` has completed, and missing dependencies are detected, Snapcraft now lists these as *stage-packages*, rather than as a simple list, for inclusion in :file:`snapcraft.yaml` to hopefully build a functioning snap.

This will be extended in upcoming versions of Snapcraft to take into account plugs using the ``content`` interface.

Plugins
-------

crystal (new plugin)
~~~~~~~~~~~~~~~~~~~~

`Crystal <https://crystal-lang.org/>`__ is a programming language with a similar syntax to Ruby. This plugin was developed by Crystal’s upstream team to work with their recently released `Crystal snap <https://snapcraft.io/crystal>`__.

The following keyword is currently accepted by the plugin:

-  **crystal-channel**: (string) The Snap Store channel to install Crystal from. Default: ``latest/stable``

Brian J. Cardiff, one of Crystal’s developers, attended the 2019 Snapcraft Summit Montréal and wrote an excellent overview of how to use the plugin as part of an event write-up. See `Snapcraft Summit Montréal <https://crystal-lang.org/2019/06/19/snapcraft-summit-montreal.html>`__ for the post.

conda (new plugin)
~~~~~~~~~~~~~~~~~~

`Conda <https://docs.conda.io>`__ is an open source package management system and environment management system that runs on Windows, macOS and Linux. This plugin was developed during the 2019 Snapcraft Summit Montréal with the `Anaconda <https://www.anaconda.com/>`__ developers.

This plugin uses the following plugin-specific keywords:

- **conda-packages** (list of strings) List of *conda* packages to install.
- **conda-python-version** (string) The Python version to use for the *conda* packages. Defaults to the latest supported by `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`__.
- **conda-miniconda-version** (string) The version of `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`__ to bootstrap. Defaults to the latest release.

rust
~~~~

The :ref:`Rust plugin <the-rust-plugin>` has been reviewed by a Rust developer and their suggestions incorporated into this release.

One such improvement is defaulting to use the ``rust-toolchain`` file (if present), unless explicitly overridden by use of ``rust-channel`` or ``rust-revision``.

Rebuilding is now also possible using this plugin.

ant
~~~

The `Ant <https://ant.apache.org/>`__ publisher has released an `Ant snap <https://snapcraft.io/ant>`__ and reviewed the :ref:`Ant plugin <the-ant-plugin>`. Consequently, the Ant plugin has been updated to support the use of this new snap for building Ant-based projects.

The following new keywords are now accepted by the plugin:

-  **ant-channel** (string) If not using the Ant tarball from the Ant archive (see :ref:`ant-version and ant-version-checksum <the-ant-plugin>`, this keyword specifies the channel to use for *ant* in the Snap Store. Default: ``latest/stable``

colcon
~~~~~~

Support for `ROS 2 Dashing Diademata <https://index.ros.org//doc/ros2/Releases/Release-Dashing-Diademata/>`__ was added to the :ref:`colcon <the-colcon-plugin>` plugin in order to support this latest ROS release.

Bug fixes
---------

There have been many bugs fixed in this release. Some of the most significant are as follows:

- improved error handling
- additional AppStream icon extraction scenarios that are now taken into account
- modified handling of in-snap symlinks, specifically to better accommodate the merged ``/usr`` directory scheme
- ``click.prompt`` and ``click.confirm`` expanded to query the existence of tty for stdin.

Full list of changes
--------------------

The full list of features and issues worked on in this release are listed below.

Sergio Schvezov
~~~~~~~~~~~~~~~

-  static: use beta channel for black (`#2606 <https://github.com/snapcore/snapcraft/pull/2606>`__)
-  catkin spread tests: dump apt-config on failures for legacy (`#2610 <https://github.com/snapcore/snapcraft/pull/2610>`__)
-  rust plugin: use toml to dump the config (`#2611 <https://github.com/snapcore/snapcraft/pull/2611>`__)
-  rust plugin: use rust-toolchain by default if present (`#2613 <https://github.com/snapcore/snapcraft/pull/2613>`__)
-  conda plugin: new plugin (`#2608 <https://github.com/snapcore/snapcraft/pull/2608>`__)
-  build providers: support injection for LXD (`#2621 <https://github.com/snapcore/snapcraft/pull/2621>`__)
-  schema: remove support for os when using bases (`#2626 <https://github.com/snapcore/snapcraft/pull/2626>`__)
-  appstream extractor: skip non icon file paths (`#2630 <https://github.com/snapcore/snapcraft/pull/2630>`__)
-  spread tests: enable LXD build provider tests (`#2631 <https://github.com/snapcore/snapcraft/pull/2631>`__)
-  build environment: detect base type and use name as base
-  plugins: use get_build_base to determine base support
-  project: add support for build-base
-  repo: add support for querying file ownership
-  pluginhandler: suggest stage-packages for missing DT_NEEDED
-  tests: add python3-toml for autopkgtests
-  spread tests: limit conda plugin to non autopkgtests x86-64 systems
-  spread tests: crystal tests should only run on x86-64

Chris Patterson
~~~~~~~~~~~~~~~

-  black: minor format changes from updated black (`#2603 <https://github.com/snapcore/snapcraft/pull/2603>`__)
-  sources: introduce SnapcraftSourceNotFoundError (`#2604 <https://github.com/snapcore/snapcraft/pull/2604>`__)
-  spread: use more workers to reduce job times
-  catkin/legacy-pull: set test to manual
-  cli: convert users of click.confirm/prompt to echo.confirm/prompt
-  echo: respect SNAPCRAFT_HAS_TTY for is_tty_connected()
-  ant plugin: switch to using ant snap for building (by default)
-  general spread tests: set base for cwd test (`#2618 <https://github.com/snapcore/snapcraft/pull/2618>`__)
-  errors: refactor exception/error handling (`#2602 <https://github.com/snapcore/snapcraft/pull/2602>`__)
-  tests/unit/pluginhandler: introduce tests to repro symlink preservation bug
-  file_utils/create_similar_directory: drop follow_symlinks option
-  pluginhandler: honour symlink directory paths for filesets (LP: #1833408)
-  test_pluginhandler: remove faulty (redundant) tests
-  schema: synchronizing snapd supported schema to snapcraft (`#2627 <https://github.com/snapcore/snapcraft/pull/2627>`__)

Brian J. Cardiff
~~~~~~~~~~~~~~~~

-  crystal plugin: new plugin (`#2598 <https://github.com/snapcore/snapcraft/pull/2598>`__)

Mike Miller
~~~~~~~~~~~

-  build providers: enforce well-known temp dir (`#2607 <https://github.com/snapcore/snapcraft/pull/2607>`__) (LP: #1833292)

Pawel Stolowski
~~~~~~~~~~~~~~~

-  schema: allow snapd as snap type (`#2609 <https://github.com/snapcore/snapcraft/pull/2609>`__)

Claudio Matsuoka
~~~~~~~~~~~~~~~~

-  echo: add wrappers for click.prompt() and click.confirm()

Kyle Fazzari
~~~~~~~~~~~~

-  colcon plugin: add support for dashing (`#2593 <https://github.com/snapcore/snapcraft/pull/2593>`__)

Anatoly Techtonik
~~~~~~~~~~~~~~~~~

-  cli: add -h short option for help (`#2527 <https://github.com/snapcore/snapcraft/pull/2527>`__) (LP: #1807423)

Stefan Bodewig
~~~~~~~~~~~~~~

-  use the stable risk level now that ant has been released

Chris MacNaughton
~~~~~~~~~~~~~~~~~

-  rust plugin: add ability to rebuild (`#2620 <https://github.com/snapcore/snapcraft/pull/2620>`__) (LP: #1825858)

Carlo Lobrano
~~~~~~~~~~~~~

-  tools: let environment-setup.sh skip unnecessary steps (`#2625 <https://github.com/snapcore/snapcraft/pull/2625>`__)


