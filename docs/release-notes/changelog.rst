:tocdepth: 2

.. _changelog:

Changelog
*********

Starting with Snapcraft 8.7.0, information about releases can be found in the
:ref:`release notes<release-notes>`.

..
  release template:

  X.Y.Z (YYYY-MMM-DD)
  -------------------

  Core
  ====

  # for everything related to the lifecycle of packing a snap

  Bases
  #####

  <coreXX>
  """"""""
  (order from newest base to oldest base)

  Plugins
  #######

  <plugin>
  """"""""

  Extensions
  ##########

  <extension>
  """""""""""

  Metadata
  ########

  Sources
  #######

  Components
  ##########

  Command line
  ============

  # for command line and UX changes

  Linter
  ======

  Init
  ====

  Metrics
  =======

  Names
  =====

  Remote build
  ============

  Store
  =====

  Documentation
  =============

  For a complete list of commits, check out the `X.Y.Z`_ release on GitHub.

8.6.3 (2025-Feb-20)
-------------------

Core
====

Plugins
#######

Flutter
"""""""

* Fix a bug where the Flutter plugin would fail to install Flutter for
  ``core22`` and ``core24`` snaps.

For a complete list of commits, check out the `8.6.3`_ release on GitHub.

8.6.2 (2025-Feb-14)
-------------------

Core
====

* Fix a bug where LXD instances would fail during setup due to a race
  condition where the setup began before the instance had started.

* Require Multipass ``1.14.1`` or higher when using Multipass to build
  ``core22`` or ``core24`` snaps.

For a complete list of commits, check out the `8.6.2`_ release on GitHub.

8.6.1 (2025-Feb-10)
-------------------

Core
====

Plugins
#######

uv
""

* Fix a bug where a classic snap using the
  :ref:`uv plugin<craft_parts_uv_plugin>` would fail to find the Python
  interpreter included in the snap itself.

Remote build
============

* Fix a bug where the source code, build recipes, and build logs for remote
  builds of private Launchpad projects would be publicly available on
  Launchpad while the build was in progress.

For a complete list of commits, check out the `8.6.1`_ release on GitHub.

8.6.0 (2025-Feb-05)
-------------------

Core
====

.. note::

  Unless stated otherwise, the changes in this section apply to snaps using
  ``core22`` or ``core24`` bases.

* Add ``chisel`` as a ``build-snap`` if any part has slices and ``chisel``
  isn't already listed as a ``build-snap``.

Plugins
#######

Ant
"""

* Set ``JAVA_HOME`` environment variable to allow the
  :ref:`Ant plugin <craft_parts_maven_plugin>` to detect the latest available
  JDK.

Go Use
""""""

* Add the new :ref:`Go Use plugin<craft_parts_go_use_plugin>` for setting up a
  `workspace <https://go.dev/ref/mod#workspaces>`_ for Go modules.

Maven
"""""

* Set ``JAVA_HOME`` environment variable to allow the
  :ref:`Maven plugin <craft_parts_maven_plugin>` to detect the latest available
  JDK.

Rust
""""

* Fix dependency validation for the ``rust`` plugin when a ``rust-deps``
  part exists (`canonical/craft-parts#890`_).

Poetry
""""""

* Add new ``poetry-export-extra-args`` and ``poetry-pip-extra-args`` keys
  (`canonical/craft-parts#932`_).

* Fix an issue where the :ref:`Poetry plugin <craft_parts_poetry_plugin>` would
  try to install Poetry from the package repositories when ``poetry-deps`` was
  declared as a dependency (`canonical/charmcraft#1996`_).

uv
""

* Add the new :ref:`uv plugin<craft_parts_uv_plugin>` for projects using
  `uv <https://docs.astral.sh/uv/>`_.

Extensions
##########

KDE Neon 6
""""""""""

* Add support for ``core24`` snaps.

* Fix a bug where themes defined in ``$SNAP_REAL_HOME/.config/kdeglobals``
  wouldn't be loaded.

ROS 2 Jazzy
"""""""""""

* Remove the experimental flag from the ROS 2 Jazzy extension.

Command line
============

* Show error details in all verbosity modes except ``quiet``.
* Improve error presentation when the snap fails to pack.
* Improve error presentation when a step or an override script fails
  by splitting stderr and stdout.

Store
=====

* Remove the ``list-registries`` and ``edit-registries`` commands. In a future
  release, these commands will be replaced with ``list-confdbs`` and
  ``edit-confdbs`` (`#5139`_).
* Fix a bug where snaps could not be uploaded on some ARM64 and AMD64
  systems (`#5132`_).

Documentation
=============

* Add a :ref:`Snapcraft quickstart guide <how-to-set-up-snapcraft>`.

* Add a :ref:`how-to guide <how-to-select-a-build-provider>` for selecting a build
  provider.

* Add :ref:`how-to guides <how-to-integrations>` for crafting snaps for common
  languages, frameworks, and meta build systems.

* Update the :doc:`component documentation </reference/components>` to include
  component types.

* Add the following reference documentation:

  * :ref:`reference-anatomy-of-snapcraft-yaml`
  * :ref:`reference-snap-build-process`
  * :ref:`reference-snap-publishing-process`
  * :ref:`reference-channels`
  * :ref:`reference-system-requirements`
  * :ref:`uv plugin <craft_parts_uv_plugin>`
  * :ref:`Go Use plugin <craft_parts_go_use_plugin>`

For a complete list of commits, check out the `8.6.0`_ release on GitHub.

8.5.1 (2024-Dec-17)
-------------------

Core
====

* Improve validation of ``app`` commands in a ``snapcraft.yaml`` file.
  This includes ``command``, ``stop-command``, ``post-stop-command``,
  ``reload-command``, and ``bus-name`` (`#4999`_).

Command line
============

* Fix crash in command line help when positional arguments exist with
  no associated help (`#5169`_).

For a complete list of commits, check out the `8.5.1`_ release on GitHub.

8.5.0 (2024-Nov-19)
-------------------

Core
====

* Remove the Snapcraft Dockerfile in favor of the `snapcraft-rocks`_ registry.
  For more information, see the ``docker/README.md`` file.

Bases
#####

core24
""""""

* Add a ``--platform <platform>`` argument to the ``clean`` command to filter
  which build environments to clean (`canonical/craft-application#425`_).

Plugins
#######

Poetry
""""""

* Add a new plugin for projects using `Poetry <https://python-poetry.org/>`_.
  For more information, see :doc:`Poetry plugin
  reference</reference/plugins/poetry_plugin>`.

Extensions
##########

Env-injector
""""""""""""

* Add a new experimental extension called
  `env-injector <https://github.com/canonical/snappy-env>`_. It simplifies
  passing environment variables to snaps at runtime using ``snap config`` or a
  ``.env`` file. For more information, see `the env-injector
  extension <https://forum.snapcraft.io/t/the-env-injector-extension/41477>`_

Command line
============

* Order lifecycle commands in the sequence they run rather than alphabetically
  in help messages.

* Show positional arguments in help messages.

* Hide the terminal cursor when Snapcraft is running.

* Fix a bug where the version and path to the log file were logged multiple
  times.

* Fix a bug where deprecated commands were suggested when an invalid command
  was entered (`canonical/craft-cli#297`_).

Init
====

* Add a ``<project-dir>`` argument to initialize the project in a particular
  directory.

* Add a ``--name <name>`` argument to set the ``name`` key in the initialized
  ``snapcraft.yaml``. If ``--name`` isn't provided, the name of the project
  directory or current working directory is used.

* Add a ``--profile <profile>`` argument to initialize a project for a specific
  purpose. Currently, only the ``simple`` profile is supported.

Store
=====

* Add support for ``registries`` with new commands ``list-registries`` and
  ``edit-registries``.

* Add support for specifying components in a validation set when using the
  ``edit-validation-sets`` command.

Documentation
=============

* Add :doc:`reference</reference/linters>` for linters.

* Update the ``HACKING`` doc with instructions for how to use ``tox``.

* Hide subheadings in the changelog's table of contents.

* Add :doc:`reference</reference/plugins/poetry_plugin>` for
  the Poetry plugin.

* Fix an issue where the auto-generated command reference wasn't rendered
  correctly.

For a complete list of commits, check out the `8.5.0`_ release on GitHub.


8.4.4 (2024-Oct-24)
-------------------

Core
====

Bases
#####

core24
""""""

* Fix a bug where ``CRAFT_PARALLEL_BUILD_COUNT`` was not evaluated in
  ``snapcraft.yaml`` files (`#4785`_).

* Fix a bug where ``package-repositories`` declarations could conflict with
  default sources present in Noble (`LP#2083013`_).

For a complete list of commits, check out the `8.4.4`_ release on GitHub.

.. _changelog-7-5-8:

7.5.8 (2024-Oct-24)
-------------------

Core
====

* Fix a regression where Snapcraft would fail to run on some architectures due
  to a ``cryptography`` dependency that attempted to load legacy algorithms
  (`#5077`_).

For a complete list of commits, check out the `7.5.8`_ release on GitHub.


8.4.3 (2024-Oct-10)
-------------------

Core
====

Components
##########

* Add support for component types ``kernel-modules`` and ``standard``
  (`#5089`_).

For a complete list of commits, check out the `8.4.3`_ release on GitHub.

8.4.2 (2024-Oct-07)
-------------------

Core
====

* Fix a regression where Snapcraft would fail to run on some architectures due
  to a ``cryptography`` dependency that attempted to load legacy algorithms
  (`#5077`_).

* Fix a regression where Snapcraft would fail to run in a container if it was
  not running as a snap (`#5079`_).

* Fix a bug where parallel installations of Snapcraft would not work if the
  Snapcraft snap was installed from the store (`#4683`_, `#4927`_).

Plugins
#######

Python
""""""

* Fix an issue where the ``python`` plugin would fail to build if the part
  had no Python scripts.

Remote build
============

* Fix a bug where the remote builder would ignore the user's response when a
  build is interrupted and always clean the launchpad project (`#4929`_).

Documentation
=============

* Update Rust plugin doc with recent changes to the Rust toolchain.

For a complete list of commits, check out the `8.4.2`_ release on GitHub.


7.5.7 (2024-Oct-03)
-------------------

Core
====

* Fix a bug where parallel installations of Snapcraft would not work if the
  Snapcraft snap was installed from the store (`#4683`_, `#4927`_).

For a complete list of commits, check out the `7.5.7`_ release on GitHub.


8.4.1 (2024-Sep-20)
-------------------

Core
====

* Fix a regression where numeric entries in ``snapcraft.yaml`` couldn't be
  parsed.

Bases
#####

core24
""""""

* Fix a regression where ``build-for`` couldn't be omitted in a ``platforms``
  entry in a ``snapcraft.yaml`` file.

* Fix a regression where ``--shell`` and ``--shell-after`` weren't supported
  for the ``pack`` command (`#4963`_).

* Fix a regression where ``--debug`` wouldn't open a shell into the build
  environment if the packing step fails (`#4959`_).

Plugins
#######

NPM
"""

* Fix a bug where NPM parts fail to build if the ``pull`` and ``build`` steps
  didn't occur in the same instance of Snapcraft.

Command line
============

* Fix a regression where store errors would be raised as an internal error
  (`#4930`_).

* Add documentation links for error messages about using an ESM base.

Remote build
============

* Fix a regression where ``--build-for`` and ``--platform`` couldn't accept
  comma-separated values (`#4990`_).

* Fix a regression where remote build errors would be raised as an internal
  error (`#4908`_).

* Add documentation links and recommended resolutions to remote-build errors.

Store
=====

* Fix a regression where Ubuntu One macaroons couldn't be refreshed
  (`#5048`_).

For a complete list of changes, check out the `8.4.1`_ release on GitHub.


8.3.4 (2024-Sep-13)
-------------------

Core
====

Plugins
#######

NPM
"""
* Fix a bug where NPM parts fail to build if the ``pull`` and ``build`` steps
  did not occur in the same execution of Snapcraft.

For a complete list of commits, check out the `8.3.4`_ release on GitHub.


8.4.0 (2024-Sep-10)
-------------------

.. note::

   8.4.0 includes changes from the :ref:`7.5.6<7.5.6_changelog>` release.

Core
====

* Fix a bug where Snapcraft would fail to inject itself into the build
  environment when not running as a snap (`canonical/charmcraft#406`_). If an
  app isn't running from snap, the installed app will now install the snap in
  the build environment using the channel in the ``CRAFT_SNAP_CHANNEL``
  environment variable, defaulting to ``latest/stable`` if none is set.

* Fix a regression where icons wouldn't be configured and installed for snaps
  with no ``apps`` defined in their ``snapcraft.yaml``.

Bases
#####

core24
""""""

* Raise an error if the build plan is empty and no snaps will be built
  (`canonical/craft-application#225`_).

* Fix a regression where ``https_proxy``, ``https_proxy``, and ``no_proxy``
  were not forwarded into the build environment.

Plugins
#######

* Fix a bug where ``snapcraft list-plugins --base core20`` would fail in a
  ``core24`` project directory (`#5008`_).

Components
##########

* Allow numbers and hyphens in component names (`LP#2069783`_).

* Fix a bug where ``stage-packages`` can't be used when components are defined
  (`canonical/craft-parts#804`_).

Command line
============

* Improve error messages when parsing a ``snapcraft.yaml`` file (`#4941`_).

* Improve error messages when using an ESM base.

* Improve error messages for missing files (`canonical/craft-parts#802`_).

* Improve error messages when a build fails because it matches multiple
  platforms (`canonical/craft-application#382`_).

* Fix a bug where multi-line error messages would overwrite the previous line
  (`canonical/craft-cli#270`_).

Remote build
============

* Add "Pending" status for queued remote builds.

* Add documentation links to remote-build errors.

* Improve error messages when multiple snaps can be built on a single
  ``build-on`` architecture (`#4995`_).

* Improve error messages when using the wrong remote builder.

* Fix a regression where ``--platform`` or ``--build-for`` could be used when
  ``platforms`` or ``architectures`` were defined in the ``snapcraft.yaml``
  file (`#4881`_).

* Fix a regression where ``--platform`` could be used for ``core22`` snaps
  (`#4881`_).

* Fix a bug where ``SNAPCRAFT_REMOTE_BUILD_STRATEGY`` would be validated when
  running commands other than ``remote-build``.

* Fix a bug where ``SNAPCRAFT_REMOTE_BUILD_STRATEGY`` was ignored for
  ``core24`` snaps.

Documentation
=============

* Add changelog notes for all Snapcraft 8.x releases

* Add :ref:`reference <reference-components>`,
  :ref:`explanation <explanation-components>`, and
  :ref:`how-to guide <how-to-create-a-component>` for components.

* Add :ref:`reference <reference-bases>`,
  :ref:`explanation <explanation-bases>`, and
  :ref:`how-to guide <how-to-specify-a-base>` for bases.

For a complete list of commits, check out the `8.4.0`_ release on GitHub.


8.3.3 (2024-Aug-28)
-------------------

Core
====

* Improve detection and error messages when LXD is not installed or not
  properly enabled.

Bases
#####

core24
""""""

* Require Multipass >= ``1.14.1`` when using Multipass to build ``core24``
  snaps.

For a complete list of commits, check out the `8.3.3`_ release on GitHub.


.. _7.5.6_changelog:

7.5.6 (2024-Aug-15)
-------------------

Core
====

Bases
#####

core22
""""""

* Fix a regression where icons would not be configured and installed for snaps
  with no ``apps`` defined in their ``snapcraft.yaml``.

For a complete list of commits, check out the `7.5.6`_ release on GitHub.


8.3.2 (2024-Aug-05)
-------------------

Core
====

Bases
#####

core24
""""""

* Fix a bug where classic snaps with a Python virtual environment would attempt
  to use the system's Python interpreter (`#4942`_).

Plugins
#######

Kernel
""""""

* Fix a bug where removing a missing symlink would cause the kernel plugin
  to fail.

Store
=====

* Fix a bug where ``edit-validation-sets`` would fail when editing a validation
  sets with snap revisions (`#4909`_).

For a complete list of commits, check out the `8.3.2`_ release on GitHub.


8.3.1 (2024-Jul-08)
-------------------

Core
====

Bases
#####

core24
""""""

* Support ``all`` as a target with ``build-for: [all]`` (`#4854`_).

* Ensure Craft Providers provider (LXD or Multipass) is available before
  launching a build environment.

* Improve presentation of ``snapcraft.yaml`` model errors.

Metadata
########

* Validate that ``update_contact``, ``donation``, ``vcs-browser``,
  ``bugtracker``, and ``homepage`` fields adopted from an appstream metadata
  file are valid URLs or email addresses.

* Ensure that ``contact``, ``donation``, ``source-code``, ``issues``, and
  ``website`` fields in a snapcraft.yaml take priority over appstream metadata
  (`#4890`_).

Remote build
============

* Require ``core20`` snaps to use the legacy remote builder (`#4886`_).

* Allow building ``core22`` snaps with ``build-for: [all]``.

* Support reading Launchpad credentials from the previous location (`#4889`_).
  If launchpad credentials do not exist in the new location
  (``$XDG_DATA_DIR/snapcraft/launchpad-credentials``) introduced in ``8.2.0``,
  then load credentials from the previous location
  (``$XDG_DATA_DIR/snapcraft/provider/launchpad/credentials``) and emit a
  deprecation notice.

.. note::

   This behavior applies to the new and legacy remote builders.

Documentation
=============

* Add an :ref:`explanation <explanation-architectures>` for the remote builders
  (`#4842`_).

* Update :ref:`reference <reference-architectures>`, :ref:`how-to guide
  <how-to-select-architectures>`, and :ref:`explanation <explanation-architectures>` for
  platforms and architectures.

For a complete list of commits, check out the `8.3.1`_ release on GitHub.


8.3.0 (2024-Jun-27)
-------------------

Core
====

* Improve logging to show which package is being fetched.

* Add support for parts to source ``7z`` archives.

* Improve error messages when sources cannot be fetched.

Bases
#####

core24
""""""

* Add support for ``core24-desktop`` snaps (`#4818`_).

core22
""""""

* Warn when multiple snaps are going to be built in destructive mode because
  it may cause unexpected behavior (`#4685`_, `#4356`_).

* Fix a regression where ``core22-desktop`` could not be built (`#4818`_).

Plugins
#######

Flutter
"""""""

* Add ``curl`` as a ``build-package`` for ``flutter`` parts (`#4804`_).

ROS 2 Jazzy
"""""""""""

* Add support for the new ROS 2 Jazzy extension which lets you snap ROS 2
  applications on ``core24`` (`#4791`_).

* Similar to ROS 2 Humble for ``core22``, content-sharing is supported
  (`#4828`_).

For more information, see :ref:`reference-ros-2-extensions` and
:ref:`reference-ros-2-content-extensions`.

NPM
"""

Various improvements for the ``core22`` and ``core24`` NPM plugins:

* Accept NVM-style version identifiers for ``npm-node-version``.

* Verify SHA256 checksums after node.js download
  (`canonical/craft-parts#717`_).

* Use new-style ``npm-install`` commands if the npm version is newer than
  ``8.x``.

* Set ``NODE_ENV`` to ``production`` by default.

List plugins
""""""""""""

* Fix a bug where ``snapcraft list-plugins`` would fail to run in a ``core24``
  project directory (`#4830`_).

* Update ``snapcraft list-plugins`` to show a list of ``core24`` plugins
  instead of ``core22`` plugins when not in a project directory (`#4830`_).

Extensions
##########

Gnome
"""""

* Make gnome extension stable for ``core24``.

* Fix ``GI_TYPELIB_PATH`` and ``XDG_DATA_DIRS`` paths in the build environment
  (`#4798`_).

* Integrate with the ``gpu-2404`` SDK (`#4744`_).

For more information, see the `gpu 2404 interface docs`_.

KDE Neon 6
""""""""""

* Fix paths to ``QtWebEngineProcess`` in the desktop launcher (`#4745`_).

Expand extensions
"""""""""""""""""

* Fix a bug where ``snapcraft expand-extensions`` could not parse a
  ``snapcraft.yaml`` file containing the ``platforms`` keyword.

Components
##########

* Include the ``provenance`` keyword in a component's metadata from a
  ``snapcraft.yaml`` file (`#4827`_).

Metadata
########

Add support for adopting more metadata fields from a project's appstream file:

* ``license``
* ``contact``
* ``source-code``
* ``issues``
* ``websites``
* ``donations``

Metrics
#######

* Add support for ``snapcraft metrics`` to retrieve the metrics
  ``installed_base_by_architecture`` and
  ``weekly_installed_base_by_architecture`` (`#4735`_).

Names
#####

* Add output formatting to ``snapcraft names`` with ``--format``. Supported
  formats are ``table`` and ``json`` (`#4778`_).

Init
####

* Update ``snapcraft init`` to create a ``core24`` project instead of a
  ``core22`` project (`#4830`_)

Documentation
#############

* Update Snapcraft's documentation to use the `canonical-sphinx`_ theme.

Add reference documentation for more plugins (`#4811`_):

* ``ant``
* ``autotools``
* ``cmake``
* ``dotnet``
* ``go``
* ``make``
* ``meson``
* ``nil``
* ``npm``
* ``qmake``
* ``scons``

For a complete list of commits, check out the `8.3.0`_ release on GitHub.

8.2.12 (2024-Jun-12)
--------------------

Core
====

Bases
#####

core24
""""""

* Fix a bug where snaps would stage Python packages already included in the
  ``core24`` base snap (`#4865`_).

Store
=====

* Fix a bug where store-related error messages would be presented as an
  internal Snapcraft error.

* Add a resolution and link to documentation for keyring errors.

Documentation
=============

* Fix Snapcraft's version in the readthedocs documentation.

For a complete list of commits, check out the `8.2.12`_ release on GitHub.

8.2.11 (2024-Jun-12)
--------------------

Core
====

Plugins
#######

Dotnet
""""""

* Fix a regression where the ``dotnet`` plugin could not be used for
  ``core22`` snaps (`#4825`_).

For a complete list of commits, check out the `8.2.11`_ release on GitHub.

8.2.10 (2024-Jun-03)
--------------------

Remote builder
==============

* Fix a bug where comma-separated architectures in ``--build-for`` could not
  be parsed (`#4780`_).

* Fix a bug where ``core22`` snaps with a top level ``architectures`` keyword
  could not be parsed (`#4780`_).

* Fix a bug where remote build log files were incorrectly named (`#4781`_).

* Retry more API calls to Launchpad (`canonical/craft-application#355`_).

* Add an exponential backoff to API retries with a maximum total delay of
  62 seconds (`canonical/craft-application#355`_).

* Fix a bug where the remote builder would not fail if no artifacts were
  created (`#4783`_).

For a complete list of commits, check out the `8.2.10`_ release on GitHub.

8.2.9 (2024-May-28)
-------------------

Core
====

Extensions
##########

KDE Neon 6
""""""""""

* Fix multiple issues to allow web processes to work correctly (`#4823`_).

* Expose the ``libplas`` and ``liblapack`` provided by the ``kf6-core22{-sdk}``
  snaps (`#4823`_).

For a complete list of commits, check out the `8.2.9`_ release on GitHub.

8.2.8 (2024-May-17)
-------------------

Core
====

Bases
#####

core24
""""""

* Fix a behavior where shared libraries from the host were loaded for
  classically confined snaps.

.. note::

   This is implemented with ``patchelf --no-default-lib`` when
   ``enable-patchelf`` is defined.

Plugins
#######

Dotnet
""""""

* Disable the ``dotnet`` plugin for ``core24`` snaps due to a pending rewrite.

For a complete list of commits, check out the `8.2.8`_ release on GitHub.

8.2.7 (2024-May-09)
-------------------

Core
====

* Add support for ``ignore-running`` in ``apps.<app-name>.refresh-mode`` in a
  ``snapcraft.yaml`` file (`#4747`_).

Remote build
============

* Fix a regression where remote build would fail to parse some
  ``architectures`` definitions (`#4780`_).

For a complete list of commits, check out the `8.2.7`_ release on GitHub.

8.2.6 (2024-May-09)
-------------------

Core
====

* Fix a regression where a directory could not be packaged as a snap
  (`#4769`_).

For a complete list of commits, check out the `8.2.6`_ release on GitHub.

8.2.5 (2024-May-07)
-------------------

Store
=====

* Fix the same ``cryptography`` regression addressed in ``8.2.4`` but for
  store-related operations.

For a complete list of commits, check out the `8.2.5`_ release on GitHub.

8.2.4 (2024-May-05)
-------------------

* Fix a regression where Snapcraft would fail to run on some architectures due
  to a ``cryptography`` dependency that attempted to load legacy algorithms
  (`LP#2064639`_).

For a complete list of commits, check out the `8.2.4`_ release on GitHub.

8.2.3 (2024-May-01)
-------------------

Core
====

Bases
#####

core24
""""""

* Fix a bug where project variables were evaluated before extensions were
  applied (`#4771`_).

* Fix a bug where ``build-for`` project variables were evaluated based on the
  host architecture (`#4770`_).

For a complete list of commits, check out the `8.2.3`_ release on GitHub.

8.2.2 (2024-Apr-30)
-------------------

Core
====

Bases
#####

core24
""""""

* Fix a bug where advanced grammar could not be combined with other data
  (`#4764`_, `LP#2061603`_).

For a complete list of commits, check out the `8.2.2`_ release on GitHub.

8.2.1 (2024-Apr-25)
-------------------

Core
====

Bases
#####

core24
""""""

* Fix a bug where :ref:`part environment variables
  <reference-part-environment-variables>` were not evaluated inside a ``snapcraft.yaml``
  file and were not available as environment variables in the build environment.

* Fix a bug where :ref:`advanced grammar <reference-advanced-grammar>` was not evaluated
  in root-level part keywords ``build-packages`` and ``build-snaps``.

* Fix a bug where local key assets in ``snap/keys/`` were not used when
  installing package repositories.

Remote build
============

* Fix a bug where ``core24`` snaps could not use package repositories
  because ``gpg`` and ``dirmngr`` were not installed in the remote build
  environment.

For a complete list of commits, check out the `8.2.1`_ release on GitHub.

8.2.0 (2024-Apr-17)
-------------------

Core
====

Bases
#####

core24
""""""

* Drop requirement for ``build-base: devel`` for ``core24`` snaps.

core22
""""""

* Extend :ref:`advanced grammar <reference-advanced-grammar>` for all part keywords
  except plugin-specific keywords.

Remote build
============

* Migrate to the upstream remote builder in `Craft Application`_.

* Allow only one remote build is allowed per project.

* Remove support for ``build-id`` with ``snapcraft remote-build --recover``.

* Remove support for deprecated ``--build-on`` argument in favor of
  ``--build-for``.

* Move Launchpad credentials file from
  ``$XDG_DATA_DIR/snapcraft/provider/launchpad/credentials``
  to ``$XDG_DATA_DIR/snapcraft/launchpad-credentials``.

* Fail if snapcraft is in a shallowly-cloned git repository instead of falling
  back to the legacy remote builder.

.. note::

  Reminder: Legacy remote-build behavior can be used for bases core22 and older
  with the environment variable
  ``SNAPCRAFT_REMOTE_BUILD_STRATEGY="force-fallback"``. See more information in
  the :doc:`remote build</explanation/remote-build>` documentation.

For a complete list of commits, check out the `8.2.0`_ release on GitHub.

8.1.0 (2024-Apr-10)
-------------------

Core
====

Bases
#####

core24
""""""

* Finalize internal refactor to use `Craft Application`_ to build ``core24``
  snaps.

For more information on deprecations and changes, see the :ref:`core24 migration guide
<how-to-change-from-core22-to-core24>`.

Plugins
#######

Matter SDK
""""""""""

* Add new Matter SDK plugin for ``core22``.

For more information, see the `Matter`_ website and the `Matter on Ubuntu`_
docs.

Maven
"""""

* Add support for the Maven plugin for ``core22`` snaps.

For more information, see :doc:`/reference/plugins/maven_plugin`.

QMake
"""""

* Add support for the QMake plugin for ``core22`` snaps.

For more information, see :ref:`craft_parts_qmake_plugin`.

Colcon
""""""

* Set build type to ``RELEASE`` if it is not defined by ``colcon_cmake_args:
  ["-DCMAKE_BUILD_TYPE=<build type>"]``).

Extensions
##########

KDE Neon 6
""""""""""

* Add new ``kde-neon-6`` extension for ``core22`` snaps that use Qt6 or the
  KDE Neon 6 framework.

Components
##########

* Add support for creating components.

* Components are parts of a snap that can be built and uploaded in
  conjunction with a snap and later optionally installed beside it.

For more information, see the :ref:`reference <reference-components>`, :ref:`explanation
<explanation-components>`, and :ref:`how-to guide <how-to-create-a-component>`
documentation pages.

Remote build
============

* Add support for user-defined Launchpad projects projects, including
  private projects.

* This is configured via ``snapcraft remote-build --project <project-name>``.

For a complete list of commits, check out the `8.1.0`_ release on GitHub.

8.0.5 (2024-Mar-18)
-------------------

Core
====

* Fix a bug where LXD versions with an "LTS" suffix could not be parsed.

For a complete list of commits, check out the `8.0.5`_ release on GitHub.

8.0.4 (2024-Mar-04)
-------------------

Core
====

Bases
#####

* Fix a bug where ``devel`` bases may not be fully validated.

* Bump the LXD compatibility tag to ``v7``.

core24
""""""

* Use ``buildd`` daily images instead of ``ubuntu`` images for ``core24``
  bases and ``build-base: devel``.

* Fix a bug where creating ``core24`` base images would fail because ``apt``
  would install packages interactively.

For a complete list of commits, check out the `8.0.4`_ release on GitHub.

8.0.3 (2024-Feb-09)
-------------------

Core
====

* Add a warning that when a part uses ``override-prime`` it cannot use
  ``enable-patchelf`` (`#4547`_).

Bases
#####

* Bump the LXD compatibility tag to ``v6``.

* Stop updating ``apt`` source config files when ``build-base: devel``
  is defined.

core24
""""""

* Use the ``core24`` alias instead of the ``devel`` alias when retrieving LXD
  images.

Plugins
#######

Ant
"""

* Use the proxy environment variables ``http_proxy`` and ``https_proxy``.

Remote build
============

* Fix a bug where ``--build-for`` and ``--build-on`` were not mutually
  exclusive options.

* Improve error messages and provide links to documentation when remote builds
  fail (`#4517`_).

* Fix a regression where comma-separated architectures in ``--build-on`` and
  ``--build-for`` were not accepted (`#4516`_).

For a complete list of commits, check out the `8.0.3`_ release on GitHub.

8.0.2 (2024-Jan-23)
-------------------

Core
====

* Fix a bug where Snapcraft fails to run on platforms where ``SSL_CERT_DIR`` is
  not set (`#4510`_, `#4520`_).

* Fix a decoding bug when logging malformed output from other processes,
  typically during the ``build`` step (`#4515`_).

For a complete list of commits, check out the `8.0.2`_ release on GitHub.

8.0.1 (2024-Jan-03)
-------------------

Remote build
============


* Fix a bug where Snapcraft would not fail if the Launchpad build itself failed
  for new and legacy remote builders (`#4142`_).

* Fix a bug where large repos could not be pushed with the new remote builder
  (`#4478`_).

* Fallback to the legacy remote builder if the project is shallowly cloned
  (`#4479`_).

For a complete list of commits, check out the `8.0.1`_ release on GitHub.

8.0.0 (2023-Dec-04)
-------------------

Core
====

Bases
#####

core22
""""""

Add new environment variables for ``build-on`` and ``build-for`` architectures:

* ``CRAFT_ARCH_TRIPLET_BUILD_FOR``, supersedes ``CRAFT_ARCH_TRIPLET``
* ``CRAFT_ARCH_TRIPLET_BUILD_ON``
* ``CRAFT_ARCH_BUILD_FOR``, supersedes ``CRAFT_TARGET_ARCH``
* ``CRAFT_ARCH_BUILD_ON``

For more information, see :doc:`/reference/architectures`.

core20
""""""

Add new environment variables for ``build-on`` and ``build-for`` architectures:

* ``SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR``, supersedes ``SNAPCRAFT_ARCH_TRIPLET``
* ``SNAPCRAFT_ARCH_TRIPLET_BUILD_ON``
* ``SNAPCRAFT_ARCH_BUILD_FOR``, supersedes ``SNAPCRAFT_TARGET_ARCH``
* ``SNAPCRAFT_ARCH_BUILD_ON``

For more information, see :doc:`/reference/architectures`.

core18
""""""

* Deprecate building snaps using the ``core18`` base.

For more information on how to continue building snaps with the ``core18``
base, see :ref:`this page<how-to-deprecated-base>`.

Stage packages
##############

* Support chiseled ``stage-packages``. This is useful for reducing the size of
  the snap when creating :ref:`base snaps<base-snap-reference>` or using a bare
  base.

For more information about chisel, see https://github.com/canonical/chisel

Plugins
#######

Rust
""""

* Use default rust toolchain with ``rustup``.

* Add option ``rust-ignore-toolchain-file``.

* Add option ``rust-inherit-ldflags``.

* Add list ``rust-cargo-parameters``.

For more information about the new options, see
:doc:`/common/craft-parts/reference/plugins/rust_plugin`.

Kernel
""""""

* Generate kernel configs for Ubuntu 22.04 (Jammy).

Python
""""""

* Add support for Python projects driven by a ``pyproject.toml``.

For more information, see the `PEP 518`_ spec.

ROS 2
"""""

* Add support for content sharing for core20 & core22 bases (ROS Noetic, Foxy,
  Humble) and the ``colcon``, ``catkin``, and ``catkin-tools`` plugins

For more information on ROS architecture, see the `ROS architectures with
snaps`_.

More information on content-sharing, see:

* :ref:`reference-ros-2-extensions`
* :ref:`reference-ros-2-content-extensions`
* :ref:`reference-ros-1-content-extensions`

Command line
============

* Stream messages in the default ``brief`` mode

* Improve presentation of build step prefixes

Linter
======

* Suggest packages to add to ``stage-packages`` to satisfy a potential missing
  library.

Remote build
============

Introduce a new remote-builder for ``core24`` snaps:

* Does not modify the project's ``snapcraft.yaml``

* Does not fetch and tarball remote sources before sending the project
  to Launchpad

* Require projects to be in the top-level of a fully-cloned (non-shallow) git
  repository

* Allow switching between the new and legacy remote builders with
  the environment variable ``SNAPCRAFT_REMOTE_BUILD_STRATEGY``.

For more information on the new remote-builder, how to switch between the
new and legacy remote builders, see :doc:`/explanation/remote-build`.

Store
=====

* Add a fallback to a file-based keyring when the system keyring cannot be
  initialized, is not fully configured, or is otherwise not available.

For more information on the file-based keyring, see :ref:`how-to-authenticate`.

For a complete list of commits, check out the `8.0.0`_ release on GitHub.


.. _canonical-sphinx: https://github.com/canonical/canonical-sphinx
.. _Craft Application: https://github.com/canonical/craft-application
.. _gpu 2404 interface docs: https://mir-server.io/docs/the-gpu-2404-snap-interface#heading--consuming-the-interface
.. _Matter: https://csa-iot.org/all-solutions/matter/
.. _Matter on Ubuntu: https://canonical-matter.readthedocs-hosted.com/en/latest/
.. _Releases page: https://github.com/canonical/snapcraft/releases
.. _PEP 518: https://peps.python.org/pep-0518/
.. _ROS architectures with snaps: https://ubuntu.com/robotics/docs/ros-architectures-with-snaps.
.. _snapcraft-rocks: https://github.com/canonical/snapcraft-rocks/pkgs/container/snapcraft

.. _canonical/charmcraft#406: https://github.com/canonical/charmcraft/issues/406
.. _canonical/charmcraft#1996: https://github.com/canonical/charmcraft/issues/1996
.. _canonical/craft-application#225: https://github.com/canonical/craft-application/pull/225
.. _canonical/craft-application#355: https://github.com/canonical/craft-application/pull/355
.. _canonical/craft-application#382: https://github.com/canonical/craft-application/pull/382
.. _canonical/craft-application#425: https://github.com/canonical/craft-application/pull/425
.. _canonical/craft-cli#270: https://github.com/canonical/craft-parts/issues/270
.. _canonical/craft-cli#297: https://github.com/canonical/craft-parts/issues/297
.. _canonical/craft-parts#717: https://github.com/canonical/craft-parts/issues/717
.. _canonical/craft-parts#802: https://github.com/canonical/craft-parts/issues/802
.. _canonical/craft-parts#804: https://github.com/canonical/craft-parts/issues/804
.. _canonical/craft-parts#890: https://github.com/canonical/craft-parts/issues/890
.. _canonical/craft-parts#932: https://github.com/canonical/craft-parts/issues/932

.. _LP#2061603: https://bugs.launchpad.net/snapcraft/+bug/2061603
.. _LP#2064639: https://bugs.launchpad.net/snapcraft/+bug/2064639
.. _LP#2069783: https://bugs.launchpad.net/snapcraft/+bug/2069783
.. _LP#2083013: https://bugs.launchpad.net/snapcraft/+bug/2083013

.. _#4142: https://github.com/canonical/snapcraft/issues/4142
.. _#4356: https://github.com/canonical/snapcraft/issues/4356
.. _#4478: https://github.com/canonical/snapcraft/issues/4478
.. _#4479: https://github.com/canonical/snapcraft/issues/4479
.. _#4510: https://github.com/canonical/snapcraft/issues/4510
.. _#4515: https://github.com/canonical/snapcraft/issues/4515
.. _#4516: https://github.com/canonical/snapcraft/issues/4516
.. _#4517: https://github.com/canonical/snapcraft/issues/4517
.. _#4520: https://github.com/canonical/snapcraft/issues/4520
.. _#4547: https://github.com/canonical/snapcraft/issues/4547
.. _#4683: https://github.com/canonical/snapcraft/issues/4683
.. _#4685: https://github.com/canonical/snapcraft/issues/4685
.. _#4735: https://github.com/canonical/snapcraft/issues/4735
.. _#4744: https://github.com/canonical/snapcraft/issues/4744
.. _#4745: https://github.com/canonical/snapcraft/issues/4745
.. _#4747: https://github.com/canonical/snapcraft/issues/4747
.. _#4764: https://github.com/canonical/snapcraft/issues/4764
.. _#4769: https://github.com/canonical/snapcraft/issues/4769
.. _#4770: https://github.com/canonical/snapcraft/issues/4770
.. _#4771: https://github.com/canonical/snapcraft/issues/4771
.. _#4778: https://github.com/canonical/snapcraft/issues/4778
.. _#4780: https://github.com/canonical/snapcraft/issues/4780
.. _#4781: https://github.com/canonical/snapcraft/issues/4781
.. _#4783: https://github.com/canonical/snapcraft/issues/4783
.. _#4785: https://github.com/canonical/snapcraft/issues/4785
.. _#4791: https://github.com/canonical/snapcraft/issues/4791
.. _#4798: https://github.com/canonical/snapcraft/issues/4798
.. _#4804: https://github.com/canonical/snapcraft/issues/4804
.. _#4811: https://github.com/canonical/snapcraft/issues/4811
.. _#4818: https://github.com/canonical/snapcraft/issues/4818
.. _#4823: https://github.com/canonical/snapcraft/pull/4823
.. _#4825: https://github.com/canonical/snapcraft/issues/4825
.. _#4827: https://github.com/canonical/snapcraft/issues/4827
.. _#4828: https://github.com/canonical/snapcraft/issues/4828
.. _#4830: https://github.com/canonical/snapcraft/issues/4830
.. _#4842: https://github.com/canonical/snapcraft/issues/4842
.. _#4854: https://github.com/canonical/snapcraft/issues/4854
.. _#4865: https://github.com/canonical/snapcraft/issues/4865
.. _#4881: https://github.com/canonical/snapcraft/issues/4881
.. _#4886: https://github.com/canonical/snapcraft/issues/4886
.. _#4889: https://github.com/canonical/snapcraft/issues/4889
.. _#4890: https://github.com/canonical/snapcraft/issues/4890
.. _#4908: https://github.com/canonical/snapcraft/issues/4908
.. _#4909: https://github.com/canonical/snapcraft/issues/4909
.. _#4927: https://github.com/canonical/snapcraft/issues/4927
.. _#4929: https://github.com/canonical/snapcraft/issues/4929
.. _#4930: https://github.com/canonical/snapcraft/issues/4930
.. _#4941: https://github.com/canonical/snapcraft/issues/4941
.. _#4942: https://github.com/canonical/snapcraft/issues/4942
.. _#4959: https://github.com/canonical/snapcraft/issues/4959
.. _#4963: https://github.com/canonical/snapcraft/issues/4963
.. _#4990: https://github.com/canonical/snapcraft/issues/4990
.. _#4995: https://github.com/canonical/snapcraft/issues/4995
.. _#4999: https://github.com/canonical/snapcraft/issues/4999
.. _#5008: https://github.com/canonical/snapcraft/issues/5008
.. _#5048: https://github.com/canonical/snapcraft/issues/5048
.. _#5077: https://github.com/canonical/snapcraft/issues/5077
.. _#5079: https://github.com/canonical/snapcraft/issues/5079
.. _#5089: https://github.com/canonical/snapcraft/issues/5089
.. _#5132: https://github.com/canonical/snapcraft/issues/5132
.. _#5139: https://github.com/canonical/snapcraft/issues/5139
.. _#5169: https://github.com/canonical/snapcraft/issues/5169

.. _7.5.6: https://github.com/canonical/snapcraft/releases/tag/7.5.6
.. _7.5.7: https://github.com/canonical/snapcraft/releases/tag/7.5.7
.. _7.5.8: https://github.com/canonical/snapcraft/releases/tag/7.5.8
.. _8.0.0: https://github.com/canonical/snapcraft/releases/tag/8.0.0
.. _8.0.1: https://github.com/canonical/snapcraft/releases/tag/8.0.1
.. _8.0.2: https://github.com/canonical/snapcraft/releases/tag/8.0.2
.. _8.0.3: https://github.com/canonical/snapcraft/releases/tag/8.0.3
.. _8.0.4: https://github.com/canonical/snapcraft/releases/tag/8.0.4
.. _8.0.5: https://github.com/canonical/snapcraft/releases/tag/8.0.5
.. _8.1.0: https://github.com/canonical/snapcraft/releases/tag/8.1.0
.. _8.2.0: https://github.com/canonical/snapcraft/releases/tag/8.2.0
.. _8.2.1: https://github.com/canonical/snapcraft/releases/tag/8.2.1
.. _8.2.2: https://github.com/canonical/snapcraft/releases/tag/8.2.2
.. _8.2.3: https://github.com/canonical/snapcraft/releases/tag/8.2.3
.. _8.2.4: https://github.com/canonical/snapcraft/releases/tag/8.2.4
.. _8.2.5: https://github.com/canonical/snapcraft/releases/tag/8.2.5
.. _8.2.6: https://github.com/canonical/snapcraft/releases/tag/8.2.6
.. _8.2.7: https://github.com/canonical/snapcraft/releases/tag/8.2.7
.. _8.2.8: https://github.com/canonical/snapcraft/releases/tag/8.2.8
.. _8.2.9: https://github.com/canonical/snapcraft/releases/tag/8.2.9
.. _8.2.10: https://github.com/canonical/snapcraft/releases/tag/8.2.10
.. _8.2.11: https://github.com/canonical/snapcraft/releases/tag/8.2.11
.. _8.2.12: https://github.com/canonical/snapcraft/releases/tag/8.2.12
.. _8.3.0: https://github.com/canonical/snapcraft/releases/tag/8.3.0
.. _8.3.1: https://github.com/canonical/snapcraft/releases/tag/8.3.1
.. _8.3.2: https://github.com/canonical/snapcraft/releases/tag/8.3.2
.. _8.3.3: https://github.com/canonical/snapcraft/releases/tag/8.3.3
.. _8.3.4: https://github.com/canonical/snapcraft/releases/tag/8.3.4
.. _8.4.0: https://github.com/canonical/snapcraft/releases/tag/8.4.0
.. _8.4.1: https://github.com/canonical/snapcraft/releases/tag/8.4.1
.. _8.4.2: https://github.com/canonical/snapcraft/releases/tag/8.4.2
.. _8.4.3: https://github.com/canonical/snapcraft/releases/tag/8.4.3
.. _8.4.4: https://github.com/canonical/snapcraft/releases/tag/8.4.4
.. _8.5.0: https://github.com/canonical/snapcraft/releases/tag/8.5.0
.. _8.5.1: https://github.com/canonical/snapcraft/releases/tag/8.5.1
.. _8.6.0: https://github.com/canonical/snapcraft/releases/tag/8.6.0
.. _8.6.1: https://github.com/canonical/snapcraft/releases/tag/8.6.1
.. _8.6.2: https://github.com/canonical/snapcraft/releases/tag/8.6.2
.. _8.6.3: https://github.com/canonical/snapcraft/releases/tag/8.6.3
