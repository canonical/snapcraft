.. 14434.md

.. _release-notes-snapcraft-3-9:

Release notes: Snapcraft 3.9
============================

These are the release notes for `Snapcraft 3.9 <https://github.com/snapcore/snapcraft/releases/tag/3.9>`__.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Remote Build
------------

With ``snapcraft remote-build``, Snapcraft gains the ability to run a multi-architecture build process on remote servers, using `Launchpad <https://launchpad.net/>`__, directly from the *snapcraft* executable.

While remote build is still considered a preview in this release, it’s now fully accessible. Setup has also been simplified thanks to using HTTPS for *git transport* to push assets to the build server (removing the reliance on ssh).

For more details, see the documentation for :ref:`Remote build <remote-build>`.

New *core* features
-------------------

Improved commands for snap.yaml
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft will now attempt to avoid using unnecessary wrappers:

-  ``command-chain`` is used to create an environment where ``snap --shell <snap-name.app-name>`` can be run without the previously required environment setup
-  when a previously generated ``.wrapper`` file is detected, Snapcraft will work out whether it too can be avoided
-  if feasible, the *snapcraft* command will be modified to avoid using a wrapper. The user is also informed of what’s changed

Binary dependency crawling
~~~~~~~~~~~~~~~~~~~~~~~~~~

A snap developer may inadvertently exclude a snap’s required libraries, either by filtering them out during ``stage`` and ``prime``, through a script, or when libraries are linked against the host (amongst other reasons).

To help mitigate these missing libraries, Snapcraft creates an internal list of libraries that may be missing. Prior releases of Snapcraft would output this list as file names.

With this release, Snapcraft outputs the exact ``stage-packages`` syntax to fix the missing dependencies. This can be pasted directly into the relevant ``part``.

Additionally, false positives from snaps that rely on the ``content`` interface for their runtime have been reduced.

Support for core20
~~~~~~~~~~~~~~~~~~

The *core20* base snap (see :ref:`base-snaps`) is now a first class citizen in Snapcraft.

As core20 itself is still under development, plugin support will be added closer to its finalisation, and snaps built against its non-stable channel releases will be marked with a grade of *devel*.

Support for AppStream Title and Version
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Titles and versions are now extracted from AppStream data.

Consequently, both are now passed into the resulting snap.yaml where they can be used by the Snap Store on first push, or after a ``snapcraft push-metadata`` call.

core18
~~~~~~

Internally, Snapcraft has fully migrated to core18. This allows for easier continued development and improved robustness.

Extensions
----------

KDE Neon
~~~~~~~~

Creating KDE application snaps is now even easier, thanks to a new KDE Neon extension.

This new extension integrates seamlessly with the KDE Neon content snap and builds on the foundations of the work done for the Gnome extension.

Documentation for this extension is being worked. See :ref:`KDE Neon extension <the-kde-neon-extension>` for further details.

Gnome 3.28 improvements
~~~~~~~~~~~~~~~~~~~~~~~

The Gnome extension has been fixed when using only a snap name as the default provider, and launch performance has also been improved.

For documentation, see :ref:`Gnome 3.28 extension <the-gnome-3-28-extension>`.

CLI
---

Error Messages
~~~~~~~~~~~~~~

The underlying implementation behind error message generation has changed, becoming more structured.

This allows for more specific messages about what may have gone wrong, why it went wrong, and how it can be fixed. Additional ancillary data, such as links to further documentation, is also presented for some of the error messages.

Guided login and register
~~~~~~~~~~~~~~~~~~~~~~~~~

Prior to this release, using the ‘register’ command with snapcraft running under an interactive shell would generate an error message if the user was not logged in.

With Snapcraft 3.9, the user is instead prompted for their credential to continue with the registration. Similarly, when trying to push a snap, if the snap-name is currently unregistered, the user will be asked if they want to register the snap-name for the snap to be pushed, and then continue with the process.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 3.9 are reflected in the following change list:

Sergio Schvezov
~~~~~~~~~~~~~~~

-  tests: print journal logs when spread tests fail
-  meta: new application handler
-  meta: add desktop file handling to application
-  meta: move desktop file cleanup to a separate method
-  meta: replace logic in \_SnapPackaging with Application
-  tests: completely mock bzr tests
-  tests: completely mock mercurial tests
-  tests: completely mock 7z tests
-  snap: migrate to core18
-  ci: move unit tests to spread
-  build providers: inject core18 instead of core
-  tests: completely mock subversion tests
-  docs: add a Code of Conduct (`#2724 <https://github.com/snapcore/snapcraft/pull/2724>`__)
-  pluginhandler: remove the exception for elf patching go
-  project: support for base bare
-  tests: update rust-toolchain test so it pulls from beta
-  storeapi: use the channels attribute in push
-  meta: take no command-chain being prepended into account
-  cli: add -s back to clean for legacy (LP: #1834628)
-  cli: prompt for login if required
-  extensions: new kde-neon extension
-  cli: use click utilities for login prompts
-  meta: warn about command mangling
-  storeapi: add StoreErrorList to handle store errors
-  cli: clean up StoreClientCLI
-  tests: move cli store push/upload tests to FakeStoreCommands…
-  cli: use click utilities for registering on push (LP: #1805211)
-  meta: support the case of a plug without a default provider
-  remote build: switch from core to core18
-  make plugin: support for core20
-  snaps: invalidate cache on refresh or install
-  snaps: allow installation of non stable bases
-  meta: force grade devel when using non stable bases
-  build providers: inject snapd snap for latest feature availability
-  repo: convey proper error message when refreshing to invalid channel
-  cli: pass channels None when not doing a push –release

Chris Patterson
~~~~~~~~~~~~~~~

-  tests: change default spread provider to lxd outside of travis
-  meta: handle desktop files with multiple sections
-  meta: preserve desktop file Exec= arguments
-  snaps: if snap is installed, don’t check is_valid()
-  mypy.ini: set python version to 3.6
-  tests: minor fixups for mypy to run successfully
-  runtests: add mypy coverage of unit tests to static target
-  errors: add new abstract base class for snapcraft exceptions
-  cli: add support for new-style snapcraft exceptions
-  tests: fix mypy error with test_errors.py
-  meta: introduce snap, hook, plug, and slot types
-  application: refactor to work with introduced snap meta objects
-  command: refactor to work with Snap meta
-  project: instantiate snap meta
-  project: introduce \_get_content_snaps() and \_get_provider_content_dirs()
-  project-loader: initialize project._snap_meta when data is updated
-  runner: install content snaps when installing build snaps
-  meta: remove create_snap_packaging from init to prevent import loop
-  snap-packaging: refactor to use Snap
-  pluginhandler: refactoring dependency resolution
-  elf: consider content directories for determining dependencies
-  common: rename get_core_path() to get_installed_snap_path()
-  pluginhandler: add some type annotations
-  fixtures: mock patch Project._get_provider_content_dirs()
-  spread tests: update unicode-metadata expect_snap.yaml’s ordering
-  snap-packaging: do not write command-chain wrapper if there are no apps
-  project options: add compatibility shims for tests
-  elf: handle missing dependencies not found on system
-  tests: update gnome-3-28 extension spread test to use gtk
-  tests: update gnome extension spread task to account for content snaps
-  tests: update kde extension spread task to account for content snaps
-  remote-build: detect early build errors (`#2642 <https://github.com/snapcore/snapcraft/pull/2642>`__)
-  fixtures/SnapcraftYaml: rewrite snapcraft.yaml on updates
-  remote-build: fully preserve local sources
-  remote-build: introduce –package-all-sources flag
-  git: add init, add, commit, push, version, check_if_installed functions
-  remote-build/launchpad: pivot to git source handler
-  remote-build: use project name in build-id for launchpad git repo
-  remote-build: error if –user is required
-  requirements: add lazr.restfulclient dependency for launchpad
-  windows: update snapcraft.spec for new remote-build dependencies
-  remote-build: make –user required and drop config file handling
-  remote-build: only prepare project if starting build
-  project: add ``_get_project_directory_hash`` method
-  remote-build: use project directory hash for id
-  remote-build: introduce LaunchpadGitPushError
-  tests/remote-build: minor cleanup for mock usage
-  remote-build: update launchpad to support git https tokens
-  tests/remote-build: cleanup usage of mock_lp
-  remote-build: graduate from preview -> experimental
-  errors: migrate handful of errors to SnapcraftException
-  project: truncate project directory hash (`#2766 <https://github.com/snapcore/snapcraft/pull/2766>`__)
-  setup.py: convert classifiers from tuple to list
-  sources: add some initial support for win32
-  file_utils: fix create_similar_directory on Windows platforms
-  file_utils: add cross-platform rmtree (Windows support)
-  remote-build: use file_utils.rmtree for Windows support
-  remote-build: use posix pathing when creating paths for snapcraft yaml
-  remote-build: gunzip downloaded log files
-  manifest: sort package and snap lists for consistency
-  remote-build: cleanup and fix architecture handling
-  remote-build: explicitly default build arch to host arch
-  remote-build: remove ``all`` option for ``--arch``
-  remote-build: remove old TODO comment
-  errors: preserve quotes when printing SnapcraftPluginCommandError
-  remote-build: improve resiliency for https connection issues
-  remote-build: add unit tests for errors
-  remote-build: support autorecovery of builds

Claudio Matsuoka
~~~~~~~~~~~~~~~~

-  cli: add remote build (`#2500 <https://github.com/snapcore/snapcraft/pull/2500>`__)
-  remote build: add warning before sending data (`#2567 <https://github.com/snapcore/snapcraft/pull/2567>`__)
-  remote build: retrieve build log files (`#2574 <https://github.com/snapcore/snapcraft/pull/2574>`__)
-  remote build: don’t send log files back to remote
-  remote build: handle git push in detached head state (`#2564 <https://github.com/snapcore/snapcraft/pull/2564>`__)
-  remote build: add option to skip public upload question (`#2590 <https://github.com/snapcore/snapcraft/pull/2590>`__)

Kyle Fazzari
~~~~~~~~~~~~

-  cmake plugin: support disable-parallel option
-  project: use os.sched_getaffinity instead of multiprocessing.cpu_count
-  project_loader: load build-environment after snapcraft environment

Merlijn Sebrechts
~~~~~~~~~~~~~~~~~

-  extensions: add gsettings plug to gnome-3-28 extension
-  docs: Added ‘shellcheck’ testing dependency
-  extensions: support using gjs from gnome runtime
-  appstream: extract title and version
-  docs: use real testing examples
-  appstream: support legacy ids without desktop suffix (LP: #1778546)
-  extensions: kde-neon: add icon and sound themes

NickZ
~~~~~

-  nodejs plugin: fix errors when building with sudo (`#2747 <https://github.com/snapcore/snapcraft/pull/2747>`__)

Anatoli Babenia
~~~~~~~~~~~~~~~

-  docker: use apt-get to avoid warnings (`#2672 <https://github.com/snapcore/snapcraft/pull/2672>`__)

Ken VanDine
~~~~~~~~~~~

-  gnome extension: use the snap name only for the default-provider (`#2763 <https://github.com/snapcore/snapcraft/pull/2763>`__)
-  kde neon extension: use the snap name only for the default-provider (`#2764 <https://github.com/snapcore/snapcraft/pull/2764>`__)


