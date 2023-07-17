.. 15773.md

.. _release-notes-snapcraft-3-10:

Release notes: Snapcraft 3.10
=============================

These are the release notes for `Snapcraft 3.10 <https://github.com/snapcore/snapcraft/releases/tag/3.10>`__.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

New *core* features
-------------------

Configurable system usernames
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft now supports the configuration of :ref:`system usernames <system-usernames>` for daemons. This functionality was recently added to snapd, and enables a daemon to run as the user specified within an application’s ‘command’ entry, for example, among other functions.

Primed stage packages
~~~~~~~~~~~~~~~~~~~~~

Snapcraft now tracks the files within a snap that are derived from those listed under ``staged-packages``.

A build manifest already includes the set of all staged packages in ``staged-packages``, but the build manifest also includes files that were installed, then removed or filtered out, from the resulting snap.

The resulting list of staged and tracked packages is maintained within ``primed-stage-packages``. Security notices will eventually use ``primed-stage-packages`` to reduce the number of false positive alerts that are caused by staged-packages listing packages that are not eventually present in the snap.

CLI improvements
----------------

New CLI configuration file
~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft now reads a configuration file to populate the default *snapcraft* command-line options.

-  Location: ``$HOME/.config/snapcraft/config.yaml``
-  Syntax: ``YAML``
-  Valid configuration options: Anything that matches a snapcraft CLI command option (e.g. provider, bind-ssh, debug, http-proxy, https-proxy) for arguments. The configuration file uses YAML syntax and supports any option that matches a valid snapcraft argument.

For example, a config like:

.. code:: yaml

   use-lxd: true
   bind-ssh: true
   http-proxy: http://192.168.1.10:3128
   https-proxy: http://192.168.1.10:3128 `

Matches:

.. code:: bash

   snapcraft --use-lxd \
             --bind-ssh \
             --http-proxy http://10.155.149.232:3128 \
             --https-proxy http://10.155.149.232:3128

..

   ℹ While an argument can be overridden, it cannot be unset. For instance, if using http-proxy`` or ``debug``, there is no method to currently unset these values.

Remote Build
~~~~~~~~~~~~

Many improvements have been made to :ref:`Remote build <remote-build>`, including the removal of unnecessary options such as entering a Launchpad ID, which is provided via the login process.

To build snaps, ``remote build`` previously used snapcraft from the ``edge``\ channel. From this release, snaps are created using snapcraft from the ``stable`` channel.

The user interface has also been cleaned-up, with more changes on the way.

Build providers
---------------

HTTP proxies
~~~~~~~~~~~~

Snapcraft now supports passing ``http_proxy`` and ``https_proxy`` through to LXD and Multipass build environments. Options have also been added to specify these using ``--http-proxy <proxy>`` and ``-https-proxy <proxy>``.

Bind SSH
~~~~~~~~

Local SSH configuration can now be passed through to LXD and Multipass environments.

The user’s SSH directory, (eg. ``${HOME}/.ssh``), is bind-mounted to the build-environment, enabling SSH configuration within that build environment. This option is enabled by using ``--bind-ssh`` with the LXD or Multipass providers.

Plugins
-------

Rust
~~~~

The Rust plugin now correctly works with `cargo workspaces <https://doc.rust-lang.org/book/ch14-03-cargo-workspaces.html>`__.

From this release, the default is to now use the minimal rust profile for building, which solves a problem of generally building on s390x, arm64 and Ppcel64 architectures.

A few smaller issues, such as using the same path for ``CARGO_HOME`` and ``RUSTUP_HOME``, have been fixes with this release, and ``Cargo.lock`` files are now also properly respected.

Go
~~

The Go plugin has been cleaned-up and, as part of this release, now includes support for `Go Mod <https://blog.golang.org/using-go-modules>`__

If a project makes use of the Go Modules feature, the **right** things will happen.

Catkin
~~~~~~

A long lived workaround has been removed from the robotics tooling. This includes the ‘rospack’ workaround now that ``rosdep -i`` works correctly.

Another improvement to the plugin includes fixing an issue when using ’–destructive-mode`, where some installation paths from the host machine leaked into the list of paths,considered by the Catkin plugin. This resulted in dependencies being found in the host path and consequently not being installed by the plugin in the expected **parts** path.

Python
~~~~~~

From this release, the Python plugin will process its requirements separately from ``setup.py. This helps to better satisfying local dependencies declared in``\ install_requires`. As a result, building becomes less costly as wheels are not unnecessarily generated with every build.

The Python plugin also creates a cleaner ``sitecustomize`` that no longer leaks Snapcraft’s ``site-packages`` into the part that the plugin processes.

Extensions
----------

Projects using extensions will gain better performance with this release, thanks to the scaffolding to bring up desktop applications being improved. These improvements include pre-checks being run before spawning shells to run some setup commands (even if they were idempotent - pre-checking avoids their cost).

Icon caching pre-checks are now also in place, and can reduce the time to bring up an application by up to 10 seconds.

Information Parsing
-------------------

AppStream
~~~~~~~~~

Several AppStream fixes arrive with this release. These include a fix related to silently breaking XSLT transformations, when comments were present in the AppStream file, and support for the recently specced ``<em>`` and ``<code>`` tags. These are now supported by Snapcraft when using the ``parse-info`` functionality with AppStream files.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 3.10 are reflected in the following change list:

Adam Collard:
-------------

-  snap: add license to snapcraft.yaml

Anatoli Babenia:
----------------

-  docker: test image builds with Travis CI (`#2851 <https://github.com/snapcore/snapcraft/pull/2851>`__)

Anton Maminov:
--------------

-  crystal plugin: add flags to use during shards build (`#2740 <https://github.com/snapcore/snapcraft/pull/2740>`__)

Chris Patterson:
----------------

-  cli: add new YAML configuration file support
-  cli: enable experimental snapcraft configuration file
-  cli: introduce –provider
-  spread tests: unset SNAPCRAFT_BUILD_ENVIRONMENT=host for lxd tests
-  cli: explicitly set show_envvar=False for –provider
-  spread tests: ensure multipass tests use multipass for clean
-  xattrs: introduce internal xattrs support
-  xattrs: handle case where attribute may be too long
-  manifest: track and annotate ``primed-stage-packages``
-  cli: treat “managed-host” as separate provider than “host”
-  cli: annotate return type for \_sanity_check_build_provider_flags()
-  cli: generic handling for provider-specific flag errors
-  cli: introduce get_build_provider_flags
-  cli: introduce http and https proxy options
-  cli: introduce apply_host_provider_flags for host providers
-  cli: apply provider flags for each provider
-  build providers: introduce build provider flags
-  build providers: set configured environment flags in ``run()``
-  build providers: add ``hide_output`` flag to abstract \_run() definition
-  build providers: passthrough flags for http_proxy and https_proxy
-  tests: introduce unit tests for options
-  tests: mock patch sys.argv for CLI runner
-  cli: exempt containers from requiring –destructive-mode
-  tests: add env-passthrough spread tests for LXD and host
-  base provider: introduce generalized \_get_home_directory()
-  multipass provider: rename \_mount to \__mount
-  build providers: introduce \_mount() and \_is_mounted()
-  build providers: unify per-provider mount_project()
-  build providers: unify per-provider \_mount_prime_directory()
-  multipass provider: remove now-unused \__mount() method
-  build providers: implement support for bind-ssh
-  cli: introduce –bind-ssh option
-  meta: fix string comparison operator in Snap validate
-  snapcraft: add missing imports for typing
-  requirements-devel: uprev flake8 to 3.7.9
-  requirements-devel: uprev coverage to 4.5.4
-  codespell: various spelling fixes
-  codespell: add snapcraft.spec to ignore list
-  codespell: address codespell error in test_common unit test
-  codespell: address codespell error in circular-dependencies test
-  requirements-devel: uprev codespell to 1.16.0
-  requirements-devel: uprev pycodestyle to 2.5.0
-  requirements-devel: uprev pyflakes to 2.1.1
-  requirements-devel: uprev pyftpdlib to 1.5.5
-  requirements-devel: uprev pyramid to 1.10.4
-  project loader: remove noqa on import that’s no longer required
-  conda plugin: simplify source url/checksum handling
-  repo: fix fetch_binary()’s return type for deb repo
-  cli: add missing argument to click.BadOptionUsage()
-  cli: label Optional types in lifecycle
-  meta: declare optional types for Snap
-  meta: fix Slot from_dict() to handle case where interface is undefined
-  meta: fix Plug’s from_dict() if interface is undefined
-  meta: fix typing error in ContentPlug’s from_dict()
-  meta: various fixes to better annotate type definitions and optionals
-  cmake plugin: declare type for \_Flag.value
-  python plugin: declare return type Optional for \_find_file()
-  python plugin: declare Optional arguments for \_process_package_args()
-  extractors: ensure valid loader available in setuppy’s extract()
-  appstream: fix mypy typing error in \_get_icon_from_theme()
-  extractors: set Optional types in ExtractedMetadata
-  extractors: use None as default parameter value for ExtractedMetadata
-  store: remove incorrect default for ``store`` parameter in (_try)_login()
-  tests: fix incorrect regex format strings in test_store_push
-  storeapi: verify snap information before using
-  storeapi: only use errors with codes in StoreErrorList
-  storeapi: update return definition in acl()
-  storeapi: annotate Optional types in Channel
-  store: validate type for snap_ids in \_human_readable_acls()
-  plugin handler: annotate Optionals in PluginHandler
-  plugin handler: address mypy errors in \_handle_dependencies()
-  multipass provider: improve safety in \_requests_exception_hint()
-  multipass provider: fix except in \_fetch_installer_url()
-  lxd provider: fix \_run() return when hide_output is False
-  lxd provider: address mypy uprev errors
-  lxd provider: fix incorrectly formatted error message
-  snap provider: add assertions to address mypy errors
-  multipass provider: label \_instance_info as Optional
-  base provider: address mypy errors in cached_home_directory()
-  tests: fix type definition for Provider mock in test_build_providers
-  tests: annotate get_version_codename() return type as Optional
-  steps: label return types as Optional
-  dirs: ensure SNAP is defined when running as snap
-  yaml_utils: fix type annotations for dump()
-  file utils: ensure SNAP is defined in get_tool_path()
-  config: label Optionals to address mypy errors
-  project: minor refactoring for is_host_compatible_with_base()
-  cache: label Optional return for cache()
-  elf: minor type fix in \_extract()
-  tests: fix invalid format string in fake_servers
-  lifecycle: minor type annotation fixes
-  tests: fix format string in HookTestCase
-  remote-build: minor type fixes for LaunchpadClient
-  project_loader: check latest_step is valid before returning
-  extensions: raise KeyError if kde or gnome are initialized without base
-  project loader: explicitly check match in ToStatement grammar processing
-  project loader: explicitly check match in OnStatement grammar processing
-  extensions: correctly annotate base as Optional
-  grammar: address GrammarProcessor typing issues
-  inspection: annotate state_getter as Optional
-  grammar: annotate that statement’s else_bodies allows None
-  grammar: address mypy-detected type issues in Statement
-  lifecycle: add optionals to StatusCache reports
-  project: label get_build_base() return as optional
-  project: ignore project.info’s Optional[ProjectInfo] type
-  requirements-devel: uprev mypy to 0.740
-  CODE_STYLE: update command to install black
-  tools: add shellcheck to developer environment
-  HACKING: use code blocks rather than indents for commands
-  HACKING: remove odd blockquote
-  xattrs: switch to python’s os package for reading/writing xattrs
-  xattrs: ignore errors if SNAPCRAFT_BUILD_INFO is unset
-  remote-build: remove option to specify launchpad username
-  remote-build: login automatically when initialized
-  remote-build: fix AcceptPublicUploadError option
-  remote-build: remove \_waiting from LaunchpadClient
-  sources: disable gpg signing for git commit
-  sources: improve command quoting in SnapcraftPullError
-  sources: introduce GitCommandError for improved user-facing errors
-  remote-build: use easier to read git commit message format
-  rust plugin: split RUSTUP_HOME and CARGO_HOME
-  hooks: enable command-chain in snapcraft.yaml (`#2850 <https://github.com/snapcore/snapcraft/pull/2850>`__)
-  base plugin: use shlex quoting for logged command in run() (`#2846 <https://github.com/snapcore/snapcraft/pull/2846>`__)
-  project: remove unused errors (`#2855 <https://github.com/snapcore/snapcraft/pull/2855>`__)
-  rust: add support for workspaces (`#2842 <https://github.com/snapcore/snapcraft/pull/2842>`__)
-  remote-build: configurable timeout/deadline for starting and monitoring build (`#2845 <https://github.com/snapcore/snapcraft/pull/2845>`__)
-  meta: enable Snap to be fully initialized with init parameters (`#2857 <https://github.com/snapcore/snapcraft/pull/2857>`__)
-  common: generate run scripts which can execute independently (`#2848 <https://github.com/snapcore/snapcraft/pull/2848>`__)
-  meta: remove Application’s ``prepend_command_chain`` (`#2861 <https://github.com/snapcore/snapcraft/pull/2861>`__)
-  add support for system-usernames (`#2858 <https://github.com/snapcore/snapcraft/pull/2858>`__)
-  elf: remove return parameters for ElfFile’s \_extract() (`#2867 <https://github.com/snapcore/snapcraft/pull/2867>`__)
-  extensions: change extension merge-strategy to fix build-environment (`#2882 <https://github.com/snapcore/snapcraft/pull/2882>`__)
-  elf: read ELF type when extracting attributes (`#2888 <https://github.com/snapcore/snapcraft/pull/2888>`__)
-  meta: always generate snapcraft-runner to workaround classic PATH bug (`#2889 <https://github.com/snapcore/snapcraft/pull/2889>`__)
-  lifecycle: raise detailed error if mksquashfs fails (`#2895 <https://github.com/snapcore/snapcraft/pull/2895>`__)
-  meta: include environment in hook wrappers (`#2897 <https://github.com/snapcore/snapcraft/pull/2897>`__)
-  meta: remove dead code from snap packaging (`#2898 <https://github.com/snapcore/snapcraft/pull/2898>`__)
-  requirements: uprev pyinstaller to 3.6 (`#2905 <https://github.com/snapcore/snapcraft/pull/2905>`__)
-  meta: move Snap’s from_dict() system-username parsing into SystemUser (`#2904 <https://github.com/snapcore/snapcraft/pull/2904>`__)
-  meta: do not prime commands with adapter == “none” (`#2912 <https://github.com/snapcore/snapcraft/pull/2912>`__)
-  spread: disable journal debug dump unless configured (`#2913 <https://github.com/snapcore/snapcraft/pull/2913>`__)
-  meta: ensure Application passthrough is scrubbed for snap.yaml (`#2914 <https://github.com/snapcore/snapcraft/pull/2914>`__)
-  rust plugin: respect Cargo.lock if present in project (`#2915 <https://github.com/snapcore/snapcraft/pull/2915>`__)
-  rust plugin: fetch correct (locked) crates during pull (`#2917 <https://github.com/snapcore/snapcraft/pull/2917>`__)
-  meta: initialize Snap at once in from_dict() (`#2920 <https://github.com/snapcore/snapcraft/pull/2920>`__)
-  elf: ensure \_GNU_VERSION_R section is of type GNUVerNeedSection (`#2918 <https://github.com/snapcore/snapcraft/pull/2918>`__)
-  plugin handler: process elf files only if base is specified (`#2926 <https://github.com/snapcore/snapcraft/pull/2926>`__)
-  elf: fixes for corrupt shared objects (`#2929 <https://github.com/snapcore/snapcraft/pull/2929>`__)
-  meta: fix for missing content slot’s ‘content’ property (`#2934 <https://github.com/snapcore/snapcraft/pull/2934>`__)
-  spread tests: do not attempt to remove snapd snap (`#2937 <https://github.com/snapcore/snapcraft/pull/2937>`__)
-  remote build: default to snapcraft’s stable channel (`#2938 <https://github.com/snapcore/snapcraft/pull/2938>`__)

Heather Ellsworth (2):
----------------------

-  Remove gsettings from comment in kde extension
-  docs: add punctuation rule for comments (`#2844 <https://github.com/snapcore/snapcraft/pull/2844>`__)

James Henstridge:
-----------------

-  elf: extract build ID and presence of debug info (`#2229 <https://github.com/snapcore/snapcraft/pull/2229>`__)

Jeremie Deray:
--------------

-  catkin plugin: consider only ‘local’ workspaces (`#2847 <https://github.com/snapcore/snapcraft/pull/2847>`__)

Kyle Fazzari:
-------------

-  elf: properly handle corrupted ELF files
-  wstool: don’t rely on host git (`#2852 <https://github.com/snapcore/snapcraft/pull/2852>`__)

Marcus Tomlinson:
-----------------

-  extensions: use ensure_dir_exists instead of mkdir -p (`#2886 <https://github.com/snapcore/snapcraft/pull/2886>`__)
-  extensions: symlink $XDG_RUNTIME_DIR/../dconf/user for desktop parts (`#2874 <https://github.com/snapcore/snapcraft/pull/2874>`__)

Merlijn Sebrechts:
------------------

-  extensions: skip icon cache creation for theme and runtime snaps
-  extensions: Handle case when only user-dirs.locale doesn’t exist (`#2930 <https://github.com/snapcore/snapcraft/pull/2930>`__)

NickZ:
------

-  build providers: fix multipass mount on win32 (`#2894 <https://github.com/snapcore/snapcraft/pull/2894>`__)
-  coherence checks: fix expressions so Windows paths are considered (`#2919 <https://github.com/snapcore/snapcraft/pull/2919>`__)

Sergio Schvezov:
----------------

-  cli: improve the remote-build upload messaging
-  spread tests: update checkbox-ng dependency in plainbox run
-  static tests: fix static tests
-  store cli: push title and license on push-metadata
-  appstream extractor: simplify the XSLT
-  appstream extractors: remove skips from tests
-  appstream extractor: add support for ``<em>``
-  appstream extractor: add support for code
-  appstream extractor: take xml comments into account
-  go plugin: cleanup build procedure
-  go plugin: cleanup pull procedure
-  go plugin: add type annotations
-  go plugin: extract CGO_FLAGS into its own method
-  go plugin: support for go.mod
-  spread tests: use source-depth: 1 for plainbox tests (`#2863 <https://github.com/snapcore/snapcraft/pull/2863>`__)
-  python plugin: first try processing setup.py without PyPI (`#2771 <https://github.com/snapcore/snapcraft/pull/2771>`__)
-  cli: implement progressive releases (`#2868 <https://github.com/snapcore/snapcraft/pull/2868>`__)
-  docker: add core18 snap that snapcraft now uses as a base (`#2883 <https://github.com/snapcore/snapcraft/pull/2883>`__)
-  static: fix some valid flake8 issues (`#2902 <https://github.com/snapcore/snapcraft/pull/2902>`__)
-  tests: fix status test for staging store (`#2903 <https://github.com/snapcore/snapcraft/pull/2903>`__)
-  ci: publish the CI built snap to the Snap Store (`#2900 <https://github.com/snapcore/snapcraft/pull/2900>`__)
-  python plugin: do not leak snapcraft’s site-packages (`#2901 <https://github.com/snapcore/snapcraft/pull/2901>`__)
-  elf: search for host libraries within search paths (`#2909 <https://github.com/snapcore/snapcraft/pull/2909>`__)
-  storeapi: remove exposure of series (`#2921 <https://github.com/snapcore/snapcraft/pull/2921>`__)
-  logging: use .warning instead of deprecated .warn (`#2928 <https://github.com/snapcore/snapcraft/pull/2928>`__)
-  store: improve platform detection (`#2931 <https://github.com/snapcore/snapcraft/pull/2931>`__)
-  build providers: clean up LXD startup message (`#2936 <https://github.com/snapcore/snapcraft/pull/2936>`__)
-  build providers: remove tzdata workaround (`#2935 <https://github.com/snapcore/snapcraft/pull/2935>`__)
-  store: temporarily remove support for progressive releases (`#2946 <https://github.com/snapcore/snapcraft/pull/2946>`__)

Ted Kern:
---------

-  catkin plugin: remove rospack workaround now that rosdep -i works (`#2833 <https://github.com/snapcore/snapcraft/pull/2833>`__)

dalance:
--------

-  rust plugin: set rustup profile to minimal (`#2767 <https://github.com/snapcore/snapcraft/pull/2767>`__) 
