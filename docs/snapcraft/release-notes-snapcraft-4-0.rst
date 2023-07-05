.. 17515.md

.. _release-notes-snapcraft-4-0:

Release notes: Snapcraft 4.0
============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.0 <https://github.com/snapcore/snapcraft/releases/tag/4.0>`__.

Among its many updates, fixes and additions, the following are what we consider its highlights:

-  the ``core20`` base is now supported
-  ``--use-lxd`` can now be used with all snap supported architectures
-  improved plugins for ``core20``
-  support for adding external repositories

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Plugins V2
----------

A new plugin infrastructure has been developed to support ``core20``.

This new infrastructure greatly simplifies plugins and delivers another great advantage: “*quick rebuilds*”.

Thanks to ``core20`` plugins being much simpler, so to is the environment setup process, removing most of the perceived *magic* during Snapcraft builds for parts.

Plugins are now applicable only to Snapcraft’s *build* step. The *pull* step, for instance, has become completely owned by Snapcraft and dedicated to managing the ``source`` related entries for parts.

The following plugins have been updated to work with ``core20``:

-  `autotools`_
-  `cmake`_
-  `dump`_
-  `go`_
-  `make`_
-  `meson`_
-  `nil`_
-  `npm`_
-  `python`_
-  `rust`_

While the list of plugins is not as broad as for ``core`` or ``core18``, they offer a strong foundation for the majority of snaps, and the list will grow after this initial release.

The command line related to plugins has gained some additional parameters to specifically reach *base* relevant information:

-  ``snapcraft help <plugin-name> [--base <base>]``
-  ``snapcraft list-plugins [--base <base>]``

These commands will default to using the ``base`` defined in the current Snapcraft project or to the latest supported base (i.e., ``core20``).

Also, plugins now have their properties scoped (i.e.; prefixed with the plugin-name).

autotools
~~~~~~~~~

The autotools plugin for ``core20`` works mostly in the same way, with the following exceptions:

-  the plugin checks for the existence of ``configure`` in the source. If not found, ``autoreconf --install`` is executed instead
-  ``configflags`` has been renamed to ``autotools-configure-parameters``
-  ``install-via`` has been removed

cmake
~~~~~

This plugin works mostly the same, except for the fact that ``configflags`` has been renamed to ``cmake-parameters``.

dump
~~~~

This behaves in the same way for ``core20`` as for ``core18`` or ``core``.

go
~~

The plugin has been revamped for ``core20``. It now only supports projects using ``go.mod``, which means it only supports version of ``go`` that support this.

The following are the only configuration parameters available to the plugin when using ``core20``:

-  ``go-channel``
-  ``go-buildtags``

These options are no longer available when using ``core20``:

-  ``go-importpath``
-  ``go-packages``

make
~~~~

The following parameters will be accepted by the plugin when setting ``core20`` as the base:

-  ``make-parameters``

The following are no longer accepted but should instead be easily managed with ``override-build``:

-  ``makefile``
-  ``artifacts``
-  ``make-install-var``

meson
~~~~~

This plugin works the same in ``core20`` although it has been enhanced for easier rebuilds.

nil
~~~

This plugin behaves in the same way for ``core20`` as for ``core18`` or ``core``.

npm
~~~

This is a new plugin for ``core20``. It is intended to replace the ``nodejs`` plugin, which is only available for ``core`` and ``core18``.

The only parameter the plugin now accepts is ``npm-node-version``.

python
~~~~~~

The python plugin has been simplified the most for ``core20`` and yet provides the most new functionality. It essentially behaves like a virtual environment, preferring the python interpreter shipped in the ``core20`` base.

By behaving this way, the plugin operates more like how a Python developer would expect, allowing for easier *snap* customisation whilst still using the plugin.

The plugin can use an interpreter if it is added through a comprehensive list of ``stage-packages`` (an extension shall be evaluated in the future to provide alternative complete python stacks).

When used with ``core20``, the plugin accepts the following parameters, with the same semantics as the V1 plugin used in ``core`` and ``core18``:

-  ``python-packages``
-  ``requirements``

rust
~~~~

This is another plugin that has been simplified to reduce the number of parameters when targeting ``core20`` as a base:

-  ``rust-features`` same behaviour as for ``core`` and ``core18``
-  ``rust-path``, defaulting to the current working directory, but can be set to the relative path of the crate to build when using workspaces

Package Management
------------------

This feature adds high-level package-management to snapcraft.yaml, enabling users to configure additional repositories & components.

Specifically, the scope of package-management is for anything affecting the behaviour and availability of:

-  build-packages
-  stage-packages
-  build-snaps
-  stage-snaps
-  python-packages

The scope of this spec will focus on the configuration of ``apt`` repositories, affecting the availability of ``build-packages`` and ``stage-packages``.

To use, simply configure ``package-repositories`` in snapcraft.yaml.

**Note**: *snapcraft* will log an ‘experimental feature’ warning until the schema is considered stable.

Here are some example configurations:

.. code:: yaml

       name: apt-example
       base: core18

       <snip>

       package-repositories:
         - type: apt
           ppa: snappy-dev/snapcraft-daily

         - type: apt
           deb-types: [deb, deb-src]
           components: [main]
           suites: [$SNAPCRAFT_APT_RELEASE]
           key-id: 78E1918602959B9C59103100F1831DDAFC42E99D
           url: http://ppa.launchpad.net/snappy-dev/snapcraft-daily/ubuntu

         - type: apt
           deb-types: [deb, deb-src]
           name: default
           components: [main, multiverse, restricted, universe]
           suites: [$SNAPCRAFT_APT_RELEASE, $SNAPCRAFT_APT_RELEASE-updates]
           key-id: test-key
           url: http://archive.ubuntu.com/ubuntu

Build Environments
------------------

The ``--use-lxd`` flag has been released from its experimental phase and now supports the same build roots as build.snapcraft.io (or Launchpad), bringing the two environments closer together. With these new images, there is now support for all the snap enabled architectures too.

Progressive Releases
--------------------

Initial *experimental* support for progressive releases has landed in Snapcraft. To view any existing progressive release use the ``status`` command, as an example:

.. code:: bash

       $ snapcraft status candycane
       Track     Arch      Channel    Version    Revision    Progress
       latest    all       stable     -          -           -
                           candidate  -          -           -
                           beta       0.6        8           → 20%
                                      10         13          → 80%
                           edge       ↑          ↑           -

To perform a progressive release, use the ``release`` command with the with the ``--progressive`` option. After releasing, the status of the release will be shown.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 4.0 are reflected in the following change list: ### Andrey M (1):

-  dotnet plugin: add dotnet runtime version and support core18 (`#3005 <https://github.com/snapcore/snapcraft/pull/3005>`__)

Chris Patterson (45):
~~~~~~~~~~~~~~~~~~~~~

-  catkin plugins: remove bash workaround for catkin cmake args (`#2972 <https://github.com/snapcore/snapcraft/pull/2972>`__)
-  repo: remove dead code from deb implementation (`#2993 <https://github.com/snapcore/snapcraft/pull/2993>`__)
-  repo: move filtered package list from manifest.txt into a python list (`#2994 <https://github.com/snapcore/snapcraft/pull/2994>`__)
-  yaml_utils: don’t sort keys when dumping (`#2991 <https://github.com/snapcore/snapcraft/pull/2991>`__)
-  repo: always use host source lists and remove those found in plugins (`#3003 <https://github.com/snapcore/snapcraft/pull/3003>`__)
-  repo: type annotations and mypy fixes for base (`#3001 <https://github.com/snapcore/snapcraft/pull/3001>`__)
-  repo: use functools.lru_cache for dpkg -L queries (`#3002 <https://github.com/snapcore/snapcraft/pull/3002>`__)
-  requirements: uprev python-apt to 1.6.0 (bionic package) (`#2999 <https://github.com/snapcore/snapcraft/pull/2999>`__)
-  go plugin: support projects with multiple binaries when using go.mod (`#3007 <https://github.com/snapcore/snapcraft/pull/3007>`__)
-  repo: use python-apt’s fetch_binary implementation (`#3009 <https://github.com/snapcore/snapcraft/pull/3009>`__)
-  repo: always use host release and arch for Ubuntu (`#3006 <https://github.com/snapcore/snapcraft/pull/3006>`__)
-  spread tests: set appropriate default base in snapcraft.yamls (`#2987 <https://github.com/snapcore/snapcraft/pull/2987>`__)
-  repo: introduce install_source() and install_gpg_key() to Ubuntu (`#3011 <https://github.com/snapcore/snapcraft/pull/3011>`__)
-  plugins: install required apt sources and keys to system (`#3012 <https://github.com/snapcore/snapcraft/pull/3012>`__)
-  cli: remove experimental config.yaml support (`#3016 <https://github.com/snapcore/snapcraft/pull/3016>`__)
-  remote build: remove artifact sanity check (`#3021 <https://github.com/snapcore/snapcraft/pull/3021>`__)
-  tests: remove usage of FakeApt fixtures in lifecycle (`#3024 <https://github.com/snapcore/snapcraft/pull/3024>`__)
-  tests: move FakeApt fixtures into deb tests (`#3025 <https://github.com/snapcore/snapcraft/pull/3025>`__)
-  repo: drop \_AptCache and add migrate to install_stage_packages() (`#3030 <https://github.com/snapcore/snapcraft/pull/3030>`__)
-  ci: use stable channel for building snapcraft snap in Travis (`#3036 <https://github.com/snapcore/snapcraft/pull/3036>`__)
-  repo: fix resolution of virtual build packages (`#3035 <https://github.com/snapcore/snapcraft/pull/3035>`__)
-  ci: add and ship a self-hosting build of snapcraft in Travis (`#3038 <https://github.com/snapcore/snapcraft/pull/3038>`__)
-  repo: minor debug log tweaks (`#3042 <https://github.com/snapcore/snapcraft/pull/3042>`__)
-  build providers: setup initial apt source configuration (`#3039 <https://github.com/snapcore/snapcraft/pull/3039>`__)
-  build providers: use ubuntu-ports mirrors for non-x86 platforms (`#3044 <https://github.com/snapcore/snapcraft/pull/3044>`__)
-  package repositories: initial schema and meta read/write support (`#3043 <https://github.com/snapcore/snapcraft/pull/3043>`__)
-  repo: fix returned strings for install_stage_packages() (`#3047 <https://github.com/snapcore/snapcraft/pull/3047>`__)
-  build providers: rename default sources (`#3049 <https://github.com/snapcore/snapcraft/pull/3049>`__)
-  project: introduce ‘keys’ for project assets (`#3051 <https://github.com/snapcore/snapcraft/pull/3051>`__)
-  meta: split up package repository sanity checks (`#3050 <https://github.com/snapcore/snapcraft/pull/3050>`__)
-  repo: add identifiers for gpg keys and sources (`#3055 <https://github.com/snapcore/snapcraft/pull/3055>`__)
-  repo: format $SNAPCRAFT_APT_RELEASE instead of ${release} for suites (`#3057 <https://github.com/snapcore/snapcraft/pull/3057>`__)
-  package repositories: make ‘name’ optional (`#3058 <https://github.com/snapcore/snapcraft/pull/3058>`__)
-  remote build: package up local sources with source-type ‘git’ (`#3056 <https://github.com/snapcore/snapcraft/pull/3056>`__)
-  requirements: uprev python-apt (`#3067 <https://github.com/snapcore/snapcraft/pull/3067>`__)
-  [experimental] package-management repository configuration (`#2911 <https://github.com/snapcore/snapcraft/pull/2911>`__)
-  schema: minor tweaks/fixes for package-repositories (`#3072 <https://github.com/snapcore/snapcraft/pull/3072>`__)
-  repo: fix decoding of CalledProcessError output (`#3071 <https://github.com/snapcore/snapcraft/pull/3071>`__)
-  remote-build: fix case where build log url is None (`#3076 <https://github.com/snapcore/snapcraft/pull/3076>`__)
-  repo: fix for multi-arch stage-package scenario (`#3080 <https://github.com/snapcore/snapcraft/pull/3080>`__)
-  repo: fix for multi-arch virtual-packages (`#3084 <https://github.com/snapcore/snapcraft/pull/3084>`__)
-  repo: restore marked-install strategy for apt-cache (`#3086 <https://github.com/snapcore/snapcraft/pull/3086>`__)
-  repo: filter stage-packages using base’s manifest (core20)
-  tests: add tests for python with stage and python-package dep
-  tests: fully stage python3 requirements for python-hello-staged-python

Heather Ellsworth (1):
~~~~~~~~~~~~~~~~~~~~~~

-  extensions: add gcc to the build-packages for the gnome-3-34 (`#2995 <https://github.com/snapcore/snapcraft/pull/2995>`__)

James Henstridge (1):
~~~~~~~~~~~~~~~~~~~~~

-  build providers: pass through SNAPCRAFT_{BUILD,IMAGE}_INFO to container or VM (`#3031 <https://github.com/snapcore/snapcraft/pull/3031>`__)

Michał Sawicz (2):
~~~~~~~~~~~~~~~~~~

-  build providers: use stdio to get data in/out of Multipass (`#2784 <https://github.com/snapcore/snapcraft/pull/2784>`__)
-  meta: quote final LD_LIBRARY_PATH for command-chain (`#3053 <https://github.com/snapcore/snapcraft/pull/3053>`__)

Sergio Schvezov (56):
~~~~~~~~~~~~~~~~~~~~~

-  static: ignore direnv created artifacts (`#2985 <https://github.com/snapcore/snapcraft/pull/2985>`__)
-  tests: only run catkin based snap on 16.04 (`#2989 <https://github.com/snapcore/snapcraft/pull/2989>`__)
-  ci: remove osx test from Travis (`#2990 <https://github.com/snapcore/snapcraft/pull/2990>`__)
-  packaging: use find_namespace_packages in setup.py (`#2986 <https://github.com/snapcore/snapcraft/pull/2986>`__)
-  plugins: move the existing plugin to a new package (`#2984 <https://github.com/snapcore/snapcraft/pull/2984>`__)
-  requirements: uprev mypy to 0.770 (`#2996 <https://github.com/snapcore/snapcraft/pull/2996>`__)
-  specifications: progressive delivery (`#2997 <https://github.com/snapcore/snapcraft/pull/2997>`__)
-  CODE_STYLE: update to reflect latest conventions (`#2998 <https://github.com/snapcore/snapcraft/pull/2998>`__)
-  storeapi: add channel-map endpoint (`#3004 <https://github.com/snapcore/snapcraft/pull/3004>`__)
-  cli: use the channel-map api for status (`#3008 <https://github.com/snapcore/snapcraft/pull/3008>`__)
-  cli: add progressive releases support to the release command (`#3010 <https://github.com/snapcore/snapcraft/pull/3010>`__)
-  plugins: use v1 import path for all plugins (`#3013 <https://github.com/snapcore/snapcraft/pull/3013>`__)
-  meta: migrate get_build_base to Snap (`#3014 <https://github.com/snapcore/snapcraft/pull/3014>`__)
-  pluginhandler: deterministic load depending on plugin and build-base (`#3017 <https://github.com/snapcore/snapcraft/pull/3017>`__)
-  spread tests: default base for local plugin tests (`#3020 <https://github.com/snapcore/snapcraft/pull/3020>`__)
-  static: consolidate tooling setup to setup.cfg (`#3019 <https://github.com/snapcore/snapcraft/pull/3019>`__)
-  pluginhandler: move plugin attributes to PluginHandler (`#3023 <https://github.com/snapcore/snapcraft/pull/3023>`__)
-  static: mypy requires init.py (`#3027 <https://github.com/snapcore/snapcraft/pull/3027>`__)
-  static: add codespell excludes for .direnv (`#3028 <https://github.com/snapcore/snapcraft/pull/3028>`__)
-  spread tests: add core20 and cleanup systems (`#3026 <https://github.com/snapcore/snapcraft/pull/3026>`__)
-  plugins: introduce v2.PluginV2 and v2.NilPlugin (`#3022 <https://github.com/snapcore/snapcraft/pull/3022>`__)
-  build providers: move to buildd images for LXD (`#2966 <https://github.com/snapcore/snapcraft/pull/2966>`__)
-  tests: speed up step, pack and clean command unit tests (`#3029 <https://github.com/snapcore/snapcraft/pull/3029>`__)
-  build providers: do not print network test output for LXD (`#3033 <https://github.com/snapcore/snapcraft/pull/3033>`__)
-  plugins: introduce v2.MakePlugin with rebuilding (`#3032 <https://github.com/snapcore/snapcraft/pull/3032>`__)
-  plugins: introduce v2.CMakePlugin (`#3037 <https://github.com/snapcore/snapcraft/pull/3037>`__)
-  plugins: introduce v2.AutotoolsPlugin (`#3040 <https://github.com/snapcore/snapcraft/pull/3040>`__)
-  grammar: pick from properties if attributes not in the plugin (`#3045 <https://github.com/snapcore/snapcraft/pull/3045>`__)
-  plugins: introduce v2.PythonPlugin (`#3041 <https://github.com/snapcore/snapcraft/pull/3041>`__)
-  plugins: introduce v2.GoPlugin (`#3046 <https://github.com/snapcore/snapcraft/pull/3046>`__)
-  plugins: introduce v2.DumpPlugin (`#3048 <https://github.com/snapcore/snapcraft/pull/3048>`__)
-  build providers: wait for systemd and better nameserver setup on LXD (`#3052 <https://github.com/snapcore/snapcraft/pull/3052>`__)
-  plugins: introduce v2.MesonPlugin (`#3059 <https://github.com/snapcore/snapcraft/pull/3059>`__)
-  plugins: introduce v2.NpmPlugin (`#3060 <https://github.com/snapcore/snapcraft/pull/3060>`__)
-  plugins: introduce v2.RustPlugin (`#3061 <https://github.com/snapcore/snapcraft/pull/3061>`__)
-  cli: update command names to new design (`#3063 <https://github.com/snapcore/snapcraft/pull/3063>`__)
-  tests: fix local plugin spread test to be multi-arch aware (`#3065 <https://github.com/snapcore/snapcraft/pull/3065>`__)
-  cmake v2 plugin: rename configflags to cmake-parameters (`#3064 <https://github.com/snapcore/snapcraft/pull/3064>`__)
-  autotools v2 plugin: rename configflags to autotools-configure-parameters (`#3066 <https://github.com/snapcore/snapcraft/pull/3066>`__)
-  plugins v2: update plugins so they have a similar behavior (`#3070 <https://github.com/snapcore/snapcraft/pull/3070>`__)
-  storeapi: remove strict additionalProperties from store responses (`#3073 <https://github.com/snapcore/snapcraft/pull/3073>`__)
-  make v2 plugin: make use of make-parameters (`#3069 <https://github.com/snapcore/snapcraft/pull/3069>`__)
-  pluginhandler: skip plugin clean_pull for PluginV2 (`#3077 <https://github.com/snapcore/snapcraft/pull/3077>`__)
-  meson v2 plugin: ignore any staged python when installing meson (`#3078 <https://github.com/snapcore/snapcraft/pull/3078>`__)
-  cli: add plugin help for core20 (`#3079 <https://github.com/snapcore/snapcraft/pull/3079>`__)
-  make v2 plugin: also pass make-parameters to install (`#3081 <https://github.com/snapcore/snapcraft/pull/3081>`__)
-  cli: add list-plugins for core20 (`#3082 <https://github.com/snapcore/snapcraft/pull/3082>`__)
-  repo: revert logic to get deb_arch (`#3083 <https://github.com/snapcore/snapcraft/pull/3083>`__)
-  build providers: dist-upgrade the environment on bootstrap (`#3085 <https://github.com/snapcore/snapcraft/pull/3085>`__)
-  meta: remove snapd workaround for classic for core20 onwards (`#3087 <https://github.com/snapcore/snapcraft/pull/3087>`__)
-  repo: add interface to get packages from base (`#3088 <https://github.com/snapcore/snapcraft/pull/3088>`__)
-  ci: install the snapd snap when preparing spread systems
-  tests: add python3-wheel to staged python spread test
-  tests: remove MATCH from build-and-run-hello spread task
-  tests: add python3-minimal to python-staged spread
-  pluginhandler: cleanup before rebuilding for anything not PluginV1 
