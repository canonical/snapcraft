.. 10704.md

.. _release-notes-snapcraft-3-0:

Release notes: Snapcraft 3.0
============================

These are the release notes for `Snapcraft 3.0 <https://github.com/snapcore/snapcraft/releases/tag/3.0>`__, a major overhaul of the snap build environment.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Build environments
------------------

This release adds specific build environments for each snapcraft project you work on. These environments are tuned for each project, and ensure both API and ABI compatibility are in place for every binary built within each respective environment.

Snapcraft’s build environments leverage a snap architecture feature called :ref:`bases <base-snaps>`. At build time, the *snapcraft* tool ensures you are creating your applications inside an environment specifically tailored for the specified base.

To make the transition to Snapcraft 3.0 easy, the entire functionality for this new tool behavior is triggered by making use of the ``base`` keyword in :file:`snapcraft.yaml`.

Snapcraft 3.0 also remains backwards compatible. This means you can omit the ``base`` keyword and continue working as you did previously until you are ready to move to a newer or different stack provided by a different *base*.

The environment runs in a container, which means it’s isolated from the user during normal operation. However, the following commands enable you to step into this encapsulated environment:

-  ``--shell``: builds your snap to the :ref:`lifecycle step <parts-lifecycle-steps>` prior to that specified and opens a shell into the environment (e.g. running ``snapcraft prime --shell`` will run up to the ``stage`` step and open a shell).
-  ``--shell-after``: builds your snap to the :ref:`lifecycle step <parts-lifecycle-steps>` specified and opens a shell into the environment. (eg. running ``snapcraft prime --shell-after`` will run up to the ``prime`` step and then drop into a shell).
-  ``--debug``, opens a shell inside the environment after an error occur.

The below video shows an example of how the system behaves with the new functionality in place:

|asciicast|

macOS support
-------------

With this release of Snapcraft 3.0, we are happy to announce support for macOS via `Homebrew <https://formulae.brew.sh/formula/snapcraft>`__. Moreover, the experience is transparent thanks to the use of build environments and its underlying technology.

See :ref:`installing-snapcraft-macos` for further details.


.. _release-notes-snapcraft-3-0-base-exceptions:

Features incompatible with bases
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using the ``base`` keyword in :file:`snapcraft.yaml` the following (long deprecated) features become unavailable:

-  *wiki parts*, and their quirks in the code base, that enabled specific corner cases, such as allowing ``/`` in parts.
-  ``snapcraft cleanbuild`` and triggering builds with LXD in certain environment variables.
-  ``prepare``, ``build`` and ``install`` in parts. These have been replaced by ``override-build`` and ``snapcraftctl``. These offer ``override-`` for ``pull``, ``stage`` and ``prime`` too.
-  The ``snap`` keyword has been superseded by the ``prime`` keyword.
-  ``--disable-parallel-build`` is no longer available when calling build commands through *snapcraft*. It can be setup, per part, using the ``build-attributes`` property.
-  ``--use-geoip`` is no longer available when calling build commands through *snapcraft*. This affected ``stage-packages``.

In addition to the above, the following plugins also become unavailable when using the ``base`` keyword:

-  ``tar-content``, superseded by the ``dump`` plugin.
-  ``copy``, superseded by the ``dump`` plugin.
-  ``jhbuild``, it has too many dependencies against ``core``.

New *core* features
-------------------

The following are new when using ``base``:

License
~~~~~~~

A snap’s license can now be defined using the new *license* keyword and a `SPDX 2.0 format <https://spdx.org/licenses/>`__ value within :file:`snapcraft.yaml`.

Validation of the license syntax is done using ``snap pack`` schema validation to ensure consistency across the snap ecosystem.

Wrapperless-snaps
~~~~~~~~~~~~~~~~~

When originally introduced, *wrappers* were useful when setting up the environment appropriately. However, the runtime architecture has now evolved in such a way that wrappers no longer make sense.

Although it is not the default today, the intention is to make the ``adapter`` value (``full``) the default behaviour. The advantage with this design is that when entering a shell through ``snap run --shell``, the environment would be properly loaded through use of ``command-chain`` entries in ``snap.yaml``.

The recording below shows how the original ``command``, defined in :file:`snapcraft.yaml`, is still the ``command``\ that makes it to ``snap.yaml`` (and the ``command-chain`` feature is used instead):

|image1|


.. _release-notes-snapcraft-3-0-extensions:

Extensions
~~~~~~~~~~

The architecture and framework has been cemented into the :command:`snapcraft` tool to help :file:`snapcraft.yaml` grow declarative functionality we call *Extensions*. We have done this to avoid repetitive tasks, and to avoid snap builders needing deep knowledge of a target software stack.

Extensions have the unique property of being applied to :file:`snapcraft.yaml` itself, where they can be expanded upon and, potentially, used in lieu of the extension itself. This would allow for project-specific modifications of the extension.

You can interact with extensions using the following new commands:

- ``list-extensions``, to view the available extensions.
- ``extension``, to show information about the extension.
- ``expand-extensions``, to display how the :file:`snapcraft.yaml` file will look like with the extensions applied.

Lifecycle cleaning
~~~~~~~~~~~~~~~~~~

Prior to Snapcraft 3.0, you needed to manually clean any *part* that was found to be dirty due to modifications in the code itself, or because modifications had been made to the *part* definition in :file:`snapcraft.yaml`. This become an unnecessary burden for developers.

The default action for *snapcraft* to now rebuild parts, either by re-running a lifecycle step without cleaning, for plugins that allow for it (through their underlying architecture), or automatically cleaning and re-running the necessary lifecycle steps for that part.

You can see this in action below:

|image2|

Implicit source
~~~~~~~~~~~~~~~

Previously, if a part did not specify a source, an implicit default source of ``.`` was set by default. This caused considerable confusion.

Starting with Snapcraft 3.0, if a plugins requires a source to be specified, it will be required through the schema and an appropriate error message will be generated.

For plugins where a source isn’t a requirement, such as ``nil``, no action will be taken and no default will be set.

Plugins
-------

With the exception of deprecated and removed plugins, the majority of plugins have been reworked to be ``base`` aware.

Since the declaration of the ``base`` keyword is done manually by the user, some plugins have introduced semantic changes for how they operate.

Below is the set of plugins with interesting changes and new properties available to the user:

ant
~~~

These are the properties the ``ant`` plugin now operates with:

::

       - ant-properties:
         (object)
         A dictionary of key-value pairs. Set the following properties when
         running ant.

       - ant-build-targets:
         (list of strings)
         Run the given ant targets.

       - ant-version:
         (string)
         The version of ant you want to use to build the source artifacts.
         Defaults to the current release downloadable from
         https://archive.apache.org/dist/ant/binaries/.

       - ant-version-checksum:
         (string)
         The checksum for ant-version in the form of <digest-type>/<digest>.
         As an example "sha512/2a803f578f341e164f6753e410413d16ab60fab...".

       - ant-openjdk-version:
         (string)
         openjdk version available to the base to use. If not set the latest
         version available to the base will be used.

``catkin``
~~~~~~~~~~

These are the properties the ``catkin`` plugin now operates with:

::

       - catkin-packages:
         (list of strings)
         List of catkin packages to build.
       - source-space:
         (string)
         The source space containing Catkin packages. By default this is 'src'.
       - include-roscore:
         (boolean)
         Whether or not to include roscore with the part. Defaults to true.
       - rosinstall-files:
         (list of strings)
         List of rosinstall files to merge while pulling. Paths are relative to
         the source.
       - recursive-rosinstall:
         (boolean)
         Whether or not to recursively merge/update rosinstall files from fetched
         sources. Will continue until all rosinstall files have been merged.
         Defaults to false.
       - catkin-cmake-args:
         (list of strings)
         Configure flags to pass onto the cmake invocation from catkin.
       - underlay:
         (object)
         Used to inform Snapcraft that this snap isn't standalone, and is actually
         overlaying a workspace from another snap via content sharing. Made up of
         two properties:
         - build-path:
           (string)
           Build-time path to existing workspace to underlay the one being built,
           for example '$SNAPCRAFT_STAGE/opt/ros/kinetic'.
         - run-path:
           (string)
           Run-time path of the underlay workspace (e.g. a subdirectory of the
           content interface's 'target' attribute.)
       - catkin-ros-master-uri:
         (string)
         The URI to ros master setting the env variable ROS_MASTER_URI. Defaults
         to http://localhost:11311.

``go``
~~~~~~

These are the properties the ``go`` plugin now operates with:

::

       - go-channel:
         (string, default: latest/stable)
         The Snap Store channel to install go from. If set to an empty string,
         go will be installed using the system's traditional package manager.

       - go-packages:
         (list of strings)
         Go packages to fetch, these must be a "main" package. Dependencies
         are pulled in automatically by `go get`.
         Packages that are not "main" will not cause an error, but would
         not be useful either.
         If the package is a part of the go-importpath the local package
         corresponding to those sources will be used.

       - go-importpath:
         (string)
         This entry tells the checked out `source` to live within a certain path
         within `GOPATH`.
         This is not needed and does not affect `go-packages`.

       - go-buildtags:
         (list of strings)
         Tags to use during the go build. Default is not to use any build tags.

``godeps``
~~~~~~~~~~

These are the properties the ``godeps`` plugin now operates with:

::

       - go-channel:
         (string, default: latest/stable)
         The Snap Store channel to install go from. If set to an empty string,
         go will be installed using the system's traditional package manager.

       - go-packages:
         (list of strings)
         Go packages to build/install, these must be a "main" package.
         Dependencies should have already been retrieved by the `godeps-file`
         used for this part.
         Packages that are not "main" will not cause an error, but would
         not be useful either.

       - godeps-file:
         (string)
         Path to the godeps dependencies file contained within the source
         (default: dependencies.tsv)

       - go-importpath:
         (string)
         This entry tells the checked out `source` to live within a certain path
         within `GOPATH`. This is required in order to work with absolute imports
         and import path checking.

``gradle``
~~~~~~~~~~

These are the properties the ``gradle`` plugin now operates with:

::

       - gradle-options:
         (list of strings)
         Flags to pass to the build using the gradle semantics for parameters.
         The 'jar' option is always passed in as the last parameter.

       - gradle-output-dir:
         (string; default: 'build/libs')
         The output directory where the resulting jar or war files from gradle[w]
         are generated.

       - gradle-version:
         (string)
         The version of gradle you want to use to build the source artifacts.
         Defaults to the current release downloadable from
         https://services.gradle.org/distributions/
         The entry is ignored if gradlew is found.

       - gradle-version-checksum:
         (string)
         The checksum for gradle-version in the form of <digest-type>/<digest>.
         As an example "sha512/2a803f578f341e164f6753e410413d16ab60fab...".

       - gradle-openjdk-version:
         (string)
         openjdk version available to the base to use. If not set the latest
         version available to the base will be used.

``meson``
~~~~~~~~~

These are the properties the ``meson`` plugin now operates with:

::

       - meson-version:
         (string)
         The version of meson to install from PyPI.
         If unspecified, the latest released version of meson will be used.
       - meson-parameters:
         (list of strings)
         Pass the given parameters to the meson command.

``nodejs``
~~~~~~~~~~

These are the properties the ``nodejs`` plugin now operates with:

::

       - nodejs-version:
         (string)
         The version of nodejs you want the snap to run on.
         This includes npm, as would be downloaded from https://nodejs.org
         Defaults to the current LTS release.

       - nodejs-package-manager
         (string; default: yarn)
         The language package manager to use to drive installation
         of node packages. Can be either `npm` or `yarn` (default).

       - nodejs-yarn-version:
         (string)
         Applicable when using yarn. Defaults to the latest if not set.

``python``
~~~~~~~~~~

These are the properties the ``python`` plugin now operates with:

::

       - requirements:
         (list of strings)
         List of paths to requirements files.

       - constraints:
         (list of strings)
         List of paths to constraint files.

       - process-dependency-links:
         (bool; default: false)
         Enable the processing of dependency links in pip, which allow one
         project to provide places to look for another project

       - python-packages:
         (list)
         A list of dependencies to get from PyPI

       - python-version:
         (string; default: python3)
         The python version to use. Valid options are: python2 and python3

Full list of changes
--------------------

The issues and features worked on for 3.0 can be seen on the `3.0 launchpad milestone <https://launchpad.net/snapcraft/+milestone/3.0>`__ which are reflected in the following change list:

-  snap: add the https transport (`#2244 <https://github.com/snapcore/snapcraft/pull/2244>`__)
-  build providers: environment setup for projects (`#2225 <https://github.com/snapcore/snapcraft/pull/2225>`__)
-  build providers: provide support to shell in (`#2249 <https://github.com/snapcore/snapcraft/pull/2249>`__)
-  build providers: shell in provider if debug is used (`#2252 <https://github.com/snapcore/snapcraft/pull/2252>`__)
-  build-providers: add support for –shell-after (`#2253 <https://github.com/snapcore/snapcraft/pull/2253>`__)
-  build providers: add support for –shell (`#2254 <https://github.com/snapcore/snapcraft/pull/2254>`__)
-  build providers: snapcraft images for multipass (`#2258 <https://github.com/snapcore/snapcraft/pull/2258>`__)
-  build providers: allow setting ram and disk size (`#2260 <https://github.com/snapcore/snapcraft/pull/2260>`__)
-  build providers: inject the base for classic (`#2261 <https://github.com/snapcore/snapcraft/pull/2261>`__)
-  build providers: allow snapcraft channel selection (`#2265 <https://github.com/snapcore/snapcraft/pull/2265>`__)
-  build providers: refresh packages on bring up (`#2267 <https://github.com/snapcore/snapcraft/pull/2267>`__)
-  build providers: let the implementor pick the image (`#2269 <https://github.com/snapcore/snapcraft/pull/2269>`__)
-  reporting: fail gracefully on submit errors (`#2271 <https://github.com/snapcore/snapcraft/pull/2271>`__)
-  meta: friendlier message for incorrect app command (`#2272 <https://github.com/snapcore/snapcraft/pull/2272>`__)
-  snap: use a newer PyYAML and drop patches (`#2274 <https://github.com/snapcore/snapcraft/pull/2274>`__)
-  build providers: use the best CPU configuration (`#2273 <https://github.com/snapcore/snapcraft/pull/2273>`__)
-  build providers: use the provider if exported (`#2275 <https://github.com/snapcore/snapcraft/pull/2275>`__)
-  snap: move to a newer pysha3 (`#2277 <https://github.com/snapcore/snapcraft/pull/2277>`__)
-  spread: move legacy wiki tests to spread (`#2276 <https://github.com/snapcore/snapcraft/pull/2276>`__)
-  snap: pull early (`#2278 <https://github.com/snapcore/snapcraft/pull/2278>`__)
-  build providers: re-exec as root (`#2281 <https://github.com/snapcore/snapcraft/pull/2281>`__)
-  build providers: cleaner start and launch messaging (`#2282 <https://github.com/snapcore/snapcraft/pull/2282>`__)
-  build providers: make use of time for multipass stop (`#2284 <https://github.com/snapcore/snapcraft/pull/2284>`__)
-  meta: support relocatable prime for path verification (`#2287 <https://github.com/snapcore/snapcraft/pull/2287>`__)
-  build providers: use multipass automatically when on darwin (`#2288 <https://github.com/snapcore/snapcraft/pull/2288>`__)
-  snap: workaround the dirty tree (`#2294 <https://github.com/snapcore/snapcraft/pull/2294>`__)
-  tests: use SNAPCRAFT_PACKAGE_TYPE everywhere (`#2295 <https://github.com/snapcore/snapcraft/pull/2295>`__)
-  tests: move most tests to spread and reorder travis.yaml (`#2301 <https://github.com/snapcore/snapcraft/pull/2301>`__)
-  snap: improve early base detection logic (`#2309 <https://github.com/snapcore/snapcraft/pull/2309>`__)
-  meta: link the icon correctly across filesystems (`#2313 <https://github.com/snapcore/snapcraft/pull/2313>`__)
-  project loader: remove remote parts support for bases (`#2304 <https://github.com/snapcore/snapcraft/pull/2304>`__)
-  tests: use mocked plugins for list-plugins (`#2315 <https://github.com/snapcore/snapcraft/pull/2315>`__)
-  tests: add spread suite for plainbox plugin (`#2317 <https://github.com/snapcore/snapcraft/pull/2317>`__)
-  plugins: remove the tar-content plugin when using a base (`#2319 <https://github.com/snapcore/snapcraft/pull/2319>`__)
-  plugins: remove the copy plugin when using a base (`#2308 <https://github.com/snapcore/snapcraft/pull/2308>`__)
-  meta: add support for the license field (`#2318 <https://github.com/snapcore/snapcraft/pull/2318>`__)
-  build providers: use the new snapcraft: remote for multipass (`#2293 <https://github.com/snapcore/snapcraft/pull/2293>`__)
-  plugins: remove the python2 and python3 plugin when using a base (`#2325 <https://github.com/snapcore/snapcraft/pull/2325>`__)
-  plugins: remove the ament plugin when using a base (`#2324 <https://github.com/snapcore/snapcraft/pull/2324>`__)
-  plugins: remove implicit source (`#2326 <https://github.com/snapcore/snapcraft/pull/2326>`__)
-  go plugin: support for bases (`#2323 <https://github.com/snapcore/snapcraft/pull/2323>`__)
-  pluginhandler: remove legacy plugin loading without project (`#2329 <https://github.com/snapcore/snapcraft/pull/2329>`__)
-  godeps plugin: support for bases (`#2328 <https://github.com/snapcore/snapcraft/pull/2328>`__)
-  pluginhandler: remove big solidus workaround (`#2330 <https://github.com/snapcore/snapcraft/pull/2330>`__)
-  pluginhandler: remove prepare, build and install scriptlets (`#2327 <https://github.com/snapcore/snapcraft/pull/2327>`__)
-  waf plugin: support for bases (`#2332 <https://github.com/snapcore/snapcraft/pull/2332>`__)
-  meson plugin: add support for bases (`#2331 <https://github.com/snapcore/snapcraft/pull/2331>`__)
-  lifecycle: remove lxd support for bases (`#2335 <https://github.com/snapcore/snapcraft/pull/2335>`__)
-  tests: remove dependency on snapcraft for integration tests (`#2353 <https://github.com/snapcore/snapcraft/pull/2353>`__)
-  schema: enforce string for versions (`#2334 <https://github.com/snapcore/snapcraft/pull/2334>`__)
-  lifecycle: switch to multipass by default (`#2339 <https://github.com/snapcore/snapcraft/pull/2339>`__)
-  schema: remove the deprecated snap keyword for bases (`#2344 <https://github.com/snapcore/snapcraft/pull/2344>`__)
-  tests: use valid snap names in unit tests (`#2352 <https://github.com/snapcore/snapcraft/pull/2352>`__)
-  scons plugin: add support for bases (`#2357 <https://github.com/snapcore/snapcraft/pull/2357>`__)
-  nodejs plugin: add support for bases (`#2356 <https://github.com/snapcore/snapcraft/pull/2356>`__)
-  pluginhandler: library detection instead of injection (`#2337 <https://github.com/snapcore/snapcraft/pull/2337>`__)
-  dotnet plugin: add support for bases (`#2358 <https://github.com/snapcore/snapcraft/pull/2358>`__)
-  schema: remove deprecated plugin pull and build-properties (`#2361 <https://github.com/snapcore/snapcraft/pull/2361>`__)
-  plainbox-provider plugin: add support for bases (`#2360 <https://github.com/snapcore/snapcraft/pull/2360>`__)
-  multipass: change default CPU value (`#2365 <https://github.com/snapcore/snapcraft/pull/2365>`__)
-  python plugin: add support for bases (`#2362 <https://github.com/snapcore/snapcraft/pull/2362>`__)
-  maven plugin: add support for bases (`#2364 <https://github.com/snapcore/snapcraft/pull/2364>`__)
-  gradle plugin: add support for bases (`#2372 <https://github.com/snapcore/snapcraft/pull/2372>`__)
-  ant plugin: add support for bases (`#2370 <https://github.com/snapcore/snapcraft/pull/2370>`__)
-  jdk plugin: remove jdk (`#2376 <https://github.com/snapcore/snapcraft/pull/2376>`__)
-  build providers: destroy on create failures (`#2374 <https://github.com/snapcore/snapcraft/pull/2374>`__)
-  cli: remove disable-parallel-build and geoip toggles (`#2377 <https://github.com/snapcore/snapcraft/pull/2377>`__)
-  yaml loading: properly handle unhashable types (`#2247 <https://github.com/snapcore/snapcraft/pull/2247>`__)
-  pluginhandler: stop using alias for snapcraftctl (`#2251 <https://github.com/snapcore/snapcraft/pull/2251>`__)
-  local source: don’t include .snapcraft directory (`#2256 <https://github.com/snapcore/snapcraft/pull/2256>`__)
-  meta: take charge of environment used to run commands (`#2257 <https://github.com/snapcore/snapcraft/pull/2257>`__)
-  cli: show trace if no tty (`#2259 <https://github.com/snapcore/snapcraft/pull/2259>`__)
-  catkin plugin: use SnapcraftException (`#2255 <https://github.com/snapcore/snapcraft/pull/2255>`__)
-  project_loader: add preflight check (`#2250 <https://github.com/snapcore/snapcraft/pull/2250>`__)
-  project: catch parent YAML exceptions (`#2263 <https://github.com/snapcore/snapcraft/pull/2263>`__)
-  tests: disable integration tests using snaps in bionic container (`#2266 <https://github.com/snapcore/snapcraft/pull/2266>`__)
-  catkin, rosdep: stop using FileNotFoundErrors (`#2270 <https://github.com/snapcore/snapcraft/pull/2270>`__)
-  coherence checks: allow snap/local dir (`#2268 <https://github.com/snapcore/snapcraft/pull/2268>`__)
-  coherence checks: run properly on build VMs (`#2279 <https://github.com/snapcore/snapcraft/pull/2279>`__)
-  snapcraft snap: refactor override-build into a script (`#2283 <https://github.com/snapcore/snapcraft/pull/2283>`__)
-  config: change default outdated action to clean (`#2286 <https://github.com/snapcore/snapcraft/pull/2286>`__)
-  snapcraft snap: vendor legacy snapcraft (`#2285 <https://github.com/snapcore/snapcraft/pull/2285>`__)
-  schema: add “legacy” adapter type (`#2262 <https://github.com/snapcore/snapcraft/pull/2262>`__)
-  sources: properly handle pull failures (`#2292 <https://github.com/snapcore/snapcraft/pull/2292>`__)
-  packaging: pin click to v6 in requirements.txt (`#2298 <https://github.com/snapcore/snapcraft/pull/2298>`__)
-  meta: put environment into runner instead of app wrapper (`#2291 <https://github.com/snapcore/snapcraft/pull/2291>`__)
-  part grammar processor: lazily capture attributes from plugin (`#2296 <https://github.com/snapcore/snapcraft/pull/2296>`__)
-  pluginhandler: update build should overwrite organize (`#2290 <https://github.com/snapcore/snapcraft/pull/2290>`__)
-  requirements.txt: stop using pymacaroons-pynacl (`#2302 <https://github.com/snapcore/snapcraft/pull/2302>`__)
-  project_loader: add build-environment part property (`#2322 <https://github.com/snapcore/snapcraft/pull/2322>`__)
-  catkin, catkin-tools plugins: add support for bases (`#2333 <https://github.com/snapcore/snapcraft/pull/2333>`__)
-  schema, meta: support layout (`#2338 <https://github.com/snapcore/snapcraft/pull/2338>`__)
-  schema, meta: support app command-chain (`#2341 <https://github.com/snapcore/snapcraft/pull/2341>`__)
-  schema, meta: add “full” app adapter (`#2343 <https://github.com/snapcore/snapcraft/pull/2343>`__)
-  ruby plugin: add support for base (`#2346 <https://github.com/snapcore/snapcraft/pull/2346>`__)
-  extensions: support adding root properties (`#2347 <https://github.com/snapcore/snapcraft/pull/2347>`__)
-  extensions: remove root extensions (`#2348 <https://github.com/snapcore/snapcraft/pull/2348>`__)
-  extensions: use extension docstring (`#2349 <https://github.com/snapcore/snapcraft/pull/2349>`__)
-  extensions: parse all declared extensions before applying (`#2350 <https://github.com/snapcore/snapcraft/pull/2350>`__)
-  extensions: cleanup and generic tests (`#2355 <https://github.com/snapcore/snapcraft/pull/2355>`__)
-  {make,cmake,autotools} plugin: add support for bases (`#2363 <https://github.com/snapcore/snapcraft/pull/2363>`__)
-  qmake plugin: add support for bases (`#2366 <https://github.com/snapcore/snapcraft/pull/2366>`__)
-  {kbuild,kernel} plugin: add support for bases (`#2368 <https://github.com/snapcore/snapcraft/pull/2368>`__)
-  tests: add spread test exercising multipass build VMs (`#2367 <https://github.com/snapcore/snapcraft/pull/2367>`__)
-  plugins: remove jhbuild (`#2371 <https://github.com/snapcore/snapcraft/pull/2371>`__)
-  rust plugin: add support for bases (`#2373 <https://github.com/snapcore/snapcraft/pull/2373>`__)
-  coherence checks: verify that command-chain is not used with legacy adapter (`#2375 <https://github.com/snapcore/snapcraft/pull/2375>`__)
-  cli: use the better snapcraft.io/account URL (`#2280 <https://github.com/snapcore/snapcraft/pull/2280>`__)
-  storeapi: use structured data for the conflicted current value (`#2316 <https://github.com/snapcore/snapcraft/pull/2316>`__)
-  rust plugin: do not ignore the cross compile target (`#2264 <https://github.com/snapcore/snapcraft/pull/2264>`__)
-  nodejs plugin: add support for ppc64el and s390x (`#2310 <https://github.com/snapcore/snapcraft/pull/2310>`__) (`#2310 <https://github.com/snapcore/snapcraft/pull/2310>`__)
-  nodejs plugin: update to the latest 8.x LTS version (`#2342 <https://github.com/snapcore/snapcraft/pull/2342>`__)
-  yaml: replace yaml.safe_load() with CSafeLoader (`#2218 <https://github.com/snapcore/snapcraft/pull/2218>`__) 

.. |asciicast| image:: https://camo.githubusercontent.com/e31b1f2e1d5512c3fc67993c17630e0ce7f945d4/68747470733a2f2f61736369696e656d612e6f72672f612f74353437663049744279367057336b43534c786456304a564c2e737667
   :target: https://asciinema.org/a/t547f0ItBy6pW3kCSLxdV0JVL
.. |image1| image:: https://camo.githubusercontent.com/d59f96e630f91f32d4ba690af9238717ad3aeaed/68747470733a2f2f61736369696e656d612e6f72672f612f74713844325268455862734e596b5644567a35457a594448302e737667
   :target: https://asciinema.org/a/tq8D2RhEXbsNYkVDVz5EzYDH0
.. |image2| image:: https://camo.githubusercontent.com/4309e0614246524ff2bff73ea44170bb83bc2935/68747470733a2f2f61736369696e656d612e6f72672f612f6c33674e355179517933446771304b7a50353253644e7631712e737667
   :target: https://asciinema.org/a/l3gN5QyQy3Dgq0KzP52SdNv1q
