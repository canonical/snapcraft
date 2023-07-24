.. 10719.md

.. _release-notes-snapcraft-3-1:

Release notes: Snapcraft 3.1
============================

These are the release notes for `Snapcraft 3.1 <https://github.com/snapcore/snapcraft/releases/tag/3.1>`__, a minor release that builds on the foundations of :ref:`Snapcraft 3.0 <release-notes-snapcraft-3-0>`.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Build environments
------------------

When using the :ref:`base <base-snaps>` keyword, it is once again possible to clean parts:

.. code:: bash

   $ snapcraft clean <part-name>

Cleaning individual steps from a specific part, by adding ``--step`` to ``clean``, is being redesigned to be more intuitive and straightforward in its use.

New *core* features
-------------------

``before`` and ``after``
~~~~~~~~~~~~~~~~~~~~~~~~

``before`` and ``after`` keywords can now be used to order service launching within a snap.

AppStream extractor
~~~~~~~~~~~~~~~~~~~

The `AppStream <https://www.freedesktop.org/software/appstream/docs/>`__ metadata extractor can now properly handle tags inside the relevant nodes and properly filter ``xml:lang``.

Taking the following AppStream metadata as an example input:

.. code:: xml

     <description>
       <p>List:</p>
       <p xml:lang="es">Lista:</p>
       <ul>
         <li>First item.</li>
         <li xml:lang="es">Primer item.</li>
         <li>Second item.</li>
         <li xml:lang="es">Segundo item.</li>
       </ul>
     </description>

…would generate the following description in ``snap.yaml``:

.. code:: yaml

   List:

   - First item.
   - Second item.

Additionally, desktop files are now properly found from either the AppStream ``launchable`` entries, or by falling back to legacy mode and inferring the desktop file from the appstream ``id``.

Plugins
-------

cmake
~~~~~

The plugin can now use ``build-snaps`` within the build environment. When any given ``build-snaps`` entry exists for a part that uses the ``cmake`` plugin, the plugin will make use of ``CMAKE_FIND_ROOT_PATH`` so that libraries and headers from that snap are preferred.

Additionally, ``cmake`` primitives are now used to drive the build instead of just calling ``make``.

These features have already been used to create an initial set of KDE applications leveraging ``core18`` as a base as described on the `KDE apps at the snap of your fingers <https://snapcraft.io/blog/kde-apps-at-the-snap-of-your-fingers>`__ article.

rust
~~~~

The ``rust`` plugin has been refactored in a backwards compatible way to work better with the non-legacy ``rustup`` tool.

Platform updates
----------------

macOS
~~~~~

When using :command:`snapcraft` with Homebrew for the first time, if :command:`multipass` is not found, the user will be prompted to install it before proceeding.

Full list of changes
--------------------

The issues and features worked on for 3.1 can be seen on the `3.1 launchpad milestone <https://launchpad.net/snapcraft/+milestone/3.1>`__ which are reflected in the following change list:

-  cmake plugin: use native primitives (`#2397 <https://github.com/snapcore/snapcraft/pull/2397>`__)
-  cmake plugin: use build snaps to search paths (`#2399 <https://github.com/snapcore/snapcraft/pull/2399>`__)
-  static: update to the latest flake8 (`#2420 <https://github.com/snapcore/snapcraft/pull/2420>`__)
-  project: state file path change (`#2419 <https://github.com/snapcore/snapcraft/pull/2419>`__)
-  tests: do not use ``bash`` as a reserved package name on staging (`#2423 <https://github.com/snapcore/snapcraft/pull/2423>`__)
-  nodejs plugin: fail gracefully when a package.json is missing (`#2424 <https://github.com/snapcore/snapcraft/pull/2424>`__)
-  tests: use fixed version for idna in plainbox (`#2426 <https://github.com/snapcore/snapcraft/pull/2426>`__)
-  tests: remove obsolete snap and external tests (`#2421 <https://github.com/snapcore/snapcraft/pull/2421>`__)
-  snap: re-add pyc files for snapcraft (`#2425 <https://github.com/snapcore/snapcraft/pull/2425>`__)
-  tests: increase test timeout for plainbox (`#2428 <https://github.com/snapcore/snapcraft/pull/2428>`__)
-  lifecycle: query for multipass install on darwin (`#2427 <https://github.com/snapcore/snapcraft/pull/2427>`__)
-  cli: fix usage string in help command (`#2429 <https://github.com/snapcore/snapcraft/pull/2429>`__)
-  repo: document package purpose (`#2390 <https://github.com/snapcore/snapcraft/pull/2390>`__)
-  extractors: better appstream support for descriptions (`#2430 <https://github.com/snapcore/snapcraft/pull/2430>`__)
-  tests: re-enable spread tests on gce
-  rust plugin: refactor to use the latest rustup
-  tests: temporarily disable osx tests
-  snap: add build-package for xml
-  appstream extractor: properly find desktop files
-  appstream extractor: support legacy launchables
-  snap: add xslt dependencies for lxml
-  repo,baseplugin: support trusting repo keys (`#2437 <https://github.com/snapcore/snapcraft/pull/2437>`__)
-  schema: allow before and after (`#2443 <https://github.com/snapcore/snapcraft/pull/2443>`__)
-  meta: make hooks executable instead of complaining they are not (`#2440 <https://github.com/snapcore/snapcraft/pull/2440>`__)
-  build providers: remove SIGUSR1 signal ignore workaround for multipass (`#2447 <https://github.com/snapcore/snapcraft/pull/2447>`__)
-  cli: enable cleaning of parts (`#2442 <https://github.com/snapcore/snapcraft/pull/2442>`__)
-  tests: appstream unit tests are xenial specific
-  tests: skip rust unit tests on s390x
-  tests: use more fine grained assertions in lifecycle tests
-  tests: remove rust revision testing for i386


