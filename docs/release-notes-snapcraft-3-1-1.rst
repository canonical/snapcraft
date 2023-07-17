.. 10722.md

.. _release-notes-snapcraft-3-1-1:

Release notes: Snapcraft 3.1.1
==============================

These are the release notes for `Snapcraft 3.1.1 <https://github.com/snapcore/snapcraft/releases/tag/3.1.1>`__, which is primarily a bugifx release.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

| The issues and features worked on for 3.1.1:

- ant, maven and gradle plugins: use correct defaults for jre (`#2453 <https://github.com/snapcore/snapcraft/pull/2453>`__) (LP: #1813637, #1813636)
- rust plugin: new link for rustup (`#2438 <https://github.com/snapcore/snapcraft/pull/2438>`__) (LP: #1662960)
- baseplugin: add a proper exception for cross-compilation support (`#2454 <https://github.com/snapcore/snapcraft/pull/2454>`__) (LP: #1808454)
- clean: error out on invalid or missing yaml (`#2458 <https://github.com/snapcore/snapcraft/pull/2458>`__) (LP: #1777501)
- lifecycle: avoid installation of snaps in docker (`#2457 <https://github.com/snapcore/snapcraft/pull/2457>`__) (LP: #1814148)
- ci: use non virt-enabled gce instances for 16.04 (`#2461 <https://github.com/snapcore/snapcraft/pull/2461>`__)
- extractors: fix typo in code comment (`#2452 <https://github.com/snapcore/snapcraft/pull/2452>`__)
- pluginhandler: handle removal of inconsistent files (`#2450 <https://github.com/snapcore/snapcraft/pull/2450>`__) (LP: #1813033)
- cli: retrieve error data from provider (`#2455 <https://github.com/snapcore/snapcraft/pull/2455>`__) (LP: #1793082)
- build providers: recreate instance if base changed (`#2460 <https://github.com/snapcore/snapcraft/pull/2460>`__) (LP: #1794506)
- catkin plugin: describe how to build all packages (`#2459 <https://github.com/snapcore/snapcraft/pull/2459>`__)
