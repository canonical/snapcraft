.. 23142.md

.. _release-notes-snapcraft-4-5-2:

Release notes: Snapcraft 4.5.2
==============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.5.2 <https://github.com/snapcore/snapcraft/releases/tag/4.5.2>`__.

Highlights for this release include:

-  more improvements to the Python v2 plugin
-  specify arch-specific bundle directories with the Flutter plugin
-  build providers will now clean the build environment when the project directory changes

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Full list of changes
--------------------

The issues and features worked on for this release are reflected in the following change list:

- extensions: Fix Documents, Pictures etc symlinks `@diddledan <https://github.com/diddledan>`__ (`#3435 <https://github.com/snapcore/snapcraft/pull/3435>`__)
- python v2 plugin: fix typo restoring shell state `@cjp256 <https://github.com/cjp256>`__ (`#3441 <https://github.com/snapcore/snapcraft/pull/3441>`__)
- extensions: support fontless systems in configure hook `@kenvandine <https://github.com/kenvandine>`__ (`#3439 <https://github.com/snapcore/snapcraft/pull/3439>`__)
- flutter: specify arch specific bundle dirs `@kenvandine <https://github.com/kenvandine>`__ (`#3438 <https://github.com/snapcore/snapcraft/pull/3438>`__)
- repo: apt sources management refactor `@cjp256 <https://github.com/cjp256>`__ (`#3363 <https://github.com/snapcore/snapcraft/pull/3363>`__)
- python v2 plugin: consistent linking for interpreter `@cjp256 <https://github.com/cjp256>`__ (`#3320 <https://github.com/snapcore/snapcraft/pull/3320>`__)
- repo: address issue with fix_symlink() when pointed at directory `@cjp256 <https://github.com/cjp256>`__ (`#3370 <https://github.com/snapcore/snapcraft/pull/3370>`__)
- build providers: clean environment if project directory is changed `@cjp256 <https://github.com/cjp256>`__ (`#3434 <https://github.com/snapcore/snapcraft/pull/3434>`__) 
