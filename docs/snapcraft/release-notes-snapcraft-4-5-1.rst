.. 22788.md

.. _release-notes-snapcraft-4-5-1:

Release notes: Snapcraft 4.5.1
==============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.5.1 <https://github.com/snapcore/snapcraft/releases/tag/4.5.1>`__.

Highlights for this release include:

-  more improvements for using python3.8 from within a snap
-  allow revoking validation assertions
-  experimental notice for compression has been removed
-  SDK snap paths in ACLOCAL_PATH are now included

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 4.5 are reflected in the following change list:

- More improvements for using python3.8 from within a snap `@kenvandine <https://github.com/kenvandine>`__ (`#3430 <https://github.com/snapcore/snapcraft/pull/3430>`__)
- Allow revoking validation assertions (LP: #1912332) `@nessita <https://github.com/nessita>`__ (`#3433 <https://github.com/snapcore/snapcraft/pull/3433>`__)
- spread tests: remove legacy plugin tests `@cjp256 <https://github.com/cjp256>`__ (`#3432 <https://github.com/snapcore/snapcraft/pull/3432>`__)
- godeps spread test: use latest/stable go snap `@cjp256 <https://github.com/cjp256>`__ (`#3431 <https://github.com/snapcore/snapcraft/pull/3431>`__)
- plugins v1: Pin pip to supported versions `@philroche <https://github.com/philroche>`__ (`#3428 <https://github.com/snapcore/snapcraft/pull/3428>`__)
- cli: remove experimental notice for compression `@cjp256 <https://github.com/cjp256>`__ (`#3421 <https://github.com/snapcore/snapcraft/pull/3421>`__)
- Ensure PYTHONPATH is appropriate for building packages with gnome-3-38 `@kenvandine <https://github.com/kenvandine>`__ (`#3424 <https://github.com/snapcore/snapcraft/pull/3424>`__)
- Ensure PYTHONPATH is properly set for gnome-3-34 builds `@kenvandine <https://github.com/kenvandine>`__ (`#3426 <https://github.com/snapcore/snapcraft/pull/3426>`__)
- Revert “cli: allow validation assertions to be revoked (`#3417 <https://github.com/snapcore/snapcraft/pull/3417>`__)” `@sergiusens <https://github.com/sergiusens>`__ (`#3422 <https://github.com/snapcore/snapcraft/pull/3422>`__)
- Include SDK snap paths in ACLOCAL_PATH `@kenvandine <https://github.com/kenvandine>`__ (`#3419 <https://github.com/snapcore/snapcraft/pull/3419>`__)
- plainbox spread tests: set tasks to manual `@cjp256 <https://github.com/cjp256>`__ (`#3420 <https://github.com/snapcore/snapcraft/pull/3420>`__) 
