.. 11651.md

.. _release-notes-snapcraft-3-5:

Release notes: Snapcraft 3.5
============================

These are the release notes for `Snapcraft 3.5 <https://github.com/snapcore/snapcraft/releases/tag/3.5>`__.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

New *core* features
-------------------

snapcraft promote
~~~~~~~~~~~~~~~~~

A new hidden command for this release is *promote*, which allows you to release a set of revisions as a *build set*.

A *build set* is a collection of snap revisions that meet a certain criteria, such as a set of revisions released to a channel.

For example. the following uses *promote* to move build set from beta to candidate:

.. code:: bash

   $ snapcraft promote --from-channel beta --to-channel candidate snapcraft
   snapcraft promote does not have a stable CLI interface. Use with caution in scripts.
   Build set information for 'beta'
   Arch       Revision    Version
   amd64          2947        3.5
   arm64          2952        3.5
   armhf          2951        3.5
   i386           2949        3.5
   ppc64el        2950        3.5
   s390x          2948        3.5
   Do you want to promote the current set to the 'candidate' channel? [y/N]:

..

   This feature is currently under development, which means the *promote* command syntax may change.

Full list of changes
--------------------

The issues and features worked on for 3.5 can be seen on the `3.5 <https://bugs.launchpad.net/snapcraft/+milestone/3.5>`__ launchpad milestone which are reflected in the following change list:

Sergio Schvezov
~~~~~~~~~~~~~~~

-  vcs: ignore .idea (`#2558 <https://github.com/snapcore/snapcraft/pull/2558>`__)
-  ci: remove dependency on LXD from travis tests (`#2557 <https://github.com/snapcore/snapcraft/pull/2557>`__)
-  cli: snapcraft promote (`#2556 <https://github.com/snapcore/snapcraft/pull/2556>`__) (LP: #1827513)
-  manifest: expose snapcraft-started-at (`#2559 <https://github.com/snapcore/snapcraft/pull/2559>`__) (LP: #1806658)
-  storeapi: allow promotion from branches
-  tests: release and promote to grade devel channels
-  tests: fix the status test
-  tests: add ppc64el to the fake server info results

Claudio Matsuoka
~~~~~~~~~~~~~~~~

-  project: read local plugins from build-aux (`#2551 <https://github.com/snapcore/snapcraft/pull/2551>`__) (LP: #1827095)
-  extensions: block direct use of private extensions (`#2555 <https://github.com/snapcore/snapcraft/pull/2555>`__)

Daniel Llewellyn
~~~~~~~~~~~~~~~~

-  meta: take ${SNAP} into account .desktop icon checks (`#2541 <https://github.com/snapcore/snapcraft/pull/2541>`__)


