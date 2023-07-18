.. 24252.md

.. _release-notes-snapcraft-4-7:

Release notes: Snapcraft 4.7
============================

Snapcraft 4.7 is a feature-packed release, including:

-  Validation sets for gated refresh control
-  UA/ESM token support
-  More architectures for the conda plugin and ``core20``
-  *Desktop* extension improvements around fonts

For general details, including installation instructions, see `Snapcraft overview <https://snapcraft.io/docs/snapcraft-overview>`__, or take a look at `Snapcraft release notes <https://snapcraft.io/docs/snapcraft-release-notes>`__ for other *Snapcraft* releases.

Validation sets
---------------

A validation set is an `assertion <https://snapcraft.io/docs/assertions>`__ that lists specific snaps that are either required to be installed together or are permitted to be installed together on a device or system.

This release of Snapcraft adds two commands for working with validation sets:

-  list-validation-sets
-  edit-validation-sets

For more details, see :ref:`Validation sets <validation-sets>`.

UA Token
--------

At the end of April, Ubuntu 16.04 LTS reached the end of its five years of mainstream support and entered the `Extended Security Maintenance <https://ubuntu.com/security/esm>`__ (ESM) phase.

To be able to continue building Ubuntu 16.04 LTS using the ESM base for local and on-premise builds, snap publishers and developers will need to obtain UA tokens. `These tokens are free <https://ubuntu.com/blog/ua-services-deployed-from-the-command-line-with-ua-client>`__ for all community users, for up to three machines, and up to 50 machines for Ubuntu members.

With this release of snapcraft, a new ``--ua-token`` argument can be used to specify a token:

.. code:: bash

   snapcraft <step> --ua-token <token>

See :ref:`Snapcraft and Extended Security Maintenance <snapcraft-and-extended-security-maintenance>` for further details.

Conda plugin
------------

When using *core20*, the recently introduced :ref:`conda plugin <the-conda-plugin>` now supports more architectures, with the new ones being:

-  i386 (x86)
-  armhf (armv7l)
-  ppc64el (ppc64le)

Extension improvements
----------------------

:ref:`Extensions <snapcraft-extensions>` now have better font handling by integrating a new snapd feature to not expose the host font cache to the snap when using the desktop related extensions.

General cleanup into the *launcher* script which ensures a proper environment has been setup is also part of this release.

Store *whoami* migration
------------------------

The ``snapcraft whoami`` command has fully migrated to the store ``whoami`` endpoint, enabling logged in users, either with the existing flow or the experimental one, to query for their identity.

Stage Snaps
-----------

The :ref:`stage-snaps <build-and-staging-dependencies>` keyword now allows specifying channel branches. This solves a long standing request.

Full list of changes
--------------------

-  cli: introduce edit-validation-sets `@sergiusens <https://github.com/sergiusens>`__ (`#3512 <https://github.com/snapcore/snapcraft/pull/3512>`__)
-  cli: introduce list-validation-sets `@sergiusens <https://github.com/sergiusens>`__ (`#3510 <https://github.com/snapcore/snapcraft/pull/3510>`__)
-  extensions: don’t expose host system fontconfig cache `@jhenstridge <https://github.com/jhenstridge>`__ (`#3509 <https://github.com/snapcore/snapcraft/pull/3509>`__)
-  storeapi: add binding for validations-sets `@sergiusens <https://github.com/sergiusens>`__ (`#3508 <https://github.com/snapcore/snapcraft/pull/3508>`__)
-  storeapi: add classes for validation sets `@sergiusens <https://github.com/sergiusens>`__ (`#3507 <https://github.com/snapcore/snapcraft/pull/3507>`__)
-  extensions/desktop: use fonts from $XDG_DATA_DIRS, and remove unnecessary includes `@jhenstridge <https://github.com/jhenstridge>`__ (`#3504 <https://github.com/snapcore/snapcraft/pull/3504>`__)
-  cli, repo: add support for UA tokens `@cjp256 <https://github.com/cjp256>`__ (`#3488 <https://github.com/snapcore/snapcraft/pull/3488>`__)
-  snaps: don’t validate snaps before ``SnapPackage.download()`` `@Saviq <https://github.com/Saviq>`__ (`#3505 <https://github.com/snapcore/snapcraft/pull/3505>`__)
-  deb: do not filter python3 packages on core20 `@cjp256 <https://github.com/cjp256>`__ (`#3503 <https://github.com/snapcore/snapcraft/pull/3503>`__)
-  Update Docker image instructions `@abitrolly <https://github.com/abitrolly>`__ (`#3499 <https://github.com/snapcore/snapcraft/pull/3499>`__)
-  conda v2 plugin: support for more architectures `@sergiusens <https://github.com/sergiusens>`__ (`#3495 <https://github.com/snapcore/snapcraft/pull/3495>`__)
-  snaps: do not validate snaps before install/refresh (Fixes LP#1901733) `@Saviq <https://github.com/Saviq>`__ (`#3502 <https://github.com/snapcore/snapcraft/pull/3502>`__)
-  docker: Need to repeat ARG in every section `@abitrolly <https://github.com/abitrolly>`__ (`#3500 <https://github.com/snapcore/snapcraft/pull/3500>`__)
-  store: use whoami dashboard endpoint for cli `@sergiusens <https://github.com/sergiusens>`__ (`#3501 <https://github.com/snapcore/snapcraft/pull/3501>`__)
