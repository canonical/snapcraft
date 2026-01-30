Snapcraft 9.0 release notes
===========================

.. add date before releasing
(upcoming release)

Learn about the new features, changes, and fixes introduced in Snapcraft 9.0.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


Minor features
--------------

Snapcraft 9.0 brings the following minor changes.

Promote edge channels with ``--yes``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`ref_commands_promote` command now supports promoting releases from the edge
channel non-interactively with the ``--yes`` flag.


Backwards-incompatible changes
------------------------------

The following changes are incompatible with previous versions of Snapcraft.

Removed snapcraftctl for core26
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft 8 deprecated the ``snapcraftctl`` command in override scripts for core22 and
core24 snaps in favor of :ref:`craftctl <reference-external-package-scriptlets>`.

Core26 snaps only support ``craftctl``. To use core26, you must replace all
instances of ``snapcraftctl`` in your scripts.

Core22 and core24 aren't affected by this change.


Contributors
------------

We would like to express a big thank you to all the people who contributed to
this release.

.. update contributors before releasing
:literalref:`@alex<https://example.com/alex>`,
:literalref:`@blair<https://example.com/blair>`,
:literalref:`@cam<https://example.com/cam>`,
and :literalref:`@devin<https://example.com/devin>`
