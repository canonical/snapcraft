.. _release-8.11:

Snapcraft 8.11 release notes
============================

4 August 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.11.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.

What's new
----------

Snapcraft 8.11 brings the following features, integrations, and improvements.

Unified behavior for the pack command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When a new base is introduced in Snapcraft, it expects new vocabulary and expressions in
the project file. Projects on old bases aren't affected, and can continue building using
the older vocabulary. It's only when a crafter switches their existing project to a new
base that the breaking changes are exposed, which leads to an in-depth migration.

This strategy has downsides. In particular, the user experience can be inconsistent when
building snaps with different bases. Running ``snapcraft pack`` would behave differently
depending on the base.

With this release, the pack command for core20 snaps is ported to have the same
functionality as the pack command for core22 and core24 snaps. With no arguments,
running ``snapcraft`` builds and packages the snap. If a directory is passed as the
first positional argument, Snapcraft skips the build process and instead packs the
directory as a snap.


Backwards-incompatible changes
------------------------------

The following changes are incompatible with previous versions of Snapcraft.


Snap file name changes
~~~~~~~~~~~~~~~~~~~~~~

The file names for core24 snaps are now derived from the name of the entry in the
``platforms`` key instead of the value of its ``build-for`` key.

For example, consider the following snippet:

.. code-block:: yaml
  :caption: snapcraft.yaml

  name: hello-world
  version: 1.0
  platforms:
    rpi:
      build-on: [arm64]
      build-for: [arm64]
    rpi-debug:
      build-on: [arm64]
      build-for: [arm64]

Previously, Snapcraft would build both snaps and use the same name,
``hello-world_1.0_arm64.snap``, for both snaps. This would result in the first snap
being overwritten by the second snap.

Now, Snapcraft builds two separate snaps, ``hello-world_1.0_rpi.snap`` and
``hello-world_1.0_rpi-debug.snap``.


Scheduled feature deprecations
------------------------------

The following features will be deprecated in a future major release:

Running Snapcraft without a command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If Snapcraft is run without a command (``snapcraft``), the default behavior is to
pack a snap.

This release adds a deprecation warning when Snapcraft is run without a command.
``snapcraft pack`` should be used instead.
