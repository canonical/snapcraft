.. _release-8.11:

Snapcraft 8.11 release notes
============================

4 August 2025

Learn about the new features, changes, and fixes introduced in Snapcraft 8.11.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


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
