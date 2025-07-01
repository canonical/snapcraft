.. _release-8.11:

Snapcraft 8.11 release notes
============================

4 August 2024

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

The file name for core24 snaps now use the platform name instead of
the build-for architecture.

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

Now, Snapcraft builds 2 separate snaps, ``hello-world_1.0_rpi.snap`` and
``hello-world_1.0_rpi-debug.snap``.
