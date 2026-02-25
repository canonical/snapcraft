.. _how-to-use-the-gpu-linter:

Use the GPU linter
==================

The following guidelines describe how to address issues flagged by the GPU linter.

In snaps needing GPU support, the GPU content snap should be used to provide the necessary libraries.
This is so that newer hardware can be supported without needing to update the snap itself, and to de-duplicate a significant amount of libraries across snaps.

If the GPU linter flags a library as being present in the snap, follow :external+ubuntu-frame:ref:`the-gpu-2404-snap-interface`.

See :ref:`reference-gnome-extension`, :ref:`reference-kde-neon-extensions` as well, as these include GPU support in snaps using them.

See :ref:`how-to-disable-a-linter-ignore-specific-files` for how to ignore GPU linter warnings for specific files, if needed.
