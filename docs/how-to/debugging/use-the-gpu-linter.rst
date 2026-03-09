.. meta::
    :description: How to lint snaps that have GPU support and address problems raised by the linter.

.. _how-to-use-the-gpu-linter:

Use the GPU linter
==================

The following guidelines describe how to address issues flagged by the GPU linter.

In snaps needing GPU support, the GPU content snap should be used to provide the
necessary libraries. This is so that newer hardware can be supported without needing to
update the snap itself, and to de-duplicate a significant amount of libraries across
snaps.

If the GPU linter flags a library as being present in the snap, follow
:external+ubuntu-frame:ref:`use-snap-graphics`.

The GPU linter checks snaps that use the :ref:`reference-gnome-extension` and the
:ref:`reference-kde-neon-extensions`. If the GPU linter reports libraries that should
come from a content provider snap, there are three pathways you can try to remove them:

- Remove them from the ``stage-packages`` key. They will be supplied by the GPU
  provider snap.
- Exclude them in the ``prime`` key.
- Add a script that purges them, and call the script in the ``override-prime`` key of
  the final part in the project. Make sure the part is processed last with the ``after``
  key.

If the previous methods don't work, :ref:`suppress the warnings
<how-to-disable-a-linter-ignore-specific-files>` for the problem files.
