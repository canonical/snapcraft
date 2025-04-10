.. _reference-layouts:

Layouts
=======

A layout creates bridge for precompiled binaries and libraries that expect to find
files and directories outside of the snap.

Layouts are defined as a key-value map in a project file, mapping from a target path to
a layout definition.

.. code-block:: yaml
    :caption: snapcraft.yaml

    layout:
      <target-path>:
        <layout>
      <target-path>:
        <layout>

There are four types of layout definitions:

- ``symlink: <source-path>`` for symbolic links
- ``bind: <source-path>`` for bind-mounting a directory
- ``bind-file: <source-path>`` for bind-mounting a file
- ``tmpfs: <source-path>`` for mounting a private, temporary, in-memory filesystem

If ``<source-path>`` and ``<target-path>`` don't already exist, snapd automatically
creates them. This includes the creation of new empty files, but doesn't include the
creation of symbolic link targets.

For mounting read-only files, snapd uses a temporary filesystem (tmpfs) mounted to
``/usr/share`` and populated with a set of empty files and directories. These are then
used for bind mounts as well as symlinks to reconstruct the original ``/usr/share``.
This enables snapd to make ``/usr/share`` writable, and consequently snapd can create app directories, such as ``/usr/share/foo``, and configure it as desired.


Limitations
-----------

- **Target path restrictions**. The target path in a layout definition can't be:
  - ``/boot``
  - ``/dev``
  - ``/home``
  - ``/lib/firmware``
  - ``/usr/lib/firmware``
  - ``/lib/modules``
  - ``/usr/lib/modules``
  - ``/lost+found``
  - ``/media``
  - ``/proc``
  - ``/run``
  - ``/var/run``
  - ``/sys``
  - ``/tmp``
  - ``/var/lib/snapd``
  - ``/var/snap``
- **Strict confinement only**. Layouts only work with `strictly-confined
  <https://snapcraft.io/docs/snap-confinement>`_ snaps, and not with classic
  confinement.
- **Root filesystem objects**. Layouts can't create new files or directories at the root
  of the snap filesystem.
- **Like for like replacement**. Layouts can't replace an existing but incompatible
  filesystem object. This means that files can't replace directories or symbolic links,
  and you can't redirect existing symbolic links to a new target. You can, however,
  replace a directory with another directory.
