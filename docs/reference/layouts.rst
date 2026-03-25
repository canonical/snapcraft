.. _reference-layouts:

Layouts
=======

A layout creates a bridge for precompiled binaries and libraries that expect to find
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
creates them. This includes the creation of new empty files, but not the creation of
symbolic link targets.

For mounting read-only files, snapd uses a temporary filesystem (tmpfs) mounted to
``/usr/share`` and populated with a set of empty files and directories. These are then
used for bind mounts as well as symlinks to reconstruct the original ``/usr/share``.
This enables snapd to make ``/usr/share`` writable, and consequently snapd can create app directories, such as ``/usr/share/foo``, and configure it as desired.


.. _reference-layouts-requirements:

Requirements
------------

Layouts and their definitions must satisfy the following requirements.

**Strictly-confined snaps**. Layouts only work with `strictly-confined
<https://snapcraft.io/docs/snap-confinement>`_ snaps, and not with classic confinement.

**Disallowed target paths**. The target path in a layout definition can't be:

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

**Non-root filesystem objects**. The file or directory in a layout definition can't be
at the root of the snap filesystem.

**Like-for-like replacement**. Layouts can only replace filesystem objects of the same
type. This means that, for example, you can route a file to a file, a directory with a
directory, and so on. You can't, on the other hand, replace directories with symbolic
links.
