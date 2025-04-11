.. _how-to-use-layouts:

Use layouts
===========

A :ref:`layout <reference-layouts>` exposes elements inside the snap filesystem to the
host system at runtime. They can make files, folders, and links in ``$SNAP``,
``$SNAP_DATA``, ``$SNAP_COMMON`` accessible to host locations such as ``/usr``,
``/var``, and ``/etc``. These connections between the snap and host aren't routed for
other snaps.

Layouts can help `hooks <https://snapcraft.io/docs/supported-snap-hooks>`_ access any
executables they may require.


Define a layout
---------------

Define a layout as a map from target paths to source paths. Each source path takes the
shape of a layout definition.

.. code-block:: yaml
    :caption: snapcraft.yaml

    layout:
      <target-path>:
        <layout>
      <target-path>:
        <layout>

Set the target path to the file or directory to exposed to the snap. Layouts can't
target every path on the host filesystem, so consult the :ref:`target path restrictions
<reference-layouts-requirements>` to check if the destination isn't supported.

Select the appropriate layout type for the target and replace ``<layout>`` with the
matching syntax:

- ``symlink: <source-path>`` creates a symbolic link
- ``bind: <source-path>`` bind-mounts a directory
- ``bind-file: <source-path>`` bind-mounts a file
- ``tmpfs: <source-path>`` mounts a private, temporary, in-memory filesystem

Set the root of the source path to the appropriate location in the filesystem, being one
of ``$SNAP``, ``$SNAP_DATA``, or ``$SNAP_COMMON``.

The ``symlink`` layout type is recommended because it's the fastest. The ``bind*`` and
``tmpfs`` types *significantly* increase the startup time of the app. Some apps,
however, treat symlinks differently than regular files or directories, so you might
need a bind mount for them.

As ``/lib`` and ``/run`` directories in Linux are symbolic links to ``/usr/lib`` and
``/var/run`` respectively, they require separate exceptions to ensure certain locations,
such as ``/lib/firmware``, can't be worked around.


Examples
--------

As an example, consider an app you want to package that:

- Stores all data in ``/var/lib/foo``
- Has a configuration file in ``/etc/foo.conf``
- Uses read-only data in ``/usr/share/foo``

A layout map that allows such software to be used without snap-specific modifications
would look like:

.. code-block:: yaml
    :caption: snapcraft.yaml

    layout:
      /var/lib/foo:
        bind: $SNAP_DATA/var/lib/foo
      /usr/share/foo:
        bind: $SNAP/usr/share/foo
      /etc/foo.conf:
        bind-file: $SNAP_DATA/etc/foo.conf

Here's a realistic example of a layout map that makes the OpenGL library available to
the snap at runtime:

.. code-block:: yaml
    :caption: snapcraft.yaml

    layouts:
      /usr/lib/x86_64-linux-gnu/libEGL.so:
        symlink: $SNAP/usr/lib/x86_64-linux-gnu/libEGL.so
      /usr/lib/x86_64-linux-gnu/libGL.so:
        symlink: $SNAP/usr/lib/x86_64-linux-gnu/libGL.so


Manage files in read-only locations
-----------------------------------

Layouts can create new directories and files even in read-only locations such as
``/usr/share``.

For example, assume ``/usr/share/foo`` *doesn't* exist in the base snap. The following
layout map creates ``/usr/share/foo``, visible only to executing snap apps:

.. code-block:: yaml
    :caption: snapcraft.yaml

    layout:
      /usr/share/foo:
        bind: $SNAP/usr/share/foo
