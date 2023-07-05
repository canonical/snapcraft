.. 7207.md

.. _snap-layouts:

Snap layouts
============

Layouts modify the execution environment of a :ref:`strictly-confined <snap-confinement>` snap.

With layouts, you can make elements in ``$SNAP``, ``$SNAP_DATA``, ``$SNAP_COMMON`` accessible from locations such as ``/usr``, ``/var`` and ``/etc``. This helps when using pre-compiled binaries and libraries that expect to find files and directories outside of locations referenced by ``$SNAP`` or ``$SNAP_DATA``.

   ℹ Layouts can only help *within* a snap’s environment. They cannot be used to expose elements within a snap to the host environment.

Layouts require snap version *2.36+*.

Using layouts
-------------

Layouts are transparently supported by :ref:`Snapcraft <snapcraft-overview>` when using a :ref:`base snap <base-snaps>` (e.g. ``base: core18``).

   ⓘ When not using a base snap, layout options within ``snapcraft.yaml`` must be made within the scope of a top-level :ref:`passthrough <using-in-development-features-in-snapcraft-yaml>` field.

As a simple example, consider an application you want to snap that:

-  stores all data in ``/var/lib/foo``
-  has a configuration file in ``/etc/foo.conf``
-  uses read-only data in ``/usr/share/foo``

A layout that allows such software to be used without snap-specific modifications can be defined as follows:

.. code:: yaml

   layout:
     /var/lib/foo:
       bind: $SNAP_DATA/var/lib/foo
     /usr/share/foo:
       bind: $SNAP/usr/share/foo
     /etc/foo.conf:
       bind-file: $SNAP_DATA/etc/foo.conf

None of the above filesystem modifications are visible to any other snaps, or from the wider user session. They’re only visible within the per-snap mount namespace.

Layout reference
----------------

The syntax for defining a layout is:

.. code:: yaml

   layout:
     <target-path>: <declaration>
     <target-path>: <declaration>
     ..

Layouts are defined as a key-value map, mapping from a ``<target-path>`` to a layout declaration. Each declaration may be one of the following:

-  ``symlink: <source-path>``: create a symbolic link. This method is preferred because it is the cheapest; the other methods significantly increase the startup time of your application.
-  ``bind: <source-path>``: bind-mount a directory.
-  ``bind-file: <source-path>``: bind-mount a file
-  ``type: tmpfs``: mount a private temporary in-memory filesystem

Layouts using ``bind*`` and ``tmpfs`` **significantly increase the startup time** of your snap. We recommend using ``symlink`` instead, because it has the least amount of overhead.

.. code:: yaml

     /usr/lib/x86_64-linux-gnu/libEGL.so:
       symlink: $SNAP/usr/lib/x86_64-linux-gnu/libEGL.so
     /usr/lib/x86_64-linux-gnu/libGL.so:
       symlink: $SNAP/usr/lib/x86_64-linux-gnu/libGL.so

Some applications, however, might treat symlinks differently than regular files or directories so you may need to use a bind mount in those cases.

``<source-path>`` must refer to either ``$SNAP``, ``$SNAP_DATA`` or ``$SNAP_COMMON``.

``<target-path>`` can include nearly any path except for:

- ``/boot``
- ``/dev``
- ``/home``
- ``/lib/firmware``, ``/usr/lib/firmware``
- ``/lib/modules``, ``/usr/lib/modules``
- ``/lost+found``
- ``/media``
- ``/proc``
- ``/run``, ``/var/run``
- ``/sys``
- ``/tmp``
- ``/var/lib/snapd``
- ``/var/snap``

As ``/lib`` and ``/run`` are symbolic links to ``/usr/lib`` and ``/var/run`` respectively, they require separate exceptions to ensure certain locations, such as ``/lib/firmware``, can’t be worked around. See below for further limitations.

If ``<source-path>`` and ``<target-path>`` don’t already exist, they will be automatically created by snapd. This includes the creation of new empty files, but doesn’t include the creation of symbolic link targets. This is because snapd doesn’t know what kind of objects they may eventually point to. In the previous example, ``$SNAP_DATA/etc/foo.conf`` is created before any snap application code is executed.

Creating new files and directories in read-only spaces
------------------------------------------------------

Layouts can create new directories and files even in read-only locations such as ``/usr/share``. The following declaration will create ``/usr/share/foo``, for example, visible only to executing snap applications (it’s assumed that ``/usr/share/foo`` does **not** exist in the base snap declared by the application developer):

.. code:: yaml

   layout:
     /usr/share/foo:
       bind: $SNAP/usr/share/foo

To accomplish the above, snapd uses a temporary filesystem (tmpfs) mounted on ``/usr/share`` and populated with a set of empty files and directories. These are then used for bind mounts as well as symlinks to reconstruct the original ``/usr/share``. This allows snapd to make ``/usr/share`` writable, and consequently, allows snapd to create ``/usr/share/foo`` and configure it as desired.

Current limitations
-------------------

The following apply as of snapd 2.36:

Layouts do not work with *classic* snaps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This functionality only works with :ref:`strictly-confined <snap-confinement>` snaps, and does not work with snaps using *classic* confinement. This may change in the future.

New entries in / (root)
~~~~~~~~~~~~~~~~~~~~~~~

Layouts cannot currently create new top-level files or directories. For example, the following layout declaration will not work:

.. code:: yaml

   layout:
     /foo: # Unsupported, cannot create new top-level directories.
        bind: $SNAP/foo

Incompatible existing file, directory or symbolic link
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Layouts cannot replace an existing but incompatible filesystem object. This means, for example, that files cannot replace directories or symbolic links, files cannot replace a directory, and existing symbolic links cannot be redirected to a new target. You can, however, replace a directory with another directory.
