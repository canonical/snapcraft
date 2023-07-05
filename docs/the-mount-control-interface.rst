.. 28953.md

.. _the-mount-control-interface:

The mount-control interface
===========================

The ``mount-control`` interface allows the mounting and unmounting of both transient (non-persistent) and persistent filesystem mount points. The interface does not itself create or maintain mount points, but instead permits the snapped application or service to create and maintain its own.

This interface gives privileged access to the device.

Requires snapd version *2.54+*.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _`the-mount-control-interface-dev-details`:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

**Attributes**:

* ``persistent`` (plug): defines whether the mount can be persistent or not. Either ``true`` or ``false``. Defaults to ``false``.
* ``type`` (plug): lists one or more `acceptable filesystems <the-mount-control-interface-filesystems_>`__ for this mount. Example: ``[ext2, ext3, ext4]``
* ``what`` (plug): absolute path of what to mount using a matching wildcard. Example: ``/dev/sd*``
* ``where`` (plug): either an absolute destination path for the mount, or a starting reference to ``$SNAP_COMMON`` or ``$SNAP_DATA`` optionally followed by a path. Wildcards are also supported. Example: ``/media/$USER/**``
* ``options`` (plug, required): a list of one or more `permitted options <the-mount-control-interface-options_>`__. Example: ``[rw, sync]``
* ``namespace`` (plug): either ``snap``, for locations within the snap, ``host``, for locations outside the snap, or ``auto`` to set the namespace according to the *where* pattern. Defaults to ``auto``.

See `AppArmor globbing syntax <https://gitlab.com/apparmor/apparmor/-/wikis/AppArmor_Core_Policy_Reference#apparmor-globbing-syntax>`__ for details on how the pattern matching and wildcards work.

It’s the responsibility of the snapped application or service to create and maintain a mount point, using either of the following:

- the `mount <https://man7.org/linux/man-pages/man8/mount.8.html>`__ command
- the `mount() <https://man7.org/linux/man-pages/man2/mount.2.html>`__ system call
- an accompanying :ref:`snapctl mount <snapctl-tool-mount-control>`

The ``snapctl mount`` command is the only option for creating a persistent mount point via its ``--persistent`` mount option.


.. _the-mount-control-interface-filesystems:

Accepted filesystems
~~~~~~~~~~~~~~~~~~~~

Most filesystem types which are supported by the host system are allowed by this interface, with a few exceptions.

The following filesystems are **not** accepted:

``bpf``, ``cgroup``, ``cgroup2``, ``debugfs``, ``devpts``, ``encryptfs``, ``hugetlbfs``, ``overlayfs``, ``proc``, ``securityfs``, ``sysfs``, and ``tracefs``.

While other filesystems are accepted, the following filesystems are supported along with their respective filesystem-specific mount options (more on mount options below):

``adfs``, ``affs``, ``aufs``, ``autofs``, ``btrfs``, ``cifs``, ``ext2``, ``ext3``, ``ext4``, ``fat``, ``functionfs``, ``fuse``, ``hfs``, ``hpfs``, ``iso9660``, ``jfs``, ``msdos``, ``nfs``, ``nfs4``, ``ntfs``, ``ntfs-3g``, ``lowntfs-3g``, ``ramfs``, ``reiserfs``, ``squashfs``, ``tmpfs``, ``ubifs``, ``udf``,\ ``ufs``, ``umsdos``, ``usbfs``, ``vfat``, ``xfs``, and ``zfs``.

**Note:** If only ``tmpfs`` is specified, ``what`` must be ``none``.


.. _the-mount-control-interface-options:

Permitted mount options
~~~~~~~~~~~~~~~~~~~~~~~

The following filesystem-independent mount options are permitted:

``async``, ``atime``, ``auto``, ``bind``, ``defaults``, ``diratime``, ``dirsync``, ``iversion``, ``lazytime``, ``noatime``, ``noauto``, ``nodev``, ``nodiratime``, ``noexec``, ``nofail``, ``nogroup``, ``noiversion``, ``nolazytime``, ``nomand``, ``noowner``, ``norelatime``, ``nosuid``, ``nostrictatime``, ``nouser``, ``nousers``, ``relatime``, ``ro``, ``rw``, ``strictatime``, and ``sync``.

**Note:** Option ``bind`` is only allowed when no filesystem is specified.

Filesystem-specific mount options are also permitted, so long as each is supported by every filesystem listed in the corresponding ``type`` section. The mount options supported by a given filesystem can usually be found in its respective manpage or in the `mount <https://man7.org/linux/man-pages/man8/mount.8.html>`__ manpage. The complete list of supported filesystem-specific mount options for each filesystem type is included in the `source code <https://github.com/snapcore/snapd/blob/master/interfaces/builtin/mount_control.go>`__ for this interface.

Code examples
-------------

.. code:: yaml

   plugs:
    mntctl:
     interface: mount-control
     mount:
     - what: /dev/sd*
       where: /media/**
       type: [ext2, ext3, ext4]
       options: [rw, sync, user_xattr]
     - what: /usr/**
       where: $SNAP_COMMON/**
       options: [bind]
     - what: /dev/sda{0,1}
       where: $SNAP_COMMON/**
       options: [ro]
     - what: /dev/sdb[0-1]
       where: $SNAP_COMMON/{foo,other,**}
       options: [sync]
   apps:
    app:
     plugs: [mntctl]

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/mount_control_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/mount_control.go
