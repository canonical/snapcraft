.. 24905.md

.. _data-locations:

Data locations
==============

Most snaps use strict confinement to isolate both their execution environments and their data from your system (see :ref:`Snap Confinement <snap-confinement>` for further details).

This confinement is an important security feature, but it can affect how and where you access a snap’s files, including:

-  configuration files
-  user documents
-  internal databases

A snap which needs user-access to files should use an :ref:`interface <interface-management>`, such as the :ref:`home interface <the-home-interface>`, to bridge this confinement gap.

But it can also be helpful to understand how snaps operate on and access data, even without an interface, and this is outlined below:

Snap installation
-----------------

When a user `installs a snap <https://snapcraft.io/docs/getting-started#heading--install-snap>`__ from the `Snap Store <https://snapcraft.io/store>`__, the following happens:

1. The :term:`snapd` service downloads the snap as a single file – a compressed SquashFS archive with a *.snap* suffix.
2. The snap file is uncompressed and mounted as a read-only filesystem under */snap.* See :ref:`The snap format <the-snap-format>` for further details on what is included in a snap.

By design, the read-only filesystem cannot provide a persistent experience between application launches, which is why snaps also have writable parts for system data and for user data.

System data
-----------

Within the snap environment, :ref:`environment variables <environment-variables>` are used to reference different accessible locations. The following variables and default referenced locations are used to store system data:

-  **SNAP_COMMON**: ``/var/snap/<snap name>/common`` This directory is owned and writable by root and is used to store data that is common across multiple :term:`revisions` of the snap (e.g.: revision 6, revision 7, etc.).

-  **SNAP\_ DATA**: ``/var/snap/<snap name>/<revision number>`` This location it is also used to store data, mostly information utilised by background application and services, for logging, and other tasks that require persistence between snap launches.

A `snapshot <https://snapcraft.io/docs/snapshots>`__ of ``SNAP_DATA`` and ``SNAP_COMMON`` is created and restored when performing a snap update (refresh) or revert operation. The contents of ``SNAP_DATA`` is specific to the snap revision, while the contents of ``SNAP_COMMON`` is applicable to all revisions and will overwrite the contents of ``SNAP_COMMON`` when restored. See `What a snapshot stores <https://snapcraft.io/docs/snapshots#data-locations-heading--what-is-stored>`__ for more details.

User data
---------

Snaps can also contain user data. As with the system data environment variables, ``SNAP_COMMON`` and ``SNAP_DATA``, the following user-specific environment variables point to directories for user data:

-  **SNAP_USER_COMMON**: ``/home/<username>/snap/<snap name>/common`` This location maps user data across each revision of a snap. It is not backed up or restored on snap operations.

-  **SNAP_USER_DATA**: ``/home/<username>/snap/<snap name>/<revision>`` This contains any user data that the snap writes to its own home. This is *in contrast* to what the Linux user would consider *their* home. It is important to note this distinction, because it can be useful, and even important when users decide to perform maintenance operations with their snaps (like removal). By default, every snap will use a symlink *current*, pointing to the latest available revision.

   Snaps without the home interface declared and/or connected cannot access the disk. The ``$SNAP_USER_DATA`` directory will still be created and exist in the user’s home directory but it can contain no files

Both ``SNAP_USER_COMMON`` and ``SNAP_USER_DATA`` only become available after a snap has been run once.

Ubuntu Core
-----------

On Ubuntu Core, the **SNAP_SAVE_DATA** environment variable within a snap’s environment points to a snap-specific location on the `ubuntu-save <https://ubuntu.com/core/docs/uc20/inside#data-locations-heading--layouts>`__ volume. This is used to store data that can be accessed during recovery or after re-installation of Ubuntu Core.

For example, in the reference Ubuntu Core 22 image, from within the *hello-world* snap, ``SNAP_SAVE_DATA`` has the following value:

::

   SNAP_SAVE_DATA=/var/lib/snapd/save/snap/hello-world

The above environment variable references a mount point at the following location:

.. code:: bash

   $ mount | grep "snapd/save"
   /dev/mapper/ubuntu-save on /var/lib/snapd/save type ext4 (rw,relatime)


.. _data-locations-other:

Other locations
---------------

There are several other directories you should be aware of:

-  **/var/lib/snapd/cache** This is the working cache and is used to minimise download size and speed-up refreshes.
-  **/var/lib/snapd/snaps** Contains all the versions of snaps installed on your system.
-  **/var/lib/snapd/snapshots/** Contains both the manually generated and automatically generated `snapshots <https://snapcraft.io/docs/snapshots>`__.


.. _data-locations-delete:

Deleting a snap
---------------

When deleting and removing a snap from a system, the following will happen:

-  The snap will be unmounted and no longer shown under ``/snap``.
-  The data under ``/var/snap/<snap name>/`` and ``/home/<username>/snap/`` will be deleted. However, a copy is be retained as a `snapshot <https://snapcraft.io/docs/snapshots>`__ for 30-days (except on Ubuntu Core systems), allowing data to be restored or manually retrieved.

`Snapshot management <https://snapcraft.io/docs/snapshots>`__ can be used to restore data, or unzip the archives, and only copy the data you consider necessary. With the right permissions, you can also create your own backup routine, which copies the important data like application databases, configurations or similar content to a backup path.

To remove a snap without generating a snapshot, use the additional ‘–purge’ argument:

.. code:: bash

   $ sudo snap remove vlc --purge
   vlc removed
