.. 14117.md

.. _updating-gadget-boot-assets:

Updating gadget boot assets
===========================

A :ref:`gadget snap’s <gadget-snaps>` boot assets can be automatically updated when the snap is refreshed. It’s the responsibility of the gadget snap publisher to ensure the correctness and consistency of the update data, as outlined below.

-  the update process relies heavily on the information provided in :ref:`gadget.yaml <gadget-snaps-gadget>` as it attempts to verify the internal consistency of the gadget description
-  gadget and kernel snap versions are not coupled. This means updated boot assets must be able to boot the old and new kernels of a given gadget
-  a device will reboot after boot assets have been updated

..

   ℹ *snapd 2.42+* is required for automatic boot assets updates and *snapd 2.43+* employs early consistency checks when building gadget snaps.

Enabling boot asset updates
---------------------------

Automatic boot asset updates are enabled by adding an ``update`` section below the ``structure:`` definition in a gadget snap’s gadget.yaml:

.. code:: yaml

   volumes:
     my-volume:
       structure:
         - name: some-structure
           ..
           update:
             edition: 2

The ``edition`` property identifies the version of assets for a particular ``structure``.

Updates are performed when the ``edition`` value is higher than the value in the currently installed gadget snap. If this is the case, *snapd* next attempts to analyse the currently written assets so that only those that differ are updated.

Backup and rollback
~~~~~~~~~~~~~~~~~~~

When an update is performed, snapd first creates a backup of all modified boot assets for both filesystem and non-filesystem structures. Backups are kept on the ``writable`` partition inside the snapd *state* directory and are cleaned-up after an update has been applied.

A rollback is only performed when the process of writing updated boot assets fails. Once the assets have been written, the changes will not be undone, even if a later step of snap installation fails.

Volume and structures
---------------------

Snapd analyses the ``volumes`` declaration in ``gadget.yaml`` to map its members against partitions on the main block device used by the system.

   ℹ The main block device is the one with ``/writable`` partition on it.

A volume layout is built with the following constraints:

-  **512 byte** sector size
-  **1MB** default starting offset for the first non-MBR partition, unless otherwise specified

The volume layout describes the exact position of each structure (partition) and their content. At runtime, structures are identified by either their defined ``name`` or a ``filesystem-label``:

-  ``name`` structures are identified using ``/dev/disk/by-partlabel`` symlinks
-  ``filesystem-label`` structures are identified using ``/dev/disk/by-label`` symlinks

Both ``name`` and ``filesystem-label`` must be unique amongst all named structures and labelled filesystems.

Volume structures obviously need to be named, and the filesystem needs to be labelled, to allow snapd to easily identify corresponding partitions and mount locations.

Filesystem structures
---------------------

Structures containing filesystems can be updated if the filesystem is mounted at runtime and is accessible to snapd. The update process then writes the files and directories listed in the ``content`` section of the structure.

Specific entries can be retained during the update by listing each individual item in the ``preserve`` list of the ``update`` structure, as shown below:

.. code:: yaml

   volumes:
     my-volume:
       structure:
         - name: some-fs-structure
           type: <uuid>
           filesystem: ext4
           size: 10M
           content:
             - source: a.data
               target: /
             - source: some-assets/
               target: dir/
           update:
             edition: 2
             preserve:
               - a.data
               - b.env
               - dir/keep

In the above example, the boot assets update process will:

- deploy the ``a.data`` file into the root (``/``)
- copy the contents of ``some-assets/`` to \`dir/

Should any of the entries listed in the ``preserve`` section exist beforehand, they will be preserved intact.

Unnamed and non-filesystem structures
-------------------------------------

Support for unnamed *non-filesystem* structures, or structures without a partition table entry, ``type: bare`` or ``filesystem: none``, for example, are enabled via a fallback mechanism:

snapd identifies the partition carrying the ``writable`` filesystem and proceeds to apply the updates to the parent device. For example, assuming ``/writable`` is mounted from ``/dev/mmcblk0p2``, the fallback mechanism would identify ``/dev/mmcblk0`` as the parent device.

The contents of these structures can also be updated in the boot assets update process. Each image listed in the ``content`` section is written to the structure, as shown below:

.. code:: yaml

   volumes:
     my-volume:
       structure:
         - name: some-structure
           type: <uuid>
           filesystem: none
           size: 1M
           content:
             - image: raw.img
             - image: other.img
               offset: 10240
           update:
             edition: 2

With the above example, the boot assets update process will write the contents of ``raw.img`` at the 0 offset inside the partition corresponding to the structure, while ``other.img`` is written at 10kB offset from the start of the partition.

Use of ``preserve`` to retain specific files inside non-filesystem structures is unsupported.

Caveats
-------

Currently, boot asset updates have the following limitations:

- the gadget snap can have only one defined volume
- the updated gadget snap must use the same structure-level layout
- once all of changed boot assets have been updated, the original files and images will not be restored from the backup, even if a later step of the installation fails
- a device will still reboot when an update is applied, even when no boot assets are effectively changed
- there must be enough space on the ``writable`` partition to hold a backup copy of all modified boot assets

The following are also unsupported:

- encrypted or otherwise nested structures, such as LVM volumes
- using ``preserve`` inside non-filesystem structures
- updating unnamed and unlabelled filesystem structures
- updating devices not described inside ``gadget.yaml``, such as the *BOOT1/2* regions of SD cards
