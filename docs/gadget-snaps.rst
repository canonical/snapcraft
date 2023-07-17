.. 696.md

.. _gadget-snaps:

Gadget snaps
============

The gadget snap is responsible for defining and configuring system properties specific to one or more devices.

The gadget metadata and content defines:

- The layout of the volumes that comprise the device storage and image
- Configuration for the bootloader to use. The gadget also ships the bootloader itself and other boot assets.
- Default configuration options to use when snaps are installed.
- Interface connections configured in the ``connections:`` section are executed on the device’s first boot only. Later changes to this section – that is, changes added to the device at run time through gadget refreshes – are not applied.
- Optional hooks that are invoked to control and customise the behaviour over the device lifecycle, e.g. installation, initialisation and establishing device identity, factory reset.

See `Building a gadget snap <https://ubuntu.com/core/docs/gadget-building>`__ for details on how a gadget snap can be built. For store deployment, gadget snaps must be produced by the device :term:`brand`, as defined in the `model assertion <https://ubuntu.com/core/docs/reference/assertions/model>`__, or a reference gadget must be used. It is perfectly possible for different models to share a gadget snap.

--------------

A typical gadget snap will consist of the following:

- `Setup files <gadget-snaps-setup_>`__
- `Gadget.yaml <gadget-snaps-gadget_>`__
- `Volumes <gadget-snaps-volumes_>`__
- `Specification <gadget-snaps-specification_>`__
- `Raspberry Pi example <gadget-snaps-piexample_>`__
- `Prepare-device hook <gadget-snaps-prepare_>`__ - `Example script <gadget-snaps-example-prepare_>`__


.. _gadget-snaps-setup:

Setup files
-----------

In addition to traditional snap metadata, the gadget snap also holds some setup files fundamental to the initialisation and lifecycle of the device:

-  **meta/snap.yaml**: traditional snap details, with ``type: gadget`` explicitly defined.
-  **meta/gadget.yaml**: gadget-specific information. See below.
-  **grub.conf**: required grub configuration when using this bootloader.
-  **u-boot.conf**: required U-Boot configuration when using this bootloader.
-  **cloud.conf**: optional `cloud-init <https://cloudinit.readthedocs.io/en/latest/>`__ configuration; cloud-init is disabled if missing.

Sample configuration files may be found in the current reference gadget snaps:

- `Raspberry Pi (2B, 3B, 3A+, 3B+, 4B, Compute Module 3, and Compute Module 3+) <https://github.com/snapcore/pi-gadget>`__
- `amd64 <https://github.com/snapcore/pc-amd64-gadget>`__ - `i386 <https://github.com/snapcore/pc-i386-gadget>`__

In the near future, we expect to add a RISC-V reference gadget snap to this list.


.. _gadget-snaps-gadget:

The gadget.yaml file
--------------------

Two YAML keys are used to describe your target device:

-  **defaults** (YAML sub-section, optional): default configuration options for the defined snaps, applied on installation: ``yaml  defaults:        <snap id>:            <key>: <value>`` In addition to ``<snap id>``, it is also possible to use ``system`` to define system-wide configuration options, similar to running ``snap set system key=value``: ``yaml  defaults:        system:            <key>: <value>``

.. note::
          Defaults only become available during snap installation.

          Values in ``defaults:`` (other than ``system:``) are not consumed and do not become available until either the :ref:`configure hook <supported-snap-hooks-the-configure-hook>` or the :ref:`default-configure hook <supported-snap-hooks-default-configure>` are run as part of the corresponding snap installation. ``system:`` values are set immediately.



-  **volumes** (YAML sub-section, required): the volumes layout, where each disk image is represented as a YAML sub-section.


.. _gadget-snaps-volumes:

The volumes mapping sub-section
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each volume entry is described by:

- a name as defined by the entry key
- a partition structure (required)
- a bootloader definition (``grub``, ``u-boot``)
- a partitioning schema eg. ``mbr``. Defaults to ``gpt`` if unspecified.

Volumes define the structure and content of the images to be written into one or more block devices of the gadget device. Each volume in the mapping represents a different image for a “disk” in the device.

Ubuntu Core 20 typically uses the following storage partitions:

-  **ubuntu-seed** (role: system-seed; *read-only, ext4 or typically vfat*)
-  **ubuntu-boot** (role: system-boot; *read-only, ext4* or *vfat*):
-  **ubuntu-save** (role: system-save; *writable, ext4*, **encrypted**)
-  **ubuntu-data** (role: system-data; *writable*, ext4, **encrypted**)

**ubuntu-save** is mandatory on an encrypted system. The *initramfs* bootstrapped from **ubuntu-boot** is responsible for decrypting both the **ubuntu-save** and **ubuntu-data** partitions.

**ubuntu-data** needs to be the last partition. No extra partitions can be inserted between **ubuntu-boot**, **ubuntu-save** and **ubuntu-data**. If extra partitions are required, they need to be declared and created before **ubuntu-boot**.

The structure section lists entities with gadget data inside the image, most of which are partitions with a file system inside, with the exception of structures of type: bare, which can describe a region of data without a corresponding entry in the partition table.


.. _gadget-snaps-specification:

Specification
~~~~~~~~~~~~~

The ``meta/gadget.yaml`` file contains the basic metadata for gadget-specific functionality, including a detailed specification of which structure items compose an image. The latter is used both by snapd and by ubuntu-image when creating images for these devices.

A gadget snap’s boot assets can also be automatically updated when the snap is refreshed. See :ref:`Updating gadget boot assets <updating-gadget-boot-assets>` for further details.

The following specification defines what is supported in ``gadget.yaml``:

.. code:: yaml

   # Define the format of this file. The default and latest format is zero.
   # Clients reading this file must reject it the format is greater than
   # the supported one. (optional)
   format: <int>

   # Default configuration options for defined snaps, applied on installation.
   # The snap ID may be discovered via the snap info command.
   # Since 2.33 snap ID can be the "system" nick to cover the system
   # configuration. (optional)
   defaults:
       <snap id>:
           <key>: <value>

   # Interface connection instructions for plugs and slots of seeded
   # snaps to connect at first boot. snap IDs can be the "system"
   # nick as well. Omitting "slot" in an instruction is allowed
   # and equivalent then to: slot: system:<plug>
   # (since 2.34) (optional)
   connections:
      -  plug: <plug snap id>:<plug>
         slot: <slot snap id>:<slot>

   # Volumes defining the structure and content for the images to be written
   # into one or more block devices of the gadget device. Each volume in
   # in the structure represents a different image for a "disk" in the device.
   # (optional)
   volumes:

     # Name of volume and output image file. Must match [a-z-]+. (required)
     <volume name>:

       # 2-digit hex code for MBR disk ID or GUID for GPT disk id. (optional)
       id: <id>

       # Bootloader in the volume. Required in one volume. (required/optional)
       bootloader: grub | u-boot

       # Which partitioning schema to use. Defaults to gpt. (optional)
       schema: mbr | gpt | mbr,gpt

       # Structure defines layout of the volume, including partitions,
       # Master Boot Records, or any other relevant content. (required)
       structure:
         - # Structure value is a list.

           # Structure item name. There's an implementation-specific constraint
           # on the maximum length. The maximum length of a partition name
           # for GPT is 36 characters in the UTF-16 character set. (optional)
           name: <name>

           # GPT unique partition id, disallowed on MBR volumes. (optional)
           id: <id>

           # Role defines a special role for this item in the image. (optional)
           # Must be either unset, or one of:
           #   mbr - Master Boot Record of the image.
           #   system-seed - Partition holding first-stage/recovery boot loader and at
           #                 least one recovery system containing the following
           #                 set of snaps: base, kernel, gadget and application snaps
           #   system-boot - Partition holding the boot assets.
           #   system-data - Partition holding the main operating system data.
           #   system-boot-image - Partition holding kernel images for the Little Kernel bootloader.
           #   system-boot-select - Partition holding state for snapd Little Kernel support.
           #   system-save - Partition for Ubuntu Core 20 to store backup data relative to device
           #                 identity and to facilitate recovery or re-install.
           #
           # A structure with role:system-data must either have an implicit
           # file system label, or 'writable'.
           # A structure with role:system-boot-select must have 'snapbootsel' label.
           role: mbr | system-boot | system-data | system-boot-image | system-boot-select | system-save

           # Type of structure. May be specified as a two-hex-digit MBR partition
           # type, a GPT partition type GUID, or both on hybrid schemas.  The
           # special value `bare` says to not create a disk partition for this
           # structure. (required)
           type: <mbr type> | <gpt guid> | <mbr type>,<gpt guid> | bare

           # Size for structure item. Maximum of 446 for the mbr role. (required)
           size: <bytes> | <bytes/2^20>M | <bytes/2^30>G

           # The offset from the beginning of the image. Defaults to right after
           # prior structure item. (optional)
           offset: <bytes> | <bytes/2^20>M | <bytes/2^30>G

           # Offset of this structure element (in units of 512-byte sectors) is
           # written to the provided position within the volume in LBA48 pointer
           # format (32-bit little-endian). This position may be specified as a
           # byte-offset relative to the start of another named structure item.
           # (optional)
           offset-write: [<name>+]<bytes> |
                         [<name>+]<bytes/2^20>M |
                         [<name>+]<bytes/2^30>G

           # Filesystem type. Defaults to none. (optional)
           filesystem: none | vfat | ext4

           # Filesystem label. Defaults to name of structure item. (optional)
           filesystem-label: <label>

           # Content to be copied from gadget snap into the structure. This
           # field takes a list of one of the following formats. (required)
           content:

               # Copy source (relative to gadget base directory) into filesystem
               # at target (relative to root). Directories must end in a slash.
               - source: <filename> | <dir>/  # (required)
                 target: <filename> | <dir>/  # (required)

               # Dump image (relative to gadget base directory) of the raw data
               # as-is into the structure at offset. If offset is omitted it
               # defaults to right after the prior content item. If size is omitted,
               # defaults to size of contained data.
               - image: <filename>                                 # (required)
                 offset: <bytes> | <bytes/2^20>M | <bytes/2^30>G   # (optional)
                 offset-write: (see respective item above)         # (optional)
                 size: <bytes> | <bytes/2^20>M | <bytes/2^30>G     # (optional)

           # Support automatic asset updates. (optional)
           update:
               # update only if the new edition is higher than the old edition.
               edition: uint32
               # This field takes a list of files to be preserved.
               # No support for preserving inside images.
               # i.e. update will overwrite the whole image in this case.
               preserve:
                 - <filename>




.. _gadget-snaps-piexample:

Example: Raspberry Pi 3 gadget.yaml
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: yaml

   device-tree: bcm2709-rpi-3-b-plus
   volumes:
     pi:
       schema: mbr
       bootloader: u-boot
       structure:
         - name: ubuntu-seed
           role: system-seed
           filesystem: vfat
           type: 0C
           size: 1200M
           content:
             - source: boot-assets/
               target: /
         - name: ubuntu-boot
           role: system-boot
           filesystem: vfat
           type: 0C
           # what's the appropriate size?
           size: 750M
           content:
             # TODO:UC20: install the boot.sel via snapd instead of via the gadget
             - source: boot.sel
               target: uboot/ubuntu/boot.sel
         # NOTE: ubuntu-save is optional for unencrypted devices like the pi, so
         # this structure can be dropped in favor of a different partition for
         # users who wish to instead use a different partition, since with MBR we
         # are limited to only 4 primary partitions.
         # TODO: look into switching over to GPT, the pi bootloader firmware now
         #       has support for this
         - name: ubuntu-save
           role: system-save
           filesystem: ext4
           type: 83,0FC63DAF-8483-4772-8E79-3D69D8477DE4
           size: 16M
         - name: ubuntu-data
           role: system-data
           filesystem: ext4
           type: 83,0FC63DAF-8483-4772-8E79-3D69D8477DE4
           # XXX: make auto-grow to partition
           size: 1500M




.. _gadget-snaps-prepare:

prepare-device hook
-------------------

The optional ``prepare-device`` hook will be called on the gadget at the start of the device initialisation process, after the gadget snap has been installed.

The hook will also be called if this process is retried later from scratch in case of initialisation failures.

The device initialisation process is, for example, responsible for setting the serial identification of the device through an exchange with a device service.

The ``prepare-device`` hook can for example redirect this exchange and dynamically set options relevant to it.

One must ensure that ``registration.proposed-serial`` is set to a *unique value* across all devices of the brand and model and that it does not contain a ``/``. It is going to be used as the “serial number” (a string, not necessarily a number) part of the identification in case the device service supports setting it or **requires** it, as is the case with the *serial-vault*.


.. _gadget-snaps-example-prepare:

Example: prepare-device hook
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   #!/bin/sh

   # optionally set the url of the service
   snapctl set device-service.url="https://device-service"

   # set optional extra HTTP headers for requests to the service
   snapctl set device-service.headers='{"api-key": "API-KEY-VALUE"}'

   # set an optional proposed serial identifier, depending on the service
   # this can end up being ignored
   #
   # this might need to be obtained dynamically. as the expected value must be a JSON string
   # one must be careful with proper shell quotation especially if using command
   # substitution, e.g.:
   #   snapctl set registration.proposed-serial='"'"$(get-serial-number)"'"'
   snapctl set registration.proposed-serial="DEVICE-SERIAL"

   # optionally pass details of the device as the body of registration request,
   # the body is text, typically YAML;

   # this might need to be obtained dynamically
   snapctl set registration.body='mac: "00:00:00:00:ff:00"'
