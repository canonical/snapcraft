.. _reference-initrd-plugin:

Initrd plugin
==============

The Initrd plugin simplifies the building of initrds used by kernel snaps.
It is intended to generate an initrd for Ubuntu Core systems.

The initrd used by Ubuntu Core is constructed by the ubuntu-core-initramfs tool
and, while its usage is straightforward, constructing a reliable environment
where it can be used can be quite cumbersome in the context of cross-building a
kernel snap. This plugin alleviates those challenges by constructing a minimal
chroot for the target architecture where any required packages can be added to
end up in the final initrd.


Keys
----

This plugin provides the following unique keys.


initrd-addons
~~~~~~~~~~~~~

**Type**: list of strings

**Default**: ``[]``

A list of files to include in the initrd, provided as relative paths to
``${CRAFT_STAGE}/addons``.


initrd-firmware
~~~~~~~~~~~~~~~

**Type**: list of strings

**Default**: ``[]``

A list of firmware to include in the initrd, provided as relative paths to
``${CRAFT_STAGE}/firmware``.


initrd-modules
~~~~~~~~~~~~~~

**Type**: list of strings

**Default**: ``[]``

A list of module to include in the initrd, provided as a list of module names.
If the specified module(s) have dependencies, they are also installed.


For core24 and later, the following options are also supported. However, they
will only work if the target architecture has a `systemd-boot-efi <https://packages.ubuntu.com/noble/systemd-boot-efi>`_
package in the archive. Otherwise, the build will fail.


initrd-build-efi-image
~~~~~~~~~~~~~~~~~~~~~~

**Type**: bool

**Default**: ``false``

If true, a Unified Kernel Image or UKI will be built instead of a standalone
initrd compressed CPIO archive.


initrd-efi-image-key
~~~~~~~~~~~~~~~~~~~~

**Type**: string

**Default**: ``/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key``

A signing key file to use when creating a UKI EFI image, provided as a relative
path to ``${CRAFT_STAGE}/signing``.

If set, ``initrd-build-efi-image`` must be ``true`` and
``initrd-efi-image-cert`` must be specified. If not specified, the snakeoil key
provided by the ubuntu-core-initramfs package is used.


initrd-efi-image-cert
~~~~~~~~~~~~~~~~~~~~~

**Type**: string

**Default:** ``/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem``

A certificate file to use when creating a UKI EFI image, provided as a relative
path to ``${CRAFT_STAGE}/signing``.

If set, ``initrd-build-efi-image`` must be ``true`` and
``initrd-efi-image-key`` must be specified. If not specified, the snakeoil
certificate provided by the ubuntu-core-initramfs package is used.


Environment variables
---------------------

This plugin does not set its own special variables.


Dependencies
------------

The plugin includes the common build-time requirements for creating a minimal
initrd, but most of the actual dependencies are installed within the chroot
environment this plugin builds.


How it works
------------

During the build step the plugin performs the following actions:

#. Pass a collection of flags built from the selected options to an initrd
   build script within the snapcraft snap
#. Fetch a daily compressed archive base of the targeted Ubuntu series.
#. Unpack the archive into ``${CRAFT_PART_SRC}`` to use as the chroot base.
#. Copy any kernel firmware or modules from
   ``${CRAFT_STAGE}/{firmware,modules}`` into the chroot base.
#. Install any build-time dependencies such as the ubuntu-core-initramfs
   package into the chroot base.
#. Any addons, firmware, or modules are added to the chroot base as follows:

.. code:: yaml

    initrd-addons:
      - usr/bin/foo
    initrd-firmware:
        - foo/bar.bin

   Will result in ``${CRAFT_STAGE}/addons/usr/bin/foo`` and
   ``${CRAFT_STAGE}/firmware/foo/bar.bin`` being placed in the initrd as
   ``/usr/bin/foo`` and ``/usr/lib/firmware/foo/bar.bin`` by being copied into a
   directory ubuntu-core-initramfs uses to construct the initrd.

#. ubuntu-core-initramfs is called to create a compressed CPIO archive.
#. If ``initrd-build-efi-image`` is true, ubuntu-core-initramfs is called again
   to build the UKI.
#. The built ``initrd.img`` or UKI is copied to ``${CRAFT_PART_INSTALL}``.


Examples
--------

The following snippet declares a part using the Initrd plugin.

It doesn't specify a source but does include some files staged by other parts
not shown here. Those parts are responsible for ensurirng that each of they
stage these contents into ``${CRAFT_STAGE}/{addons,firmware,signing}``.

.. code-block:: yaml

    initrd:
      after: [kernel]
      plugin: initrd
      initrd-build-efi-image: true
      initrd-addons:
        - usr/bin/fde-reveal-key
        - usr/bin/fde-setup
        - usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libteec.so*
      initrd-firmware:
        - regulatory.db
        - regulatory.db.p7s

Some further examples of snaps using this plugin can be found at the following links:

* In the `snapcraft test suite <https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts>`_
* In The `IoT Field Kernel Snaps repository <https://github.com/canonical/iot-field-kernel-snap>`_
* In the `craft-examples <https://github.com/canonical/craft-examples>`_ repository
