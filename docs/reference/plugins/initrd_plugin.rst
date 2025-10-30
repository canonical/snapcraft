.. _reference-initrd-plugin:

Initrd plugin
==============

The Initrd plugin builds initial ramdisks, or initrds, for kernel snaps. Its
intended use is to generate the initrds for Ubuntu Core systems.

The initrd used by Ubuntu Core is constructed by the ubuntu-core-initramfs tool
and, while its usage is straightforward, constructing a reliable environment
where it can be used can be quite cumbersome in the context of building a kernel
snap where the ``build-on`` architecture is different from the ``build-for``
architecture. This plugin alleviates those challenges by constructing a minimal
chroot for the target architecture where any required packages can be added to
end up in the final initrd.


Keys
----

This plugin provides the following unique keys.


initrd-addons
~~~~~~~~~~~~~

**Type**: list of strings

A list of files to include in the initrd, provided as relative paths to
``${CRAFT_STAGE}/addons`` like so:

.. code-block:: yaml
   :caption: snapcraft.yaml

      initrd-addons:
        - usr/bin/foo
        - usr/lib/bar


initrd-firmware
~~~~~~~~~~~~~~~

**Type**: list of strings

A list of firmware to include in the initrd, provided as relative paths to
``${CRAFT_STAGE}/firmware`` like so:

.. code-block:: yaml
   :caption: snapcraft.yaml

      initrd-firmware:
        - foo
        - bar/baz


initrd-modules
~~~~~~~~~~~~~~

**Type**: list of strings

The kernel modules to add to the initrd. If a listed module has dependencies,
the plugin sources and installs them automatically.


Core22 and higher
~~~~~~~~~~~~~~~~~

For snaps on core22 and higher, the following keys are also available. However,
they will only work if the target architecture has a package in the Ubuntu
archive providing an EFI stub. Otherwise, the build will fail.


initrd-build-efi-image
^^^^^^^^^^^^^^^^^^^^^^

**Type**: bool

**Default**: ``false``

If true, a Unified Kernel Image (UKI) will be built instead of a standalone
initrd compressed CPIO archive.


initrd-efi-image-key
^^^^^^^^^^^^^^^^^^^^

**Type**: string

A signing key file to use when creating a UKI EFI executable, provided as a
relative path to ``${CRAFT_STAGE}/signing``.

If set, the ``initrd-build-efi-image`` key should be set to ``true``, and
the ``initrd-efi-image-cert`` key must also be set. If not set, the snakeoil key
provided by the ubuntu-core-initramfs package is used.


initrd-efi-image-cert
^^^^^^^^^^^^^^^^^^^^^

**Type**: string

A certificate file to use when creating a UKI EFI executable, provided as a
relative path to ``${CRAFT_STAGE}/signing``.

If set, the ``initrd-build-efi-image`` key should be ``true`` and the
``initrd-efi-image-key`` key must be specified. If not specified, the snakeoil
certificate provided by the ubuntu-core-initramfs package is used.


Dependencies
------------

The plugin includes the common build-time requirements for creating a minimal
initrd, but most of the actual dependencies are installed within the chroot
environment this plugin builds.


How it works
------------

During the build step the plugin performs the following actions:

#. Pass a collection of flags built from the selected options to an initrd
   build script within the Snapcraft snap.
#. Fetch a daily compressed archive base of the targeted Ubuntu series.
#. Unpack the archive into ``${CRAFT_PART_SRC}`` to use as the chroot base.
#. Copy any kernel firmware or modules from ``${CRAFT_STAGE}/firmware`` and
   ``${CRAFT_STAGE}/modules`` into the chroot base.
#. Install any build-time dependencies such as the ubuntu-core-initramfs
   package into the chroot base.
#. Add any addons, firmware, or modules to the chroot base.
#. Call ubuntu-core-initramfs to create a compressed CPIO archive.
#. Call ubuntu-core-initramfs again to build the UKI If the
   ``initrd-build-efi-image`` key is set to ``true``.
#. The built ``initrd.img`` or UKI is copied to ``${CRAFT_PART_INSTALL}``.


Examples
--------

The following snippet declares a part using the Initrd plugin.

The initrd being built supports full disk encryption (FDE) using OP-TEE, a
commonly used trusted execution environment on ARM platforms. It supports FDE
using OP-TEE by specifying the ``fde-reveal-key`` and ``fde-setup`` binaries and
the ``libteec.so`` for the target architecture in the ``initrd-addons`` key. This
method for FDE is commonly used on Ubuntu Core for ARM64 platforms starting with
Ubuntu Core 20.

The Initrd plugin does not necessarily require that a ``source`` be specified.
However, the files specified by both the ``initrd-addons`` and ``initrd-firmware``
keys must be provided by either this part or some other part. In this case, the
files specified by the ``initrd-addons`` key are provided by the ``uc-fde`` part
elsewhere in this ``snapcraft.yaml``. The ``uc-fde`` part places those files
within ``${CRAFT_STAGE}/addons``.

.. code-block:: yaml
   :caption: snapcraft.yaml

    initrd:
      after: [kernel, uc-fde]
      plugin: initrd
      initrd-build-efi-image: false
      initrd-addons:
        - usr/bin/fde-reveal-key
        - usr/bin/fde-setup
        - usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libteec.so*
      initrd-firmware:
        - regulatory.db
        - regulatory.db.p7s

The files specified by the ``initrd-addons`` key will appear in the initrd in
``/usr/bin`` and ``/usr/lib`` and those specified by the ``initrd-firmware`` key
will be in ``/lib/firmware``.

Some further examples of snaps using this plugin can be found at the following links:

* In the `snapcraft test suite <https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts>`_
* In the `IoT Field Kernel Snaps repository <https://github.com/canonical/iot-field-kernel-snap>`_
* In the `craft-examples <https://github.com/canonical/craft-examples/tree/project/c/nezha-kernel>`_ repository
