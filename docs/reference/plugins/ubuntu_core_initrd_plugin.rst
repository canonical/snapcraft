
.. _reference-ubuntu-core-initrd-plugin:

Ubuntu Core Initrd plugin
=========================


The Ubuntu Core Initrd plugin builds initramfs for Ubuntu Core based systems.
This plugin can be used with the Ubunut Kernel plugin to produce a bootable kernel snap
package for a device.

Keys
----

ubuntu-core-initrd-compression
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
**Type** string

**Default** zstd

The compression method supports the following formats:

* bzip2
* gzip
* gz
* lz4
* lzma
* xz
* zstd


ubuntu_core_initrd_efi_image_type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
**Type** string

**Default** none

Specify whether the image uses EFI and whether it is signed or unsigned. The following
options are available:

* signed - using a signed EFI image
* unsigned - using an unsigned EFI image
* none - not using EFI

ubuntu_core_initrd_overlay
~~~~~~~~~~~~~~~~~~~~~~~~~~
**Type** path

**Default** None

This is an optional field. When provided it is a directory containing files that should
be copied into the initrd. The copy will overwrite any files that may be produced
by the plugin itself.

ubuntu_core_initrd_extra_modules
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
**Type** list[string]

**Default** None

A list of additional kernel modules to include in the initramfs. Some kernels for
embedded targets required additional modules to be available in the initramfs during
boot. The modules must be already available in the kernel part. If a module is required
that was not built with the kernel part you can use the overlay property to copy the
required files into the initramfs.

Example
-------

This example shows the minimal plugin part that builds a generic noble kernel
from source.

.. code-block:: yaml

    name: ucore-initrd
    summary: Test the initrd snap plugin
    description: This is a kernel initramfs snap
    version: test
    grade: devel
    confinement: devmode
    base: core24
    platforms:
      amd64:
        build-on: amd64
        build-for: arm64

    package-repositories:
     - type: apt
       components: [main]
       suites: [noble]
       key-id: 78E1918602959B9C59103100F1831DDAFC42E99D
       url: https://ppa.launchpadcontent.net/snappy-dev/image/ubuntu

    parts:
        ubuntu-kernel:
            plugin: ubuntu-kernel
            ubuntu-kernel-release-name: noble
            ubuntu-kernel-flavour: raspi
            ubuntu-kernel-use-binary-package: true

        ubuntu-core-initrd:
            plugin: ubuntu-core-initrd
            ubuntu-core-initrd-compression: gzip
            after:
              - ubuntu-kernel
