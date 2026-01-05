
.. _reference-ubuntu-kernel-plugin:

Ubuntu Kernel plugin
====================

The Ubuntu Kernel plugin builds signed Ubuntu kernels using official Debian build
rules. It supports customized kernel packages that can target Ubuntu snap-based
systems.

The plugin supports building Ubuntu kernel snaps from existing Debian packages or from
source. When building from source the optional `source` key may be used to provide a
custom source repository URL. If not provided, the official Canonical Ubuntu kernel
source repository will be used.

Currently, signed kernel snaps can only be built using the official Canonical kernel
Debian packages as the source.

Keys
----

ubuntu-kernel-flavour
~~~~~~~~~~~~~~~~~~~~~
**Type** string

**Default** "generic"

The Ubuntu kernel flavour to pack. Ubuntu kernels come in different flavours that may
target different architectures and configurations.

ubuntu-kernel-dkms
~~~~~~~~~~~~~~~~~~
**Type** list[string]

**Default** []

**Note** If provided, the key `ubuntu-kernel-use-binary-package` must be false.

List of dynamic kernel modules to include in the kernel pack. When not provided, the DKMS
list will be the default list provided by the Ubuntu kernel source.

ubuntu-kernel-release-name
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type** string | None

**Default** None

**Note** This key is mutually exclusive with the `source` key.

If provided, this key specifies the Ubuntu release name to build, e.g. Jammy, Noble.
The release name dictates which kernel version will be built.

If not provided, a `source` key must be provided.

ubuntu-kernel-defconfig
~~~~~~~~~~~~~~~~~~~~~~~

**Type** path | None

**Default** None

**Note** If provided, the key `ubuntu-kernel-use-binary-package` must be false.

Path to a kernel defconfig file for custom kernel configuration.

ubuntu-kernel-config
~~~~~~~~~~~~~~~~~~~~

**Type** list[string]

**Default** []

**Note** If provided, the key `ubuntu-kernel-use-binary-package` must be false.

List of kernel config options and values. Useful if only a few kernel configuration
entries need to be modified.

ubuntu-kernel-image-target
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type** string | None

**Default** None

**Note** If provided, the key `ubuntu-kernel-use-binary-package` must be false.

Specify the kernel image target type if different from the platform image type.
The default target platform image type is defined as:

========= =============
Target    Image
========= =============
amd64     bzImage
arm64     Image.gz
armhf     zImage
i386      bzImage
powerpc   uImage
ppc64el   vmlinux.strip
riscv64   Image
s390x     bzImage
========= =============

ubuntu-kernel-tools
~~~~~~~~~~~~~~~~~~~

**Type** list[string]

**Default** []

**Note** If provided, the key `ubuntu-kernel-use-binary-package` must be false.

List of kernel tools to package. Supported tools are: ``cpupower``, ``perf``,
``bpftool``. If not provided, the default tools for the kernel flavour will be
packaged.

ubuntu-kernel-use-binary-package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type** bool

**Default** false

If true, build the kernel snap using existing kernel Debian packages. The key
`ubuntu-kernel-release-name` will define which Ubuntu kernel packages are used
as the source.


Example
-------

This example shows the minimal plugin part that builds a generic noble kernel
from source.

.. code-block:: yaml

    name: kernel-noble
    summary: Noble kernel snap package.
    description: |
        Build a kernel snap package of the latest Noble release.
    version: test
    grade: devel
    confinement: devmode
    base: core24
    platform:
        arm64:
            build-on: amd64
            build-for: arm64
    parts:
        ubuntu-kernel:
            plugin: ubuntu-kernel
            ubuntu-kernel-release-name: noble


Example
-------

This example part builds a noble kernel snap from an existing signed Ubuntu
kernel apt package.

.. code-block:: yaml

    parts:
        ubuntu-kernel:
            plugin: ubuntu-kernel
            ubuntu-kernel-release-name: noble
            ubuntu-kernel-use-binary-package: true


Example
-------
This part example shows how to build the noble kernel from source with
additional dkms packages.

.. code-block:: yaml

    parts:
        ubuntu-kernel:
            plugin: ubuntu-kernel
            ubuntu-kernel-release-name: noble
            ubuntu-kernel-dkms: [
              "vpoll-dkms",
              "r8125-dkms"
            ]

Example
-------
This part example builds an Ubuntu kernel from source. Any source can be used.

.. code-block:: yaml

    parts:
        ubuntu-kernel:
            plugin: ubuntu-kernel
            source: https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/noble
            source-type: git
            source-branch: master-next
            source-depth: 1
