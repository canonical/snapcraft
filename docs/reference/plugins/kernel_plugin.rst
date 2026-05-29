.. meta::
    :description: Understand the usage and meaning of the varies keys and environment variables used by the kernel plugin. See an example, test cases, and real world usage.
.. _reference-kernel-plugin:

Kernel plugin
=============

The Kernel plugin is the recommended way of building kernel snaps. It helps with
both Ubuntu-specific kernels as well as other upstream sources, such as the
mainline or linux-next trees.

The kernel build process tends to be highly repetitive and deterministic, where
most desired modifications include applying patches and modifying kernel config
values. This plugin allows for the easy toggling of kernel config values and
entire defconfigs used during builds to improve kernel snap iterability and maintainability.

This plugin also includes some keys specifically related to Ubuntu kernels. Those
keys take the form ``kernel-ubuntu``.


Keys
----

This plugin provides the following unique keys.


kernel-kdefconfig
~~~~~~~~~~~~~~~~~

**Type**: list of strings

**Default**: ``["defconfig"]``

The kernel configurations to use when generating a ``.config``. Multiple items
in the list may be specified, but their order matters. Items later in the list
take precedent over earlier items.

If ``kernel-ubuntu-debian-package`` is ``true``, then the specified kdefconfig
files must be in the annotations format and located in ``${CRAFT_PROJECT_DIR}/annotations``.

If ``kernel-ubuntu-debian-package`` is ``false``, then the specified kdefconfig
files must be in the regular kconfig fragment style and located in either
``${CRAFT_PART_SRC}/arch/${CRAFT_ARCH_BUILD_FOR}/configs`` or ``${CRAFT_PART_SRC}/kernel/configs``.


kernel-kconfigs
~~~~~~~~~~~~~~~

**Type**: list of strings

A list of kernel config values to enforce. If set, this list overrides the
values in the base configurations established by the ``kernel-kdefconfig`` and
``kernel-ubuntu-kconfigflavour`` keys. The kernel build system is used to resolve any
configuration dependencies or invalid combinations.

If ``kernel-ubuntu-debian-package`` is ``true``, this option is unavailable.
Instead, a config fragment should be put into ``${CRAFT_PROJECT_DIR}/annotations/``
and specified in the ``kernel-kdefconfig`` key.


kernel-tools
~~~~~~~~~~~~

**Type**: list of strings

A list of kernel tools to build. If set, the specified tools will be built and added to
the final snap package.

This option is incompatible with ``kernel-ubuntu-binary-package`` and instead,
the tools to include should be specified in ``stage-packages``.

Valid values are ``bpf``, ``cpupower``, and ``perf``.

.. admonition:: Warning
    :class: warning

    Building a kernel snap for a different target architecture than
    the build host may result in build failures when using this key
    with core22 snaps. Refer to the `kernel-cross spread test <https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts/kernel-cross/task.yaml>`__
    for a viable workaround.


kernel-ubuntu-kconfigflavour
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: string

**Default**: ``"generic"``

The Ubuntu kernel flavor to use when generating an Ubuntu kernel configuration
file. Valid names include ``lowlatency`` and ``generic``. A list of the common
variants can be found `here <https://ubuntu.com/kernel/variants>`_, but valid
choices will depend on the chosen kernel source. If something other than
``generic`` is provided, this value takes precedence over the ``kernel-kdefconfig`` key.


kernel-ubuntu-release-name
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: string

A string which specifies a particular Ubuntu release to build an Ubuntu kernel from.
By default, this key will fetch the ``master-next`` branch of one of the release
kernels available on Launchpad.

Valid values are Ubuntu release code names like ``jammy``, ``lunar``, or ``noble``.


kernel-ubuntu-abinumber
~~~~~~~~~~~~~~~~~~~~~~~

**Type**: string

A string which specifies a particular kernel version and, more importantly,
ABI number of the kernel package to build. This value is meaningful when
``kernel-ubuntu-release-name`` or ``kernel-ubuntu-binary-package`` are used. For the
former, this value will be used when cloning the git repository for the chosen release.
For the latter, this value will be used to specify the kernel version of the debian
package.


kernel-ubuntu-binary-package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: bool

**Default**: ``false``

If enabled, the kernel compilation process will be skipped. Instead, the latest kernel
image and, if available, modules and modules-extra packages from the Ubuntu archive will be fetched and
repackaged into a snap. A particular ABI may be specified with ``kernel-ubuntu-abinumber``.

.. admonition:: Warning
   :class: warning

   Building a kernel snap for a different target architecture than
   the build host may result in build failures when using this key
   for core22 snaps. Refer to the `kernel-cross spread test <https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts/kernel-cross/task.yaml>`__
   for a viable workaround.


kernel-ubuntu-debian-package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: bool

**Default**: ``false``

If enabled, the kernel will be built following how debian packages traditionally are.
This means that the build steps are handled by the ``debian/rules`` makefile, rather
than any direct ``make`` invocations as is done in other cases with this plugin.

This option is primarily intended to be used by the Canonical Kernel team. Its
usage requires the kernel source provides a valid ``debian/`` directory.


kernel-ubuntu-debian-dkms
~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: list of strings

A list of DKMS packages to include in the debian package build of the kernel.

This option is only meaningful when used with ``kernel-ubuntu-debian-package``.


Environment variables
---------------------

This plugin provides the following environment variables.


ARCH
~~~~

**Default:** ``${CRAFT_ARCH_BUILD_FOR}``

This value determines the target platform to build the kernel for.

KERNEL_IMAGE
~~~~~~~~~~~~

**Default:** Depends on ``${CRAFT_ARCH_BUILD_FOR}``

This value specifies the name of the target kernel image to build, and the
value is target-architecture dependent:

* amd64, s390x: ``bzImage``
* armhf: ``zImage``
* arm64, riscv64: ``Image.gz``
* ppc64el: ``zImage``

The default behavior is to build a compressed kernel image. On some systems, an
uncompressed kernel image is preferred. In these cases, ``KERNEL_IMAGE`` may be
set to the uncompressed names:

* armhf, arm64, riscv64: ``Image``

Some architectures support many different compression algorithms. For instance,
Depending on the kernel version, on arm64 and riscv64 the following could be
valid choices:

* ``Image``
* ``Image.gz``
* ``Image.xz``
* ``Image.bz2``
* ``Image.lz4``
* ``Image.zst``
* ``Image.lzma``

Ensure the chosen compressor is available in the build environment or listed in
the part's ``build-packages``.

Inspect ``arch/${arch}/boot/Makefile`` in the kernel source tree to see what
targets are valid.


KERNEL_TARGET
~~~~~~~~~~~~~

**Default:** Depends on ``${CRAFT_ARCH_BUILD_FOR}``

This value specifies the additional targets to build, and the value is
target-architecture dependent:

* x86, x86_64: ``modules``
* armhf, arm64, powerpc, ppc64el, riscv64, s390x: ``modules dtbs``


CROSS_COMPILE
~~~~~~~~~~~~~

**Default:** ``${CRAFT_ARCH_TRIPLET_BUILD_FOR}-``

This value specifies the prefix for the (cross-)compiler.


Dependencies
------------

The plugin specifies the most common build-time requirements as ``build-packages``
itself, but depending on the enabled kernel configs or environment variables some
additional ones may be required.

For example, requirements such as ``bash`` and ``perl`` usually come from the
build environment, and other requirements such as ``pahole`` may need to be
specified.


How it works
------------

There are three primary patterns for this plugin depending on the selected keys.

1. ``kernel-ubuntu-binary-package: true``
#. ``kernel-ubuntu-debian-package: true``
#. Neither


Binary package
~~~~~~~~~~~~~~

In the case where building from a binary package, most of the processes used by
this plugin are skipped. Instead, a prebuilt kernel image and its associated
modules are fetched from the archive and staged to the expected locations for a
kernel snap.

This option is the fastest route to producing a kernel snap when a kernel debian
package already exists, and no real changes need to be made to it.


Debian package
~~~~~~~~~~~~~~

This case performs a source-based build, following the standard Ubuntu kernel
build method as closely as possible. Ultimately, ``dpkg`` and its associated
tools are responsible for configuring and building a kernel and its modules,
with options to include DKMS packages from the archive or to build the supported
kernel tools.

By the end, several deb packages will be produced and then extracted and staged.
This option supplants the binary package option in cases where the kernel must
be customized in some way, but the standard debian tooling still needs to be
used to build the kernel.


Neither
~~~~~~~

This case performs a complete build of an arbitrary kernel source, whether it be
for a debian package or from some other maintainer.

This option is the most feature-rich path enabling end-to-end control over the
entire kernel, and is most useful for doing board development work when you have
a known-working kernel for the hardware and need to package it into a snap.

During the build step the plugin performs the following actions:

#. Pass a collection of flags built from the selected keys to a kernel build
   script within the Snapcraft snap.
#. Generate a ``.config`` from

   - A flavor, if something other than ``generic`` is specified.

   - A list of defconfigs, if something other than ``["defconfig"]`` is specified.

     - Otherwise, a generic Ubuntu config is constructed.

   - Any specified kernel config values are appended to the ``.config``.
#. ``make oldconfig`` is run to validate the configuration file.
#. The configuration file is checked against a minimal list of required kernel
   config values.
#. The kernel image and any modules or device tree files are built.
#. If provided, the specified list of kernel tools are built and installed to
   ``${CRAFT_PART_INSTALL}``.
#. The kernel image, system symbol map, and kernel config are installed to
   ``${CRAFT_PART_INSTALL}``.
#. The modules are moved to ``${CRAFT_PART_INSTALL}/modules`` and a symlink to
   it named ``${CRAFT_PART_INSTALL}/lib/modules`` is installed.
#. If a ``${CRAFT_PART_INSTALL}/lib/firmware/`` directory exists, then it is
   moved to ``${CRAFT_PART_INSTALL}/firmware`` and a symlink named
   ``${CRAFT_PART_INSTALL}/lib/firmware`` is created.


Examples
--------

The following snippet declares a part using the Kernel plugin. It specifies the
Ubuntu 22.04 kernel as the source via the ``kernel-ubuntu-release-name`` plugin
key, and so a generic ``kernel-ubuntu-kconfigflavour`` is used (as this is the
default behavior, no key is specified). A specific tag (Ubuntu-5.15.0-176.186)
is named with the ``kernel-ubuntu-abinumber`` key, which means that tag of the
Jammy tree will be cloned. A kernel config value is specified to remove debug
information. The kernel is then built following the standard ``make defconfig;
make; make install`` pattern rather than using tools like ``dpkg`` to produce
and unpack a deb.

The linux-firmware and wireless-regdb packages are staged with this part for
convenience but are not necessarily required.

.. code-block:: yaml
   :caption: snapcraft.yaml

    parts:
      kernel:
        plugin: kernel
        stage-packages:
          - linux-firmware
          - wireless-regdb
        kernel-kconfigs:
          - CONFIG_DEBUG_INFO=n
        kernel-ubuntu-release-name: jammy
        kernel-ubuntu-abinumber: Ubuntu-5.15.0-176.186

Some further examples of snaps using this plugin can be found at the following links:

* In the `snapcraft test suite <https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts>`_
* In the `IoT Field Kernel Snaps repository <https://github.com/canonical/iot-field-kernel-snap>`_
* In the `craft-examples <https://github.com/canonical/craft-examples/tree/project/c/nezha-kernel>`_ repository

.. LINKS
.. _kernel-cross spread test: https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts/kernel-cross/task.yaml
