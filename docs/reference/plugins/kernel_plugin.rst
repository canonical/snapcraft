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


Keys
----

This plugin provides the following unique keys.


kernel-kdefconfig
~~~~~~~~~~~~~~~~~

**Type**: list of strings

**Default**: ``["defconfig"]``

The kernel configurations to use when generating a ``.config``, such as those
found in ``arch/${arch}/configs`` or ``kernel/configs``.


kernel-kconfigflavour
~~~~~~~~~~~~~~~~~~~~~

**Type**: list of strings

**Default**: ``["generic"]``

The Ubuntu kernel flavor to use when generating an Ubuntu kernel configuration
file. Valid names include ``lowlatency`` and ``generic``. A list of the common
variants can be found `here <https://ubuntu.com/kernel/variants>`_, but valid
choices will depend on the chosen kernel source. If something other than
``generic`` is provided, this value takes precedence over the ``kernel-kdefconfig`` key.


kernel-kconfigs
~~~~~~~~~~~~~~~

**Type**: list of strings

A list of kernel config values to enforce. If set, this list overrides the values
in the base configurations established by the ``kernel-kdefconfig`` and
``kernel-kconfigflavour`` keys. The kernel build system is used to resolve any
configuration dependencies or invalid combinations.


kernel-tools
~~~~~~~~~~~~

**Type**: list of strings

A list of kernel tools to build. If set, the specified tools will be built and added to
the final snap package.

Valid values are ``perf``, ``cpupower``, and ``bpf``.


kernel-ubuntu-release-name
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: string

A string which specifies a particular Ubuntu release to build an Ubuntu kernel from.
This option will fetch the ``master-next`` branch of one of the release kernels
available on Launchpad.

Valid values are Ubuntu release codenames like ``jammy``, ``lunar``, ``noble``, etc.


kernel-ubuntu-binary-package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: bool

**Default**: ``false``

If enabled, the kernel compilation process will be skipped. Instead, the latest kernel
image, modules, modules-extra, and firmware from the Ubuntu archive will be fetched and
repackaged into a snap.

.. warning::
   Building a kernel snap for a different target architecture than the build host may
   result in build failures when using this option.


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
* arm64, riscv64: ``Image.xz``
* ppc64el: ``zImage``

The default behavior is to build a compressed kernel image. On some systems, an
uncompressed kernel image is preferred. In this cases, ``KERNEL_IMAGE`` may be
set to the uncompressed names:

* armhf, arm64, riscv64: ``Image``

Some architectures support many different compression algorithms. For instance on
arm64 and riscv64 the following are valid choices:

* ``Image``
* ``Image.gz``
* ``Image.xz``
* ``Image.bz2``
* ``Image.lz4``
* ``Image.zst``
* ``Image.lzma``

An uncompressed image is not available for amd64 or s390x.


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
itself, but depending on the enabled kernel configs some additional ones may be
required.

For example, requirements such as ``bash`` and ``perl`` usually come from the
build environment, and other requirements such as ``pahole`` may need to be
specified.


How it works
------------

During the build step the plugin performs the following actions:

#. Pass a collection of flags built from the selected options to a kernel build
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

The following snippet declares a part using the Kernel plugin. It specifies the Ubuntu
22.04 kernel as the source via the ``kernel-ubuntu-release-name`` plugin option, and so
a generic ``kernel-kconfigflavour`` is used (as this is the default behavior, no option
is specified). A kernel config value is specified to remove debug information.

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

Some further examples of snaps using this plugin can be found at the following links:

* In the `snapcraft test suite <https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts>`_
* In the `IoT Field Kernel Snaps repository <https://github.com/canonical/iot-field-kernel-snap>`_
* In the `craft-examples <https://github.com/canonical/craft-examples/tree/project/c/nezha-kernel>`_ repository
