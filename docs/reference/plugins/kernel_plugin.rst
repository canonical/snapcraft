.. _reference-kernel-plugin:

Kernel plugin
==============

The Kernel plugin is the recommended way of building kernel snaps. It helps with
both Ubuntu-specific kernels as well as kernels other upstream sources, such as
the mainline or linux-next trees.

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
file. Valid names include ``lowlatency`` and ``generic``. A list the common
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


kernel-enable-zfs-support
~~~~~~~~~~~~~~~~~~~~~~~~~

**Type**: bool

**Default**: ``false``

Enabling this option will build the ZFS kernel modules alongside the kernel.


kernel-enable-perf
~~~~~~~~~~~~~~~~~~

**Type**: bool

**Default**: ``false``

Enabling this option will build the ``perf`` binary.


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

* armhf: ``zImage``
* arm64, riscv64: ``Image``
* powerpc, ppc64el: ``vmlinux.strip``
* x86, x86_64, s390x: ``bzImage``


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

Requirements such as ``bash`` and ``perl`` usually come from the build
environment, and other requirements may include ``pahole``.


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
#. If enabled, ZFS for the target Ubuntu series is downloaded and built.
#. If enabled, the perf binary ``tools/perf`` is built.
#. The kernel image, system symbol map, and kernel config are installed to
   ``${CRAFT_PART_INSTALL}``.
#. The modules are moved to ``${CRAFT_PART_INSTALL}/modules`` and a symlink to
   it named ``${CRAFT_PART_INSTALL}/lib/modules`` is installed.
#. If a ``${CRAFT_PART_INSTALL}/lib/firmware/`` directory exists, then it is
   moved to ``${CRAFT_PART_INSTALL}/firmware`` and a symlink named
   ``${CRAFT_PART_INSTALL}/lib/firmware`` is created.


Examples
--------

The following snippet declares a part using the Kernel plugin. It specifies
the Ubuntu 22.04 kernel as the source, and so a generic
``kernel-kconfigflavour`` is used (as this is the default behavior, no option is
specified). A kernel config value is specified to remove debug information.

The linux-firmware and wireless-regdb packages are staged with this part for
convenience but are not necessarily required.

.. code-block:: yaml
   :caption: snapcraft.yaml

    parts:
      kernel:
        plugin: kernel
        source: https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/jammy
        source-depth: 1
        source-type: git
        source-branch: master
        stage-packages:
          - linux-firmware
          - wireless-regdb
        kernel-kconfigs:
          - CONFIG_DEBUG_INFO=n

Some further examples of snaps using this plugin can be found at the following links:

* In the `snapcraft test suite <https://github.com/canonical/snapcraft/tree/main/tests/spread/plugins/craft-parts>`_
* In the `IoT Field Kernel Snaps repository <https://github.com/canonical/iot-field-kernel-snap>`_
* In the `craft-examples <https://github.com/canonical/craft-examples/tree/project/c/nezha-kernel>`_ repository
