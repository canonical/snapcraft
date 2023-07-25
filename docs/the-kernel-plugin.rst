.. 8642.md

.. _the-kernel-plugin:

The kernel plugin
=================

The ``kernel`` plugin adds to the configuration options provided by the generic :ref:`kbuild <the-kbuild-plugin>` plugin to help build kernel snaps.

**Warning:** The API for this plugin is currently is unstable, and cross-compiling support is consequently classed as *experimental*.

   ⓘ This plugin is only available to *core* and *core18* based snaps. See :ref:`Base snaps <base-snaps>` for details.

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

The following kernel specific options are provided by this plugin:

-  **kernel-image-target** (yaml object, string or null for default target) The default target is *bzImage* and can be set to any specific target. For more complex cases where one would want to use the same :file:`snapcraft.yaml` file to target multiple architectures, a *yaml* object can be used. This yaml object would be a map of Debian architecture and kernel image build targets.

-  **kernel-initrd-modules** (array of string) List of modules to include in *initrd*. **Note**: kernel snaps do not provide the core boot logic which comes from Ubuntu Core OS snap. Include all modules you need for mounting *rootfs* here.

-  **kernel-with-firmware** (boolean; default: True) Use this flag to disable shipping binary firmwares.

-  **kernel-initrd-firmware** (array of string) List of firmware files to include in the *initrd*; these need to be relative paths to ``.installdir``. This option does not work if you disable building firmware.

-  **kernel-initrd-compression** (string; default: gz) Initrd compression to use. The only supported value is now ``gz``.

-  **kernel-device-trees** (array of string) List of device trees to build. The format is ``<device-tree-name>``.

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+kernel%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
