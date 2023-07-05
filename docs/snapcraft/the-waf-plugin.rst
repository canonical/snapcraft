.. 8630.md

.. _the-waf-plugin:

The waf plugin
==============

The ``waf`` plugin is useful when integrating parts that need to use the `Waf <https://waf.io/>`__ meta build system.

   ⓘ This plugin is only available to *core* and *core18* based snaps. See :ref:`Base snaps <base-snaps>` for details.

Waf based projects are configured and built using a local *waf* Python helper. See https://gitlab.com/ita1024/waf/ for more details.

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

In addition, this plugin uses the following plugin-specific keywords:

-  **configflags** (list of strings) Configure flags to pass to the build such as those shown by running ./waf –help

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+waf%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
