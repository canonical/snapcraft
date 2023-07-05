.. 18746.md

.. _the-flutter-plugin:

The flutter plugin
==================

This ``flutter`` plugin is useful for building `Flutter <https://flutter.dev/>`__ based parts. This plugin uses the common plugin keywords as well as those for :ref:`sources <snapcraft-parts-metadata-source>`. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-flutter-plugin-core22_>`__
-  `base: core18 <the-flutter-plugin-core18_>`__

See :ref:`Flutter applications <flutter-applications>` for a simple example, or search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+flutter%22&type=Code>`__ for projects using the plugin.

Further examples can also be found in the `Ubuntu Flutter Community <https://github.com/ubuntu-flutter-community/>`__ on GitHub.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-flutter-plugin-core22:

base: core22
~~~~~~~~~~~~

-  **flutter-channel** (enum: [stable, master, beta], default: *stable*) The default flutter channel to use for the build.
-  **flutter-target** (string, default: *lib/main.dart*) The flutter target to build.

Requires Snapcraft version *7.3+*.


.. _the-flutter-plugin-core18:

base: core18
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **flutter-revision** (string) Defines which Flutter revision to use for the build. This must be a valid revision from the `Flutter repository <https://github.com/flutter/flutter>`__.
-  **flutter-target** (string, default: *lib/main.dart*) The main entry-point file of the application.

Requires Snapcraft version *4.1.1+*.
