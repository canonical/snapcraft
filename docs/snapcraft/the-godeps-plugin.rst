.. 8506.md

.. _the-godeps-plugin:

The godeps plugin
=================

The ``godeps`` plugin can be used by `godeps <https://github.com/tools/godep>`__-enabled *Go* projects.

   ⓘ This plugin is only available to *core* and *core18* based snaps. See :ref:`Base snaps <base-snaps>` for details.

These projects have a file containing information about the project’s dependencies. This file is typically called “dependencies.tsv,” but may be named anything.

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Additionally, this plugin uses the following plugin-specific keywords:

-  **go-packages** (list of strings) Go packages to build/install. These packages must be a “main” package. Dependencies should have already been retrieved by the ``godeps-file`` used for this part. Packages that are not “main” will not cause an error, but would not be useful either.

-  **godeps-file** (string) Path to the godeps dependencies file contained within the source (default: dependencies.tsv)

-  **go-importpath** (string) This entry tells the checked out ``source`` to live within a certain path within ``GOPATH``. This is required in order to work with absolute imports and import path checking.

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+godeps%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
