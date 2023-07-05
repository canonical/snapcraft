.. 8511.md

.. _the-gulp-plugin:

The gulp plugin
===============

The ``gulp`` plugin can be used with JavaScript projects using `gulp.js <https://gulpjs.com/>`__, the streaming build system.

The plugin uses *gulp* to drive the build, and requires a ``gulpfile.js`` in the root of the source.

   ⓘ This plugin is only available to *core* and *core18* based snaps. See :ref:`Base snaps <base-snaps>` for details.

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Additionally, this plugin uses the following plugin-specific keywords:

-  **gulp-tasks** (list) A list of gulp tasks to run.
-  **node-engine** (string) The version of *node.js* to use for the build.

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+gulp%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
