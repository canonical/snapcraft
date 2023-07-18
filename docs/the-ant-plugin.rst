.. 8507.md

.. _the-ant-plugin:

The ant plugin
==============

The ``ant`` plugin is useful for `Apache Ant <https://ant.apache.org/>`__-based parts and is commonly used to build Java projects with the `Ant snap <https://snapcraft.io/ant>`__.

This plugin installs the specified version of *ant* and runs the configured build. Any jar files created by the build inside the target directory, directly inside the root of the source tree, will be copied to ``${SNAP}/jar`` in the resulting archive.

``CLASSPATH`` will be set to the list of jars in ``${SNAP}/jar`` and ``bin/java`` is available as a symlink to the installed version of OpenJDK.

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

This plugin is only available to *core22* and *core18* based snaps. See :ref:`Base snaps <base-snaps>` for details.


base: core22 and base: core18
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **ant-build-targets** (list of strings) Run the given ant targets.

-  **ant-buildfile** (string) The path, relative to the root of the source tree, to the Ant buildfile to use. Defaults to a *build.xml* file in the root of the source tree.

-  **ant-channel** (string) When not using the Ant tarball from the Ant archive (see ``ant-version`` and ``ant-version-checksum`` above), this keyword specifies the channel to use for `Apache Ant in the Snap Store <https://snapcraft.io/ant>`__. Defaults to ``latest/stable``.

-  **ant-openjdk-version** (string) OpenJDK version available to the base to use. If not set the latest version available to the base will be used.

-  **ant-properties** (object) A dictionary of key-value pairs. Set the following properties when running ant.

-  **ant-version** (string) The version of ant you want to use to build the source artefacts. Defaults to the current release downloadable from https://archive.apache.org/dist/ant/binaries/.

-  **ant-version-checksum** (string) The checksum for ant-version in the form of /. Example: ``sha512/2a803f578f341e164f6753e410413d16ab60fab...``

For examples, search `GitHub <https://github.com/search?q=path%3A**%2Fsnapcraft.yaml+maven&type=code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
