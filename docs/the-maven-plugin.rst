.. 4282.md

.. _the-maven-plugin:

The maven plugin
================

This plugin is useful for building parts that use maven.

The maven build system is commonly used to build Java projects. The plugin requires a pom.xml in the root of the source tree.

This plugin is only available to *core22* and *core18* based snaps. See :ref:`Base snaps <base-snaps>` for details.


base: core22 and base: core18
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **maven-options**: An array of maven command line options.

   Example:

   .. code:: yaml

       YourJavaApp:
        plugin: maven
        source: https://github.com/apache/tomcat.git
        source-tag: TOMCAT_9_0_5
        source_tag: trunk
        source-type: git
        maven-options:
          [-DskipTests=true, -Dsomarg=false]

For examples, search `GitHub <https://github.com/search?q=path%3A**%2Fsnapcraft.yaml+gopath&type=code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
