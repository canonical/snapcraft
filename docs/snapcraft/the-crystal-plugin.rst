.. 12527.md

.. _the-crystal-plugin:

The crystal plugin
==================

The ``crystal`` plugin is useful for parts using the `Crystal <https://crystal-lang.org/>`__ programming language with the `Crystal snap <https://snapcraft.io/crystal>`__. This plugin uses the common plugin keywords as well as those for :ref:`sources <snapcraft-parts-metadata-source>`. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core20 <the-crystal-plugin-core20_>`__
-  `base: core18 \| core <the-crystal-plugin-core18_>`__

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+crystal%22&type=Code>`__ for projects already using the plugin.

Brian J. Cardiff, one of Crystal’s developers, attended the 2019 Snapcraft Summit Montréal and wrote an excellent overview of how to use the plugin as part of an event write-up. See `Snapcraft Summit Montréal <https://crystal-lang.org/2019/06/19/snapcraft-summit-montreal.html>`__ for the post.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-crystal-plugin-core20:

base: core20
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

- **crystal-channel**: (string, default: *latest/stable*) The Snap Store channel to install Crystal from.
-  **crystal-build-options**: (list) Command line options to pass to ``shards build``. (e.g. ``[--release, --static]``)

Requires Snapcraft version *7.0+*.


.. _the-crystal-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

The following keyword is currently accepted by the plugin:

- **crystal-channel**: (string, default: *latest/stable*) The Snap Store channel to install Crystal from.

Requires Snapcraft version *3.7+*.
