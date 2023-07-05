.. 8623.md

.. _the-meson-plugin:

The meson plugin
================

The ``meson`` plugin is useful for building `Meson <https://mesonbuild.com/>`__-based parts.

Projects using the Meson build system will contain a *meson.build* file that drives the build, and the plugin runs the following commands to build your project:

1. ``meson``
2. ``ninja``
3. ``ninja install``

This plugin uses the common plugin keywords as well as those for :ref:`sources <snapcraft-parts-metadata-source>`. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+meson%22&type=Code>`__ for projects using the plugin.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-meson-plugin-core22_>`__
-  `base: core20 <the-meson-plugin-core20_>`__
-  `base: core18 \| core <the-meson-plugin-core18_>`__

..

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-meson-plugin-core22:

base: core22
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **meson-parameters** (list of strings) List of parameters to pass to the *meson* command.

Requires Snapcraft version *7.0+*.


.. _the-meson-plugin-core20:

base: core20
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **meson-parameters** (list of strings) List of parameters to pass to the *meson* command.

-  **meson-version** (string) Version of *meson* to download from `PyPI <https://pypi.org/project/meson/>`__ (e.g. 0.62.1).

Requires Snapcraft version *4.0+*.


.. _the-meson-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **meson-parameters** (list of strings) List of parameters to pass to the *meson* command.

-  **meson-version** (string) Version of *meson* to download from `PyPI <https://pypi.org/project/meson/>`__ (e.g. 0.62.1).
