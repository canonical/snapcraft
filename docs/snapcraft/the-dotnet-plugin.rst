.. 8584.md

.. _the-dotnet-plugin:

The dotnet plugin
=================

The ``dotnet`` plugin is used to build `.NET Core <https://github.com/dotnet/core>`__ runtime parts.

The plugin uses the .NET SDK to install dependencies via the `NuGet <https://www.nuget.org/>`__ package manager, and follows the standard semantics for a .NET Core project.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-dotnet-plugin-core22_>`__
-  `base: core18 \| core <the-dotnet-plugin-core18_>`__

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+dotnet%22&type=Code>`__ for projects using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-dotnet-plugin-core22:

base: core22
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **dotnet-build-configuration** (string, default: *Release*) The dotnet build configuration to use.

-  **dotnet-self-contained-runtime-identifier** (string) Runtime identifier to use when building a self-contained application (e.g. *linux-x64*).

Requires Snapcraft version *7.0+*.


.. _the-dotnet-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **debug**: builds using a Debug configuration.

Requires Snapcraft version *3.x+*.
