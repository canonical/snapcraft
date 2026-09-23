.. _how-to-craft-a-dotnet-app:

Craft a .NET app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using .NET. We'll
work through the aspects unique to .NET-based apps by examining an existing
project file.


Example whatime project file
----------------------------

The following code comprises the project file of a .NET tool, `whatime
<https://github.com/snapcraft-docs/whatime>`_. This project is a CLI command for
returning the current time in cities across the globe.

.. dropdown:: whatime project file

    .. literalinclude:: ../code/integrations/example-dotnet-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written for .NET
---------------------------

.. literalinclude:: ../code/integrations/example-dotnet-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: - libicu70

.NET parts are built with the :ref:`craft_parts_dotnet_plugin`.

To add a .NET part:

#. Declare the general part keys, such as ``source``, ``override-build``, and
   so on.
#. Set ``plugin: dotnet``.
#. If you need to override the build configuration, set
   ``dotnet-build-configuration`` to the name of a configuration.
#. If you need to build the project as a single binary:

   #. In the ``.csproj`` file, add the following to the ``<PropertyGroup>``
      tag:

      .. code:: xml

        <PublishSingleFile>true</PublishSingleFile>

   #. Set
      ``dotnet-self-contained-runtime-identifier`` to the target architecture's
      `runtime identifier
      <https://learn.microsoft.com/en-us/dotnet/core/rid-catalog#linux-rids>`_.

#. For ``build-packages``, list any required .NET SDK packages needed for build
   time.
