.. _example-dotnet-app:

Example .NET app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using .NET. We'll
work through the aspects unique to .NET-based apps by examining an existing
recipe.


Example whatime recipe
----------------------

The following code comprises the recipe of a .NET project, `whatime
<https://github.com/snapcraft-docs/whatime>`_. This project is a CLI command
for returning the current time in cities across the globe.

.. collapse:: whatime recipe

  .. literalinclude:: ../code/craft-for-platforms/example-dotnet-recipe.yaml
    :language: yaml
    :lines: 2-


Add a part written for .NET
---------------------------

.. literalinclude:: ../code/craft-for-platforms/example-dotnet-recipe.yaml
  :language: yaml
  :start-at: parts:
  :end-at: - libicu70

.NET parts are built with the `dotnet plugin
<https://snapcraft.io/docs/dotnet-plugin>`_.

To add a .NET part:

#. Declare the general part keys, such as ``source``, ``override-build``, and
   so on.
#. Set ``plugin: dotnet``.
#. If you need to override the build configuration, set
   ``dotnet-build-configuration`` to the name of a configuration.
#. If you need to build the project as a single binary:

   #. In the ``.csproj`` file, add the following:

      .. code:: xml

        <PropertyGroup>
          ...
          <PublishSingleFile>true</PublishSingleFile>
        </PropertyGroup>

   #. Set
      ``dotnet-self-contained-runtime-identifier`` to the target architecture's
      `runtime identifier
      <https://learn.microsoft.com/en-us/dotnet/core/rid-catalog#linux-rids>`_.

#. For ``build-packages``, list any required .NET SDK packages needed for build
   time.
