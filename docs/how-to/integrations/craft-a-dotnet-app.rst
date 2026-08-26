.. _how-to-craft-a-dotnet-app:

Craft a .NET app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using .NET. We'll
work through the aspects unique to .NET-based apps by examining two existing
project files: a .NET command-line tool and an Avalonia desktop app.


Craft a .NET command-line app
-----------------------------

The whatime example targets core22, where .NET parts use the original
:ref:`craft_parts_dotnet_plugin`. For core24 and newer bases, see the Avalonia
example below.


Example whatime project file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following code comprises the project file of a .NET tool, `whatime
<https://github.com/snapcraft-docs/whatime>`_. This project is a CLI command for
returning the current time in cities across the globe.

.. dropdown:: whatime project file

    .. literalinclude:: ../code/integrations/example-dotnet-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written for .NET
~~~~~~~~~~~~~~~~~~~~~~~~~~~

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


Craft a .NET Avalonia app
-------------------------

`Avalonia <https://avaloniaui.net>`_ is a cross-platform UI framework for
building .NET desktop apps. Unlike command-line tools, Avalonia apps need
access to the desktop session and display server, and require extra rendering
libraries at runtime.

On core24 and newer bases, the ``dotnet`` plugin can download the .NET SDK
itself, and the :ref:`reference-dotnet-extensions` connect the app to a shared
.NET runtime content snap at runtime. Together they remove the need to declare
the SDK as a build package or bundle the runtime inside the snap.


Example XamlPlayground project file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following code comprises the project file of an Avalonia app, `XamlPlayground
<https://github.com/AvaloniaUI/XamlPlayground>`_. This project is a desktop app
for experimenting with XAML markup and seeing the results in real time. It's
`published on the Snap Store <https://snapcraft.io/avalonia-xaml-playground>`_,
and its packaging is maintained in the `community snap repository
<https://github.com/mateusrodrigues/avalonia-xaml-playground-snap>`_.

.. dropdown:: XamlPlayground project file

    .. literalinclude:: ../code/integrations/example-dotnet-avalonia-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Declare the supported platforms
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../code/integrations/example-dotnet-avalonia-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: platforms:
    :end-at: arm64:

The ``dotnet`` plugin builds with a .NET SDK that's only published for the
amd64 and arm64 architectures, so an Avalonia snap can only target those
architectures.

To declare the supported platforms, list ``amd64`` and ``arm64`` in the
top-level ``platforms`` key. See :ref:`how-to-select-platforms` for the
available ways to declare platforms.


Add the .NET part
~~~~~~~~~~~~~~~~~

.. literalinclude:: ../code/integrations/example-dotnet-avalonia-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: opt/xamlplayground/

On core24 and newer bases, .NET parts are built with the
:ref:`craft_parts_dotnet_v2_plugin`. Both plugin versions are declared as
``plugin: dotnet`` in the project file; Snapcraft selects the version from
the project's base.

To add the .NET part:

#. Declare the general part keys, such as ``source``, ``override-build``, and
   so on.
#. Set ``plugin: dotnet``.
#. Set ``dotnet-version`` to the version of the .NET SDK to download and build
   with. It must match the framework version the project targets, as declared
   by the ``<TargetFramework>`` tag in the ``.csproj`` file. XamlPlayground
   targets ``net10.0``, so the version is ``"10.0"``.
#. If the repository contains multiple projects, set ``dotnet-project`` to the
   path of the project file that builds the app.
#. If you need to override the build configuration, set
   ``dotnet-configuration`` to the name of a configuration.
#. The plugin publishes the app directly into the part's install directory. To
   keep the published files separate from the system directories of the snap,
   use the ``organize`` key to move them into a dedicated directory, such as
   ``opt/xamlplayground/``.


Stage the rendering dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../code/integrations/example-dotnet-avalonia-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: prereqs:
    :end-at: - libx11-6

Avalonia apps need font and X11 client libraries at runtime that the .NET
runtime content snap doesn't provide. Stage them with a separate part that
runs after the .NET part:

#. Declare a part with ``plugin: nil``.
#. Set ``after`` to the name of the .NET part, so that the runtime libraries
   aren't accidentally overwritten.
#. For ``stage-packages``, list the font and X11 client libraries
   ``fontconfig``, ``libfontconfig1``, ``libice6``, ``libsm6``, and
   ``libx11-6``.


Add an app that uses Avalonia
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../code/integrations/example-dotnet-avalonia-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: apps:
    :end-at: - wayland

Avalonia apps use the versioned .NET extension that matches the framework
version the app targets, such as ``dotnet10`` for ``net10.0``. The extension
configures the runtime environment of the app and connects it to the shared
.NET runtime content snap, so that the runtime doesn't need to be bundled in
the snap. See :ref:`reference-dotnet-extensions` for the details of what the
extensions add to the project file.

To add the Avalonia app:

#. Declare the general app keys, such as ``command``. The command is the path
   of the published binary inside the snap, including the directory set with
   the part's ``organize`` key.
#. Set ``desktop`` to the path of the app's ``.desktop`` file, so that the app
   appears in the desktop environment's app grid.
#. For ``extensions``, add the .NET extension that matches the app's target
   framework.
#. For ``plugs``, add the interfaces the app needs to access the desktop
   session and render its windows:

   - ``desktop`` and ``desktop-legacy`` for integration with the desktop
     session.
   - ``wayland`` and ``opengl`` for display output and graphics acceleration.
   - ``home`` for access to the user's files.
   - ``unity7`` for desktop integration on X11 sessions.

If the app needs network access, such as to fetch online content, add the
``network`` interface as well.


Silence the library linter warnings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../code/integrations/example-dotnet-avalonia-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: lint:

Avalonia apps ship some libraries, such as the .NET runtime's own copies of
``libicu``, that the library linter reports as unused or conflicting. These
warnings are expected, and you can silence them with the top-level ``lint``
key. See :ref:`reference-linters` for how to configure which warnings a snap
ignores.
