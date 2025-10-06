.. _reference-dotnet-extensions:

.NET extensions
===============

The .NET extensions provide support for building and running .NET application snaps. They are versioned extensions,
which means that you can specify which extension you want to use based on the .NET major version your application
targets. The available extensions are:

* ``dotnet8``: For applications targeting .NET 8 (``net8.0``)
* ``dotnet9``: For applications targeting .NET 9 (``net9.0``)
* ``dotnet10``: For applications targeting .NET 10 (``net10.0``)

The .NET extensions are compatible with the core24 base.

Included parts
--------------

The .NET extensions add the following parts to the project file:

* ``prereqs``: A part that installs the prerequisite packages for running .NET applications. The installed packages are:

   * ``libicu74``
   * ``libssl3t64``
   * ``libunwind8``
   * ``liblttng-ust1t64``
   * ``libbrotli1``

* ``launcher``: A part that sets up the snap launcher to run the .NET application. It verifies whether the necessary
  .NET runtime is available through a content interface connection and launches the application using the ``dotnet`` command.

.. collapse:: Included parts

    .. code-block:: yaml
        :caption: snapcraft.yaml

        launcher:
            plugin: dump
            source: /snap/snapcraft/x4/share/snapcraft/extensions/dotnet
            override-build: |
              mkdir -p $CRAFT_PART_INSTALL/bin/command-chain
              cp launcher.sh $CRAFT_PART_INSTALL/bin/command-chain
            stage:
            - bin/command-chain/launcher.sh
        prereqs:
            plugin: nil
            stage-packages:
            - libicu74
            - libunwind8
            - libssl3t64
            - liblttng-ust1t64
            - libbrotli1

Included plugs
--------------

The .NET extensions connect a snap-wide content plug to the .NET runtime content snap that matches the application's target
.NET version.

.. collapse:: Included snap-wide runtime plug for the dotnet8 extension

    .. code-block:: yaml
        :caption: snapcraft.yaml

        plugs:
          dotnet8-runtime:
            content: dotnet-runtime-80
            interface: content
            target: $SNAP/opt/dotnet8
            default-provider: dotnet-runtime-80

They also connect the runtime plug in apps that use the extensions.

.. collapse:: Included app runtime plug for the dotnet8 extension

    .. code-block:: yaml
        :caption: snapcraft.yaml

        apps:
          example:
            plugs:
            - dotnet8-runtime

Included environment variables
------------------------------

The .NET extensions add the following runtime environment variables:

* ``DOTNET_ROOT``: Points to the location where the .NET runtime is mounted inside the snap.
* ``DOTNET_EXT_CONTENT_SNAP``: The name of the runtime content snap that provides the .NET runtime.
* ``DOTNET_EXT_SNAP_NAME``: The name of the snap using the extension.
* ``DOTNET_EXT_PLUG_NAME``: The name of the content plug used to connect to the .NET runtime content snap.

.. collapse:: Included runtime environment variables for the dotnet8 extension

    .. code-block:: yaml
        :caption: snapcraft.yaml

        environment:
          DOTNET_EXT_CONTENT_SNAP: dotnet-runtime-90
          DOTNET_EXT_SNAP_NAME: test-snap
          DOTNET_EXT_PLUG_NAME: dotnet9-runtime
          DOTNET_ROOT: $SNAP/opt/dotnet9/dotnet

Example expanded project file
------------------------------

Here is an example of the result of a project file that uses the ``dotnet8`` extension. It shows the added plugs,
packages, variables, and parts that the extension adds to the project file immediately prior to build.

This example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the text has been altered for
ease of reading.

.. collapse:: Expanded project file for the test-snap application

    .. literalinclude:: code/dotnet8-extension-test-app-expanded.diff
        :language: diff
        :emphasize-lines: 29-36, 38-45, 47-54, 56-63, 68-77, 81-90, 92-102
