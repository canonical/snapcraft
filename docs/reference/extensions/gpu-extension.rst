.. _reference-gpu-extension:

GPU extension
=============

The GPU extension, referred to internally as ``gpu``, provides hardware-accelerated graphics support for applications that need OpenGL, Vulkan, and other GPU capabilities.
This extension integrates the gpu-2404 content interface and sets up the necessary command chain wrapper for GPU acceleration.

This extension is compatible with the ``core24`` base only.


.. _gpu-extension-included-plugs:

Included plugs
--------------

When this extension is used, the following plug is connected for the snap:

.. dropdown:: Included snap-wide plugs

    .. code-block:: yaml
        :caption: snapcraft.yaml

        plugs:
          gpu-2404:
              interface: content
              target: $SNAP/gpu-2404
              default-provider: mesa-2404

Refer to :external+ubuntu-frame:ref:`the-gpu-2404-snap-interface` for more details on the packages and capabilities it provides.

Included packages
-----------------

The GPU extension defaults to the `mesa-2404 snap <https://snapcraft.io/mesa-2404>`_, which provides:

- Mesa graphics drivers
- OpenGL and Vulkan libraries
- Hardware video encode/decode acceleration support

The mesa-2404 snap is maintained by Canonical and provides up-to-date GPU driver support for applications built on the ``core24`` base.


Runtime wrapper
~~~~~~~~~~~~~~~

The extension adds a command chain entry that runs before the application:

.. code-block:: yaml
    :caption: snapcraft.yaml

    command-chain:
      - snap/command-chain/gpu-2404-wrapper

This wrapper script configures library paths and environment variables needed for GPU acceleration at runtime.


Included layouts
----------------

This extension uses :ref:`layouts <reference-layouts>` to provide access to X11 error database from the ``gpu-2404`` provider snap:

.. dropdown:: Included layouts

    .. code-block:: yaml
        :caption: snapcraft.yaml

        layout:
          /usr/share/X11/XErrorDB:
            symlink: $SNAP/gpu-2404/X11/XErrorDB


Included parts
--------------

The extension automatically adds a part to build and install the GPU wrapper:

.. dropdown:: Included parts

    .. code-block:: yaml
        :caption: snapcraft.yaml

        parts:
          gpu/wrapper:
            source: <extensions-data-dir>/gpu/command-chain
            plugin: make
            make-parameters:
              - GPU_WRAPPER=gpu-2404-wrapper


Example usage
-------------

Here's a simple example of using the GPU extension in a snapcraft.yaml file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    name: my-gpu-app
    base: core24
    version: '1.0'
    summary: An application using GPU acceleration
    description: |
      An application that requires hardware-accelerated graphics.

    confinement: strict

    apps:
      my-gpu-app:
        command: usr/bin/my-gpu-app
        extensions: [gpu]
        plugs:
          - opengl
          - x11
          - wayland

    parts:
      my-app:
        plugin: nil
        stage-packages:
          - my-gpu-application


Example expanded project file
-----------------------------

Here's an example showing what Snapcraft adds when the GPU extension is used.
This is the output before build, showing the expanded configuration:

.. dropdown:: Expanded project with GPU extension

    .. code-block:: diff
        :caption: snapcraft.yaml

        name: my-gpu-app
        base: core24
        version: '1.0'
        summary: An application using GPU acceleration
        description: |
          An application that requires hardware-accelerated graphics.

        confinement: strict

        +plugs:
        +  gpu-2404:
        +    interface: content
        +    target: $SNAP/gpu-2404
        +    default-provider: mesa-2404
        +
        +layout:
        +  /usr/share/X11/XErrorDB:
        +    symlink: $SNAP/gpu-2404/X11/XErrorDB
        +
        apps:
          my-gpu-app:
            command: usr/bin/my-gpu-app
        -   extensions: [gpu]
        +   command-chain:
        +     - snap/command-chain/gpu-2404-wrapper
            plugs:
              - opengl
              - x11
              - wayland

        parts:
        +  gpu/wrapper:
        +    source: <extensions-data-dir>/gpu/command-chain
        +    plugin: make
        +    make-parameters:
        +      - GPU_WRAPPER=gpu-2404-wrapper
        +
          my-app:
            plugin: nil
            stage-packages:
              - my-gpu-application


Combining with other extensions
-------------------------------

The GPU extension can be used as a base class for other desktop extensions (like GNOME and KDE) that need GPU acceleration support.
It can also be used standalone for applications that need GPU acceleration without the full desktop environment setup.

When combining extensions, the GPU extension's functionality is typically inherited by desktop extensions, so you don't need to specify both explicitly.
