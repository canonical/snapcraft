.. meta::
    :description: Reference documentation for the GPU extension, which enables hardware-accelerated graphics support for snaps.

.. _reference-gpu-extension:

GPU extension
=============

The GPU extension, referred to internally as ``gpu``, provides hardware-accelerated graphics support for applications that need OpenGL, Vulkan, and other GPU capabilities.
This extension adds the required GPU content interface to the snap, and sets up the necessary command chain wrapper for GPU acceleration.

This extension is compatible with the core22, core24 and core26 bases.

It integrates the provider snaps appropriate for the base.


.. _reference-gpu-extension-included-plugs:

Included plugs
--------------

When this extension is used, the following plug is connected for the snap:

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Included snap-wide plugs

            .. code-block:: yaml
                :caption: snapcraft.yaml

                plugs:
                  gpu-2604:
                      interface: content
                      target: $SNAP/gpu-2604
                      default-provider: mesa-2604

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Included snap-wide plugs

            .. code-block:: yaml
                :caption: snapcraft.yaml

                plugs:
                  gpu-2404:
                      interface: content
                      target: $SNAP/gpu-2404
                      default-provider: mesa-2404

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Included snap-wide plugs

            .. code-block:: yaml
                :caption: snapcraft.yaml

                plugs:
                  graphics-core22:
                      interface: content
                      target: $SNAP/graphics-core22
                      default-provider: mesa-core22

:external+ubuntu-frame:ref:`use-snap-graphics` in the Ubuntu Frame documentation shows how to include the GPU content provider snap.


Included packages
-----------------

The GPU extension defaults to snap providers that include:

- Mesa graphics drivers
- OpenGL and Vulkan libraries
- Hardware video encode/decode acceleration support
- VA-API and VDPAU support

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        The `mesa-2604 snap <https://snapcraft.io/mesa-2604>`_ is maintained by Canonical and provides up-to-date GPU driver support for applications built on the ``core26`` base.

    .. tab-item:: core24
        :sync: core24

        The `mesa-2404 snap <https://snapcraft.io/mesa-2404>`__ is maintained by Canonical and provides up-to-date GPU driver support for apps built on the core24 base.

    .. tab-item:: core22
        :sync: core22

        The `mesa-core22 snap <https://snapcraft.io/mesa-core22>`__ is maintained by Canonical and provides GPU driver support for apps built on the core22 base. It also supports Nvidia drivers installed with Debian packages on the host system.


Runtime wrapper
~~~~~~~~~~~~~~~

The extension adds a command chain entry that runs before the application:

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. code-block:: yaml
            :caption: snapcraft.yaml

            command-chain:
              - snap/command-chain/gpu-2604-wrapper

    .. tab-item:: core24
        :sync: core24

        .. code-block:: yaml
            :caption: snapcraft.yaml

            command-chain:
              - snap/command-chain/gpu-2404-wrapper

    .. tab-item:: core22
        :sync: core22

        .. code-block:: yaml
            :caption: snapcraft.yaml

            command-chain:
              - snap/command-chain/graphics-core22-wrapper

These wrapper scripts configure library paths and environment variables needed for GPU acceleration at runtime.


Included layouts
----------------

This extension uses :ref:`layouts <reference-layouts>` to provide access to GPU resources:

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Included layouts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                layout:
                  /usr/share/X11/XErrorDB:
                    symlink: $SNAP/gpu-2604/X11/XErrorDB

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Included layouts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                layout:
                  /usr/share/X11/XErrorDB:
                    symlink: $SNAP/gpu-2404/X11/XErrorDB

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Included layouts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                layout:
                  /usr/share/libdrm:
                    bind: $SNAP/graphics-core22/libdrm
                  /usr/share/drirc.d:
                    symlink: $SNAP/graphics-core22/drirc.d
                  /usr/share/X11/XErrorDB:
                    symlink: $SNAP/graphics-core22/X11/XErrorDB
                  /usr/share/X11/locale:
                    symlink: $SNAP/graphics-core22/X11/locale


Included parts
--------------

The extension automatically adds parts to build and install the GPU wrapper and perform cleanup:

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Included parts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                parts:
                  gpu/wrapper:
                    source: <extensions-data-dir>/gpu/command-chain
                    plugin: make
                    make-parameters:
                      - GPU_INTERFACE=gpu-2604
                  gpu/cleanup:
                    after:
                      - <all-user-parts>
                    source: https://github.com/canonical/gpu-snap.git
                    plugin: nil
                    override-prime: |
                      craftctl default
                      ${CRAFT_PART_SRC}/bin/graphics-gpu-2604-cleanup mesa-2604
                      # Workaround for https://bugs.launchpad.net/snapd/+bug/2055273
                      mkdir -p "${CRAFT_PRIME}/gpu-2604"

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Included parts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                parts:
                  gpu/wrapper:
                    source: <extensions-data-dir>/gpu/command-chain
                    plugin: make
                    make-parameters:
                      - GPU_INTERFACE=gpu-2404
                  gpu/cleanup:
                    after:
                      - <all-user-parts>
                    source: https://github.com/canonical/gpu-snap.git
                    plugin: nil
                    override-prime: |
                      craftctl default
                      ${CRAFT_PART_SRC}/bin/gpu-2404-cleanup mesa-core24
                      # Workaround for https://bugs.launchpad.net/snapd/+bug/2055273
                      mkdir -p "${CRAFT_PRIME}/gpu-2404"

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Included parts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                parts:
                  gpu/wrapper:
                    source: <extensions-data-dir>/gpu/command-chain
                    plugin: make
                    make-parameters:
                      - GPU_INTERFACE=graphics-core22
                  gpu/cleanup:
                    after:
                      - <all-user-parts>
                    source: https://github.com/canonical/gpu-snap.git
                    plugin: nil
                    override-prime: |
                      craftctl default
                      ${CRAFT_PART_SRC}/bin/graphics-core22-cleanup mesa-core22
                      # Workaround for https://bugs.launchpad.net/snapd/+bug/2055273
                      mkdir -p "${CRAFT_PRIME}/graphics-core22"


Example usage
-------------

Here's a simple example of using the GPU extension in a project file:

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. code-block:: yaml
            :caption: snapcraft.yaml

            name: my-gpu-app
            base: core26
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

    .. tab-item:: core24
        :sync: core24

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

    .. tab-item:: core22
        :sync: core22

        .. code-block:: yaml
            :caption: snapcraft.yaml

            name: my-gpu-app
            base: core22
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

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Expanded project with GPU extension

            .. code-block:: diff
                :caption: snapcraft.yaml

                name: my-gpu-app
                base: core26
                version: '1.0'
                summary: An application using GPU acceleration
                description: |
                  An application that requires hardware-accelerated graphics.

                confinement: strict

                +plugs:
                +  gpu-2604:
                +    interface: content
                +    target: $SNAP/gpu-2604
                +    default-provider: mesa-2604
                +
                +layout:
                +  /usr/share/X11/XErrorDB:
                +    symlink: $SNAP/gpu-2604/X11/XErrorDB
                +
                apps:
                  my-gpu-app:
                    command: usr/bin/my-gpu-app
                -   extensions: [gpu]
                +   command-chain:
                +     - snap/command-chain/gpu-2604-wrapper
                    plugs:
                      - opengl
                      - x11
                      - wayland

                parts:
                +  gpu/wrapper:
                +    source: <extensions-data-dir>/gpu/command-chain
                +    plugin: make
                +    make-parameters:
                +      - GPU_INTERFACE=gpu-2604
                +
                +  gpu/cleanup:
                +    after:
                +      - my-app
                +    source: https://github.com/canonical/gpu-snap.git
                +    plugin: nil
                +    override-prime: |
                +      craftctl default
                +      ${CRAFT_PART_SRC}/bin/graphics-gpu-2604-cleanup mesa-2604
                +      # Workaround for https://bugs.launchpad.net/snapd/+bug/2055273
                +      mkdir -p "${CRAFT_PRIME}/gpu-2604"
                +
                  my-app:
                    plugin: nil
                    stage-packages:
                      - my-gpu-application

    .. tab-item:: core24
        :sync: core24

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
                +      - GPU_INTERFACE=gpu-2404
                +
                +  gpu/cleanup:
                +    after:
                +      - my-app
                +    source: https://github.com/canonical/gpu-snap.git
                +    plugin: nil
                +    override-prime: |
                +      craftctl default
                +      ${CRAFT_PART_SRC}/bin/gpu-2404-cleanup mesa-core24
                +      # Workaround for https://bugs.launchpad.net/snapd/+bug/2055273
                +      mkdir -p "${CRAFT_PRIME}/gpu-2404"
                +
                  my-app:
                    plugin: nil
                    stage-packages:
                      - my-gpu-application

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Expanded project with GPU extension

            .. code-block:: diff
                :caption: snapcraft.yaml

                name: my-gpu-app
                base: core22
                version: '1.0'
                summary: An application using GPU acceleration
                description: |
                  An application that requires hardware-accelerated graphics.

                confinement: strict

                +plugs:
                +  graphics-core22:
                +    interface: content
                +    target: $SNAP/graphics-core22
                +    default-provider: mesa-core22
                +
                +layout:
                +  /usr/share/libdrm:
                +    bind: $SNAP/graphics-core22/libdrm
                +  /usr/share/drirc.d:
                +    symlink: $SNAP/graphics-core22/drirc.d
                +  /usr/share/X11/XErrorDB:
                +    symlink: $SNAP/graphics-core22/X11/XErrorDB
                +  /usr/share/X11/locale:
                +    symlink: $SNAP/graphics-core22/X11/locale
                +
                apps:
                  my-gpu-app:
                    command: usr/bin/my-gpu-app
                -   extensions: [gpu]
                +   command-chain:
                +     - snap/command-chain/graphics-core22-wrapper
                    plugs:
                      - opengl
                      - x11
                      - wayland

                parts:
                +  gpu/wrapper:
                +    source: <extensions-data-dir>/gpu/command-chain
                +    plugin: make
                +    make-parameters:
                +      - GPU_INTERFACE=graphics-core22
                +
                +  gpu/cleanup:
                +    after:
                +      - my-app
                +    source: https://github.com/canonical/gpu-snap.git
                +    plugin: nil
                +    override-prime: |
                +      craftctl default
                +      ${CRAFT_PART_SRC}/bin/graphics-core22-cleanup mesa-core22
                +      # Workaround for https://bugs.launchpad.net/snapd/+bug/2055273
                +      mkdir -p "${CRAFT_PRIME}/graphics-core22"
                +
                  my-app:
                    plugin: nil
                    stage-packages:
                      - my-gpu-application


Combining with other extensions
-------------------------------

The GPU extension is used as a base class for the GNOME and KDE extensions.

The GPU extension shouldn't be listed when using the GNOME or KDE extensions, as its functionality is already included.
