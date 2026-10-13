.. _reference-gnome-extension:

GNOME extension
===============

The GNOME extension, referred to internally as ``gnome``, helps build snaps that use GTK
3 or 4, GNOME 42 and higher, and GLib. This extension provides many of the components needed
for general desktop apps, making it useful for a broader set of apps outside of those
tailored for the GNOME desktop.

This extension is compatible with the core22, core24, and core26 bases.


.. _gnome-extension-included-plugs:

Included plugs
--------------

When this extension is used, the following plugs are connected for the app. The paths
and default providers differ between bases.

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Included snap-wide plugs

            .. code-block:: yaml
                :caption: snapcraft.yaml

                plugs:
                  desktop:
                    mount-host-font-cache: false
                  gtk-3-themes:
                    interface: content
                    target: $SNAP/data-dir/themes
                    default-provider: gtk-common-themes
                  icon-themes:
                    interface: content
                    target: $SNAP/data-dir/icons
                    default-provider: gtk-common-themes
                  sound-themes:
                    interface: content
                    target: $SNAP/data-dir/sounds
                    default-provider: gtk-common-themes
                  gnome-core26:
                    interface: content
                    target: $SNAP/gnome-platform
                    default-provider: gnome-core26
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
                  desktop:
                      mount-host-font-cache: false
                  gtk-3-themes:
                      interface: content
                      target: $SNAP/data-dir/themes
                      default-provider: gtk-common-themes
                  icon-themes:
                      interface: content
                      target: $SNAP/data-dir/icons
                      default-provider: gtk-common-themes
                  sound-themes:
                      interface: content
                      target: $SNAP/data-dir/sounds
                      default-provider: gtk-common-themes
                  gnome-46-2404:
                      interface: content
                      target: $SNAP/gnome-platform
                      default-provider: gnome-46-2404
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
                  desktop:
                      mount-host-font-cache: false
                  gtk-3-themes:
                      interface: content
                      target: $SNAP/data-dir/themes
                      default-provider: gtk-common-themes
                  icon-themes:
                      interface: content
                      target: $SNAP/data-dir/icons
                      default-provider: gtk-common-themes
                  sound-themes:
                      interface: content
                      target: $SNAP/data-dir/sounds
                      default-provider: gtk-common-themes
                  gnome-42-2204:
                      interface: content
                      target: $SNAP/gnome-platform
                      default-provider: gnome-42-2204

The extension also connects the following plugs to all apps that use it.

.. dropdown:: Included app plugs

    .. code-block:: yaml
        :caption: snapcraft.yaml

        plugs:
          - desktop
          - desktop-legacy
          - opengl
          - wayland
          - x11

On core22 and core24, the extension also adds the ``gsettings`` plug. On core26,
apps that need this interface must declare it in their ``plugs`` key. The
:ref:`GSettings guidance <how-to-use-the-gnome-extension-gsettings>` describes
when the plug is needed and how apps store settings without it.


Included packages
-----------------

The GNOME extension is derived from two separate snaps -- a `build snap
<https://github.com/ubuntu/gnome-sdk/blob/gnome-42-2204-sdk/snapcraft.yaml>`_ and a
`platform snap
<https://github.com/ubuntu/gnome-sdk/blob/gnome-42-2204/snapcraft.yaml>`_.

The build snap compiles libraries from source that are commonly used across GNOME apps.
Examples include GLib, GTK, and gnome-desktop. These are built to provide newer versions
of these packages that exist in the core22, core24, or core26 base snaps (a subset of
their respective Ubuntu archives).

The platform snap takes the build snap and makes all of those libraries available at
build time to snaps using this extension. This way, snap authors don't need to include
the pieces of the build snap that are unnecessary at runtime, like compilers, in the
final snap.

On core26, the default build snap is gnome-core26-sdk and the platform snap is
gnome-core26.


.. _reference-gnome-extension-library-cleanup:

Library cleanup
~~~~~~~~~~~~~~~

On core26, the extension adds a ``gnome/cleanup`` part that runs after the
project's parts. During the prime step, it removes shared libraries from the
prime directory that are already supplied by the GNOME content snaps or
gtk-common-themes snap. These libraries are provided at runtime by the content snaps instead of
being bundled into the project.

If your snap bundles a newer or older version of a library provided by those
content snaps, use the provided version instead. Shipping an alternative
requires at least a distinct SONAME and corresponding filenames, with your app
linked against that library.

This cleanup prevents bundled libraries from overriding the content snap's
libraries. Without it, a content snap update could introduce a dependency on a
new symbol that an older bundled library doesn't provide, causing your app to
fail.


Included environment variables
------------------------------

In addition to using the build and platform snaps, this extension sets several
environment variables, links, and default plugs for the app to use, and a default
build-environment for each part in your snap to use.


Build variables
~~~~~~~~~~~~~~~

The following build environment variables are added to each part in a snap that uses
this extension.

You can declare additional variables in the ``build-environment`` key. Furthermore,
these default variables can be overridden by declaring them in the project file.

The paths differ between bases.

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Included build environment variables

            .. code-block:: yaml
                :caption: snapcraft.yaml

                build-environment:
                  - SNAPCRAFT_GNOME_SDK: /snap/gnome-core26-sdk/current/
                  - PATH: /snap/gnome-core26-sdk/current/usr/bin${PATH:+:$PATH}
                  - XDG_DATA_DIRS: $CRAFT_STAGE/usr/share:/snap/gnome-core26-sdk/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}
                  - LD_LIBRARY_PATH: /snap/gnome-core26-sdk/current/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR:/snap/gnome-core26-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR:/snap/gnome-core26-sdk/current/usr/lib:/snap/gnome-core26-sdk/current/usr/lib/vala-current:/snap/gnome-core26-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/pulseaudio${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
                  - PKG_CONFIG_PATH: /snap/gnome-core26-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/pkgconfig:/snap/gnome-core26-sdk/current/usr/lib/pkgconfig:/snap/gnome-core26-sdk/current/usr/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}
                  - GETTEXTDATADIRS: /snap/gnome-core26-sdk/current/usr/share/gettext-current${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}
                  - GDK_PIXBUF_MODULE_FILE: /snap/gnome-core26-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/gdk-pixbuf-current/loaders.cache
                  - ACLOCAL_PATH: /snap/gnome-core26-sdk/current/usr/share/aclocal${ACLOCAL_PATH:+:$ACLOCAL_PATH}
                  - PYTHONPATH: /snap/gnome-core26-sdk/current/usr/lib/python3.10:/snap/gnome-core26-sdk/current/usr/lib/python3/dist-packages:/snap/gnome-core26-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/gobject-introspection${PYTHONPATH:+:$PYTHONPATH}
                  - GI_TYPELIB_PATH: /snap/gnome-core26-sdk/current/usr/lib/girepository-1.0:/snap/gnome-core26-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/girepository-1.0${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}
                  - CMAKE_PREFIX_PATH: $CRAFT_STAGE/usr:/snap/gnome-core26-sdk/current/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Included build environment variables

            .. code-block:: yaml
                :caption: snapcraft.yaml

                build-environment:
                  - SNAPCRAFT_GNOME_SDK: /snap/gnome-46-2404-sdk/current/
                  - PATH: /snap/gnome-46-2404-sdk/current/usr/bin${PATH:+:$PATH}
                  - XDG_DATA_DIRS: $CRAFT_STAGE/usr/share:/snap/gnome-46-2404-sdk/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}
                  - LD_LIBRARY_PATH: /snap/gnome-46-2404-sdk/current/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR:/snap/gnome-46-2404-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR:/snap/gnome-46-2404-sdk/current/usr/lib:/snap/gnome-46-2404-sdk/current/usr/lib/vala-current:/snap/gnome-46-2404-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/pulseaudio${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
                  - PKG_CONFIG_PATH: /snap/gnome-46-2404-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/pkgconfig:/snap/gnome-46-2404-sdk/current/usr/lib/pkgconfig:/snap/gnome-46-2404-sdk/current/usr/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}
                  - GETTEXTDATADIRS: /snap/gnome-46-2404-sdk/current/usr/share/gettext-current${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}
                  - GDK_PIXBUF_MODULE_FILE: /snap/gnome-46-2404-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/gdk-pixbuf-current/loaders.cache
                  - ACLOCAL_PATH: /snap/gnome-46-2404-sdk/current/usr/share/aclocal${ACLOCAL_PATH:+:$ACLOCAL_PATH}
                  - PYTHONPATH: /snap/gnome-46-2404-sdk/current/usr/lib/python3.10:/snap/gnome-46-2404-sdk/current/usr/lib/python3/dist-packages:/snap/gnome-46-2404-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/gobject-introspection${PYTHONPATH:+:$PYTHONPATH}
                  - GI_TYPELIB_PATH: /snap/gnome-46-2404-sdk/current/usr/lib/girepository-1.0:/snap/gnome-46-2404-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/girepository-1.0${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Included build environment variables

            .. code-block:: yaml
                :caption: snapcraft.yaml

                build-environment:
                  - PATH: /snap/gnome-42-2204-sdk/current/usr/bin${PATH:+:$PATH}
                  - XDG_DATA_DIRS: $SNAPCRAFT_STAGE/usr/share:/snap/gnome-42-2204-sdk/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}
                  - LD_LIBRARY_PATH: /snap/gnome-42-2204-sdk/current/lib/$CRAFT_ARCH_TRIPLET:/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET:/snap/gnome-42-2204-sdk/current/usr/lib:/snap/gnome-42-2204-sdk/current/usr/lib/vala-current:/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/pulseaudio${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
                  - PKG_CONFIG_PATH: /snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-42-2204-sdk/current/usr/lib/pkgconfig:/snap/gnome-42-2204-sdk/current/usr/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}
                  - GETTEXTDATADIRS: /snap/gnome-42-2204-sdk/current/usr/share/gettext-current${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}
                  - GDK_PIXBUF_MODULE_FILE: /snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache
                  - ACLOCAL_PATH: /snap/gnome-42-2204-sdk/current/usr/share/aclocal${ACLOCAL_PATH:+:$ACLOCAL_PATH}
                  - PYTHONPATH: /snap/gnome-42-2204-sdk/current/usr/lib/python3.10:/snap/gnome-42-2204-sdk/current/usr/lib/python3/dist-packages:/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET/gobject-introspection${PYTHONPATH:+:$PYTHONPATH}


Runtime variables
~~~~~~~~~~~~~~~~~

The following environment variables are exported when the app runs:

.. code-block:: yaml
    :caption: snapcraft.yaml

    environment:
      SNAP_DESKTOP_RUNTIME: $SNAP/gnome-platform
      GTK_USE_PORTAL: '1'


Included layouts
----------------

This extension uses :ref:`layouts <reference-layouts>` to make certain files from the
GNOME platform snap available at well-known locations from the root of the snap filesystem.

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Included layouts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                layout:
                  /usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkitgtk-6.0:
                    bind: $SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkitgtk-6.0
                  /usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1:
                    bind: $SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1
                  /usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy:
                    bind: $SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy
                  /usr/share/xml/iso-codes:
                    bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
                  /usr/libexec/glycin-loaders:
                    bind: $SNAP/gnome-platform/usr/libexec/glycin-loaders

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Included layouts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                layout:
                  /usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1:
                    bind: $SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1
                  /usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy:
                    bind: $SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy
                  /usr/share/xml/iso-codes:
                    bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
                  /usr/share/libdrm:
                    bind: $SNAP/gpu-2404/libdrm
                  /usr/share/drirc.d:
                    symlink: $SNAP/gpu-2404/drirc.d
                  /usr/share/X11/XErrorDB:
                    symlink: $SNAP/gpu-2404/X11/XErrorDB

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Included layouts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                layout:
                  /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0:
                    bind: $SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0
                  /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.1:
                    bind: $SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.1
                  /usr/share/xml/iso-codes:
                    bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
                  /usr/share/libdrm:
                    bind: $SNAP/gnome-platform/usr/share/libdrm


Example expanded project file
-----------------------------

Here's an example of the result of Snapcraft expanding a project file, as
immediately prior to build. It demonstrates the added plugs, packages, variables, and
layouts that the GNOME extension includes in a project.

The original files were for the `GNOME System Monitor snap
<https://snapcraft.io/gnome-system-monitor>`_. These texts contain the difference
between the original file and the output of the :ref:`snapcraft expand-extensions
<ref_commands_expand-extensions>` command. Some of the text has been altered for ease of
reading.

.. tab-set::

    .. tab-item:: core26
        :sync: core26

        .. dropdown:: Expanded project file for GNOME System Monitor

            .. literalinclude:: code/gnome-extension-gnome-system-monitor-core-26-expanded.diff
                :caption: snapcraft.yaml
                :language: diff
                :lines: 3-

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Expanded project file for GNOME System Monitor

            .. literalinclude:: code/gnome-extension-gnome-system-monitor-core-24-expanded.diff
                :caption: snapcraft.yaml
                :language: diff
                :lines: 3-

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Expanded project file for GNOME System Monitor

            .. literalinclude:: code/gnome-extension-gnome-system-monitor-core-22-expanded.diff
                :caption: snapcraft.yaml
                :language: diff
                :lines: 3-
