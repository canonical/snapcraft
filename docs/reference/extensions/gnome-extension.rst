.. _reference-gnome-extension:

GNOME extension
===============

The GNOME extension, referred to internally as ``gnome``, helps build snaps that use GTK
3, GNOME 42 and higher, and GLib. This extension provides many of the components needed
for general desktop apps, making it useful for a broader set of apps outside of those
tailored for the GNOME desktop.

This extension is compatible with the core22 and core24 bases.


.. _gnome-extension-included-plugs:

Included plugs
--------------

When this extension is used, the following plugs are connected for the app. The paths
slightly differ between core24 and core22 bases.

.. tab-set::

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
          - gsettings
          - opengl
          - wayland
          - x11
          - mount-observe
          - calendar-service


Included packages
-----------------

The GNOME extension is derived from two separate snaps -- a `build snap
<https://github.com/ubuntu/gnome-sdk/blob/gnome-42-2204-sdk/snapcraft.yaml>`_ and a
`platform snap
<https://github.com/ubuntu/gnome-sdk/blob/gnome-42-2204/snapcraft.yaml>`_.

The build snap compiles libraries from source that are commonly used across GNOME apps.
Examples include GLib, GTK, and gnome-desktop. These are built to provide newer versions
of these packages that exist in either the core24 or core22 base snaps (a subset of
their respective Ubuntu archives).

The platform snap takes the build snap and makes all of those libraries available at
build time to snaps using this extension. This way, snap authors don't need to include
the pieces of the build snap that are unnecessary at runtime, like compilers, in the
final snap.


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

The paths differ slightly between core24 and core22 bases.

.. tab-set::

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

This extension uses :ref:`layouts <reference-layouts>` to access files on the host. The
platform snap's GNOME JavaScript (GJS), webkit2gtk-4.0, and iso-codes are used so they
don't need to be packaged as part of the snap and would greatly inflate the size.

.. tab-set::

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Included layouts

            .. code-block:: yaml
                :caption: snapcraft.yaml

                layout:
                  /usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0:
                    bind: $SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0
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
                  /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/libgweather-4:
                    symlink: $SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/libgweather-4
                  /usr/lib/evolution-data-server:
                    symlink: $SNAP/usr/lib/evolution-data-server
                  /usr/bin/gnome-control-center:
                    symlink: $SNAP/usr/bin/gnome-control-center
                  /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0:
                    bind: $SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0
                  /usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy:
                    bind: $SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy
                  /usr/share/xml/iso-codes:
                    bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
                  /usr/share/libdrm:
                    bind: $SNAP/gnome-platform/usr/share/libdrm


Example expanded project file
-----------------------------

Here's an example of the result of Snapcraft expanding a core24-based project file, as
immediately prior to build. It demonstrates the added plugs, packages, variables, and
layouts that the GNOME extension includes in a project.

The original files were for the `GNOME System Monitor snap
<https://snapcraft.io/gnome-system-monitor>`_. These texts contain the difference
between the original file and the output of the :ref:`snapcraft expand-extensions
<ref_commands_expand-extensions>` command. Some of the text has been altered for ease of
reading.

.. tab-set::

    .. tab-item:: core24
        :sync: core24

        .. dropdown:: Expanded project file for GNOME System Monitor

            .. literalinclude:: code/gnome-extension-gnome-system-monitor-core-24-expanded.diff
                :caption: snapcraft.yaml
                :language: diff
                :lines: 3-
                :emphasize-lines: 57-66, 78-87, 100-150, 158-163, 171-173, 180-201, 210-212

    .. tab-item:: core22
        :sync: core22

        .. dropdown:: Expanded project file for GNOME System Monitor

            .. literalinclude:: code/gnome-extension-gnome-system-monitor-core-22-expanded.diff
                :caption: snapcraft.yaml
                :language: diff
                :lines: 3-
                :emphasize-lines: 60-69, 81-90, 103-155, 159-160, 164-170, 178-179, 186-207
