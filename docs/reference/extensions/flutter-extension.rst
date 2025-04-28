.. _reference-flutter-extension:

Flutter extension
=================

The Flutter family of extensions help fill in common settings for software built with
the `Flutter <https://flutter.dev>`__ framework.

The Flutter extensions require Snapcraft 7 and lower and are currently only supported
with the core18 base. Snaps using cores higher than core18 should instead use the
`flutter <https://snapcraft.io/docs/flutter-plugin>`_ plugin with the `GNOME
<https://snapcraft.io/docs/gnome-extension>`_ extension.

There are four extensions in the family. Each tracks a different `Flutter build release
channel
<https://github.com/flutter/flutter/blob/master/docs/releases/Flutter-build-release-channels.md>`_:

- ``flutter-stable`` for the stable channel
- ``flutter-beta`` for the beta channel
- ``flutter-master`` for the main channel
- ``flutter-dev`` for the dev channel

When using these extensions, the Flutter plugin is optional. The plugin drives the build
process while the extension handles its dependencies.


Included parts
--------------

The Flutter extensions add the following parts to the project file. The
``gnome-3-28-extension`` part is noteworthy, as it provides many of the dependent
components that Flutter-based apps need.

.. collapse:: Included parts

    .. code-block:: yaml
        :caption: snapcraft.yaml

        gnome-3-28-extension:
          build-packages:
            - gcc
            - libgtk-3-dev
          make-parameters:
            - PLATFORM_PLUG=gnome-3-28-1804
          plugin: make
          source: $SNAPCRAFT_EXTENSIONS_DIR/desktop
          source-subdir: gnome
        flutter-extension:
          build-snaps:
            - flutter/latest/stable
          override-pull: |
            flutter channel stable
            flutter config --enable-linux-desktop
            flutter upgrade
            flutter doctor
          plugin: nil


Included interface connections
------------------------------

The Flutter extensions connect the following snap-wide plugs.

.. collapse:: Included snap-wide plugs

    .. code-block:: yaml
        :caption: snapcraft.yaml

        plugs:
          gnome-3-28-1804:
            default-provider: gnome-3-28-1804
            interface: content
            target: $SNAP/gnome-platform
          gtk-3-themes:
            default-provider: gtk-common-themes
            interface: content
            target: $SNAP/data-dir/themes
          icon-themes:
            default-provider: gtk-common-themes
            interface: content
            target: $SNAP/data-dir/icons
          sound-themes:
            default-provider: gtk-common-themes
            interface: content
            target: $SNAP/data-dir/sounds

They also connect the following plugs in apps that use the extensions.

.. collapse:: Included app plugs

    .. code-block:: yaml
        :caption: snapcraft.yaml

        plugs:
          - desktop
          - desktop-legacy
          - gsettings
          - opengl
          - wayland
          - x11


Included environment variables
------------------------------

The extensions add one runtime environment variable:

.. collapse:: Included runtime environment variables

    .. code-block:: yaml
        :caption: snapcraft.yaml

        environment:
          SNAP_DESKTOP_RUNTIME: $SNAP/gnome-platform


Included layouts
----------------

The Flutter extensions add the following `layouts
<https://snapcraft.io/docs/snap-layouts>`_:

.. collapse:: Included layouts

    .. code-block:: yaml
        :caption: snapcraft.yaml

        layout:
          /usr/share/libdrm:
            bind: $SNAP/gnome-platform/usr/share/libdrm
          /usr/share/xml/iso-codes:
            bind: $SNAP/gnome-platform/usr/share/xml/iso-codes


Example expanded project files
------------------------------

Here's an example of the result of a project file that uses a Flutter extension. It
demonstrates the added plugs, packages, variables, and layouts that the extension adds
to the project file immediately prior to build.

This example contains the difference between the original file and the output of the
:ref:`snapcraft expand-extensions <ref_commands_expand-extensions>` command. Some of the
text has been altered for ease of reading.

The file is based on the :ref:`my-flutter-app <how-to-craft-a-flutter-app>` project.

.. collapse:: Expanded project file for my-flutter-app

    .. literalinclude:: code/flutter-extension-my-flutter-app-expanded.diff
        :language: diff
        :lines: 3-
        :emphasize-lines: 14-24, 31-78
