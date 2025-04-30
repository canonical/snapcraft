.. _how-to-expand-extensions:

Expand extensions
=================

Extensions modify the snap's project file definition before a build. Run the
``expand-extensions`` command from your project's root directory to see how the project
file will look with the extensions applied.

.. terminal::
    :user: crafter
    :host: home
    :input: snapcraft expand-extensions

    name: foliate
    [...]
    layout:
      /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0:
        bind: $SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0
      /usr/share/xml/iso-codes:
        bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
    apps:
      foliate:
        command: usr/bin/com.github.johnfactotum.Foliate
        plugs:
        - gsettings
        - home
        - desktop
        - desktop-legacy
        - wayland
        - x11
        slots:
        - dbus-daemon
        common-id: com.github.johnfactotum.Foliate.desktop
        desktop: usr/share/applications/com.github.johnfactotum.Foliate.desktop
        command-chain:
        - snap/command-chain/desktop-launch
    [...]

See the :ref:`ref_commands_expand-extensions` reference for more details on this
command.
