.. _example-gtk2-app:

Example GTK2 app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using GTK4 and GNOME. We'll
work through the aspects unique to GTK4-based apps by examining an existing
recipe.


Example Arduino IDE recipe
--------------------------

The following code comprises the recipe of a GTK2 project, the legacy `Arduino
IDE <https://github.com/arduino/Arduino>`_.

.. collapse:: Arduino IDE recipe

  .. code:: yaml

    name: arduino
    title: Arduino IDE
    version: 1.8.12
    summary: Write code and upload it to your Arduino-compatible board.
    description: |
        Arduino is an open-source physical computing platform based on a simple
        I/O board and a development environment that implements the
        Processing/Wiring language. Arduino can be used to develop stand-alone
        interactive objects or can be connected to software on your computer
        (e.g. Flash, Processing and MaxMSP). The boards can be assembled by
        hand or purchased preassembled at https://arduino.cc.
    license: GPL-2.0
    icon: snap/gui/arduino.png
    grade: stable

    base: core18
    confinement: strict

    architectures:
        - build-on: amd64
        - build-on: i386
        - build-on: armhf
        - build-on: arm64

    apps:
        arduino:
            command: desktop-launch $SNAP/arduino-snap
            environment:
                # Fallback to XWayland if running in a Wayland session.
                DISABLE_WAYLAND: 1
            plugs:
                - x11
                - unity7
                - home
                - network
                - serial-port
                - raw-usb
        builder:
            command: arduino-builder

    parts:
        upstream:
            source:
                - on amd64: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linux64.tar.xz
                - on i386: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linux32.tar.xz
                - on armhf: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linuxarm.tar.xz
                - on arm64: https://www.arduino.cc/download.php?f=/arduino-$SNAPCRAFT_PROJECT_VERSION-linuxaarch64.tar.xz
          source-type: tar
          plugin: dump
          prime:
              - -install.sh
              - -uninstall.sh
              - -java/lib/ext/jfxrt.jar
              - -java/lib/jfxswt.jar
              - -java/lib/*/libjfxwebkit.so
              - -java/lib/*/libjfxmedia.so
          stage-packages:
              - libxtst6
              - libcanberra-gtk-module
          after:
              - desktop-gtk2

        launcher:
            plugin: dump
            source: scripts
            prime:
                - arduino-snap

        desktop-gtk2:
            source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
            source-subdir: gtk
            plugin: make
            make-parameters: ["FLAVOR=gtk2"]
            build-packages:
                - build-essential
                - libgtk2.0-dev
            stage-packages:
                - libxkbcommon0  # XKB_CONFIG_ROOT
                - ttf-ubuntu-font-family
                - dmz-cursor-theme
                - light-themes
                - adwaita-icon-theme
                - gnome-themes-standard
                - shared-mime-info
                - libgtk2.0-0
                - libgdk-pixbuf2.0-0
                - libglib2.0-bin
                - libgtk2.0-bin
                - unity-gtk2-module
                - locales-all
                - libappindicator1
                - xdg-user-dirs
                - ibus-gtk
                - libibus-1.0-5

    plugs:
        gtk-2-engines:
            interface: content
            target: $SNAP/lib/gtk-2.0
            default-provider: gtk2-common-themes
        gtk-2-themes:
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


Add an app that uses GNOME
--------------------------

.. code:: yaml

  apps:
      arduino:
          command: desktop-launch $SNAP/arduino-snap
          environment:
              # Fallback to XWayland if running in a Wayland session.
              DISABLE_WAYLAND: 1
          plugs:
              - x11
              - unity7
              - home
              - network
              - serial-port
              - raw-usb

Apps that use GTK2 and GNOME as runtime libraries require a special script. It
brings in the runtime environment and dependencies so that all desktop
functionality is correctly initialised.

To add a GTK2 app:

#. Declare the general app keys, such as ``command``,
   ``plugs``, ``after``, and so on.
#. Set ``command: desktop-launch $SNAP/<command>``.
#. Since GTK2 doesn't support Wayland, it's best to set ``DISABLE_WAYLAND``
   to ``1``. Doing so instructs the app to fall back to XWayland when a Wayland
   session is detected.


Add a part written for GTK2
---------------------------

.. code:: yaml

  desktop-gtk2:
      source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
      source-subdir: gtk
      plugin: make
      make-parameters: ["FLAVOR=gtk2"]
      build-packages:
          - build-essential
          - libgtk2.0-dev
      stage-packages:
          - libxkbcommon0  # XKB_CONFIG_ROOT
          - ttf-ubuntu-font-family
          - dmz-cursor-theme
          - light-themes
          - adwaita-icon-theme
          - gnome-themes-standard
          - shared-mime-info
          - libgtk2.0-0
          - libgdk-pixbuf2.0-0
          - libglib2.0-bin
          - libgtk2.0-bin
          - unity-gtk2-module
          - locales-all
          - libappindicator1
          - xdg-user-dirs
          - ibus-gtk
          - libibus-1.0-5

GTK2 parts don't require a special plugin. Instead, the snap itself requires a
special ``desktop-gtk2`` part which containas the GTK2 runtime libraries, and
the ``desktop-launch`` script that launches the app. This part is copied from
the `Snapcraft Desktop Helpers
<https://github.com/ubuntu/snapcraft-desktop-helpers>`_ repository.

To add the ``desktop-gtk2`` part, copy the part definition provided in this
example.


Add required plugs
------------------

.. code:: yaml

  plugs:
    gtk-2-engines:
        interface: content
        target: $SNAP/lib/gtk-2.0
        default-provider: gtk2-common-themes
    gtk-2-themes:
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

Some GTK2 snaps need a number of desktop environment packages containing
common theming content. These packages are hosted on the Snap Store.

To include these snaps, copy the plug definitions provided in this example.
