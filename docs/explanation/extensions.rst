.. _explanation-extensions:

Extensions
==========

Snapcraft extensions enable snap developers to easily incorporate a set of common
requirements into a snap.

These requirements can include build and staging packages, plugs and interfaces, file
layouts and environments, and whatever other project file elements may be required to
build a functioning system.

A snap developer creating a GTK 3 application snap, for example, can use the
``gnome-3-28`` extension to expose the GTK 3 libraries to a snap at build and runtime
without the snap developer needing specific deep knowledge about GTK 3. There are
extensions for building robotics (ROS 2) applications too, including the :ref:`ROS2
Humble Extension <reference-ros-2-foxy-extension>`.

Extensions help:

* avoid repetitive tasks in the crafting process
* obviate the need for in-depth knowledge of the target software stack
* create a standard template for common application requirements
* reduce the testing and security burden, as they're tested and updated independently

For a full list of supported extensions, see :ref:`reference-extensions`.


Using extensions
----------------

To use an extension, at least one :ref:`app <reference-snapcraft-yaml-app-keys>` in the
snap's project file needs to reference the extension.

The following snippet shows how the Firefox snap uses the ``gnome-3-34`` extension to
add GNOME desktop support:

.. code-block:: yaml
    :caption: snapcraft.yaml of Firefox

    apps:
      firefox:
        command: firefox
        command-chain: [tmpdir]
        desktop: distribution/firefox.desktop
        extensions: [gnome-3-34]
    [...]

All extensions are included with Snapcraft and can be listed with ``snapcraft
extensions``.

.. terminal::
    :user: crafter
    :host: home
    :input: snapcraft extensions

    Extension name          Supported bases
    ----------------------  ------------------------------
    env-injector            core24
    flutter-beta            core18
    flutter-dev             core18
    flutter-master          core18
    flutter-stable          core18
    gnome                   core22, core24
    gnome-3-28              core18
    gnome-3-34              core18
    gnome-3-38              core20
    kde-neon                core18, core20, core22, core24
    [...]

Extensions modify the snap's project file definition before a build. You can run the
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
