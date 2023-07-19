.. 13486.md

.. _snapcraft-extensions:

Snapcraft extensions
====================

Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap.

These requirements can include build and staging packages, plugs and interfaces, file layouts and environments, and whatever other :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` elements may be required to build a functioning system.

**For a full list of supported extensions, see** :ref:`Supported extensions <supported-extensions>` **.**

A snap developer creating a GTK 3 application snap, for example, can use the ``gnome-3-28`` extension to expose the GTK 3 libraries to a snap at build and runtime without the snap developer needing specific deep knowledge about GTK 3. There are extensions for building robotics (ROS 2) applications too, including the :ref:`ROS2 Humble Extension <the-ros-2-humble-extension>`.

Extensions help:

- avoid repetitive tasks in the snap building process
- obviate the need for in-depth knowledge of the target software stack
- create a standard template for common application requirements
- reduce the testing and security burden, as they’re tested and updated independently

   ⓘ For more details on the snap building process, see :ref:`Creating a snap <creating-a-snap>`.

Using Extensions
----------------

To use an extension, the :ref:`app <snapcraft-app-and-service-metadata-extension>` metadata section of the snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` needs include the ``extensions`` definition. The following snippet, for example, shows how the `Firefox <https://github.com/mozilla/gecko-dev/blob/d36cf98aa85f24ceefd07521b3d16b9edd2abcb7/taskcluster/docker/firefox-snap/firefox.snapcraft.yaml.in#L15>`__ snap uses the ``gnome-3-34`` extension to add Gnome desktop support:

.. code:: bash

   apps:
     firefox:
       command: firefox
       command-chain: [tmpdir]
       desktop: distribution/firefox.desktop
       extensions: [gnome-3-34]
   [...]

All extensions are included with Snapcraft and can be listed with the following command:

.. code:: bash

   $ snapcraft extensions
   Extension name    Supported bases
   ----------------  -----------------
   gnome-3-28        core18
   gnome-3-38        core20
   kde-neon          core18, core20
   [...]

See :ref:`Supported Extensions <supported-extensions>` for the full list of extensions.

Further information about any specific extension can be obtained by typing ``snapcraft extension`` followed by the extension name:

.. code:: shell

   $ snapcraft extension gnome-3-28
   This extension eases creation of snaps that integrate with GNOME 3.28
   ...

Extensions modify the ``snapcraft.yaml`` definition before a build. You can use the ``expand-extensions`` command from your project’s root directory to see how the ``snapcraft.yaml`` file will look with the extensions applied.

.. code:: shell

   $ snapcraft expand-extensions
   name: foliate
   ...
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
   ...
