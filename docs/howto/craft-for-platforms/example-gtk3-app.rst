.. _example-gtk3-app:

Example GTK3 app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using GTK3 and GNOME. We'll
work through the aspects unique to GTK3-based apps by examining an existing
recipe.


Example foliate recipe
----------------------

The following code comprises the recipe of a GTK3 project, `foliate
<https://github.com/johnfactotum/foliate>`_. This project is an e-book
reader.

.. collapse:: foliate recipe

  .. code:: yaml

    name: foliate
    grade: stable
    adopt-info: foliate
    license: GPL-3.0+

    base: core20
    confinement: strict

    apps:
        foliate:
            command: usr/bin/com.github.johnfactotum.Foliate
            extensions: [gnome-3-38]
            plugs:
                - home
            slots:
                - dbus-daemon
            common-id: com.github.johnfactotum.Foliate

    parts:
        foliate:
            plugin: meson
            source: https://github.com/johnfactotum/foliate.git
            source-branch: 1.x
            meson-parameters: [--prefix=/snap/foliate/current/usr]
            override-pull: |
                snapcraftctl pull
                sed -i -e 's|@GJS@|/usr/bin/gjs|g' src/com.github.johnfactotum.Foliate.in
            build-packages:
                - libgjs-dev
                - gettext
            stage-packages:
                - gjs
            organize:
                snap/foliate/current/usr: usr
            parse-info: [usr/share/metainfo/com.github.johnfactotum.Foliate.appdata.xml]

    slots:
        dbus-daemon:
            interface: dbus
            bus: session
            name: com.github.johnfactotum.Foliate

    layout:
        /usr/bin/gjs:
            symlink: $SNAP/usr/bin/gjs


Add an app that uses GNOME
--------------------------

.. code:: yaml

  apps:
      foliate:
          command: usr/bin/com.github.johnfactotum.Foliate
          extensions: [gnome-3-38]
          plugs:
              - home
          slots:
              - dbus-daemon
          common-id: com.github.johnfactotum.Foliate

Apps that use GTK3 and GNOME as runtime libraries require the `gnome-3-38
extension <https://snapcraft.io/docs/gnome-3-38-extension>`_. The extension
configures the runtime environment of the app so that all desktop functionality
is correctly initialised. As desktop environment apps, they also need special
configuration for AppStream and ``.desktop`` file compatibility.

To add a GTK4 app:

#. Declare the general app keys, such as ``command``,
   ``plugs``, ``after``, and so on.
#. For ``extensions``, add ``gnome-3-38``.
#. Set ``common-id`` to the app's unique AppStream ID. Doing so links the app
   to the ``.desktop`` launcher specified in the AppStream file.
#. If the app requires access to D-Bus, for ``slots``, add ``dbus-daemon``.


Add a part written for GTK3
---------------------------

.. code:: yaml

  parts:
      foliate:
          plugin: meson
          source: https://github.com/johnfactotum/foliate.git
          source-branch: 1.x
          meson-parameters: [--prefix=/snap/foliate/current/usr]
          override-pull: |
              snapcraftctl pull
              sed -i -e 's|@GJS@|/usr/bin/gjs|g' src/com.github.johnfactotum.Foliate.in
          build-packages:
              - libgjs-dev
              - gettext
          stage-packages:
              - gjs
          organize:
              snap/foliate/current/usr: usr
          parse-info: [usr/share/metainfo/com.github.johnfactotum.Foliate.appdata.xml]

GTK3 parts are built with the `Meson plugin
<https://snapcraft.io/docs/meson-plugin>`_.

To add a GTK4 part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: meson``.
#. So that the app has access to its AppStream metadata, for ``parse-info`` add
   a path to the AppStream ``.xml`` file on the host system. Since we set
   ``adopt-info: foliate`` at the start of the recipe, the AppStream
   file of the ``foliate`` part will be used to fill in the
   ``summary``, ``description`` and ``icon`` of this snap and copy the
   AppStream file. See `Using AppStream metadata
   <https://snapcraft.io/docs/using-external-metadata#heading--appstream>`_ for
   technical details about how this works.


Add required slots
------------------

.. code:: yaml

  slots:
      dbus-daemon:
          interface: dbus
          bus: session
          name: com.github.johnfactotum.Foliate

Many GTK3 apps require access to DBus in order to run correctly. If your app
does, you need to explicitly grant it access as a slot:

#. Declare the slot key.
#. Set ``interface: dbus`` and ``bus: session``.
#. Set ``name`` to the app's AppStream ID.
