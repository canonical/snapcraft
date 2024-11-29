.. _example-gtk4-app:

Example GTK4 app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using GTK4 and GNOME. We'll
work through the aspects unique to GTK4-based apps by examining an existing
recipe.


Example gnome-text-editor recipe
--------------------------------

The following code comprises the recipe of a GTK4 project, `gnome-text-editor
<https://gitlab.gnome.org/GNOME/gnome-text-editor>`_. This project is the
default text editor of the GNOME desktop environment.

.. collapse:: gnome-text-editor recipe

  .. code:: yaml

    name: gnome-text-editor
    grade: stable
    adopt-info: gnome-text-editor
    license: GPL-3.0+

    base: core22
    confinement: strict

    apps:
        gnome-text-editor:
            extensions: [gnome]
            command: usr/bin/gnome-text-editor
            desktop: usr/share/applications/org.gnome.TextEditor.desktop
            common-id: org.gnome.TextEditor.desktop
            plugs:
                - gsettings
                - cups

    parts:
        gnome-text-editor:
            source: https://gitlab.gnome.org/GNOME/gnome-text-editor
            source-tag: '42.2'
            source-type: git
            plugin: meson
            meson-parameters:
                - --prefix=/usr
                - --buildtype=release
            parse-info: [usr/share/metainfo/org.gnome.TextEditor.appdata.xml]

    slots:
        gnome-text-editor:
            interface: dbus
            bus: session
            name: org.gnome.TextEditor


Add an app that uses GNOME
--------------------------

.. code:: yaml

  apps:
      gnome-text-editor:
          extensions: [gnome]
          command: usr/bin/gnome-text-editor
          desktop: usr/share/applications/org.gnome.TextEditor.desktop
          common-id: org.gnome.TextEditor.desktop
          plugs:
              - gsettings
              - cups

Apps that use GTK4 and GNOME as runtime libraries require the `gnome extension
<https://snapcraft.io/docs/the-gnome-extension>`_. The extension configures the
runtime environment of the app so that all desktop functionality is correctly
initialised. As desktop environment apps, they also need special configuration
for AppStream and ``.desktop`` file compatibility.

To add a GTK4 app:

#. Declare the general app keys, such as ``command``,
   ``plugs``, ``after``, and so on.
#. For ``extensions``, add ``gnome``.
#. Set ``common-id`` to the app's unique AppStream ID. Doing so links the app
   to the ``.desktop`` launcher specified in the AppStream file.


Add a part written for GTK4
---------------------------

.. code:: yaml

  parts:
      gnome-text-editor:
          source: https://gitlab.gnome.org/GNOME/gnome-text-editor
          source-tag: '42.2'
          source-type: git
          plugin: meson
          meson-parameters:
              - --prefix=/usr
              - --buildtype=release
          parse-info: [usr/share/metainfo/org.gnome.TextEditor.appdata.xml]

GTK4 parts are built with the `Meson plugin
<https://snapcraft.io/docs/meson-plugin>`_.

To add a GTK4 part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: meson``.
#. So that the app has access to its AppStream metadata, for ``parse-info`` add
   a path to the AppStream ``.xml`` file on the host system. Since we set
   ``adopt-info: gnome-text-editor`` at the start of the recipe, the AppStream
   file of the ``gnome-text-editor`` part will be used to fill in the
   ``summary``, ``description`` and ``icon`` of this snap and copy the
   AppStream file. See `Using AppStream metadata
   <https://snapcraft.io/docs/using-external-metadata#heading--appstream>`_ for
   technical details about how this works.


Add required slots
------------------

.. code:: yaml

  slots:
      gnome-text-editor:
          interface: dbus
          bus: session
          name: org.gnome.TextEditor

Many GTK4 apps require access to D-Bus in order to run correctly. If your app
does, you need to explicitly grant it access as a slot:

#. Declare the slot key.
#. Set ``interface: dbus`` and ``bus: session``.
#. Set ``name`` to the app's AppStream ID.

