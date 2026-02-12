.. _how-to-craft-a-gtk4-app:

Craft a GTK4 app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using GTK4 and GNOME. We'll
work through the aspects unique to GTK4-based apps by examining an existing
project file.


Example gnome-text-editor project file
--------------------------------------

The following code comprises the project file of a GTK4 app, `gnome-text-editor
<https://gitlab.gnome.org/GNOME/gnome-text-editor>`_. This project is the default text
editor of the GNOME desktop environment.

.. dropdown:: gnome-text-editor project file

    .. literalinclude:: ../code/integrations/example-gtk4-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add an app that uses GNOME
--------------------------

.. literalinclude:: ../code/integrations/example-gtk4-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: apps:
    :end-at: - cups

Apps that use GTK4 and GNOME as runtime libraries require the
:ref:`reference-gnome-extension`. The extension configures the runtime environment of
the app so that all desktop functionality is correctly initialized. As desktop
environment apps, they also need special configuration for AppStream and ``.desktop``
file compatibility.

To add a GTK4 app:

#. Declare the general app keys, such as ``command``,
   ``plugs``, ``after``, and so on.
#. For ``extensions``, add ``gnome``.
#. Set ``common-id`` to the app's unique AppStream ID. Doing so links the app
   to the ``.desktop`` launcher specified in the AppStream file.


Add a part written for GTK4
---------------------------

.. literalinclude:: ../code/integrations/example-gtk4-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: parse-info: [usr/share/metainfo/org.gnome.TextEditor.appdata.xml]

GTK4 parts are built with the :ref:`craft_parts_meson_plugin`.

To add a GTK4 part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: meson``.
#. So that the app has access to its AppStream metadata, for ``parse-info`` add a path
   to the AppStream ``.xml`` file on the host system. Since we set ``adopt-info:
   gnome-text-editor`` at the start of the project file, the AppStream file of the
   ``gnome-text-editor`` part will be used to fill in the ``summary``, ``description``
   and ``icon`` of this snap and copy the AppStream file. See
   :ref:`reference-external-package-appstream` for technical details about how this
   works.


Add required slots
------------------

.. literalinclude:: ../code/integrations/example-gtk4-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: slots:
    :end-at: name: org.gnome.TextEditor

Many GTK4 apps require access to D-Bus in order to run correctly. If your app
does, you need to explicitly grant it access as a slot:

#. Declare the slot key.
#. Set ``interface: dbus`` and ``bus: session``.
#. Set ``name`` to the app's AppStream ID.
