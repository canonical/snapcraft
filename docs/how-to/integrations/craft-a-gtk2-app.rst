.. _how-to-craft-a-gtk2-app:

Craft a GTK2 app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using GTK4 and GNOME. We'll
work through the aspects unique to GTK4-based apps by examining an existing
project file.


Example Arduino IDE project file
--------------------------------

The following code comprises the project file of a GTK2 app, the legacy `Arduino IDE
<https://github.com/arduino/Arduino>`_.

.. dropdown:: Arduino IDE project file

    .. literalinclude:: ../code/integrations/example-gtk2-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add an app that uses GNOME
--------------------------

.. literalinclude:: ../code/integrations/example-gtk2-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: apps:
    :end-at: command: arduino-builder

Apps that use GTK2 and GNOME as runtime libraries require a special script. It
brings in the runtime environment and dependencies so that all desktop
functionality is correctly initialized.

To add a GTK2 app:

#. Declare the general app keys, such as ``command``,
   ``plugs``, ``after``, and so on.
#. Set ``command: desktop-launch $SNAP/<command>``.
#. Since GTK2 doesn't support Wayland, it's best to set ``DISABLE_WAYLAND``
   to ``1``. Doing so instructs the app to fall back to XWayland when a Wayland
   session is detected.


Add a part written for GTK2
---------------------------

.. literalinclude:: ../code/integrations/example-gtk2-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :dedent: 2
    :start-at: desktop-gtk2:
    :end-at: - libibus-1.0-5

GTK2 parts don't require a special plugin. Instead, the snap itself requires a
special ``desktop-gtk2`` part which containas the GTK2 runtime libraries, and
the ``desktop-launch`` script that launches the app. This part is copied from
the `Snapcraft Desktop Helpers
<https://github.com/ubuntu/snapcraft-desktop-helpers>`_ repository.

To add the ``desktop-gtk2`` part, copy the part definition provided in this
example.


Add required plugs
------------------

.. literalinclude:: ../code/integrations/example-gtk2-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :lines: 98-

Some GTK2 snaps need a number of desktop environment packages containing
common theming content. These packages are hosted on the Snap Store.

To include these snaps, copy the plug definitions provided in this example.
