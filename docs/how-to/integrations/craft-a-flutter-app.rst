.. _how-to-craft-a-flutter-app:

Craft a Flutter app
===================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using GTK4 and GNOME. We'll
work through the aspects unique to GTK4-based apps by examining an existing
project file.


Example Flutter app project file
--------------------------------

The following code comprises the project file of a simple Flutter app, `my-flutter-app
<https://github.com/snapcraft-docs/my-flutter-app>`_. This project is merely a
demonstration of a clicker window for GNOME.

.. dropdown:: Flutter app project file

    .. literalinclude:: ../code/integrations/example-flutter-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add an app written for Flutter
------------------------------

.. literalinclude:: ../code/integrations/example-flutter-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: flutter-target: lib/main.dart

.. SNAPCRAFT-1123: Missing link to Flutter plugin.

Flutter parts are built with the Flutter plugin.

To add a Flutter part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: flutter``.
#. If the part is the main app, set ``flutter-target`` to the location of the
   project's ``main.dart`` file.
