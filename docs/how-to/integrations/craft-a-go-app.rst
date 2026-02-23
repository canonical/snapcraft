.. _how-to-craft-a-go-app:

Craft a Go app
==============

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Go-based snap. We'll work through the aspects
unique to Go apps by examining an existing project.

The process of developing a snap for a Go app builds on top of the standard Go
compiler and configuration, making it possible to adapt or integrate an app's
existing build tooling into the crafting process.


Example project file for woke
-----------------------------

The following code comprises the project file of a Go tool, `woke
<https://github.com/get-woke/woke>`_. This project is a text analysis tool that detects
exclusive language.

.. dropdown:: woke project file

    .. literalinclude:: ../code/integrations/example-go-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 3-


Add a part written in Go
------------------------

.. literalinclude:: ../code/integrations/example-go-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:


Go parts are built with the :ref:`craft_parts_go_plugin`.

To declare a Go part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: go``.
#. If necessary, you can override the Go compiler version by listing it in the
   ``build-snaps`` key, in the format ``go/<track>/<risk>``. The
   latest version is ``go/latest/stable``.
