.. _how-to-craft-a-c-or-cpp-app:

Craft a C or C++ app
====================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap based on C or C++. We'll work through the
aspects unique to these apps by examining an existing project.

The process of developing a snap for a C or C++ app builds on top of standard
tools like Autotools and Make, making it possible to adapt or integrate an
app's existing build tooling into the crafting process.


Example project file for moon-buggy
-----------------------------------

The following code comprises the project file for the C app `moon-buggy
<https://github.com/snapcraft-docs/moon-buggy>`_. This project is a small game where you
drive a buggy across the moon's surface.

.. dropdown:: moon-buggy project file

    .. literalinclude:: ../code/integrations/example-c-or-cpp-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written in C or C++
------------------------------

.. literalinclude:: ../code/integrations/example-c-or-cpp-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: - texinfo

C and C++ parts are built with the ``autotools``, ``make``, or ``cmake``
plugins.

The project file uses the :ref:`craft_parts_autotools_plugin`, and a special
``autotools-configure-parameters`` key to override the autotools command's ``--prefix``
argument, which would typically default to ``/usr/local``.

To declare a part written in C or C++:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin`` to ``autotools``, ``make``, or ``cmake``.
