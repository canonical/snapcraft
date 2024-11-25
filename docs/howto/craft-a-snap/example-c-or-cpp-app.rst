.. _example-c-or-cpp-app:

Example C or C++ app
====================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap based on C or C++. We'll work through the
aspects unique to these apps by examining an existing recipe.

The process of developing a snap for a C or C++ app builds on top of standard
tools like autotools and make, making it possible to adapt or integrate an
app's existing build tooling into the crafting process.


Example recipe for moon-buggy
-----------------------------

The following code comprises the recipe for the C project `moon-buggy
<https://github.com/snapcraft-docs/moon-buggy>`_. This project is a small game
where you drive a buggy across the moon's surface.

.. collapse:: moon-buggy recipe

  .. code:: yaml

    name: moon-buggy
    base: core22
    version: '1.0.51'
    summary: Drive a car across the moon
    description: |
        Moon-buggy is a simple character graphics game, where you drive some
        kind of car across the moon's surface. Unfortunately there are
        dangerous craters there. Fortunately your car can jump over them!
    confinement: devmode

    parts:
        moon-buggy:
            plugin: autotools
            autotools-configure-parameters: ["--prefix=/usr"]
            source: .
            build-packages:
                - libncurses-dev
                - texinfo
    apps:
        moon-buggy:
            command: usr/bin/moon-buggy


Add a part written in C or C++
------------------------------

.. code:: yaml

  parts:
    moon-buggy:
      plugin: autotools
      autotools-configure-parameters: ["--prefix=/usr"]
      source: https://github.com/sergiusens/moon-buggy.git
      build-packages:
        - libncurses-dev
        - texinfo

C and C++ parts are built with the ``autotools``, ``make``, or ``cmake``
plugins.

The recipe uses the `autotools plugin
<https://snapcraft.io/docs/the-autotools-plugin>`_, and a special
``autotools-configure-parameters`` key to override the autotools command's
``--prefix`` argument, which would typically default to ``/usr/local``.

To declare a part written in C or C++:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin`` to ``autotools``, ``make``, or ``cmake``.
