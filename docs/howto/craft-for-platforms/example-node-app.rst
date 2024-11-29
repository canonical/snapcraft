.. _example-node-app:

Example Node app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Node-based snap. We'll work through the aspects
unique to Node apps by examining an existing recipe.

The process of developing a snap for a Node app builds on top of npm and
``package.json`` manifests, making it possible to adapt or integrate an app's
existing build tooling into the crafting process.


Example recipe for wether
-------------------------

The following code comprises the recipe of a Node project, `wether
<https://github.com/snapcraft-docs/wethr>`_. This project is a CLI tool for
obtaining local weather information.


.. collapse:: wether recipe

  .. code:: yaml

    name: wethr
    version: git
    summary: Command line weather tool.
    description: |
        Get current weather:-
          $ wethr
        Get current weather in metric units
          $ wethr --metric
        Get current weather in imperial units
          $ wethr --imperial

    confinement: strict
    base: core20

    apps:
        wethr:
            command: bin/wethr

    parts:
        wethr:
            source: .
            plugin: npm
            npm-node-version: 14.16.1


Add a part written in Node
--------------------------

.. code:: yaml

  parts:
      wethr:
          source: .
          plugin: npm
          npm-node-version: 14.16.1

Node parts are built with the `npm plugin
<https://snapcraft.io/docs/npm-plugin>`_.

To declare a Node part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: npm``.
#. Determine how npm is added to the snap:

   .. tabs::

     .. group-tab:: Snapcraft 8 and higher

       - If you want Snapcraft to download and pack npm, set
         ``npm-include-node: true``, and then set ``npm-node-version`` to
         the required NPM version.
       - If you'd rather manually pack npm into the snap, don't set either of
         these keys. Instead, manually include a copy of npm in the files, and
         declare it in another part.

     .. group-tab:: Snapcraft 7

       Set ``npm-node-version`` to the required NPM version.

