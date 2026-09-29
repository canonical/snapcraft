.. _how-to-craft-a-node-app:

Craft a Node app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Node-based snap. We'll work through the aspects
unique to Node apps by examining an existing project.

The process of developing a snap for a Node app builds on top of npm and
``package.json`` manifests, making it possible to adapt or integrate an app's
existing build tooling into the crafting process.


Example project file for wethr
------------------------------

The following code comprises the project file of a Node tool, `wethr
<https://github.com/snapcraft-docs/wethr>`_. This project is a CLI tool for obtaining
local weather information.


.. dropdown:: wethr project file

    .. literalinclude:: ../code/integrations/example-node-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written in Node
--------------------------

.. literalinclude:: ../code/integrations/example-node-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: npm-node-version: 14.16.1

Node parts are built with the :ref:`craft_parts_npm_plugin`.

To declare a Node part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: npm``.
#. Determine how npm is added to the snap:

   .. tab-set::

     .. tab-item:: Snapcraft 8 and higher

       - If you want Snapcraft to download and pack npm, set
         ``npm-include-node: true``, and then set ``npm-node-version`` to
         the required NPM version.
       - If you'd rather manually pack npm into the snap, don't set either of
         these keys. Instead, manually include a copy of npm in the files, and
         declare it in another part.

     .. tab-item:: Snapcraft 7

       Set ``npm-node-version`` to the required NPM version.
