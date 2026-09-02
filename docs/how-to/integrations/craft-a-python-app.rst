.. _how-to-craft-a-python-app:

Craft a Python app
==================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Python-based snap. We'll work through the aspects
unique to Python apps by examining an existing project.

The process of developing a snap for a Python app builds on top of standard
Python packaging tools and configuration, making it possible to adapt or
integrate an app's existing build tooling into the crafting process.


Example project file for liquidctl
----------------------------------

The following code comprises the project file of a Python app, `liquidctl
<https://snapcraft.io/liquidctl>`_. This project is both a driver and a CLI tool for
power and cooling components in PCs.

.. dropdown:: liquidctl project file

    .. literalinclude:: ../code/integrations/example-python-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written in Python
----------------------------

.. literalinclude:: ../code/integrations/example-python-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: - python3-usb

Python parts are built with the :ref:`Python plugin <python_plugin>`.

To add a Python part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: python``.
