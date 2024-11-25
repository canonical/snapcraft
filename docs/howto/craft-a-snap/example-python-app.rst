.. _example-python-app:

Example Python app
==================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Python-based snap. We'll work through the aspects
unique to Python apps by examining an existing recipe.

The process of developing a snap for a Python app builds on top of standard
Python packaging tools and configuration, making it possible to adapt or
integrate an app's existing build tooling into the crafting process.

Python projects that follow `PEP 517 <https://peps.python.org/pep-0517/>`_ and
have a ``pyproject.toml`` must be packed with the ``edge`` channel of
Snapcraft.

To switch your existing Snapcraft installation to the ``edge`` channel, run:

.. code:: bash

  sudo snap refresh snapcraft --edge


To install a parallel copy of Snapcraft with the ``edge`` channel, run:

.. code:: bash

  sudo snap install snapcraft snapcraft_edge --edge

To run the copy, use ``snapcraft_edge`` as the command name.


Example recipe for liquidctl
----------------------------

The following code comprises the recipe of a Python project, `liquidctl
<https://snapcraft.io/liquidctl>`_. This project is both a driver and a CLI
tool for power and cooling components in PCs.

.. collapse:: liquidctl recipe

  .. code:: yaml

    name: liquidctl
    summary: a status and control utility to for power, cooling and LED
    components
    version: '1.0'
    description: |
        liquidctl is a command-line tool to monitor and control the fan speed,
        LED colour and pump volumes of specific power supplies, motherboards,
        graphics cards and cooling solutions. The liquidctl snap unofficial and
        is not endorsed by the upsteam project.
    base: core22
    confinement: strict

    parts:
        liquidctl:
            plugin: python
            source: .
            stage-packages:
                - python3-usb

    apps:
        liquidctl:
            command: bin/liquidctl
            plugs:
                - raw-usb
                - hardware-observe


Add a part written in Python
----------------------------

.. code:: yaml

  parts:
      liquidctl:
          plugin: python
          source: .
          stage-packages:
              - python3-usb

Python parts are built with the :ref:`Python plugin <python_plugin>`.

To add a Python part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin`` to ``python``.

