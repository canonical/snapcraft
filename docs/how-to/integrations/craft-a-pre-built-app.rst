.. _how-to-craft-a-pre-built-app:

Craft a pre-built app
=====================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of a pre-built app, which takes the form of a
tarball, zip file, or Debian package.

Crafting for compiled apps is platform-agnostic because the snap acts as a
wrapper. You could wrap an app compiled with *any* underlying language or
software, provided it was correctly compiled for the intended architecture.

For this reason, wrapping compiled apps is both a highly functional way to
craft snaps as well as an effective fallback for languages and platforms that
don't have an official plugin in Snapcraft.


Example project file for Geekbench
----------------------------------

The following code comprises the project file of a C app, `Geekbench
<https://github.com/snapcraft-docs/geekbench4>`_. This project is a popular PC hardware
benchmarking suite.

.. dropdown:: Geekbench project file

    .. literalinclude:: ../code/integrations/example-pre-built-app.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a compiled part
-------------------

.. literalinclude:: ../code/integrations/example-pre-built-app.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: source: http://cdn.geekbench.com/Geekbench-$SNAPCRAFT_PROJECT_VERSION-Linux.tar.gz

Compiled parts stored in archives are built with the :ref:`craft_parts_dump_plugin`.

To add a compiled part:

#. Declare the general part keys, such as ``override-build``,
   ``build-packages``, and so on.
#. Set ``source`` to a local or remote tarball, zip file, or Debian package.
#. Set ``plugin: dump``.
