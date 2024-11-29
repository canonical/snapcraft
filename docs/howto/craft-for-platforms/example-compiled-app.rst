.. _example-compiled-app:

Example compiled app
====================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of a pre-compiled app in the form of a tarball,
Debian package, or zip file.

Crafting for compiled apps is platform-agnostic because the snap acts as a
wrapper. You could wrap an app compiled with *any* underlying language or
software, provided it was correctly compiled for the intended architecture.

For this reason, wrapping compiled apps is both a highly functional way to
craft snaps as well as an effective fallback for languages and platforms that
don't have an official plugin in Snapcraft.


Example recipe for Geekbench
----------------------------

The following code comprises the recipe of a C project, `Geekbench
<https://github.com/snapcraft-docs/geekbench4>`_. This project is a popular PC hardware benchmarking suite.

.. collapse:: Geekbench recipe

  .. code:: yaml

    name: test-geekbench4
    version: 4.2.0
    summary: Cross-Platform Benchmark
    description: |
      Geekbench 4 measures your system's power and tells
      you whether your computer is ready to roar. How
      strong is your mobile device or desktop computer?
      How will it perform when push comes to crunch?
      These are the questions that Geekbench can answer.
    confinement: devmode
    base: core18

    parts:
      test-geekbench4:
        plugin: dump
        source: http://cdn.geekbench.com/Geekbench-$SNAPCRAFT_PROJECT_VERSION-Linux.tar.gz

    apps:
      test-geekbench4:
        command: geekbench4


Add a compiled part
-------------------

.. code:: yaml

  parts:
    test-geekbench4:
      plugin: dump
      source: http://cdn.geekbench.com/Geekbench-$SNAPCRAFT_PROJECT_VERSION-Linux.tar.gz

Compiled parts stored in archives are built with the `dump plugin
<https://snapcraft.io/docs/dump-plugin>`_.

To add a compiled part:

#. Declare the general part keys, such as ``override-build``,
   ``build-packages``, and so on.
#. Set ``source`` to a local or remote tarball, Debian package, or zip file.
#. Set ``plugin: dump``.
