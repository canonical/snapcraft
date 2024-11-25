.. _example-rust-app:

Example Rust app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Rust-based snap. We'll work through the aspects
unique to Rust apps by examining an existing recipe.

The process of developing a snap for a Rust app builds on top of standard Rust
packaging tools and configuration, making it possible to adapt or integrate an
app's existing build tooling into the crafting process.


Example recipe for XSV
----------------------

The following code comprises the recipe of a Rust project, `liquidctl
<https://github.com/snapcraft-docs/xsv>`_. The app analyses and manipulates CSV
files.

.. collapse:: XSV recipe

  .. code:: yaml

    name: xsv
    version: git
    summary: A fast CSV command line toolkit written in Rust
    description: |
        xsv is a command line program for indexing, slicing, analysing,
        splitting and joining CSV files. Commands should be simple, fast and
        composable.
    base: core18
    confinement: devmode

    parts:
        xsv:
            plugin: rust
            source: .

    apps:
        xsv:
            command: bin/xsv


Add a part written in Rust
--------------------------

.. code:: yaml

  parts:
      xsv:
          plugin: rust
          source: .

Rust parts are built with the `Rust plugin <https://snapcraft.io/docs/rust-plugin>`_.

This recipe bundles the current stable release of Rust in the snap using
Rustup. Dependencies from the project's ``Cargo.toml`` are also bundled.

To declare a Rust part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin`` to ``python``.
#. If the snap uses core18, you can override the Rust toolchain version with
   the ``rust-revision`` list key.

