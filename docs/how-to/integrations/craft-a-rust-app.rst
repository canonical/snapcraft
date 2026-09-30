.. _how-to-craft-a-rust-app:

Craft a Rust app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Rust-based snap. We'll work through the aspects
unique to Rust apps by examining an existing project.

The process of developing a snap for a Rust app builds on top of standard Rust
packaging tools and configuration, making it possible to adapt or integrate an
app's existing build tooling into the crafting process.


Example project file for XSV
----------------------------

The following code comprises the project file of a Rust app, `XSV
<https://github.com/snapcraft-docs/xsv>`_. It analyses and manipulates CSV files.

.. dropdown:: XSV project file

    .. literalinclude:: ../code/integrations/example-rust-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written in Rust
--------------------------

.. literalinclude:: ../code/integrations/example-rust-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: source: .

Rust parts are built with the :ref:`craft_parts_rust_plugin`.

This project file bundles the current stable release of Rust in the snap using
Rustup. Dependencies from the project's ``Cargo.toml`` are also bundled.

To declare a Rust part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: rust``.
#. If the snap uses core18, you can override the Rust toolchain version with
   the ``rust-revision`` list key.
