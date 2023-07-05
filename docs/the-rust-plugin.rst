.. 8588.md

.. _the-rust-plugin:

The rust plugin
===============

The ``rust`` plugin is useful for building `Rust <https://www.rust-lang.org/>`__-based parts using the `Cargo <https://crates.io/>`__ package manager to drive the build. This plugin uses the common plugin keywords as well as those for :ref:`sources <snapcraft-parts-metadata-source>`. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Additional features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-rust-plugin-core22_>`__
-  `base: core20 <the-rust-plugin-core20_>`__
-  `base: core18 \| core <the-rust-plugin-core18_>`__

See :ref:`Rust applications <rust-applications>` for a simple example, or search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+rust%22&type=Code>`__ for projects using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-rust-plugin-core22:

base: core22
~~~~~~~~~~~~

-  **rust-features** (list of strings) Features used to build optional dependencies
-  **rust-path** (list of strings) Defaults to the current working directory. Can be set to the relative path for the crate to build when using workspaces. Only one item is currently supported.

Requires Snapcraft version *7.0+*.


.. _the-rust-plugin-core20:

base: core20
~~~~~~~~~~~~

-  **rust-features** (list of strings) Features used to build optional dependencies
-  **rust-path** (list of strings) Defaults to the current working directory. Can be set to the relative path for the crate to build when using workspaces. Only one item is currently supported.

Requires Snapcraft version *4.0+*.


.. _the-rust-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

-  **rust-channel** (string) Used to select which *rust* channel (stable, beta, nightly)
-  **rust-features** (list of strings) Features used to build optional dependencies
-  **rust-revision** (string) Used to select which *rust* version

If a `rust-toolchain <https://rust-lang.github.io/rustup/overrides.html#the-toolchain-file>`__ file is detected, the toolchain it specifies will be used by default. However, if ``rust-channel`` or ``rust-revision`` are set, the rust-toolchain file will be overridden.

If neither a rust-toolchain exists nor ``rust-channel`` or ``rust-revision`` are set, the latest stable toolchain will be used.

Requires Snapcraft version *3.x+*.
