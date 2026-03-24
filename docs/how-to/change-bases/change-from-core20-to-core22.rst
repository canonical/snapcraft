.. _how-to-change-from-core20-to-core22:

Change from core20 to core22
============================

core22 support in Snapcraft introduces new underlying layers and concepts that
require some changes when migrating from core20 to core22. Support for core20 and other
bases is unaffected.


Parts
-----

Snapcraft has moved to a more general syntactical mechanism that can be used
consistently across the \*craft ecosystem. Most of the following changes are a
reflection of that alignment.

While ``snapcraftctl`` is still supported, ``craftctl`` is now preferred.


Override pull
~~~~~~~~~~~~~

.. code-block:: yaml

    override-pull: |
      snapcraftctl pull

translates to:

.. code-block:: yaml

    override-pull: |
      craftctl default


Override build
~~~~~~~~~~~~~~

.. code-block:: yaml

    override-pull: |
      snapcraftctl build

translates to:

.. code-block:: yaml

    override-pull: |
      craftctl default


Override stage
~~~~~~~~~~~~~~

.. code-block:: yaml

    override-pull: |
      snapcraftctl stage

translates to:

.. code-block:: yaml

    override-pull: |
      craftctl default


Override prime
~~~~~~~~~~~~~~

.. code-block:: yaml

    override-pull: |
      snapcraftctl prime

translates to:

.. code-block:: yaml

    override-pull: |
      craftctl default


Set a version
~~~~~~~~~~~~~

.. code-block:: yaml

    override-<step>: |
      snapcraftctl set-version 1.0.0

translates to:

.. code-block:: yaml

    override-<step>: |
      craftctl set version=1.0.0


Set a grade
~~~~~~~~~~~

.. code-block:: yaml

    override-<step>: |
      snapcraftctl set-grade stable

translates to:

.. code-block:: yaml

    override-<step>: |
      craftctl set grade=stable


Grammar
~~~~~~~

The ``try`` keyword is no longer available. Instead, use the architecture-specific
entry. Consider the following example:

.. code-block:: yaml

    stage-packages:
      - try:
        - criu

This is equivalent to using the architecture-specific entry as follows:

.. code-block:: yaml

    stage-packages:
      - on amd64:
        - criu


Architectures
~~~~~~~~~~~~~

The keywords for architectures are now ``build-on`` and ``build-for``.

.. code-block:: yaml

    architectures:
      - build-on: [amd64]
        run-on: [arm64]

translates to:

.. code-block:: yaml

    architectures:
      - build-on: [amd64]
        build-for: [arm64]


Environment variables
~~~~~~~~~~~~~~~~~~~~~

The following core20 environment variables are still supported, but should be migrated
to core22 as follows:

.. list-table::
    :header-rows: 1

    * - core20
      - core22
    * - ``SNAPCRAFT_PART_SRC_WORK``
      - ``CRAFT_PART_SRC_WORK``
    * - ``SNAPCRAFT_PART_SRC``
      - ``CRAFT_PART_SRC``
    * - ``SNAPCRAFT_PROJECT_DIR``
      - ``CRAFT_PROJECT_DIR``
    * - ``SNAPCRAFT_PART_BUILD``
      - ``CRAFT_PART_BUILD``
    * - ``SNAPCRAFT_PROJECT_NAME``
      - ``CRAFT_PROJECT_NAME``
    * - ``SNAPCRAFT_PART_BUILD_WORK``
      - ``CRAFT_PART_BUILD_WORK``
    * - ``SNAPCRAFT_ARCH_TRIPLET``
      - ``CRAFT_ARCH_TRIPLET``
    * - ``SNAPCRAFT_PARALLEL_BUILD_COUNT``
      - ``CRAFT_PARALLEL_BUILD_COUNT``
    * - ``SNAPCRAFT_PRIME``
      - ``CRAFT_PRIME``
    * - ``SNAPCRAFT_TARGET_ARCH``
      - ``CRAFT_TARGET_ARCH``
    * - ``SNAPCRAFT_STAGE``
      - ``CRAFT_STAGE``
    * - ``SNAPCRAFT_PART_NAME``
      - ``CRAFT_PART_NAME``
    * - ``SNAPCRAFT_PART_INSTALL``
      - ``CRAFT_PART_INSTALL``


Get the grade
~~~~~~~~~~~~~

While ``SNAPCRAFT_PROJECT_GRADE`` is still supported, ``craftctl get grade`` is now
preferred.


Get the version
~~~~~~~~~~~~~~~

While ``SNAPCRAFT_PROJECT_VERSION`` is still supported, ``craftctl get version`` is now
preferred.


Unbound variable verification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft will now report errors for unbound variables in user scriptlets and variables
set by the user in ``build-environment``. This can happen if ``LD_LIBRARY_PATH`` is
extended and no previous value is set. In this case, the ``:+`` parameter expansion
syntax can be used, as shown in the following example. Alternatively, a new value can be
assigned since the variable has not been previously set.

.. code-block:: yaml
    :caption: snapcraft.yaml

    part:
      user-part:
        ...
        build-environment:
          - LD_LIBRARY_PATH: $CRAFT_STAGE/usr/lib/$CRAFT_ARCH_TRIPLET${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}


Filesets
~~~~~~~~

The ``filesets`` keyword is no longer supported in snapcraft project files. Instead,
files and directories to include or exclude should be defined with the ``stage`` and
``prime`` keywords for a part.


Application environments
------------------------

Snapcraft previously set up a snap.yaml file similar to the following:

.. code-block:: yaml
    :caption: snap.yaml

    apps:
      <user-defined-app>:
        command-chain: [snap/snapcraft-runner.sh]
        command: <user-defined-command>

This couldn't be overridden, and to get rid of it, users had to define the following
in their project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    apps:
      <user-defined-app>:
        adapter: none
        command: <user-defined-command>

Snapcraft has moved to defining an environment for each application entry instead
of setting up a command-chain, with a simple way to overwrite or disable.


Default behavior
~~~~~~~~~~~~~~~~

snapcraft.yaml has no entries in the root environment. snap.yaml includes:

.. code-block:: yaml
    :caption: snap.yaml

    environment:
      LD_LIBRARY_PATH: <snapcraft-value>
      PATH: <snapcraft-value>


Override an entry
~~~~~~~~~~~~~~~~~

A user can override the default behavior by defining it themselves. If ``PATH`` is
defined as:

.. code-block:: yaml
    :caption: snapcraft.yaml

    environment:
      PATH: <user-value>

then snap.yaml will include:

.. code-block:: yaml
    :caption: snap.yaml

    environment:
      LD_LIBRARY_PATH: <snapcraft-value>
      PATH: <user-value>


Nullify an entry
~~~~~~~~~~~~~~~~

A user can nullify an entry by using a YAML null entry. If ``PATH`` is defined as:

.. code-block:: yaml
    :caption: snapcraft.yaml

    PATH: null

then snap.yaml will include:

.. code-block:: yaml
    :caption: snap.yaml

    environment:
      LD_LIBRARY_PATH: <snapcraft-value>


Plugins
-------

By default, most plugins don't install the base dependency to allow more control when
building.


Go plugin
~~~~~~~~~

The Go plugin is no longer installed by default. To add the Go snap from the
latest/stable channel, include the following in your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: go
        build-snaps: [go/latest/stable]

To install from the deb, include:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: go
        build-packages: [golang-go]

To build Go from source, include:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: go
        after: [go-deps]
      go-deps:
        source: ...
        plugin: ...


Rust plugin
~~~~~~~~~~~

The ``rustc`` and ``cargo`` plugins are no longer installed by default. To install the
deb, include the following in your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: rust
        build-packages: [cargo, rustc]


NPM plugin
~~~~~~~~~~

The Node and NPM plugins are no longer installed by default. To add the Node
snap, include the following in your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: npm
        build-snaps: [node/16/stable]

To include the Node binary in the build (and provide NPM), include:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: npm
        npm-include-node: true


Meson plugin
~~~~~~~~~~~~

The Meson plugin is no longer installed by default. To add the Meson deb, include the following in your project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: meson
        build-packages: [meson, ninja-build]

To build Meson from an alternate source, include:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      user-part:
        source: .
        plugin: meson
        after: [meson-deps]
      meson-deps:
        plugin: nil
        override-build: |
          pip install meson


Python plugin
~~~~~~~~~~~~~

The following environment variable names should be migrated as follows:

.. list-table::
    :header-rows: 1

    * - core20
      - core22
    * - ``SNAPCRAFT_PYTHON_INTERPRETER``
      - ``PARTS_PYTHON_INTERPRETER``
    * - ``SNAPCRAFT_PYTHON_VENV_ARGS``
      - ``PARTS_PYTHON_VENV_ARGS``


Destructive mode
----------------

``sudo`` is no longer run on behalf of the user. It was a leftover from the
pre-containerization era and, because Snapcraft already runs as root in the managed
environment, is considered redundant with build instance usage.

Additionally, to avoid any interaction during lifecycle processing that could be
blocking, the CLI library in today's UI doesn't support user input. If required, use
``sudo`` in the call to Snapcraft itself.
