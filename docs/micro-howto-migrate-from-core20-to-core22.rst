.. 30188.md

.. _micro-howto-migrate-from-core20-to-core22:

===========================================
(micro) Howto migrate from core20 to core22
===========================================

core22 support in Snapcraft 7.0 introduces new underlying layers and concepts that require some changes when migrating from core20 to core22; support for core20 and other bases is unaffected by Snapcraft 7.0.

Parts
=====

Snapcraft has moved to a more general syntactical mechanism that can be used consistently across the \*craft ecosystem. Most of the following changes are a reflection of that alignment.

While ``snapcraftctl`` is still supported, the new ``craftctl`` is now preferred.

Overriding pull
---------------

.. code:: yaml

   override-pull: |
       snapcraftctl pull

translates to

.. code:: yaml

   override-pull: |
       craftctl default

Overriding build
----------------

.. code:: yaml

   override-build: |
       snapcraftctl build

translates to

.. code:: yaml

   override-build: |
       craftctl default

Overriding stage
----------------

.. code:: yaml

   override-stage: |
       snapcraftctl stage

translates to

.. code:: yaml

   override-stage: |
       craftctl default

Overriding prime
----------------

.. code:: yaml

   override-prime: |
       snapcraftctl prime

translates to

.. code:: yaml

   override-prime: |
       craftctl default

Setting a version
-----------------

.. code:: yaml

   override-<step>: |
       snapcraftctl set-version 1.0.0

translates to

.. code:: yaml

   override-<step>: |
       craftctl set version=1.0.0

Setting a grade
---------------

.. code:: yaml

   override-<step>: |
       snapcraftctl set-grade stable

translates to

.. code:: yaml

   override-<step>: |
       craftctl set grade=stable

Grammar
-------

The try keyword is no longer available, instead of trying use the architecture specific entry for a predictable result such that

.. code:: yaml

   stage-packages:
       - try:
           - criu

now uses the architecture specific on entry

.. code:: yaml

   stage-packages:
       - on amd64:
           - criu

Architectures
-------------

The keywords for architectures are now ``build-on`` and ``build-for``:

::

   architectures:
     - build-on: [amd64]
       run-on: [arm64]

translates to

::

   architectures:
     - build-on: [amd64]
       build-for: [arm64]

Environment variables
---------------------

These environment variables are still supported but should be migrated to the following:

-  ``SNAPCRAFT_PART_SRC_WORK`` → ``CRAFT_PART_SRC_WORK``
-  ``SNAPCRAFT_PART_SRC`` → ``CRAFT_PART_SRC``
-  ``SNAPCRAFT_PROJECT_DIR`` → ``CRAFT_PROJECT_DIR``
-  ``SNAPCRAFT_PART_BUILD`` → ``CRAFT_PART_BUILD``
-  ``SNAPCRAFT_PROJECT_NAME`` → ``CRAFT_PROJECT_NAME``
-  ``SNAPCRAFT_PART_BUILD_WORK`` → ``CRAFT_PART_BUILD_WORK``
-  ``SNAPCRAFT_ARCH_TRIPLET`` → ``CRAFT_ARCH_TRIPLET``
-  ``SNAPCRAFT_PARALLEL_BUILD_COUNT`` → ``CRAFT_PARALLEL_BUILD_COUNT``
-  ``SNAPCRAFT_PRIME`` → ``CRAFT_PRIME``
-  ``SNAPCRAFT_TARGET_ARCH`` → ``CRAFT_TARGET_ARCH``
-  ``SNAPCRAFT_STAGE`` → ``CRAFT_STAGE``
-  ``SNAPCRAFT_PART_NAME`` → ``CRAFT_PART_NAME``
-  ``SNAPCRAFT_PART_INSTALL`` → ``CRAFT_PART_INSTALL``

Getting the grade
-----------------

While ``SNAPCRAFT_PROJECT_GRADE`` is still supported, ``craftctl get grade`` is now preferred.

Getting the version
-------------------

While ``SNAPCRAFT_PROJECT_VERSION`` is still supported, ``craftctl get version`` is now preferred.

Unbound variable verification
-----------------------------

Snapcraft will now report errors in case of unbound variables in user scriptlets and in variables set by the user in ``build-environment``. A typical situation this can happen is if ``LD_LIBRARY_PATH`` is extended and no previous value is set. In this case, the ``:+`` parameter expansion syntax can be used (such as in ``${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}`` to only expand if the variable is set), or just set the new value since there’s no previous value assigned to the variable.

.. code:: yaml

   part:
     user-part:
       ...
       build-environment:
         - LD_LIBRARY_PATH: $CRAFT_STAGE/usr/lib/$CRAFT_ARCH_TRIPLET${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

Application defaults
====================

Snapcraft provided environment
------------------------------

Snapcraft used to setup a snap.yaml that looked like the following:

.. code:: yaml

   apps:
       <user-defined-app>:
           command-chain: [snap/snapcraft-runner.sh]
           command: <user-defined-command>

This was not overridable and to get rid of it, there was a legacy way of dealing with this, which was to define the following in snapcraft.yaml

.. code:: yaml

   apps:
       <user-defined-app>:
           adapter: none
           command: <user-defined-command>

Snapcraft has moved to defining an environment for each application entry instead of setting up a command-chain, with a simple way to override or disable.

Default behavior
~~~~~~~~~~~~~~~~

snapcraft.yaml has no entries in the root environment, then snap.yaml will have

.. code:: yaml

   environment:
       LD_LIBRARY_PATH: <snapcraft-value>
       PATH: <snapcraft-value>

Overriding an entry
~~~~~~~~~~~~~~~~~~~

A user can override one of these by defining it, such that if they define PATH like,

.. code:: yaml

   environment:
       PATH: <user-value>

Then snap.yaml will have

.. code:: yaml

   environment:
       LD_LIBRARY_PATH: <snapcraft-value>
       PATH: <user-value>

Nulling an entry
~~~~~~~~~~~~~~~~

A user can nullify an entry by using a YAML null entry, such that if they define PATH like,

.. code:: yaml

   environment:
       PATH: null

Then snap.yaml will have

.. code:: yaml

   environment:
       LD_LIBRARY_PATH: <snapcraft-value>

Plugins
=======

Most plugins do not install the base dependency by default anymore to allow more control when building.

Go plugin
---------

go is no longer installed by default, to use the snap of go from the latest/stable channel do:

.. code:: yaml

   parts:
       user-part:
           source: .
           plugin: go
           build-snaps: [go/latest/stable]

to install from the deb:

.. code:: yaml

   parts:
       user-part:
           source: .
           plugin: go
           build-packages: [golang-go]

to build go from source:

.. code:: yaml

   parts:
       user-part:
           source: .
           plugin: go
           after: [go-deps]
       go-deps:
           source: ...
           plugin: ...

Rust plugin
-----------

rustc and cargo are no longer installed by default, to install the deb do:

.. code:: yaml

   parts:
       user-part:
           source: .
           plugin: rust
           build-packages: [cargo, rustc]

NPM plugin
----------

node and npm are no longer installed by default, to use the node snap do:

.. code:: yaml

   parts:
       user-part:
           source: .
           plugin: npm
           build-snaps: [node/16/stable]

to include the node binary in the actual build (and provide npm), do

.. code:: yaml

   parts:
       user-part:
           source: .
           plugin: npm
           npm-include-node: true

Meson plugin
------------

meson is no longer installed by default, to use the meson deb do:

.. code:: yaml

   parts:
       user-part:
           source: .
           plugin: meson
           build-packages: [meson, ninja-build]

to build meson from an alternate source:

.. code:: yaml

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
-------------

The following environment variable names should be migrated:

-  ``SNAPCRAFT_PYTHON_INTERPRETER`` → ``PARTS_PYTHON_INTERPRETER``
-  ``SNAPCRAFT_PYTHON_VENV_ARGS`` → ``PARTS_PYTHON_VENV_ARGS``
