.. 35643.md

.. _cross-compile-an-autotools-project:

Cross-compile an autotools project
==================================

Projects built using the :ref:`autotools plugin <the-autotools-plugin>` can be cross-compiled, allowing software to be developed on one architecture and built for a number of target architectures.

This guide shows how to cross-compile a snap built on the ``core22`` base with the autotools plugin. The example project used in this guide can be found in `this repository <https://github.com/snapcraft-docs/autotools-cross-comple-example>`__.

Overview
--------

There are a number of steps to enable cross-compilation for an autotools project:

1. Specify the architectures used.
2. Adjust the options passed to the ``configure`` script.
3. Include packages to support cross-compilation.
4. Handle linker warnings.

It is useful to begin with a reminder of the syntax used to specify architectures in :file:`snapcraft.yaml` files.

The architectures keyword
-------------------------

The :ref:`architectures <architectures>` keyword is normally used to specify which architectures should be used when building a snap. For example, in the following case the build architecture is the same as the target architecture:

.. code:: yaml

   architectures:
     - amd64

It is also possible to specify different architectures for the build process and the target system. This enables cross-compilation to be managed by Snapcraft.

Enable cross-compilation
------------------------

The ``architecture`` keyword can also be used to specify sets of architectures using the ``build-on`` and ``build-for`` sets. In the simplest case, this can be used to cross-compile on one architecture for a single, different architecture:

.. code:: yaml

   architectures:
     - build-on: amd64
       build-for: armhf

In this example, Snapcraft will only build the project on the ``amd64`` architecture, and it will only build it for the ``armhf`` architecture.

Adjust the autotools configuration
----------------------------------

When building for a particular architecture, Snapcraft will initialise the ``SNAPCRAFT_ARCH_TRIPLET`` :ref:`environment variable <environment-variables>` in the build environment. This variable describes the platform and architecture that autotools uses to configure cross-compilation.

A project that uses the autotools plugin can adjust the options passed to the ``configure`` script for a given part by using the ``autotools-configure-parameters`` keyword that the plugin provides. In this case, the ``cross-compilation-part`` is configured in this way:

.. code:: yaml

   parts:
     cross-compilation-part:
       plugin: autotools
       source: .
       autotools-configure-parameters:
         - --prefix=/usr
         - --host=${SNAPCRAFT_ARCH_TRIPLET}

In this case, the parameters that the plugin passes to ``configure`` are overridden to specify the installation prefix and the host architecture to build for. This enables autotools to select the correct toolchain to use for building applications and libraries.

It is also necessary to specify which packages will supply the toolchain and libraries needed for cross-compilation. In this example, the ``cross-compilation-part`` also includes these definitions:

.. code:: yaml

       build-packages:
         - on amd64 to armhf:
           - gcc-arm-linux-gnueabihf
           - libc6-dev-armhf-cross
           - libc6-armhf-cross

:ref:`Advanced grammar <snapcraft-advanced-grammar>` is used in the ``build-packages`` definition to specify the packages containing the cross-compiler, C library and header files when building on ``amd64`` for ``armhf`` platforms.

Build the snap
--------------

The snap is built in the usual way, by running :command:`snapcraft` in the project directory:

.. code:: bash

   $ snapcraft

Because linters are enabled by default for ``core22`` snaps, this will produce warnings like the following:

::

   not a dynamic executable
   arm-binfmt-P: Could not open '/lib/ld-linux-armhf.so.3': No such file or directory
   Unable to determine library dependencies for '/root/prime/usr/bin/autotools-cross-compile-example'

This is because Snapcraft is unable to resolve the dependencies for the target architecture using the library linter.

Since the ``/lib/ld-linux-armhf.so.3`` library will be present in the base of the target system, these warnings can be suppressed for this case by including a ``lint`` section in the project file:

.. code:: yaml

   lint:
     ignore:
       - library

Rebuilding the snap should result in a build process without warnings and a snap in the project directory called ``autotools-cross-compile-example_1.0_armhf.snap`` or similar. The ``_armhf`` component of the file name indicates that the snap has been built for that architecture.
