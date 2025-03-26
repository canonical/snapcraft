.. _example-cross-compiled-app:

Example Cross-Compiled App
==========================

Projects built using the :ref:`craft_parts_autotools_plugin` can be cross-compiled,
allowing software to be developed on one architecture and built for a number of target
architectures.

Overview
--------

There are a number of steps to enable cross-compilation for an autotools project:

#. Specify the architectures used.
#. Adjust the options passed to the ``configure`` script.
#. Include packages to support cross-compilation.
#. Handle linker warnings.

Example project file for example-curl
-------------------------------------

The following code comprises the project file for a project that uses ``libcurl``. The
complete code can be found in `this repository
<https://github.com/snapcraft-docs/hello-curl>`_.

.. collapse:: hello-curl project

  .. literalinclude:: ../code/craft-a-snap/example-cross-compiled-app-recipe.yaml
      :caption: snapcraft.yaml
      :language: yaml

The ``platforms`` key
-------------------------

The :ref:`platforms <architectures-howto>` key is normally used to specify which
architectures should be used when building a snap. For example, in the following case
the build architecture is the same as the target architecture:

.. code:: yaml

    platforms:
      - amd64

It is also possible to specify different architectures for the build process and the
target system. This enables cross-compilation to be managed by Snapcraft.

Enable cross-compilation
------------------------

The ``platforms`` key can also be used to specify sets of architectures using the
``build-on`` and ``build-for`` sets. In the simplest case, this can be used to
cross-compile on one architecture for a single, different architecture:

.. code:: yaml

    platforms:
      - build-on: amd64
        build-for: armhf

In this example, Snapcraft will only build the project on the ``amd64`` architecture,
and it will only build it for the ``armhf`` architecture.

The ``hello-curl`` project uses the ``platforms`` key to list many cross-compilation
targets:

.. literalinclude:: ../code/craft-a-snap/example-cross-compiled-app-recipe.yaml
   :caption: platforms
   :language: yaml
   :start-at: platforms:
   :end-before: lint:

Adjust the autotools configuration
----------------------------------

When building for a particular architecture, Snapcraft will initialise the
``CRAFT_ARCH_TRIPLET_BUILD_FOR`` environment variable in the build environment. This variable
describes the platform and architecture that autotools uses to configure
cross-compilation. For more information on environment variables, see `environment
variables <https://snapcraft.io/docs/parts-environment-variables>`_.

A project that uses the autotools plugin can adjust the options passed to the
``configure`` script for a given part by using the ``autotools-configure-parameters``
key that the plugin provides. In the example project, the ``hello-curl`` part is
configured this way:

.. literalinclude:: ../code/craft-a-snap/example-cross-compiled-app-recipe.yaml
   :caption: autotools-configure-parameters
   :language: yaml
   :start-at: autotools-configure-parameters:

Supply the dependencies
-----------------------

It is necessary to specify which packages will supply the toolchain and libraries
needed for cross-compilation. In the example project, the ``hello-curl`` part also
includes these definitions:

.. literalinclude:: ../code/craft-a-snap/example-cross-compiled-app-recipe.yaml
   :caption: build-packages
   :language: yaml
   :start-at: build-packages:
   :end-before: stage-packages:

`Package repositories <https://snapcraft.io/docs/package-repositories>`_ can also be
specified for the target platform. The example project defines additional repositories
for target platforms using the ``package-repositories`` key and the
``CRAFT_ARCH_BUILD_FOR`` environment variables:

.. literalinclude:: ../code/craft-a-snap/example-cross-compiled-app-recipe.yaml
   :caption: package-repositories
   :language: yaml
   :start-at: package-repositories:
   :end-at: url:

These repositories can then be used with the ``package:repo`` syntax to select the
correct library for the target platform. The example project uses this in the
``stage-packages`` key:

.. literalinclude:: ../code/craft-a-snap/example-cross-compiled-app-recipe.yaml
   :caption: stage-packages
   :language: yaml
   :start-at: stage-packages:
   :end-before: autotools-configure-parameters:

The ``build-packages`` and ``stage-packages`` keys additionally support the `advanced
grammar <https://snapcraft.io/docs/snapcraft-advanced-grammar>`_ keys, which allow
further customisation of packages installed per-platform.

Build the snap
--------------

The snap is built in the usual way, by running ``snapcraft`` in the project directory:

.. terminal::
   :input: snapcraft

Because linters are enabled by default for ``core24`` snaps, this will produce warnings
like the following:

.. code::

    not a dynamic executable
    arm-binfmt-P: Could not open '/lib/ld-linux-armhf.so.3': No such file or directory
    Unable to determine library dependencies for '/root/prime/usr/bin/autotools-cross-compile-example'

This is because snapcraft is unable to resolve the dependencies for the target
architecture using the library linter.

Since the :file:`/lib/ld-linux-armhf.so.3` library will be present in the base of the
target system, these warnings can be suppressed for this case by including a ``lint``
section in the project file:

.. literalinclude:: ../code/craft-a-snap/example-cross-compiled-app-recipe.yaml
   :caption: linters
   :language: yaml
   :start-at: lint:
   :end-at: - library

Rebuilding the snap should result in a build process without warnings and a snap in the
project directory called :file:`hello-curl_1.0_armhf.snap` or similar. The ``_armhf``
component of the file name indicates that the snap has been built for that architecture.
