.. _how-to-craft-a-cross-compiled-app:

Craft a cross-compiled app
==========================

Projects built using the :ref:`craft_parts_autotools_plugin` can be cross-compiled,
allowing software to be developed on one architecture and built for a number of target
architectures.

Example project file for example-curl
-------------------------------------

The following code comprises the project file for a project that uses libcurl. The
complete code can be found in `this repository
<https://github.com/snapcraft-docs/hello-curl>`_.

.. dropdown:: hello-curl project

  .. literalinclude:: ../code/integrations/example-cross-compiled-app-recipe.yaml
      :caption: snapcraft.yaml
      :language: yaml

Enable cross-compilation
------------------------

The :ref:`platforms <how-to-select-architectures>` key specifies which architectures
should be used when building a snap. The ``platforms`` key can also be used to specify
sets of architectures with the ``build-on`` and ``build-for`` keys. In the simplest
case, this can be used to cross-compile on one architecture for a single, different
architecture:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      - build-on: amd64
        build-for: armhf

In this example, Snapcraft will only build the project on the AMD64 architecture,
and it will only build it for the armhf architecture.

In the ``hello-curl`` project, the ``platforms`` key to lists several cross-compilation
targets:

.. literalinclude:: ../code/integrations/example-cross-compiled-app-recipe.yaml
   :caption: snapcraft.yaml of hello-curl
   :language: yaml
   :start-at: platforms:
   :end-before: lint:

Adjust the Autotools configuration
----------------------------------

When building for a particular architecture, Snapcraft will initialize the
``CRAFT_ARCH_TRIPLET_BUILD_FOR`` environment variable in the build environment. This
variable describes the platform and architecture that Autotools uses to configure
cross-compilation. For more information on environment variables, see
:ref:`reference-part-environment-variables`.

A project building with the Autotools plugin can adjust the options passed to the
``configure`` script for a given part with the plugin's
``autotools-configure-parameters`` key.

.. literalinclude:: ../code/integrations/example-cross-compiled-app-recipe.yaml
   :caption: snapcraft.yaml of hello-curl
   :language: yaml
   :start-at: autotools-configure-parameters:

Satisfy the dependencies
------------------------

The ``build-packages`` key specifies which packages will supply the toolchain and
libraries necessary for cross-compilation.

.. literalinclude:: ../code/integrations/example-cross-compiled-app-recipe.yaml
   :caption: snapcraft.yaml of hello-curl
   :language: yaml
   :start-at: build-packages:
   :end-before: stage-packages:

:ref:`reference-package-repositories` can also be specified for the target platform
using the ``package-repositories`` key and the ``CRAFT_ARCH_BUILD_FOR`` environment
variable.

.. literalinclude:: ../code/integrations/example-cross-compiled-app-recipe.yaml
   :caption: snapcraft.yaml of hello-curl
   :language: yaml
   :start-at: package-repositories:
   :end-at: url:

To select the correct library for the target platform, include these packages in the
``stage-packages`` key with the ``package:repo`` syntax.

.. literalinclude:: ../code/integrations/example-cross-compiled-app-recipe.yaml
   :caption: snapcraft.yaml of hello-curl
   :language: yaml
   :start-at: stage-packages:
   :end-before: autotools-configure-parameters:

The ``build-packages`` and ``stage-packages`` keys additionally support the
:ref:`advanced grammar <reference-advanced-grammar>` keys, which allow further
customization of packages installed per-platform.

Build the snap
--------------

To build the snap, navigate to the project directory and run:

.. code-block::

    snapcraft

Because linters are enabled by default for ``core24`` snaps, this will produce warnings
like the following:

.. terminal::
    :input: snapcraft

    not a dynamic executable
    arm-binfmt-P: Could not open '/lib/ld-linux-armhf.so.3': No such file or directory
    Unable to determine library dependencies for '/root/prime/usr/bin/autotools-cross-compile-example'

This is because snapcraft is unable to resolve the dependencies for the target
architecture using the library linter.

Since the :file:`/lib/ld-linux-armhf.so.3` library will be present in the base of the
target system, these warnings can be suppressed for this case by including a ``lint``
section in the project file:

.. literalinclude:: ../code/integrations/example-cross-compiled-app-recipe.yaml
   :caption: snapcraft.yaml of hello-curl
   :language: yaml
   :start-at: lint:
   :end-at: - library

Rebuild the snap, and this time it should finish without warnings. A ``.snap`` file for
each target architecture is deposited at the root of the project directory. Each snap is
distinguished by the architecture name at the end of its name. The
hello-curl project should've created three files, ending in ``_armhf.snap``,
``_arm64.snap``, and ``_riscv.snap``.
