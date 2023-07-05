.. 8505.md

.. _the-go-plugin:

The go plugin
=============

The Go plugin integrates projects written in `Go <https://golang.org/>`__. This plugin uses the common plugin keywords as well as those for :ref:`sources <snapcraft-parts-metadata-source>`. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-go-plugin-core22_>`__
-  `base: core20 <the-go-plugin-core20_>`__
-  `base: core18 \| core <the-go-plugin-core18_>`__

See :ref:`Go applications <go-applications>` for a simple example, or search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+go%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-go-plugin-core22:

base: core22
~~~~~~~~~~~~

The ``go`` plugin in ``core22`` exclusively requires the use of `go.mod <https://golang.org/ref/mod>`__.

Additionally, the build environment does not include ``go`` by default. To install the latest version, add ``go`` to a :ref:`build-snaps <build-and-staging-dependencies-package>` section for the part:

.. code:: yaml

   build-snaps:
     - go

This plugin uses the following plugin-specific keywords:

-  **go-buildtags** (list of strings) Tags to use during the go build. Default is not to use any build tags.

Requires Snapcraft version *7.0+*.


.. _the-go-plugin-core20:

base: core20
~~~~~~~~~~~~

The ``go`` plugin in ``core20`` exclusively requires the use of `go.mod <https://golang.org/ref/mod>`__.

This plugin uses the following plugin-specific keywords:

-  **go-channel** (string, default: latest/stable) The snap Store channel to install go from. If set to an empty string, go will be installed using the system’s traditional package manager.

-  **go-buildtags** (list of strings) Tags to use during the go build. Default is not to use any build tags.

Requires Snapcraft version *4.0+*.


.. _the-go-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

The ``go`` plugin support in core and core18 can be used by Go projects using `go get <https://golang.org/pkg/cmd/go/internal/get/>`__, the command used to grab a project’s dependencies or ``go mod``.

This plugin uses the following plugin-specific keywords:

-  **go-channel** (string, default: latest/stable) The snap Store channel to install go from. If set to an empty string, go will be installed using the system’s traditional package manager.

-  **go-packages** (list of strings) Go packages to fetch, these must be a “main” package. Dependencies are pulled in automatically by ``go get``. Packages that are not “main” will not cause an error, but would not be useful either. If the package is a part of the go-importpath the local package corresponding to those sources will be used

-  **go-importpath** (string) This entry tells the checked out ``source`` to live within a certain path within ``GOPATH``. This is not needed and does not affect ``go-packages``.

-  **go-buildtags** (list of strings) Tags to use during the go build. Default is not to use any build tags.

Requires Snapcraft version *3.x*.
