.. _explanation-architectures:

Architectures
=============

The ``platforms`` and ``architectures`` keys in a project file
are used to define where snaps are built and where they will execute.

The keys are base-dependent:

* ``platforms`` is used for ``core24`` snaps
* ``architectures`` is used for ``core20`` and ``core22`` snaps


Build-on, build-for, and run-on
-------------------------------

``build-on``, ``build-for``, and ``run-on`` are used within the
``architectures`` and ``platforms`` keys in a project file file.

* ``core24`` uses ``build-on`` and ``build-for``
* ``core22`` uses ``build-on`` and ``build-for``
* ``core20`` uses ``build-on`` and ``run-on``

build-on
^^^^^^^^

The architecture of the machine that the snap is built on (i.e. the machine
where snapcraft runs).

build-for
^^^^^^^^^

The architecture of the machine that the snap is built for (i.e. the machine
where the snap will be installed).

run-on
^^^^^^

This is used for ``core20`` and has the same meaning as ``build-for``. It has
been replaced with the preferred term ``build-for`` in ``core22`` and newer
bases.

.. _build-plans:

Build plans
-----------

A build plan is a list of what snaps snapcraft will build. It can be defined
in the project file and filtered with command-line arguments and
environment variables.

Snapcraft can create multiple snaps, each built for a different architecture.
These project file snippets both select two architectures.

.. code-block:: yaml
    :caption: snapcraft.yaml

    base: core24
    platforms:
      amd64:
      arm64:
        build-on: [amd64, arm64]
        build-for: [arm64]

.. code-block:: yaml
    :caption: snapcraft.yaml

    base: core22
    architectures:
      - build-on: [amd64]
        build-for: [amd64]
      - build-on: [amd64, arm64]
        build-for: [arm64]

If snapcraft executes on an ``amd64`` machine, then it will create the
following build plan:

.. terminal::

  Created build plan:
    build-on: amd64 build-for: amd64
    build-on: amd64 build-for: arm64

Two snap files will be created: ``my-snap_1.0_amd64.snap`` and
``my-snap_1.0_arm64.snap``.

If snapcraft executes on an ``arm64`` machine, then it will create the
following build plan:

.. terminal::

  Created build plan:
    build-on: arm64 build-for: arm64

One snap file will be created: ``my-snap_1.0_arm64.snap``.

core24
^^^^^^

The build plan can be filtered with one of the following methods:

* environment variable ``CRAFT_BUILD_FOR=<arch>``
* environment variable ``SNAPCRAFT_BUILD_FOR=<platform>``
* command-line argument ``--build-for=<arch>``
* command-line argument ``--platform=<platform>``

The command-line argument takes priority over the environment variable. In the
example above, using ``--build-for=arm64`` would cause snapcraft to build one
snap for ``arm64``.

Building with a provider
""""""""""""""""""""""""

When building a snap with LXD or Multipass, each build in the build plan
occurs in its own environment.

Destructive mode
""""""""""""""""

In destructive mode, snapcraft will only build one snap at a time. If multiple
snaps can be built, snapcraft will fail to run. The build plan must be narrowed
down with the ``--build-for`` or ``--platform`` arguments.

core22
^^^^^^

This build plan can be filtered with the environment variable
``SNAPCRAFT_BUILD_FOR=<arch>`` or the command-line argument
``--build-for=<arch>``. The command-line argument takes priority over the
environment variable. In the example above, using ``--build-for=arm64`` would
cause snapcraft to only build one snap for ``arm64``.

Building with a provider
""""""""""""""""""""""""

When building a snap with LXD or Multipass, each build in the build plan occurs
in its own environment.

Destructive mode
""""""""""""""""

In destructive mode, all builds in the build plan occur in the same location.
This can cause unintended consequences, such as parts not being re-built for
each ``architecture``.

To work around this, use ``--build-for`` or ``SNAPCRAFT_BUILD_FOR`` to build
one snap at a time and run ``snapcraft clean --destructive-mode`` when changing
the build-for architecture.

core20
^^^^^^

Build plans are not supported in ``core20`` so building a ``core20`` snap will
only produce one snap.

Snapcraft does not automatically clean the build environment when the
``architecture`` key is changed. Therefore ``snapcraft clean`` should be
run when changing architectures.

Remote build
^^^^^^^^^^^^

Launchpad supports building snaps on multiple architectures or platforms.

If ``architectures`` or ``platforms`` are not defined in the project file,
then Launchpad will build the snap on ``amd64``.

When a snap can be built on multiple architectures, Launchpad can choose which
``build-on`` platform to use. Consider the following equivalent snippets:

.. code-block:: yaml
    :caption: snapcraft.yaml

    base: core24
    platforms:
      ppc64el:
        build-on: [amd64, arm64]
        build-for: [ppc64el]

.. code-block:: yaml
    :caption: snapcraft.yaml

    base: core22
    architectures:
      - build-on: [amd64, arm64]
        build-for: [ppc64el]

Launchpad may build the snap on an ``amd64`` or ``arm64`` platform. This choice
is controlled by Launchpad and cannot be influenced by the user.

Architecture errors
-------------------

.. _build-plan-error:

Could not make build plan
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

  Could not make build plan: build-on architectures in snapcraft.yaml does
  not match host architecture (amd64).

This ``core22`` error has two common causes.

The first cause is that snapcraft is not able to create a build plan because
the there are no ``build-on`` architectures matching the host's architecture.
To resolve this, build the snap on an architecture listed in the
project file or add the host architecture as a ``build-on`` value.

The second cause is due to not enclosing a list of multiple architectures
with brackets. For example:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: amd64, arm64
        build-for: [arm64]

should be changed to:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64, arm64]
        build-for: [arm64]

The brackets are required for lists. This problem is described in
more detail `here <issue 4340_>`_.

Unsupported architectures in remote build
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

  The following architectures are not supported by the remote builder:
  amd64, arm64.
  Please remove them from the architecture list and try again.

This error has two common causes. First, the architecture may not be supported
by launchpad. See :ref:`here <supported-architectures-launchpad>` for a list of
architectures supported by Launchpad.

The second cause is the same :ref:`as above<build-plan-error>` - not enclosing
a list of multiple architectures with brackets.

.. _`issue 4340`: https://github.com/canonical/snapcraft/issues/4340
