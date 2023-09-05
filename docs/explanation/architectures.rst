Architectures
=============

Build-on, build-for, and run-on
-------------------------------

``build-on``, ``build-for``, and ``run-on`` are used inside the root
``architectures`` keyword in a ``snapcraft.yaml``.

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
been replaced with the preferred term ``build-for`` in ``core22``.

.. _build-plans:

Build plans
-----------

A build plan is a list of what snaps snapcraft will build. It can be defined
in the ``snapcraft.yaml`` and filtered with command-line arguments and
environment variables.

core22
^^^^^^

Snapcraft can create multiple snaps, each built for a different architecture.
Consider the following ``snapcraft.yaml`` snippet:

.. code-block:: yaml

  architectures:
    - build-on: amd64
      build-for: amd64
    - build-on: [amd64, arm64]
      build-for: arm64

If snapcraft executes on an ``amd64`` machine, then it will create the
following build plan:

.. code-block:: text

  Created build plan:
    build-on: amd64 build-for: amd64
    build-on: amd64 build-for: arm64

Two snap files will be created: ``my-snap_1.0_amd64.snap`` and
``my-snap_1.0_arm64.snap``.

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
This can cause unintended consequences, such as parts not being re-built. For
more information, see `this issue`_.

To workaround this, use ``--build-for`` or ``SNAPCRAFT_BUILD_FOR`` to build
one snap at a time and run ``snapcraft clean --destructive-mode`` when changing
the build-for architecture.

core20
^^^^^^

Build plans are not supported in ``core20`` so building a ``core20`` snap will
only produce one snap.

Snapcraft does not automatically clean the build environment when the
``architecture`` keyword is changed. Therefore ``snapcraft clean`` should be
run when changing architectures.

Remote build
^^^^^^^^^^^^

Launchpad supports building snaps on multiple architectures.

If architectures are not defined in the ``snapcraft.yaml``, then Launchpad will
build the snap on all architectures
:ref:`supported by Launchpad<supported-architectures-launchpad>`.

If architectures are defined in the ``snapcraft.yaml``, then Launchpad will
build the snap on all ``build-on`` architectures.

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
the there are no build-on architectures matching the host's architecture.
To resolve this, build the snap on an architecture listed in the
``snapcraft.yaml`` or add the host architecture as a ``build-on`` value.

The second cause is due to not enclosing a list of multiple architectures
with brackets. For example:

.. code-block:: yaml

  architectures:
    - build-on: amd64, arm64
      build-for: arm64

should be changed to:

.. code-block:: yaml

  architectures:
    - build-on: [amd64, arm64]
      build-for: arm64

The brackets are required for lists. This problem is described in
more detail `here`_.

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

.. _`here`: https://github.com/snapcore/snapcraft/issues/4340
.. _`this issue`: https://github.com/snapcore/snapcraft/issues/4356
