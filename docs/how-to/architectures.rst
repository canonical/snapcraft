Architectures
=============

By default, Snapcraft builds a snap to run on the same architecture as the build
environment. This behaviour can be modified with the top-level keys
``architectures`` and ``platforms`` in the snap's project file.

The ``architectures`` and ``platforms`` keys are used to create a build
plan. See :ref:`build plans<build-plans>` for an explanation on how build
plans are created.

The keys are base-dependent:

* ``platforms`` is used for ``core24`` snaps
* ``architectures`` is used for ``core20`` and ``core22`` snaps

How to create a snap for a specific architecture
------------------------------------------------

To create a snap that will be built on ``amd64`` and built for ``amd64``, use
one of the project file snippets below.

core24
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      amd64:
        build-on: [amd64]
        build-for: [amd64]

Building on ``amd64`` will produce one snap built for ``amd64``.

If the platform name is a valid architecture and ``build-for`` is omitted,
``build-for`` will assume the platform name. The following snippet will produce
the same result as above:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      amd64:
        build-on: [amd64]

If the platform name is a valid architecture, then ``build-on`` and
``build-for`` will assume the platform name. The following snippet will
produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      amd64:

Note that ``build-for`` can be omitted if ``build-on`` is defined but the
converse is not true; ``build-on`` cannot be omitted if ``build-for`` is
defined.

core22
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        build-for: [amd64]

Building on ``amd64`` will produce one snap built for ``amd64``. Snapcraft will
raise an error when building on another architecture.

If ``build-for`` is omitted, it will assume the value of ``build-on``. The
following snippet will produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]

core20
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        run-on: [amd64]

Building on ``amd64`` will produce one snap built for ``amd64``. Snapcraft will
not raise an error when building on another architecture. Instead, it will
ignore the ``architectures`` key and build for the build-on architecture.

If ``run-on`` is omitted, it will assume the value of ``build-on``. The
following snippet will produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]

The shorthand format will also produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - amd64

How to create a set of snaps for multiple architectures
-------------------------------------------------------

core24
^^^^^^

``core24`` snaps accept a single build-for architecture per-platform. To create
a set of snaps for multiple architectures, define a set of platforms:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      amd64:
        build-on: [amd64]
        build-for: [amd64]
      arm64:
        build-on: [arm64]
        build-for: [arm64]

Building on ``amd64`` will produce one snap for ``amd64``. Building on
``arm64`` will produce one snap for ``arm64``. Snapcraft will raise an error
when building on another architecture.

If the platform name is a valid architecture and ``build-for`` is omitted,
``build-for`` will assume the platform name. The following snippet will produce
the same result as above:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      amd64:
        build-on: [amd64]
      arm64:
        build-on: [arm64]

If the platform name is a valid architecture, then ``build-on`` and
``build-for`` will assume the platform name. The following snippet will
produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      amd64:
      arm64:

core22
^^^^^^

``core22`` snaps accept a single ``build-for`` architecture per
``build-on``/``build-for`` pair. To create a set of snaps for multiple
architectures, define a set of ``build-on``/``build-for`` pairs:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        build-for: [amd64]
      - build-on: [arm64]
        build-for: [arm64]

Building on ``amd64`` will produce one snap for ``amd64``. Building on ``arm64``
will produce one snap for ``arm64``. Snapcraft will raise an error when building
on another architecture.

If ``build-for`` is omitted, it will assume the value of ``build-on``. The
following snippet will produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
      - build-on: [arm64]

core20
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        run-on: [amd64]
      - build-on: [arm64]
        run-on: [arm64]

Building on ``amd64`` will produce one snap built for ``amd64``. Building on
``arm64`` will produce one snap built for ``arm64``. Snapcraft will not raise
an error when building on another architecture. Instead, it will ignore the
``architectures`` key and build for the build-on architecture.

If ``run-on`` is omitted, it will assume the value of ``build-on``. The
following snippet will produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
      - build-on: [arm64]

The shorthand format will also produce the same result:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures: [amd64, arm64]

.. _how-to-arch-build-for-all:

How to create an architecture independent snap
----------------------------------------------

``build-for: [all]`` is used for a snap that can run on all architectures, like
a snap that is a shell or python script. It cannot be combined with other
architectures. Click :ref:`here<reference-build-for>` for more information on
the ``all`` key.

core24
^^^^^^

platforms:
  all:
    build-on: [amd64]
    build-for: [all]

core22
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        build-for: [all]

core20
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        run-on: [all]

How to build a snap for a different architecture
------------------------------------------------

core24
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      arm64:
        build-on: [amd64]
        build-for: [arm64]

Building on ``amd64`` will produce one snap built for ``arm64``.

If the platform name is a valid architecture and ``build-for`` is omitted,
``build-for`` will assume the platform name. The following snippet will produce
the same result as above:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      arm64:
        build-on: [amd64]

``core24`` can handle complex build plans. For example:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
        amd64:
            build-on: [amd64]
            build-for: [amd64]
        arm64:
            build-on: [amd64, arm64]
            build-for: [arm64]

Building on ``arm64`` will produce one snap built for ``arm64``.

Building on ``amd64`` will produce two snaps, one built for ``amd64`` and one
built for ``arm64``. This only occurs using remote-build or a build provider.
In destructive mode, Snapcraft can only produce one snap. ``--build-for`` or
``--platform`` must be used to narrow down the build plan to a single snap.
For example, ``snapcraft pack --destructive-mode --platform arm64`` on
``amd64`` will produce one snap built for ``arm64``.

Snapcraft will raise an error when building on another architecture.

core22
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        build-for: [arm64]

Building on ``amd64`` will produce one snap built for ``arm64``. Snapcraft will
raise an error when building on another architecture.

``core22`` can handle complex build plans. For example:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        build-for: [amd64]
      - build-on: [amd64, arm64]
        build-for: [arm64]

Building on ``amd64`` will produce two snaps, one built for ``amd64`` and one
built for ``arm64``. Building on ``arm64`` will produce one snap built for
``arm64``. Snapcraft will raise an error when building on another architecture.

core20
^^^^^^

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        run-on: [arm64]

Building on ``amd64`` will produce one snap built for ``arm64``. Snapcraft will
not raise an error when building on another architecture. Instead, it will
ignore the ``architectures`` key and build for the build-on architecture.

Complex build plans like the previous ``core22`` example are not supported for
``core20``.

How to stage packages from another architecture
-----------------------------------------------

To use an ``i386`` package for an ``amd64`` snap, use the following
project file snippets for ``core22``:

.. code-block:: yaml
    :caption: snapcraft.yaml

    architectures:
      - build-on: [amd64]
        build-for: [amd64]

    package-repositories:
      - type: apt
        formats: [deb]
        architectures: [i386]
        components: [main]
        suites: [jammy]
        key-id: F23C5A6CF475977595C89F51BA6932366A755776
        url: https://ppa.launchpadcontent.net/deadsnakes/ppa/ubuntu

    parts:
      mypart:
        stage-packages:
          - libpython3.11-minimal:i386

This is supported for related architectures. A snap built for ``amd64`` can
stage ``i386`` packages and a snap built for ``i386`` can stage ``amd64``
packages. Similarly, a snap built for ``arm64`` can stage ``armhf`` packages
and a snap built for ``armhf`` can stage ``amd64`` packages.
