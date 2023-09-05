Architectures
=============

By default, Snapcraft builds a snap to run on the same architecture as the build
environment. This behaviour can be modified with the root keyword
``architectures`` in the snap’s ``snapcraft.yaml``.

The ``architectures`` keyword is used to create a build plan. See
:ref:`build plans<build-plans>` for an explanation on how build plans are
created.

How to create a snap for a specific architecture
------------------------------------------------

To create a snap that will be built on ``amd64`` and built for ``amd64``, use
one of the ``snapcraft.yaml`` snippets below.

core22
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      build-for: amd64

Building on ``amd64`` will produce one snap built for ``amd64``. Snapcraft will
raise an error when building on another architecture.

If ``build-for`` is omitted, then it will assume the value of ``build-on``. The
following snippet snippet will produce the same result:

.. code-block:: yaml

  architectures:
    - build-on: amd64

The shorthand format will also produce the same result:

.. code-block:: yaml

  architectures:
    - amd64

core20
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      run-on: amd64

Building on ``amd64`` will produce one snap built for ``amd64``. Snapcraft will
not raise an error when building on another architecture. Instead, it will
ignore the ``architectures`` keyword and build for the build-on architecture.

If ``run-on`` is omitted, then it will assume the value of ``build-on``. The
following snippet snippet will produce the same result:

.. code-block:: yaml

  architectures:
    - build-on: amd64

The shorthand format will also produce the same result:

.. code-block:: yaml

  architectures:
    - amd64

How to create a set of snaps for multiple architectures
-------------------------------------------------------

core22
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      build-for: amd64
    - build-on: arm64
      build-for: arm64

Building on ``amd64`` will produce one snap for ``amd64``. Building on ``arm64``
will produce one snap for ``arm64``. Snapcraft will raise an error when building
on another architecture.

If ``build-for`` is omitted, then it will assume the value of ``build-on``. The
following snippet snippet will produce the same result:

.. code-block:: yaml

  architectures:
    - build-on: amd64
    - build-on: arm64

The shorthand format will also produce the same result:

.. code-block:: yaml

  architectures:
    - amd64
    - arm64

core20
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      run-on: amd64
    - build-on: arm64
      run-on: arm64

Building on ``amd64`` will produce one snap built for ``amd64``. Building on
``arm64`` will produce one snap built for ``arm64``. Snapcraft will not raise
an error when building on another architecture. Instead, it will ignore the
``architectures`` keyword and build for the build-on architecture.

If ``run-on`` is omitted, then it will assume the value of ``build-on``. The
following snippet snippet will produce the same result:

.. code-block:: yaml

  architectures:
    - build-on: amd64
    - build-on: arm64

The shorthand format will also produce the same result:

.. code-block:: yaml

  architectures:
    - amd64
    - arm64

How to create an architecture independent snap
----------------------------------------------

core22
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      build-for: all

core20
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      run-on: all

How to create a cross-compiling snap
------------------------------------

core22
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      build-for: arm64

Building on ``amd64`` will produce one snap built for ``arm64``. Snapcraft will
raise an error when building on another architecture.

``core22`` can handle complex build plans. For example:

.. code-block:: yaml

  architectures:
    - build-on: amd64
      build-for: amd64
    - build-on: [amd64, arm64]
      build-for: arm64

Building on ``amd64`` will produce two snaps, one built for ``amd64`` and one
built for ``arm64``. Building on ``arm64`` will produce one snap built for
``arm64``. Snapcraft will raise an error when building on another architecture.

core20
^^^^^^

.. code-block:: yaml

  architectures:
    - build-on: amd64
      run-on: arm64

Building on ``amd64`` will produce one snap built for ``arm64``. Snapcraft will
not raise an error when building on another architecture. Instead, it will
ignore the ``architectures`` keyword and build for the build-on architecture.

Complex build plans like the previous ``core22`` example are not supported for
``core20``.

How to stage packages from another architecture
-----------------------------------------------

To use an ``i386`` package for an ``amd64`` snap, use the following
``snapcraft.yaml`` snippets for ``core22``:

.. code-block:: yaml

  architectures:
    - build-on: amd64

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

This is supported for related architectures. ``amd64`` and ``i386`` can stage
packages for each other. ``arm64`` and ``armhf`` can also stage packages for
each other.
