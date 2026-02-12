.. _how-to-specify-a-base:

Specify a base
==============

.. include:: /reuse/bases-intro.rst

How to use a base
-----------------

The base a snap will use is defined in the snap's project file.

To use the ``core24`` base for a snap:

.. code-block:: yaml
    :caption: snapcraft.yaml

    base: core24


How to migrate to a newer base
------------------------------

See :ref:`how-to-change-bases` for details on migrating to a newer base.

.. _how-to-deprecated-base:

How to use a deprecated base
----------------------------

The latest release of Snapcraft does not support older bases. Prior major
Snapcraft releases are supported and can be installed from their respective tracks.

See :ref:`base snaps<base-snap-reference>` for a list of which Snapcraft
major releases support a particular base.

The :ref:`reference-support-schedule` details the support timeline of each base.

``core18``
^^^^^^^^^^

To build ``core18`` snaps, install snapcraft 7 from the *7.x* track:

.. code-block:: bash

   snap install snapcraft --channel 7.x

``core``
^^^^^^^^

To build ``core`` snaps, install snapcraft 4 from the *4.x* track:

.. code-block:: bash

   snap install snapcraft --channel 4.x


The base snap mounts itself as the root filesystem within your snap such that
when your application runs, the base's library paths are searched directly
after the paths for your specific snap.

How to develop supported and deprecated bases
---------------------------------------------

When developing snaps using supported and deprecated bases at the same time,
developers must use different versions of Snapcraft. There are a few options:

Refresh channels
^^^^^^^^^^^^^^^^

Switch between Snapcraft releases by refreshing channels. ``snapd`` retains the
previously installed snap, so refreshing between two channels should not
require re-downloading Snapcraft.

.. code-block:: bash

   snap refresh snapcraft --channel 7.x
   snap refresh snapcraft --channel 8.x

Parallel installs
^^^^^^^^^^^^^^^^^

Multiple instances of Snapcraft can be installed via ``snapd``'s experimental
parallel install feature. See the `Parallel installs`_ documentation for
details.

.. code-block:: bash

   snap install snapcraft snapcraft_7 --channel 7.x
   snap install snapcraft snapcraft_8 --channel 8.x
   snapcraft_8 pack

Containers
^^^^^^^^^^

Isolated development environments allow using different versions of Snapcraft
simultaneously.

`Snapcraft rocks`_ are the recommended way to build snaps in a container.


How to bootstrap a base snap
----------------------------

The ``build-base`` key is used to bootstrap and create new bases.

To bootstrap the ``core28`` base snap, use the following project file
snippet:

.. code-block:: yaml
    :caption: snapcraft.yaml

    name: core28
    type: base
    build-base: core26

This snippet will do the following:

* ``name: core28`` sets the snap's name to ``core28``.
* ``type: base`` creates a base snap.
* ``build-base: core26`` builds the snap inside an Ubuntu 26.04 build
  environment.
* ``base`` cannot be set in the project file


.. _kernel-snap-how-to:

How to build a kernel snap
--------------------------

The ``build-base`` key is used to build kernel snaps for Ubuntu LTS
releases.

To build a kernel snap targeting the Ubuntu 22.04 release, use the following
project file snippet:

.. code-block:: yaml
    :caption: snapcraft.yaml

    name: pc-kernel
    type: kernel
    build-base: core22

This snippet will do the following:

* create a kernel snap for Ubuntu 22.04
* build the snap inside an Ubuntu 22.04 build environment
* use the ``core22`` feature set and ``snapcraft.yaml`` schema

How to build a bare base snap
-----------------------------

Bare base snaps are useful for fully statically linked applications and will
not have access to a base snap at runtime.

To build a bare base snap, use the following project file snippet:

.. code-block:: yaml
    :caption: snapcraft.yaml

    name: my-snap
    base: bare
    build-base: core24

This snippet will build a bare base snap inside an Ubuntu 24.04 build
environment.


.. _`Snapcraft and ESM`: https://snapcraft.io/docs/snapcraft-esm
.. _`Snapcraft rocks`: https://github.com/canonical/snapcraft-rocks
.. _`parallel installs`: https://snapcraft.io/docs/parallel-installs
