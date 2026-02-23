.. _reference-bases:

Bases
=====

.. include:: /reuse/bases-intro.rst

.. _base-snap-reference:

Base snaps
----------

There are six supported base snaps:

.. list-table::
   :header-rows: 1

   * - Name
     - Description
     - Latest snapcraft track with support
   * - `core24`_
     - built from `Ubuntu 24.04 LTS`_
     - 8.x
   * - `core22`_
     - built from `Ubuntu 22.04 LTS`_
     - 8.x
   * - `core20`_
     - built from `Ubuntu 20.04 LTS`_
     - 8.x
   * - `core18`_
     - built from `Ubuntu 18.04 ESM`_
     - 7.x
   * - `core`_
     - built from `Ubuntu 16.04 ESM`_, not to be confused with core16 (see
       below)
     - 4.x
   * - `bare`_
     - an empty base that's useful with fully statically linked snaps and when
       testing
     - 8.x

Older releases of core were occasionally referred to as *core 16*, but ``core``
and ``core16`` are two distinct packages.

.. warning::

   core16 is not a supported base. With no stable release, its beta and
   candidate releases are classed as experimental.

The :ref:`reference-support-schedule` details the support timeline for ESM releases.

``base``
--------

The ``base`` key in a project file:

* defines the feature set used by Snapcraft
* the ``snapcraft.yaml`` schema
* the environment where the snap is built if ``build-base`` is not defined
* and which base snap is used at runtime

``base`` must be defined except for base, snapd, and kernel snaps.

``base`` must be a :ref:`supported base<base-snap-reference>`.

``build-base``
--------------

The ``build-base`` key defines the environment where the snap will be
built.

``build-base`` can only be defined for the following scenarios:

Bare base snaps
^^^^^^^^^^^^^^^

``build-base`` must be a :ref:`supported base<base-snap-reference>` when
``base: bare`` is defined.

Devel builds
^^^^^^^^^^^^

``build-base`` must be ``devel`` when ``base`` is unstable.

*Unstable* means that the base snap has not been released to the ``stable``
channel.

Snaps with a ``devel`` build base must have a ``grade`` of ``devel`` and cannot
be promoted to ``stable`` or ``candidate`` channels.

.. _kernel-snap-reference:

Kernel snaps
^^^^^^^^^^^^

``build-base`` must be a :ref:`supported base<base-snap-reference>` when
``type: kernel`` is defined.

See :ref:`How to build a kernel snap<kernel-snap-how-to>` for details on how to
use ``build-base`` for kernel snaps.

.. _`Snapcraft and ESM`: https://snapcraft.io/docs/snapcraft-esm
.. _`Ubuntu 16.04 ESM`: https://releases.ubuntu.com/16.04/
.. _`Ubuntu 18.04 ESM`: https://releases.ubuntu.com/18.04/
.. _`Ubuntu 20.04 LTS`: https://releases.ubuntu.com/20.04/
.. _`Ubuntu 22.04 LTS`: https://releases.ubuntu.com/22.04/
.. _`Ubuntu 24.04 LTS`: https://releases.ubuntu.com/24.04/
.. _`bare`: https://snapcraft.io/bare
.. _`core18`: https://snapcraft.io/core18
.. _`core20`: https://snapcraft.io/core20
.. _`core22`: https://snapcraft.io/core22
.. _`core24`: https://snapcraft.io/core24
.. _`core`: https://snapcraft.io/core
.. _`supported LTS or interim release`: https://ubuntu.com/about/release-cycle
