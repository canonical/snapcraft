.. 4972.md

.. _architectures:

Architectures
=============

By default, :ref:`Snapcraft <snapcraft-overview>` builds a snap to run on the same architecture as the build environment. This behaviour can be modified with an optional root ``architectures`` keyword in the snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`.

Architecture features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below in :ref:`architectures-core22` and
:ref:`architectures-core20-core18`.


.. _architectures-core22:

base: core22
------------

Syntax
------

The ``architectures`` keyword defines a list of ``build-on``->\ ``build-for`` sets with the following syntax:

.. code:: yaml

   architectures:
     - build-on: [<arch 1>, <arch 2>]
       build-for: [<arch 1>]
     - build-on: [<arch 3>]
       build-for: [<arch 4>]
       ...

-  valid architectures in ``core22`` are ``arm64``, ``armhf``, ``amd64``, ``ppc64el``, and ``s390x``
-  ``build-for: all`` can be used for snaps that can run on all architectures (e.g. a snap that is only a shell or python script)
-  only 1 architecture can be defined for each ``build-for`` parameter
-  the environmental variable ``SNAPCRAFT_BUILD_FOR=<arch>`` or the command-line argument ``--build-for=<arch>`` limits snapcraft to building only the snap matching ``build-for: [<arch>]``
-  if the value list is a single item, it can be simplified to a scalar (e.g. ``build-on: amd64``)

Examples
--------

Example 1 - single architecture
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: yaml

   architectures:
     - build-on: amd64
       build-for: amd64

-  building on ``amd64`` will produce 1 snap for ``amd64``
-  building on another architecture will cause Snapcraft to raise an error

Example 2 - single architecture, shorthand
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: yaml

   architectures:
     - amd64

-  this is the same as the previous example, but in a shorthand format

Example 3 - multiple architectures
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: yaml

   architectures:
     - build-on: amd64
       build-for: amd64
     - build-on: arm64
       build-for: arm64

-  building on ``amd64`` will produce 1 snap for ``amd64``
-  building on ``arm64`` will produce 1 snap for ``arm64``
-  building on another architecture will cause Snapcraft to raise an error

Example 4 - multiple architectures, shorthand
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: yaml

   architectures:
     - amd64
     - arm64

-  this is the same as the previous example, but in a shorthand format

Example 5 - multiple architecture, cross-compiling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: yaml

   architectures:
     - build-on: amd64
       build-for: amd64
     - build-on: [amd64, arm64]
       build-for: arm64

-  building on ``amd64`` will produce 2 snaps, 1 snap for ``amd64`` and 1 snap for ``arm64``
-  building on ``arm64`` will produce 1 snap for ``arm64``
-  building on another architecture will cause Snapcraft to raise an error

Example 6 - Architecture independent
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: yaml

   architectures:
     - build-on: amd64
       build-for: all

-  for snaps that can run on any architecture (e.g. python or shell scripts), use ``build-for: all``
-  building on ``amd64`` will produce 1 snap that can run on any architecture


.. _architectures-core20-core18:

base: core20 \| core18 \| core
------------------------------

.. _syntax-1:

Syntax
------

The ``architectures`` keyword defines a list of build and run architecture sets with the following syntax:

.. code:: yaml

   architectures:
     - build-on: [<arch 1>, <arch 2>]
       run-on: [<arch 1>, <arch 2>]
     - build-on: [<arch 3>]
       run-on: [<arch 4>]
       ...

-  valid architectures include ``arm64``, ``armhf``, ``amd64``, ``i386``, ``ppc64el``, and ``s390x``
-  snaps using a base of ``core`` and ``core18`` can additionally support ``i386``
-  support for ``i386`` was removed in ``core20`` (see :ref:`Migrating bases <migrating-between-bases-arch>` for details)
-  the default value for ``run-on`` is the value of ``build-on``
-  ``run-on:`` supports a value of ``all`` to denote a snap that can run everywhere (e.g. a snap that is only shell scripts or python)
-  if the value list is a single item, it can be simplified to a scalar (e.g. ``build-on: amd64``)

.. _examples-1:

Examples
--------

Example 1
~~~~~~~~~

.. code:: yaml

       architectures:
         - build-on: i386
           run-on: [amd64, i386]

Snapcraft’s interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

If running on an i386 host, Snapcraft will build a snap that claims it runs on both amd64 and i386. If running elsewhere, Snapcraft will follow its default behavior, building a snap that runs on the build architecture.

CI systems’ interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

As there is a single non-scalar object in this list, CI systems know to produce only a single snap. Checking the ``build-on`` key, they know that it needs to be built on i386.

Example 2
~~~~~~~~~

.. code:: yaml

       architectures:
         - build-on: amd64
           run-on: all

.. _snapcrafts-interpretation-1:

Snapcraft’s interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

If running on an amd64 host, Snapcraft will build a snap that claims it can run on all architectures. If running elsewhere, Snapcraft will follow its default behavior, building a snap that runs on the build architecture.

.. _ci-systems-interpretation-1:

CI systems’ interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

CI systems can assume that the user only wants the snap built on amd64.

Example 3
~~~~~~~~~

.. code:: yaml

       architectures:
         - build-on: amd64
           run-on: amd64
         - build-on: i386
           run-on: i386

Which is the same as:

.. code:: yaml

       architectures:
         - build-on: amd64
         - build-on: i386

.. _snapcrafts-interpretation-2:

Snapcraft’s interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

As far as Snapcraft is concerned, this is no different from its default behavior.

.. _ci-systems-interpretation-2:

CI systems’ interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

CI systems can assume that the user only wants the snap built on amd64 and i386, and the resulting snaps are to be considered a build set (e.g. if amd64 succeeds but i386 fails, the entire set should be considered to have failed).

Example 4
~~~~~~~~~

.. code:: yaml

       architectures:
         - build-on: amd64
           run-on: amd64
         - build-on: i386
           run-on: i386
         - build-on: armhf
           run-on: armhf
           build-error: ignore

.. _snapcrafts-interpretation-3:

Snapcraft’s interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

Again, as far as Snapcraft is concerned, this is no different from its default behavior.

.. _ci-systems-interpretation-3:

CI systems’ interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

CI systems can assume that the user only wants the snap built on amd64, i386, and armhf. While the resulting snaps are considered a build set, armhf may fail. If it does, release the rest of the build set as normal (i.e. don’t fail the entire build set if armhf fails). If amd64 or i386 fails, however, still consider the entire build set to have failed.

Example 5
~~~~~~~~~

.. code:: yaml

       architectures:
         - build-on: [amd64, i386]
           run-on: all

.. _snapcrafts-interpretation-4:

Snapcraft’s interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

If building on amd64 or i386, Snapcraft will produce a snap that claims it runs on all architectures. If running elsewhere, Snapcraft will follow its default behavior, building a snap that runs on the build architecture.

.. _ci-systems-interpretation-4:

CI systems’ interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

There is only a single non-scalar item in ``architectures``, so CI systems know there is only a single snap to be produced from this, and the resulting snap will claim it runs on all architectures. However, the snap author has specified that either amd64 or i386 could be used to produce this snap, which leaves it up to the CI system to decide which architecture to use. Which one has a smaller build queue?

Example 6
~~~~~~~~~

.. code:: yaml

       architectures: [amd64, i386]

Which is the same as:

.. code:: yaml

       architectures:
         - build-on: [amd64, i386]

Which is the same as:

.. code:: yaml

       architectures:
         - build-on: [amd64, i386]
           run-on: [amd64, i386]

.. _snapcrafts-interpretation-5:

Snapcraft’s interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

If building on amd64 or i386, Snapcraft will produce a snap that claims it runs on both amd64 and i386. If running elsewhere, Snapcraft will follow its default behavior, building a snap that runs on the build architecture.

.. _ci-systems-interpretation-5:

CI systems’ interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

There is only a single non-scalar item in ``architectures``, so CI systems know there is only a single snap to be produced from this, and the resulting snap will claim it runs on both amd64 and i386. However, the snap author has specified that either amd64 or i386 could be used to produce this snap, which leaves it up to the CI system to decide which architecture to use. Which one has a smaller build queue?

Example 7
~~~~~~~~~

.. code:: yaml

       architectures:
         - build-on: amd64
           run-on: all
         - build-on: i386
           run-on: i386

.. _snapcrafts-interpretation-6:

Snapcraft’s interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

Technically Snapcraft could work with this, and treat it similarly to Example 5. However, in this proposal it is an error, mostly to inform the user because of the CI systems’ interpretation of this.

.. _ci-systems-interpretation-6:

CI systems’ interpretation
^^^^^^^^^^^^^^^^^^^^^^^^^^

There are two non-scalar items in ``architectures``, which implies that two snaps will be built. However, one of the snaps to be produced would claim it runs on i386, while the other would claim it runs *everywhere* (including i386). That means they would *both* be released to i386, which is likely not what the developer intended (since the user will only receive the latest). This is an **error case**.

CI systems and build-sets
-------------------------

Continuous Integration (CI) systems, such as `build.snapcraft.io <https://build.snapcraft.io>`__, can use the *architectures* keyword to determine which architectures to build a snap on. With none specified, a snap is built on all architectures.

A build-set is a set of snaps built at the same time from the same snapcraft.yaml, such as from a CI-build triggered by a *git commit*.

Rather than manage build revisions separately, a build-set’s revisions can be managed as a group. Assuming a CI system will fail when a single build fails within a build-set, ``build-error: ignore`` can be used to indicate an *experimental* or *in-progress* architecture that is included in a build-set if its build succeeds but not cause a CI build failure if it fails.

For example, without ``build-error: ignore``, and given a build set of ``[amd64, i386, armhf]``. If the ``armhf`` build fails, the entire build-set is considered to have failed, regardless of whether or not ``amd64`` and ``i386`` builds succeeded.

Even without local access to a specific hardware architecture, ``snapcraft remote-build`` enables anyone to run a multi-architecture snap build process on remote servers using `Launchpad <https://launchpad.net/>`__. See :ref:`Remote build <remote-build>` for more details.
