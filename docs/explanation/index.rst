.. _explanation:

Explanation
===========

Explanations provide a wider perspective of Snapcraft. They aid in understanding the
concepts and relationships of Snapcraft as a complete system.


Bases and architectures
-----------------------

Snaps are configured to be built with specific technologies and to run on specific CPU
architectures. This configuration depends on the application's requirements and the
machines that will run it.

- :ref:`explanation-bases`
- :ref:`explanation-architectures`


Parts
-----

Parts are how software is brought into snaps. When a snap is packed, its parts are
processed in a series of ordered, reproducible steps.

- :ref:`explanation-parts`
- :ref:`explanation-parts-lifecycle`

Files are migrated between lifecycle steps by declaring groups of paths, known as
filesets.

:doc:`/common/craft-parts/explanation/filesets`


Snap confinement
----------------

A snap's confinement determines how much access it has to resources in the host system.
Classically confined snaps aren't sandboxed and can access shared libraries from their
base snap.

:ref:`explanation-classic-confinement`

Strict snaps, on the other hand, run in a restricted environment and must be provided
explicit access to external resources.

:ref:`explanation-interfaces`


Cryptography
------------

Snapcraft and its external libraries use cryptographic tools for fetching files,
communicating with local processes, and storing user credentials.

:ref:`Cryptographic technology <explanation-cryptographic-technology>`


.. toctree::
    :titlesonly:
    :maxdepth: 1
    :hidden:

    Cryptographic technology <cryptography>
    architectures
    bases
    parts
    parts-lifecycle
    interfaces
    extensions
    components
    snap-configurations
    remote-build
    /common/craft-parts/explanation/filesets
    classic-confinement
