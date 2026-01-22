.. _explanation:

Explanation
===========

Explanations provide a wider perspective of Snapcraft. They aid in understanding the
concepts and relationships of Snapcraft as a complete system.


Bases and architectures
-----------------------

Bases and architectures are key concepts in snaps. They ensure that snaps are stable
and compatible across different machines.

- :ref:`explanation-bases`
- :ref:`explanation-architectures`


Parts
-----

Parts are how software is brought into snaps. When a snap is packed, its parts are
processed in a series of ordered, reproducible steps.

- :ref:`explanation-parts`
- :ref:`explanation-parts-lifecycle`

Files travel as bundles through the parts lifecycle. These bundles are called
*filesets*.

- :ref:`filesets_explanation`


Snap confinement
----------------

A snap's confinement determines how much access it has to the host system's resources.

A snap is sandboxed by default, with basic access to the host's file system. If it needs
other resources from the host, access is mediated through special interfaces.

- :ref:`explanation-interfaces`

A classically-confined snap has broad access to the host's resources and runs more like
a traditional app.

- :ref:`explanation-classic-confinement`


Cryptography
------------

Snapcraft and its external libraries use cryptographic tools for fetching files,
communicating with local processes, and storing user credentials.

- :ref:`Cryptographic technology <explanation-cryptographic-technology>`


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
