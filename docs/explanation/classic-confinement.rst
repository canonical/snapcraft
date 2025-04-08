.. _explanation-classic-confinement:

Classic confinement
===================

This page is for software developers who intend or need to craft their snaps
classically, and provides explanations and examples on what happens to classic snaps at
build-time.


Requirements
------------

Snapcraft determines confinement based on the value of the ``confinement`` key in the
project file. For classic confinement, it must be set as ``confinement: classic``. There
are no further requirements for the configuration -- when Snapcraft builds the snap, it
won't be sandboxed. That covers the build and local side of the confinement.

Once the snap is prepared, if the snap author wishes to distribute it through official
channels, they must :ref:`apply for classic confinement
<how-to-enable-classic-confinement-request-snap-store>`.


Build-time process
------------------

Snapcraft builds classic snaps differently from snaps with strict confinement.

This is because in order to execute correctly, packages in classic snaps require
dynamic executables to load shared libraries from the appropriate base snap instead of
using the host's root filesystem.

To prevent incompatibilities, binaries in classic snaps must be built with appropriate
linker parameters, or patched to allow loading shared libraries from their base snap. In
case of potential dynamic linking issues, the snap author must be aware that their
package may not run as expected.

There are multiple ways dynamic linking parameters can be manipulated:

.. image:: https://assets.ubuntu.com/v1/24ce3093-confinement_03.png
    :alt: The flow diagram for dynamic linking parameteres in the snap confinement process at build-time.

- **Runtime library paths**. The dynamic section of an ELF file contains the RPATH
  entry, which lists the runtime paths to shared libraries to be searched before the
  paths set in the LD_LIBRARY_PATH environment variable. Multiple paths separated by a
  colon can be specified.
- ``$ORIGIN`` **path**. The special value ``$ORIGIN`` represents the path where the
  binary is located, thus allowing the runtime library path to be set relative to that
  location. For example, ``$ORIGIN/../lib`` for an executable installed under ``bin/``
  with libraries in ``lib/``.
- **File interpreter**. The special ELF section ``.interp`` holds the path to the
  program interpreter. If used, it must be set to the path of the appropriate dynamic
  linker -- the dynamic linker from the snap package being created If libc is staged, or
  the dynamic linker provided by the base snap otherwise. Usually, the program
  interpreter is provided by the base, but it can also be provided by the snap. This
  happens before any library resolution takes place.

To execute as expected, binaries in a classic snap app must be configured to look for
shared libraries provided by the base snap or bundled as part of the app snap. This is
achieved by setting the runtime path to shared libraries in all ELF binaries (except
relocatable object files) that are present in the package payload.

- The ``$RPATH`` value must be set to reach all needed entries in the dynamic section of
  the ELF binary.
- If the binary already contains an ``$RPATH``, only those that mention ``$ORIGIN`` are
  kept.
- ``$RPATH`` entries that point to locations inside the payload are changed to be relative to ``$ORIGIN``.


Potential conflicts
-------------------

When crafting a classic snap, the snap author must consider the following conflicts that
can arise.

A guide for apply remedies to these conflicts can be found in
:ref:`how-to-debug-classic-confinement`.


At runtime
~~~~~~~~~~

Since there's no isolation between classic snaps and the underlying host system, at
runtime, classic snaps may load dynamic library dependencies in a way that could create
a possible error or conflict, leading to app instability, unknown behavior or crash.

A classic snap created with Snapcraft using one of the Ubuntu bases with dynamically
linked binaries will try to load the required dependencies at runtime.

- It will try to load the dependencies, including stage packages and any other libraries
  inside the snap.
- If not found, it will try to find the dependencies in the base snap under
  ``/snap/<base>``, where base can be something like core20, core22, and so on. The
  libraries will need to match the name and version of libraries as provided by the
  Ubuntu repository archives for the specific base. In other words, snaps built with
  core20 will need to use the relevant libraries (by name or version) the way they are
  defined for Ubuntu 20.04 LTS.
- If not found, it will try to find the dependencies on the host system.
- If found, the libraries will be used.
- The loaded host libraries might not match the expected snap/core version, which could
  result in app instability, unknown behavior, or crashing.


With pre-built binaries
~~~~~~~~~~~~~~~~~~~~~~~

Since there's no isolation between classic snaps and the underlying host system,
special care needs to be taken care of any pre-built binaries with hard-coded library
dependency paths, as they will skip the normal loading order of libraries at runtime.
