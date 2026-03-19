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
    :alt: The flow diagram for dynamic linking parameters in the snap confinement process at build-time.

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

To execute as expected, binaries in a classic snap must be configured to look for shared
libraries provided by the base or bundled in the snap. This is achieved by setting the
runtime path to shared libraries in all ELF binaries (except relocatable object files)
that are present in the package payload.

`PatchELF <https://snapcraft.io/patchelf>`_ is a recommended companion tool that helps
automatically configure shared libraries in snaps. It ensures that:

- The ``$RPATH`` value is set to reach all needed entries in the dynamic section of the
  ELF binary.
- If the binary already contains an ``$RPATH``, only those that mention ``$ORIGIN`` are
  kept.
- ``$RPATH`` entries that point to locations inside the payload are changed to be relative to ``$ORIGIN``.


Potential conflicts
-------------------

When crafting a classic snap, the snap author must consider the following conflicts that
can arise.

A guide for applying remedies to these conflicts can be found in
:ref:`how-to-debug-classic-confinement`.


At runtime
~~~~~~~~~~

Since there's no isolation between classic snaps and the underlying host system, at
runtime, classic snaps may load dynamic library dependencies in a way that could create
a possible error or conflict, leading to app instability, unknown behavior or crash.

A classic snap created with Snapcraft using one of the Ubuntu bases with dynamically
linked binaries will try to load the required dependencies at runtime:

- It tries to load the dependencies, including stage packages and any other
  libraries, inside the snap.
- (core24) If not found, it looks for the dependencies in the base snap under
  ``/snap/<base>``. The libraries must match the name and version of libraries as
  provided by the Ubuntu package archives for the specific base. In other words, snaps
  built with core24 must use the relevant libraries by name or version, the way they are
  defined for Ubuntu 24.04 LTS.
- (core22) If not found, it looks for the dependencies on the host system. If found
  there, the snap daemon can't guarantee that the dependencies will match the expected
  snap and core version. They might result in app instability, unknown behavior, or
  crashing.


With pre-built binaries
~~~~~~~~~~~~~~~~~~~~~~~

Since there's no isolation between classic snaps and the underlying host system,
special care must be taken for any pre-built binaries with hard-coded library
dependency paths, as they will skip the normal loading order of libraries at runtime.
