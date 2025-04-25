.. _reference-snap-build-process:

Snap build process
==================

This page describes at a high level the workflow between a snap project file and its
resulting artifact.


The project file
----------------

Snaps are created using a build configuration -- called from here on the *project file*
-- defined in a file named ``snapcraft.yaml``. The file is written in a declarative
style in YAML, with keys and values defining the snap's structure and behavior. When
Snapcraft runs, it parses this file and uses the declared information to build the snap.
For developers more familiar with the traditional Linux build systems, the process is
somewhat similar a Makefiles and RPM SPEC files.

The project file is a plaintext file encoded in UTF-8, can be composed manually or
generated from a template. The template contains enough boilerplate keys to
build the snap with little effort.

Designing a snap is an open-ended craft. However, most project files have some
common elements with special names in the Snapcraft world.

* The snap's *metadata* provides identifiers and descriptions by which the snap
  can account for itself and be discovered in the Snap Store.
* *Confinement* describes how confined and secure the snap is.
* The *base* describes which set of libraries the snap will use for its
  functionality. The base also defines the operating system version for the
  snap build instance in the virtual machine (VM) or container during build.
  For instance, ``base: core24`` means that Snapcraft will launch an Ubuntu
  24.04 LTS VM or container, the set of tools and libraries used inside the
  snap will originate from the Ubuntu 24.04 LTS repository archives, and the
  snap apps will run as if on an Ubuntu 24.04 host, regardless of the
  actual system's actual underlying Linux distribution.
* *Parts* describe the software components inside the snap.
* *Apps* describes the apps and their commands that are to run inside
  the snap.

It's important to note several details about snaps and their project files:

* A snap can contain one or more parts.
* A snap can contain one or more app.
* Parts can be pre-assembled binaries or they may be compiled as part of the
  build process.
* Parts use special language- and framework-specific plugins to handle
  different software build systems.
* Parts can make use of build dependencies but that won't be shipped in the
  resulting snap. Common examples of such dependencies are gcc and Make.
* The parts can ship runtime dependencies in the snap for use by the apps, such
  as ``python-bcrypt``.


The build
---------

When Snapcraft is executed, it looks for the project file in the current working
directory, either at the root or in a ``snap`` sub-directory. If the file is
found, Snapcraft will then parse its contents and begin the build.

The command will start an instance of a minimal Ubuntu install, either as VM
with Multipass or container instance with LXD, download the necessary packages
and begin building the snap.


The parts lifecycle
-------------------

During build, Snapcraft loops through several steps, collectively known as the
:ref:`parts lifecycle <explanation-parts-lifecycle>`. Snapcraft inherits much of the
build DNA from other craft tools authored by Canonical.


The result
----------

The result of a successful Snapcraft build is a snap file, which is itself a
compressed Squashfs archive with a ``.snap`` extension.

After the build is complete, the resulting artifact is placed in the current
working directory. Snapcraft then halts the VM or container and preserves it
for reuse in any re-builds of the snap, to reduce processing time.

A snap may contain one or more files that allow the apps to run without
reliance on the host's libraries. A snap will contain one or more apps,
services, configuration files, assets like icons, and other files.

Typically, the content of a snap will resemble a Linux filesystem layout, like
this:

.. terminal::

    drwxr-xr-x 10 igor igor  4096 Jun 10  2020 ./
    drwxrwxrwx 14 igor igor 16384 Oct 17 16:40 ../
    drwxr-xr-x  2 igor igor  4096 Jun 10  2020 bin/
    drwxr-xr-x 10 igor igor  4096 Jun 10  2020 etc/
    -rw-r--r--  1 igor igor    14 Jun 10  2020 flavor-select
    drwxr-xr-x  3 igor igor  4096 Jun 10  2020 lib/
    drwxr-xr-x  2 igor igor  4096 Jun 10  2020 lib64/
    drwxr-xr-x  3 igor igor  4096 Jun 10  2020 meta/
    drwxr-xr-x  3 igor igor  4096 Jun 10  2020 snap/
    drwxr-xr-x  7 igor igor  4096 Jun 10  2020 usr/
    drwxr-xr-x  3 igor igor  4096 Feb 26  2018 var/

The contents of a snap can be examined directly by extracting it as an archive:

.. code:: bash

  unsquashfs <file>.snap
