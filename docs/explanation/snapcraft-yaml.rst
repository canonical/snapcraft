.. meta::
    :description: Explanation of snapcraft.yaml, the project file used by Snapcraft that defines the contents and structure of a snap.

:relatedlinks: [YAML&#32;specification](https://yaml.org)


.. _explanation-snapcraft-yaml:

About snapcraft.yaml
====================

Every snap project depends on a file called ``snapcraft.yaml``, which tells Snapcraft
how to build the snap.

This project file is written in YAML, and declares key-value pairs for all the
configuration and contents. The :ref:`snapcraft.yaml reference
<reference-snapcraft-yaml>` provides a complete index of the keys and accepted values in
a project file.

The file can be stored in any of the locations in a project:

- ``<project>/snapcraft.yaml``
- ``<project>/.snapcraft.yaml``
- ``<project>/snap/snapcraft.yaml``
- ``<project>/build-aux/snap/snapcraft.yaml``

The file is usually organized into four sections:

- Top-level keys
- Platform keys
- App keys
- Part keys, which describe how to import and build the apps inside the snap.

This structure isn't mandated, though. As long as the file is valid YAML and each
key-value pair follows its expected signature, the snap will build.

For covering the anatomy of ``snapcraft``, we'll examine the snap for `wethr <https://github.com/twobucks/wethr>`__, which is written in Node.


Top-level keys
--------------

The top-level keys provide information about the snap and describe its basic properties
and information about the software.

.. literalinclude:: code/wethr-snapcraft.yaml
    :caption: wethr project file
    :language: yaml
    :lines: 4-11


.. _reference-anatomy-of-snapcraft-yaml-metadata:
.. _explanation-anatomy-snapcraft-yaml-project-information:

Project information
~~~~~~~~~~~~~~~~~~~

For convenience, the first set of keys comprise information about the snap, such as its
name, version, summary, and description. These identifying keys allow users to find the
snap through the Snap Store. When the snap is published, this information will be made
available to users.

This block also describes the essential characteristics of the snap, such as its
architecture and underlying software set.

The wethr snap keeps the most basic information about the project, but doesn't provide
contact information or a link to a donation page, because the original software didn't.


Base
~~~~

As a security default, snaps can't see the host's resources. This follows the principle
of least privilege and prevents conflict with other snaps and increases. However, apps
inside the snap still need some location in the host's storage to act as their root
filesystem. They would also benefit if common libraries were drawn from this root file
system rather than being bundled into each.

The *base* is the minimal set of libraries common to most snaps. The base is itself a
snap that is common to any snap that uses the base. It's mounted and used as the root
filesystem for the apps inside the snap. In essence, this means the snaps behave as
though they were running on a system that matches the base.

Bases correspond to Ubuntu LTS releases, and are named ``core<version>``. For example,
the library set in core26 is equivalent to a subset found in Ubuntu 26.04 LTS.

The wethr snap uses core26 as its base.


Quality grade
~~~~~~~~~~~~~

The snap's grade defines the quality level of the snap. Two levels are available,
``devel`` and ``stable``. Snaps with the ``devel`` grade can't be uploaded to either of
the stable or candidate channels in the Snap Store.

The wethr snap has been built and tested over a period of years, so it has a stable
grade.


Confinement
~~~~~~~~~~~

Security confinement distinguishes snaps from most traditional Linux software.
:external+snap:ref:`explanation-security-snap-confinement` provides a high level of
isolation and security, and prevents snaps from being affected by underlying system
changes, snaps affecting each other, or snaps affecting the host.

The snap's confinement defines its default access to the host. Confinement is set to one
of three levels, higher levels acting as progressively stricter filters.

``strict`` confinement uses Linux kernel security features to lock down the
apps inside the snap. By default, a strictly-confined snaps can't access the network,
the user's home directory, any audio subsystems or webcams, and it can't display any
graphical output through X or Wayland. With strict confinement, if an app needs access
to a system resource, it must be granted exhaustively. This is the default and recommended level for snaps.

``devmode`` confinement is a debug mode for development and debugging, since the
original software may behave differently when confined. This level is a temporary measure while a snap is being crafted.

``classic`` confinement is the maximally-permissive level, equivalent to the full system
access that traditional software has. It's often used as a stop-gap measure to let
developers publish apps the same level of access as traditional software. This
confinement is a security risk, and should only be used when no other confinement is
possible. Before a snap can be published with classic confinement, it must be approved
by the Snap Store team according to a :external+snap:ref:`candidate review process
<interfaces-reviewing-classic-confinement-snaps>`. Snaps may be rejected if they don't
meet safety and security requirements.

The wethr snap is small and simple, so it's strictly confined. It contains a small app
that only needs internet access to query local weather services, so the app is connected
to the network interface.


Platforms
---------

The snap's platforms define the target platforms and CPU architectures. Platforms are
namespaces for different CPU architectures, and are a topic in themselves covered in the
:ref:`platforms reference <reference-platforms>`.

.. literalinclude:: code/wethr-snapcraft.yaml
    :caption: wethr project file
    :language: yaml
    :lines: 14-16

The wethr snap can build for two different platforms. When built on an AMD64 host, it
will compile for AMD64. On an ARM64 host, it will compile for ARM64.


Apps
----

The app keys describe the execution, interfaces, and resources available to each app in
the snap.

.. literalinclude:: code/wethr-snapcraft.yaml
    :caption: wethr project file
    :language: yaml
    :lines: 19-23

Each entry defines the command path for an app, which is an executable runtime. The
entry also declares how it will be run, optional parameters, and the interface
connections that the app needs.

The wethr snap declares a single app, which is the main binary for the command. Other
snaps may have multiple sub-apps or executables.


Parts
-----

The part keys define all the pieces of software that will be placed inside the snap.

.. literalinclude:: code/wethr-snapcraft.yaml
    :caption: wethr project file
    :language: yaml
    :lines: 26-31

The wethr snap only has one part, which is for the app's binary. The part is in fact the
project's source code. Since the wethr source is hosted on GitHub, Snapcraft will
download it first before building. The source is written in Node, so it must be
assembled with the NPM plugin. The plugin has settings for including a Node runtime in
the snap itself. Most plugins have unique keys like this, that pass special instructions
to the underlying build systems.


Complete project file
---------------------

Here is the whole, working project file covered in this document.

.. dropdown:: wethr project file

    .. literalinclude:: code/wethr-snapcraft.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-
