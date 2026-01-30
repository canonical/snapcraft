.. _reference-anatomy-of-snapcraft-yaml:

Anatomy of snapcraft.yaml
=========================

This page provides a breakdown of the keys in complete Snapcraft project files, as
declared in project files.

Two project files will be examined. The first is a simple project file from a
Python-based snap called `yt-dlp <https://github.com/yt-dlp/yt-dlp>`_. The second is
more complex and builds the `wethr <https://github.com/twobucks/wethr>`_ app, which is
written in Node.

Each key in the yt-dlp project file will be covered, and the more advanced keys in
wethr will be examined.


Simple project file
-------------------

yt-dlp is a command line tool for extracting online videos, and is a
self-contained Python project.

.. collapse:: yt-dlp project file

    .. literalinclude:: code/yt-dlp-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Top-level directives
~~~~~~~~~~~~~~~~~~~~

Several :ref:`top-level keys <reference-snapcraft-yaml-top-level-keys>` define and
describe the snap.


.. _reference-anatomy-of-snapcraft-yaml-metadata:

Metadata
^^^^^^^^

For convenience, the first set of keys comprise the snap's metadata, or
manifest. These declare information about the snap itself, such as the snap's
name, version, summary, and description. These identifying keys allow users to
find the snap through the Snap Store.

The ``name`` key defines the name of the snap. It must start with an ASCII
character and can only use ASCII lowercase letters, numbers, and hyphens. It
must be between 1 and 40 characters in length. It must be unique in the Snap
Store.

The ``version`` key defines the version of the snap to the user. The maximum
length is 32 characters.

The ``summary`` key describes the software in brief. It can't exceed 79
characters.

The ``description`` key describes the software in full. It also accepts
multi-line syntax. It has a 4,096 character limit, and can be split across
multiple lines with `YAML's multi-line syntax
<https://yaml.org/spec/1.2.2/#example-indentation-determines-scope>`_.

When the snap is published, this metadata will be made available to users. It
can also be used to pre-populate certain keys in the snap's Snap Store page
during upload.


Base
^^^^

As part of their security design, snaps can't see the host's root file system
by default. This prevents conflict with other snaps and increases security.
However, apps inside the snap still need some location to act as their root
filesystem. They would also benefit if common libraries were drawn from this
root file system rather than being bundled into each.

The ``base`` key specifies the :doc:`core snap </reference/bases>`, which
provides a minimal set of libraries common to most snaps. It's mounted and used
as the root filesystem for the apps inside the snap. In essence, this means the
snaps behave as though they were running on a system that matches the base.

The core24, core22, and core20 bases are available. Bases correspond to Ubuntu LTS
releases. For example, the library set in core24 is equivalent to a subset found in the
Ubuntu 24.04 LTS general release. For most practical purposes, the use of either core24
and core22 is recommended, depending on the :ref:`plugins <reference-plugins>` the snap
uses.

Confinement
^^^^^^^^^^^

Security confinement distinguishes snaps from software distributed using the
traditional repository methods. `Confinement
<https://snapcraft.io/docs/snap-confinement>`_ allows for a high level of
isolation and security, and prevents snaps from being affected by underlying
system changes, snaps affecting each other, or snaps affecting the host.

The ``confinement`` key describes what type of access the snap's apps will have
once installed on the host. Confinement levels can be treated as filters that
define what type of system resources outside the snap that the app can access.

Confinement is defined by a *level* and fine-tuned using interfaces.

There are three confinement levels:

- ``strict``. This confinement level uses Linux kernel security features to
  lock down the apps inside the snap. By default, a strictly-confined app can't
  access the network, the users' home directory, any audio subsystems or
  webcams, and it can't display any graphical output through X or Wayland.
  Interfaces override its access.

  This is the preferred confinement for most apps.
- ``devmode``. This is a debug mode level used by developers as they iterate on
  the creation of their snap. This allows developers to troubleshoot apps,
  because they may behave differently when confined.

  This confinement is a temporary measure while a snap is being crafted.
- ``classic``. This is the maximally permissive level equivalent to the full
  system access that traditional apps have. Classic confinement is often used
  as a stop-gap measure to enable developers to publish apps that need more
  access than the current set of permissions allow.

  This confinement should be used only when required for functionality, as its
  lack of restrictions is a security risk. Before a snap can be published with
  classic confinement, it must be approved by the Snap Store team according to
  a `candidate review process
  <https://forum.snapcraft.io/t/process-for-reviewing-classic-confinement-snaps/1460>`_.
  Snaps may be rejected if they don't meet the necessary requirements.

Parts
~~~~~

The :ref:`part keys <reference-snapcraft-yaml-part-keys>` define all the pieces of
software that will be used to build the apps inside the snap. It describes how the snap
is going to be built.

The yt-dlp snap only has one part, for the app itself.

The ``plugin`` key instructs the part to use the Python plugin, which will
build the app's Python code. The plugin automatically handles all building and
dependency installation.

The ``source`` key specifies the path to the software source or a download URL
to it. It can be a local or remote path, and can refer to a directory tree, a
compressed archive or a revision control repository. In this particular case,
the app is built the project's upstream GitHub repository.


Apps
~~~~

The :ref:`app keys <reference-snapcraft-yaml-part-keys>` define the command path for
each app, how it will be run, optional parameters, and the interface connections that
will be established at runtime.

The yt-dlp project file declares a single app, which is the main app itself. Other
snaps may have multiple sub-apps or executables.

The ``command`` key defines the path to the executable -- relative to the snap
-- and arguments to use when the app runs.

The ``plugs`` key defines the list of interfaces to which the app will have
access to. This enables the intended app functionality. In this specific case,
the yt-dlp snap will be allowed access to the home, network and removable-media
interfaces, which are not available by default under strict confinement. This
will allow the user of the tool to access files in the user's home directory,
from a network connection, or from any mounted removable media locations.


Advanced project file
---------------------

For a more complex example, there's the project file for wethr, a CLI command for
retrieving local weather conditions.

The metadata, base, and confinement declarations are rather similar to the
simple example, but with some notable differences.

.. collapse:: wethr project file

    .. literalinclude:: code/wethr-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-

.. note::

  The project file shown here has been modified from the actual snap's project file to
  highlight features of Snapcraft.


Adopting metadata
~~~~~~~~~~~~~~~~~

The ``adopt-info`` key instructs Snapcraft to import metadata from another
source. Such use can be useful for continuous integration and delivery systems,
where the declarations in the project file can be obtained from scripts rather than
manually.

There are multiple ways that information can be obtained. For a how-to guide on bringing
in external metadata, see :ref:`configure-package-information-reuse-information`.

Multiple metadata fields can be populated using this key. In this project file, the
snap's version is obtained from the Git repository release tag, which proceeds
in two stages:

#. The ``adopt-info`` key instructs Snapcraft to populate the metadata fields
   that aren't already declared in the project file.
#. In the parts section at the end of the project file:

   #. A step in the build lifecycle is manually overridden.
   #. A custom script is used to derive the version string.
   #. The version string is set using the craftctl scriptlet.

Alternatively, in this particular example, the version field could also be
manually set with ``version: '1.5'``.


Quality grade
~~~~~~~~~~~~~

The ``grade`` key defines the quality level of the snap. Two levels are
available, devel and stable. Snaps with the devel grade can't be uploaded to
either of the stable or candidate channels in the Snap Store.


Architectures
~~~~~~~~~~~~~

The ``architectures`` key defines the target :doc:`platforms
</reference/architectures>` for which the snap should be built on and built
for. It requires the build system that is running the Snapcraft tool to be able
to compile and build the snap for the listed platforms.


Parts
~~~~~

Compared to the project file of yt-dlp, wethr has a part that's notably more
intricate.

It too has one part, but it's built with the :ref:`craft_parts_npm_plugin`, which is
designed to simplify the building of Node and JavaScript-based apps, and contains custom
options for Node.

The ``npm-include-node`` key determines whether to download and include a Node
runtime in the snap, which in turn is specified by the ``npm-node-version``
key.

The ``source`` key like before defines the URL or a path of the app code that
needs to be downloaded for the build. It points to the original wethr project's
source code.

The ``override-pull`` key is an inline Bash script that runs during the pull step of the
:ref:`part lifecycle <explanation-parts-lifecycle>`. It's used to perform operations
that can't be satisfied by the default pull operation in the lifecycle. In the wethr
example, the listed commands are used to derive the correct version of the app, and set
it using the craftctl scriptlet. More details about overrides can be found in
:ref:`explanation-build-overrides`.

The ``build-packages`` key defines the list of tools and libraries required to
successfully build or compile the part. The build packages are obtained from
the repository archives that match the base, and need to be written in the
syntax that can be correctly interpreted by the apt package manager. For
instance, a foo build package from core22 would be installed (``apt install
foo``) in the snap build environment during build. In the case of wethr, the
snap needs Git to retrieve the sources from a remote Git repository and sed
to search and replace the string and yield a Git tag.
