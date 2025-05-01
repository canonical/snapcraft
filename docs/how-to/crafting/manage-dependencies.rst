.. _how-to-manage-dependencies:

Manage dependencies
===================

When building a snap and constructing a :ref:`part <explanation-parts>`, you commonly
need to specify build and staging dependencies. Build dependencies are required for your
part to successfully build or compile on the development host, while staging
dependencies are required for your snap to run.

For further help on solving build dependencies, see
:ref:`iterate-on-the-build-lifecycle`.


Identify a dependency
---------------------

Package dependencies are listed as package names for the snap's build environment.

Snapcraft's build environment is dependent on the :ref:`base snap <base-snap-reference>`
used in the project file. Consequently, dependencies are listed using their respective
apt package names from the `Ubuntu package archive <https://packages.ubuntu.com/>`_. See
:ref:`reference-build-environment-options` for further details.

It's also feasible to have a build environment built on Fedora for example, using Fedora
packages, or those of your own host environment.

The required packages are likely to be identical to those needed to either build the
project (``build-packages``) or install your project (``stage-packages``) natively.
You'll often find them listed in a project's README, or alongside any build
instructions.

Snap names are identical to the names used to install the snap outside of the build
environment, and those listed by the Snap Store. If a specific channel is required, the
syntax is of the form ``<snap-name>/<channel>`` (see :ref:`reference-channels` for more
details on the syntax).


Define a dependency
-------------------

Build and staging dependencies are added to a snap's :ref:`project file
<reference-snapcraft-yaml>` within a part definition. They can be added as standard
packages for the chosen build environment, such as Debian packages for Ubuntu, or as a
cross-platform snap using the following keys:

.. list-table::
    :header-rows: 1

    * - Key
      - Description
    * - ``build-packages``
      - packages required for the part to build
    * - ``stage-packages``
      - packages required to run the part
    * - ``build-snaps``
      - snaps required for the part to build
    * - ``stage-snaps``
      - snaps required to run the part

The following is a typical example of a part's ``build-`` and ``stage-`` sections for a
snap of a command-line tool that interacts with Git:

.. code-block:: yaml
    :caption: snapcraft.yaml

    build-packages:
      - pkg-config
      - libreadline-dev
      - libncurses5-dev
    build-snaps:
      - go
    stage-snaps:
      - ffmpeg/latest/edge
    stage-packages:
      - git

Staged snaps are downloaded from the `Snap Store <https://snapcraft.io/store>`_ and
unpacked into the snap being built.

For staged snaps, the ``meta`` and ``snap`` directories from the snap will be available
as ``meta.<snap-name>`` and ``snap.<snap-name>`` for cases where assets from those
locations are desired for reuse.


Resolve missing dependencies
----------------------------

Working out your project's dependencies can be an iterative process, much like compiling
a third-party package, with the process split into identifying the dependencies a snap
needs for a successful build, and those required for running the resulting application.


Missing build dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~

The following steps outline a basic process for identifying missing packages in your
snap:

#. Once your snap has a functional project file, run ``snapcraft --debug`` until you
   encounter an error.
#. If the error is a build dependency, use the interactive debug shell to work out
   which package is required.
#. Add the package to your project file and repack the snap to see if you encounter
   the same error.

A typical missing build dependency will generate an error similar to the following:

.. terminal::

    configure: error: can't find the Boehm GC library.  Please install it.
    Failed to run 'override-build': Exit code was 1.

In most cases, the error will provide some indication of what needs to be installed. To
resolve the error from the previous example output, the Boehm GC library (``libgc``)
will need to be installed, and because this is the building stage, so too will its
header package (``libgc-dev``).

If the package is unknown, its correct name can normally be found with a search from
within the build environment, on the `Ubuntu package archive
<https://packages.ubuntu.com/>`_, or searching online.


Missing staging dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After a snap has been successfully built, if Snapcraft detects that the resulting
executable will require further packages, it will attempt to guess these and output a
list that can be copied and pasted into the snap's project file. The output will look
similar to the following:

.. terminal::

    The 'example' part is missing libraries that are not included in the snap or base. They can be satisfied by adding the following entries to the existing stage-packages for this part:
    - libxext6
    - libxft2
    - libxrender1
    - libxss1

To resolve the issue, copy the output list into the staging section of the referenced
part.

Another common problem is that even after snapcraft has successfully built a snap,
running its executable will result in an error. The reasons for these errors are varied,
but the most common is a missing library, as shown in the following example output:

.. terminal::

    /snap/mysnap/current/bin/mybin: error while loading shared libraries: libpaho-mqtt3a.so.1: cannot open shared object file: No such file or directory

The following are the most common solutions for these kinds of errors:

* If a required library might not have been installed by the snap, add the missing
  package to the appropriate part's ``stage-packages`` key.
* If the snap's ``LD_LIBRARY_PATH`` environment variable doesn't include the path to the
  missing library, update ``LD_LIBRARY_PATH`` in the snap's project file. The following
  example adds ``$SNAP/usr/lib``:

.. code-block:: yaml
    :caption: snapcraft.yaml

    apps:
      example-app:
        [...]
        environment:
          LD_LIBRARY_PATH: $LD_LIBRARY_PATH:$SNAP/usr/lib

* If the missing library is shared to the snap by a content interface that isn't
  properly connected, connect the snapd interface. This should be done manually for
  testing and automatically for production.


Override a core dependency
--------------------------

Snapcraft minimizes the size of a target snap by filtering out staging dependencies, if
they are available in the base.

In some cases, it may be desirable to stage a package's dependencies inside the snap,
avoiding the use of the package available in the base snap. For example, if your snap
requires a more recent package to be installed explicitly, add each desired package to
the ``stage-packages`` list. Snapcraft will always stage any package explicitly listed.

To find the list of packages that are available in the base snap, you may find the
manifest at: ``/snap/<base>/current/usr/share/snappy/dpkg.list``
