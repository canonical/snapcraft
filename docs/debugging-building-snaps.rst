.. 6274.md

.. _debugging-building-snaps:

Debugging building snaps
========================

Building snaps can be a new experience for many developers. Here’s some tips which can help you successfully, reliably build snaps for publication in the Snap Store. This is a wiki post so feel free to add more tips to this page, to help build successful snaps!


.. _debugging-building-snaps-build-environment:

Build Environment
-----------------

Snaps are built to run on top of a :ref:`base snap <base-snaps>` runtime. This base is provided by an automatically-installed snap. Currently the most widely used core image is based on Ubuntu 22.04 LTS.


.. _debugging-building-snaps-missing-libraries:

Missing libraries
-----------------

Most applications will need additional libraries added to the snap in order to function correctly. As the developer of the application, you’re best placed to know which libraries you need to stage in the snap.

Sometimes when a snap is initially built, libraries are missing because they were not explicitly specified by the developer. There’s a couple of ways to bundle required libraries in a snap, both of which are covered below, but more details can be found in :ref:`Build and staging dependencies <build-and-staging-dependencies>`.


.. _debugging-building-snaps-staging-packages:

Staging Packages
~~~~~~~~~~~~~~~~

It’s common to bundle required libraries in snaps using ``stage-packages`` in the :file:`snapcraft.yaml` file. These are standard package names from the Ubuntu repository used by the base snap. For those unfamiliar with the naming of packages in the Ubuntu archive, the package search at https://packages.ubuntu.com/ can be an invaluable tool. It enables search for files (such as libraries) within packages in the archive. Just be sure to choose the Ubuntu version used by your base snap when searching.

Don’t include ``glibc``/``libc6`` in your list of staged packages. Doing so is unnecessary as the base snap contains those libraries already, and bundling them into your snap can cause unexpected behaviour.


.. _debugging-building-snaps-staging-individual-libraries:

Staging individual libraries
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some application developers already have ‘vendored’ libraries which they’ve well tested with their application. If that’s the case, those libraries can be bundled in the snap in a similar fashion. Place the libraries in the ``/lib`` folder when constructing the snap. This folder is added to the ``LD_LIBRARY_PATH`` and as such should be found successfully by your application when the resulting snap is installed on an end-user computer


.. _debugging-building-snaps-interfaces:

Interfaces
----------

When applications are confined in a snap, they have a restricted view of the world, with access to resources governed by standard Linux security features apparmor and seccomp. Interfaces enable the developer to choose specify what access is required by the application to resources such as the network, camera, joystick and X11 display.


.. _debugging-building-snaps-identifying-missing-interfaces:

Identifying missing interfaces
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The full list of :ref:`interfaces <supported-interfaces>` details the capabilities enabled by each. Developers should consult this list to identify the necessary interfaces required by their application. When an interface is omitted, this may result in the application misbehaving.

The Snap security team have provided a tool to debug these situations. Install the tool with ``snap install snappy-debug``. This helps identify missing interfaces by reporting on application security failures, and will make suggestions on how to improve the snap, perhaps by adding interfaces.

1. In a terminal, run ``snappy-debug``
2. Launch the snapped application (in another terminal)
3. Operate the snapped application until failure occurs
4. Examine the output from ``snappy-debug``

Typically the output will report on failed attempts to access system resources, and suggest additional interfaces which should be specified. If so, add the interface(s) listed and rebuild the snap.


.. _debugging-building-snaps-iterating-without-rebuilding:

Iterating without rebuilding
----------------------------

It can be time consuming to iterate over a snap via tweaking the :file:`snapcraft.yaml` file or application itself, then rebuild and re-install.

To speed a build up, see :ref:`Iterating over a build <iterating-over-a-build>` for build and testing best-practices.
