.. 14400.md

.. _remote-build:

Remote build
============

There are two methods for building snaps on Canonical-hosted servers, and both are available to every snap publisher:

-  **Build from GitHub** This is a build service integrated into every publisher’s :ref:`Developer Account <create-a-developer-account>` on `snapcraft.io <https://snapcraft.io/>`__. It works by linking a snap’s GitHub repository with our Launchpad build service. See :ref:`Build from GitHub <build-from-github>` for further details.

-  **Snapcraft remote-build** The ``snapcraft remote-build`` command offloads the snap build process to the `Launchpad build farm <https://launchpad.net/builders>`__, pushing the potentially multi-architecture snap back to your machine. See below for further details.

Remote build is a feature in :ref:`Snapcraft <snapcraft-overview>` that enables anyone to run a multi-architecture snap build process on remote servers using `Launchpad <https://launchpad.net/>`__.

With remote build, you can build snaps for hardware you don’t have access to and free up your local machine for other tasks.

Supported build architectures are: **amd64**, **arm64**, **armhf**, **i386**, **ppc64el** and **s390x**.

   ℹ See :ref:`Creating a snap <creating-a-snap>` for details on creating the metadata required to build a snap. For other ways to build a snap, see :ref:`Build options <build-options>`.


Prerequisites
-------------

Prospective snaps need to be open source, as the code will be publicly available, and you’ll need a `Launchpad account <https://login.launchpad.net/+new_account>`__.

Build architectures can be defined within a snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` using the *architectures* keyword. To target all architectures, for example, use the following:

.. code:: yaml

   architectures:
     - build-on: s390x
     - build-on: ppc64el
     - build-on: arm64
     - build-on: armhf
     - build-on: amd64
     - build-on: i386

If *architectures* is not defined within snapcraft.yaml, target architectures can be specified at build-time with the ``--build-on`` argument:

.. code:: bash

   snapcraft remote-build --build-on=amd64,arm64

If no architecture is specified, remote build will default to ``amd64``. For more details on how snaps handle build and run architectures, see :ref:`Architectures <architectures>`.


Using remote build
------------------

To instantiate a remote build, use the ``remote-build`` argument with snapcraft:

.. code:: bash

   snapcraft remote-build

1. You are first asked to confirm that you’re happy for your local project to be transferred to a remote build server and become publicly available:

   .. code:: text

      All data sent to remote builders will be publicly available. Are you sure
      you want to continue? [y/N]: y

   Skip the above by passing ``--launchpad-accept-public-upload`` to snapcraft as an extra argument.

2. Snapcraft will now launch your default browser with an authorisation URL. The URL is also output to the terminal to allow you to copy and paste it.

   .. code:: text

      The authorization page:
       (https://launchpad.net/+authorize-token?
      oauth_token=xxx&allow_permission=DESKTOP_INTEGRATION)
      should be opening in your browser. Use your browser to authorize
      this program to access Launchpad on your behalf.
      Waiting to hear from Launchpad about your decision...

   This prompt occurs the first time you use remote build from an new machine. Access can be enabled until you disable it, for one hour, for one day, or for one week. Alternatively, you can use the same link to disable access completely.

The remote build process will now start.

The following is typical output for a successful single architecture remote build:

.. code:: bash

   Sending build data to Launchpad... (https://<username>:<token>@git.launchpad.net/<username>/+git/snapcraft-hello-22ef03/)
   If interrupted, resume with: 'snapcraft remote-build --recover'
   Building snap package for amd64. This may take some time to finish.
   Build status as of 2019-11-29 11:44:50.017631:
           arch=amd64      state=Needs building
   Build status as of 2019-11-29 11:45:20.215169:
           arch=amd64      state=Currently building
   Build status as of 2019-11-29 11:45:50.472400:
           arch=amd64      state=Currently building
   Build status as of 2019-11-29 11:46:20.968422:
           arch=amd64      state=Currently building
   Build status as of 2019-11-29 11:46:51.206255:
           arch=amd64      state=Uploading build
   Build status as of 2019-11-29 11:47:21.871779:
           arch=amd64      state=Uploading build
   Build status as of 2019-11-29 11:47:52.197560:
           arch=amd64      state=Successfully built
   Snapped hello_2.10_amd64.snap
   Build log available at 'hello_amd64.1.txt'
   Build complete.



Snapcraft waits for the build to complete before retrieving the resultant snaps, and build logs, and placing them all in your local build directory. Build time depends on the target architecture, the package size, and the availability of builder back-ends.

If your build is interrupted for any reason, it can be resumed with the ``--recover`` argument:

.. code:: bash

   snapcraft remote-build --recover


Monitor a build
---------------

Command output from remote build will show build progress for each architecture. You can retrieve the same output from another terminal session within the build directory using the ``--status`` argument:

.. code:: bash

   snapcraft remote-build --status

To see build progress outside of your command line session, open the following URL in a web browser: https://launchpad.net/~/+snaps.

From the snap packages web page, select the build data for the job you want to monitor. The specific name for a job is part of the output from the remote-build command, such as ``snapcraft-hello-22ef03``.

.. figure:: https://assets.ubuntu.com/v1/04cd2c65-snapcraft-hello_01.png
   :alt: Launchpad remote build management


Selecting the build page for a build allows you to monitor the build progress for each architecture, and access the completed build log for each.

The Launchpad build page, and the remote build, is removed after a build terminates, regardless of whether the build was successful or not.
