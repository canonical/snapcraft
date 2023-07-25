.. 26004.md

.. _build-from-github:

Build from GitHub
=================

There are two methods for building snaps on Canonical-hosted servers, and both are available to every snap publisher:

-  **Snapcraft remote-build** The ``snapcraft remote-build`` command offloads the snap build process to the `Launchpad build farm <https://launchpad.net/builders>`__, pushing the potentially multi-architecture snap back to your machine. See :ref:`Remote build <remote-build>` for further details.

-  **Build from GitHub** This is a build service integrated into every publisher’s :ref:`Developer Account <create-a-developer-account>` on `snapcraft.io <https://snapcraft.io/>`__. It works by linking a snap’s GitHub repository with our Launchpad build service. See below for further details.

With *Build from GitHub*, a snap is rebuilt whenever a change is merged into the main branch of its respective GitHub repository. When a build successfully completes, it’s automatically released to a snap’s :ref:`edge channel <channels-risk-levels>`.

Supported build architectures are: **amd64** , **arm64** , **armhf** , **i386** , **ppc64el** and **s390x** .

   ℹ See :ref:`Creating a snap <creating-a-snap>` for details on creating the metadata required to build a snap. For other ways to build a snap, see :ref:`Build options <build-options>`.


Prerequisites
-------------

You will need a :ref:`Developer account <create-a-developer-account>` and accept that the source code for a prospective snap will be publicly accessible while on the build server. Projects also need to be hosted on a public `GitHub <https://github.com/>`__ repository.

The GitHub repository must contain at least a :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` file, and the snap build from a clone of the repository. The snap name needs to be :ref:`registered <registering-your-app-name>` with the Snap Store, and the same name needs to be declared in the :file:`snapcraft.yaml`.

Build architectures can be defined within a snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` using the :ref:`architectures <architectures>` keyword. To target all architectures, for example, use the following:

::

   architectures:
     - build-on: s390x
     - build-on: ppc64el
     - build-on: arm64
     - build-on: armhf
     - build-on: amd64
     - build-on: i386

A :ref:`snap base <base-snaps>` of ``core18`` is assumed by default, unless specified otherwise. If a snap’s base doesn’t support a specified architecture, it will not be built. If no architecture is specified, snaps for all base-compatible architectures will attempt to be built.


Link to GitHub
--------------

To link your snap’s GitHub repository to your snap developer account, make sure you’re logged in to the developer account and go to the `My snaps <https://snapcraft.io/snaps>`__ overview page. This is the default landing page when you log in.

Select the target snap and open its ‘Builds’ tab in the web UI. Use the *GitHub login* button to connect to GitHub. You will be asked to authorise read and write access for webhook creation, which is the mechanism used to trigger builds. Your GitHub account is now connected.


Select a repository
-------------------

With the GitHub account connected, the next step is to choose a repository.

This is accomplished by using the two drop-down menus, first to choose an organisation and then to choose the repository itself. When a repository is selected it is scanned for an appropriate :file:`snapcraft.yaml` configuration which, if detected, enables the *Start building* button:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/b/bfc72bc1a38e19de984786d4163d27afc852fb49.png
   :alt: image|677x361


Click on *Start building* to instantiate the build process and complete the linking process:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/a/adcfaf6fb18ef99655535c31875f2a980e8a9ec5.png
   :alt: 352253a18ea8e99a914ce6697d83cddfc9d3dc89|648x146


Monitor the build process
-------------------------

The *Builds* tab in the web UI will always show the build status for each supported architecture:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/e/e1274b75d1d4f61af27c4a4ad1a11d94b19fb27c.png
   :alt: image|648x380


Clicking on a build ID will take you to the status page for that specific job. This is useful if a build fails as it will contain the build log for analysis:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/e/e961a00115dee7d1f5a45c5b6e8be25920df079b.png
   :alt: image|672x396


When a build succeeds, it’s automatically released to the edge channel. The release history for those builds can be viewed from the *Releases* tab on the web UI by selecting *Launchpad* beneath the *Revisions available to release* heading:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/3/330e0d32ed9fb1496246f2db38548c417274e214.png
   :alt: image|672x341


See `Release management <https://snapcraft.io/docs/release-management>`__ for more details on how to promote and monitor release revisions and their channels.


Unlink and disable GitHub builds
--------------------------------

To unlink your GitHub repo and disable automatic snap builds, navigate to the *Builds* tab in the web UI and click on *Disconnect repo* at the top of the page and confirm the action:

.. figure:: https://forum-snapcraft-io.s3.dualstack.us-east-1.amazonaws.com/original/2X/f/f6af192ff385ad69a25d235f5386806a967997e1.png
   :alt: image|665x115


This will clear the build history on the same page, but you can still release any successful builds from the *Releases* page of the web UI.
