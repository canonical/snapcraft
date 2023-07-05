.. 11658.md

.. _upgrading-snapcraft:

Upgrading snapcraft
===================

:ref:`Snapcraft 3 <snapcraft-release-notes>` contains fundamental improvements to the way snaps are built.

In particular, snaps are now built within an isolated build environments that’s tuned for a desired target. This provides API and ABI compatibility for every binary built within the environment, and ensures a build isn’t affected by outside dependencies and system configuration.

Build environments work by leveraging :ref:`base snaps <base-snaps>`. A base snap offers a run-time environment with a minimal set of libraries that are common to most applications. At build time, the *snapcraft* tool ensures you are creating a snap inside an environment specifically tailored for its base.

During normal operation, the build environment is isolated from the user, but there are ways to step into the environment to help with debugging:

-  ``--shell`` runs the build up to the lifecycle step specified and opens a shell within the build environment. Example: ``snapcraft prime --shell`` will the build up to the ``stage`` step and open a shell.
-  ``--shell-after`` runs the build up to and through the lifecycle step specified and opens a shell within the build environment. Example: ``snapcraft prime --shell-after`` will open a shell after the ``prime`` step has been processed.
-  ``--debug`` opens a shell inside the build environment after an error occurs.

Migrating snaps to bases
------------------------

To make the transition to bases easier, base functionality is only triggered when adding the ``base`` keyword in ``snapcraft.yaml``.

Both *snapd* and *snapcraft* remain compatible non-base snap configurations, but these snaps cannot take advantage of the latest features. Additionally, with no base defined, *snapcraft* runs in a special *2.x* compatibility mode that lacks the options found in later releases.

Features no longer available with bases
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When the ``base`` keyword is used within ``snapcraft.yaml`` the following, long-deprecated, features are no longer available:

-  *wiki* parts and their corner case quirks, such as allowing ``/`` (forward slash) in parts.
-  ``cleanbuild`` and triggering builds using LXD from certain environment variables. See :ref:`Build on LXD <build-providers>` for more details.
-  ``prepare``, ``build`` and ``install`` keywords, used in parts, have been replaced by ``override-build`` and ``snapcraftctl``. This means you can use ``override-`` for ``pull``, ``stage`` and ``prime`` stages too.
-  the ``snap`` keyword has been superseded by the ``prime`` keyword.
-  when calling build commands through snapcraft, ``--disable-parallel-build`` is no longer available. It can be setup per-part using the :ref:`build-attributes <snapcraft-parts-metadata-build-attributes>` property.
-  similarly, when calling build commands through snapcraft, ``--use-geoip`` (which affected ``stage-packages``) is no longer available.

Migrating from *deb* to *snap*
------------------------------

On Ubuntu and its derivatives, the *deb* package of Snapcraft may have been installed via the desktop package manager, or the ``apt`` command, and cannot be upgraded to Snapcraft 3.x. You can check by locating the *snapcraft* executable.

The following location indicates installation from a *deb*:

.. code:: bash

   $ which snapcraft
   /usr/bin/snapcraft

The following is the executable location for a *snap*:

.. code:: bash

   $ which snapcraft
   /snap/bin/snapcraft

To migrate from a *deb* snapcraft installation to a snap, first remove the *deb* package and then install the *snap*:

.. code:: bash

   sudo apt autoremove --purge snapcraft
   sudo snap install snapcraft --classic

The latest version of the snap will now be installed and will remain updated.
