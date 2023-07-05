.. 1460.md

.. _process-for-reviewing-classic-confinement-snaps:

Process for reviewing classic confinement snaps
===============================================

Classic confinement review process
----------------------------------

Background
~~~~~~~~~~

As of snapd 2.20, snappy supports ``confinement: classic`` which allows the snap to run without restrictions. Future releases of snapd will also support a classic interface (name TBD) that operates similarly. Snaps specifying classic confinement may target the stable channel, but are only supported on classic distro systems (ie, not on Ubuntu Core).

Because classic confinement snaps run without restrictions, use of classic confinement effectively grants device ownership to the snap. Due to the sensitive nature of classic confinement:

-  users must specify ``--classic`` when using ``snap install`` to install a snap using classic confinement
-  the review process in the snap store will flag for human review snaps that specify classic confinement
-  the store provides a mechanism for the reviewer to allow classic confinement to the snap so that subsequent uploads do not trigger human review
-  the publisher shall be vetted using the processes in this topic before classic confinement is granted by the store

Definitions
~~~~~~~~~~~

-  reviewers are @reviewers
-  snappy architects are Mark, Gustavo, Samuele, etc
-  advocacy team is @advocacy
-  classic confinement is defined as ``confinement: classic`` and the upcoming ``classic`` interface (final name TBD)
-  classic confinement applies to a particular snap ID and may be revoked by the store

Process
~~~~~~~

1. the publisher makes the request for classic confinement in the forum using the `store-requests category <https://forum.snapcraft.io/c/store-requests>`__
2. the technical reasons for why the snap uses classic confinement are gathered in the forum post and captured for potential future snapd improvements. The technical requirements will be reviewed by the security team and/or an architect
3. the advocacy team, reviewers team and/or architects participate in vetting the snap/publisher
4. once the publisher has been vetted, the technical reasons are captured and the request is approved, a store reviewer will issue a snap declaration for the snap and add a comment to the store, giving the URL to the forum post

Known categories
~~~~~~~~~~~~~~~~

Classic requests generally fall under a number of categories. Below lists categories that developers may consult for things that are known to be allowed/disallowed use of classic confinement. @reviewers may consult these lists when processing classic requests. If something falls outside of these lists, then the requirements must be gathered by a senior reviewer and discussed with an architect (after which, it can be added to the lists).

Supported
^^^^^^^^^

-  compilers
-  debug tools
-  IDEs
-  juju helpers
-  kubernetes tools requiring `arbitrary authentication agents <https://snapcraft.io/docs/classic-confinement-for-kontena-lens18>`__
-  `nautilus scripts <https://snapcraft.io/docs/synchrorep-need-classic-confinement8>`__
-  programming languages
-  public cloud agents
-  tools for local, non-root user driven configuration of/switching to development workspaces/environments
-  terminal emulators, multiplexers and shells
-  HPC or orchestration agents/software for running workloads on systems without traditional users where the systems are otherwise managed outside of the agent (ie, the software simply orchestrates running workloads on systems and doesn’t manage the systems themselves). Note: many HPC/orchestration applications can run with strict confinement and classic should only be granted if snapd does not support the specific use case (eg, the `need for user accounting <https://snapcraft.io/docs/request-for-classic-confinement-slurm11>`__).

Unsupported
^^^^^^^^^^^

-  management snaps
-  3rd party installer snaps (eg, for native packages, appimages, flatpaks, snaps, etc)
-  difficulty making strict confinement work
-  dependent software only available on host (ship in instead snap (eg, stage-packages, build from source))
-  access to dot files in $HOME (use $HOME instead of getent*, personal-files)
-  access to /etc (use layouts, system-files)
-  hard-coded paths (use snapcraft-preload, layouts)
-  ability to run other snaps directly (as opposed to defined interfaces)
-  access to arbitrary files on the system because the application isn’t designed with confinement in mind (if a desktop application, use portals or `xdg-open <https://snapcraft.io/docs/allowing-xdg-open-to-open-files11>`__)
-  access to arbitrary files on the system due to developer/user inertia (home and removable-media is almost always sufficient, though :ref:`personal-files <the-personal-files-interface>` and :ref:`system-files <the-system-files-interface>` may be used under certain circumstances.
-  access to arbitrary files on the system to avoid increasing a snap’s size
-  `GNOME shell extensions <https://snapcraft.io/docs/yaru-dark-theme-toggle-review-request7>`__
-  `nautilus extensions <https://snapcraft.io/docs/synchrorep-need-classic-confinement8>`__
-  access to `org.kde.PlasmaShell.evaluateScript <https://snapcraft.io/docs/issue-establishing-dbus-interface-with-org-kde-plasmashell4>`__
-  access to `org.kde.klauncher5 (klauncher) <https://snapcraft.io/docs/kde-error-unable-to-create-io-slave-cannot-talk-to-klauncher>`__ (`modify application <https://forum.snapcraft.io/t/18377/3>` to launch programs directly)
-  direct access to sudo (modify program (eg, check if root and if not, alert user to run under sudo))
-  direct access to pkexec (modify program (eg, check if root and if not, alert user to run under sudo; note a polkit backend is planned but not roadmapped, so a snap may one day be able to use pkexec, but this is TBD)).

Criteria
^^^^^^^^

This lists some criteria that might require classic (non-exhaustive):

* access to files on the host outside the snap’s runtime (eg, /usr)
* running arbitrary command (esp if user-configurable such as a developer tool to organize dev environments)
* access to resources not yet supported by snapd and where the requirement is clearly understood to be supportable by snapd. This may result in temporarily granting classic until snapd supports the use case in strict mode

NOTE: while something may be known to require classic, that alone may not justify granting classic confinement.

Caveats
^^^^^^^

Classic confinement sometimes might seem like the perfect solution to a publisher’s problem, but snaps that use ``confinement: classic`` differ from strict mode snaps in important ways:

1. they are not installable on Ubuntu Core (all snaps) devices
2. they run in the global mount namespace (ie, the host’s filesystem) as opposed to what is specified by ``base`` in the snap’s yaml.

Because of ‘2’, great care must be taken for the snap to work reliably across all distributions since, for example, as part of the build process snapcraft will adjust the snap’s binaries through binary patching and/or setting environment variables to look into the ``$SNAP`` directory for paths, either of which could affect the snap’s reliability when running on arbitrary cross-distribution host filesystems. In contrast, strict mode snaps use what is specified by ``base`` in the snap’s yaml as the basis for its root filesystem at runtime and can depend on it not changing.

Additional
^^^^^^^^^^

Sometimes it might make sense for a snap to be allowed the use of classic (eg, for classic distro) but be usable in strict mode (eg, for Ubuntu Core). In these cases, rather than having two separate snaps, it is considered best to have `two separate tracks <https://snapcraft.io/docs/new-track-classic-request-for-the-nano-snap11>`__, the default track and another called ``classicmode``.
