.. 13386.md

.. _system-usernames:

System usernames
================

System usernames can be used by snap developers to enable them to run :ref:`services and daemons <services-and-daemons>` as a user other than the default ``root``.

Outside of snaps, applications traditionally adopt the concept of users and groups from the host operating system to use as a security mechanism that grants access to specific system and software resources.

Snap confinement prohibits a system’s users and groups from being used in this way within a snap but a *snap_daemon* user and group can alternatively be created within a snap to provide similar user and group level control outside of a snap’s confinement.

snap_daemon user and group
--------------------------

From version 2.41 onwards, snapd supports the creation of a ``snap_daemon`` user and group within a snap, exposed as user ID (UID) and group ID (GID) ``584788`` on the host system.

   ℹ Future releases of snapd will support more users and groups.

To create the ``snap_daemon`` user/group inside a snap, add the following ``system-usernames`` section to the snap’s :ref:`snapcraft.yaml <snapcraft-yaml-reference>` (or ``snap.yaml``):

.. code:: yaml

   system-usernames:
     snap_daemon: shared

With the above added, the ``584788`` user and group can be used by a snap via standard APIs (eg, ``getpwent()``, the ``getent`` command, etc) to perform various ownership (eg, ``chown()`` and ``chgrp()``) and privilege separation operations (eg, ``setuid()`` and ``setgid()``).

Usage considerations
--------------------

In general, the snap may use the allocated system username as desired. However, due to snapd’s security sandbox, the snap may need to be adjusted to work reliably.

Using ``system-usernames`` allows a snap to drop privileges and transition to the specified user, but the daemon will still start as root and be started from systemd on boot.

Dropping privileges
~~~~~~~~~~~~~~~~~~~

Security best practice dictates that applications drop privileges as follows:

.. code:: c

   // Error messages, etc., omitted for clarity
   char *user = "snap_daemon";
   struct passwd *pwd = getpwnam(user);
   if (pwd == NULL) {
       exit(1);
   }
   if (setgroups(0, NULL) < 0) {
       exit(1);
   }
   if (setgid(pwd->pw_gid) < 0) {
       exit(1);
   }
   if (setuid(pwd->pw_uid) < 0) {
       exit(1);
   }

The observant reader will notice that the call to ``setgroups()`` uses the non-portable, Linux-specific, invocation *size 0* and *NULL list* to clear the list completely. This contrasts with a more portable *size 1* and a list consisting of the single primary group. This is due to current sandbox limitations. If desired, ``setgroups()`` may be overridden via LD_PRELOAD. See the `test-setgroups snap <https://git.launchpad.net/~jdstrand/+git/test-setgroups/tree/>`__ for an example).

For those using ``base: core18`` or higher, the ``setpriv`` command may be used to drop privileges like so:

::

   $SNAP/usr/bin/setpriv --clear-groups --reuid snap_daemon \
     --regid snap_daemon -- <your command>

``base: core18`` snaps should stage ``setpriv`` while ``base: core20`` should stage ``util-linux``. On snapd < 2.45, LD_PRELOAD is still needed due to how the sandbox is configured and how ``setpriv --clear-groups`` is using ``setgroups(0, <NON_NULL>)``:

::

   LD_PRELOAD=$SNAP_COMMON/wraplib.so $SNAP/usr/bin/setpriv \
     --clear-groups --reuid snap_daemon --regid snap_daemon -- <your command>

Some applications may try to drop privileges themselves using ``initgroups()`` which under the hood uses ``setgroups()`` in a way that is blocked by the sandbox. In this case, the snap would need to use the LD_PRELOAD method. See the `test-setpriv snap <https://git.launchpad.net/~jdstrand/+git/test-setpriv/tree/>`__ for how to use ``setpriv`` with and without LD_PRELOAD.

   ℹ Future releases of snapd will support setting ``User=snap_daemon`` and ``Group=snap_daemon`` in the systemd unit file.

Process management via signals
------------------------------

By default, a snap’s security sandbox limits the sending and receiving of signals to processes in the snap where the sender and the receiver have the same owner, even when the process runs as root (unless the process has CAP_KILL).

When developing a snap, care must therefore be taken to drop privileges *before* sending a signal to a process that has privilege dropped.

For example, a management process that runs as root may fork off worker processes that drop privileges to the snap_daemon user. Whenever this management process wants to send a signal to its workers, it must drop privileges to the snap_daemon user first. Alternatively, the snap could use ``plugs: [ process-control ]``, which among other things, grants CAP_KILL.

Since the `process-control <https://snapcraft.io/docs/process-control-interface>`__ interface grants considerable access for system-wide process management, best practice dictates that privileges *must* be dropped as needed when sending signals to other processes in the snap.

Ownership (discretionary access controls)
-----------------------------------------

The security sandbox generally limits file access to where the object id of the file (ie, the owner) matches the uid of the running process. This is true even for root, unless the process has CAP_DAC_OVERRIDE or CAP_DAC_READ_SEARCH defined, which the security sandbox intentionally denies.

As such, care must be taken when creating files after dropping privileges *if* those files are intended to be accessed by other processes in the snap that have not had their privileges dropped.

While each snap’s requirements may differ, in general, a reasonable approach is to create files and directories with ``<dropped user>:root`` ownership, allowing owner and group read and write. This allows allows both the *privilege dropped* user and the *root* user within the snap to access to the files.

For example, if the snap utilises the ``snap_daemon`` user, the snap might (as part of a configure hook or a wrapper script, for instance) do:

.. code:: sh

   if [ ! -d "$SNAP_DATA/dir" ]; then
       mkdir "$SNAP_DATA/dir"
       chmod 770 "$SNAP_DATA/dir" # must be before chown
       chown snap_daemon "$SNAP_DATA/dir"
       chgrp root "$SNAP_DATA/dir"  # not needed but shown for clarity
   fi

Security best practice dictates file access should be performed with the minimal privileges necessarily, so the snap is of course free to privilege-drop prior to accessing the file instead.

subuid, subgid and other container technologies
-----------------------------------------------

Snapd takes great care to avoid overlapping with other container technologies (or in the case of systemd, working with `systemd-nspawn\ ’s collision detection <https://github.com/systemd/systemd/blob/master/docs/UIDS-GIDS.md>`__). It uses the ``524288-589823`` UID/GID range, for instance, to help avoid the default ranges for LXD, Docker and other container systems.

Some administrators may adjust their non-snap container runtimes to use non-default values (eg, via ``/etc/subuid``, ``/etc/subgid``, etc). While it is non-fatal for other container ranges to overlap with snapd’s range, best practice dictates that a different range should always be used to ensure a clean separation between snapd and other container ranges in the kernel on the system.

References
==========

-  https://snapcraft.io/docs/multiple-users-and-groups-in-snaps
