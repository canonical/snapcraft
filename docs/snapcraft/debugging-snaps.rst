.. 18420.md

.. _debugging-snaps:

Debugging snaps
===============

.. note::
          **NOTE TO EDITORS**

          This topic is currently under construction. In particular, we want to add usage documentation on the following:

          □ –strace

Each snap runs inside its own :ref:`confined environment <snap-confinement>`, also called “sandbox”. The policy of each sandbox describes what the application is allowed to do. When an application tries to do something that is not allowed, the system logs a policy violation. The following techniques can help you investigate and solve these policy violations.

-  Use :ref:`snap try <debug-snaps-with-snap-try>` to quickly test changes without rebuilding your snap.

-  Use `snap run --shell <debugging-snaps-shell_>`__ to inspect and test the confined environment.

-  Use `developer mode <debugging-snaps-developer_>`__ to try your snap without confinement.

-  Investigating policy violation logs.

   -  Use `snappy-debug <debugging-snaps-snappy-debug_>`__ to investigate violation logs and receive suggestions.
   -  `Manually search the raw logs <debugging-snaps-manual-log_>`__.
   -  `Understanding AppArmor <debugging-snaps-apparmor_>`__ violations.
   -  `Understanding seccomp <debugging-snaps-seccomp_>`__ violations.

-  Investigate `file permissions and cgroup device access <debugging-snaps-permissions_>`__ violations.

-  Use :ref:`GDB and gdbserver from within a snap’s environment <using-gdb-and-gdbserver>` to isolate and identify potential issues.

For more details on how AppArmor, seccomp and device permission security policies are implemented, see `Security policy and sandboxing <https://snapcraft.io/docs/security-policy-and-sandboxing>`__.


.. _debugging-snaps-shell:

Run a shell in the confined environment
---------------------------------------

To investigate and test the confined environment of a snap, you can open a ``bash`` shell in it. After the snap is installed, use the ``--shell  <name>.<command>`` argument of ``snap run``.

.. code:: bash

   $ snap run --shell mysnap.mycommand
   To run a command as administrator (user "root"), use "sudo <command>".
   See "man sudo_root" for details.

This will create the confined environment of the Snap, execute the :ref:`command-chain <snapcraft-app-and-service-metadata-command-chain>` and then run ``bash`` inside that environment.

You can then investigate which files your snap has access to by running commands such as ``ls`` and ``cat``.

   ⓘ It’s important to put ``--shell`` *before* the name of the snap. Otherwise it will be interpreted as an argument to the application instead of an argument to ``snap run``.


.. _debugging-snaps-developer:

Developer mode
--------------

To help isolate runtime errors when building and testing a snap, a snap can be installed using *developer mode*.

To install a snap in developer mode, use the ``--devmode`` argument:

.. code:: bash

   sudo snap install --devmode mysnap

When a snap is installed with developer mode, violations against a snap’s security policy are permitted to proceed but logged via journald.


.. _debugging-snaps-debugging:

Debugging policy violation logs
-------------------------------


.. _debugging-snaps-snappy-debug:

Using snappy-debug to show violations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The easiest way to find and fix policy violations is to use `the ``snappy-debug`` tool <https://snapcraft.io/snappy-debug>`__. It

-  watches syslog for policy violations,
-  shows them in a human readable format,
-  and makes recommendations for how to solve them.

First, install the tool by running the following command.

.. code:: shell

   sudo snap install snappy-debug

Then, run the following command to start watching policy violations.

.. code:: shell

   $ sudo snappy-debug
   INFO: Following '/var/log/syslog'. If have dropped messages, use:
   INFO: $ sudo journalctl --output=short --follow --all | sudo snappy-debug

If you have dropped messages, try the following command instead.

.. code:: shell

   sudo journalctl --output=short --follow --all | sudo snappy-debug

Note: these commands only show policy violations that happen *after* you run them. So first run one of these commands and then run the snap that you want to debug.

See ``snappy-debug --help`` for more information about this tool.

If you believe there is a bug in a security policy or want to request and/or contribute a new interface, please `file a bug <https://bugs.launchpad.net/snappy/+filebug>`__, adding the ``snapd-interface`` tag, and feel free to discuss policy issues `on the forum <https://forum.snapcraft.io/c/snapd>`__.


.. _debugging-snaps-manual-log:

Manually extracting violation logs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

..

   Note that this method does not show *all* violation logs, since not all logs contain the term “audit” in them. Use ``snappy-debug`` to see all violation logs.

You can also manually show snap policy violations by searching the logs for *audit*.

.. code:: bash

   $ sudo journalctl --since=yesterday | grep audit

The above command uses ``--since=yesterday`` to limit the typically verbose logging output from journalctl.

A handy debugging technique is to tail/follow journalctl output while exercising the snap:

.. code:: bash

   $ sudo sysctl -w kernel.printk_ratelimit=0 ; journalctl --follow | grep audit

As shown above, kernel log rate limiting can be disabled manually with: ``bash $ sudo sysctl -w kernel.printk_ratelimit=0``


.. _debugging-snaps-apparmor:

Understanding AppArmor violations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An AppArmor violation will look something like the following and include ``apparmor=DENIED``:

.. code:: text

   audit: type=1400 audit(1431384420.408:319): apparmor="DENIED" operation="mkdir" profile="snap.foo.bar" name="/var/lib/foo" pid=637 comm="bar" requested_mask="c" denied_mask="c" fsuid=0 ouid=0

If there are no AppArmor denials, AppArmor shouldn’t be blocking the snap.

To better understand AppArmor policy for a strictly installed snap, modify the AppArmor policy *in place* on the target system. Changes aren’t persistent, but this can help when considering a snapd patch or bug report.

For example:

1. build the snap
2. copy the snap to the target device and install it (or use :ref:`snap try <debug-snaps-with-snap-try>`)
3. use the snap (perhaps using `snap run --shell <name>.<command> <debugging-snaps-shell_>`__), monitoring via journalctl for denials
4. modifying ``/var/lib/snapd/apparmor/profiles/snap.<name>.<command>`` as needed (eg, adding rules before the final ``'}'``)and running ``sudo apparmor_parser -r /var/lib/snapd/apparmor/profiles/snap.<name>.<command>`` to compile and load the policy into the kernel
5. use ``sudo service snap.<name>.<command> stop/start/etc`` as needed for daemons
6. repeat until AppArmor policy issues are resolved


.. _debugging-snaps-seccomp:

Understanding seccomp violations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A seccomp violation will look something like:

.. code:: text

   audit: type=1326 audit(1430766107.122:16): auid=1000 uid=1000 gid=1000 ses=15 pid=1491 comm="env" exe="/bin/bash" sig=31 arch=40000028 syscall=983045 compat=0 ip=0xb6fb0bd6 code=0x0

The ``syscall=983045`` can be resolved by running the ``scmp_sys_resolver`` command on a system of the same architecture as the one with the seccomp violation:

.. code:: bash

   $ scmp_sys_resolver 983045
   set_tls

If there are no seccomp violations, seccomp isn’t blocking the snap.

If you notice ``compat=1`` in the seccomp denial, then specify the correct compatibility architecture to ``scmp_sys_resolver`` with ``-a <arch>``. For example, when on an amd64 system, use ``scmp_sys_resolver -a x86 191`` (use ``-a arm`` on arm64 systems).

The seccomp filter profile in expected to be located in ``/var/lib/snapd/seccomp/bpf/*.src`` (formerly ``/var/lib/snapd/seccomp/profiles``).

The seccomp profile source (the ``*.src`` file in the profile directory) needs to be recompiled into the profile binary (``*.bin`` in the profile directory) as follows:

.. code:: bash

   sudo /usr/lib/snapd/snap-seccomp compile /var/lib/snapd/seccomp/bpf/snap.$SNAP_NAME.src /var/lib/snapd/seccomp/bpf/snap.$SNAP_NAME.bin

The ``snap-confine`` command will load the bpf in the ``.bin`` file for the command when you (re)launch the command or ``snap run --shell``. The seccomp policy language is considerably simpler and is essentially a list of allowed syscalls.

When done, copy any changes you make to ``/var/lib/snapd/apparmor/profiles/snap.<name>.<command>`` or ``/var/lib/snapd/seccomp/bpf/snap.<name>.<command>.src`` to your interface code.


.. _debugging-snaps-snapseccomp:

snap-seccomp versions and paths
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Tools such as snap-confine, snap-seccomp and snap-exec are internal to snapd and are initially installed with a distribution’s snapd package.

On certain distributions, these tools can become superseded by versions embedded in subsequently installed core and snapd snaps. When developing a seccomp profile, it is important that the correct snap-seccomp binary is used. This can be determined by inspecting which binary is running as *snapd*.

With re-execution from the subsequently installed core and snapd snaps, these tools get called using their full path from the same location as the currently running binary. This is visible from ``/proc``:

.. code:: bash

   # with reexecution
   $ sudo ls -l /proc/$(pidof snapd)/exe
   lrwxrwxrwx 1 root root 0 Jun  5 10:10 /proc/1994/exe -> /snap/snapd/7777/usr/lib/snapd/snapd

Thus tools such as snap-seccomp will be called using its full path, ``/snap/snapd/7777/usr/lib/snapd/snap-seccomp``.

Without re-execution, the snapd process is using a binary located in the host filesystem:

.. code:: bash

   # no reexecution
   $ sudo ls -l /proc/$(pidof snapd)/exe
   lrwxrwxrwx 1 root root 0 06-05 12:49 /proc/808335/exe -> /usr/lib/snapd/snapd

Correspondingly, ``snap-seccomp`` will be called using its full path ``/usr/lib/snapd/snapd``.


.. _debugging-snaps-permissions:

File permissions
----------------

While tradition file permissions are respected and enforced, any violations are not currently logged. Similarly, device cgroups may also block access without logging denials.

To check whether device cgroups are affecting a snap’s device access:

1. see if there are any snapd-generated udev rules in ``/etc/udev/rules.d/70-snap.$SNAPNAME.rules``
2. if rules are defined, use ``udevadm info /dev/$DEVICE`` to see if the snap shows up in TAGS, or see if the ``/run/udev/tags/snap_$SNAPNAME_$COMMAND`` directory exists
3. examine if the ``/sys/fs/cgroup/snap.$SNAPNAME.$COMMAND`` directory exists and if the device is listed in ``/sys/fs/cgroup/devices/snap.$SNAPNAME.$COMMAND/devices.allow`` (eg, ``/dev/kmsg`` would have ‘``c 1:11 rwm``’ since ``/dev/kmsg`` is a character device with MAJOR:MINOR as 1:11 (see ``ls -l /dev/kmsg``))

For device cgroups, create or modify ``/etc/udev/rules.d/70-snap.$SNAPNAME.rules`` as necessary (eg, ``KERNEL=="kmsg" TAGS+="snap_$YOURSNAPNAME_$YOURCOMMAND"`` would tag ``/dev/kmsg`` for your snap), then run ``sudo udevadm trigger --action=change``. To undo the access, remove the file and run the ``udevadm`` command again. When done, update the interfaces code based on your changes.

If you believe there is a bug in the security policy or want to request and/or contribute a new interface, please `file a bug <https://bugs.launchpad.net/snappy/+filebug>`__, adding the ``snapd-interface`` tag.

.. raw:: html
   ### Interface development and security policy

   When participating in snappy development and implementing new interfaces for others to use, you will almost always need to write security policy for both the slots and the plugs side of the interface but keep in mind you are not expected to write perfect security policy on the first try. The review process for snapd includes a security review of the interface security policy and it is expected that the security policy will be iterated on during the review process (in other words, if you are stuck on writing security policy but the interface otherwise works, feel free to submit the interface and ask for help).

   In addition to the above, here are some other useful techniques when debugging/developing policy:

    * temporarily specify `@unrestricted` in the seccomp policy and this will allow all syscalls
    * temporarily use a combination of bare AppArmor rules to focus on only the parts you want. For example:

       ```
       file,
       capability,
       network,
       mount,
       remount,
       pivot_root,
       umount,
       dbus,
       signal,
       ptrace,
       unix,
       ```
    * look at existing policy in `interfaces/apparmor/template.go`, `interfaces/seccomp/template.go` and `interfaces/builtin/*` for examples of the policy language
    * [stracing snaps](stracing-snap-commands.md). In addition to simply stracing the app, it can also be helpful to strace the app in both devmode and strict confinement and comparing the results.
    * when testing new versions of snappy-app-dev, if re-exec is enabled you will need to copy the new version to the location udev expects it (eg, `/lib/udev`) and then bind mount it over where the re-exec'd snap-confine expects it (eg, `mount --bind /lib/udev/snappy-app-dev /snap/core/<version>/lib/udev/snappy-app-dev`)

   The above command has changed to snap-device-helper
   

.. raw:: html

   <h2 id="debugging-snaps-further">

Further reading

-  https://github.com/snapcore/snapd/tree/master/interfaces for existing interface code and policy
-  https://manpages.ubuntu.com/manpages/jammy/man5/apparmor.d.5.html
-  https://gitlab.com/apparmor/apparmor/-/wikis/Profiling_by_hand (but use the paths listed above and don’t use the ``aa-genprof`` or ``aa-logprof`` tools because they are not yet snappy-aware)
-  https://github.com/snapcore/snapd/wiki/snap-confine-Overview
-  https://assets.ubuntu.com/v1/66fcd858-ubuntu-core-security-whitepaper.pdf
-  https://github.com/snapcore/snapd/wiki/Snap-Execution-Environment
-  stracing-snap-commands.md
