.. 12601.md

.. _services-and-daemons:

Services and daemons
====================

When :ref:`creating snapcraft.yaml <creating-snapcraft-yaml>` to build a new snap, a snap’s executable component can be either exposed as a command or run as a background service or daemon.

For details on how to expose an executable from its constituent parts, see :ref:`Defining a command <defining-a-command>`.

A snap daemon or service behaves the same as a native daemon or service, and will either start automatically at boot time and end when the machine is shutdown, or start and stop on demand through socket activation.

Snap confinement prohibits a system’s users and groups from running as traditional services might, such as under a user’s ownership. But a *snap_daemon* user and group can alternatively be created within a snap to provide similar user and group level control outside of a snap’s confinement. See :ref:`System usernames <system-usernames>` for more details.

See `Service management <https://snapcraft.io/docs/service-management>`__ for details on starting and stopping services from the *snap* command. Services and daemons can also be managed from within a snap, such as via a hook, with the :ref:`snapctl <snapctl-tool-services>`.

To set memory and CPU resource limits for a service or daemon, see `Quota groups <https://snapcraft.io/docs/quota-groups>`__

If you need to add user configurable options to your service or daemon, such as which port it should use, see :ref:`Adding snap configuration <adding-snap-configuration>`.

.. _defining-a-daemon:

Defining a daemon
-----------------

To define an executable as a daemon or service, add ``daemon: simple`` to its *apps* stanza:

.. code:: yaml

   apps:
     part-os-release:
       command: bin/os-release.sh
       daemon: simple

The value for the ``daemon`` keyword can be one of the following:

``simple``
   Run for as along as the service is active. This is typically the default. option.
``oneshot``
   Run once and exit after completion, notifying systemd. After completion, the daemon is still considered active and *running*.
``forking``
   The configured command calls ``fork()`` as part of its start-up and the parent process is then expected to exit when start-up is complete. This is not the recommended behaviour on a modern Linux system.
``notify``
   Assumes that the command will send a signal to *systemd* to indicate its running state. Note this requires usage of the :ref:`daemon-notify interface <the-daemon-notify-interface>`.
``dbus``
    Assumes that the command will claim a D-Bus name to indicate its running state to *systemd*.

In addition to the above types of daemon or service, the following can be set to help manage how a service is run, how it can be stopped, and what should happen after it stops:

- **after** An ordered list of applications the daemon is to be started *after*. Applications must be part of the same snap. - **before** An ordered list of applications the daemon is to be started *before*. Applications must be part of the same snap.
- **install-mode** Defines whether a freshly installed daemon is started automatically, or whether startup control is deferred to the snap. The snap could then use :ref:`snapctl <using-the-snapctl-tool>` with a :ref:`hook <supported-snap-hooks>`, for instance, or another management agent. Can be ``enable`` (default) or ``disable``.
- **post-stop-command** Sets the command to run from inside the snap *after* a service stops.
- **refresh-mode** Controls whether a daemon should be restarted during a snap refresh. Can be either ``restart`` , ``endure``, (do not restart) or ``ignore-running`` (does not refresh running services to facilitate the refresh app awareness feature). Defaults to *restart* .
- **reload-command** Defines the command within the snap to be executed when the service is restarted. Example: ``reload-command: sbin/nginx -s reload``
- **restart-condition** Defines when a service should be restarted, using values returned from `systemd service exit status <https://www.freedesktop.org/software/systemd/man/systemd.service.html#Restart=>`__. Can be one of ``[on-failure|on-success|on-abnormal|on-abort|on-watchdog|always|never]``.
- **restart-delay** The delay between service restarts. Defaults to unset. See the systemd.service manual on RestartSec for details. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``.
- **sockets** Requires an activated daemon socket, and works with the :ref:`network-bind interface <the-network-bind-interface>` to map a daemon’s socket to a service and activate it.
- **socket-mode** The mode of a socket in octal.
- **start-timeout** Optional time to wait for daemon to start. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``.
- **stop-command** An optional executable command to run before the daemon is stopped, and the daemon is not stopped until the specified *stop-command* terminates. This can be to used to gracefully handle a daemon stop or restart, such as when a *refresh* happens, by allowing the daemon to reach a stoppable state first.
- **stop-mode** Defines which `termination signal <https://www.gnu.org/software/libc/manual/html_node/Termination-Signals.html>`__ to use when stopping the daemon. Can be one of either ``sigterm``, ``sigterm-all``, ``sighup``, ``sighup-all``, ``sigusr1``, ``sigusr1-all``, ``sigusr2``, ``sigusr2-all``, ``sigint`` and ``sigint-all``.
- **stop-timeout** The length of time to wait before terminating a service. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``. Termination is via ``SIGTERM`` (and ``SIGKILL`` if that doesn’t work).
- **timer** Declares that the service is activated by a timer and that the app must be a daemon. See `Timer string format <https://snapcraft.io/docs/timer-string-format>`__ for syntax examples.
- **watchdog-timeout** This value declares the service watchdog timeout. For watchdog to work, the application requires access to the *systemd* notification socket, which can be declared by listing a daemon-notify plug in the plugs section. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``.

For further details, see :ref:`Snapcraft app and service metadata <snapcraft-app-and-service-metadata>`.


Daemons and D-Bus
-----------------

Daemons can configured to interact with D-Bus in a number of ways.
D-Bus can be used to indicate to *systemd* that a daemon is running, it can
be used as the mechanism to activate a daemon, and it can be used generally
to expose services to applications.

D-Bus activation can only be used for services on the system bus.

Daemon type
~~~~~~~~~~~

The :ref:`daemon <snapcraft-yaml-daemon>` keyword is used to specify
the :ref:`type of a daemon <defining-a-daemon>` and the mechanism it uses to
inform *systemd* that it is running.

A daemon can be configured to use D-Bus to notify *systemd* that it is running
by claiming a D-Bus name. This behaviour is enabled by setting the ``daemon`` keyword to a value of ``dbus`` in the app metadata.

Daemons that use D-Bus in other ways that do not need this feature can set the
``daemon`` type to a value other than ``dbus``. This enables other methods to
be used to indicate to *systemd* that they are running.

If the ``dbus`` type is used, either the
:ref:`bus-name <snapcraft-yaml-bus-name>` keyword or
:ref:`activates-on <snapcraft-yaml-activates-on>` keyword must be
used to define a bus name for the daemon. If both keywords are defined, the
bus name takes precedence. If only the ``activates-on`` keyword is defined,
the last name in its list of slots is used as the bus name.

Activation
~~~~~~~~~~

The :ref:`activates-on <snapcraft-yaml-activates-on>` keyword is used
to define a list of names that will be exposed via D-Bus. These names are
automatically added to the slots for the snap.

This provides a way for a daemon to be started on a D-Bus method call. When a
method on any of the names is invoked, the daemon's
:ref:`command <snapcraft-yaml-command>` is run to start the daemon.

General use
~~~~~~~~~~~

A daemon that needs to provide services to applications can be configured
to use a bus name by setting its
:ref:`bus-name <snapcraft-yaml-bus-name>` keyword. This enables the
system bus to be used for communication, as with regular system daemons.

As noted above, the :ref:`daemon <snapcraft-yaml-daemon>` keyword
does not need to specify the ``dbus`` type for this use case, unless it is
convenient to notify *systemd* about start-up by claiming a D-Bus name.
