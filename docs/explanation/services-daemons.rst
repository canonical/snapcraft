.. meta::
     :description: Learn how to define services and daemons in snapcraft.yaml, including daemon models, D-Bus activation, and runtime management.


.. _explanation-services-daemons:

Services and daemons
====================

A snap’s executables are either exposed as a command or run as a background service or daemon.

A snap daemon or service behaves the same as a native daemon or service, and will either start automatically at boot time and end when the machine is shutdown, or start and stop on-demand through socket activation. This behavior is controlled by the ``daemon`` key in the :ref:`project file <explanation-snapcraft-yaml>`, which defines how the daemon behaves at runtime.


Defining a daemon
-----------------

A daemon is defined by adding an entry to the ``apps`` key and choosing a daemon model that matches how the process behaves at runtime. In practice, the most important decision is whether your service should be a long-running process, complete work and exit, signal readiness explicitly, or integrate with D-Bus activation and naming. Picking the model based on process lifecycle and startup semantics avoids fragile workarounds later, especially when you add restart behavior, hooks, or socket/timer activation.

When selecting daemon behavior for the exposed executable, it is best to treat the daemon type as one part of a broader service contract with systemd. Consider aligning the startup expectations, readiness signaling, shutdown behavior, and activation mechanism.

The example following shows how to define a simple daemon by using the ``daemon`` key:

.. code-block:: yaml
   :caption: snapcraft.yaml

   apps:
     part-os-release:
       command: bin/os-release.sh
       daemon: simple

The ``daemon`` key specifies the type of a daemon and the mechanism it uses to inform systemd that it's running.

For the complete list of app daemon keys and supported values, see :ref:`App keys <reference-snapcraft-yaml-app-keys>` in the ``snapcraft.yaml`` reference.

Runtime management
------------------

Snap confinement prohibits a system's users and groups from running as traditional services might, such as under a user's ownership. But a user and group named daemon can alternatively be created within a snap to provide similar user and group level control outside of a snap's confinement. See :external+snap:ref:`System usernames <interfaces-system-usernames>` for more details.

Services and daemons can also be managed from within a snap, such as via a hook, with the :external+snap:ref:`snapctl <how-to-guides-manage-snaps-use-snapctl>`.

:external+snap:ref:`Quota groups <how-to-guides-manage-snaps-use-resource-quotas>` can put constraints on a daemon's memory and CPU resources.

:ref:`Snap configurations <how-to-add-a-snap-configuration>` can control the daemon's behavior at runtime, providing a way for the user to modify properties such as its network port, logging level, or other runtime options.


Daemons and D-Bus
-----------------

Daemons can be configured to interact with D-Bus in a number of ways. D-Bus can be used to indicate to systemd that a daemon is running, as the mechanism to activate a daemon, or to expose services to apps.

D-Bus activation can only be used for services on the system bus.

A daemon can be configured to use D-Bus to notify systemd that it is running by claiming a D-Bus name. This behavior is enabled by setting the ``daemon`` key to ``dbus`` in the app entry.

Daemons that use D-Bus in other ways that do not need this feature can set the ``daemon`` type to a value other than ``dbus``. Then, another method is needed to indicate to systemd that they are running.

If the ``dbus`` type is used, either the ``bus-name`` or ``activates-on`` keys must be set to define a bus name for the daemon. If both keys are defined, the bus name takes precedence. If only the ``activates-on`` key is defined, the last name in its list of slots is used as the bus name.

Activation
~~~~~~~~~~

The ``activates-on`` key is used to define a list of names that will be exposed via D-Bus. These names are automatically added to the slots for the snap.

This provides a way for a daemon to be started on a D-Bus method call. When a method on any of the names is invoked, the daemon's ``command`` is run to start the daemon.

General use
~~~~~~~~~~~

A daemon that needs to provide services to applications can be configured to use a bus name by setting its ``bus-name`` key. This enables the system bus to be used for communication, as with regular system daemons.

The ``daemon`` key does not need to specify the ``dbus`` type for this use case, unless it is convenient to notify systemd about startup by claiming a D-Bus name.

.. The following needs to be removed once any missing pieces get added to the pydantic kitbash schema and the reference documentation gets updated to match so that none of the additional usage description for fields is missing from the reference documentation.

In addition to the above types of daemon or service, the following can be set to help manage how a service is run, how it can be stopped, and what should happen after it stops:

.. list-table::
   :header-rows: 1
   :widths: 24 76
   :align: left

   * - Option
     - Description
   * - ``after``
     - An ordered list of applications the daemon is to be started after. Applications must be part of the same snap.
   * - ``before``
     - An ordered list of applications the daemon is to be started before. Applications must be part of the same snap.
   * - ``install-mode``
     - Defines whether a freshly installed daemon is started automatically, or whether startup control is deferred to the snap. The snap could then use :external+snap:ref:`snapctl <how-to-guides-manage-snaps-use-snapctl>` with a :external+snap:ref:`hook <reference-development-supported-snap-hooks>`, for instance, or another management agent. Can be ``enable`` (default) or ``disable``.
   * - ``post-stop-command``
     - Sets the command to run from inside the snap after a service stops.
   * - ``refresh-mode``
     - Controls whether a daemon should be restarted during a snap refresh. Can be either ``restart``, ``endure`` (do not restart), or ``ignore-running`` (does not refresh running services to facilitate the refresh app awareness feature). Defaults to ``restart``.
   * - ``reload-command``
     - Defines the command within the snap to be executed when a service needs to be restarted or reloaded after a configuration change, as initiated with the ``snap restart --reload`` command. Example: ``reload-command: sbin/nginx -s reload``
   * - ``restart-condition``
     - Defines when a service should be restarted, using values returned from `systemd service exit status <https://www.freedesktop.org/software/systemd/man/systemd.service.html#Restart=>`__. Can be one of ``[on-failure|on-success|on-abnormal|on-abort|on-watchdog|always|never]``.
   * - ``restart-delay``
     - The delay between service restarts. Defaults to unset. See the systemd.service manual on RestartSec for details. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``.
   * - ``sockets``
     - Requires an activated daemon socket, and works with the :external+snap:ref:`network-bind interface <interfaces-network-interface>` to map a daemon’s socket to a service and activate it.
   * - ``socket-mode``
     - The mode of a socket in octal.
   * - ``start-timeout``
     - Optional time to wait for daemon to start. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``.
   * - ``stop-command``
     - An optional executable command to run before the daemon is stopped. The daemon is not considered stopped until the specified command terminates. This can be used to gracefully handle a daemon stop or restart, such as when a refresh happens, by allowing the daemon to reach a stoppable state first.
   * - ``stop-mode``
     - Defines which `termination signal <https://www.gnu.org/software/libc/manual/html_node/Termination-Signals.html>`__ to use when stopping the daemon. Can be one of either ``sigterm``, ``sigterm-all``, ``sighup``, ``sighup-all``, ``sigusr1``, ``sigusr1-all``, ``sigusr2``, ``sigusr2-all``, ``sigint`` and ``sigint-all``.
   * - ``stop-timeout``
     - The length of time to wait before terminating a service. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``. Termination is via ``SIGTERM`` (and ``SIGKILL`` if that doesn't work).
   * - ``timer``
     - Declares that the service is activated by a timer and that the app must be a daemon. See :external+snap:ref:`Timer string format <interfaces-timer-string-format>` for syntax examples.
   * - ``watchdog-timeout``
     - This value declares the service watchdog timeout. For watchdog to work, the application requires access to the systemd notification socket, which can be declared by specifying the ``daemon-notify`` plug in the ``plugs`` key. Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``.
   * - ``daemon-scope``
     - Scope of the daemon. Accepts ``system`` (default) or ``user`` as values. ``system`` runs regardless of user login, best for when the service needs elevated privileges or must run without an active session. ``user`` starts on login, stops on logout, best for unprivileged services that only need to run within a user session.
