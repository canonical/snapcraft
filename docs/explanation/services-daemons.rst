.. meta::
     :description: Explanation of how background services and daemons are defined inside snaps, their interfacing with systemd and D-Bus, and their confinement.


.. _explanation-services-daemons:

Services and daemons
====================

A snap's executables are either exposed as a command or run as a background service or daemon.

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
