.. 8335.md

.. _snapcraft-app-and-service-metadata:

Snapcraft app and service metadata
==================================

The *app* keys and values in :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` detail the applications and services that a snap wants to expose, including how they’re executed and which resources they can access.

   See :ref:`Snapcraft top-level metadata <snapcraft-top-level-metadata>` and :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>` for details on how apps and parts are configured within a :file:`snapcraft.yaml` file.

apps
----

Type: ``dict``

A map of ``app-names`` representing entry points to run for the snap.

apps.<app-name>
---------------

Type: ``dict``

The name exposed to run a program inside the snap.

If ``<app-name>`` is the same as ``name``, the program will be invoked as ``app-name``. However, if they differ, the program will be exposed as ``<snap-name>.<app-name>``.

Keys for apps
-------------

The following are keys that can be within **apps.** (for example, ``apps.<app-name>.daemon``):

adapter
~~~~~~~

Type: ``enum`` Can be one of the following:

-  ``none`` (Disables the creation of an env variable wrapper.)
-  ``full`` *(default)*

Snapcraft normally creates a wrapper holding common environment variables. Disabling this could be useful for minimal base snaps without a shell, and for statically linked binaries with no use for an environment.


.. _snapcraft-app-and-service-metadata-after:

after
~~~~~

Type: Array of ``string``

A list of applications to be started after ``<app-name>``. Applications must be part of the same snap. The order of the applications in the list has no effect on their launch order.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.

See also `before <snapcraft-app-and-service-metadata-before_>`__.


.. _snapcraft-app-and-service-metadata-autostart:

autostart
~~~~~~~~~

Type: ``string``

Defines the name of the ``.desktop`` file used to start an application with the desktop session.

The desktop file is placed in ``$SNAP_USER_DATA/.config/autostart``, and the application is started using the app’s command wrapper (``<name>.<app>``) plus any argument present in the ``Exec=`` line within the *.desktop* file.

Example: ``autostart: my-chat.desktop``

See :ref:`Autostart desktop files <the-snap-format-autostart>` for an example of both the desktop file and the *Exec* file entry.


.. _snapcraft-app-and-service-metadata-before:

before
~~~~~~

Type: Array of ``string``

An ordered list of applications to be started before ``<app-name>`` . Applications must be part of the same snap.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.

See also `after <snapcraft-app-and-service-metadata-after_>`__.

command
~~~~~~~

Type: ``string``

The command to run inside the snap when ``<app-name>`` is invoked.

The command can be in either a snap runtime’s command path, ``$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin``, or an executable path relative to $SNAP.

If daemon is set, this will be the command to run the service. Only a snap with *classic* confinement can use a relative path because ``PATH`` isn’t modified by a wrapper in classic confinement. See :ref:`Classic confinement <snap-confinement>` for more details.

Examples: ``app-launch`` for an executable placed under ``$SNAP/bin``. With ``classic`` confinement, ``bin/app-launch`` for an executable placed under ``$SNAP/bin``.


.. _snapcraft-app-and-service-metadata-command-chain:

command-chain
~~~~~~~~~~~~~

Type: Array of ``string``

A list of command to be executed, in order, before the command referenced by ``apps.<app-name>.command``.

See `Proposal: support command-chain in apps and hooks <https://snapcraft.io/docs/proposal-support-command-chain-in-apps-and-hooks>`__ for further details.

To ensure that the Snapd distribution user running supports this feature, add the ``command-chain`` value to the ``assumes`` property.

common-id
~~~~~~~~~

Type: ``string``

An identifier to a desktop-id within an external appstream file.

See :ref:`Using external metadata <using-external-metadata>` for more details.

daemon
~~~~~~

Type: ``enum``

Declares that ``<app-name>`` is a system daemon.

Can be one of the following:

- ``simple``: the command is the main process.
- ``oneshot``: the configured command will exit after completion
- ``forking``: the configured command calls ``fork()`` as part of its start-up. The parent process is then expected to exit when start-up is complete
- ``notify``: the command configured will send a signal to systemd to indicate that it’s running.

desktop
~~~~~~~

Type: ``string``

Location of the *.desktop* file.

A path relative to the *prime* directory pointing to a desktop file, commonly used to add an application to the launch menu. Snapcraft will take care of the rest.

Examples: ``usr/share/applications/my-app.desktop`` and ``share/applications/my-app.desktop``

environment
~~~~~~~~~~~

Type: ``dict``

A set of key-value pairs specifying the contents of environment variables.

Key is the environment variable name; Value is the contents of the environment variable.

Example: ``LANG: C.UTF-8``


.. _snapcraft-app-and-service-metadata-extension:

extensions
~~~~~~~~~~

Type: ``list[string] | string`` (*optional*)

Snapcraft extensions enable snap developers to easily incorporate a set of common requirements into a snap, such as those to integrate an application with a desktop environment.

For further details, see :ref:`Snapcraft extensions <snapcraft-extensions>`, and see :ref:`Supported extensions <supported-extensions>` for a full list of supported extensions.

Example: ``[gnome-3-38]``


.. _snapcraft-app-and-service-metadata-install-mode:

install-mode
~~~~~~~~~~~~

Type: ``string``

Defines whether a freshly installed daemon is started automatically, or whether startup control is deferred to the snap.

If a snap was installed prior to the daemon component being added, *install-mode* will determine whether or not the daemon is started automatically when the component is delivered via a snap update.

When disabled, the snap needs to use `snapctl <https://snapcraft.io/docs/using-the-snapctl-tool>`__ with a :ref:`hook <supported-snap-hooks>`, or another management agent, to start the daemon.

Can be either of the following:

-  ``enable``: the daemon is started after being installed.
-  ``disable``: the daemon *will not* be started after installation.

Defaults to ``enable``.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.

listen-stream
~~~~~~~~~~~~~

Type: ``string``

The socket abstract name or socket path.

Sockets should go to a map of <socket-name> to objects which specify the listen-stream and (optionally) the socket-mode.

TCP socket syntax: ``<port>``, ``[::]:<port>``, ``[::1]:<port>`` and ``127.0.0.1:<port>`` UNIX socket syntax: ``$SNAP_DATA/<path>``, ``$SNAP_COMMON/<path>`` and ``@snap.<snap name>.<suffix>``

Example:

.. code:: yaml

         unix:
           listen-stream: $SNAP_COMMON/lxd/unix.socket
           socket-mode: 0660

passthrough
~~~~~~~~~~~

Type: ``type[object]``

``<app-name>`` attributes to pass through to ``snap.yaml`` without snapcraft validation.

See :ref:`Using in-development features <using-in-development-features-in-snapcraft-yaml>` for further details.

plugs
~~~~~

Type: ``list[string]``

Plugs for :ref:`interfaces <interface-management>` to connect to.

``<app-name>`` will make these plug connections when running in ``strict`` ``confinement`` For interfaces that need *attributes*, see top-level :ref:`plugs <snapcraft-top-level-metadata>`.

Example: ``[home, removable-media, raw-usb``]

post-stop-command
~~~~~~~~~~~~~~~~~

Type: ``string``

Runs a command from inside the snap after a service stops.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.

refresh-mode
~~~~~~~~~~~~

Type: ``string``

Controls how the daemon or app should be treated during a snap refresh.

Can be either of the following:

-  ``endure``: the daemon *will not* be restarted during a snap refresh.
-  ``restart``: the daemon *will* be restarted during a snap refresh.
-  ``ignore-running``: the app *will not* block a snap refresh (can only be set for apps).

Defaults to ``restart``.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.

restart-condition
~~~~~~~~~~~~~~~~~

Type: ``enum``

Condition to restart the daemon under.

Defaults to ``on-failure``. Other values are ``[on-failure|on-success|on-abnormal|on-abort|always|never]``. Refer to `systemd.service manual <https://www.freedesktop.org/software/systemd/man/systemd.service.html#Restart=>`__ for details.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.

slots
~~~~~

Type: ``list[string]``

Slots for :ref:`interfaces <interface-management>` to connect to.

``<app-name>`` will make these slot connections when running in ``strict`` confinement only. For interfaces that need *attributes*, see top-level :ref:`slots <snapcraft-top-level-metadata>`.

Example: ``[home, removable-media, raw-usb``]

sockets
~~~~~~~

Type: ``dict``

Maps a daemon’s sockets to services and activates them.

Requires an activated daemon socket.

Requires ``apps.<app-name>.plugs`` to declare the ``network-bind`` plug.

socket-mode
~~~~~~~~~~~

Type: ``integer``

The mode of a socket in *octal*.

stop-command
~~~~~~~~~~~~

Type: ``string``

The path to a command inside the snap to run to stop the service.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.

stop-timeout
~~~~~~~~~~~~

Type: ``string``

The length of time to wait before terminating a service.

Time duration units can be ``10ns``, ``10us``, ``10ms``, ``10s``, ``10m``. Termination is via ``SIGTERM`` (and ``SIGKILL`` if that doesn’t work).

timer
-----

Type: ``timer-string``

Schedules when, or how often, to run a service or command.

See `Timer string format <https://snapcraft.io/docs/timer-string-format>`__ for further details on the required syntax.

Requires ``daemon`` to be set in the *app* metadata. See :ref:`Services and daemons <services-and-daemons>` for details.
