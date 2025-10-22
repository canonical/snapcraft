# Copyright 2025 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Definition of an app model."""

import re
from typing import Any, Literal

import pydantic
from craft_application import models
from craft_application.models.constraints import UniqueList

from snapcraft.extensions.registry import get_extension_names
from snapcraft.models._validators import validate_command_chain


class Socket(models.CraftBaseModel):
    """Snapcraft app socket definition."""

    listen_stream: int | str = pydantic.Field(
        description="The socket's abstract name or socket path.",
        examples=["$SNAP_COMMON/lxd/unix.socket", "80"],
    )
    """The socket's abstract name or socket path.

    TCP and UNIX sockets are supported.
    """

    socket_mode: int | None = pydantic.Field(
        default=None,
        description="The mode or permissions of the socket in octal.",
        examples=["0660"],
    )

    @pydantic.field_validator("listen_stream")
    @classmethod
    def _validate_list_stream(cls, listen_stream: int | str) -> int | str:
        if isinstance(listen_stream, int):
            if listen_stream < 1 or listen_stream > 65535:
                raise ValueError(
                    f"{listen_stream!r} is not an integer between 1 and 65535 (inclusive)."
                )
        elif isinstance(listen_stream, str):
            if not listen_stream.startswith("@snap.") and not re.match(
                r"^[A-Za-z0-9/._#:$-]*$", listen_stream
            ):
                raise ValueError(
                    f"{listen_stream!r} is not a valid socket path (e.g. /tmp/mysocket.sock)."
                )

        return listen_stream


class App(models.CraftBaseModel):
    """Snapcraft project app definition."""

    command: str = pydantic.Field(
        description="The command to run inside the snap when the app is invoked.",
        examples=["bin/foo-app"],
    )
    """The command to run inside the snap when the app is invoked.

    The command can be in either a snap runtime's command path,
    ``$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin``, or an executable path
    relative to ``$SNAP``.

    The command must consist only of alphanumeric characters, spaces, and the following
    special characters: ``/, ., _, #, :, $, -``. If other characters are required, a
    wrapper script should be used for the command.

    If the ``daemon`` is set, this will be the command to run the service. Only a snap
    with classic confinement can use a relative path because PATH isn't modified by a
    wrapper in classic confinement. See `Classic confinement
    <https://documentation.ubuntu.com/snapcraft/stable/explanation/classic-confinement>`_
    for more details.
    """

    autostart: str | None = pydantic.Field(
        default=None,
        description="The desktop file used to start an app when the desktop environment starts.",
        examples=["foo-app.desktop"],
    )
    """The desktop file used to start an app when the desktop environment starts.

    The desktop file is placed in ``$SNAP_USER_DATA/.config/autostart`` and the app
    is launched by the app's command wrapper (``<name>.<app>``) plus any argument
    present in the ``Exec=`` line in the ``.desktop`` file when the desktop
    environment is started.

    See `Autostart desktop files
    <https://snapcraft.io/docs/the-snap-format#heading--autostart>`_ for an
    example of both the desktop file and the ``Exec`` file entry.
    """

    common_id: str | None = pydantic.Field(
        default=None,
        description="The identifier to a desktop ID within an external appstream file.",
        examples=["org.canonical.foo"],
    )
    """The identifier to a desktop ID within an external appstream file.

    See `Configure package information
    <https://documentation.ubuntu.com/snapcraft/stable/how-to/crafting/configure-package-information>`_
    for more information.
    """

    bus_name: str | None = pydantic.Field(
        default=None,
        description="The bus name that the application or service exposes through D-Bus.",
        examples=["org.bluez"],
    )

    desktop: str | None = pydantic.Field(
        default=None,
        description="The desktop file used to start an app.",
        examples=["my-app.desktop"],
    )
    """The desktop file used to start an app.

    See `Configure package information
    <https://documentation.ubuntu.com/snapcraft/stable/how-to/crafting/configure-package-information>`_
    for more information.
    """

    completer: str | None = pydantic.Field(
        default=None,
        description="The name of the bash completion script for the app.",
        examples=["bash-complete.sh"],
    )

    stop_command: str | None = pydantic.Field(
        default=None,
        description="The command to run to stop the service.",
        examples=["bin/foo-app --halt"],
    )
    """The command to run to stop the service.

    Requires the ``daemon`` key to be specified for the app.

    This allows a daemon to gracefully stop or restart, such as when a snap refresh
    occurs.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    post_stop_command: str | None = pydantic.Field(
        default=None,
        description="The command to run after the service is stopped.",
        examples=["bin/logrotate --force"],
    )
    """The command to run after the service is stopped.

    Requires the ``daemon`` key to be specified for the app.

    This allows a daemon to gracefully stop or restart, such as when a snap
    refresh occurs.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    start_timeout: str | None = pydantic.Field(
        default=None,
        description="The maximum amount of time to wait for the service to start.",
        examples=["10s", "2m"],
    )
    """The maximum amount of time to wait for the service to start.

    If the service does not start before the ``start-timeout`` elapses, then
    snapd will take further action based on the ``restart-condition`` key in
    the app.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    stop_timeout: str | None = pydantic.Field(
        default=None,
        description="The maximum amount of time to wait for the service to stop.",
        examples=["10s", "2m"],
    )
    """The maximum amount of time to wait for the service to stop.

    If the service does not stop before the ``stop-timeout`` elapses, then snapd
    will send a ``SIGTERM`` signal.  If the service still does not stop, snapd
    will send a ``SIGKILL`` signal.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    watchdog_timeout: str | None = pydantic.Field(
        default=None,
        description="The maximum amount of time the service can run without sending a heartbeat to the watchdog.",
        examples=["10s", "2m"],
    )
    """The maximum amount of time the service can run without sending a
    heartbeat to the watchdog.

    For the watchdog to work, the application must have access to the
    ``systemd`` notification socket by specifying ``daemon-notify`` plug in the
    apps ``plugs`` definition.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    reload_command: str | None = pydantic.Field(
        default=None,
        description="The command to run to restart the service.",
        examples=["bin/foo-app --restart"],
    )
    """The command to run to restart the service.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    restart_delay: str | None = pydantic.Field(
        default=None,
        description="The time to wait between service restarts.",
        examples=["10s", "2m"],
    )
    """The time to wait between service restarts.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    timer: str | None = pydantic.Field(
        default=None,
        description="The time or schedule to run a service.",
        examples=[
            "23:00",
            "00:00-24:00/24",
            "mon,10:00,,fri,15:00",
        ],
    )
    """The time or schedule to run a service.

    The timer field uses a flexible syntax to schedule when a service should
    run.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    daemon: Literal["simple", "forking", "oneshot", "notify", "dbus"] | None = (
        pydantic.Field(
            default=None,
            description="Configures the app as a service, and sets its runtime and notification behavior.",
            examples=["simple", "oneshot"],
        )
    )
    """Configures the app as a service, and sets its runtime and
    notification behavior.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``simple``
          - Run as long as the service is active. This is the most common option.
        * - ``forking``
          - The app's ``command`` will call ``fork()`` as part of its start-up
            and the parent process is expected to exit when start-up is
            complete. This is used to support legacy fork-based Unix daemons.
        * - ``oneshot``
          - Run once and exit after completion, notifying systemd. After
            completion, the daemon is still considered active and running.
        * - ``notify``
          - Allows the service to be managed by systemd. Requires the service to
            send signals to the systemd notification socket by specifying
            ``daemon-notify`` in the app's ``plugs`` definition.
        * - ``dbus``
          - ``Registers a D-Bus name to notify systemd. Requires ``bus-name`` or
            ``activates-on`` to be specified.

    """

    after: UniqueList[str] = pydantic.Field(
        default_factory=list,
        description="The ordered list of apps that the service runs after it launches.",
        examples=["[foo-app, bar-app]"],
    )
    """The ordered list of apps that the service runs after it launches.

    The apps must be part of the same snap.

    Requires the ``daemon`` key to be specified for the app. Apps in the
    ``after`` key must also specify the ``daemon`` key.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    before: UniqueList[str] = pydantic.Field(
        default_factory=list,
        description="The ordered list of apps that the service runs before it launches.",
        examples=["[baz-app, quz-app]"],
    )
    """The ordered list of apps that the service runs before it launches.

    The apps must be part of the same snap.

    Requires the ``daemon`` key to be specified for the app. Apps in the
    ``before`` key must also specify the ``daemon`` key.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    refresh_mode: Literal["endure", "restart", "ignore-running"] | None = (
        pydantic.Field(
            default=None,
            description="Determines how the service should restart when the snap refreshes.",
            examples=["restart"],
        )
    )
    """Determines how the service should restart when the snap refreshes.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``restart``
          - Restart the service when the snap is refreshed. Default.
        * - ``endure``
          - Do not restart the service when the snap is refreshed.
        * - ``ignore-running``
          - Do not refresh the snap if the service is running.

    """

    stop_mode: (
        Literal[
            "sigterm",
            "sigterm-all",
            "sighup",
            "sighup-all",
            "sigusr1",
            "sigusr1-all",
            "sigusr2",
            "sigusr2-all",
            "sigint",
            "sigint-all",
        ]
        | None
    ) = pydantic.Field(
        default=None,
        description="The signal to send when stopping the service.",
        examples=["sigterm"],
    )
    """The signal to send when stopping the service.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    restart_condition: (
        Literal[
            "on-success",
            "on-failure",
            "on-abnormal",
            "on-abort",
            "on-watchdog",
            "always",
            "never",
        ]
        | None
    ) = pydantic.Field(
        default=None,
        description="The conditions that cause the service to restart.",
        examples=["on-failure"],
    )
    """The conditions that cause the service to restart.

    The conditions for ``restart-condition`` match those defined by ``systemd``.
    See the `systemd manual
    <https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html#Restart=>`_
    for information on what exit codes will trigger a restart for each
    condition.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    install_mode: Literal["enable", "disable"] | None = pydantic.Field(
        default=None,
        description="Whether snapd can automatically start the service when the snap is installed.",
        examples=["enable"],
    )
    """Whether snapd can automatically start the service when the snap is installed.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``enable``
          - The service is started when the snap is installed. Additionally, if the snap
            was installed without a service, then the snap is refreshed to include a
            service. This will start the service too.
        * - ``disable``
          - The service is not automatically started. Instead, the service will be
            started with `craftctl
            <https://documentation.ubuntu.com/snapcraft/stable/how-to/crafting/customize-lifecycle-steps-and-part-variables>`_
            and another management agent, which is most commonly a `hook
            <https://documentation.ubuntu.com/snapcraft/stable/reference/hooks>`_.

    """

    slots: UniqueList[str] | None = pydantic.Field(
        default=None,
        description="The list of slots that the app provides.",
        examples=["[dbus-daemon]"],
    )
    """The list of slots that the app provides.

    Slot connections are only made when the snap is running in ``strict``
    confinement.

    Slots are used to define what code and data can be shared with other snaps.

    See the `content interface <https://snapcraft.io/docs/content-interface>`_
    for more information about plugs and slots.
    """

    plugs: UniqueList[str] | None = pydantic.Field(
        default=None,
        description="The interfaces that the app can connect to.",
        examples=["[home, removable-media]"],
    )
    """The list of interfaces that the app can connect to.

    See the `content interface <https://snapcraft.io/docs/content-interface>`_
    for more information about plugs and slots.
    """

    aliases: UniqueList[str] | None = pydantic.Field(
        default=None,
        description="The aliases that can be used to run the app.",
        examples=["[my-app]"],
    )
    """The aliases that can be used to run the app.

    See `Commands and aliases <https://snapcraft.io/docs/commands-and-aliases>`_
    for more information.
    """

    environment: dict[str, str] | None = pydantic.Field(
        default=None,
        description="The runtime environment variables.",
        examples=[
            "{PYTHONPATH: $SNAP/usr/lib/python3/dist-packages, DISABLE_WAYLAND: 1}"
        ],
    )
    """The runtime environment variables.

    To set an environment variable for all apps, use the top-level
    ``environment`` key.
    """

    command_chain: list[str] = pydantic.Field(
        default_factory=list,
        description="The ordered list of commands to run before the app's command runs.",
        examples=["[bin/alsa-launch, bin/desktop-launch]"],
    )
    """The ordered list of commands to run before the app's command
    runs.

    Command chains are useful to run setup scripts before running an app.
    """

    sockets: dict[str, Socket] | None = pydantic.Field(
        default=None,
        description="The sockets used to activate an app.",
        examples=[
            "{my-socket: {listen-stream: $SNAP_COMMON/lxd/unix.socket, socket-mode: 0660}}"
        ],
    )
    """The sockets used to activate an app.

    Requires the ``network-bind`` interface in the app's ``plug`` key.

    This value is used for services that are activated by a connection to a
    socket.
    """

    daemon_scope: Literal["system", "user"] | None = pydantic.Field(
        default=None,
        description="Determines whether the service is run on a system or user instance of systemd.",
        examples=["user"],
    )
    """Determines whether the service is run on a system or a user instance of
    systemd.

    See `Enabling user daemons
    <https://forum.snapcraft.io/t/enabling-user-daemons-and-d-bus-activation/22318>`_
    for more information on how snapd manages daemon scope.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``system``
          - Run the service under the system instance of systemd.
        * - ``user``
          - Run the service under a user-session instance of systemd.
    """

    activates_on: UniqueList[str] | None = pydantic.Field(
        default=None,
        description="The slots exposed by the snap to activate the service with D-Bus.",
        examples=["gnome-shell-dbus"],
    )
    """The slots exposed by the snap to activate the service with D-Bus.

    This is useful for services that are activated by other applications or
    services.

    See `D-Bus activation
    <https://forum.snapcraft.io/t/enabling-user-daemons-and-d-bus-activation/22318>`_
    for more information on how snapd activates services with D-Bus.

    Requires the ``daemon`` key to be specified for the app.

    See the `daemon key
    <https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.%3Capp-name%3E.daemon>`_
    reference for more information.
    """

    passthrough: dict[str, Any] | None = pydantic.Field(
        default=None,
        description="The attributes to pass to the snap's metadata file for the app.",
        examples=["{daemon: complex}"],
    )
    """The attributes to pass to the snap's metadata file for the app.

    Attributes to passthrough to snap.yaml without validation from Snapcraft.
    This is useful for early testing of a new feature in snapd that isn't
    supported yet by Snapcraft.

    To pass a value for the entire project, see the top-level ``passthrough``
    key.

    See `Using development features in Snapcraft
    <https://snapcraft.io/docs/using-in-development-features>`_ for more
    details.
    """

    extensions: UniqueList[str] | None = pydantic.Field(
        default=None,
        description="The extensions to add to the project.",
        examples=[["gnome"], ["ros2-humble"]],
    )
    """The extensions to add to the project

    Snapcraft extensions enable you to easily incorporate a set of common
    requirements into a snap, saving time spent replicating the same general
    requirements shared across similar apps.

    Extensions instruct Snapcraft to operate on the project file prior to
    build, causing it to add any needed scaffolding and boilerplate keys to
    enable a particular technology. The procedure is merely a postprocessor
    acting on the project’s keys in memory – the actual project file on
    disk is unaffected.

    For guidance on specific extensions, see `Extensions
    <https://documentation.ubuntu.com/snapcraft/stable/how-to/extensions/#how-to-extensions>`_.
    """

    @pydantic.field_validator("autostart")
    @classmethod
    def _validate_autostart_name(cls, name: str) -> str:
        if not re.match(r"^[A-Za-z0-9. _#:$-]+\.desktop$", name):
            raise ValueError(
                f"{name!r} is not a valid desktop file name (e.g. myapp.desktop)"
            )

        return name

    @pydantic.field_validator(
        "command", "stop_command", "post_stop_command", "reload_command", "bus_name"
    )
    @classmethod
    def _validate_apps_section_content(cls, command: str) -> str:
        # Find any invalid characters in the field.
        # The regex below is derived from snapd's validator code.
        # https://github.com/canonical/snapd/blob/0706e2d0b20ae2bf030863f142b8491b66e80bcb/snap/validate.go#L756
        if not re.match(r"^[A-Za-z0-9/. _#:$-]*$", command):
            message = "App commands must consist of only alphanumeric characters, spaces, and the following characters: / . _ # : $ -"
            raise ValueError(message)

        return command

    @pydantic.field_validator(
        "start_timeout", "stop_timeout", "watchdog_timeout", "restart_delay"
    )
    @classmethod
    def _validate_time(cls, timeval: str) -> str:
        if not re.match(r"^[0-9]+(ns|us|ms|s|m)*$", timeval):
            raise ValueError(f"{timeval!r} is not a valid time value")

        return timeval

    @pydantic.field_validator("command_chain")
    @classmethod
    def _validate_command_chain(cls, command_chains: list[str]) -> list[str]:
        return validate_command_chain(command_chains)

    @pydantic.field_validator("aliases")
    @classmethod
    def _validate_aliases(cls, aliases: list[str]) -> list[str]:
        for alias in aliases:
            if not re.match(r"^[a-zA-Z0-9][-_.a-zA-Z0-9]*$", alias):
                raise ValueError(
                    f"{alias!r} is not a valid alias. Aliases must be strings, begin with an ASCII "
                    "alphanumeric character, and can only use ASCII alphanumeric characters and "
                    "the following special characters: . _ -"
                )

        return aliases

    @pydantic.field_validator("extensions")
    @classmethod
    def _validate_extensions(cls, extensions: list[str]) -> list[str]:
        valid_extensions = set(get_extension_names())
        invalid_extensions = [ext for ext in extensions if ext not in valid_extensions]
        if invalid_extensions:
            raise ValueError(
                f"The following extensions are invalid: {invalid_extensions!r}.\n"
                f"Valid extensions are {valid_extensions!r}."
            )
        return extensions
