# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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

"""Project file definition and helpers."""

from __future__ import annotations

import re
import shlex
import textwrap
from typing import TYPE_CHECKING, Annotated, Any, Literal, cast

import pydantic
from annotated_types import Ge, Le
from craft_application import models
from craft_application.errors import CraftValidationError
from craft_application.models import (  # noqa: TC002 (typing-only-third-party-import) # pydantic needs to import types at runtime for validation
    SummaryStr,
    VersionStr,
)
from craft_application.models.constraints import (
    SingleEntryDict,
    SingleEntryList,
    UniqueList,
)
from craft_application.util import humanize_list
from craft_cli import emit
from craft_grammar.models import (  # noqa: TC002 (typing-only-third-party-import) # pydantic needs to import types at runtime for validation
    Grammar,
)
from craft_platforms import DebianArchitecture
from pydantic import ConfigDict, PrivateAttr, StringConstraints, error_wrappers
from pydantic.json_schema import (
    SkipJsonSchema,  # noqa: TC002 (typing-only-third-party-import) # pydantic needs to import types at runtime for validation
)
from typing_extensions import Self, override

from snapcraft import utils
from snapcraft.const import (
    SUPPORTED_ARCHS,
    ProjectType,
    StableBase,
)
from snapcraft.elf.elf_utils import get_arch_triplet
from snapcraft.errors import ProjectValidationError
from snapcraft.extensions.registry import get_extension_names
from snapcraft.providers import SNAPCRAFT_BASE_TO_PROVIDER_BASE
from snapcraft.utils import get_effective_base

if TYPE_CHECKING:
    from collections.abc import Iterable, Mapping

    from craft_application.models.project import Part
    from craft_providers import bases

ProjectName = Annotated[str, StringConstraints(max_length=40)]


def _validate_command_chain(command_chains: list[str]) -> list[str]:
    """Validate command_chain."""
    for command_chain in command_chains:
        if not re.match(r"^[A-Za-z0-9/._#:$-]*$", command_chain):
            raise ValueError(
                f"{command_chain!r} is not a valid command chain. Command chain entries must "
                "be strings, and can only use ASCII alphanumeric characters and the following "
                "special characters: / . _ # : $ -"
            )
    return command_chains


def validate_architectures(
    architectures: list[Architecture | str],
) -> list[Architecture]:
    """Expand and validate architecture data.

    Validation includes:
        - The list cannot be a combination of strings and Architecture objects.
        - The same architecture cannot be defined in multiple `build-for` fields,
          even if the implicit values are used to define `build-for`.
        - Only one architecture can be defined in the `build-for` list.
        - The `all` key is properly used. (see `_validate_architectures_all_key()`)

    :raise ValueError: If architecture data is invalid.
    """
    # validate strings and Architecture objects are not mixed
    if not (
        all(isinstance(architecture, str) for architecture in architectures)
        or all(isinstance(architecture, Architecture) for architecture in architectures)
    ):
        raise ValueError(
            f"every item must either be a string or an object for {architectures!r}"
        )

    expanded_archs = _expand_architectures(architectures)

    # validate `build_for` after expanding data
    if any(len(cast(UniqueList[str], arch.build_for)) > 1 for arch in expanded_archs):
        raise ValueError("only one architecture can be defined for 'build-for'")

    _validate_architectures_all_key(expanded_archs)

    if len(expanded_archs) > 1:
        # validate multiple uses of the same architecture
        unique_build_fors = set()
        for element in expanded_archs:
            for arch in cast(UniqueList[str], element.build_for):
                if arch in unique_build_fors:
                    raise ValueError(
                        f"multiple items will build snaps that claim to run on {arch}"
                    )
                unique_build_fors.add(arch)

    # validate architectures are supported
    if len(expanded_archs):
        for element in expanded_archs:
            for arch in cast(UniqueList[str], element.build_for) + cast(
                UniqueList[str], element.build_on
            ):
                if arch != "all" and arch not in utils.get_supported_architectures():
                    supported_archs = utils.humanize_list(
                        utils.get_supported_architectures(), "and"
                    )
                    raise ValueError(
                        f"Architecture {arch!r} is not supported. Supported "
                        f"architectures are {supported_archs}."
                    )

    return expanded_archs


def _expand_architectures(
    architectures: list[Architecture | str],
) -> list[Architecture]:
    """Expand architecture data.

    Expansion to fully-defined Architecture objects includes the following:
        - strings (shortform notation) are converted to Architecture objects
        - `build-on` and `build-for` strings are converted to single item lists
        - Empty `build-for` fields are implicitly set to the same architecture used in `build-on`

    :param architectures: The architecture data to expand.

    :returns: A list of expanded architecture objects.
    """
    result: list[Architecture] = []
    for arch in architectures:
        # convert strings into Architecture objects
        if isinstance(arch, str):
            build_on = build_for = [arch]
        else:
            # convert strings to lists
            if isinstance(arch.build_on, str):
                build_on = [arch.build_on]
            else:
                build_on = arch.build_on

            if isinstance(arch.build_for, str):
                build_for = [arch.build_for]
            elif isinstance(arch.build_for, list):
                build_for = arch.build_for
            # implicitly set build_for from build_on
            else:
                build_for = build_on

        result.append(
            Architecture(
                build_on=cast(UniqueList[str], build_on),
                build_for=cast(UniqueList[str], build_for),
            )
        )

    return result


def _validate_architectures_all_key(architectures: list[Architecture]) -> None:
    """Validate `all` key is properly used.

    Validation rules:
    - `all` cannot be used to `build-on`
    - If `all` is used for `build-for`, no other architectures can be defined
      for `build-for`.

    :raise ValueError: if `all` key isn't properly used.
    """
    # validate use of `all` inside each build-on list
    for architecture in architectures:
        if "all" in architecture.build_on:
            raise ValueError("'all' cannot be used for 'build-on'")

    # validate use of `all` across all items in architecture list
    if len(architectures) > 1:
        if any(
            "all" in cast(UniqueList[str], architecture.build_for)
            for architecture in architectures
        ):
            raise ValueError(
                "one of the items has 'all' in 'build-for', but there are"
                f" {len(architectures)} items: upon release they will conflict."
                "'all' should only be used if there is a single item"
            )


def apply_root_packages(yaml_data: dict[str, Any]) -> None:
    """Create a new part with root level attributes, in place.

    Root level attributes such as build-packages and build-snaps
    are known to Snapcraft but not Craft Parts. Create a new part
    "snapcraft/core" with these attributes and apply it to the
    current yaml_data.
    """
    if "build-packages" not in yaml_data and "build-snaps" not in yaml_data:
        return

    yaml_data.setdefault("parts", {})
    yaml_data["parts"]["snapcraft/core"] = {"plugin": "nil"}

    if "build-packages" in yaml_data:
        yaml_data["parts"]["snapcraft/core"]["build-packages"] = yaml_data.pop(
            "build-packages"
        )

    if "build-snaps" in yaml_data:
        yaml_data["parts"]["snapcraft/core"]["build-snaps"] = yaml_data.pop(
            "build-snaps"
        )


def _validate_version_name(version: str, model_name: str) -> None:
    """Validate a version complies to the naming convention.

    :param version: version string to validate
    :param model_name: name of the model that contains the version

    :raises ValueError: if the version contains invalid characters
    """
    if version and not re.match(
        r"^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$", version
    ):
        raise ValueError(
            f"Invalid version '{version}': {model_name.title()} versions consist of "
            "upper- and lower-case alphanumeric characters, as well as periods, colons, "
            "plus signs, tildes, and hyphens. They cannot begin with a period, colon, "
            "plus sign, tilde, or hyphen. They cannot end with a period, colon, or "
            "hyphen"
        )


def validate_name(*, name: str, field_name: str) -> str:
    """Validate a name.

    :param name: The name to validate.
    :param field_name: The name of the field being validated.

    :returns: The validated name.
    """
    if not re.match(r"^[a-z0-9-]*[a-z][a-z0-9-]*$", name):
        raise ValueError(
            f"{field_name} names can only use lowercase alphanumeric "
            "and hyphens and must have at least one letter"
        )

    if name.startswith("-"):
        raise ValueError(f"{field_name} names cannot start with a hyphen")

    if name.endswith("-"):
        raise ValueError(f"{field_name} names cannot end with a hyphen")

    if "--" in name:
        raise ValueError(f"{field_name} names cannot have two hyphens in a row")

    return name


def _validate_component(name: str) -> str:
    """Validate a component name.

    :param name: The component name to validate.

    :returns: The validated component name.
    """
    if name.startswith("snap-"):
        raise ValueError(
            "component names cannot start with the reserved prefix 'snap-'"
        )
    return validate_name(name=name, field_name="component")


def _get_partitions_from_components(
    components_data: dict[str, Any] | None,
) -> list[str] | None:
    """Get a list of partitions based on the project's components.

    :returns: A list of partitions formatted as ['default', 'component/<name>', ...]
    or None if no components are defined.
    """
    if components_data:
        return ["default", *[f"component/{name}" for name in components_data.keys()]]

    return None


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


class Lint(models.CraftBaseModel):
    """Linter configuration.

    :ivar ignore: A list describing which files should have issues ignored for given linters.
        The items in the list can be either:
        - a string, which must be the name of one of the known linters (see below). All issues
          from this linter will be ignored.
        - a dict containing exactly one key, which must be the name of one of the known linters.
            The value is then a list of strings corresponding to the filenames/patterns that
            should be ignored for that linter.
        The "known" linter names are the keys in :ref:`LINTERS`
    """

    ignore: list[str | dict[str, list[str]]] = pydantic.Field(
        description="Linters or files to skip when linting.",
        examples=["{ignore: [classic, library: [usr/lib/**/libfoo.so*]]}"],
    )

    # A private field to simplify lookup.
    _lint_ignores: dict[str, list[str]] = PrivateAttr(default_factory=dict)

    def __eq__(self, other: object):
        """Compare two Lint objects and ignore private attributes."""
        if not isinstance(other, Lint):
            return False
        return self.ignore == other.ignore

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        for item in self.ignore:
            if isinstance(item, str):
                self._lint_ignores[item] = []
            else:
                if len(item) != 1:
                    raise ValueError("Expected exactly one key in lint ignore entry.")
                name, files = list(item.items())[0]
                self._lint_ignores[name] = files

    def all_ignored(self, linter_name: str) -> bool:
        """Whether all issues for linter `lint_name` should be ignored."""
        return (
            linter_name in self._lint_ignores
            and len(self._lint_ignores[linter_name]) == 0
        )

    def ignored_files(self, linter_name: str) -> list[str]:
        """Get a list of filenames/patterns to ignore for `lint_name`.

        Since the main usecase for this method is a for-loop with `fnmatch()`, it will
        return `['*']` when *all* files should be ignored for `linter_name`, and `[]`
        when *no* files should be ignored.
        """
        if linter_name not in self._lint_ignores:
            return []

        if self.all_ignored(linter_name):
            return ["*"]

        return self._lint_ignores[linter_name]


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
    special characters: ``/``, ``.``, ``_``, ``#``, ``:``, ``$``, ``-``. If other
    characters are required, a wrapper script should be used for the command.

    If the ``daemon`` key is set, this command runs the service. Only
    classically-confined snaps can use a relative path because ``PATH`` isn't modified
    by a wrapper in classic confinement. See :ref:`explanation-classic-confinement` for
    more details.
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

    See :ref:`how-to-configure-package-information` for details.
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

    See :ref:`how-to-configure-package-information` for details.
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

    See the :ref:`daemon key <App.daemon>` reference for more information.
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

    See the :ref:`daemon key <App.daemon>` reference for more information.

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

    See the :ref:`daemon key <App.daemon>` reference for more information.

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

    See the :ref:`daemon key <App.daemon>` reference for more information.
    """

    watchdog_timeout: str | None = pydantic.Field(
        default=None,
        description="The maximum amount of time the service can run without sending a heartbeat to the watchdog.",
        examples=["10s", "2m"],
    )
    """The maximum amount of time the service can run without sending a
    heartbeat to the watchdog.

    For the watchdog to work, the application must have access to the systemd
    notification socket by declaring ``daemon-notify`` in the app's ``plugs``
    definition.

    Requires the ``daemon`` key to be specified for the app.

    See the :ref:`daemon key <App.daemon>` reference for more information.
    """

    reload_command: str | None = pydantic.Field(
        default=None,
        description="The command to run to restart the service.",
        examples=["bin/foo-app --restart"],
    )
    """The command to run to restart the service.

    Requires the ``daemon`` key to be specified for the app.

    See the :ref:`daemon key <App.daemon>` reference for more information.
    """

    restart_delay: str | None = pydantic.Field(
        default=None,
        description="The time to wait between service restarts.",
        examples=["10s", "2m"],
    )
    """The time to wait between service restarts.

    Requires the ``daemon`` key to be specified for the app.

    See the :ref:`daemon key <App.daemon>` reference for more information.
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

    See the :ref:`daemon key <App.daemon>` reference for more information.
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
          - Registers a D-Bus name to notify systemd. Requires ``bus-name`` or
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

    See the :ref:`daemon key <App.daemon>` reference for more information.
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

    See the :ref:`daemon key <App.daemon>` reference for more information.
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

    See the :ref:`daemon key <App.daemon>` reference for more information.

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

    See the :ref:`daemon key <App.daemon>` reference for more information.
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

    The conditions for ``restart-condition`` match those defined by systemd.
    See the `systemd manual
    <https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html#Restart=>`_
    for information on what exit codes will trigger a restart for each
    condition.

    Requires the ``daemon`` key to be specified for the app.

    See the :ref:`daemon key <App.daemon>` reference for more information.
    """

    success_exit_status: list[Annotated[int, Ge(1), Le(255)]] | None = pydantic.Field(
        default=None,
        description="The list of additional successful exit statuses that the service can return.",
        examples=["[42, 250]"],
    )
    """The list of additional successful exit statuses that the service can return.

    This key corresponds to the `SuccessExitStatus= <https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html#SuccessExitStatus=>`_ directive in a systemd service configuration.

    Requires the ``daemon`` key to be set for the app.
    """

    install_mode: Literal["enable", "disable"] | None = pydantic.Field(
        default=None,
        description="Whether snapd can automatically start the service when the snap is installed.",
        examples=["enable"],
    )
    """Whether snapd can automatically start the service when the snap is installed.

    Requires the ``daemon`` key to be specified for the app.

    See the :ref:`daemon key <App.daemon>` reference for more information.

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
            started with :ref:`craftctl <how-to-customize-the-build-and-part-variables>`.
            and another management agent, which is most commonly a :ref:`hooks
            <reference-hooks>`.

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

    See the :ref:`daemon key <App.daemon>` reference for more information.

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

    See the :ref:`daemon key <App.daemon>` reference for more information.
    """

    passthrough: dict[str, Any] | None = pydantic.Field(
        default=None,
        description="The attributes to pass to the snap's metadata file for the app.",
        examples=["{daemon: complex}"],
    )
    """The attributes to pass to the snap's metadata file for the app.

    These attributes are passed to the ``snap.yaml`` file without validation from Snapcraft.
    This is useful for early testing of a new feature in snapd that isn't supported yet
    by Snapcraft.

    To pass a value for the entire project, see the top-level :ref:`passthrough key
    <Project.passthrough>`.

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

    For guidance on specific extensions, see :ref:`how-to-extensions`."""

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
        return _validate_command_chain(command_chains)

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

    @pydantic.field_validator("success_exit_status")
    @classmethod
    def _validate_success_exit_status(
        cls, status_list: list[int] | None
    ) -> list[int] | None:
        if status_list == []:
            return None
        return status_list


class Hook(models.CraftBaseModel):
    """Snapcraft project hook definition."""

    command_chain: list[str] = pydantic.Field(
        default_factory=list,
        description="The ordered list of commands to run before the hook runs.",
        examples=["[bin/alsa-launch, bin/desktop-launch]"],
    )
    """The ordered list of commands to run before the hook runs.

    Command chains are useful to run setup scripts before running a hook.
    """

    environment: dict[str, str | None] | None = pydantic.Field(
        default=None,
        description="The environment variables for the hook.",
        examples=["{PYTHONPATH: /custom/path/:$PYTHON_PATH, DISABLE_WAYLAND: 1}"],
    )

    plugs: UniqueList[str] | None = pydantic.Field(
        default=None,
        description="The list of interfaces that the hook can connect to.",
        examples=["[home, removable-media]"],
    )
    """The list of interfaces that the hook can connect to.

    See the `content interface <https://snapcraft.io/docs/content-interface>`_ for more
    information about plugs and slots.
    """

    passthrough: dict[str, Any] | None = pydantic.Field(
        default=None,
        description="The attributes to pass to the snap's metadata file for the hook.",
        examples=["{daemon: complex}"],
    )
    """The attributes to pass to the snap's metadata file for the hook.

    These attributes are passed to the ``snap.yaml`` file without validation from
    Snapcraft. This is useful for early testing of a new feature in snapd that isn't
    supported yet by Snapcraft.

    To pass a value for the entire project, see the top-level :ref:`passthrough key
    <Project.passthrough>`.

    See `Using development features in Snapcraft
    <https://snapcraft.io/docs/using-in-development-features>`_ for more details.
    """

    @pydantic.field_validator("command_chain")
    @classmethod
    def _validate_command_chain(cls, command_chains: list[str]) -> list[str]:
        return _validate_command_chain(command_chains)

    @pydantic.field_validator("plugs")
    @classmethod
    def _validate_plugs(cls, plugs: list[str]) -> list[str]:
        if not plugs:
            raise ValueError("'plugs' field cannot be empty.")
        return plugs


class Architecture(models.CraftBaseModel, extra="forbid"):
    """Snapcraft project architecture definition."""

    build_on: str | UniqueList[str] = pydantic.Field(
        description="The architectures to build the snap on.",
        examples=["arm64", "[amd64, riscv64]"],
    )
    """The architectures to build the snap on.

    Must be paired with a ``build-for`` entry.

    This list must contain unique values. If the value is a string, it will be parsed
    into a single-entry list at runtime.
    """

    build_for: str | UniqueList[str] | None = pydantic.Field(
        default=None,
        description="The target architecture for the build.",
        examples=["amd64", "[riscv64]"],
    )
    """The target architecture for the build.

    Must be paired with a ``build-on`` entry.

    If the value is a string, it will be parsed into a single-entry list at runtime.
    """


class ContentPlug(models.CraftBaseModel):
    """Snapcraft project content plug definition."""

    content: str | None = pydantic.Field(
        default=None,
        description="The name for the content type.",
        examples=["themes"],
    )
    """The name for the content type.

    This is an arbitrary identifier for content interfaces. If it is not specified, it
    will default to the plug's name.
    """

    interface: str = pydantic.Field(
        description="The name of the interface.",
        examples=["network"],
    )
    """The name of the interface.

    See `Supported interfaces <https://snapcraft.io/docs/supported-interfaces>`_ for a
    list of supported interfaces.

    When using the content interface, this should be set to ``content``.
    """

    target: str = pydantic.Field(
        description="The path to where the producer's files will be available in the snap.",
        examples=["$SNAP/data-dir/themes"],
    )
    """The path to where the producer's files will be available in the snap.

    This is only needed when using the content interface. See the `Content
    interface <https://snapcraft.io/docs/content-interface>`_ for more information.
    """

    default_provider: str | None = pydantic.Field(
        default=None,
        description="The name of the producer snap..",
        examples=["gtk-common-themes"],
    )
    """The name of the producer snap.

    This is only needed when using the content interface. See the `Content interface
    <https://snapcraft.io/docs/content-interface>`_ for more information.
    """

    @pydantic.field_validator("default_provider")
    @classmethod
    def _validate_default_provider(cls, default_provider: str) -> str:
        if default_provider and "/" in default_provider:
            raise ValueError(
                "Specifying a snap channel in 'default_provider' is not supported: "
                f"{default_provider}"
            )
        return default_provider


class Platform(models.Platform):
    """Snapcraft project platform definition."""

    build_on: UniqueList[str] | str | None = pydantic.Field(  # type: ignore[assignment]
        description="The architectures to build the snap on.",
        examples=["arm64", "[amd64, riscv64]"],
        min_length=1,
    )
    """The architectures to build the snap on.

    This list must contain unique values. If the value is a string, it will be parsed
    into a single-entry list at runtime.
    """

    build_for: SingleEntryList | str | None = pydantic.Field(  # type: ignore[assignment]
        default=None,
        description="The target architecture for the build.",
        examples=["amd64", "[riscv64]"],
    )
    """The target architecture for the build.

    If the value is a string, it will be parsed into a single-entry list at runtime.
    This field is optional if the name of the platform is a valid ``build-for`` entry.
    """

    @pydantic.field_validator("build_on", "build_for", mode="before")
    @classmethod
    def _vectorise_build_on_build_for(cls, val: str | list[str]) -> list[str]:
        """Vectorise architectures if needed."""
        if isinstance(val, str):
            val = [val]
        return val

    @pydantic.model_validator(mode="before")
    @classmethod
    def _validate_platform_set(cls, values: Mapping[str, Any]) -> Mapping[str, Any]:
        """If build_for is provided, then build_on must also be.

        This aligns with the precedent set by the `architectures` key.
        """
        if not values.get("build_on") and values.get("build_for"):
            raise CraftValidationError(
                "'build_for' expects 'build_on' to also be provided."
            )

        return values

    @classmethod
    def from_architectures(
        cls,
        architectures: list[str | dict[str, Any]],
    ) -> dict[str, Self]:
        """Convert a core22 architectures configuration to core24 platforms."""
        platforms: dict[str, Self] = {}
        for architecture in architectures:
            if isinstance(architecture, str):
                build_on = build_for = cast(UniqueList[str], [architecture])
            else:
                build_on_val = architecture.get("build-on")
                build_for_val = architecture.get("build-for")
                if isinstance(build_on_val, str):
                    build_on = build_for = cast(UniqueList[str], [build_on_val])
                else:
                    build_on = build_for = cast(UniqueList[str], build_on_val)
                if build_for_val:
                    if isinstance(build_for_val, str):
                        build_for = cast(UniqueList[str], [build_for_val])
                    else:
                        build_for = cast(UniqueList[str], build_for_val)

            platforms[build_for[0]] = cls(build_for=build_for, build_on=build_on)

        return platforms


class Component(models.CraftBaseModel):
    """Snapcraft component definition."""

    summary: SummaryStr = pydantic.Field(
        description="The summary of the component.",
        examples=["Language translations for the app"],
    )
    """The summary of the component.

    This is a freeform field used to describe the purpose of the component.
    """

    description: str = pydantic.Field(
        description="The full description of the component.",
        examples=[
            "Contains optional translation packs to allow the user to change the language."
        ],
    )
    """The multi-line description of the component.

    This is a freeform field used to describe the purpose of the component.
    """

    type: Literal["test", "kernel-modules", "standard"] = pydantic.Field(
        description="The type of the component.",
        examples=["standard"],
    )
    """The type of the component.

    Different component types may have special handling by snapd.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Values
          - Description
        * - ``standard``
          - General use type. Use when no specific type applies.
        * - ``kernel-modules``
          - For kernel modules in snaps with type ``kernel``.
    """

    version: VersionStr | None = pydantic.Field(
        default=None,
        description="The version of the component.",
        examples=["1.2.3"],
    )
    """The version of the component.

    If the version is not provided, the component will be unversioned.
    """

    hooks: dict[str, Hook] | None = pydantic.Field(
        default=None,
        description="The configuration for the component's hooks.",
        examples=["{configure: {plugs: [home]}}"],
    )

    adopt_info: str | None = pydantic.Field(
        default=None,
        description=textwrap.dedent(
            """\
            Selects a part to inherit metadata from and reuse for the component's
            metadata.

            Only the component's version can be set.
            """
        ),
        examples=["foo-part"],
    )
    """Selects a part to inherit metadata from and reuse for the component's metadata.

    Only the component's version can be set.

    Metadata can be set using the ``craftctl set`` command in an ``override-`` script.
    For example, ``craftctl set components.my-component.version=$(git describe)``.
    """


MANDATORY_ADOPTABLE_FIELDS = ("version", "summary", "description")


class Project(models.Project):
    """A basic Snapcraft project.

    This model defines the basics of how a Snapcraft project works, extending from
    the craft-application project model.
    """

    # snapcraft's `name` is more general than craft-application
    name: ProjectName = pydantic.Field(  # type: ignore[assignment]
        description="The identifying name of the snap.",
        examples=["my-app", "powershell", "jupyterlab-desktop"],
    )
    """The identifying name of the snap.

    It must start with an ASCII character and can only contain lower case letters,
    numbers, and hyphens. It must contain at least one letter and it can't start or end
    with a hyphen. The maximum length is 40 characters.

    The name must be unique if you want to :ref:`publish it to the Snap Store
    <how-to-publish-a-snap>`.

    For help on choosing a name and registering it on the Snap Store, see :ref:`Register
    a snap <how-to-register-a-snap>`.
    """

    compression: Literal["lzo", "xz"] = pydantic.Field(
        default="xz",
        description="Specifies the algorithm that compresses the snap.",
        examples=["xz", "lzo"],
    )
    """Specifies the algorithm that compresses the snap.

    Snaps are compressed using ``xz`` data compression by default. This
    offers the optimal performance to compression ratio for the majority of
    snaps.

    However, there are certain types of snap, such as large desktop applications,
    that can benefit from using LZO compression. Snaps compressed with LZO are
    slightly larger but can decompress quicker, reducing the time it takes for
    freshly installed or refreshed snaps to launch.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``xz``
          - Default. Use `XZ <https://en.wikipedia.org/wiki/XZ_Utils>`_ compression.
        * - ``lzo``
          - Use `LZO <https://en.wikipedia.org/wiki/Lempel%E2%80%93Ziv%E2%80%93Oberhumer>`_ compression.

    """

    version: VersionStr | None = pydantic.Field(
        default=None,
        description="The version of the snap.",
        examples=["1.2.3"],
    )
    """The version of the snap.

    This field is required unless version information is provided by the ``adopt-info``
    key.

    See :ref:`how-to-configure-package-information` for details.
    """

    donation: UniqueList[str] | str | None = pydantic.Field(
        default=None,
        description="The snap's donation links.",
        examples=["[donate@example.com, https://example.com/donate]"],
    )
    """The snap's donation links.

    Donation links can be adopted from appstream metadata files.

    See :ref:`configure-package-information-reuse-information` for details.
    """

    # snapcraft's `source_code` is more general than craft-application
    source_code: UniqueList[str] | str | None = pydantic.Field(  # type: ignore[assignment]
        default=None,
        description="The links to the source code of the snap or the original project.",
        examples=["[https://example.com/source-code]"],
    )
    """The links to the source code of the snap or the original product.

    Source code links can be adopted from appstream metadata files.

    See :ref:`configure-package-information-reuse-information` for details.
    """

    contact: UniqueList[str] | str | None = pydantic.Field(  # type: ignore[reportIncompatibleVariableOverride]
        default=None,
        description="The snap author's contact links and email addresses.",
        examples=["[contact@example.com, https://example.com/contact]"],
    )
    """The snap author's contact links and email addresses.

    Contact information can be adopted from appstream metadata files.

    See :ref:`configure-package-information-reuse-information` for details.
    """

    issues: UniqueList[str] | str | None = pydantic.Field(  # type: ignore[reportIncompatibleVariableOverride]
        default=None,
        description="The links and email addresses for submitting issues, bugs, and feature requests.",
        examples=["[issues@email.com, https://example.com/issues]"],
    )
    """The links and email addresses for submitting issues, bugs, and feature
    requests.

    Issue links can be adopted from appstream metadata files.

    See :ref:`configure-package-information-reuse-information` for details.
    """

    website: UniqueList[str] | str | None = pydantic.Field(
        default=None,
        description="The links to the original software's web pages.",
        examples=["[https://example.com]"],
    )
    """The links to the original software's web pages.

    Websites can be adopted from appstream metadata files.

    See :ref:`configure-package-information-reuse-information` for details.
    """

    type: ProjectType | None = pydantic.Field(
        default=None, description="The snap's type.", examples=["kernel"]
    )

    icon: str | None = pydantic.Field(
        default=None,
        description="The path to the snap's icon.",
        examples=["snap/gui/icon.svg"],
    )
    """The path to the snap's icon.

    Icon size can be between 40x40 and 512x512 pixels.  256x256 is
    recommended.  The file should be less than 256 KB.

    The icon is used in the snap store and other graphical store fronts. The
    icon defined in the ``.desktop`` file is used as the icon in desktop menus.
    """

    confinement: Literal["classic", "devmode", "strict"] = pydantic.Field(
        description="The amount of isolation the snap has from the host system.",
        examples=[
            "strict",
            "classic",
            "devmode",
        ],
    )
    """The amount of isolation the snap has from the host system.

    Snap confinement determines the amount of access an application has to
    system resources, such as files, the network, peripherals and services.

    For core22 and newer bases, confinement is a required property and has no
    default value.

    For more information, see `Snap confinement
    <https://snapcraft.io/docs/snap-confinement>`_.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``strict``
          - Default for core20 and older bases. Use strict confinement.
        * - ``classic``
          - Use classic confinement.
        * - ``devmode``
          - Use devmode confinement.

    """

    layout: (
        dict[str, SingleEntryDict[Literal["symlink", "bind", "bind-file", "type"], str]]
        | None
    ) = pydantic.Field(
        default=None,
        description="The file layouts in the execution environment.",
        examples=["{/var/lib/foo: {bind: $SNAP_DATA/var/lib/foo}}"],
    )
    """The file layouts in the execution environment.

    Layouts modify the execution environment of a strictly-confined snap.

    With layouts, you can make elements in ``$SNAP``, ``$SNAP_DATA``, and
    ``$SNAP_COMMON`` accessible from locations such as ``/usr``, ``/var`` and ``/etc``.
    This helps when using pre-compiled binaries and libraries that expect to find files
    and directories outside of locations referenced by ``$SNAP`` or ``$SNAP_DATA``.

    See :ref:`reference-layouts` for details.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``symlink: <source-path>``
          - Create a symbolic link. This method is preferred because it is the cheapest;
            the other methods significantly increase the startup time of your
            application.
        * - ``bind: <source-path>``
          - Bind-mount a directory.
        * - ``bind-file: <source-path>``
          - Bind-mount a file.
        * - ``tmpfs: <source-path>``
          - Mount a private, temporary, in-memory filesystem.

    """

    grade: Literal["stable", "devel"] | None = pydantic.Field(
        default=None,
        description="The quality grade of the snap.",
        examples=["stable", "devel"],
    )
    """The quality grade of the snap.

    The grade can only be stable if the ``base`` and ``build-base`` are stable.
    For example, if the ``build-base`` is ``devel``, then the grade must be
    ``devel``.

    The grade must be ``stable`` to publish the snap to the ``candidate`` or
    ``stable`` channels on the snap store.  If the grade is ``devel``, it can only
    be published to the ``beta`` and ``edge`` channels.

    **Values**

    .. list-table::
        :header-rows: 1

        * - Value
          - Description
        * - ``stable``
          - Default. The snap is stable.
        * - ``devel``
          - The snap is still under development.

    """

    architectures: list[str | Architecture] | None = pydantic.Field(
        default=None,
        description="The architectures that the snap builds and runs on.",
        examples=[
            "[amd64, riscv64]",
            "[{build-on: [amd64], build-for: [amd64]}]",
            "[{build-on: [amd64, riscv64], build-for: [riscv64]}]",
        ],
    )
    """The architectures that the snap builds and runs on.

    Architectures can be defined as either a shorthand list of architectures or as pairs
    of ``build-on`` and ``build-for`` entries.

    The ``architectures`` key is only used in core22 and older snaps. For core24 and
    newer snaps, use the ``platforms`` key.
    """

    _architectures_in_yaml: bool | None = None

    platforms: dict[str, Platform] | None = pydantic.Field(  # type: ignore[assignment,reportIncompatibleVariableOverride]
        default=None,
        description="The architectures that the snap builds and runs on.",
        examples=[
            "{amd64: {build-on: [amd64], build-for: [amd64]}, arm64: {build-on: [amd64, arm64], build-for: [arm64]}}"
        ],
    )
    """The architectures that the snap builds and runs on.

    If the platform name is a valid Debian architecture, ``build-on`` and ``build-for``
    can be omitted.

    The platform name describes a ``build-on``/``build-for`` pairing.  When
    specifying ``build-on`` and ``build-for``, the the name is arbitrary but
    it's recommended to set the platform name to the ``build-for`` architecture.

    The ``platforms`` key is only used in core24 and newer snaps. For core22 and older
    snaps, use the ``architectures`` key.
    """

    assumes: UniqueList[str] = pydantic.Field(
        default_factory=list,
        description="The minimum version of snapd and its features that the snap requires from the host.",
        examples=["[snapd2.66, common-data-dir]"],
    )
    """The minimum version of snapd and its features that the snap requires from the
    host.

    If the host doesn't meet any of the requirements, snapd won't install the snap.

    You can declare both a snapd version and its features at the same time.

    List a minimum version of snapd with ``snapd<version>``.

    List required snapd features by name:

    - ``common-data-dir`` for the common data directory across revisions of a snap.
    - ``snap-env`` for declaring runtime environment variables in the project file.
    - ``command-chain`` for chaining commands in apps and hooks in the project file.
    - ``kernel-assets`` for kernel assets declared in a gadget snap project file.
    """

    hooks: dict[str, Hook] | None = pydantic.Field(
        default=None,
        description="Configures the snap's hooks.",
        examples=["{configure: {plugs: [home]}}"],
    )
    """Configures the snap's hooks.

    Hooks are run automatically if they're packaged with the snap. They only needed to
    be listed in the project file if they need additional configuration, such as an
    environment variable.

    See :ref:`hooks <reference-hooks>` for more information.
    """

    passthrough: dict[str, Any] | None = pydantic.Field(
        default=None,
        description="The attributes to pass to the snap's metadata file.",
        examples=["{daemon: complex}"],
    )
    """The attributes to pass to the snap's metadata file.

    These attributes are passed to the ``snap.yaml`` file without validation from
    Snapcraft. This is useful for early testing of a new feature in snapd that isn't yet
    supported by Snapcraft.

    To pass a value for a particular app, see the :ref:`passthrough key
    <App.passthrough>` for apps.

    See `Using development features in Snapcraft
    <https://snapcraft.io/docs/using-in-development-features>`_.
    """

    apps: dict[str, App] | None = pydantic.Field(
        default=None,
        description="The individual programs and services that the snap runs.",
        examples=["{app-1: {command: bin/app-1}}"],
    )
    """The individual programs and services that the snap runs.

    Each entry defines the executable, how it's run, and the resources it can access.

    If an app name matches the snap name, the app is exposed to the host with that name.
    If they differ, the app is exposed to the host as '<snap-name>.<app-name>'.
    """

    plugs: dict[str, ContentPlug | Any] | None = pydantic.Field(
        default=None,
        description="Declares the snap's plugs.",
        examples=[
            "{dot-gitconfig: {interface: personal-files, read: [$HOME/.gitconfig]}}"
        ],
    )
    """Declares the snap's plugs.

    See :ref:`explanation-interfaces` for more information.
    """

    slots: dict[str, Any] | None = pydantic.Field(
        default=None,
        description="Declares the snap's slots.",
        examples=[
            "{slot-1: {interface: content, content: my-binaries, source: {read: [$SNAP/bin]}}}"
        ],
    )
    """Declares the snap's slots.

    See :ref:`explanation-interfaces` for more information.
    """

    lint: Lint | None = pydantic.Field(
        default=None,
        description="The linter configuration settings.",
        examples=["{ignore: [classic, library: [usr/lib/**/libfoo.so*]]}"],
    )
    """The linter configuration settings.

    Snapcraft runs the following linters:

    - :ref:`classic <how-to-use-the-classic-linter>`: Verifies binary file parameters
      for snaps using :ref:`classic confinement <explanation-classic-confinement>`.
    - :ref:`library <how-to-use-the-library-linter>`: Verifies that no ELF file
      dependencies, such as libraries, are missing and that no extra libraries are
      included in the snap package.
    - :ref:`metadata <how-to-use-the-metadata-linter>`: Verifies that the snap contains
      all the :ref:`metadata <reference-anatomy-of-snapcraft-yaml-metadata>` needed for
      a better listing in the Snap Store.

    See :ref:`reference-linters` for more information.
    """

    epoch: str | None = pydantic.Field(
        default=None,
        description="The epoch associated with this version of the snap.",
        examples=["1", "2*"],
    )
    """The epoch associated with this version of the snap.

    Controls when users receive configuration-breaking application releases and ensures
    upgrades migrate through each epoch.

    Asterisks after the epoch denote it can read configuration data from the previous
    epoch.

    This is an uncommonly used key.

    See :ref:`how-to-manage-data-compatibility` for more information.
    """

    adopt_info: str | None = pydantic.Field(
        default=None,
        description=textwrap.dedent(
            """\
        Selects a part to inherit metadata from and reuse for the snap's metadata.

        Required if one of ``version``, ``summary``, or ``description`` isn't set."""
        ),
        examples=["foo-part"],
    )
    """Selects a part to inherit metadata from and reuse for the snap's
    metadata.

    Required if one of ``version``, ``summary``, or ``description`` isn't set.

    There are two ways to adopt metadata from a part:

    - Using the 'craftctl set' command in an 'override-' script.
    - Using appstream metadata files listed in the 'parse-info'  field of the
      adopting part.
    """

    system_usernames: dict[str, Any] | None = pydantic.Field(
        default=None,
        description="The system usernames the snap can use to run daemons and services.",
        examples=["{snap-daemon: shared}"],
    )
    """The system usernames the snap can use to run daemons and services.

    This is used to run daemons with the ``snap_daemon`` user defined by
    snapd. Otherwise, this is an uncommon key.

    See `system usernames <https://snapcraft.io/docs/system-usernames>`_ for more
    information.
    """

    environment: dict[str, str | None] | None = pydantic.Field(
        default=None,
        description="The snap's runtime environment variables.",
        examples=[
            "{PYTHONPATH: $SNAP/usr/lib/python3/dist-packages:$PYTHON_PATH, DISABLE_WAYLAND: 1}"
        ],
    )
    """The snap's runtime environment variables.

    Environment variables are set at runtime for all apps.  To set an
    environment variable for a particular app, use the ``environment`` key for
    that ``app`` entry.
    """

    build_packages: Grammar[list[str]] | None = pydantic.Field(
        default=None,
        description="The list of packages to install when building a snap.",
        examples=["[libssl-dev, libyaml-dev]"],
    )
    """The list of packages to install when building a snap.

    All build packages are installed before any part is built. However, if a
    package is only needed for one part, it's recommended to use the
    ``build-packages`` key for that part.  This organization makes it easier to
    track which parts require which build packages.
    """

    build_snaps: Grammar[list[str]] | None = pydantic.Field(
        default=None,
        description="The snaps to install when building a snap.",
        examples=["[go/1.22/stable, yq]"],
    )
    """The snaps to install when building a snap.

    If only the snap name is provided, the snap will be installed from the
    ``latest/stable`` channel. Otherwise, a channel can be specified with
    ``<name>/<channel>``.

    All build snaps are installed before any part is built.  However, if a
    snap is only needed for one part, it's recommended to use the
    ``build-snaps`` key for that part.  This organization makes it easier to
    track which parts require which build snaps.
    """

    ua_services: set[str] | None = pydantic.Field(
        default=None,
        description="The Ubuntu Pro (formerly Ubuntu Advantage) services to enable when building the snap.",
        examples=["[esm-apps]"],
    )
    """The Ubuntu Pro (formerly Ubuntu Advantage) services to enable when
    building the snap.

    Enabling `Ubuntu Pro <https://ubuntu.com/pro>`_ services allows building
    snaps in an Ubuntu Pro enabled environment.
    """

    provenance: str | None = pydantic.Field(
        default=None,
        description="The primary-key header for snaps signed by third parties.",
        examples=["test-provenance"],
    )
    """The primary-key header for snaps signed by third parties.

    This is an uncommonly used key.
    """

    components: dict[ProjectName, Component] | None = pydantic.Field(
        default=None,
        description="Declares the components to build in conjunction with the snap.",
        examples=["{foo-component: {type: standard}}"],
    )

    @override
    @classmethod
    def _providers_base(cls, base: str) -> bases.BaseAlias | None:
        """Get a BaseAlias from snapcraft's base.

        :param base: The application-specific base name.

        :returns: The BaseAlias for the base.

        :raises CraftValidationError: If the project's base cannot be determined.
        """
        if base == "bare":
            return None
        # Desktop bases are extended versions of the regular core bases.
        # For bases that end with "-desktop", use the equivalent provider base.
        if base.endswith("-desktop"):
            core_base = base.rpartition("-")[0]
        else:
            core_base = base

        try:
            return SNAPCRAFT_BASE_TO_PROVIDER_BASE[core_base]
        except KeyError as err:
            raise CraftValidationError(f"Unknown base {base!r}") from err

    @pydantic.field_validator("plugs")
    @classmethod
    def _validate_plugs(
        cls, plugs: dict[str, ContentPlug | Any]
    ) -> dict[str, ContentPlug | Any]:
        empty_plugs = []
        if plugs is not None:
            for plug_name, plug in plugs.items():
                if (
                    isinstance(plug, dict)
                    and plug.get("interface") == "content"
                    and not plug.get("target")
                ):
                    raise ValueError(
                        f"ContentPlug '{plug_name}' must have a 'target' parameter."
                    )

                if isinstance(plug, list):
                    raise ValueError(f"Plug '{plug_name}' cannot be a list.")

                if isinstance(plug, dict) and plug.get("default-provider"):
                    default_provider: str = plug.get("default-provider", "")
                    if "/" in default_provider:
                        raise ValueError(
                            "Specifying a snap channel in 'default_provider' is not supported: "
                            f"{default_provider}"
                        )

                if plug is None:
                    empty_plugs.append(plug_name)

        if empty_plugs:
            message = _format_global_key_warning("plug", empty_plugs)
            emit.message(message)

        return plugs

    @pydantic.field_validator("slots")
    @classmethod
    def _validate_slots(cls, slots: dict[str, Any]) -> dict[str, Any]:
        empty_slots = []
        if slots is not None:
            for slot_name, slot in slots.items():
                if slot is None:
                    empty_slots.append(slot_name)

        if empty_slots:
            message = _format_global_key_warning("slot", empty_slots)
            emit.message(message)

        return slots

    @pydantic.model_validator(mode="after")
    def _validate_adoptable_fields(self) -> Self:
        for field in MANDATORY_ADOPTABLE_FIELDS:
            if getattr(self, field) is None and self.adopt_info is None:
                raise ValueError(
                    f"Required field '{field}' is not set and 'adopt-info' not used."
                )
        return self

    @pydantic.field_validator("name")
    @classmethod
    def _validate_snap_name(cls, name: str) -> str:
        return validate_name(name=name, field_name="snap")

    @pydantic.field_validator("components")
    @classmethod
    def _validate_components(
        cls, components: dict[str, Component]
    ) -> dict[str, Component]:
        """Validate component names."""
        for component_name in components.keys():
            _validate_component(name=component_name)

        return components

    @pydantic.field_validator("platforms")
    @classmethod
    def _validate_all_platforms(
        cls, platforms: dict[str, Platform]
    ) -> dict[str, Platform]:
        """Validate and convert platform data to a dict of Platforms."""
        for platform_label, platform in platforms.items():
            error_prefix = f"Error for platform entry '{platform_label}'"
            # build_on and build_for are validated
            # let's also validate the platform label
            build_on_one_of = platform.build_on or [platform_label]

            # If the label maps to a valid architecture and
            # `build-for` is present, then both need to have the same value,
            # otherwise the project is invalid.
            if platform.build_for:
                build_target = platform.build_for[0]
                if platform_label in SUPPORTED_ARCHS and platform_label != build_target:
                    raise ValueError(
                        str(
                            f"{error_prefix}: if 'build_for' is provided and the "
                            "platform entry label corresponds to a valid architecture, then "
                            f"both values must match. {platform_label} != {build_target}"
                        )
                    )
            # if no build-for is present, then the platform label needs to be a valid architecture
            elif platform_label not in SUPPORTED_ARCHS:
                raise ValueError(
                    str(
                        f"{error_prefix}: platform entry label must correspond to a "
                        "valid architecture if 'build-for' is not provided."
                    )
                )

            # Both build and target architectures must be supported
            if not any(b_o in SUPPORTED_ARCHS for b_o in build_on_one_of):
                raise ValueError(
                    str(
                        f"{error_prefix}: trying to build snap in one of "
                        f"{build_on_one_of}, but none of these build architectures are supported. "
                        f"Supported architectures: {SUPPORTED_ARCHS}"
                    )
                )

            platforms[platform_label] = platform

        return platforms

    @pydantic.model_validator(mode="after")
    def _validate_grade_and_build_base(self) -> Self:
        """If build_base is devel, then grade must be devel."""
        if self.build_base == "devel" and self.grade == "stable":
            raise ValueError("grade must be 'devel' when build-base is 'devel'")
        return self

    @pydantic.field_validator("build_base")
    @classmethod
    def _validate_build_base(
        cls, value: str | None, info: pydantic.ValidationInfo
    ) -> str | None:
        """Build-base defaults to the base value if not specified."""
        return value or info.data.get("base")

    @pydantic.field_validator("epoch")
    @classmethod
    def _validate_epoch(cls, epoch: str) -> str:
        """Verify epoch format."""
        if epoch is not None and not re.match(r"^(?:0|[1-9][0-9]*[*]?)$", epoch):
            raise ValueError(
                "Epoch is a positive integer followed by an optional asterisk"
            )

        return epoch

    @pydantic.field_validator("provenance")
    @classmethod
    def _validate_provenance(cls, provenance: str) -> str:
        if provenance and not re.match(r"^[a-zA-Z0-9-]+$", provenance):
            raise ValueError(
                "provenance must consist of alphanumeric characters and/or hyphens."
            )

        return provenance

    @pydantic.field_validator(
        "contact", "donation", "issues", "source_code", "website", mode="before"
    )
    @classmethod
    def _validate_urls(cls, field_value: list[str] | str) -> list[str]:
        if isinstance(field_value, str):
            field_value = cast(UniqueList[str], [field_value])
        return field_value

    @pydantic.field_validator("parts")
    @classmethod
    def _validate_no_snapcraftctl(
        cls, parts: dict[str, Part], info: pydantic.ValidationInfo
    ) -> dict[str, Part]:
        """Provide a helpful error for using snapcraftctl in core26+."""
        override_keys = [
            "override-pull",
            "override-build",
            "override-stage",
            "override-prime",
        ]

        # core22 and core24 can use snapcraftctl
        if {"core22", "core24"} & {info.data.get("base"), info.data.get("build-base")}:
            return parts

        for name, part in parts.items():
            for key in override_keys:
                script = part.get(key)
                if not script:
                    continue

                for line in script.splitlines():
                    try:
                        # ignore snapcraftctl in comments
                        tokens = shlex.split(line, comments=True)
                    except ValueError:
                        # ignore malformed lines
                        continue

                    # error only if `snapcraftctl` is the command (`echo "snapcraftctl"` isn't an error)
                    # also ignore the path prefixing the command (`${SNAP}/libexec/snapcraft/snapcraftctl default` is an error)
                    if tokens and tokens[0].split("/")[-1] == "snapcraftctl":
                        raise ValueError(
                            f"Can't use 'snapcraftctl' in the {key} script for part {name!r}. "
                            "Use 'craftctl' instead."
                        )

        return parts

    @override
    @classmethod
    def unmarshal(cls, data: dict[str, Any]) -> Self:
        """Create a Snapcraft project from a dictionary of data."""
        return pydantic.TypeAdapter(SnapcraftProject).validate_python(data)

    @override
    def marshal(self) -> dict[str, str | list[str] | dict[str, Any]]:
        """Convert to a dictionary."""
        data: dict = super().marshal()
        if isinstance(data.get("type"), ProjectType):
            data["type"] = data["type"].value
        return data

    def _get_content_plugs(self) -> list[ContentPlug]:
        """Get list of content plugs."""
        if self.plugs is not None:
            return [
                plug for plug in self.plugs.values() if isinstance(plug, ContentPlug)
            ]
        return []

    def get_content_snaps(self) -> list[str]:
        """Get list of snaps from ContentPlug `default-provider` fields."""
        return [
            x.default_provider
            for x in self._get_content_plugs()
            if x.default_provider is not None
        ]

    def get_extra_build_snaps(self) -> list[str]:
        """Get list of extra snaps required to build."""
        # Build snaps defined by the user with channel stripped
        build_snaps: list[str] = []
        for part in self.parts.values():
            build_snaps.extend(part.get("build-snaps", []))
        part_build_snaps = {p.split("/")[0] for p in build_snaps}

        # Content snaps the project uses
        content_snaps = set(self.get_content_snaps())

        # Do not add the content snaps if provided by the user
        extra_build_snaps = list(content_snaps - part_build_snaps)

        # Always add the base as an extra build snap
        if self.base is not None:
            extra_build_snaps.append(self.base)
        extra_build_snaps.sort()

        return extra_build_snaps

    def get_effective_base(self) -> str:
        """Return the base to use to create the snap."""
        base = get_effective_base(
            base=self.base,
            build_base=self.build_base,
            project_type=self.type.value if self.type else None,
            name=self.name,
        )

        # will not happen after schema validation
        if base is None:
            raise RuntimeError("cannot determine build base")

        return base

    def get_build_on(self) -> str:
        """Get the first build_on architecture from the project for core22."""
        if (
            self.architectures
            and isinstance(self.architectures[0], Architecture)
            and isinstance(self.architectures[0].build_on, list)
        ):
            return self.architectures[0].build_on[0]

        # will not happen after schema validation
        raise RuntimeError("cannot determine build-on architecture")

    def get_build_for(self) -> str:
        """Get the first build_for architecture from the project for core22."""
        if (
            self.architectures
            and isinstance(self.architectures[0], Architecture)
            and isinstance(self.architectures[0].build_for, list)
        ):
            return self.architectures[0].build_for[0]

        # will not happen after schema validation
        raise RuntimeError("cannot determine build-for architecture")

    def get_build_for_arch_triplet(self) -> str | None:
        """Get the architecture triplet for the first build-for architecture for core22.

        :returns: The build-for arch triplet. If build-for is "all", then return None.
        """
        arch = self.get_build_for()

        if arch != "all":
            return get_arch_triplet(
                DebianArchitecture.from_machine(arch).to_platform_arch()
            )

        return None

    def get_component_names(self) -> list[str]:
        """Get a list of component names.

        :returns: A list of component names.
        """
        return list(self.components.keys()) if self.components else []

    def get_partitions(self) -> list[str] | None:
        """Get a list of partitions based on the project's components.

        :returns: A list of partitions formatted as ['default', 'component/<name>', ...]
        or None if no components are defined.
        """
        return _get_partitions_from_components(self.components)


def _custom_error(error_msg: str):
    def _validator(v: Any, next_: Any, ctx: pydantic.ValidationInfo):
        try:
            return next_(v, ctx)
        except ValueError as exc:
            raise ValueError(error_msg) from exc

    return pydantic.WrapValidator(_validator)


class _BaselessProject(Project):
    """Project types that do not require a base."""

    type: Literal[  # type: ignore[reportIncompatibleVariableOverride]
        ProjectType.BASE, ProjectType.KERNEL, ProjectType.SNAPD
    ]
    base: SkipJsonSchema[str | None] = None
    build_base: Literal["core24", "core26", "devel"] = pydantic.Field(  # type: ignore[reportIncompatibleVariableOverride]
        description="The baseline system that the snap is built in.",
    )

    @pydantic.model_validator(mode="after")
    def _validate_no_base(self) -> Self:
        """Baseless projects cannot have a base value set."""
        if self.base is not None:
            raise ValueError(f"{self.type!r} snaps cannot have a base.")
        return self

    @override
    @pydantic.field_validator("build_base", mode="after")
    @classmethod
    def _validate_devel_base(
        cls, build_base: str, info: pydantic.ValidationInfo
    ) -> str:
        """Override _validate_devel_base from craft-application to be a no-op.

        We're overriding this because pydantic does not allow before validators on
        discriminator fields.
        """
        return build_base


class _BaselessCore22Project(_BaselessProject):
    build_base: Literal["core22"] = pydantic.Field(  # type: ignore[assignment]
        description="The baseline system that the snap is built in.",
    )

    @pydantic.field_validator("architectures")
    @classmethod
    def _validate_architecture_data(
        cls, architectures: list[str | Architecture], info: pydantic.ValidationInfo
    ) -> list[Architecture]:
        """Validate architecture data."""
        return validate_architectures(architectures)

    @pydantic.model_validator(mode="after")
    def _validate_platforms_and_architectures(self) -> Self:
        """Validate usage of platforms and architectures.

        - can optionally define architectures
        - cannot define platforms
        """
        # this is a one-shot - the value should not change when re-validating
        if self.architectures and self._architectures_in_yaml is None:
            # record if architectures are defined in the yaml for remote-build (#4881)
            self._architectures_in_yaml = True

        elif not self.architectures:
            self._architectures_in_yaml = False
            # set default value
            host_arch = str(DebianArchitecture.from_host())
            self.architectures = [
                Architecture(
                    build_on=[host_arch],
                    build_for=[host_arch],
                )
            ]

        return self


BaselessProject = Annotated[
    Annotated[_BaselessCore22Project, pydantic.Tag("baseless-core22")]
    | Annotated[_BaselessProject, pydantic.Tag("baseless")],
    pydantic.Discriminator("build_base"),
]


class StableBaseProject(Project):
    """Project types that require a base."""

    type: Literal[  # type: ignore[reportIncompatibleVariableOverride]
        ProjectType.APP, ProjectType.GADGET, None
    ] = None
    base: StableBase  # type: ignore[reportIncompatibleVariableOverride]
    build_base: str | None = pydantic.Field(
        validate_default=True,
        default=None,
        description="The baseline system that the snap is built in.",
        examples=["core20", "core22", "core24", "devel"],
    )
    """The baseline system that the snap is built in.

    If the snap has a stable base, ``build-base`` must either be unset
    or the same as the base.

    If the snap has an unstable base, ``build-base`` must be set to ``devel``.

    If the snap's type doesn't use a base, ``build-base`` must be set.
    """

    @pydantic.field_validator("type", mode="before")
    @classmethod
    def _validate_type(cls, snap_type: str | None | ProjectType) -> ProjectType | None:
        if isinstance(snap_type, str):
            try:
                snap_type = ProjectType(snap_type)
            except ValueError:
                valid_types = humanize_list(
                    (t.value for t in ProjectType), conjunction="or"
                )
                raise ValueError(f"Input should be {valid_types}") from None

        return snap_type


class Core22Project(StableBaseProject):
    base: Literal["core22"]  # type: ignore[assignment]

    platforms: SkipJsonSchema[dict[str, Platform] | None] = pydantic.Field(  # type: ignore[assignment,reportIncompatibleVariableOverride]
        default=None,
        description="Not available for core22. Use the `architectures` key.",
        exclude=True,
        repr=False,
    )

    @pydantic.model_validator(mode="after")
    def _validate_platforms_and_architectures(self) -> Self:
        """Validate usage of platforms and architectures.

        - can optionally define architectures
        - cannot define platforms
        """
        # this is a one-shot - the value should not change when re-validating
        if self.architectures and self._architectures_in_yaml is None:
            # record if architectures are defined in the yaml for remote-build (#4881)
            self._architectures_in_yaml = True

        elif not self.architectures:
            self._architectures_in_yaml = False
            # set default value
            host_arch = str(DebianArchitecture.from_host())
            self.architectures = [
                Architecture(
                    build_on=[host_arch],
                    build_for=[host_arch],
                )
            ]

        return self

    @pydantic.field_validator("architectures")
    @classmethod
    def _validate_architecture_data(
        cls, architectures: list[str | Architecture], info: pydantic.ValidationInfo
    ) -> list[Architecture]:
        """Validate architecture data."""
        return validate_architectures(architectures)


class BareCore22Project(Core22Project):
    base: Literal["bare"]  # type: ignore[assignment,reportIncompatibleVariableOverride]
    build_base: Literal["core22"]  # type: ignore[reportIncompatibleVariableOverride]

    @override
    @pydantic.field_validator("build_base", mode="after")
    @classmethod
    def _validate_devel_base(
        cls, build_base: str, info: pydantic.ValidationInfo
    ) -> str:
        """Override _validate_devel_base from craft-application to be a no-op.

        We're overriding this because pydantic does not allow before validators on
        discriminator fields.
        """
        return build_base


class Core24Project(StableBaseProject):
    base: Literal["core24"]  # type: ignore[assignment]

    architectures: SkipJsonSchema[  # type: ignore[reportIncompatibleVariableOverride]
        Annotated[
            None,
            _custom_error(
                "'architectures' key is not supported for base 'core24'. Use 'platforms' key instead."
            ),
        ]
    ] = pydantic.Field(
        default=None,
        description="The architectures key is only used in core22 snaps and below. For core24 and newer snaps, use the ``platforms`` key.",
    )

    platforms: dict[str, Platform | None] | None = pydantic.Field(  # type: ignore[assignment,reportIncompatibleVariableOverride]
        default=None,
        description="The platforms where the snap can be built and where the resulting snap can run.",
        examples=[
            "{amd64: {build-on: [amd64], build-for: [amd64]}, arm64: {build-on: [amd64, arm64], build-for: [arm64]}}"
        ],
    )


class BareCore24Project(Core24Project):
    base: Literal["bare"]  # type: ignore[assignment,reportIncompatibleVariableOverride]
    build_base: Literal["core24"]  # type: ignore[reportIncompatibleVariableOverride]

    @override
    @pydantic.field_validator("build_base", mode="after")
    @classmethod
    def _validate_devel_base(
        cls, build_base: str, info: pydantic.ValidationInfo
    ) -> str:
        """Override _validate_devel_base from craft-application to be a no-op.

        We're overriding this because pydantic does not allow before validators on
        discriminator fields.
        """
        return build_base


class Core26Project(Core24Project):
    base: Literal["core26", "devel"]  # type: ignore[assignment]
    build_base: Literal["devel"]  # pyright: ignore[reportGeneralTypeIssues,reportIncompatibleVariableOverride]

    grade: Annotated[  # type: ignore[reportIncompatibleVariableOverride]
        Literal["devel"],
        _custom_error("grade must be 'devel' when build-base is 'devel'"),
    ]


class BareCore26Project(Core26Project):
    base: Literal["bare"]  # type: ignore[assignment]
    build_base: Literal["devel"]  # type: ignore[assignment]

    @override
    @pydantic.field_validator("build_base", mode="after")
    @classmethod
    def _validate_devel_base(
        cls, build_base: str, info: pydantic.ValidationInfo
    ) -> str:
        """Override _validate_devel_base from craft-application to be a no-op.

        We're overriding this because pydantic does not allow before validators on
        discriminator fields.
        """
        return build_base


_BareProject = Annotated[
    BareCore22Project | BareCore24Project | BareCore26Project,
    pydantic.Discriminator("build_base"),
]

_StandardProject = Annotated[
    Core22Project | Core24Project | Core26Project | _BareProject,
    pydantic.Discriminator("base"),
]


def _validate_type(data: dict[str, Any]) -> dict[str, Any]:
    if (
        data.get("base") is None  # Catch both missing base key and null base.
        and data.get("type") not in ("base", "kernel", "snapd")
    ):
        snap_type = data.get("type")
        if snap_type is None:
            raise ValueError("Missing 'base' key for snap.")
        raise ValueError(f"Missing 'base' key for snap type {snap_type!r}.")
    return data


def _has_type(data: dict[str, Any]) -> str:
    if data.get("type") in ("base", "kernel", "snapd"):
        return "baseless"
    return "has-base"


SnapcraftProject = Annotated[
    Annotated[_StandardProject, pydantic.Tag("has-base")]
    | Annotated[BaselessProject, pydantic.Tag("baseless")],
    pydantic.Discriminator(_has_type),
    pydantic.BeforeValidator(_validate_type),
]


class _GrammarAwareModel(pydantic.BaseModel):
    model_config = ConfigDict(
        validate_assignment=True,
        extra="allow",
        alias_generator=lambda s: s.replace("_", "-"),
        populate_by_name=True,
    )


class _GrammarAwarePart(_GrammarAwareModel):
    source: Grammar[str] | None = None
    build_environment: Grammar[list[SingleEntryDict[str, str]]] | None = None
    build_packages: Grammar[list[str]] | None = None
    stage_packages: Grammar[list[str]] | None = None
    build_snaps: Grammar[list[str]] | None = None
    stage_snaps: Grammar[list[str]] | None = None
    parse_info: list[str] | None = None


class GrammarAwareProject(_GrammarAwareModel):
    """Project definition containing grammar-aware components."""

    parts: dict[str, _GrammarAwarePart] = pydantic.Field(
        description="Declares the self-contained software pieces needed to create the snap.",
        examples=["parts: {part-1: {source: src, plugin: go}}"],
    )

    @classmethod
    def validate_grammar(cls, data: dict[str, Any]) -> None:
        """Ensure grammar-enabled entries are syntactically valid."""
        try:
            cls(**data)
        except pydantic.ValidationError as err:
            raise ProjectValidationError(_format_pydantic_errors(err.errors())) from err


class ArchitectureProject(models.CraftBaseModel, extra="ignore"):
    """Project definition containing only architecture data."""

    architectures: list[str | Architecture] = pydantic.Field(
        default=[str(DebianArchitecture.from_host())],
        validate_default=True,
    )

    @pydantic.field_validator("architectures")
    @classmethod
    def _validate_architecture_data(
        cls, architectures: list[str | Architecture]
    ) -> list[Architecture]:
        """Validate architecture data."""
        return validate_architectures(architectures)


class ComponentProject(models.CraftBaseModel, extra="ignore"):
    """Project definition containing only component data."""

    components: dict[ProjectName, Component] | None = None

    @pydantic.field_validator("components")
    @classmethod
    def _validate_components(
        cls, components: dict[str, Component]
    ) -> dict[str, Component]:
        """Validate component names."""
        for component_name in components.keys():
            _validate_component(name=component_name)

        return components

    def get_component_names(self) -> list[str]:
        """Get a list of component names.

        :returns: A list of component names.
        """
        return list(self.components.keys()) if self.components else []

    def get_partitions(self) -> list[str] | None:
        """Get a list of partitions based on the project's components.

        :returns: A list of partitions formatted as ['default', 'component/<name>', ...]
        or None if no components are defined.
        """
        return _get_partitions_from_components(self.components)


def _format_pydantic_errors(
    errors: Iterable[error_wrappers.ErrorDict],
    *,
    file_name: str = "snapcraft.yaml",
) -> str:
    """Format errors.

    Example 1: Single error.

    Bad snapcraft.yaml content:
    - field: <some field>
      reason: <some reason>

    Example 2: Multiple errors.

    Bad snapcraft.yaml content:
    - field: <some field>
      reason: <some reason>
    - field: <some field 2>
      reason: <some reason 2>
    """
    combined = [f"Bad {file_name} content:"]
    for error in errors:
        formatted_loc = _format_pydantic_error_location(error["loc"])
        formatted_msg = _format_pydantic_error_message(error["msg"])

        if formatted_msg == "field required":
            field_name, location = _printable_field_location_split(formatted_loc)
            combined.append(
                f"- field {field_name} required in {location} configuration"
            )
        elif formatted_msg == "extra fields not permitted":
            field_name, location = _printable_field_location_split(formatted_loc)
            combined.append(
                f"- extra field {field_name} not permitted in {location} configuration"
            )
        elif formatted_msg == "the list has duplicated items":
            field_name, location = _printable_field_location_split(formatted_loc)
            combined.append(
                f" - duplicate entries in {field_name} not permitted in {location} configuration"
            )
        elif formatted_loc == "__root__":
            combined.append(f"- {formatted_msg}")
        else:
            combined.append(f"- {formatted_msg} (in field {formatted_loc!r})")

    return "\n".join(combined)


def _format_pydantic_error_location(loc: Iterable[str | int]) -> str:
    """Format location."""
    loc_parts = []
    for loc_part in loc:
        if isinstance(loc_part, str):
            loc_parts.append(loc_part)
        elif isinstance(loc_part, int):
            # Integer indicates an index. Go
            # back and fix up previous part.
            previous_part = loc_parts.pop()
            previous_part += f"[{loc_part}]"
            loc_parts.append(previous_part)
        else:
            raise RuntimeError(f"unhandled loc: {loc_part}")

    loc = ".".join(loc_parts)

    # Filter out internal __root__ detail.
    loc = loc.replace(".__root__", "")
    return loc


def _format_pydantic_error_message(msg: str) -> str:
    """Format pydantic's error message field."""
    # Replace shorthand "str" with "string".
    msg = msg.replace("str type expected", "string type expected")
    return msg


def _printable_field_location_split(location: str) -> tuple[str, str]:
    """Return split field location.

    If top-level, location is returned as unquoted "top-level".
    If not top-level, location is returned as quoted location, e.g.

    (1) field1[idx].foo => 'foo', 'field1[idx]'
    (2) field2 => 'field2', top-level

    :returns: Tuple of <field name>, <location> as printable representations.
    """
    loc_split = location.split(".")
    field_name = repr(loc_split.pop())

    if loc_split:
        return field_name, repr(".".join(loc_split))

    return field_name, "top-level"


def _format_global_key_warning(key: str, empty_entries: list[str]) -> str:
    """Create a warning message about global assignment in the ``key`` field.

    :param key:
        The top-level key that contains empty entries (currently either
        "plug" or "slot").
    :param empty_entries:
        The entries inside the ``key`` dict that are empty.
    :return:
        A properly-formatted warning message.
    """
    culprits = utils.humanize_list(empty_entries, "and")
    return (
        f"Warning: implicit {key.lower()} assignment in {culprits}. "
        f"{key.capitalize()}s should be assigned to the app to which they apply, "
        f"and not implicitly assigned via the global '{key.lower()}s:' "
        "stanza which is intended for configuration only."
        "\n(Reference: https://documentation.ubuntu.com/snapcraft/stable/reference/"
        "project-file/snapcraft-yaml)"
    )
