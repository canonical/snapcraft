# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

"""Create snap.yaml metadata file."""

import re
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Set, Union, cast

import yaml
from craft_cli import emit
from pydantic import Extra, ValidationError, validator
from pydantic_yaml import YamlModel

from snapcraft import errors
from snapcraft.projects import App, Project
from snapcraft.utils import get_ld_library_paths, process_version


class _SnapMetadataModel(YamlModel):
    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


class Socket(_SnapMetadataModel):
    """snap.yaml app socket entry."""

    listen_stream: Union[int, str]
    socket_mode: Optional[int]


class SnapApp(_SnapMetadataModel):
    """Snap.yaml app entry.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.

    TODO: implement desktop (CRAFT-804)
    TODO: implement extensions (CRAFT-805)
    """

    class Config:
        """Pydantic model configuration."""

        extra = Extra.allow

    command: str
    autostart: Optional[str]
    common_id: Optional[str]
    bus_name: Optional[str]
    completer: Optional[str]
    stop_command: Optional[str]
    post_stop_command: Optional[str]
    start_timeout: Optional[str]
    stop_timeout: Optional[str]
    watchdog_timeout: Optional[str]
    reload_command: Optional[str]
    restart_delay: Optional[str]
    timer: Optional[str]
    daemon: Optional[str]
    after: Optional[List[str]]
    before: Optional[List[str]]
    refresh_mode: Optional[str]
    stop_mode: Optional[str]
    restart_condition: Optional[str]
    install_mode: Optional[str]
    plugs: Optional[List[str]]
    slots: Optional[List[str]]
    aliases: Optional[List[str]]
    environment: Optional[Dict[str, Any]]
    command_chain: Optional[List[str]]
    sockets: Optional[Dict[str, Socket]]
    daemon_scope: Optional[str]
    activates_on: Optional[List[str]]


class ContentPlug(_SnapMetadataModel):
    """Content plug definition in the snap metadata."""

    interface: Literal["content"]
    target: str
    content: Optional[str]
    default_provider: Optional[str]

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "ContentPlug":
        """Create and populate a new ``ContentPlug`` object from dictionary data.

        :param data: The dictionary data to unmarshal.
        :return: The newly created object.
        :raise TypeError: If data is not a dictionary.
        """
        if not isinstance(data, dict):
            raise TypeError("data is not a dictionary")

        return cls(**data)

    @validator("target")
    @classmethod
    def _validate_target_not_empty(cls, val):
        if val == "":
            raise ValueError("value cannot be empty")
        return val

    @property
    def provider(self) -> Optional[str]:
        """Return the default content provider name."""
        if self.default_provider is None:
            return None

        # ignore :<slot> if present
        if ":" in self.default_provider:
            return self.default_provider.split(":")[0]

        return self.default_provider


class ContentSlot(_SnapMetadataModel):
    """Content slot definition in the snap metadata."""

    interface: Literal["content"]
    content: Optional[str]
    read: List[str] = []
    write: List[str] = []

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "ContentSlot":
        """Create and populate a new ``ContentSlot`` object from dictionary data.

        :param data: The dictionary data to unmarshal.
        :return: The newly created object.
        :raise TypeError: If data is not a dictionary.
        """
        if not isinstance(data, dict):
            raise TypeError("data is not a dictionary")

        return cls(**data)

    def get_content_dirs(self, installed_path: Path) -> Set[Path]:
        """Obtain the slot's content directories."""
        content_dirs: Set[Path] = set()

        for path in self.read + self.write:
            # Strip leading "$SNAP" and "/".
            path = re.sub(r"^\$SNAP", "", path)
            path = re.sub(r"^/", "", path)
            path = re.sub(r"^./", "", path)
            content_dirs.add(installed_path / path)

        return content_dirs


class SnapMetadata(_SnapMetadataModel):
    """The snap.yaml model.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.
    """

    class Config:
        """Pydantic model configuration."""

        extra = Extra.allow

    name: str
    title: Optional[str]
    version: str
    summary: str
    description: str
    license: Optional[str]
    type: Optional[str]
    architectures: List[str]
    base: Optional[str]
    build_base: Optional[str]
    assumes: Optional[List[str]]
    epoch: Optional[str]
    apps: Optional[Dict[str, SnapApp]]
    confinement: str
    grade: str
    environment: Optional[Dict[str, Any]]
    plugs: Optional[Dict[str, Any]]
    slots: Optional[Dict[str, Any]]
    hooks: Optional[Dict[str, Any]]
    layout: Optional[Dict[str, Dict[str, str]]]
    system_usernames: Optional[Dict[str, Any]]
    provenance: Optional[str]

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "SnapMetadata":
        """Create and populate a new ``SnapMetadata`` object from dictionary data.

        The unmarshal method validates entries in the input dictionary, populating
        the corresponding fields in the data object.

        :param data: The dictionary data to unmarshal.

        :return: The newly created object.

        :raise TypeError: If data is not a dictionary.
        """
        if not isinstance(data, dict):
            raise TypeError("data is not a dictionary")

        return cls(**data)

    def get_provider_content_directories(self) -> List[Path]:
        """Get provider content directories from installed snaps."""
        provider_dirs: Set[Path] = set()

        for plug in self.get_content_plugs():
            # Get matching slot provider for plug
            if not plug.provider:
                continue

            provider_path = Path("/snap", plug.provider, "current")
            provider_yaml_path = provider_path / "meta" / "snap.yaml"

            emit.debug(f"check metadata for provider snap {str(provider_path)!r}")
            if not provider_yaml_path.exists():
                continue

            provider_metadata = read(provider_path)
            for slot in provider_metadata.get_content_slots():
                content_dirs = slot.get_content_dirs(installed_path=provider_path)
                emit.debug(f"content dirs: {content_dirs}")
                provider_dirs |= content_dirs

        return sorted(provider_dirs)

    def get_content_plugs(self) -> List[ContentPlug]:
        """Return a list of content plugs from the snap metadata plugs."""
        if not self.plugs:
            return []

        content_plugs: List[ContentPlug] = []
        for name, data in self.plugs.items():
            try:
                plug = ContentPlug.unmarshal(data)
                if not plug.content:
                    plug.content = name
                content_plugs.append(plug)
            except (TypeError, ValidationError):
                continue
        return content_plugs

    def get_content_slots(self) -> List[ContentSlot]:
        """Return a list of content slots from the snap metadata slots."""
        if not self.slots:
            return []

        content_slots: List[ContentSlot] = []
        for name, slot_data in self.slots.items():
            try:
                slot = ContentSlot.unmarshal(slot_data)
                if not slot.content:
                    slot.content = name
                content_slots.append(slot)
            except (TypeError, ValidationError):
                continue
        return content_slots


def read(prime_dir: Path) -> SnapMetadata:
    """Read snap metadata file.

    :param prime_dir: The directory containing the snap payload.
    :return: The populated snap metadata.
    """
    snap_yaml = prime_dir / "meta" / "snap.yaml"
    try:
        with snap_yaml.open(encoding="utf-8") as file:
            data = yaml.safe_load(file)
    except OSError as error:
        raise errors.SnapcraftError(f"Cannot read snap metadata: {error}") from error

    return SnapMetadata.unmarshal(data)


def _create_snap_app(app: App, assumes: Set[str]) -> SnapApp:
    app_sockets: Dict[str, Socket] = {}
    if app.sockets:
        for socket_name, socket in app.sockets.items():
            app_sockets[socket_name] = Socket(
                listen_stream=socket.listen_stream,
                socket_mode=socket.socket_mode,
            )

    if app.command_chain:
        assumes.add("command-chain")

    snap_app = SnapApp(
        command=app.command,
        autostart=app.autostart,
        common_id=app.common_id,
        bus_name=app.bus_name,
        completer=app.completer,
        stop_command=app.stop_command,
        post_stop_command=app.post_stop_command,
        start_timeout=app.start_timeout,
        stop_timeout=app.stop_timeout,
        watchdog_timeout=app.watchdog_timeout,
        reload_command=app.reload_command,
        restart_delay=app.restart_delay,
        timer=app.timer,
        daemon=app.daemon,
        after=app.after or None,
        before=app.before or None,
        refresh_mode=app.refresh_mode,
        stop_mode=app.stop_mode,
        restart_condition=app.restart_condition,
        install_mode=app.install_mode,
        plugs=app.plugs,
        slots=app.slots,
        aliases=app.aliases,
        environment=app.environment,
        command_chain=app.command_chain or None,
        sockets=app_sockets or None,
        daemon_scope=app.daemon_scope or None,
        activates_on=app.activates_on or None,
    )

    if app.passthrough:
        for name, value in app.passthrough.items():
            setattr(snap_app, name, value)

    return snap_app


def _get_grade(grade: Optional[str], build_base: Optional[str]) -> str:
    """Get the grade for a project.

    If the build_base is `devel`, then the grade should be `devel`.
    If the grade is not defined, default to `stable`.

    :param grade: Grade set by project or during the lifecycle.
    :param build_base: build-base defined in the snapcraft.yaml.

    :returns: Grade of project (`stable` or `devel`).
    """
    if build_base == "devel":
        emit.debug("Setting grade to 'devel' because build_base is 'devel'.")
        return "devel"

    if not grade:
        emit.debug("Grade not specified, using default value 'stable'.")
        return "stable"

    return grade


def write(project: Project, prime_dir: Path, *, arch: str, arch_triplet: str):
    """Create a snap.yaml file.

    :param project: Snapcraft project.
    :param prime_dir: The directory containing the content to be snapped.
    :param arch: Target architecture the snap project is built to.
    :param arch_triplet: Architecture triplet of the platform.
    """
    meta_dir = prime_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    assumes: Set[str] = set()

    snap_apps: Dict[str, SnapApp] = {}
    if project.apps:
        for name, app in project.apps.items():
            snap_apps[name] = _create_snap_app(app, assumes)

    if project.hooks and any(h for h in project.hooks.values() if h.command_chain):
        assumes.add("command-chain")

    environment = _populate_environment(project.environment, prime_dir, arch_triplet)
    version = process_version(project.version)

    snap_metadata = SnapMetadata(
        name=project.name,
        title=project.title,
        version=version,
        summary=project.summary,
        description=project.description,  # type: ignore
        license=project.license,
        type=project.type,
        architectures=[arch],
        base=cast(str, project.base),
        assumes=list(assumes) if assumes else None,
        epoch=project.epoch,
        apps=snap_apps or None,
        confinement=project.confinement,
        grade=_get_grade(project.grade, project.build_base),
        environment=environment,
        plugs=project.plugs,
        slots=project.slots,
        hooks=project.hooks,
        layout=project.layout,
        system_usernames=project.system_usernames,
        provenance=project.provenance,
    )
    if project.passthrough:
        for name, value in project.passthrough.items():
            setattr(snap_metadata, name, value)

    yaml.add_representer(str, _repr_str, Dumper=yaml.SafeDumper)
    yaml_data = snap_metadata.yaml(
        by_alias=True,
        exclude_none=True,
        allow_unicode=True,
        sort_keys=False,
        width=1000,
    )

    snap_yaml = meta_dir / "snap.yaml"
    snap_yaml.write_text(yaml_data)


def _repr_str(dumper, data):
    """Multi-line string representer for the YAML dumper."""
    if "\n" in data:
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)


def _populate_environment(
    environment: Optional[Dict[str, Optional[str]]], prime_dir: Path, arch_triplet: str
):
    """Populate default app environmental variables.

    Three cases for LD_LIBRARY_PATH and PATH variables:
        - If LD_LIBRARY_PATH or PATH are defined, keep user-defined values.
        - If LD_LIBRARY_PATH or PATH are not defined, set to default values.
        - If LD_LIBRARY_PATH or PATH are null, do not use default values.
    """
    if environment is None:
        return {
            "LD_LIBRARY_PATH": get_ld_library_paths(prime_dir, arch_triplet),
            "PATH": "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH",
        }

    try:
        if not environment["LD_LIBRARY_PATH"]:
            environment.pop("LD_LIBRARY_PATH")
    except KeyError:
        environment["LD_LIBRARY_PATH"] = get_ld_library_paths(prime_dir, arch_triplet)

    try:
        if not environment["PATH"]:
            environment.pop("PATH")
    except KeyError:
        environment["PATH"] = "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH"

    if len(environment):
        return environment

    # if the environment only contained a null LD_LIBRARY_PATH and a null PATH, return None
    return None
