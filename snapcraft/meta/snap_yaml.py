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

"""Create snap.yaml metadata file."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Literal, cast

import pydantic
import yaml
from craft_application.models import BaseMetadata, SummaryStr, base, constraints
from craft_application.models.constraints import (
    SingleEntryDict,  # noqa: TC002 (typing-only-third-party-import) # pydantic needs to import types at runtime for validation
)
from craft_cli import emit
from craft_platforms import DebianArchitecture

from snapcraft import errors, models
from snapcraft.elf.elf_utils import get_arch_triplet
from snapcraft.utils import (
    get_ld_library_paths,
    process_version,
)


class SnapcraftMetadata(BaseMetadata):
    """Snapcraft-specific metadata base model."""

    model_config = pydantic.ConfigDict(
        validate_assignment=True,
        extra="allow",
        populate_by_name=True,
        alias_generator=base.alias_generator,
        # snap metadata may contain versions as ints or floats
        coerce_numbers_to_str=True,
    )

    def marshal(self) -> dict[str, str | list[str] | dict[str, Any]]:
        """Convert to a dictionary and exclude defaults."""
        return self.model_dump(
            mode="json", by_alias=True, exclude_unset=True, exclude_defaults=True
        )


class Socket(SnapcraftMetadata):
    """snap.yaml app socket entry."""

    listen_stream: int | str
    socket_mode: int | None = None


class SnapApp(SnapcraftMetadata):
    """Snap.yaml app entry.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.

    TODO: implement desktop (CRAFT-804)
    TODO: implement extensions (CRAFT-805)
    """

    command: str
    autostart: str | None = None
    common_id: str | None = None
    bus_name: str | None = None
    completer: str | None = None
    stop_command: str | None = None
    post_stop_command: str | None = None
    start_timeout: str | None = None
    stop_timeout: str | None = None
    watchdog_timeout: str | None = None
    reload_command: str | None = None
    restart_delay: str | None = None
    timer: str | None = None
    daemon: str | None = None
    after: list[str] | None = None
    before: list[str] | None = None
    refresh_mode: str | None = None
    stop_mode: str | None = None
    restart_condition: str | None = None
    success_exit_status: list[int] | None = None
    install_mode: str | None = None
    plugs: list[str] | None = None
    slots: list[str] | None = None
    aliases: list[str] | None = None
    environment: dict[str, Any] | None = None
    command_chain: list[str] | None = None
    sockets: dict[str, Socket] | None = None
    daemon_scope: str | None = None
    activates_on: list[str] | None = None


class ContentPlug(SnapcraftMetadata):
    """Content plug definition in the snap metadata."""

    model_config = pydantic.ConfigDict(
        validate_assignment=True,
        # allow extra parameters in content plugs
        extra="ignore",
        populate_by_name=True,
        alias_generator=base.alias_generator,
    )

    interface: Literal["content"]
    target: str
    content: str | None = None
    default_provider: str | None = None

    @pydantic.field_validator("target")
    @classmethod
    def _validate_target_not_empty(cls, val: str) -> str:
        if val == "":
            raise ValueError("value cannot be empty")
        return val

    @pydantic.field_validator("default_provider")
    @classmethod
    def _validate_default_provider(cls, default_provider: str) -> str:
        if default_provider and "/" in default_provider:
            raise ValueError(
                "Specifying a snap channel in 'default_provider' is not supported: "
                f"{default_provider}"
            )
        return default_provider

    @property
    def provider(self) -> str | None:
        """Return the default content provider name."""
        if self.default_provider is None:
            return None

        # ignore :<slot> if present
        if ":" in self.default_provider:
            return self.default_provider.split(":")[0]

        return self.default_provider


class ContentSlot(SnapcraftMetadata):
    """Content slot definition in the snap metadata."""

    interface: Literal["content"]
    content: str | None = None
    read: list[str] = []
    write: list[str] = []

    def get_content_dirs(self, installed_path: Path) -> set[Path]:
        """Obtain the slot's content directories."""
        content_dirs: set[Path] = set()

        for path_ in self.read + self.write:
            # Strip leading "$SNAP" and "/".
            path = re.sub(r"^\$SNAP", "", path_)
            path = re.sub(r"^/", "", path)
            path = re.sub(r"^./", "", path)
            content_dirs.add(installed_path / path)

        return content_dirs


class Links(SnapcraftMetadata):
    """Metadata links used in snaps."""

    contact: constraints.UniqueStrList | None = None
    donation: constraints.UniqueStrList | None = None
    issues: constraints.UniqueStrList | None = None
    source_code: constraints.UniqueStrList | None = None
    website: constraints.UniqueStrList | None = None

    @staticmethod
    def _normalize_value(
        value: str | constraints.UniqueStrList | None,
    ) -> constraints.UniqueStrList | None:
        result: constraints.UniqueStrList | None
        if isinstance(value, str):
            result = cast(constraints.UniqueStrList, [value])
        else:
            result = value
        return result

    @classmethod
    def from_project(cls, project: models.Project) -> Links:
        """Create Links from a Project."""
        return cls(
            contact=cls._normalize_value(project.contact),
            donation=cls._normalize_value(project.donation),
            issues=cls._normalize_value(project.issues),
            source_code=cls._normalize_value(project.source_code),
            website=cls._normalize_value(project.website),
        )

    def __bool__(self) -> bool:
        """Return True if any of the Links attributes are set."""
        return any(
            [self.contact, self.donation, self.issues, self.source_code, self.website]
        )


class ComponentMetadata(SnapcraftMetadata):
    """Component metadata model.

    This model contains different information than the model in the
    `component_yaml` module. For example, version and architecture metadata
    is not included.
    """

    summary: SummaryStr
    description: str
    type: str
    hooks: dict[str, models.Hook] | None = None

    model_config = pydantic.ConfigDict(
        validate_assignment=True,
        # ignore extra parameters in component metadata
        extra="ignore",
        populate_by_name=True,
        alias_generator=base.alias_generator,
    )

    @classmethod
    def from_component(cls, component: models.Component) -> ComponentMetadata:
        """Create a ComponentMetadata model from a Component model."""
        return cls.unmarshal(component.marshal())


class SnapMetadata(SnapcraftMetadata):
    """The snap.yaml model.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.

    TODO: should platforms replace architectures for core24?
    """

    name: str
    title: str | None = None
    version: str
    summary: SummaryStr
    description: str
    license: str | None = None
    type: str | None = None
    architectures: list[str]
    base: str | None = None
    assumes: list[str] | None = None
    epoch: str | None = None
    apps: dict[str, SnapApp] | None = None
    confinement: str
    grade: str
    environment: dict[str, Any] | None = None
    plugs: dict[str, Any] | None = None
    slots: dict[str, Any] | None = None
    hooks: dict[str, Any] | None = None
    layout: (
        dict[str, SingleEntryDict[Literal["symlink", "bind", "bind-file", "type"], str]]
        | None
    ) = None
    system_usernames: dict[str, Any] | None = None
    provenance: str | None = None
    links: Links | None = None
    components: dict[str, ComponentMetadata] | None = None

    def get_provider_content_directories(self) -> list[Path]:
        """Get provider content directories from installed snaps."""
        provider_dirs: set[Path] = set()

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

    def get_content_plugs(self) -> list[ContentPlug]:
        """Return a list of content plugs from the snap metadata plugs."""
        if not self.plugs:
            return []

        content_plugs: list[ContentPlug] = []
        for name, data in self.plugs.items():
            try:
                plug = ContentPlug.unmarshal(data)
                if not plug.content:
                    plug.content = name
                content_plugs.append(plug)
            except (TypeError, pydantic.ValidationError):
                continue
        return content_plugs

    def get_content_slots(self) -> list[ContentSlot]:
        """Return a list of content slots from the snap metadata slots."""
        if not self.slots:
            return []

        content_slots: list[ContentSlot] = []
        for name, slot_data in self.slots.items():
            try:
                slot = ContentSlot.unmarshal(slot_data)
                if not slot.content:
                    slot.content = name
                content_slots.append(slot)
            except (TypeError, pydantic.ValidationError):
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


def _create_snap_app(app: models.App, assumes: set[str]) -> SnapApp:
    app_sockets: dict[str, Socket] = {}
    if app.sockets:
        for socket_name, socket in app.sockets.items():
            app_sockets[socket_name] = Socket(
                listen_stream=socket.listen_stream,
                socket_mode=socket.socket_mode,
            )

    if app.command_chain:
        assumes.add("command-chain")

    if app.success_exit_status:
        assumes.add("snapd2.74")

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
        success_exit_status=app.success_exit_status,
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


def _get_grade(grade: str | None = None, build_base: str | None = None) -> str:
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


def get_metadata_from_project(
    project: models.Project, prime_dir: Path, *, arch: str
) -> SnapMetadata:
    """Get a SnapMetadata object from a project.

    :param project: Snapcraft project.
    :param prime_dir: The directory containing the content to be snapped.
    :param arch: Target architecture the snap project is built to.
    """
    assumes: set[str] = set()

    snap_apps: dict[str, SnapApp] = {}
    if project.apps:
        for name, app in project.apps.items():
            snap_apps[name] = _create_snap_app(app, assumes)

    if project.hooks and any(h for h in project.hooks.values() if h.command_chain):
        assumes.add("command-chain")

    if arch == "all":
        # if arch is "all", do not include architecture-specific paths in the environment
        arch_triplet: str | None = None
    elif isinstance(project, models.Core22Project):
        arch_triplet = project.get_build_for_arch_triplet()
    else:
        arch_triplet = get_arch_triplet(
            DebianArchitecture.from_machine(arch).to_platform_arch()
        )

    environment = _populate_environment(
        project.environment, prime_dir, arch_triplet, project.confinement
    )
    version = process_version(project.version)
    components = _process_components(project.components)

    # project provided assumes and computed assumes
    total_assumes = sorted(project.assumes + list(assumes))

    links = Links.from_project(project)
    snap_type = project.type.value if project.type else None

    snap_metadata = SnapMetadata(
        name=project.name,
        title=project.title,
        version=version,
        summary=cast(SummaryStr, project.summary),
        description=cast(str, project.description),
        license=project.license,
        type=snap_type,
        architectures=[arch],
        base=cast(str, project.base),
        assumes=total_assumes if total_assumes else None,
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
        links=links if links else None,
        components=components,
    )
    if project.passthrough:
        for name, value in project.passthrough.items():
            setattr(snap_metadata, name, value)

    return snap_metadata


def write(project: models.Project, prime_dir: Path, *, arch: str):
    """Create a snap.yaml file.

    :param project: Snapcraft project.
    :param prime_dir: The directory containing the content to be snapped.
    :param arch: Target architecture the snap project is built to.
    """
    snap_metadata = get_metadata_from_project(project, prime_dir, arch=arch)

    meta_dir = prime_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    snap_metadata.to_yaml_file(meta_dir / "snap.yaml")


def _repr_str(dumper: yaml.Dumper, data: str):
    """Multi-line string representer for the YAML dumper."""
    if "\n" in data:
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)


def _populate_environment(
    environment: dict[str, str | None] | None,
    prime_dir: Path,
    arch_triplet: str | None,
    confinement: str,
) -> dict[str, str | None] | None:
    """Populate default app environment variables, LD_LIBRARY_PATH and PATH.

    Three cases for environment variables:
      - If defined, keep user-defined value.
      - If not defined, set to default value.
      - If null, do not use default value.

    :param environment: Dictionary of environment variables from the project.
    :param prime_dir: The directory containing the content to be snapped.
    :param arch_triplet: Architecture triplet of the target arch. If None, the
    environment will not contain architecture-specific paths.
    :param confinement: If classically confined, then no default values will be used.

    :returns: Dictionary of environment variables or None if all envvars are null or
    confinement is classic.
    """
    if environment is None:
        if confinement == "classic":
            return None
        return {
            "LD_LIBRARY_PATH": get_ld_library_paths(prime_dir, arch_triplet),
            "PATH": "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH",
        }

    # if LD_LIBRARY_PATH is not defined, use default value when not classic
    if "LD_LIBRARY_PATH" not in environment and confinement != "classic":
        environment["LD_LIBRARY_PATH"] = get_ld_library_paths(prime_dir, arch_triplet)
    # else if null, then remove from environment
    elif "LD_LIBRARY_PATH" in environment and not environment["LD_LIBRARY_PATH"]:
        del environment["LD_LIBRARY_PATH"]

    # if PATH is not defined, use default value when not classic
    if "PATH" not in environment and confinement != "classic":
        environment["PATH"] = "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH"
    # else if null, then remove from environment
    elif "PATH" in environment and not environment["PATH"]:
        del environment["PATH"]

    return environment if environment else None


def _process_components(
    components: dict[str, models.Component] | None,
) -> dict[str, ComponentMetadata] | None:
    """Convert Components from a project to ComponentMetadata for a snap.yaml.

    :param components: Component data from a project model.

    :returns: A dictionary of ComponentMetadata or None if no components are defined.
    """
    if not components:
        return None

    return {
        name: ComponentMetadata.from_component(data)
        for name, data in components.items()
    }
