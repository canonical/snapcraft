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

import copy
import re
from typing import Any, Literal, Mapping, Tuple, cast

import pydantic
from craft_application import models
from craft_application.errors import CraftValidationError
from craft_application.models import BuildInfo, SummaryStr, VersionStr
from craft_application.models.constraints import (
    SingleEntryDict,
    SingleEntryList,
    UniqueList,
)
from craft_cli import emit
from craft_grammar.models import Grammar  # type: ignore[import-untyped]
from craft_platforms import Platforms, snap
from craft_providers import bases
from pydantic import ConfigDict, PrivateAttr, StringConstraints
from typing_extensions import Annotated, Self, override

from snapcraft import utils
from snapcraft.const import SUPPORTED_ARCHS, SnapArch
from snapcraft.elf.elf_utils import get_arch_triplet
from snapcraft.errors import ProjectValidationError
from snapcraft.providers import SNAPCRAFT_BASE_TO_PROVIDER_BASE
from snapcraft.utils import (
    convert_architecture_deb_to_platform,
    get_effective_base,
    get_host_architecture,
    get_supported_architectures,
    is_architecture_supported,
)

ProjectName = Annotated[str, StringConstraints(max_length=40)]


def _validate_command_chain(command_chains: list[str] | None) -> list[str] | None:
    """Validate command_chain."""
    if command_chains is not None:
        for command_chain in command_chains:
            if not re.match(r"^[A-Za-z0-9/._#:$-]*$", command_chain):
                raise ValueError(
                    f"{command_chain!r} is not a valid command chain. Command chain entries must "
                    "be strings, and can only use ASCII alphanumeric characters and the following "
                    "special characters: / . _ # : $ -"
                )
    return command_chains


def validate_architectures(architectures):
    """Expand and validate architecture data.

    Validation includes:
        - The list cannot be a combination of strings and Architecture objects.
        - The same architecture cannot be defined in multiple `build-for` fields,
          even if the implicit values are used to define `build-for`.
        - Only one architecture can be defined in the `build-for` list.
        - The `all` keyword is properly used. (see `_validate_architectures_all_keyword()`)

    :raise ValueError: If architecture data is invalid.
    """
    if not architectures:
        return architectures

    # validate strings and Architecture objects are not mixed
    if not (
        all(isinstance(architecture, str) for architecture in architectures)
        or all(isinstance(architecture, Architecture) for architecture in architectures)
    ):
        raise ValueError(
            f"every item must either be a string or an object for {architectures!r}"
        )

    _expand_architectures(architectures)

    # validate `build_for` after expanding data
    if any(len(architecture.build_for) > 1 for architecture in architectures):
        raise ValueError("only one architecture can be defined for 'build-for'")

    _validate_architectures_all_keyword(architectures)

    if len(architectures) > 1:
        # validate multiple uses of the same architecture
        unique_build_fors = set()
        for element in architectures:
            for architecture in element.build_for:
                if architecture in unique_build_fors:
                    raise ValueError(
                        f"multiple items will build snaps that claim to run on {architecture}"
                    )
                unique_build_fors.add(architecture)

    # validate architectures are supported
    if len(architectures):
        for element in architectures:
            for arch in element.build_for + element.build_on:
                if arch != "all" and not is_architecture_supported(arch):
                    supported_archs = utils.humanize_list(
                        get_supported_architectures(), "and"
                    )
                    raise ValueError(
                        f"Architecture {arch!r} is not supported. Supported "
                        f"architectures are {supported_archs}."
                    )

    return architectures


def _expand_architectures(architectures):
    """Expand architecture data.

    Expansion to fully-defined Architecture objects includes the following:
        - strings (shortform notation) are converted to Architecture objects
        - `build-on` and `build-for` strings are converted to single item lists
        - Empty `build-for` fields are implicitly set to the same architecture used in `build-on`
    """
    for index, architecture in enumerate(architectures):
        # convert strings into Architecture objects
        if isinstance(architecture, str):
            architectures[index] = Architecture(
                build_on=cast(UniqueList[str], [architecture]),
                build_for=cast(UniqueList[str], [architecture]),
            )
        elif isinstance(architecture, Architecture):
            # convert strings to lists
            if isinstance(architecture.build_on, str):
                architectures[index].build_on = [architecture.build_on]
            if isinstance(architecture.build_for, str):
                architectures[index].build_for = [architecture.build_for]
            # implicitly set build_for from build_on
            elif architecture.build_for is None:
                architectures[index].build_for = architectures[index].build_on


def _validate_architectures_all_keyword(architectures):
    """Validate `all` keyword is properly used.

    Validation rules:
    - `all` cannot be used to `build-on`
    - If `all` is used for `build-for`, no other architectures can be defined
      for `build-for`.

    :raise ValueError: if `all` keyword isn't properly used.
    """
    # validate use of `all` inside each build-on list
    for architecture in architectures:
        if "all" in architecture.build_on:
            raise ValueError("'all' cannot be used for 'build-on'")

    # validate use of `all` across all items in architecture list
    if len(architectures) > 1:
        if any("all" in architecture.build_for for architecture in architectures):
            raise ValueError(
                "one of the items has 'all' in 'build-for', but there are"
                f" {len(architectures)} items: upon release they will conflict."
                "'all' should only be used if there is a single item"
            )


def apply_root_packages(yaml_data: dict[str, Any]) -> dict[str, Any]:
    """Create a new part with root level attributes.

    Root level attributes such as build-packages and build-snaps
    are known to Snapcraft but not Craft Parts. Create a new part
    "snapcraft/core" with these attributes and apply it to the
    current yaml_data.
    """
    if "build-packages" not in yaml_data and "build-snaps" not in yaml_data:
        return yaml_data

    yaml_data = copy.deepcopy(yaml_data)
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

    return yaml_data


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
    components_data: dict[str, Any] | None
) -> list[str] | None:
    """Get a list of partitions based on the project's components.

    :returns: A list of partitions formatted as ['default', 'component/<name>', ...]
    or None if no components are defined.
    """
    if components_data:
        return ["default", *[f"component/{name}" for name in components_data.keys()]]

    return None


def _validate_mandatory_base(base: str | None, snap_type: str | None) -> None:
    """Validate that the base is specified, if required by the snap_type."""
    if (base is not None) ^ (snap_type not in ["base", "kernel", "snapd"]):
        raise ValueError(
            "Snap base must be declared when type is not base, kernel or snapd"
        )


class Socket(models.CraftBaseModel):
    """Snapcraft app socket definition."""

    listen_stream: int | str
    socket_mode: int | None = None

    @pydantic.field_validator("listen_stream")
    @classmethod
    def _validate_list_stream(cls, listen_stream):
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

    ignore: list[str | dict[str, list[str]]]

    # A private field to simplify lookup.
    _lint_ignores: dict[str, list[str]] = PrivateAttr(default_factory=dict)

    def __eq__(self, other):
        """Compare two Lint objects and ignore private attributes."""
        return self.ignore == other.ignore

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        for item in self.ignore:
            if isinstance(item, str):
                self._lint_ignores[item] = []
            else:
                assert len(item) == 1, "Expected exactly one key in lint ignore entry."
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

    command: str
    autostart: str | None = None
    common_id: str | None = None
    bus_name: str | None = None
    desktop: str | None = None
    completer: str | None = None
    stop_command: str | None = None
    post_stop_command: str | None = None
    start_timeout: str | None = None
    stop_timeout: str | None = None
    watchdog_timeout: str | None = None
    reload_command: str | None = None
    restart_delay: str | None = None
    timer: str | None = None
    daemon: Literal["simple", "forking", "oneshot", "notify", "dbus"] | None = None
    after: UniqueList[str] = pydantic.Field(default_factory=list)
    before: UniqueList[str] = pydantic.Field(default_factory=list)
    refresh_mode: Literal["endure", "restart", "ignore-running"] | None = None
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
    ) = None
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
    ) = None
    install_mode: Literal["enable", "disable"] | None = None
    slots: UniqueList[str] | None = None
    plugs: UniqueList[str] | None = None
    aliases: UniqueList[str] | None = None
    environment: dict[str, str] | None = None
    command_chain: list[str] = []
    sockets: dict[str, Socket] | None = None
    daemon_scope: Literal["system", "user"] | None = None
    activates_on: UniqueList[str] | None = None
    passthrough: dict[str, Any] | None = None

    @pydantic.field_validator("autostart")
    @classmethod
    def _validate_autostart_name(cls, name):
        if not re.match(r"^[A-Za-z0-9. _#:$-]+\.desktop$", name):
            raise ValueError(
                f"{name!r} is not a valid desktop file name (e.g. myapp.desktop)"
            )

        return name

    @pydantic.field_validator(
        "command", "stop_command", "post_stop_command", "reload_command", "bus_name"
    )
    @classmethod
    def _validate_apps_section_content(
        cls, command: str, info: pydantic.ValidationInfo
    ) -> str:
        # Find any invalid characters in the command.
        # The regex below is derived from snapd's validator code, modified to be the inverse (^).
        # https://github.com/canonical/snapd/blob/0706e2d0b20ae2bf030863f142b8491b66e80bcb/snap/validate.go#L756
        if not re.match(r"^[A-Za-z0-9/. _#:$-]*$", command):
            # Guaranteed not-none as the pydantic field_validator decorator always populates it.
            deserialized = cast(str, info.field_name).replace("_", "-")
            message = f"{deserialized}: App commands must consist of only alphanumeric characters, spaces, and the following characters: / . _ # : $ -"
            raise ValueError(message)

        return command

    @pydantic.field_validator(
        "start_timeout", "stop_timeout", "watchdog_timeout", "restart_delay"
    )
    @classmethod
    def _validate_time(cls, timeval):
        if not re.match(r"^[0-9]+(ns|us|ms|s|m)*$", timeval):
            raise ValueError(f"{timeval!r} is not a valid time value")

        return timeval

    @pydantic.field_validator("command_chain")
    @classmethod
    def _validate_command_chain(cls, command_chains):
        return _validate_command_chain(command_chains)

    @pydantic.field_validator("aliases")
    @classmethod
    def _validate_aliases(cls, aliases):
        for alias in aliases:
            if not re.match(r"^[a-zA-Z0-9][-_.a-zA-Z0-9]*$", alias):
                raise ValueError(
                    f"{alias!r} is not a valid alias. Aliases must be strings, begin with an ASCII "
                    "alphanumeric character, and can only use ASCII alphanumeric characters and "
                    "the following special characters: . _ -"
                )

        return aliases


class Hook(models.CraftBaseModel):
    """Snapcraft project hook definition."""

    command_chain: list[str] | None = None
    environment: dict[str, str] | None = None
    plugs: UniqueList[str] | None = None
    passthrough: dict[str, Any] | None = None

    @pydantic.field_validator("command_chain")
    @classmethod
    def _validate_command_chain(cls, command_chains):
        return _validate_command_chain(command_chains)

    @pydantic.field_validator("plugs")
    @classmethod
    def _validate_plugs(cls, plugs):
        if not plugs:
            raise ValueError("'plugs' field cannot be empty.")
        return plugs


class Architecture(models.CraftBaseModel, extra="forbid"):
    """Snapcraft project architecture definition."""

    build_on: str | UniqueList[str]
    build_for: str | UniqueList[str] | None = None


class ContentPlug(models.CraftBaseModel):
    """Snapcraft project content plug definition."""

    content: str | None = None
    interface: str
    target: str
    default_provider: str | None = None

    @pydantic.field_validator("default_provider")
    @classmethod
    def _validate_default_provider(cls, default_provider):
        if default_provider and "/" in default_provider:
            raise ValueError(
                "Specifying a snap channel in 'default_provider' is not supported: "
                f"{default_provider}"
            )
        return default_provider


class Platform(models.Platform):
    """Snapcraft project platform definition."""

    build_on: UniqueList[str] | None = pydantic.Field(min_length=1)
    build_for: SingleEntryList | None = None

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

        This aligns with the precedent set by the `architectures` keyword.
        """
        if not values.get("build_on") and values.get("build_for"):
            raise CraftValidationError(
                "'build_for' expects 'build_on' to also be provided."
            )

        return values

    @classmethod
    def from_architectures(
        cls,
        architectures: list[str | Architecture],
    ) -> dict[str, Self]:
        """Convert a core22 architectures configuration to core24 platforms."""
        platforms: dict[str, Self] = {}
        for architecture in architectures:
            if isinstance(architecture, str):
                build_on = build_for = cast(UniqueList[str], [architecture])
            else:
                if isinstance(architecture.build_on, str):
                    build_on = build_for = cast(
                        UniqueList[str], [architecture.build_on]
                    )
                else:
                    build_on = build_for = cast(UniqueList[str], architecture.build_on)
                if architecture.build_for:
                    if isinstance(architecture.build_for, str):
                        build_for = cast(UniqueList[str], [architecture.build_for])
                    else:
                        build_for = cast(UniqueList[str], architecture.build_for)

            platforms[build_for[0]] = cls(build_for=build_for, build_on=build_on)

        return platforms


class Component(models.CraftBaseModel):
    """Snapcraft component definition."""

    summary: SummaryStr
    description: str
    type: Literal["test", "kernel-modules", "standard"]
    version: VersionStr | None = None
    hooks: dict[str, Hook] | None = None


MANDATORY_ADOPTABLE_FIELDS = ("version", "summary", "description")


class Project(models.Project):
    """Snapcraft project definition.

    See https://snapcraft.io/docs/snapcraft-yaml-reference

    XXX: Not implemented in this version
    - system-usernames
    """

    # snapcraft's `name` is more general than craft-application
    name: ProjectName  # type: ignore[assignment]
    build_base: str | None = pydantic.Field(validate_default=True, default=None)
    compression: Literal["lzo", "xz"] = "xz"
    version: VersionStr | None = None
    donation: UniqueList[str] | None = None
    # snapcraft's `source_code` is more general than craft-application
    source_code: UniqueList[str] | None = None  # type: ignore[assignment]
    contact: UniqueList[str] | None = None  # type: ignore[assignment]
    issues: UniqueList[str] | None = None  # type: ignore[assignment]
    website: UniqueList[str] | None = None
    type: Literal["app", "base", "gadget", "kernel", "snapd"] | None = None
    icon: str | None = None
    confinement: Literal["classic", "devmode", "strict"]
    layout: (
        dict[str, SingleEntryDict[Literal["symlink", "bind", "bind-file", "type"], str]]
        | None
    ) = None
    grade: Literal["stable", "devel"] | None = None
    architectures: list[str | Architecture] | None = None
    _architectures_in_yaml: bool | None = None
    platforms: dict[str, Platform] | None = None  # type: ignore[assignment,reportIncompatibleVariableOverride]
    assumes: UniqueList[str] = pydantic.Field(default_factory=list)
    hooks: dict[str, Hook] | None = None
    passthrough: dict[str, Any] | None = None
    apps: dict[str, App] | None = None
    plugs: dict[str, ContentPlug | Any] | None = None
    slots: dict[str, Any] | None = None
    lint: Lint | None = None
    epoch: str | None = None
    adopt_info: str | None = None
    system_usernames: dict[str, Any] | None = None
    environment: dict[str, str | None] | None = None
    build_packages: Grammar[list[str]] | None = None
    build_snaps: Grammar[list[str]] | None = None
    ua_services: set[str] | None = None
    provenance: str | None = None
    components: dict[ProjectName, Component] | None = None

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
    def _validate_plugs(cls, plugs):
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
            message = _format_global_keyword_warning("plug", empty_plugs)
            emit.message(message)

        return plugs

    @pydantic.field_validator("slots")
    @classmethod
    def _validate_slots(cls, slots):
        empty_slots = []
        if slots is not None:
            for slot_name, slot in slots.items():
                if slot is None:
                    empty_slots.append(slot_name)

        if empty_slots:
            message = _format_global_keyword_warning("slot", empty_slots)
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

    @pydantic.model_validator(mode="after")
    def _validate_mandatory_base(self):
        _validate_mandatory_base(self.base, self.type)
        return self

    @pydantic.field_validator("name")
    @classmethod
    def _validate_snap_name(cls, name):
        return validate_name(name=name, field_name="snap")

    @pydantic.field_validator("components")
    @classmethod
    def _validate_components(cls, components):
        """Validate component names."""
        for component_name in components.keys():
            _validate_component(name=component_name)

        return components

    @pydantic.model_validator(mode="after")
    def _validate_platforms_and_architectures(self) -> Self:
        """Validate usage of platforms and architectures.

        core22 base:
         - can optionally define architectures
         - cannot define platforms

        core24 and newer bases:
         - cannot define architectures
         - can optionally define platforms
        """
        base = get_effective_base(
            base=self.base,
            build_base=self.build_base,
            project_type=self.type,
            name=self.name,
        )
        if base == "core22":
            if self.platforms:
                raise ValueError(
                    f"'platforms' keyword is not supported for base {base!r}. "
                    "Use 'architectures' keyword instead."
                )

            # this is a one-shot - the value should not change when re-validating
            if self.architectures and self._architectures_in_yaml is None:
                # record if architectures are defined in the yaml for remote-build (#4881)
                self._architectures_in_yaml = True
            elif not self.architectures:
                self._architectures_in_yaml = False
                # set default value
                self.architectures = [
                    Architecture(
                        build_on=[get_host_architecture()],
                        build_for=[get_host_architecture()],
                    )
                ]

        elif self.architectures:
            raise ValueError(
                f"'architectures' keyword is not supported for base {base!r}. "
                "Use 'platforms' keyword instead."
            )

        return self

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
    def _validate_epoch(cls, epoch):
        """Verify epoch format."""
        if epoch is not None and not re.match(r"^(?:0|[1-9][0-9]*[*]?)$", epoch):
            raise ValueError(
                "Epoch is a positive integer followed by an optional asterisk"
            )

        return epoch

    @pydantic.field_validator("architectures")
    @classmethod
    def _validate_architecture_data(cls, architectures):
        """Validate architecture data."""
        return validate_architectures(architectures)

    @pydantic.field_validator("provenance")
    @classmethod
    def _validate_provenance(cls, provenance):
        if provenance and not re.match(r"^[a-zA-Z0-9-]+$", provenance):
            raise ValueError(
                "provenance must consist of alphanumeric characters and/or hyphens."
            )

        return provenance

    @pydantic.field_validator(
        "contact", "donation", "issues", "source_code", "website", mode="before"
    )
    @classmethod
    def _validate_urls(cls, field_value):
        if isinstance(field_value, str):
            field_value = cast(UniqueList[str], [field_value])
        return field_value

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
            project_type=self.type,
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
            return get_arch_triplet(convert_architecture_deb_to_platform(arch))

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

    parts: dict[str, _GrammarAwarePart]

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
        default=[get_host_architecture()],
        validate_default=True,
    )

    @pydantic.field_validator("architectures")
    @classmethod
    def _validate_architecture_data(cls, architectures):
        """Validate architecture data."""
        return validate_architectures(architectures)


class ComponentProject(models.CraftBaseModel, extra="ignore"):
    """Project definition containing only component data."""

    components: dict[ProjectName, Component] | None = None

    @pydantic.field_validator("components")
    @classmethod
    def _validate_components(cls, components):
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


def _format_pydantic_errors(errors, *, file_name: str = "snapcraft.yaml"):
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


def _format_pydantic_error_location(loc):
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


def _format_pydantic_error_message(msg):
    """Format pydantic's error message field."""
    # Replace shorthand "str" with "string".
    msg = msg.replace("str type expected", "string type expected")
    return msg


def _printable_field_location_split(location: str) -> Tuple[str, str]:
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


def _format_global_keyword_warning(keyword: str, empty_entries: list[str]) -> str:
    """Create a warning message about global assignment in the ``keyword`` field.

    :param keyword:
        The top-level keyword that contains empty entries (currently either
        "plug" or "slot").
    :param empty_entries:
        The entries inside the ``keyword`` dict that are empty.
    :return:
        A properly-formatted warning message.
    """
    culprits = utils.humanize_list(empty_entries, "and")
    return (
        f"Warning: implicit {keyword.lower()} assignment in {culprits}. "
        f"{keyword.capitalize()}s should be assigned to the app to which they apply, "
        f"and not implicitly assigned via the global '{keyword.lower()}s:' "
        "stanza which is intended for configuration only."
        "\n(Reference: https://snapcraft.io/docs/snapcraft-top-level-metadata"
        "#heading--plugs-and-slots-for-an-entire-snap)"
    )


class SnapcraftBuildPlanner(models.BuildPlanner):
    """A project model that creates build plans."""

    base: str | None = None
    build_base: str | None = None
    name: str
    type: Literal["app", "base", "gadget", "kernel", "snapd"] | None = None
    platforms: dict[str, Platform] | None = None  # type: ignore[assignment]
    architectures: list[str | Architecture] | None = None
    project_type: str | None = pydantic.Field(default=None, alias="type")

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

    def get_build_plan(self) -> list[BuildInfo]:
        """Get the build plan for this project."""
        effective_base = SNAPCRAFT_BASE_TO_PROVIDER_BASE[
            str(
                get_effective_base(
                    base=self.base,
                    build_base=self.build_base,
                    project_type=self.project_type,
                    name=self.name,
                    translate_devel=False,  # We want actual "devel" if set.
                )
            )
        ].value

        # set default value
        if self.platforms is None:
            self.platforms = {
                get_host_architecture(): Platform(
                    build_on=[SnapArch(get_host_architecture()).value],
                    build_for=[SnapArch(get_host_architecture()).value],
                )
            }
            # For backwards compatibility with core22, convert the platforms.
            if effective_base == "22.04" and self.architectures:
                self.platforms = (  # type: ignore[reportIncompatibleVariableOverride]
                    Platform.from_architectures(self.architectures)
                )

        platforms = cast(
            Platforms,
            {name: platform.marshal() for name, platform in self.platforms.items()},
        )

        # In _validate_mandatory_base, we ensure that the possible values of
        # 'base' and 'snap_type' are narrowed so they'll always match one of
        # the two overloads of get_platforms_snap_build_plan.  But, pyright and
        # mypy aren't smart enough to realize this, so we need the type checker
        # ignores.
        _validate_mandatory_base(self.base, self.type)
        return [
            BuildInfo(
                platform=buildinfo.platform,
                build_on=str(buildinfo.build_on),
                build_for=str(buildinfo.build_for),
                base=bases.BaseName(
                    name=buildinfo.build_base.distribution,
                    version=buildinfo.build_base.series,
                ),
            )
            for buildinfo in snap.get_platforms_snap_build_plan(  # pyright: ignore[reportCallIssue]
                base=self.base,  # type: ignore[arg-type]
                build_base=self.build_base,
                snap_type=self.type,  # type: ignore[arg-type]
                platforms=platforms,
            )
        ]
