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

"""Project file definition and helpers."""

import re
from typing import TYPE_CHECKING, Any, Dict, List, Literal, Optional, Tuple, Union, cast

import pydantic
from craft_application import models
from craft_application.models import BuildInfo, UniqueStrList
from craft_archives import repo
from craft_cli import emit
from craft_grammar.models import GrammarSingleEntryDictList, GrammarStr, GrammarStrList
from pydantic import PrivateAttr, constr

from snapcraft import parts, utils
from snapcraft.elf.elf_utils import get_arch_triplet
from snapcraft.errors import ProjectValidationError
from snapcraft.utils import (
    convert_architecture_deb_to_platform,
    get_effective_base,
    get_host_architecture,
    get_supported_architectures,
    is_architecture_supported,
)

# A workaround for mypy false positives
# see https://github.com/samuelcolvin/pydantic/issues/975#issuecomment-551147305
# fmt: off
if TYPE_CHECKING:
    ProjectName = str
    ProjectVersion = str
else:
    ProjectName = constr(max_length=40)
    ProjectVersion = constr(max_length=32, strict=True)
# fmt: on


def _validate_command_chain(command_chains: Optional[List[str]]) -> Optional[List[str]]:
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


def _validate_architectures(architectures):
    """Expand and validate architecture data.

    Validation includes:
        - The list cannot be a combination of strings and Architecture objects.
        - The same architecture cannot be defined in multiple `build-for` fields,
          even if the implicit values are used to define `build-for`.
        - Only one architecture can be defined in the `build-for` list.
        - The `all` keyword is properly used. (see `_validate_architectures_all_keyword()`)

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
                build_on=cast(UniqueStrList, [architecture]),
                build_for=cast(UniqueStrList, [architecture]),
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


class Socket(models.CraftBaseModel):
    """Snapcraft app socket definition."""

    listen_stream: Union[int, str]
    socket_mode: Optional[int]

    @pydantic.validator("listen_stream")
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

    ignore: List[Union[str, Dict[str, List[str]]]]

    # A private field to simplify lookup.
    _lint_ignores: Dict[str, List[str]] = PrivateAttr(default_factory=dict)

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

    def ignored_files(self, linter_name: str) -> List[str]:
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
    autostart: Optional[str]
    common_id: Optional[str]
    bus_name: Optional[str]
    desktop: Optional[str]
    completer: Optional[str]
    stop_command: Optional[str]
    post_stop_command: Optional[str]
    start_timeout: Optional[str]
    stop_timeout: Optional[str]
    watchdog_timeout: Optional[str]
    reload_command: Optional[str]
    restart_delay: Optional[str]
    timer: Optional[str]
    daemon: Optional[Literal["simple", "forking", "oneshot", "notify", "dbus"]]
    after: UniqueStrList = cast(UniqueStrList, [])
    before: UniqueStrList = cast(UniqueStrList, [])
    refresh_mode: Optional[Literal["endure", "restart"]]
    stop_mode: Optional[
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
    ]
    restart_condition: Optional[
        Literal[
            "on-success",
            "on-failure",
            "on-abnormal",
            "on-abort",
            "on-watchdog",
            "always",
            "never",
        ]
    ]
    install_mode: Optional[Literal["enable", "disable"]]
    slots: Optional[UniqueStrList]
    plugs: Optional[UniqueStrList]
    aliases: Optional[UniqueStrList]
    environment: Optional[Dict[str, str]]
    command_chain: List[str] = []
    sockets: Optional[Dict[str, Socket]]
    daemon_scope: Optional[Literal["system", "user"]]
    activates_on: Optional[UniqueStrList]
    passthrough: Optional[Dict[str, Any]]

    @pydantic.validator("autostart")
    @classmethod
    def _validate_autostart_name(cls, name):
        if not re.match(r"^[A-Za-z0-9. _#:$-]+\.desktop$", name):
            raise ValueError(
                f"{name!r} is not a valid desktop file name (e.g. myapp.desktop)"
            )

        return name

    @pydantic.validator("bus_name")
    @classmethod
    def _validate_bus_name(cls, name):
        if not re.match(r"^[A-Za-z0-9/. _#:$-]*$", name):
            raise ValueError(f"{name!r} is not a valid bus name")

        return name

    @pydantic.validator(
        "start_timeout", "stop_timeout", "watchdog_timeout", "restart_delay"
    )
    @classmethod
    def _validate_time(cls, timeval):
        if not re.match(r"^[0-9]+(ns|us|ms|s|m)*$", timeval):
            raise ValueError(f"{timeval!r} is not a valid time value")

        return timeval

    @pydantic.validator("command_chain")
    @classmethod
    def _validate_command_chain(cls, command_chains):
        return _validate_command_chain(command_chains)

    @pydantic.validator("aliases")
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

    command_chain: Optional[List[str]]
    environment: Optional[Dict[str, str]]
    plugs: Optional[UniqueStrList]
    passthrough: Optional[Dict[str, Any]]

    @pydantic.validator("command_chain")
    @classmethod
    def _validate_command_chain(cls, command_chains):
        return _validate_command_chain(command_chains)

    @pydantic.validator("plugs")
    @classmethod
    def _validate_plugs(cls, plugs):
        if not plugs:
            raise ValueError("'plugs' field cannot be empty.")
        return plugs


class Architecture(models.CraftBaseModel, extra=pydantic.Extra.forbid):
    """Snapcraft project architecture definition."""

    build_on: Union[str, UniqueStrList]
    build_for: Optional[Union[str, UniqueStrList]]


class ContentPlug(models.CraftBaseModel):
    """Snapcraft project content plug definition."""

    content: Optional[str]
    interface: str
    target: str
    default_provider: Optional[str]

    @pydantic.validator("default_provider")
    @classmethod
    def _validate_default_provider(cls, default_provider):
        if default_provider and "/" in default_provider:
            raise ValueError(
                "Specifying a snap channel in 'default_provider' is not supported: "
                f"{default_provider}"
            )
        return default_provider


MANDATORY_ADOPTABLE_FIELDS = ("version", "summary", "description")


class Project(models.Project):
    """Snapcraft project definition.

    See https://snapcraft.io/docs/snapcraft-yaml-reference

    XXX: Not implemented in this version
    - system-usernames
    """

    # snapcraft's `name` is more general than craft-application
    name: ProjectName  # type: ignore[assignment]
    build_base: Optional[str]
    compression: Literal["lzo", "xz"] = "xz"
    # TODO: ensure we have a test for version being retrieved using adopt-info
    # snapcraft's `version` is more general than craft-application
    version: Optional[ProjectVersion]  # type: ignore[assignment]
    donation: Optional[Union[str, UniqueStrList]]
    # snapcraft's `source_code` is more general than craft-application
    source_code: Optional[str]  # type: ignore[assignment]
    website: Optional[str]
    type: Optional[Literal["app", "base", "gadget", "kernel", "snapd"]]
    icon: Optional[str]
    confinement: Literal["classic", "devmode", "strict"]
    layout: Optional[
        Dict[str, Dict[Literal["symlink", "bind", "bind-file", "type"], str]]
    ]
    grade: Optional[Literal["stable", "devel"]]
    architectures: List[Union[str, Architecture]] = [get_host_architecture()]
    assumes: UniqueStrList = cast(UniqueStrList, [])
    package_repositories: List[Dict[str, Any]] = []  # handled by repo
    hooks: Optional[Dict[str, Hook]]
    passthrough: Optional[Dict[str, Any]]
    apps: Optional[Dict[str, App]]
    plugs: Optional[Dict[str, Union[ContentPlug, Any]]]
    slots: Optional[Dict[str, Any]]
    lint: Optional[Lint]
    epoch: Optional[str]
    adopt_info: Optional[str]
    system_usernames: Optional[Dict[str, Any]]
    environment: Optional[Dict[str, Optional[str]]]
    build_packages: Optional[GrammarStrList]
    build_snaps: Optional[GrammarStrList]
    ua_services: Optional[UniqueStrList]
    provenance: Optional[str]

    @pydantic.validator("plugs")
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

    @pydantic.validator("slots")
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

    @pydantic.root_validator(pre=True)
    @classmethod
    def _validate_adoptable_fields(cls, values):
        for field in MANDATORY_ADOPTABLE_FIELDS:
            if field not in values and "adopt-info" not in values:
                raise ValueError(f"Snap {field} is required if not using adopt-info")
        return values

    @pydantic.root_validator(pre=True)
    @classmethod
    def _validate_mandatory_base(cls, values):
        snap_type = values.get("type")
        base = values.get("base")
        if (base is not None) ^ (snap_type not in ["base", "kernel", "snapd"]):
            raise ValueError(
                "Snap base must be declared when type is not base, kernel or snapd"
            )
        return values

    @pydantic.validator("name")
    @classmethod
    def _validate_name(cls, name):
        if not re.match(r"^[a-z0-9-]*[a-z][a-z0-9-]*$", name):
            raise ValueError(
                "Snap names can only use ASCII lowercase letters, numbers, and hyphens, "
                "and must have at least one letter"
            )

        if name.startswith("-"):
            raise ValueError("Snap names cannot start with a hyphen")

        if name.endswith("-"):
            raise ValueError("Snap names cannot end with a hyphen")

        if "--" in name:
            raise ValueError("Snap names cannot have two hyphens in a row")

        return name

    @pydantic.validator("version")
    @classmethod
    def _validate_version(cls, version, values):
        if not version and "adopt_info" not in values:
            raise ValueError("Version must be declared if not adopting metadata")

        if version and not re.match(
            r"^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$", version
        ):
            raise ValueError(
                f"Invalid version '{version}': Snap versions consist of upper- and lower-case "
                "alphanumeric characters, as well as periods, colons, plus signs, tildes, "
                "and hyphens. They cannot begin with a period, colon, plus sign, tilde, or "
                "hyphen. They cannot end with a period, colon, or hyphen"
            )

        return version

    @pydantic.validator("grade", "summary", "description")
    @classmethod
    def _validate_adoptable_field(cls, field_value, values, field):
        if not field_value and "adopt_info" not in values:
            raise ValueError(
                f"{field.name.capitalize()} must be declared if not adopting metadata"
            )
        return field_value

    @pydantic.root_validator()
    @classmethod
    def _validate_grade_and_build_base(cls, values):
        """If build_base is devel, then grade must be devel."""
        if values.get("build_base") == "devel" and values.get("grade") == "stable":
            raise ValueError("grade must be 'devel' when build-base is 'devel'")
        return values

    @pydantic.validator("base", always=True)
    @classmethod
    def _validate_base(cls, base, values):
        """Not allowed to use unstable base without devel build-base."""
        if values.get("base") == "core24" and values.get("build_base") != "devel":
            raise ValueError("build-base must be 'devel' when base is 'core24'")

        return base

    @pydantic.validator("build_base", always=True)
    @classmethod
    def _validate_build_base(cls, build_base, values):
        """Build-base defaults to the base value if not specified."""
        if not build_base:
            build_base = values.get("base")
        return build_base

    @pydantic.validator("package_repositories", each_item=True)
    @classmethod
    def _validate_package_repositories(cls, item):
        """Ensure package-repositories format is correct."""
        repo.validate_repository(item)
        return item

    @pydantic.validator("parts", each_item=True)
    @classmethod
    def _validate_parts(cls, item):
        """Verify each part (craft-parts will re-validate this)."""
        parts.validate_part(item)
        return item

    @pydantic.validator("epoch")
    @classmethod
    def _validate_epoch(cls, epoch):
        """Verify epoch format."""
        if epoch is not None and not re.match(r"^(?:0|[1-9][0-9]*[*]?)$", epoch):
            raise ValueError(
                "Epoch is a positive integer followed by an optional asterisk"
            )

        return epoch

    @pydantic.validator("architectures", always=True)
    @classmethod
    def _validate_architecture_data(cls, architectures):
        """Validate architecture data."""
        return _validate_architectures(architectures)

    @pydantic.validator("provenance")
    @classmethod
    def _validate_provenance(cls, provenance):
        if provenance and not re.match(r"^[a-zA-Z0-9-]+$", provenance):
            raise ValueError(
                "provenance must consist of alphanumeric characters and/or hyphens."
            )

        return provenance

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "Project":
        """Create and populate a new ``Project`` object from dictionary data.

        The unmarshal method validates entries in the input dictionary, populating
        the corresponding fields in the data object.

        :param data: The dictionary data to unmarshal.

        :return: The newly created object.

        :raise TypeError: If data is not a dictionary.
        """
        if not isinstance(data, dict):
            raise TypeError("Project data is not a dictionary")

        try:
            project = Project(**data)
        except pydantic.ValidationError as err:
            raise ProjectValidationError(_format_pydantic_errors(err.errors())) from err

        return project

    def _get_content_plugs(self) -> List[ContentPlug]:
        """Get list of content plugs."""
        if self.plugs is not None:
            return [
                plug for plug in self.plugs.values() if isinstance(plug, ContentPlug)
            ]
        return []

    def get_content_snaps(self) -> List[str]:
        """Get list of snaps from ContentPlug `default-provider` fields."""
        return [
            x.default_provider
            for x in self._get_content_plugs()
            if x.default_provider is not None
        ]

    def get_extra_build_snaps(self) -> List[str]:
        """Get list of extra snaps required to build."""
        # Build snaps defined by the user with channel stripped
        build_snaps: List[str] = []
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
        """Get the first build_on architecture from the project."""
        if isinstance(self.architectures[0], Architecture) and isinstance(
            self.architectures[0].build_on, List
        ):
            return self.architectures[0].build_on[0]

        # will not happen after schema validation
        raise RuntimeError("cannot determine build-on architecture")

    def get_build_for(self) -> str:
        """Get the first build_for architecture from the project."""
        if isinstance(self.architectures[0], Architecture) and isinstance(
            self.architectures[0].build_for, List
        ):
            return self.architectures[0].build_for[0]

        # will not happen after schema validation
        raise RuntimeError("cannot determine build-for architecture")

    def get_build_for_arch_triplet(self) -> Optional[str]:
        """Get the architecture triplet for the first build-for architecture.

        :returns: The build-for arch triplet. If build-for is "all", then return None.
        """
        arch = self.get_build_for()

        if arch != "all":
            return get_arch_triplet(convert_architecture_deb_to_platform(arch))

        return None

    def get_build_plan(self) -> List[BuildInfo]:
        """Get the build plan for this project."""
        # TODO
        raise NotImplementedError("Not implemented yet!")


class _GrammarAwareModel(pydantic.BaseModel):
    class Config:
        """Default configuration for grammar-aware models."""

        validate_assignment = True
        extra = "allow"  # this is required to verify only grammar-aware parts
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731
        allow_population_by_field_name = True


class _GrammarAwarePart(_GrammarAwareModel):
    source: Optional[GrammarStr]
    build_environment: Optional[GrammarSingleEntryDictList]
    build_packages: Optional[GrammarStrList]
    stage_packages: Optional[GrammarStrList]
    build_snaps: Optional[GrammarStrList]
    stage_snaps: Optional[GrammarStrList]
    parse_info: Optional[List[str]]


class GrammarAwareProject(_GrammarAwareModel):
    """Project definition containing grammar-aware components."""

    parts: Dict[str, _GrammarAwarePart]

    @classmethod
    def validate_grammar(cls, data: Dict[str, Any]) -> None:
        """Ensure grammar-enabled entries are syntactically valid."""
        try:
            cls(**data)
        except pydantic.ValidationError as err:
            raise ProjectValidationError(_format_pydantic_errors(err.errors())) from err


class ArchitectureProject(models.CraftBaseModel, extra=pydantic.Extra.ignore):
    """Project definition containing only architecture data."""

    architectures: List[Union[str, Architecture]] = [get_host_architecture()]

    @pydantic.validator("architectures", always=True)
    @classmethod
    def _validate_architecture_data(cls, architectures):
        """Validate architecture data."""
        return _validate_architectures(architectures)

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "ArchitectureProject":
        """Create and populate a new ``Project`` object from dictionary data.

        The unmarshal method validates entries in the input dictionary, populating
        the corresponding fields in the data object.

        :param data: The dictionary data to unmarshal.

        :return: The newly created object.

        :raise TypeError: If data is not a dictionary.
        """
        if not isinstance(data, dict):
            raise TypeError("Project data is not a dictionary")

        try:
            architectures = ArchitectureProject(**data)
        except pydantic.ValidationError as err:
            raise ProjectValidationError(_format_pydantic_errors(err.errors())) from err

        return architectures


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


def _format_global_keyword_warning(keyword: str, empty_entries: List[str]) -> str:
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
        "\n(Reference: https://snapcraft.io/docs/snapcraft-interfaces)"
    )
