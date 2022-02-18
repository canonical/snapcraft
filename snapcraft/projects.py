# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

# XXX: mypy doesn't like Literal
from typing import Literal  # type: ignore
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple, Union

import pydantic
from pydantic import conlist, constr

from snapcraft.errors import ProjectValidationError
from snapcraft.parts import validation as parts_validation


class ProjectModel(pydantic.BaseModel):
    """Base model for snapcraft project classes."""

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        validate_assignment = True
        # extra = "forbid"
        allow_mutation = False
        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


# A workaround for mypy false positives
# see https://github.com/samuelcolvin/pydantic/issues/975#issuecomment-551147305
# fmt: off
if TYPE_CHECKING:
    CommandChainStr = str
    KeyIdStr = str
    UniqueStrList = List[str]
    UniqueAliasList = List[str]
else:
    CommandChainStr = constr(regex=r"^[A-Za-z0-9/._#:$-]*$")
    KeyIdStr = constr(regex=r"^[A-Z0-9]{40}$")
    UniqueStrList = conlist(str, unique_items=True)
    UniqueAliasList = conlist(constr(regex=r"^[a-zA-Z0-9][-_.a-zA-Z0-9]*$"), unique_items=True)
# fmt: on


class App(ProjectModel):
    """Snapcraft project app definition."""

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
    daemon: Optional[Literal["simple", "forking", "oneshot", "notify", "dbus"]]
    after: UniqueStrList = []
    before: UniqueStrList = []
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
    aliases: Optional[UniqueAliasList]
    environment: Optional[Dict[str, str]]
    command_chain: List[CommandChainStr] = []
    # TODO: sockets

    @pydantic.validator("autostart")
    @classmethod
    def _validate_desktop_name(cls, name):
        if not re.match(r"^[A-Za-z0-9. _#:$-]+\\.desktop$", name):
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


class Hook(ProjectModel):
    """Snapcraft project hook definition."""

    command_chain: List[CommandChainStr] = []
    environment: List[Dict[str, str]] = []
    plugs: UniqueStrList = []
    passthrough: Optional[Dict[str, Any]]


class Architecture(ProjectModel):
    """Snapcraft project architecture definition."""

    build_on: Union[str, UniqueStrList]
    build_to: Optional[Union[str, UniqueStrList]]


class AptDeb(ProjectModel):
    """Apt package repository definition."""

    type: Literal["apt"]
    url: str
    key_id: KeyIdStr
    architectures: Optional[List[str]]
    formats: Optional[List[Literal["deb", "deb-src"]]]
    components: Optional[List[str]]
    key_server: Optional[str]
    path: Optional[str]
    suites: Optional[List[str]]


class AptPPA(ProjectModel):
    """PPA package repository definition."""

    type: Literal["apt"]
    ppa: str


class Project(ProjectModel):
    """Snapcraft project definition.

    See https://snapcraft.io/docs/snapcraft-yaml-reference

    XXX: Not implemented in this version
    - environment (top-level)
    - system-usernames
    - adopt-info (after adding craftctl support to craft-parts)

    FIXME: package-repositories needs better validation and less
           confusing error messages
    """

    name: constr(max_length=40)  # type: ignore
    title: Optional[constr(max_length=40)]  # type: ignore
    base: Optional[str]
    build_base: Optional[str]
    compression: Literal["lzo", "xz"] = "xz"
    version: Optional[constr(max_length=32, strict=True)]  # type: ignore
    contact: Optional[Union[str, UniqueStrList]]
    donation: Optional[Union[str, UniqueStrList]]
    issues: Optional[Union[str, UniqueStrList]]
    source_code: Optional[str]
    website: Optional[str]
    summary: constr(max_length=78)  # type: ignore
    description: str
    type: Literal["app", "base", "gadget", "kernel", "snapd"] = "app"
    icon: Optional[str]
    confinement: Literal["classic", "devmode", "strict"]
    layout: Optional[Dict[str, Dict[str, Any]]]
    license: Optional[str]
    grade: Literal["stable", "devel"]
    architectures: List[Architecture] = []
    assumes: UniqueStrList = []
    package_repositories: List[Union[AptDeb, AptPPA]] = []
    hooks: Optional[Dict[str, Hook]]
    passthrough: Optional[Dict[str, Any]]
    apps: Optional[Dict[str, App]]
    plugs: Optional[Dict[str, Dict[str, str]]]  # TODO: add plug name validation
    slots: Optional[Dict[str, Dict[str, str]]]  # TODO: add slot name validation
    parts: Dict[str, Any]  # parts are handled by craft-parts
    epoch: Optional[str]

    @pydantic.root_validator(pre=True)
    @classmethod
    def _validate_mandatory_version(cls, values):
        if "version" not in values and "adopt-info" not in values:
            raise ValueError("Snap version is required if not using adopt-info")
        return values

    @pydantic.root_validator(pre=True)
    @classmethod
    def _validate_mandatory_base(cls, values):
        snap_type = values.get("type")
        if ("base" in values) ^ (snap_type not in ["base", "kernel", "snapd"]):
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
    def _validate_version(cls, version):
        if not re.match(r"^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$", version):
            raise ValueError(
                "Snap versions consist of upper- and lower-case alphanumeric characters, "
                "as well as periods, colons, plus signs, tildes, and hyphens. They cannot "
                "begin with a period, colon, plus sign, tilde, or hyphen. They cannot end "
                "with a period, colon, or hyphen"
            )

        return version

    @pydantic.validator("build_base", always=True)
    @classmethod
    def _validate_build_base(cls, build_base, values):
        """Build-base defaults to the base value if not specified."""
        if not build_base:
            build_base = values.get("base")
        return build_base

    @pydantic.validator("parts", each_item=True)
    @classmethod
    def _validate_parts(cls, item):
        """Verify each part (craft-parts will re-validate this)."""
        parts_validation.validate_part(item)
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
            raise TypeError("part data is not a dictionary")

        try:
            project = Project(**data)
        except pydantic.ValidationError as err:
            raise ProjectValidationError(_format_pydantic_errors(err.errors())) from err

        return project


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
