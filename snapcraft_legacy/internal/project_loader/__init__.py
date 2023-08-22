# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017, 2023 Canonical Ltd
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

from typing import TYPE_CHECKING, Dict, List, Optional, Union, cast

from ._env import environment_to_replacements, runtime_env  # noqa: F401
from ._extensions import (  # noqa: F401
    apply_extensions,
    find_extension,
    supported_extension_names,
)
from ._parts_config import PartsConfig  # noqa: F401
from .errors import VariableEvaluationError

if TYPE_CHECKING:
    from snapcraft_legacy.project import Project  # noqa: F401


def load_config(project: "Project"):
    from ._config import Config

    return Config(project)


def replace_attr(
    attr: Union[List[str], Dict[str, str], str], replacements: Dict[str, Optional[str]]
) -> Union[List[str], Dict[str, str], str]:
    """Recurse through a complex data structure and replace values.

    :param attr: The data to modify, which may contain nested lists, dicts, and strings.
    :param replacements: A mapping of replacements to make.

    :returns: The data structure with replaced values.
    """
    if isinstance(attr, str):
        for replacement, value in replacements.items():
            _validate_replacement(attr, replacement, value)
            attr = attr.replace(replacement, str(value))
        return attr
    elif isinstance(attr, list) or isinstance(attr, tuple):
        return [cast(str, replace_attr(i, replacements)) for i in attr]
    elif isinstance(attr, dict):
        result = dict()  # type: Dict[str, str]
        for key, value in attr.items():
            # Run replacements on both the key and value
            key = cast(str, replace_attr(key, replacements))
            value = cast(str, replace_attr(value, replacements))
            result[key] = value
        return result

    return attr


def _validate_replacement(attr: str, replacement: str, value: Optional[str]) -> None:
    """Validate if a replacement can occur for an attribute.

    Some replacement data cannot be used if it is not defined, such as the build-for
    arch and build-for arch triplet. If the value of these replacements is None, then
    a validation error is raised.

    :param attr: String that may contain data to replace.
    :param replacement: Project variable to replace.
    :param value: New value for replacement.

    :raises Exception: If a replacement cannot occur.
    """
    project_variables = [
        "SNAPCRAFT_ARCH_BUILD_FOR",
        "SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR",
    ]

    # expand to shell syntax for variables (`$item` and `${item}`)
    expanded_project_variables = (
        [f"${item}" for item in project_variables ] +
        [f"${{{item}}}" for item in project_variables]
    )

    if (
        replacement in attr
        and replacement in expanded_project_variables
        and value is None
    ):
        raise VariableEvaluationError(
            variable=replacement,
            reason="the build-for architecture could not be determined"
        )
