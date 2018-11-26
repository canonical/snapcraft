# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import enum
from typing import cast, Dict, List, Union
from typing import TYPE_CHECKING

from ._env import (  # noqa: F401
    environment_to_replacements,
    snapcraft_global_environment,
    snapcraft_part_environment,
)
from ._parts_config import PartsConfig  # noqa: F401
from ._extensions import (  # noqa: F401
    apply_extensions,
    find_extension,
    supported_extension_names,
)

if TYPE_CHECKING:
    from snapcraft.project import Project  # noqa: F401


@enum.unique
class Adapter(enum.Enum):
    NONE = 1
    LEGACY = 2
    FULL = 3


def load_config(project: "Project"):
    from ._config import Config

    return Config(project)


def replace_attr(
    attr: Union[List[str], Dict[str, str], str], replacements: Dict[str, str]
) -> Union[List[str], Dict[str, str], str]:
    if isinstance(attr, str):
        for replacement, value in replacements.items():
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
