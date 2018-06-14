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

from typing import cast, Dict, List, Union
from typing import TYPE_CHECKING

from ._env import (                        # noqa: F401
    environment_to_replacements,
    snapcraft_global_environment,
    snapcraft_part_environment,
)
from ._schema import Validator             # noqa: F401
from ._parts_config import PartsConfig     # noqa: F401

if TYPE_CHECKING:
    from snapcraft.project import Project  # noqa: F401


def load_config(project: 'Project'):
    from ._config import Config
    return Config(project)


def replace_attr(
        attr: Union[List[str], Dict[str, str], str],
        replacements: Dict[str, str]) -> Union[List[str], Dict[str, str], str]:
    if isinstance(attr, str):
        for replacement, value in replacements.items():
            attr = attr.replace(replacement, str(value))
        return attr
    elif isinstance(attr, list) or isinstance(attr, tuple):
        return [cast(str, replace_attr(i, replacements))
                for i in attr]
    elif isinstance(attr, dict):
        return {k: cast(str, replace_attr(attr[k], replacements))
                for k in attr}

    return attr
