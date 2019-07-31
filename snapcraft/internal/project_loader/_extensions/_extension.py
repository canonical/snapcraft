# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

import abc
from typing import Any, Dict
from typing import Tuple


class Extension(metaclass=abc.ABCMeta):
    """Extension is the class from which all extensions inherit.

    Extensions have the ability to add snippets to apps, parts, and indeed add new parts
    to a given snapcraft.yaml. All they need to do is define the proper variables.

    :ivar root_snippet: Instance variable, dict of properties to apply to root of the
                        snapcraft.yaml.
    :ivar app_snippet: Instance variable, dict of properties to apply to apps using this
                       extension.
    :ivar part_snippet: Instance variable, dict of properties to apply to parts using
                        this extension.
    :ivar parts: Instance variable, dict of part definitions required by this extension.
    """

    @staticmethod
    @abc.abstractmethod
    def get_supported_bases() -> Tuple[str, ...]:
        """Return a tuple of supported bases."""

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new Extension.

        :param dict yaml_data: Loaded snapcraft.yaml data.
        """
        self.root_snippet = dict()  # type: Dict[str, Any]
        self.app_snippet = dict()  # type: Dict[str, Any]
        self.part_snippet = dict()  # type: Dict[str, Any]
        self.parts = dict()  # type: Dict[str, Any]
