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
from typing import Any, Dict, Optional, Tuple

from .. import errors


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

    @staticmethod
    @abc.abstractmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        """Return a tuple of supported confinement settings."""

    @staticmethod
    def is_experimental(base: Optional[str]) -> bool:
        """Return whether or not this extension is unstable for given base."""
        return False

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        """Create a new Extension.

        :param str extension_name: The name of the extension.
        :param dict yaml_data: Loaded snapcraft.yaml data.
        """
        self._sanity_check(extension_name=extension_name, yaml_data=yaml_data)

        self.root_snippet = dict()  # type: Dict[str, Any]
        self.app_snippet = dict()  # type: Dict[str, Any]
        self.part_snippet = dict()  # type: Dict[str, Any]
        self.parts = dict()  # type: Dict[str, Any]

    def _sanity_check(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        base = yaml_data.get("base")

        # A base is required in order to use extensions, so raise an error if not specified.
        if not base:
            raise errors.ExtensionBaseRequiredError()

        if base not in self.get_supported_bases():
            raise errors.ExtensionUnsupportedBaseError(extension_name, base)

        # Default to devmode if confinement is not set.
        confinement = yaml_data.get("confinement", "devmode")
        if confinement not in self.get_supported_confinement():
            raise errors.ExtensionUnsupportedConfinementError(
                extension_name, confinement
            )
