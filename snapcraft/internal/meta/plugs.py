# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import logging
from collections import OrderedDict
from copy import deepcopy
from typing import Any, Dict, Optional, Type

from snapcraft.internal.meta.errors import PlugValidationError

logger = logging.getLogger(__name__)


class Plug:
    """Generic plug."""

    def __init__(
        self,
        *,
        plug_name: str,
        plug_dict: Optional[Dict[str, Any]] = None,
        use_string_representation: bool = False,
    ) -> None:
        self._plug_name = plug_name
        if plug_dict is None:
            self._plug_dict: Dict[str, Any] = dict()
        else:
            self._plug_dict = plug_dict

        self.use_string_representation = use_string_representation

    @property
    def plug_name(self) -> str:
        """Read-only to ensure consistency with Snap dictionary mappings."""

        return self._plug_name

    def validate(self) -> None:
        """Validate plug, raising an exception on failure."""

        # Nothing to validate (yet).
        return

    @classmethod
    def from_dict(cls, *, plug_dict: Dict[str, Any], plug_name: str) -> "Plug":
        """Create plug from dictionary."""
        interface = plug_dict.get("interface", plug_name)

        plug_class = PLUG_MAPPINGS.get(interface, None)
        if plug_class is not None:
            return plug_class.from_dict(plug_dict=plug_dict, plug_name=plug_name)

        # Handle the general case.
        plug = Plug(plug_name=plug_name)
        plug._plug_dict.update(plug_dict)
        return plug

    @classmethod
    def from_object(cls, *, plug_object: Any, plug_name: str) -> "Plug":
        if plug_object is None:
            return Plug(plug_name=plug_name)
        elif isinstance(plug_object, str):
            plug = Plug(plug_name=plug_name, use_string_representation=True)
            plug._plug_dict["interface"] = plug_object
            return plug
        elif isinstance(plug_object, dict):
            return Plug.from_dict(plug_dict=plug_object, plug_name=plug_name)

        raise RuntimeError(f"unknown syntax for plug {plug_name!r}: {plug_object!r}")

    def to_yaml_object(self) -> Any:
        # To output string short-form: "slot-name: interface-type"
        if self.use_string_representation:
            return self._plug_dict["interface"]

        # To output shortest-form: "slot-name-is-interface: <empty>"
        if not self._plug_dict:
            return None

        return OrderedDict(deepcopy(self._plug_dict))

    def __repr__(self) -> str:
        return repr(self.__dict__)

    def __str__(self) -> str:
        return str(self.__dict__)


class ContentPlug(Plug):
    """Representation of a snap content plug."""

    def __init__(
        self,
        *,
        plug_name: str,
        content: Optional[str] = None,
        default_provider: Optional[str] = None,
        target: str,
    ) -> None:
        super().__init__(plug_name=plug_name)

        self._content = content
        self._default_provider = default_provider
        self.target = target

    @property
    def interface(self) -> str:
        return "content"

    @property
    def content(self) -> str:
        if self._content:
            return self._content

        # Defaults to plug_name if unspecified.
        return self.plug_name

    @content.setter
    def content(self, content) -> None:
        self._content = content

    @property
    def provider(self) -> Optional[str]:
        if self._default_provider is None:
            return None

        if ":" in self._default_provider:
            return self._default_provider.split(":")[0]

        return self._default_provider

    def validate(self) -> None:
        if not self.target:
            raise PlugValidationError(
                plug_name=self.plug_name,
                message="`target` is required for content slot",
            )

    @classmethod
    def from_dict(cls, *, plug_dict: Dict[str, str], plug_name: str) -> "ContentPlug":
        interface = plug_dict.get("interface")
        if interface != "content":
            raise PlugValidationError(
                plug_name=plug_name,
                message="`interface={}` is invalid for content slot".format(interface),
            )

        target = plug_dict.get("target", None)
        if target is None:
            raise PlugValidationError(
                plug_name=plug_name, message="`target` is required for content slot"
            )

        return ContentPlug(
            plug_name=plug_name,
            content=plug_dict.get("content", None),
            target=target,
            default_provider=plug_dict.get("default-provider", None),
        )

    def to_yaml_object(self) -> Dict[str, str]:
        props = [("interface", self.interface)]

        # Only include content if set explicitly.
        if self._content:
            props.append(("content", self.content))

        props.append(("target", self.target))

        if self.provider:
            props.append(("default-provider", self.provider))

        return OrderedDict(props)


PLUG_MAPPINGS: Dict[str, Type[Plug]] = dict(content=ContentPlug)
