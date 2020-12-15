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
import os
import re
from collections import OrderedDict
from copy import deepcopy
from typing import Any, Dict, List, Optional, Set, Tuple, Type

from snapcraft.internal.meta.errors import SlotValidationError

logger = logging.getLogger(__name__)


class Slot:
    """Representation of a generic snap slot."""

    def __init__(
        self,
        *,
        slot_name: str,
        slot_dict: Optional[Dict[str, Any]] = None,
        use_string_representation: bool = False,
    ) -> None:
        self._slot_name = slot_name

        if slot_dict is None:
            self._slot_dict: Dict[str, Any] = dict()
        else:
            self._slot_dict = slot_dict

        self.use_string_representation = use_string_representation

    @property
    def slot_name(self) -> str:
        """Read-only to ensure consistency with Snap dictionary mappings."""
        return self._slot_name

    def validate(self) -> None:
        """Validate slot, raising exception on error."""

        # Nothing to validate (yet).
        return

    @classmethod
    def from_dict(cls, *, slot_dict: Dict[str, Any], slot_name: str) -> "Slot":
        """Return applicable Slot instance from dictionary properties."""
        interface = slot_dict.get("interface", slot_name)

        # If we explicitly support the type, use it instead.
        slot_class = SLOT_MAPPINGS.get(interface, None)
        if slot_class is not None:
            return slot_class.from_dict(slot_dict=slot_dict, slot_name=slot_name)

        # Handle the general case.
        return Slot(slot_name=slot_name, slot_dict=slot_dict)

    @classmethod
    def from_object(cls, *, slot_object: Any, slot_name: str) -> "Slot":
        if slot_object is None:
            return Slot(slot_name=slot_name)
        elif isinstance(slot_object, str):
            slot = Slot(slot_name=slot_name, use_string_representation=True)
            slot._slot_dict["interface"] = slot_object
            return slot
        elif isinstance(slot_object, dict):
            return Slot.from_dict(slot_dict=slot_object, slot_name=slot_name)

        raise RuntimeError(f"unknown syntax for slot {slot_name!r}: {slot_object!r}")

    def to_yaml_object(self) -> Optional[Dict[str, Any]]:
        # To output string short-form: "slot-name: interface-type"
        if self.use_string_representation:
            return self._slot_dict["interface"]

        # To output shortest-form: "slot-name-is-interface: <empty>"
        if not self._slot_dict:
            return None

        return OrderedDict(deepcopy(self._slot_dict))

    def __repr__(self) -> str:
        return repr(self.__dict__)

    def __str__(self) -> str:
        return str(self.__dict__)


class ContentSlot(Slot):
    """Representation of a snap content slot."""

    def __init__(
        self,
        *,
        slot_name: str,
        content: Optional[str] = None,
        read: List[str] = None,
        write: List[str] = None,
        use_source_key: bool = True,
    ) -> None:
        super().__init__(slot_name=slot_name)

        if read is None:
            self.read: List[str] = list()
        else:
            self.read = read

        if write is None:
            self.write: List[str] = list()
        else:
            self.write = write

        self.use_source_key = use_source_key
        self._content = content

    @property
    def content(self) -> str:
        if self._content:
            return self._content

        # Defaults to slot_name if unspecified.
        return self.slot_name

    @content.setter
    def content(self, content) -> None:
        self._content = content

    @property
    def interface(self) -> str:
        return "content"

    def validate(self) -> None:
        """Validate content slot, raising exception on error."""

        if not self.read and not self.write:
            raise SlotValidationError(
                slot_name=self.slot_name,
                message="`read` or `write` is required for slot",
            )

    def get_content_dirs(self, installed_path: str) -> Set[str]:
        content_dirs: Set[str] = set()

        for path in self.read + self.write:
            # Strip leading "$SNAP" and "/".
            path = re.sub(r"^\$SNAP", "", path)
            path = re.sub(r"^/", "", path)
            path = re.sub(r"^./", "", path)
            content_dirs.add(os.path.join(installed_path, path))

        return content_dirs

    @classmethod
    def from_dict(cls, *, slot_dict: Dict[str, Any], slot_name: str) -> "ContentSlot":
        slot = ContentSlot(slot_name=slot_name, content=slot_dict.get("content"))

        # Content directories may be nested under "source",
        # but they cannot be in both places according to snapd:
        # https://github.com/snapcore/snapd/blob/master/interfaces/builtin/content.go#L81
        if "source" in slot_dict:
            source_data = slot_dict["source"]
            slot.use_source_key = True
        else:
            source_data = slot_dict
            slot.use_source_key = False

        if "read" in source_data:
            slot.read = source_data["read"]

        if "write" in source_data:
            slot.write = source_data["write"]

        return slot

    def to_yaml_object(self) -> Dict[str, Any]:
        props: List[Tuple[str, Any]] = [("interface", self.interface)]

        # Only include content if set explicitly.
        if self._content:
            props.append(("content", self.content))

        if self.use_source_key:
            source = dict()
            if self.read:
                source["read"] = self.read
            if self.write:
                source["write"] = self.write
            props.append(("source", source))
        else:
            if self.read:
                props.append(("read", self.read))
            if self.write:
                props.append(("write", self.write))

        return OrderedDict(props)


class DbusSlot(Slot):
    """Representation of a snap dbus slot."""

    def __init__(self, *, slot_name: str, bus: str, name: str) -> None:
        super().__init__(slot_name=slot_name)

        self.bus = bus
        self.name = name

    @property
    def interface(self) -> str:
        return "dbus"

    @classmethod
    def from_dict(cls, *, slot_dict: Dict[str, Any], slot_name: str) -> "DbusSlot":
        """Instantiate DbusSlot from dict."""

        if slot_dict.get("interface") != "dbus":
            raise SlotValidationError(
                slot_name=slot_name, message="invalid interface for DbusSlot"
            )

        if "bus" not in slot_dict:
            raise SlotValidationError(
                slot_name=slot_name, message="bus required for DbusSlot"
            )

        if "name" not in slot_dict:
            raise SlotValidationError(
                slot_name=slot_name, message="name required for DbusSlot"
            )

        return DbusSlot(
            slot_name=slot_name, bus=slot_dict["bus"], name=slot_dict["name"]
        )

    def validate(self) -> None:
        """Validate dbus slot. Raise exception if invalid."""

        if not self.bus:
            raise SlotValidationError(
                slot_name=self.slot_name,
                message="valid `bus` is required for dbus slot",
            )

        if not self.name:
            raise SlotValidationError(
                slot_name=self.slot_name,
                message="valid `name` is required for dbus slot",
            )

    def to_yaml_object(self) -> Dict[str, Any]:
        """Return dict of dbus slot."""

        return OrderedDict(
            [("interface", self.interface), ("bus", self.bus), ("name", self.name)]
        )


SLOT_MAPPINGS: Dict[str, Type[Slot]] = dict(content=ContentSlot, dbus=DbusSlot)
