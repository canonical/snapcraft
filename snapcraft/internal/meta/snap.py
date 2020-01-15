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
from collections import OrderedDict
from copy import deepcopy
from typing import Dict, Set, Sequence, List, Any

from snapcraft import yaml_utils
from snapcraft.internal import common
from snapcraft.internal.meta import errors
from snapcraft.internal.meta.application import Application
from snapcraft.internal.meta.hooks import Hook
from snapcraft.internal.meta.plugs import ContentPlug, Plug
from snapcraft.internal.meta.slots import ContentSlot, Slot

logger = logging.getLogger(__name__)


_MANDATORY_PACKAGE_KEYS = ["name", "version", "summary", "description"]
_OPTIONAL_PACKAGE_KEYS = [
    "apps",
    "architectures",
    "assumes",
    "base",
    "confinement",
    "environment",
    "epoch",
    "grade",
    "hooks",
    "layout",
    "license",
    "plugs",
    "slots",
    "title",
    "type",
]


class Snap:
    """Representation of snap meta object, writes snap.yaml."""

    def __init__(self) -> None:
        self.adopt_info: str = None
        self.base: str = None
        self.name: str = None
        self.version: str = None
        self.summary: str = None
        self.description: str = None
        self.architectures: Sequence[str] = list()
        self.assumes: Set[str] = set()
        self.confinement: str = None
        self.environment: Dict[str, Any] = dict()
        self.epoch: Any = None
        self.grade: str = None
        self.license: str = None
        self.title: str = None
        self.type: str = None
        self.plugs: Dict[str, Plug] = dict()
        self.slots: Dict[str, Slot] = dict()
        self.apps: Dict[str, Application] = dict()
        self.hooks: Dict[str, Hook] = dict()
        self.passthrough: Dict[str, Any] = dict()
        self.layout: Dict[str, Any] = dict()

    @classmethod
    def from_file(cls, snap_yaml_path: str) -> "Snap":
        with open(snap_yaml_path, "r") as f:
            snap_dict = yaml_utils.load(f)
            return cls.from_dict(snap_dict=snap_dict)

    @property
    def is_passthrough_enabled(self) -> bool:
        if self.passthrough:
            return True

        for app in self.apps.values():
            if app.passthrough:
                return True

        for hook in self.hooks.values():
            if hook.passthrough:
                return True

        return False

    def get_content_plugs(self) -> List[ContentPlug]:
        """Get list of content plugs."""
        return [plug for plug in self.plugs.values() if isinstance(plug, ContentPlug)]

    def get_content_slots(self) -> List[ContentSlot]:
        """Get list of content slots."""
        return [slot for slot in self.slots.values() if isinstance(slot, ContentSlot)]

    def get_provider_content_directories(self) -> Set[str]:
        """Get provider content directories from installed snaps."""
        provider_dirs: Set[str] = set()

        for plug in self.get_content_plugs():
            # Get matching slot provider for plug.
            provider = plug.provider
            provider_path = common.get_installed_snap_path(provider)
            yaml_path = os.path.join(provider_path, "meta", "snap.yaml")

            snap = Snap.from_file(yaml_path)
            for slot in snap.get_content_slots():
                slot_installed_path = common.get_installed_snap_path(plug.provider)
                provider_dirs |= slot.get_content_dirs(
                    installed_path=slot_installed_path
                )

        return provider_dirs

    def _validate_required_keys(self) -> None:
        """Verify that all mandatory keys have been satisfied."""
        missing_keys: List[str] = []
        for key in _MANDATORY_PACKAGE_KEYS:
            if key is "version" and self.adopt_info:
                continue

            if not self.__dict__[key]:
                missing_keys.append(key)

        if missing_keys:
            raise errors.MissingSnapcraftYamlKeysError(keys=missing_keys)

    def validate(self) -> None:
        """Validate snap, raising exception on error."""
        self._validate_required_keys()

        for app in self.apps.values():
            app.validate()

        for hook in self.hooks.values():
            hook.validate()

        for plug in self.plugs.values():
            plug.validate()

        for slot in self.slots.values():
            slot.validate()

        if self.is_passthrough_enabled:
            logger.warn(
                "The 'passthrough' property is being used to "
                "propagate experimental properties to snap.yaml "
                "that have not been validated."
            )

    def _ensure_command_chain_assumption(self) -> None:
        """Ensure command-chain is in assumes (if used)."""
        if "command-chain" in self.assumes:
            return

        for app in self.apps.values():
            if app.command_chain or app.prepend_command_chain:
                self.assumes.add("command-chain")
                return

    @classmethod  # noqa: C901
    def from_dict(cls, snap_dict: Dict[str, Any]) -> "Snap":
        snap_dict = deepcopy(snap_dict)

        snap = Snap()

        if "passthrough" in snap_dict:
            snap.passthrough = snap_dict.pop("passthrough")

        for key in snap_dict:
            if key == "plugs":
                for plug_name, plug_object in snap_dict[key].items():
                    plug = Plug.from_object(
                        plug_object=plug_object, plug_name=plug_name
                    )
                    snap.plugs[plug_name] = plug
            elif key == "slots":
                for slot_name, slot_object in snap_dict[key].items():
                    slot = Slot.from_object(
                        slot_object=slot_object, slot_name=slot_name
                    )
                    snap.slots[slot_name] = slot
            elif key == "apps":
                for app_name, app_dict in snap_dict[key].items():
                    app = Application.from_dict(app_dict=app_dict, app_name=app_name)
                    snap.apps[app_name] = app
            elif key == "hooks":
                for hook_name, hook_dict in snap_dict[key].items():
                    hook = Hook.from_dict(hook_dict=hook_dict, hook_name=hook_name)
                    snap.hooks[hook_name] = hook
            elif key in _MANDATORY_PACKAGE_KEYS:
                snap.__dict__[key] = snap_dict[key]
            elif key in _OPTIONAL_PACKAGE_KEYS:
                snap.__dict__[key] = snap_dict[key]
            else:
                logger.debug("ignoring or passing through unknown key: {}".format(key))
                continue

        if "adopt-info" in snap_dict:
            snap.adopt_info = snap_dict["adopt-info"]

        return snap

    def to_dict(self):  # noqa: C901
        snap_dict = OrderedDict()

        # Ensure command-chain is in assumes, if required.
        self._ensure_command_chain_assumption()

        for key in _MANDATORY_PACKAGE_KEYS + _OPTIONAL_PACKAGE_KEYS:
            if self.__dict__[key] is None:
                continue

            # Skip fields that are empty lists/dicts.
            if (
                key
                in [
                    "apps",
                    "architectures",
                    "assumes",
                    "environment",
                    "hooks",
                    "layout",
                    "plugs",
                    "slots",
                ]
                and not self.__dict__[key]
            ):
                continue

            # Sort where possible for consistency.
            if key == "apps":
                snap_dict[key] = dict()
                for name, app in sorted(self.apps.items()):
                    snap_dict[key][name] = app.to_dict()
            elif key == "assumes":
                snap_dict[key] = sorted(set(self.assumes))
            elif key == "hooks":
                snap_dict[key] = dict()
                for name, hook in sorted(self.hooks.items()):
                    snap_dict[key][name] = hook.to_dict()
            elif key == "plugs":
                snap_dict[key] = dict()
                for name, plug in sorted(self.plugs.items()):
                    snap_dict[key][name] = plug.to_dict()
            elif key == "slots":
                snap_dict[key] = dict()
                for name, slot in sorted(self.slots.items()):
                    snap_dict[key][name] = slot.to_dict()
            else:
                snap_dict[key] = deepcopy(self.__dict__[key])

        # Apply passthrough keys.
        snap_dict.update(deepcopy(self.passthrough))
        return snap_dict

    def write_snap_yaml(self, path: str) -> None:
        """Write snap.yaml contents to specified path."""
        snap_dict = self.to_dict()

        # If the base is core in snapcraft.yaml we do not set it in
        # snap.yaml LP: #1819290
        if self.base == "core":
            snap_dict.pop("base")

        with open(path, "w") as f:
            yaml_utils.dump(snap_dict, stream=f)

    def __repr__(self) -> str:
        return repr(self.__dict__)

    def __str__(self) -> str:
        return str(self.__dict__)
