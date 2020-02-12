# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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
from typing import Any, Dict, List, Set, Sequence, Optional

from snapcraft import yaml_utils
from snapcraft.internal import common
from snapcraft.internal.meta import errors
from snapcraft.internal.meta.application import Application
from snapcraft.internal.meta.hooks import Hook
from snapcraft.internal.meta.plugs import ContentPlug, Plug
from snapcraft.internal.meta.slots import ContentSlot, Slot
from snapcraft.internal.meta.system_user import SystemUser

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
    "system-usernames",
    "title",
    "type",
]


class Snap:
    """Representation of snap meta object, writes snap.yaml."""

    def __init__(  # noqa: C901
        self,
        adopt_info: Optional[str] = None,
        apps: Optional[Dict[str, Application]] = None,
        architectures: Optional[Sequence[str]] = None,
        assumes: Optional[Set[str]] = None,
        base: Optional[str] = None,
        confinement: Optional[str] = None,
        description: Optional[str] = None,
        environment: Optional[Dict[str, Any]] = None,
        epoch: Any = None,
        grade: Optional[str] = None,
        hooks: Optional[Dict[str, Hook]] = None,
        layout: Optional[Dict[str, Any]] = None,
        license: Optional[str] = None,
        name: Optional[str] = None,
        passthrough: Optional[Dict[str, Any]] = None,
        plugs: Optional[Dict[str, Plug]] = None,
        slots: Optional[Dict[str, Slot]] = None,
        summary: Optional[str] = None,
        system_usernames: Optional[Dict[str, SystemUser]] = None,
        title: Optional[str] = None,
        type: Optional[str] = None,
        version: Optional[str] = None,
    ) -> None:
        self.adopt_info = adopt_info

        if apps is None:
            self.apps: Dict[str, Application] = dict()
        else:
            self.apps = apps

        if architectures is None:
            self.architectures: Sequence[str] = list()
        else:
            self.architectures = architectures

        if assumes is None:
            self.assumes: Set[str] = set()
        else:
            self.assumes = assumes

        self.base = base
        self.confinement = confinement
        self.description = description

        if environment is None:
            self.environment: Dict[str, Any] = dict()
        else:
            self.environment = environment

        self.epoch = epoch
        self.grade = grade

        if hooks is None:
            self.hooks: Dict[str, Hook] = dict()
        else:
            self.hooks = hooks

        if layout is None:
            self.layout: Dict[str, Any] = dict()
        else:
            self.layout = layout

        self.license = license
        self.name = name

        if passthrough is None:
            self.passthrough: Dict[str, Any] = dict()
        else:
            self.passthrough = passthrough

        if plugs is None:
            self.plugs: Dict[str, Plug] = dict()
        else:
            self.plugs = plugs

        if slots is None:
            self.slots: Dict[str, Slot] = dict()
        else:
            self.slots = slots

        self.summary = summary

        if system_usernames is None:
            self.system_usernames: Dict[str, SystemUser] = dict()
        else:
            self.system_usernames = system_usernames

        self.title = title
        self.type = type
        self.version = version

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
            if not provider:
                continue

            provider_path = common.get_installed_snap_path(provider)
            yaml_path = os.path.join(provider_path, "meta", "snap.yaml")

            if not os.path.exists(yaml_path):
                continue

            snap = Snap.from_file(yaml_path)
            for slot in snap.get_content_slots():
                slot_installed_path = common.get_installed_snap_path(provider)
                provider_dirs |= slot.get_content_dirs(
                    installed_path=slot_installed_path
                )

        return provider_dirs

    def _validate_required_keys(self) -> None:
        """Verify that all mandatory keys have been satisfied."""
        missing_keys: List[str] = []
        for key in _MANDATORY_PACKAGE_KEYS:
            if key == "version" and self.adopt_info:
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

        for user in self.system_usernames.values():
            user.validate()

        if self.is_passthrough_enabled:
            logger.warning(
                "The 'passthrough' property is being used to "
                "propagate experimental properties to snap.yaml "
                "that have not been validated."
            )

    def _ensure_command_chain_assumption(self) -> None:
        """Ensure command-chain is in assumes (if used)."""
        if "command-chain" in self.assumes:
            return

        for app in self.apps.values():
            if app.command_chain:
                self.assumes.add("command-chain")
                return
        for hook in self.hooks.values():
            if hook.command_chain:
                self.assumes.add("command-chain")
                return

    @classmethod  # noqa: C901
    def from_dict(cls, snap_dict: Dict[str, Any]) -> "Snap":
        snap_dict = deepcopy(snap_dict)

        # Using pop() so we can catch if we *miss* fields
        # with whatever remains in the dictionary.
        adopt_info = snap_dict.pop("adopt-info", None)
        architectures = snap_dict.pop("architectures", None)

        # Process apps into Applications.
        apps: Dict[str, Application] = dict()
        apps_dict = snap_dict.pop("apps", None)
        if apps_dict:
            for app_name, app_dict in apps_dict.items():
                app = Application.from_dict(app_dict=app_dict, app_name=app_name)
                apps[app_name] = app

        # Treat `assumes` as a set, not as a list.
        assumes = set(snap_dict.pop("assumes", set()))

        base = snap_dict.pop("base", None)
        confinement = snap_dict.pop("confinement", None)
        description = snap_dict.pop("description", None)
        environment = snap_dict.pop("environment", None)
        epoch = snap_dict.pop("epoch", None)
        grade = snap_dict.pop("grade", None)

        # Process hooks into Hooks.
        hooks: Dict[str, Hook] = dict()
        hooks_dict = snap_dict.pop("hooks", None)
        if hooks_dict:
            for hook_name, hook_dict in hooks_dict.items():
                # This can happen, but should be moved into Hook.from_object().
                if hook_dict is None:
                    continue

                hook = Hook.from_dict(hook_dict=hook_dict, hook_name=hook_name)
                hooks[hook_name] = hook

        layout = snap_dict.pop("layout", None)
        license = snap_dict.pop("license", None)
        name = snap_dict.pop("name", None)
        passthrough = snap_dict.pop("passthrough", None)

        # Process plugs into Plugs.
        plugs: Dict[str, Plug] = dict()
        plugs_dict = snap_dict.pop("plugs", None)
        if plugs_dict:
            for plug_name, plug_object in plugs_dict.items():
                plug = Plug.from_object(plug_object=plug_object, plug_name=plug_name)
                plugs[plug_name] = plug

        # Process slots into Slots.
        slots: Dict[str, Slot] = dict()
        slots_dict = snap_dict.pop("slots", None)
        if slots_dict:
            for slot_name, slot_object in slots_dict.items():
                slot = Slot.from_object(slot_object=slot_object, slot_name=slot_name)
                slots[slot_name] = slot

        summary = snap_dict.pop("summary", None)

        # Process sytemusers into SystemUsers.
        system_usernames: Dict[str, SystemUser] = dict()
        system_usernames_dict = snap_dict.pop("system-usernames", None)
        if system_usernames_dict:
            for user_name, user_object in system_usernames_dict.items():
                system_username = SystemUser.from_object(
                    user_object=user_object, user_name=user_name
                )
                system_usernames[user_name] = system_username

        title = snap_dict.pop("title", None)
        type = snap_dict.pop("type", None)
        version = snap_dict.pop("version", None)

        # Report unhandled keys.
        for key, value in snap_dict.items():
            logger.debug(f"ignoring or passing through unknown {key}={value}")

        return Snap(
            adopt_info=adopt_info,
            architectures=architectures,
            apps=apps,
            assumes=assumes,
            base=base,
            confinement=confinement,
            description=description,
            environment=environment,
            epoch=epoch,
            grade=grade,
            hooks=hooks,
            layout=layout,
            license=license,
            name=name,
            passthrough=passthrough,
            plugs=plugs,
            slots=slots,
            summary=summary,
            system_usernames=system_usernames,
            title=title,
            type=type,
            version=version,
        )

    def to_dict(self):  # noqa: C901
        snap_dict = OrderedDict()

        # Ensure command-chain is in assumes, if required.
        self._ensure_command_chain_assumption()

        for key in _MANDATORY_PACKAGE_KEYS + _OPTIONAL_PACKAGE_KEYS:
            if key != "system-usernames" and self.__dict__[key] is None:
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
                    snap_dict[key][name] = plug.to_yaml_object()
            elif key == "slots":
                snap_dict[key] = dict()
                for name, slot in sorted(self.slots.items()):
                    snap_dict[key][name] = slot.to_yaml_object()
            elif key == "system-usernames":
                if not self.system_usernames:
                    continue
                snap_dict["system-usernames"] = OrderedDict()
                for name in sorted(self.system_usernames.keys()):
                    user = self.system_usernames[name]
                    snap_dict["system-usernames"][name] = deepcopy(user.to_dict())
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
