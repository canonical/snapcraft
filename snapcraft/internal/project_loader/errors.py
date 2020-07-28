# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

from typing import Set

from snapcraft.internal.errors import SnapcraftException


class SnapcraftAfterPartMissingError(SnapcraftException):
    def __init__(self, *, part_name: str, after_part_name: str) -> None:
        self.part_name = part_name
        self.after_part_name = after_part_name

    def get_brief(self) -> str:
        return f"Part {self.part_name!r} after configuration refers to unknown part {self.after_part_name!r}."

    def get_resolution(self) -> str:
        return "Ensure 'after' configuration and referenced part name is correct."


class SnapcraftDuplicateAliasError(SnapcraftException):
    def __init__(self, *, aliases: Set[str]) -> None:
        self.aliases = ", ".join(sorted(aliases))

    def get_brief(self) -> str:
        return f"Multiple parts have the same alias defined: {self.aliases}"

    def get_resolution(self) -> str:
        return "Use each alias only once."


class SnapcraftExtensionBaseRequiredError(SnapcraftException):
    def get_brief(self) -> str:
        return "Extensions can only be used if the snapcraft.yaml specifies a 'base'."

    def get_resolution(self) -> str:
        return "Ensure your base configuration is correct."


class SnapcraftExtensionNotFoundError(SnapcraftException):
    def __init__(self, *, extension_name: str) -> None:
        self.extension_name = extension_name

    def get_brief(self) -> str:
        return f"Failed to find extension {self.extension_name!r}."

    def get_resolution(self) -> str:
        return "Ensure the extension name is correct."


class SnapcraftExtensionPartConflictError(SnapcraftException):
    def __init__(self, *, extension_name: str, part_name: str) -> None:
        self.extension_name = extension_name
        self.part_name = part_name

    def get_brief(self) -> str:
        return f"Failed to apply extension {self.extension_name!r}."

    def get_details(self) -> str:
        return f"This extension adds a part named {self.part_name!r}, but a part by that name already exists."

    def get_resolution(self) -> str:
        return f"Rename the {self.part_name!r} part."


class SnapcraftExtensionUnsupportedBaseError(SnapcraftException):
    def __init__(self, *, extension_name: str, base: str) -> None:
        self.extension_name = extension_name
        self.base = base

    def get_brief(self) -> str:
        return f"Failed to load extension {self.extension_name!r}."

    def get_details(self) -> str:
        return f"This extension does not support the {self.base!r} base."

    def get_resolution(self) -> str:
        return "Either use a different extension, or use a base supported by this extension."


class SnapcraftExtensionUnsupportedConfinementError(SnapcraftException):
    def __init__(self, *, extension_name: str, confinement: str) -> None:
        self.extension_name = extension_name
        self.confinement = confinement

    def get_brief(self) -> str:
        return f"Failed to load extension {self.extension_name!r}."

    def get_details(self) -> str:
        return f"This extension does not support {self.confinement!r} confinement."

    def get_resolution(self) -> str:
        return "Either use a different extension, or use a confinement supported by this extension."


class SnapcraftFilesetReferenceError(SnapcraftException):
    def __init__(self, *, item: str, step: str) -> None:
        self.item = item
        self.step = step

    def get_brief(self) -> str:
        return f"Fileset {self.item!r} referred to in the {self.step!r} step was not found."

    def get_resolution(self) -> str:
        return "Ensure the fileset configuration is correct."


class SnapcraftInvalidEpochError(SnapcraftException):
    def __init__(self, *, epoch: str) -> None:
        self.epoch = epoch

    def get_brief(self) -> str:
        return f"Invalid epoch format for {self.epoch!r}."

    def get_resolution(self) -> str:
        return "Valid epochs are positive integers followed by an optional asterisk."
