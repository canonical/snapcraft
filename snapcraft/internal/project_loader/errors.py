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

import pathlib

import snapcraft.internal.errors


class ProjectLoaderError(snapcraft.internal.errors.SnapcraftError):

    fmt = ""


class InvalidEpochError(ProjectLoaderError):

    fmt = "epochs are positive integers followed by an optional asterisk"


class DuplicateAliasError(ProjectLoaderError):

    fmt = "Multiple parts have the same alias defined: {aliases!r}"

    def __str__(self):
        if isinstance(self.aliases, (list, set)):
            self.aliases = ",".join(self.aliases)

        return super().__str__()


class SnapcraftLogicError(ProjectLoaderError):

    fmt = "Issue detected while analyzing snapcraft.yaml: {message}"

    def __init__(self, message):
        super().__init__(message=message)


class ExtensionBaseRequiredError(ProjectLoaderError):
    fmt = "Extensions can only be used if the snapcraft.yaml specifies a 'base'"


class ExtensionNotFoundError(ProjectLoaderError):
    fmt = (
        "Failed to find extension {extension_name!r}: "
        "a extension by that name does not exist.\n"
        "Check the extension name and try again."
    )

    def __init__(self, extension_name: str) -> None:
        super().__init__(extension_name=extension_name)


class ExtensionPartConflictError(ProjectLoaderError):
    fmt = (
        "Failed to apply extension {extension_name!r}: "
        "this extension adds a part named {part_name!r}, but a part by that name "
        "already exists.\n"
        "Rename the {part_name!r} part to something else and try again."
    )

    def __init__(self, extension_name: str, part_name: str) -> None:
        super().__init__(extension_name=extension_name, part_name=part_name)


class ExtensionUnsupportedBaseError(ProjectLoaderError):
    fmt = (
        "Failed to load extension {extension_name!r}: "
        "this extension does not support the {base!r} base.\n"
        "Either use a different extension, or use a base supported by this extension."
    )

    def __init__(self, extension_name: str, base: str) -> None:
        super().__init__(extension_name=extension_name, base=base)


class ExtensionUnsupportedConfinementError(ProjectLoaderError):
    fmt = (
        "Failed to load extension {extension_name!r}: "
        "this extension does not support {confinement!r} confinement.\n"
        "Either use a different extension, or use a confinement setting "
        "supported by this extension."
    )

    def __init__(self, extension_name: str, confinement: str) -> None:
        super().__init__(extension_name=extension_name, confinement=confinement)


class ExtensionMissingDocumentationError(ProjectLoaderError):
    fmt = (
        "The {extension_name!r} extension appears to be missing documentation.\n"
        "We would appreciate it if you created a bug report about this at "
        "https://launchpad.net/snapcraft/+filebug"
    )

    def __init__(self, extension_name: str) -> None:
        super().__init__(extension_name=extension_name)


class SnapcraftAfterPartMissingError(ProjectLoaderError):

    fmt = (
        "Failed to get part information: "
        "Cannot find the definition for part {after_part_name!r}, required by part "
        "{part_name!r}.\n"
        "Remote parts are not supported with bases, so make sure that this part is "
        "defined in the `snapcraft.yaml`."
    )

    def __init__(self, part_name, after_part_name):
        super().__init__(part_name=part_name, after_part_name=after_part_name)


class SnapcraftProjectUnusedKeyAssetError(snapcraft.internal.errors.SnapcraftException):
    def __init__(self, key_path: pathlib.Path):
        self.key_path = key_path

    def get_brief(self) -> str:
        return f"Found unused key asset {str(self.key_path)!r}."

    def get_details(self) -> str:
        return "All configured key assets must be utilized."

    def get_resolution(self) -> str:
        return "Verify key usage and remove all unused keys."
