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

from typing import List

from snapcraft import formatting_utils
from snapcraft.internal import errors


class CommandError(errors.SnapcraftError):
    def __init__(self, message: str) -> None:
        self.fmt = message


class SnapMetaGenerationError(errors.SnapcraftError):
    pass


class MissingSnapcraftYamlKeysError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "Missing required key(s) in snapcraft.yaml: {keys}. "
        "Either specify the missing key(s), or use 'adopt-info' to get them "
        "from a part."
    )

    def __init__(self, keys: List[str]) -> None:
        super().__init__(keys=formatting_utils.humanize_list(keys, "and"))


class AdoptedPartMissingError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'adopt-info' refers to a part named {part!r}, but it is not defined "
        "in the 'snapcraft.yaml' file."
    )

    def __init__(self, part: str) -> None:
        super().__init__(part=part)


class AdoptedPartNotParsingInfo(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "'adopt-info' refers to part {part!r}, but that part is lacking the "
        "'parse-info' property."
    )

    def __init__(self, part: str) -> None:
        super().__init__(part=part)


class AmbiguousPassthroughKeyError(SnapMetaGenerationError):

    fmt = (
        "Failed to generate snap metadata: "
        "The following keys are specified in their regular location "
        "as well as in passthrough: {keys}. "
        "Remove duplicate keys."
    )

    def __init__(self, keys: List[str]) -> None:
        super().__init__(keys=formatting_utils.humanize_list(keys, "and"))


class InvalidAppCommandError(errors.SnapcraftError):

    fmt = (
        "Failed to generate snap metadata: "
        "The specified command {command!r} defined in the app {app_name!r} does "
        "not exist or is not executable.\n"
        "Ensure that {command!r} is installed with the correct path."
    )

    def __init__(self, command, app_name):
        super().__init__(command=command, app_name=app_name)


class InvalidAppCommandNotFound(errors.SnapcraftError):

    fmt = (
        "Failed to generate snap metadata: "
        "The specified command {command!r} defined in the app {app_name!r} does "
        "not exist.\n"
        "Ensure that {command!r} is installed with the correct path."
    )

    def __init__(self, command, app_name):
        super().__init__(command=command, app_name=app_name)


class InvalidAppCommandNotExecutable(errors.SnapcraftError):

    fmt = (
        "Failed to generate snap metadata: "
        "The specified command {command!r} defined in the app {app_name!r} "
        "is not executable."
    )

    def __init__(self, command: str, app_name: str) -> None:
        super().__init__(command=command, app_name=app_name)


class InvalidAppCommandFormatError(errors.SnapcraftError):

    fmt = (
        "Failed to generate snap metadata: "
        "The specified command {command!r} defined in the app {app_name!r} does "
        "not match the pattern expected by snapd.\n"
        "The command must consist only of alphanumeric characters, spaces, and the "
        "following special characters: / . _ # : $ -"
    )

    def __init__(self, command: str, app_name: str) -> None:
        super().__init__(command=command, app_name=app_name)


class InvalidCommandChainError(errors.SnapcraftError):

    fmt = (
        "Failed to generate snap metadata: "
        "The command-chain item {item!r} defined in the app {app_name!r} does "
        "not exist or is not executable.\n"
        "Ensure that {item!r} is relative to the prime directory."
    )

    def __init__(self, item: str, app_name: str) -> None:
        super().__init__(item=item, app_name=app_name)


class InvalidDesktopFileError(errors.SnapcraftError):

    fmt = (
        "Failed to generate desktop file: "
        "Invalid desktop file {filename!r}: {message}."
        # FIXME include how to fix each of the possible desktop file errors.
        # https://bugs.launchpad.net/snapcraft/+bug/1727435
        # --elopio - 2017-10-25
    )

    def __init__(self, filename: str, message: str) -> None:
        super().__init__(filename=filename, message=message)


class PrimedCommandNotFoundError(errors.SnapcraftError):
    fmt = (
        "Failed to generate snap metadata: "
        "Specified command {command!r} was not found.\n"
        "Verify the command is correct and for a more "
        "deterministic outcome, specify the relative path "
        "to the command from the prime directory."
    )

    def __init__(self, command: str) -> None:
        super().__init__(command=command)


class SlotValidationError(errors.SnapcraftError):
    fmt = "failed to validate slot={slot_name}: {message}"

    def __init__(self, *, slot_name: str, message: str) -> None:
        super().__init__(slot_name=slot_name, message=message)


class PlugValidationError(errors.SnapcraftError):
    fmt = "failed to validate plug={plug_name}: {message}"

    def __init__(self, *, plug_name: str, message: str) -> None:
        super().__init__(plug_name=plug_name, message=message)


class HookValidationError(errors.SnapcraftError):
    fmt = "failed to validate hook={hook_name}: {message}"

    def __init__(self, *, hook_name: str, message: str) -> None:
        super().__init__(hook_name=hook_name, message=message)


class SystemUsernamesValidationError(errors.SnapcraftException):
    def __init__(self, *, name: str, message: str) -> None:
        self._name = name
        self._message = message

    def get_brief(self) -> str:
        return f"Invalid system-usernames for {self._name!r}: {self._message}"

    def get_resolution(self) -> str:
        return "Please configure system user according to documentation."

    def get_docs_url(self) -> str:
        return "https://forum.snapcraft.io/t/system-usernames/13386"


class GradeDevelRequiredError(errors.SnapcraftException):
    def __init__(self, *, set_grade: str) -> None:
        self.set_grade = set_grade

    def get_brief(self) -> str:
        return f"Snap 'grade' was set to {self.set_grade!r} but must be 'devel'."

    def get_resolution(self) -> str:
        return "Set 'grade' to 'devel' or use a stable base for this snap."
