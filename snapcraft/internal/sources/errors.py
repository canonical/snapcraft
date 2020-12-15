# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017, 2019 Canonical Ltd
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

import shlex
from typing import List

from snapcraft import formatting_utils
from snapcraft.internal import errors


class SnapcraftSourceError(errors.SnapcraftError):
    pass


class VCSError(SnapcraftSourceError):
    fmt = "{message}"


class SnapcraftSourceNotFoundError(SnapcraftSourceError):

    fmt = (
        "Failed to pull source: {source!r}.\n"
        "Please ensure the source path is correct and that it is accessible.\n"
        "See `snapcraft help sources` for more information."
    )

    def __init__(self, source):
        super().__init__(source=source)


class SnapcraftSourceUnhandledError(SnapcraftSourceError):

    fmt = (
        "Failed to pull source: "
        "unable to determine source type of {source!r}.\n"
        "Check that the URL is correct or "
        "consider specifying `source-type` for this part. "
        "See `snapcraft help sources` for more information."
    )

    def __init__(self, source):
        super().__init__(source=source)


class SnapcraftSourceInvalidOptionError(SnapcraftSourceError):

    fmt = (
        "Failed to pull source: "
        "{option!r} cannot be used with a {source_type} source.\n"
        "See `snapcraft help sources` for more information."
    )

    def __init__(self, source_type: str, option: str) -> None:
        super().__init__(source_type=source_type, option=option)


class SnapcraftSourceIncompatibleOptionsError(SnapcraftSourceError):

    fmt = (
        "Failed to pull source: "
        "cannot specify both {humanized_options} for a {source_type} source.\n"
        "See `snapcraft help sources` for more information."
    )

    def __init__(self, source_type: str, options: List[str]) -> None:
        self.options = options
        super().__init__(
            source_type=source_type,
            humanized_options=formatting_utils.humanize_list(options, "and"),
        )


class DigestDoesNotMatchError(SnapcraftSourceError):

    fmt = "Expected the digest for source to be {expected}, but it was {calculated}"

    def __init__(self, expected, calculated):
        super().__init__(expected=expected, calculated=calculated)


class InvalidDebError(SnapcraftSourceError):

    fmt = (
        "The {deb_file} used does not contain valid data. "
        "Ensure a proper deb file is passed for .deb files "
        "as sources."
    )


class InvalidSnapError(SnapcraftSourceError):

    fmt = (
        "The snap file does not contain valid data. "
        "Ensure the source lists a proper snap file"
    )


class SourceUpdateUnsupportedError(SnapcraftSourceError):

    fmt = "Failed to update source: {source!s} sources don't support updating."

    def __init__(self, source):
        super().__init__(source=source)


class SnapcraftPullError(SnapcraftSourceError):

    fmt = "Failed to pull source, command {command!r} exited with code {exit_code}."

    def __init__(self, command, exit_code):
        if isinstance(command, list):
            string_command = " ".join(shlex.quote(i) for i in command)
        else:
            string_command = command
        super().__init__(command=string_command, exit_code=exit_code)


class SnapcraftRequestError(SnapcraftSourceError):

    fmt = "Network request error: {message}"


class GitCommandError(errors.SnapcraftException):
    def __init__(self, *, command: List[str], exit_code: int, output: str) -> None:
        self._command = " ".join(shlex.quote(i) for i in command)
        self._exit_code = exit_code
        self._output = output

    def get_brief(self) -> str:
        return f"Failed to execute git command: {self._command}"

    def get_details(self) -> str:
        return f"Command failed with exit code {self._exit_code!r} and output:\n{self._output}"

    def get_resolution(self) -> str:
        return "Consider checking your git configuration for settings which may cause issues."
