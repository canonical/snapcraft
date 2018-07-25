# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

from snapcraft import formatting_utils
from snapcraft.internal import errors

from typing import List


class SnapcraftSourceError(errors.SnapcraftError):
    pass


class VCSError(SnapcraftSourceError):
    fmt = "{message}"


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


class SourceUpdateUnsupportedError(SnapcraftSourceError):

    fmt = "Failed to update source: {source!s} sources don't support updating."

    def __init__(self, source):
        super().__init__(source=source)
