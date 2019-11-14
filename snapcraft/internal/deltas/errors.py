# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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


from snapcraft.internal.errors import SnapcraftError


class DeltaGenerationError(SnapcraftError):
    """A delta failed to generate."""

    fmt = (
        "Could not generate {delta_format} delta.\n"
        "stdout log: {stdout_path}\n"
        "stdout: \n{stdout}\n"
        "---------"
        "stderr log: {stderr_path}\n"
        "stderr: \n{stderr}\n"
        "---------"
        "returncode: {returncode}"
    )


class DeltaGenerationTooBigError(SnapcraftError):
    """The generated delta was too large."""

    fmt = "delta saving is less than {delta_min_percentage}%."


class DeltaFormatOptionError(SnapcraftError):
    """A delta format option is not in the defined list."""

    fmt = (
        "delta_format must be a option in {format_options_list}.\n"
        "for now delta_format={delta_format!r}"
    )
