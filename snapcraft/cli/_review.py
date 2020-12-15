# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019 Canonical Ltd
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

import sys

import click

from snapcraft.internal import review_tools

from . import echo


def review_snap(*, snap_file: str) -> None:
    # Review the snap.
    if review_tools.is_available():
        echo.info(
            "Running the review tools before pushing this snap to the Snap " "Store."
        )
        # TODO just raise when we can override.
        try:
            review_tools.run(snap_filename=snap_file)
        except review_tools.errors.ReviewError as review_error:
            echo.warning(review_error.get_brief())
            echo.warning(review_error.get_resolution())
            # This can get too colorful if we don't just print it.
            click.echo(review_error.get_details())
    elif sys.platform == "linux":
        echo.warning(
            "Install the review-tools from the Snap Store for enhanced checks "
            "before uploading this snap."
        )
