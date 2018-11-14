# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import click

from ._options import get_project
from snapcraft.internal import errors
from snapcraft.project._sanity_checks import conduct_project_sanity_check


@click.group()
@click.pass_context
def legacycli(ctx, **kwargs):
    pass


@legacycli.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
def cleanbuild():
    """This command is no longer available when using the base keyword."""
    # We can only end up here with an invalid yaml
    project = get_project()
    conduct_project_sanity_check(project)

    # if we got this far, the environment must be broken
    raise errors.SnapcraftEnvironmentError(
        "The cleanbuild command is no longer supported when using the base keyword."
    )
