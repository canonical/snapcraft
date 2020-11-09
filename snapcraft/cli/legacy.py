# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

from snapcraft.internal import errors
from snapcraft.project._sanity_checks import conduct_project_sanity_check

from ._command import SnapcraftProjectCommand, run_legacy_snapcraft
from ._options import get_project


@click.group()
@click.pass_context
def legacycli(ctx, **kwargs):
    pass


@legacycli.command(
    cls=SnapcraftProjectCommand,
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True),
)
def cleanbuild():
    """Create a snap using a clean environment managed by a build provider.

    \b
    Examples:
        snapcraft cleanbuild

    The cleanbuild command requires a properly setup lxd environment that
    can connect to external networks. Refer to the "Ubuntu Desktop and
    Ubuntu Server" section on
    https://linuxcontainers.org/lxd/getting-started-cli
    to get started.

    If using a remote, a prior setup is required which is described on:
    https://linuxcontainers.org/lxd/getting-started-cli/#multiple-host

    This command is no longer available when using the base keyword."""
    # We can only end up here with an invalid yaml
    project = get_project()
    conduct_project_sanity_check(project)

    # if we got this far, the environment must be broken
    raise errors.SnapcraftEnvironmentError(
        "The cleanbuild command is no longer supported when using the base keyword."
    )


@legacycli.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
def define():
    """Shows the definition for the cloud part.

    \b
    Examples:
        snapcraft define my-part1

    This command is no longer available when using the base keyword.
    """
    run_legacy_snapcraft()


@legacycli.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
def update():
    """Updates the parts listing from the cloud.

    This command is no longer available when using the base keyword.
    """
    run_legacy_snapcraft()


@legacycli.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
def search():
    """Searches the remote parts cache for matching parts.

    \b
    Examples:
        snapcraft search desktop

    This command is no longer available when using the base keyword.
    """
    run_legacy_snapcraft()
