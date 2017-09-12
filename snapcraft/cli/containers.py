# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

from snapcraft.internal import errors, lifecycle
from ._options import get_project_options
from . import env


@click.group()
def containerscli():
    pass


@containerscli.command()
@click.option('--debug', is_flag=True,
              help='Shells into the environment if the build fails.')
def refresh(debug, **kwargs):
    """Refresh an existing LXD container.

    \b
    Examples:
        SNAPCRAFT_CONTAINER_BUILDS=1 snapcraft refresh

    This will take care of updating the apt package cache, upgrading packages
    as needed as well as refreshing snaps.
    """

    if not env.is_containerbuild():
        raise errors.SnapcraftEnvironmentError(
            "The 'refresh' command only applies to LXD containers but "
            "SNAPCRAFT_CONTAINER_BUILDS is not set or 0.\n"
            "Maybe you meant to update the parts cache instead? "
            "You can do that with the following command:\n\n"
            "snapcraft update")

    project_options = get_project_options(**kwargs, debug=debug)
    lifecycle.containerbuild('refresh', project_options)
