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
import os

from snapcraft.internal import errors, lxd, project_loader
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
        snapcraft refresh

    This will take care of updating the apt package cache, upgrading packages
    as needed as well as refreshing snaps.
    """

    build_environment = env.BuilderEnvironmentConfig()
    if build_environment.is_host:
        raise errors.SnapcraftEnvironmentError(
            "The 'refresh' command only applies to LXD containers but "
            "SNAPCRAFT_BUILD_ENVIRONMENT is not set or set to host.\n"
            "Maybe you meant to update the parts cache instead? "
            "You can do that with the following command:\n\n"
            "snapcraft update")

    project_options = get_project_options(**kwargs, debug=debug)
    config = project_loader.load_config(project_options)
    lxd.Project(project_options=project_options,
                output=None, source=os.path.curdir,
                metadata=config.get_metadata()).refresh()
