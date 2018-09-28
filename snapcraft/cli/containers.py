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

import click
import os

from snapcraft.internal import lxd, repo
from ._options import get_project
from . import env


@click.group()
def containerscli():
    pass


@containerscli.command()
@click.option(
    "--debug", is_flag=True, help="Shells into the environment if the build fails."
)
def refresh(debug, **kwargs):
    """Refresh existing packages.

    \b
    Examples:
        snapcraft refresh
    """

    build_environment = env.BuilderEnvironmentConfig()

    if build_environment.is_lxd:
        project = get_project(**kwargs, debug=debug)
        lxd.Project(project=project, output=None, source=os.path.curdir).refresh()
    else:
        repo.Repo.refresh_build_packages()
