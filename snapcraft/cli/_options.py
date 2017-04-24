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
import click

from snapcraft import ProjectOptions


_build_options = [
    click.option('--enable-geoip', is_flag=True,
                 help=('Detect best candidate location for stage-packages '
                       'using geoip')),
    click.option('--no-parallel-builds', is_flag=True,
                 help='Force a squential build.'),
    click.option('--target-arch', metavar='<arch>',
                 help='Target architecture to cross compile to'),
]


def add_build_options():
    def _add_build_options(func):
        for option in reversed(_build_options):
            func = option(func)
        return func
    return _add_build_options


def get_project_options(**kwargs):
    project_args = dict(
        use_geoip=kwargs.pop('enable_geoip'),
        parallel_builds=not kwargs.pop('no_parallel_builds'),
        target_deb_arch=kwargs.pop('target_arch'),
    )

    return ProjectOptions(**project_args)
