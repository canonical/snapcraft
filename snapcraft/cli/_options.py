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

from snapcraft.project import Project, get_snapcraft_yaml


class HiddenOption(click.Option):
    def get_help_record(self, ctx):
        pass


_BUILD_OPTION_NAMES = ["--enable-geoip", "--no-parallel-builds", "--target-arch"]

_BUILD_OPTIONS = [
    dict(
        is_flag=True,
        help=("Detect best candidate location for stage-packages using geoip"),
    ),
    dict(is_flag=True, help="Force a sequential build."),
    dict(metavar="<arch>", help="Target architecture to cross compile to"),
]


def add_build_options(hidden=False):
    def _add_build_options(func):
        for name, params in zip(
            reversed(_BUILD_OPTION_NAMES), reversed(_BUILD_OPTIONS)
        ):
            option = click.option(
                name, **params, cls=HiddenOption if hidden else click.Option
            )
            func = option(func)
        return func

    return _add_build_options


def get_project(*, skip_snapcraft_yaml: bool = False, **kwargs):
    if skip_snapcraft_yaml:
        snapcraft_yaml_file_path = None
    else:
        snapcraft_yaml_file_path = get_snapcraft_yaml()

    ctx = click.get_current_context()
    for key, value in ctx.parent.params.items():
        if not kwargs.get(key):
            kwargs[key] = value

    project = Project(
        debug=kwargs.pop("debug"),
        use_geoip=kwargs.pop("enable_geoip"),
        parallel_builds=not kwargs.pop("no_parallel_builds"),
        target_deb_arch=kwargs.pop("target_arch"),
        snapcraft_yaml_file_path=snapcraft_yaml_file_path,
    )
    return project
