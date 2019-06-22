# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

import os

import click

from snapcraft.project import Project, get_snapcraft_yaml
from snapcraft.cli.echo import confirm, prompt


class HiddenOption(click.Option):
    def get_help_record(self, ctx):
        pass


class PromptOption(click.Option):
    def prompt_for_value(self, ctx):
        default = self.get_default(ctx)

        # If this is a prompt for a flag we need to handle this
        # differently.
        if self.is_bool_flag:
            return confirm(self.prompt, default)

        return prompt(
            self.prompt,
            default=default,
            type=self.type,
            hide_input=self.hide_input,
            show_choices=self.show_choices,
            confirmation_prompt=self.confirmation_prompt,
            value_proc=lambda x: self.process_value(ctx, x),
        )


_BUILD_OPTION_NAMES = [
    "--enable-geoip",
    "--no-parallel-builds",
    "--target-arch",
    "--debug",
    "--shell",
    "--shell-after",
]

_BUILD_OPTIONS = [
    dict(
        is_flag=True,
        help="Detect best candidate location for stage-packages using geoip",
    ),
    dict(is_flag=True, help="Force a sequential build."),
    dict(metavar="<arch>", help="Target architecture to cross compile to"),
    dict(is_flag=True, help="Shells into the environment if the build fails."),
    dict(is_flag=True, help="Shells into the environment in lieu of the step to run."),
    dict(is_flag=True, help="Shells into the environment after the step has run."),
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


def get_project(
    *, is_managed_host: bool = False, skip_snapcraft_yaml: bool = False, **kwargs
):
    # We need to do this here until we can get_snapcraft_yaml as part of Project.
    if is_managed_host:
        os.chdir(os.path.expanduser(os.path.join("~", "project")))

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
        is_managed_host=is_managed_host,
    )
    return project
