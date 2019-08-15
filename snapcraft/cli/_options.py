# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2019 Canonical Ltd
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

from . import env
from snapcraft.project import Project, get_snapcraft_yaml
from snapcraft.cli.echo import confirm, prompt
from snapcraft.internal import errors


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


_BUILD_OPTIONS = [
    dict(
        param_decls="--target-arch",
        metavar="<arch>",
        help="Target architecture to cross compile to",
    ),
    dict(
        param_decls="--debug",
        is_flag=True,
        help="Shells into the environment if the build fails.",
    ),
    dict(
        param_decls="--shell",
        is_flag=True,
        help="Shells into the environment in lieu of the step to run.",
    ),
    dict(
        param_decls="--shell-after",
        is_flag=True,
        help="Shells into the environment after the step has run.",
    ),
    dict(
        param_decls="--destructive-mode",
        is_flag=True,
        help="Forces snapcraft to try and use the current host to build.",
    ),
    dict(
        param_decls="--use-lxd",
        is_flag=True,
        help="Forces snapcraft to use LXD to build.",
    ),
]


def add_build_options(hidden=False):
    def _add_build_options(func):
        for build_option in reversed(_BUILD_OPTIONS):
            # Pop param_decls option to prevent exception further on down the
            # line for: `got multiple values for keyword argument`.
            build_option = build_option.copy()
            param_decls = build_option.pop("param_decls")
            option = click.option(param_decls, **build_option, hidden=hidden)
            func = option(func)
        return func

    return _add_build_options


def get_build_environment(**kwargs):
    force_use_lxd = kwargs.get("use_lxd")
    force_destructive_mode = kwargs.get("destructive_mode")

    if force_destructive_mode and force_use_lxd:
        raise click.BadOptionUsage(
            "--destructive-mode",
            "--use-lxd and --destructive-mode cannot be used together.",
        )
    elif force_use_lxd:
        provider = "lxd"
    elif force_destructive_mode:
        provider = "host"
    else:
        provider = None
    return env.BuilderEnvironmentConfig(force_provider=provider)


def get_project(*, is_managed_host: bool = False, **kwargs):
    # We need to do this here until we can get_snapcraft_yaml as part of Project.
    if is_managed_host:
        try:
            os.chdir(os.path.expanduser(os.path.join("~", "project")))
        except FileNotFoundError:
            # No project found (fresh environment).
            raise errors.ProjectNotFoundError()

    snapcraft_yaml_file_path = get_snapcraft_yaml()

    # This method may be called from a click.Command with no parent.
    ctx = click.get_current_context()
    if ctx.parent is not None:
        for key, value in ctx.parent.params.items():
            if not kwargs.get(key):
                kwargs[key] = value

    project = Project(
        debug=kwargs.pop("debug", False),
        target_deb_arch=kwargs.pop("target_arch", None),
        snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        is_managed_host=is_managed_host,
    )
    return project
