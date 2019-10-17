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

from snapcraft.project import Project, get_snapcraft_yaml
from snapcraft.cli.echo import confirm, prompt
from snapcraft.internal import common, errors
from typing import Tuple


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
]

_SUPPORTED_PROVIDERS = ["host", "lxd", "multipass"]
_HIDDEN_PROVIDERS = ["managed-host"]
_PROVIDER_OPTIONS = [
    dict(
        param_decls="--destructive-mode",
        is_flag=True,
        help="Forces snapcraft to try and use the current host to build (implies `--provider=host`).",
    ),
    dict(
        param_decls="--use-lxd",
        is_flag=True,
        help="Forces snapcraft to use LXD to build (implies `--provider=lxd`).",
    ),
    dict(
        param_decls="--provider",
        envvar="SNAPCRAFT_BUILD_ENVIRONMENT",
        show_envvar=False,
        help="Build provider to use.",
        metavar="[{}]".format("|".join(_SUPPORTED_PROVIDERS)),
        type=click.Choice(_SUPPORTED_PROVIDERS + _HIDDEN_PROVIDERS),
    ),
]


def _add_options(options, func, hidden):
    for option in reversed(options):
        # Pop param_decls option to prevent exception further on down the
        # line for: `got multiple values for keyword argument`.
        option = option.copy()
        param_decls = option.pop("param_decls")
        click_option = click.option(param_decls, **option, hidden=hidden)
        func = click_option(func)
    return func


def add_build_options(hidden=False):
    def _add_build_options(func):
        return _add_options(_BUILD_OPTIONS, func, hidden)

    return _add_build_options


def add_provider_options(hidden=False):
    def _add_provider_options(func):
        return _add_options(_PROVIDER_OPTIONS, func, hidden)

    return _add_provider_options


def _sanity_check_build_environment_flags(**kwargs):
    provider = kwargs.get("provider")
    use_lxd = kwargs.get("use_lxd")
    destructive_mode = kwargs.get("destructive_mode")
    env_provider = os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT")

    if destructive_mode and use_lxd:
        raise click.BadArgumentUsage(
            "--use-lxd and --destructive-mode cannot be used together."
        )

    # Specifying --provider=host requires the use of --destructive-mode.
    # SNAPCRAFT_BUILD_ENVIRONMENT=host does not.
    if provider == "host" and not destructive_mode and not env_provider:
        raise click.BadArgumentUsage(
            "--provider=host requires --destructive-mode to acknowledge side effects"
        )

    # Remaining checks are for conflicts when --provider is specified.
    if not provider:
        return

    if env_provider and env_provider != provider:
        raise click.BadArgumentUsage(
            "mismatch between --provider={} and SNAPCRAFT_BUILD_ENVIRONMENT={}".format(
                provider, env_provider
            )
        )

    if use_lxd and provider != "lxd":
        raise click.BadArgumentUsage(
            "--use-lxd cannot be used with --provider={}".format(provider)
        )

    if destructive_mode and provider != "host":
        raise click.BadArgumentUsage(
            "--destructive-mode cannot be used with --provider={}".format(provider)
        )


def get_build_environment(
    skip_sanity_checks: bool = False, **kwargs
) -> Tuple[str, bool]:
    """Get build provider and determine if running as managed instance."""

    # Sanity checks may be skipped for the purpose of checking legacy.
    if not skip_sanity_checks:
        _sanity_check_build_environment_flags(**kwargs)

    provider = kwargs.get("provider")

    if not provider:
        if kwargs.get("use_lxd"):
            provider = "lxd"
        elif kwargs.get("destructive_mode"):
            provider = "host"
        elif common.is_process_container():
            provider = "host"
        else:
            # Default is multipass.
            provider = "multipass"

    # Handle special case for managed-host.
    if provider == "managed-host":
        provider = "host"
        is_managed_host = True
    else:
        is_managed_host = False

    return provider, is_managed_host


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
