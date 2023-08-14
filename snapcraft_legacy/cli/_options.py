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

import distutils.util
import os
import sys
from typing import Any, Dict, List, Optional

import click

from snapcraft_legacy.cli.echo import confirm, prompt, warning
from snapcraft_legacy.internal import common, errors
from snapcraft_legacy.internal.meta.snap import Snap
from snapcraft_legacy.project import Project, get_snapcraft_yaml


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


class BoolParamType(click.ParamType):
    name = "boolean"

    def convert(self, value, param, ctx):
        """Convert option string to value.

        Unlike click's BoolParamType, use distutils.util.strtobool to
        convert values.
        """
        if isinstance(value, bool):
            return value
        try:
            return bool(distutils.util.strtobool(value))
        except ValueError:
            self.fail("%r is not a valid boolean" % value, param, ctx)

    def __repr__(self):
        return "BOOL"


_SUPPORTED_PROVIDERS = ["host", "lxd", "multipass"]
_HIDDEN_PROVIDERS = ["managed-host"]
_ALL_PROVIDERS = _SUPPORTED_PROVIDERS + _HIDDEN_PROVIDERS
_PROVIDER_OPTIONS: List[Dict[str, Any]] = [
    dict(
        param_decls=["--target-arch"],
        metavar="<arch>",
        help="Target architecture to cross compile to",
        supported_providers=_SUPPORTED_PROVIDERS,
    ),
    dict(
        param_decls=["--debug"],
        is_flag=True,
        help="Shells into the environment if the build fails.",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--shell"],
        is_flag=True,
        help="Shells into the environment in lieu of the step to run.",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--offline"],
        is_flag=True,
        help="Operate in offline mode.",
        envvar="SNAPCRAFT_OFFLINE",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--shell-after"],
        is_flag=True,
        help="Shells into the environment after the step has run.",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--destructive-mode"],
        is_flag=True,
        help="Forces snapcraft to try and use the current host to build (implies `--provider=host`).",
        supported_providers=["host", "managed-host"],
    ),
    dict(
        param_decls=["--use-lxd"],
        is_flag=True,
        help="Forces snapcraft to use LXD to build (implies `--provider=lxd`).",
        supported_providers=["lxd"],
    ),
    dict(
        param_decls=["--provider"],
        envvar="SNAPCRAFT_BUILD_ENVIRONMENT",
        show_envvar=False,
        help="Build provider to use.",
        metavar="[{}]".format("|".join(_SUPPORTED_PROVIDERS)),
        type=click.Choice(_ALL_PROVIDERS),
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--http-proxy"],
        metavar="<http-proxy>",
        help="HTTP proxy for host build environments.",
        envvar="http_proxy",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--https-proxy"],
        metavar="<https-proxy>",
        help="HTTPS proxy for host build environments.",
        envvar="https_proxy",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--add-ca-certificates"],
        metavar="<certificate-path>",
        help="File or directory containing CA certificates to install into build environments.",
        envvar="SNAPCRAFT_ADD_CA_CERTIFICATES",
        supported_providers=["lxd", "multipass"],
        type=click.Path(
            exists=True, file_okay=True, dir_okay=True, readable=True, resolve_path=True
        ),
    ),
    dict(
        param_decls=["--bind-ssh"],
        is_flag=True,
        help="Bind ~/.ssh directory to locally-run build environments.",
        envvar="SNAPCRAFT_BIND_SSH",
        supported_providers=["lxd", "multipass"],
    ),
    dict(
        param_decls=["--enable-developer-debug"],
        is_flag=True,
        help="Enable developer debug logging.",
        envvar="SNAPCRAFT_ENABLE_DEVELOPER_DEBUG",
        supported_providers=_ALL_PROVIDERS,
        hidden=True,
    ),
    dict(
        param_decls=["--enable-manifest"],
        is_flag=True,
        type=BoolParamType(),
        help="Generate snap manifest.",
        envvar="SNAPCRAFT_BUILD_INFO",
        supported_providers=_ALL_PROVIDERS,
        hidden=True,
    ),
    dict(
        param_decls=["--manifest-image-information"],
        metavar="<json string>",
        help="Set snap manifest image-info",
        envvar="SNAPCRAFT_IMAGE_INFO",
        supported_providers=_ALL_PROVIDERS,
        hidden=True,
    ),
    dict(
        param_decls=["--enable-experimental-extensions"],
        is_flag=True,
        help="Enable extensions that are experimental and not considered stable.",
        envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--enable-experimental-plugins"],
        is_flag=True,
        help="Enable plugins that are experimental and not considered stable.",
        envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_PLUGINS",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--enable-experimental-target-arch"],
        is_flag=True,
        help="Enable experimental `--target-arch` support for core20.",
        envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--ua-token"],
        metavar="<ua-token>",
        help="Configure build environment with ESM using specified UA token.",
        envvar="SNAPCRAFT_UA_TOKEN",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--enable-experimental-ua-services"],
        is_flag=True,
        help="Allow selection of UA services to enable.",
        envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_UA_SERVICES",
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls=["--verbose", "-v"],
        is_flag=True,
        help="Show debug information and be more verbose.",
        supported_providers=_ALL_PROVIDERS,
    ),
]


def _add_options(options, func, hidden):
    for option in reversed(options):
        # Pop param_decls option to prevent exception further on down the
        # line for: `got multiple values for keyword argument`.
        option = option.copy()
        param_decls = option.pop("param_decls")

        # Pop supported_providers option because it is snapcraft-only.
        if "supported_providers" in option:
            option.pop("supported_providers")

        hidden_override = option.pop("hidden", hidden)
        click_option = click.option(*param_decls, **option, hidden=hidden_override)
        func = click_option(func)
    return func


def add_provider_options(hidden=False):
    def _add_provider_options(func):
        return _add_options(_PROVIDER_OPTIONS, func, hidden)

    return _add_provider_options


def _sanity_check_build_provider_flags(build_provider: str, **kwargs) -> None:
    destructive_mode = kwargs.get("destructive_mode")
    env_provider = os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT")

    # Specifying --provider=host requires the use of --destructive-mode.
    # Exceptions include:
    # (1) SNAPCRAFT_BUILD_ENVIRONMENT=host.
    # (2) Running inside of a container.
    if (
        build_provider == "host"
        and not env_provider == "host"
        and not destructive_mode
        and not common.is_process_container()
    ):
        raise click.BadArgumentUsage(
            "--provider=host requires --destructive-mode to acknowledge side effects"
        )

    if env_provider and env_provider != build_provider:
        raise click.BadArgumentUsage(
            "mismatch between --provider={} and SNAPCRAFT_BUILD_ENVIRONMENT={}".format(
                build_provider, env_provider
            )
        )

    # Error if any sys.argv params are for unsupported providers.
    # Values from environment variables and configuration files only
    # change defaults, so they are safe to ignore due to filtering
    # in get_build_provider_flags().
    for option in _PROVIDER_OPTIONS:
        keys: List[str] = option["param_decls"]  # type: ignore
        supported_providers: List[str] = option["supported_providers"]  # type: ignore
        for key in keys:
            if key in sys.argv and build_provider not in supported_providers:
                raise click.BadArgumentUsage(
                    f"{key} cannot be used with build provider {build_provider!r}"
                )

    # Check if running as sudo but only if the host is not managed-host where Snapcraft
    # runs as root already. This effectively avoids the warning when using the default
    # build provider (Multipass) that uses "sudo" to get "root".
    if (
        build_provider != "managed-host"
        and os.getenv("SUDO_USER")
        and os.geteuid() == 0
    ):
        warning(
            "Running with 'sudo' may cause permission errors and is discouraged. Use 'sudo' when cleaning."
        )


def get_build_provider(**kwargs) -> str:
    """Get the appropriate provider and determine if running as a managed instance.

    To determine the appropriate provider:
    (1) use provider specified by `--provider=<provider>`
    (2) use provider from the environment variable `SNAPCRAFT_BUILD_ENVIRONMENT`
        (`click` loads this env var into `--provider=<provider>`)
    (3) use provider 'lxd' if `--use-lxd` was passed
    (4) use provider 'host' if `--destructive-mode` was passed
    (5) use provider 'host' if snapcraft is running as a process container
    (6) use provider specified by snap config
    (7) else default to multipass

    :return: provider name
    :raises ValueError: if provider is not supported
    """
    # load provider from snap config
    snap_config = common.get_snap_config()
    snap_provider = snap_config.get("provider") if snap_config else None

    # (1) use provider specified by `--provider=<provider>`
    # (2) use provider from the environment variable `SNAPCRAFT_BUILD_ENVIRONMENT`
    provider = kwargs.get("provider")

    if not provider:
        # (3) use provider 'lxd' if `--use-lxd` was passed
        if kwargs.get("use_lxd"):
            provider = "lxd"
        # (4) use provider 'host' if `--destructive-mode` was passed
        elif kwargs.get("destructive_mode"):
            provider = "host"
        # (5) use provider 'host' if snapcraft is running as a process container
        elif common.is_process_container():
            provider = "host"
        # (6) use provider specified by snap config
        elif snap_provider:
            # ignore case
            provider = snap_provider.lower()
        # (7) else default to multipass
        else:
            provider = "multipass"

    if provider not in _ALL_PROVIDERS:
        raise ValueError(f"unsupported provider specified: {provider!r}")

    _sanity_check_build_provider_flags(provider, **kwargs)

    return provider


def _param_decls_to_kwarg(key: str) -> str:
    """Format a param_decls to keyword argument name."""
    # Drop leading dashes ("-" or "--")
    if key.startswith("--"):
        key = key.replace("--", "", 1)
    elif key.startswith("-"):
        key = key.replace("-", "", 1)

    # Convert dashes to underscores.
    return key.replace("-", "_")


def get_build_provider_flags(build_provider: str, **kwargs) -> Dict[str, str]:
    """Get provider options from kwargs for a build provider.

    Boolean options that are false are not collected from kwargs.
    Options without an environment variable are not collected from kwargs.

    :param build_provider: Build provider to collect options for. Valid providers are
    'host', 'lxd', 'multipass', and 'managed-host'.
    :param kwargs: Dictionary containing provider options.

    :return: Dictionary of provider options with their environment variable as the key.

    :raises RuntimeError: If build provider is invalid.
    """
    build_provider_flags: Dict[str, str] = {}

    # Should not happen - developer safety check.
    if build_provider not in _ALL_PROVIDERS:
        raise RuntimeError(f"Invalid build provider: {build_provider}")

    for option in _PROVIDER_OPTIONS:
        keys: List[str] = option["param_decls"]  # type: ignore
        is_flag: bool = option.get("is_flag", False)  # type: ignore
        envvar: Optional[str] = option.get("envvar")  # type: ignore
        supported_providers: List[str] = option["supported_providers"]  # type: ignore

        for key in keys:
            # TODO: skip single character flags (e.g. keep `verbose` but discard `v`)

            # Skip --provider option.
            if key == "--provider":
                continue

            # Skip options that do not apply to configured provider.
            if build_provider not in supported_providers:
                continue

            # Skip boolean flags that have not been set.
            key_formatted = _param_decls_to_kwarg(key)
            if is_flag and not kwargs.get(key_formatted):
                continue

            # Add build provider flag using envvar as key.
            if envvar is not None and key_formatted in kwargs:
                build_provider_flags[envvar] = kwargs[key_formatted]

    return build_provider_flags


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

    # Validate yaml info from schema prior to consumption.
    if project.info is not None:
        project.info.validate_raw_snapcraft()

        # TODO: this should be automatic on get_project().
        # This is not the complete meta parsed by the project loader.
        project._snap_meta = Snap.from_dict(project.info.get_raw_snapcraft())

    return project


def apply_host_provider_flags(build_provider_flags: Dict[str, str]) -> None:
    """Set build environment flags in snapcraft process."""

    # Snapcraft plugins currently check for environment variables,
    # e.g. http_proxy and https_proxy.  Ensure they are set if configured.
    for key, value in build_provider_flags.items():
        if value is None:
            if key in os.environ:
                os.environ.pop(key)
        else:
            os.environ[key] = str(value)

    # Clear false/unset boolean environment flags.
    for option in _PROVIDER_OPTIONS:
        if not option.get("is_flag", False):
            continue

        env_name = option.get("envvar")
        if env_name is None:
            continue

        if not build_provider_flags.get(env_name):
            os.environ.pop(env_name, None)
            continue

    issue_build_provider_warnings(build_provider_flags)


def issue_build_provider_warnings(build_provider_flags: Dict[str, str]) -> None:
    # Log any experimental flags in use.
    if build_provider_flags.get("SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS"):
        warning("*EXPERIMENTAL* extensions enabled.")

    if build_provider_flags.get("SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH"):
        warning("*EXPERIMENTAL* --target-arch for core20 enabled.")

    if build_provider_flags.get("SNAPCRAFT_OFFLINE"):
        warning("*EXPERIMENTAL* --offline enabled.")
