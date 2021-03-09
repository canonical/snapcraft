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
import logging
import os
import pathlib
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import click

from snapcraft.cli.echo import confirm, prompt, warning
from snapcraft.internal import common, errors, log
from snapcraft.internal.meta.snap import Snap
from snapcraft.project import Project, get_snapcraft_yaml


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
        param_decls="--lxd-image-name",
        metavar="<image>",
        help="Override LXD image.",
        envvar="SNAPCRAFT_LXD_IMAGE_NAME",
        supported_providers=["lxd"],
        hidden=True,
    ),
    dict(
        param_decls="--lxd-image-remote",
        metavar="<remote>",
        help="Override LXD image remote.",
        envvar="SNAPCRAFT_LXD_IMAGE_REMOTE",
        supported_providers=["lxd"],
        hidden=True,
    ),
    dict(
        param_decls="--lxd-project",
        metavar="<project>",
        help="LXD project to use.",
        envvar="SNAPCRAFT_LXD_PROJECT",
        default="snapcraft",
        supported_providers=["lxd"],
        hidden=True,
    ),
    dict(
        param_decls="--lxd-remote",
        metavar="<remote>",
        help="LXD Remote to use.",
        envvar="SNAPCRAFT_LXD_REMOTE",
        default="local",
        supported_providers=["lxd"],
        hidden=True,
    ),
    dict(
        param_decls="--target-arch",
        metavar="<arch>",
        help="Target architecture to cross compile to",
        supported_providers=["host", "lxd", "multipass"],
    ),
    dict(
        param_decls="--debug",
        is_flag=True,
        help="Shells into the environment if the build fails.",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
    ),
    dict(
        param_decls="--shell",
        is_flag=True,
        help="Shells into the environment in lieu of the step to run.",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
    ),
    dict(
        param_decls="--shell-after",
        is_flag=True,
        help="Shells into the environment after the step has run.",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
    ),
    dict(
        param_decls="--destructive-mode",
        is_flag=True,
        help="Forces snapcraft to try and use the current host to build (implies `--provider=host`).",
        supported_providers=["host", "managed-host"],
    ),
    dict(
        param_decls="--use-lxd",
        is_flag=True,
        help="Forces snapcraft to use LXD to build (implies `--provider=lxd`).",
        supported_providers=["lxd"],
    ),
    dict(
        param_decls="--provider",
        envvar="SNAPCRAFT_BUILD_ENVIRONMENT",
        show_envvar=False,
        help="Build provider to use.",
        metavar="[{}]".format("|".join(_SUPPORTED_PROVIDERS)),
        type=click.Choice(_ALL_PROVIDERS),
        supported_providers=_ALL_PROVIDERS,
    ),
    dict(
        param_decls="--http-proxy",
        metavar="<http-proxy>",
        help="HTTP proxy for host build environments.",
        envvar="http_proxy",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
    ),
    dict(
        param_decls="--https-proxy",
        metavar="<https-proxy>",
        help="HTTPS proxy for host build environments.",
        envvar="https_proxy",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
    ),
    dict(
        param_decls="--add-ca-certificates",
        metavar="<certificate-path>",
        help="File or directory containing CA certificates to install into build environments.",
        envvar="SNAPCRAFT_ADD_CA_CERTIFICATES",
        supported_providers=["lxd", "multipass"],
        type=click.Path(
            exists=True, file_okay=True, dir_okay=True, readable=True, resolve_path=True
        ),
    ),
    dict(
        param_decls="--bind-ssh",
        is_flag=True,
        help="Bind ~/.ssh directory to locally-run build environments.",
        envvar="SNAPCRAFT_BIND_SSH",
        supported_providers=["lxd", "multipass"],
    ),
    dict(
        param_decls="--enable-developer-debug",
        is_flag=True,
        help="Enable developer debug logging.",
        envvar="SNAPCRAFT_ENABLE_DEVELOPER_DEBUG",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
        hidden=True,
    ),
    dict(
        param_decls="--enable-manifest",
        is_flag=True,
        type=BoolParamType(),
        help="Generate snap manifest.",
        envvar="SNAPCRAFT_BUILD_INFO",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
        hidden=True,
    ),
    dict(
        param_decls="--enable-snapshots",
        is_flag=True,
        type=BoolParamType(),
        help="Enable snapshots if supported by build provider.",
        envvar="SNAPCRAFT_ENABLE_SNAPSHOTS",
        supported_providers=["lxd"],
        hidden=True,
    ),
    dict(
        param_decls="--manifest-image-information",
        metavar="<json string>",
        help="Set snap manifest image-info",
        envvar="SNAPCRAFT_IMAGE_INFO",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
        hidden=True,
    ),
    dict(
        param_decls="--enable-experimental-extensions",
        is_flag=True,
        help="Enable extensions that are experimental and not considered stable.",
        envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
    ),
    dict(
        param_decls="--enable-experimental-package-repositories",
        is_flag=True,
        help="Enable `package-repositories` support in schema.",
        envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_PACKAGE_REPOSITORIES",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
    ),
    dict(
        param_decls="--enable-experimental-target-arch",
        is_flag=True,
        help="Enable experimental `--target-arch` support for core20.",
        envvar="SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH",
        supported_providers=["host", "lxd", "managed-host", "multipass"],
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
        click_option = click.option(param_decls, **option, hidden=hidden_override)
        func = click_option(func)
    return func


def add_provider_options(hidden=False):
    def _add_provider_options(func):
        return _add_options(_PROVIDER_OPTIONS, func, hidden)

    return _add_provider_options


def determine_provider(use_lxd: bool, destructive_mode: bool) -> str:
    """Determine provider based on user inputs & defaults."""
    if use_lxd:
        return "lxd"

    if destructive_mode:
        return "host"

    if common.is_process_container():
        return "host"

    # Default to multipass.
    return "multipass"


@dataclass
class ProviderOptions:
    bind_ssh: bool = False
    add_ca_certificates: Optional[pathlib.Path] = None
    debug: bool = False
    destructive_mode: bool = False
    enable_developer_debug: bool = True
    enable_experimental_extensions: bool = False
    enable_experimental_package_repositories: bool = False
    enable_experimental_target_arch: Optional[str] = None
    enable_manifest: bool = False
    enable_snapshots: bool = True
    http_proxy: Optional[str] = None
    https_proxy: Optional[str] = None
    lxd_image_name: Optional[str] = None
    lxd_image_remote: Optional[str] = None
    lxd_project: str = "snapcraft"
    lxd_remote: str = "local"
    manifest_image_information: Optional[str] = None
    provider: str = "multipass"
    shell: bool = False
    shell_after: bool = False
    target_arch: Optional[str] = None
    use_lxd: bool = False

    def apply_host_environment(self) -> None:
        """Set build environment flags in snapcraft process."""
        if self.enable_experimental_extensions:
            os.environ["SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS"] = "True"

        if self.enable_experimental_package_repositories:
            os.environ["SNAPCRAFT_ENABLE_EXPERIMENTAL_PACKAGE_REPOSITORIES"] = "True"

        if self.target_arch:
            os.environ["SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH"] = "True"

        if self.enable_manifest:
            os.environ["SNAPCRAFT_BUILD_INFO"] = "True"

        if self.manifest_image_information:
            os.environ["SNAPCRAFT_IMAGE_INFO"] = self.manifest_image_information

        if self.http_proxy:
            os.environ["http_proxy"] = self.http_proxy

        if self.https_proxy:
            os.environ["https_proxy"] = self.https_proxy

        if self.enable_developer_debug:
            log.configure(log_level=logging.DEBUG)

    def issue_build_provider_warnings(self) -> None:
        """Log any experimental flags in use."""
        if self.enable_experimental_extensions:
            warning("*EXPERIMENTAL* extensions enabled.")

        if self.enable_experimental_package_repositories:
            warning("*EXPERIMENTAL* package-repositories enabled.")

        if self.enable_experimental_target_arch:
            warning("*EXPERIMENTAL* --target-arch for core20 enabled.")

    def perform_sanity_checks(self) -> None:
        env_provider = os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT")

        # Specifying --provider=host requires the use of --destructive-mode.
        # Exceptions include:
        # (1) SNAPCRAFT_BUILD_ENVIRONMENT=host.
        # (2) Running inside of a container.
        if (
            self.provider == "host"
            and not env_provider == "host"
            and not self.destructive_mode
            and not common.is_process_container()
        ):
            raise click.BadArgumentUsage(
                "--provider=host requires --destructive-mode to acknowledge side effects"
            )

        if env_provider and env_provider != self.provider:
            raise click.BadArgumentUsage(
                "mismatch between --provider={} and SNAPCRAFT_BUILD_ENVIRONMENT={}".format(
                    self.provider, env_provider
                )
            )

        # Error if any sys.argv params are for unsupported providers.
        # Values from environment variables and configuration files only
        # change defaults, so they are safe to ignore due to filtering
        # in get_build_provider_flags().
        for option in _PROVIDER_OPTIONS:
            key: str = option["param_decls"]  # type: ignore
            supported_providers: List[str] = option["supported_providers"]  # type: ignore
            if key in sys.argv and self.provider not in supported_providers:
                raise click.BadArgumentUsage(
                    f"{key} cannot be used with build provider {self.provider!r}"
                )

        # Check if running as sudo but only if the host is not managed-host where Snapcraft
        # runs as root already. This effectively avoids the warning when using the default
        # build provider (Multipass) that uses "sudo" to get "root".
        if (
            self.provider != "managed-host"
            and os.getenv("SUDO_USER")
            and os.geteuid() == 0
        ):
            warning(
                "Running with 'sudo' may cause permission errors and is discouraged. Use 'sudo' when cleaning."
            )

    @classmethod
    def from_click_group_args(
        cls,
        add_ca_certificates: Optional[pathlib.Path],
        bind_ssh: bool,
        debug: bool,
        destructive_mode: bool,
        enable_developer_debug: bool,
        enable_manifest: bool,
        enable_experimental_extensions: bool,
        enable_experimental_package_repositories: bool,
        enable_experimental_target_arch: Optional[str],
        enable_snapshots: bool,
        http_proxy: Optional[str],
        https_proxy: Optional[str],
        lxd_image_name: Optional[str],
        lxd_image_remote: Optional[str],
        lxd_project: str,
        lxd_remote: str,
        manifest_image_information: Optional[str],
        provider: Optional[str],
        shell: bool,
        shell_after: bool,
        target_arch: Optional[str],
        use_lxd: bool,
        **kwargs,
    ) -> "ProviderOptions":
        """Initialize options from group (global) arguments."""
        if add_ca_certificates is not None:
            certs_path: Optional[pathlib.Path] = pathlib.Path(add_ca_certificates)
        else:
            certs_path = None

        if provider is None:
            provider = determine_provider(
                destructive_mode=destructive_mode, use_lxd=use_lxd
            )

        provider_options = ProviderOptions(
            bind_ssh=bind_ssh,
            add_ca_certificates=certs_path,
            debug=debug,
            destructive_mode=destructive_mode,
            enable_developer_debug=enable_developer_debug,
            enable_experimental_extensions=enable_experimental_extensions,
            enable_experimental_package_repositories=enable_experimental_package_repositories,
            enable_experimental_target_arch=enable_experimental_target_arch,
            enable_manifest=enable_manifest,
            enable_snapshots=enable_snapshots,
            http_proxy=http_proxy,
            https_proxy=https_proxy,
            lxd_image_name=lxd_image_name,
            lxd_image_remote=lxd_image_remote,
            lxd_project=lxd_project,
            lxd_remote=lxd_remote,
            manifest_image_information=manifest_image_information,
            provider=provider,
            use_lxd=use_lxd,
        )

        provider_options.perform_sanity_checks()
        return provider_options

    def update_from_click_command_args(  # noqa: C901
        self,
        add_ca_certificates: Optional[pathlib.Path],
        bind_ssh: bool,
        debug: bool,
        destructive_mode: bool,
        enable_developer_debug: bool,
        enable_manifest: bool,
        enable_experimental_extensions: bool,
        enable_experimental_package_repositories: bool,
        enable_experimental_target_arch: Optional[str],
        enable_snapshots: bool,
        http_proxy: Optional[str],
        https_proxy: Optional[str],
        lxd_image_name: Optional[str],
        lxd_image_remote: Optional[str],
        lxd_project: str,
        lxd_remote: str,
        manifest_image_information: Optional[str],
        provider: Optional[str],
        shell: bool,
        shell_after: bool,
        target_arch: Optional[str],
        use_lxd: bool,
        **kwargs,
    ) -> None:
        """Update provider options from command-specific arguments."""
        if bind_ssh:
            self.bind_ssh = True

        if add_ca_certificates:
            self.add_ca_certificates = pathlib.Path(add_ca_certificates)

        if debug:
            self.debug = True

        if destructive_mode:
            self.destructive_mode = True

        if enable_developer_debug:
            self.enable_developer_debug = True

        if enable_experimental_extensions:
            self.enable_experimental_extensions = True

        if enable_experimental_package_repositories:
            self.enable_experimental_package_repositories = True

        if enable_experimental_target_arch:
            self.enable_experimental_target_arch = enable_experimental_target_arch

        if enable_manifest:
            self.enable_manifest = True

        if enable_snapshots:
            self.enable_snapshots = True

        if http_proxy:
            self.http_proxy = http_proxy

        if https_proxy:
            self.https_proxy = https_proxy

        if lxd_image_name:
            self.lxd_image_name = lxd_image_name

        if lxd_image_remote:
            self.lxd_image_remote = lxd_image_remote

        if lxd_project:
            self.lxd_project = lxd_project

        if lxd_remote:
            self.lxd_remote = lxd_remote

        if manifest_image_information:
            self.manifest_image_information = manifest_image_information

        if provider:
            self.provider = provider

        if shell:
            self.shell = True

        if shell_after:
            self.shell_after = True

        if target_arch:
            self.target_arch = target_arch

        if use_lxd:
            self.use_lxd = True

        self.perform_sanity_checks()


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
    # TODO: this should be automatic on get_project().
    # This is not the complete meta parsed by the project loader.
    project._snap_meta = Snap.from_dict(project.info.get_raw_snapcraft())

    return project
