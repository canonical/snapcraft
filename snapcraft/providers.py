# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

"""Snapcraft-specific code to interface with craft-providers."""
import io
import os
import sys
from pathlib import Path
from textwrap import dedent
from typing import Dict, Optional

from craft_cli import emit
from craft_providers import Provider, ProviderError, bases, executor
from craft_providers.lxd import LXDProvider
from craft_providers.multipass import MultipassProvider

from snapcraft.snap_config import get_snap_config
from snapcraft.utils import (
    confirm_with_user,
    get_managed_environment_home_path,
    get_managed_environment_log_path,
    get_managed_environment_project_path,
    get_managed_environment_snap_channel,
)

SNAPCRAFT_BASE_TO_PROVIDER_BASE = {
    "core18": bases.BuilddBaseAlias.BIONIC,
    "core20": bases.BuilddBaseAlias.FOCAL,
    "core22": bases.BuilddBaseAlias.JAMMY,
    "devel": bases.BuilddBaseAlias.DEVEL,
}

# TODO: move to a package data file for shellcheck and syntax highlighting
# pylint: disable=line-too-long
BASHRC = dedent(
    """\
    #!/bin/bash

    env_file="$HOME/environment.sh"

    # save default environment on first login
    if [[ ! -e $env_file ]]; then
        env > "$env_file"
        sed -i 's/^/export /' "$env_file"      # prefix 'export' to variables
        sed -i 's/=/="/; s/$/"/' "$env_file"   # surround values with quotes
        sed -i '1i#! /bin/bash\\n' "$env_file" # add shebang
        fi
        previous_pwd=$PWD

    function set_environment {
        # only update the environment when the directory changes
        if [[ ! $PWD = "$previous_pwd" ]]; then
            # set part's environment when inside a part's build directory
            if [[ "$PWD" =~ $HOME/parts/.*/build ]] && [[ -e "${PWD/build*/run/environment.sh}" ]] ; then
                part_name=$(echo "${PWD#$"HOME"}" | cut -d "/" -f 3)
                echo "build environment set for part '$part_name'"
                # shellcheck disable=SC1090
                source "${PWD/build*/run/environment.sh}"

            # else clear and set the default environment
            else
                # shellcheck disable=SC2046
                unset $(/usr/bin/env | /usr/bin/cut -d= -f1)
                # shellcheck disable=SC1090
                source "$env_file"
                PWD="$(pwd)"
                export PWD
            fi
        fi
        previous_pwd=$PWD
    }

    function set_prompt {
        # do not show path in HOME directory
        if [[ "$PWD" = "$HOME" ]]; then
            export PS1="\\h # "

        # show relative path inside a subdirectory of HOME
        elif [[ "$PWD" =~ ^$HOME/* ]]; then
            export PS1="\\h ..${PWD/$HOME/}# "

        # show full path outside the home directory
        else
            export PS1="\\h $PWD# "
        fi
    }

    PROMPT_COMMAND="set_environment; set_prompt"
    """
)
# pylint: enable=line-too-long


def capture_logs_from_instance(instance: executor.Executor) -> None:
    """Capture and emit snapcraft logs from an instance.

    :param instance: instance to retrieve logs from
    """
    source_log_path = get_managed_environment_log_path()
    with instance.temporarily_pull_file(
        source=source_log_path, missing_ok=True
    ) as log_path:
        if log_path:
            emit.debug("Logs retrieved from managed instance:")
            with open(log_path, "r", encoding="utf8") as log_file:
                for line in log_file:
                    emit.debug(":: " + line.rstrip())
        else:
            emit.debug(
                f"Could not find log file {source_log_path.as_posix()} in instance."
            )


def ensure_provider_is_available(provider: Provider) -> None:
    """Ensure provider is installed, running, and properly configured.

    If the provider is not installed, the user is prompted to install it.

    :param instance: the provider to ensure is available

    :raises ProviderError: if provider is unknown, not available, or if the user
    chooses not to install the provider.
    """
    if isinstance(provider, LXDProvider):
        if not LXDProvider.is_provider_installed() and not confirm_with_user(
            "LXD is required but not installed. Do you wish to install LXD and configure "
            "it with the defaults?",
            default=False,
        ):
            raise ProviderError(
                "LXD is required, but not installed. Visit https://snapcraft.io/lxd "
                "for instructions on how to install the LXD snap for your distribution",
            )
        LXDProvider.ensure_provider_is_available()
    elif isinstance(provider, MultipassProvider):
        if not MultipassProvider.is_provider_installed() and not confirm_with_user(
            "Multipass is required but not installed. Do you wish to install Multipass"
            " and configure it with the defaults?",
            default=False,
        ):
            raise ProviderError(
                "Multipass is required, but not installed. Visit https://multipass.run/"
                "for instructions on installing Multipass for your operating system."
            )
        MultipassProvider.ensure_provider_is_available()
    else:
        raise ProviderError("cannot install unknown provider")


def get_base_configuration(
    *,
    alias: bases.BuilddBaseAlias,
    instance_name: str,
    http_proxy: Optional[str] = None,
    https_proxy: Optional[str] = None,
) -> bases.BuilddBase:
    """Create a BuilddBase configuration for rockcraft."""
    environment = get_command_environment(
        http_proxy=http_proxy, https_proxy=https_proxy
    )

    # injecting a snap on a non-linux system is not supported, so default to
    # install snapcraft from the store's stable channel
    snap_channel = get_managed_environment_snap_channel()
    if sys.platform != "linux" and not snap_channel:
        snap_channel = "stable"

    return bases.BuilddBase(
        alias=alias,
        compatibility_tag=f"snapcraft-{bases.BuilddBase.compatibility_tag}.0",
        environment=environment,
        hostname=instance_name,
        snaps=[
            bases.buildd.Snap(
                name="snapcraft",
                channel=snap_channel,
                classic=True,
            )
        ],
        # Requirement for apt gpg and version:git
        packages=["gnupg", "dirmngr", "git"],
    )


def get_command_environment(
    http_proxy: Optional[str] = None, https_proxy: Optional[str] = None
) -> Dict[str, Optional[str]]:
    """Construct an environment needed to execute a command.

    :param http_proxy: http proxy to add to environment
    :param https_proxy: https proxy to add to environment

    :return: Dictionary of environmental variables.
    """
    env = bases.buildd.default_command_environment()
    env["SNAPCRAFT_MANAGED_MODE"] = "1"

    # Pass-through host environment that target may need.
    for env_key in [
        "http_proxy",
        "https_proxy",
        "no_proxy",
        "SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS",
        "SNAPCRAFT_BUILD_FOR",
        "SNAPCRAFT_BUILD_INFO",
        "SNAPCRAFT_IMAGE_INFO",
        "SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT",
    ]:
        if env_key in os.environ:
            env[env_key] = os.environ[env_key]

    # if http[s]_proxy was specified as an argument, then prioritize this proxy
    # over the proxy from the host's environment.
    if http_proxy:
        env["http_proxy"] = http_proxy
    if https_proxy:
        env["https_proxy"] = https_proxy

    return env


def get_instance_name(
    *, project_name: str, project_path: Path, build_on: str, build_for: str
) -> str:
    """Formulate the name for an instance using each of the given parameters.

    Incorporate each of the parameters into the name to come up with a
    predictable naming schema that avoids name collisions across multiple
    projects.

    :param project_name: Name of the project.
    :param project_path: Directory of the project.
    """
    return "-".join(
        [
            "snapcraft",
            project_name,
            "on",
            build_on,
            "for",
            build_for,
            str(project_path.stat().st_ino),
        ]
    )


def get_provider(provider: Optional[str] = None) -> Provider:
    """Get the configured or appropriate provider for the host OS.

    To determine the appropriate provider,
    (1) use provider specified in the function argument,
    (2) get the provider from the environment,
    (3) use provider specified with snap configuration,
    (4) default to platform default (LXD on Linux, otherwise Multipass).

    :return: Provider instance.
    """
    chosen_provider = ""
    env_provider = os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT")

    # load snap config file
    snap_config = get_snap_config()
    snap_provider = snap_config.provider if snap_config else None

    # (1) use provider specified in the function argument
    if provider:
        emit.debug(f"Using provider {provider!r} passed as an argument.")
        chosen_provider = provider

    # (2) get the provider from the environment
    elif env_provider:
        emit.debug(
            f"Using provider {env_provider!r} from environmental "
            "variable 'SNAPCRAFT_BUILD_ENVIRONMENT'."
        )
        chosen_provider = env_provider

    # (3) use provider specified with snap configuration
    elif snap_provider:
        emit.debug(f"Using provider {snap_provider!r} from snap config.")
        chosen_provider = snap_provider

    # (4) default to platform default (LXD on Linux, otherwise Multipass)
    elif sys.platform == "linux":
        emit.debug("Using default provider 'lxd' on linux system.")
        chosen_provider = "lxd"
    else:
        emit.debug("Using default provider 'multipass' on non-linux system.")
        chosen_provider = "multipass"

    # ignore case
    chosen_provider = chosen_provider.lower()

    # return the chosen provider
    if chosen_provider == "lxd":
        return LXDProvider(lxd_project="snapcraft")
    if chosen_provider == "multipass":
        return MultipassProvider()

    raise ValueError(f"unsupported provider specified: {chosen_provider!r}")


def prepare_instance(
    instance: executor.Executor, host_project_path: Path, bind_ssh: bool
) -> None:
    """Prepare an instance to run snapcraft.

    The preparation includes:
    - mounting the project directory
    - mounting the `.ssh` directory, if specified
    - setting up `.bashrc` and the command line prompt
    """
    # mount project
    instance.mount(
        host_source=host_project_path,
        target=get_managed_environment_project_path(),
    )

    # mount ssh directory
    if bind_ssh:
        instance.mount(
            host_source=Path.home() / ".ssh",
            target=get_managed_environment_home_path() / ".ssh",
        )

    instance.push_file_io(
        destination=Path("/root/.bashrc"),
        content=io.BytesIO(BASHRC.encode()),
        file_mode="644",
    )
