# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import os
import sys
from pathlib import Path
from typing import Dict, Optional

from craft_cli import emit
from craft_providers import bases, executor

from snapcraft.utils import (
    get_managed_environment_log_path,
    get_managed_environment_snap_channel,
)

SNAPCRAFT_BASE_TO_PROVIDER_BASE = {
    "core18": bases.BuilddBaseAlias.BIONIC,
    "core20": bases.BuilddBaseAlias.FOCAL,
    "core22": bases.BuilddBaseAlias.JAMMY,
}


def capture_logs_from_instance(instance: executor.Executor) -> None:
    """Capture and emit snapcraft logs from an instance.

    :param instance: instance to retrieve logs from
    """
    source_log_path = get_managed_environment_log_path()
    with instance.temporarily_pull_file(
        source=source_log_path, missing_ok=True
    ) as log_path:
        if log_path:
            emit.trace("Logs retrieved from managed instance:")
            with open(log_path, "r", encoding="utf8") as log_file:
                for line in log_file:
                    emit.trace(":: " + line.rstrip())
        else:
            emit.trace(
                f"Could not find log file {source_log_path.as_posix()} in instance."
            )


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
