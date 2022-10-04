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
from pathlib import Path
from typing import Dict, Optional

from craft_providers import bases


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
