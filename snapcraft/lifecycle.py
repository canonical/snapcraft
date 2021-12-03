# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd.
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

"""Lifecycle integration."""

import subprocess
from pathlib import Path

from snapcraft import providers, ui, utils
from snapcraft.meta import snap_yaml
from snapcraft.parts import PartsLifecycle, Step
from snapcraft.projects import Project
from snapcraft.providers import capture_logs_from_instance


def run(project: Project):
    """Pack a snap."""
    destructive_mode = False  # XXX: obtain from command line

    managed_mode = utils.is_managed_mode()
    if not managed_mode and not destructive_mode:
        pack_in_provider(project)
        return

    if managed_mode:
        work_dir = utils.get_managed_environment_home_path()
    else:
        work_dir = Path("work").absolute()

    lifecycle = PartsLifecycle(project.parts, work_dir=work_dir,)
    lifecycle.run(Step.PRIME)

    prime_dir = lifecycle.prime_dir

    ui.emit.progress("Creating snap.yaml...")
    snap_yaml.write(project, prime_dir, arch=lifecycle.target_arch)
    ui.emit.message("Created snap.yaml", intermediate=True)

    ui.emit.progress("Creating snap package...")
    subprocess.run(["snap", "pack", "--compression=xz", prime_dir], capture_output=True)  # type: ignore
    ui.emit.message("Created snap package", intermediate=True)


def pack_in_provider(project: Project):
    """Pack image in provider instance."""
    provider = providers.get_provider()
    provider.ensure_provider_is_available()

    cmd = ["snapcraft"]

    # TODO: append appropriate command line arguments

    output_dir = utils.get_managed_environment_project_path()

    with provider.launched_environment(
        project_name=project.name, project_path=Path().absolute(), base=project.base
    ) as instance:
        try:
            instance.execute_run(
                cmd, check=True, cwd=output_dir,
            )
        except subprocess.CalledProcessError as err:
            capture_logs_from_instance(instance)
            raise providers.ProviderError(
                f"Failed to pack image '{project.name}:{project.version}'."
            ) from err
