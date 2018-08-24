# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import tarfile
from typing import Callable, Sequence

from snapcraft.project import Project
from snapcraft.internal import build_providers, errors, lxd


def _create_tar_filter(tar_filename) -> Callable:
    def _tar_filter(tarinfo):
        fn = tarinfo.name
        if fn.startswith("./parts/") and not fn.startswith("./parts/plugins"):
            return None
        elif fn in ("./stage", "./prime", tar_filename):
            return None
        elif fn.endswith(".snap"):
            return None
        elif fn.endswith("_source.tar.bz2"):
            return None
        return tarinfo

    return _tar_filter


def containerbuild(
    *, command: str, project: Project, output: str = None, args: Sequence[str] = None
) -> None:
    if args is None:
        args = []

    lxd.Project(output=output, source=os.path.curdir, project=project).execute(
        command, args
    )


def cleanbuild(*, project: Project, echoer, build_environment, remote: str = "") -> str:
    tar_filename = _create_tar_file(project.info.name)

    if build_environment.is_lxd:
        return _deprecated_cleanbuild(project, remote, tar_filename)

    build_provider_class = build_providers.get_provider_for("multipass")
    with build_provider_class(
        project=project, echoer=echoer, is_ephemeral=True
    ) as instance:
        instance.provision_project(tar_filename)
        instance.build_project()
        instance.retrieve_snap()
        return instance.snap_filename


def _create_tar_file(project_name: str) -> str:
    tar_filename = "{}_source.tar.bz2".format(project_name)
    with tarfile.open(tar_filename, "w:bz2") as t:
        t.add(os.path.curdir, filter=_create_tar_filter(tar_filename))

    return tar_filename


def _deprecated_cleanbuild(project: Project, remote: str, tar_filename: str) -> str:
    if remote and not lxd._remote_is_valid(remote):
        raise errors.InvalidContainerRemoteError(remote)

    builder = lxd.Cleanbuilder(source=tar_filename, project=project, remote=remote)
    builder.execute()
    return builder.snap_filename
