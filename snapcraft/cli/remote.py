# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019 Canonical Ltd
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

import click
import os
import uuid

from snapcraft.internal.remote_build import (
    Worktree,
    LaunchpadClient,
    Repo,
    InfoFile,
    errors,
)
from snapcraft.formatting_utils import humanize_list
from typing import List, Tuple
from xdg import BaseDirectory
from . import echo
from ._options import get_project

_SUPPORTED_ARCHS = ["amd64", "arm64", "armhf", "i386", "ppc64el", "s390x"]


@click.group()
def remotecli():
    """Remote build commands"""
    pass


@remotecli.command("remote-build")
@click.option(
    "--recover",
    metavar="<build-number>",
    type=int,
    nargs=1,
    required=False,
    help="Recover interrupted remote build.",
)
@click.option(
    "--status",
    metavar="<build-number>",
    type=int,
    nargs=1,
    required=False,
    help="Display remote build status.",
)
@click.option(
    "--arch",
    metavar="<arch-list>",
    type=str,
    nargs=1,
    required=False,
    help="Set architectures to build.",
)
@click.option(
    "--git", is_flag=True, required=False, help="Build a local git repository."
)
@click.option(
    "--user", metavar="<username>", nargs=1, required=False, help="Launchpad username."
)
def remote_build(
    recover: int, status: int, user: str, arch: str, git: bool, echoer=echo
) -> None:
    """Dispatch a snap for remote build.

    Command remote-build sends the current project to be built remotely. After the build
    is complete, packages for each architecture are retrieved and will be available in
    the local filesystem.

    If the local files are under git revision control, use option --git to build the
    current branch of this repository. Be aware, however, that untracked files and
    uncommitted changes will not be used.

    If not specified in the snapcraft.yaml file, the list of architectures to build
    can be set using the --arch option. If both are specified, the architectures listed
    in snapcraft.yaml take precedence.

    Interrupted remote builds can be resumed using the --recover option, followed by
    the build number informed when the remote build was originally dispatched. The
    current state of the remote build for each architecture can be checked using the
    --status option.

    \b
    Examples:
        snapcraft remote-build
        snapcraft remote-build --arch=all
        snapcraft remote-build --git --arch=arm64,armhf
        snapcraft remote-build --recover 47860738
        snapcraft remote-build --status 47860738
    """
    echo.warning(
        "snapcraft remote-build is offered as a preview. Authentication and transport "
        "mechanisms will change in future releases. Use with caution in scripts."
    )

    project = get_project()

    remote_dir = os.path.join(
        BaseDirectory.save_data_path("snapcraft"),
        "projects",
        project.info.name,
        "remote-build",
    )

    remote_info = InfoFile(os.path.join(remote_dir, "remote.yaml"))
    remote_info.load()
    provider = "launchpad"
    if "id" in remote_info:
        build_id = remote_info["id"]
        if "provider" in remote_info:
            provider = remote_info["provider"]
    else:
        build_id = "snapcraft-{}".format(uuid.uuid4().hex)
        remote_info["id"] = build_id
        remote_info["provider"] = provider
        remote_info.save()

    # TODO: change login strategy after launchpad infrastructure is ready (LP #1827679)
    lp = LaunchpadClient(project, build_id)
    lp.login(user)

    if status:
        # Show build status
        lp.recover_build(status)
        for arch, build_status in lp.get_build_status():
            echo.info("{}: {}".format(arch, build_status))
    elif recover:
        # Recover from interrupted build
        echo.info("Recover build {}...".format(recover))
        lp.recover_build(recover)
        _monitor_build(lp)
    else:
        # If build architectures not set in snapcraft.yaml, let the user override
        # Launchpad defaults using --arch.
        project_architectures = _get_project_architectures(project)
        if project_architectures and arch:
            raise click.BadOptionUsage(
                "Cannot use --arch, architecture list is already set in snapcraft.yaml."
            )
        archs = _choose_architectures(project_architectures, arch)

        # Sanity check for build architectures
        _check_supported_architectures(archs)

        # The default branch to build
        branch = "master"

        # Send local data to the remote repository
        echo.info("Sending data to remote builder...")
        if git:
            url, branch = _send_current_tree(provider, lp.user, build_id + "-git")
        else:
            url = _copy_and_send_tree(
                provider, lp.user, remote_dir, build_id, project.info.name
            )

        # Create build recipe
        # (delete any existing snap to remove leftovers from previous builds)
        lp.delete_snap()
        lp.create_snap(url, branch, archs)

        targets = " for {}".format(humanize_list(archs, "and", "{}")) if archs else ""
        echo.info(
            "Building package{}. This may take some time to finish.".format(targets)
        )

        # Start building
        req_number = lp.start_build(timeout=5, attempts=5)
        echo.info(
            "If interrupted, resume with: 'snapcraft remote-build --recover {}'".format(
                req_number
            )
        )
        _monitor_build(lp)


def _monitor_build(lp: LaunchpadClient) -> None:
    lp.monitor_build()
    echo.info("Build complete.")
    lp.delete_snap()


def _send_current_tree(provider: str, user: str, build_id: str) -> Tuple[str, str]:
    if not os.path.exists(".git"):
        raise errors.NotGitRepositoryError
    repo = Repo(".")
    branch = repo.branch_name
    if repo.is_dirty:
        echo.warning(
            "The following files have uncommitted changes that will not be used:"
        )
        echo.warning("\n".join(repo.uncommitted_files))
    url = repo.push_remote(provider, user, branch, build_id)
    return url, branch


def _copy_and_send_tree(
    provider: str, user: str, remote_dir: str, build_id: str, name: str
) -> str:
    if os.path.exists(".git"):
        echo.warning(
            "Git repository found, use option --git to build the current branch."
        )
    work_dir = os.path.join(remote_dir, build_id)
    wt = Worktree(
        ".",
        work_dir,
        ignore=[name + "_*.snap", "buildlog_*.txt*", "parts", "stage", "prime"],
    )
    url = wt.add_remote(provider, user, build_id)
    wt.sync()
    wt.push()
    return url


def _check_supported_architectures(archs: List[str]) -> None:
    unsupported_archs = []
    for item in archs:
        if item not in _SUPPORTED_ARCHS:
            unsupported_archs.append(item)
    if unsupported_archs:
        raise errors.UnsupportedArchitectureError(archs=unsupported_archs)


def _choose_architectures(project_architectures: List[str], arch: str) -> List[str]:
    if project_architectures:
        archs = project_architectures
    elif arch == "all":
        archs = _SUPPORTED_ARCHS
    else:
        archs = arch.split(",") if arch else []

    return archs


def _get_project_architectures(project) -> List[str]:
    archs = []
    if project.info.architectures:
        for item in project.info.architectures:
            if "build-on" in item:
                new_arch = item["build-on"]
                if isinstance(new_arch, list):
                    # FIXME: LP builders don't recognize a list of architectures
                    archs.extend(new_arch)
                else:
                    archs.append(new_arch)
    return archs
