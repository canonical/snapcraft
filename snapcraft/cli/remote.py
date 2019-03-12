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

from snapcraft.internal.remote_build import Worktree, LaunchpadClient, Repo, errors
from typing import Any, Dict, List, Tuple
from xdg import BaseDirectory
from . import echo
from ._options import get_project
from snapcraft import yaml_utils
from snapcraft.internal.sources import Git

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

    project = get_project()
    version = project.info.version

    remote_dir = os.path.join(
        BaseDirectory.save_data_path("snapcraft"),
        "projects",
        project.info.name,
        "remote-build",
    )

    remote_info = _load_info(remote_dir)
    provider = "launchpad"
    if "id" in remote_info:
        build_id = remote_info["id"]
        if "provider" in remote_info:
            provider = remote_info["provider"]
    else:
        build_id = "snapcraft-{}".format(uuid.uuid4().hex)
        remote_info["id"] = build_id
        remote_info["provider"] = provider
        _save_info(remote_dir, **remote_info)

    # TODO: change login strategy after launchpad infrastructure is ready
    lp = LaunchpadClient(project, build_id)
    lp.login(user)

    if status:
        # Show build status
        lp.recover_build(status)
        lp.show_build_status()
        return
    elif recover:
        # Recover from interrupted build
        echo.info("Recover build {}...".format(recover))
        lp.recover_build(recover)
    else:
        # If build architectures not set in snapcraft.yaml, let the user override
        # Launchpad defaults using --arch.
        archs = _list_architectures(project, arch)

        # Sanity check for build architectures
        _check_supported_archs(archs)

        # The default branch to build
        branch = "master"

        # Special handling for "git" version
        if version == "git":
            if git:
                version = Git.generate_version().replace("-dirty", "")
            else:
                raise errors.InvalidVersionGitError

        # Send local data to the remote repository
        echo.info("Sending data to remote builder...")
        if git:
            url, branch = _send_current_tree(provider, lp.user, build_id + "-git")
        else:
            url = _copy_and_send_tree(
                provider, lp.user, remote_dir, build_id, project.info.name
            )

        # Create build recipe
        lp.delete_snap()
        lp.create_snap(url, branch, archs)

        targets = " for {}".format(", ".join(archs)) if archs else ""
        echo.info(
            "Building package{}. This may take some time to finish.".format(targets)
        )

        # Start building
        req_number = lp.start_build()
        echo.info(
            "If interrupted, resume with: 'snapcraft remote-build --recover {}'".format(
                req_number
            )
        )

    lp.monitor_build(version)
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
        echo.warning("\n".join(repo.uncommited_files))
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


def _check_supported_archs(archs: List[str]) -> None:
    unsupported_archs = []
    for item in archs:
        if item not in _SUPPORTED_ARCHS:
            unsupported_archs.append(item)
    if unsupported_archs:
        raise errors.UnsupportedArchitectureError(archs=unsupported_archs)


def _list_architectures(project, arch: str) -> List[str]:
    arch_from_yaml = _project_architectures(project)
    if arch_from_yaml:
        if arch:
            echo.warning(
                "Architecture list already set in snapcraft.yaml, ignoring --arch option."
            )
        archs = arch_from_yaml
    elif arch == "all":
        archs = _SUPPORTED_ARCHS
    else:
        archs = arch.split(",") if arch else []

    return archs


def _project_architectures(project) -> List[str]:
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


def _load_info(path: str) -> Dict[str, Any]:
    filepath = os.path.join(path, "remote.yaml")
    if not os.path.exists(filepath):
        return dict()

    with open(filepath) as info_file:
        return yaml_utils.load(info_file)


def _save_info(path: str, **data: Dict[str, Any]) -> None:
    filepath = os.path.join(path, "remote.yaml")

    dirpath = os.path.dirname(filepath)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)

    with open(filepath, "w") as info_file:
        yaml_utils.dump(data, stream=info_file)
