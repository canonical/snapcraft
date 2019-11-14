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

from snapcraft.project import Project
from snapcraft.internal.remote_build import WorkTree, LaunchpadClient, errors
from snapcraft.formatting_utils import humanize_list
from typing import List
from xdg import BaseDirectory
from . import echo
from ._config import enable_snapcraft_config_file
from ._options import get_project, PromptOption

_SUPPORTED_ARCHS = ["amd64", "arm64", "armhf", "i386", "ppc64el", "s390x"]


@click.group()
def remotecli():
    """Remote build commands"""
    pass


@remotecli.command("remote-build")
@enable_snapcraft_config_file()
@click.option("--recover", is_flag=True, help="Recover interrupted build.")
@click.option("--status", is_flag=True, help="Display remote build status.")
@click.option(
    "--build-on",
    metavar="<arch-list>",
    type=str,
    nargs=1,
    required=False,
    help="Set architectures to build on.",
)
@click.option(
    "--launchpad-accept-public-upload",
    is_flag=True,
    prompt=(
        "All data sent to remote builders will be publicly available. "
        "Are you sure you want to continue?"
    ),
    help="Acknowledge that uploaded code will be publicly available.",
    cls=PromptOption,
)
@click.option(
    "--launchpad-user",
    metavar="<username>",
    nargs=1,
    required=True,
    help="Launchpad username.",
    prompt="Launchpad username",
)
@click.option(
    "--package-all-sources",
    is_flag=True,
    help="Package all sources to send to remote builder, not just local sources.",
)
def remote_build(
    recover: int,
    status: int,
    build_on: str,
    launchpad_accept_public_upload: bool,
    launchpad_user: str,
    package_all_sources: bool,
    echoer=echo,
) -> None:
    """Dispatch a snap for remote build.

    Command remote-build sends the current project to be built remotely. After the build
    is complete, packages for each architecture are retrieved and will be available in
    the local filesystem.

    If not specified in the snapcraft.yaml file, the list of architectures to build
    can be set using the --build-on option. If both are specified, an error will occur.

    Interrupted remote builds can be resumed using the --recover option, followed by
    the build number informed when the remote build was originally dispatched. The
    current state of the remote build for each architecture can be checked using the
    --status option.

    \b
    Examples:
        snapcraft remote-build --launchpad-user <user>
        snapcraft remote-build --launchpad-user <user> --build-on=amd64
        snapcraft remote-build --launchpad-user <user> --build-on=amd64,arm64,armhf,i386,ppc64el,s390x
        snapcraft remote-build --launchpad-user <user> --recover 47860738
        snapcraft remote-build --launchpad-user <user> --status 47860738
    """
    if not launchpad_accept_public_upload:
        raise errors.AcceptPublicUploadError()

    echo.warning(
        "snapcraft remote-build is experimental and is subject to change - use with caution."
    )

    project = get_project()

    # TODO: use project.is_legacy() when available.
    base = project.info.get_build_base()
    if base is None:
        raise errors.BaseRequiredError()

    # Use a hash of current working directory to distinguish between other
    # potential project builds occurring in parallel elsewhere.
    project_hash = project._get_project_directory_hash()
    build_id = f"snapcraft-{project.info.name}-{project_hash}"
    architectures = _determine_architectures(project, build_on)

    lp = LaunchpadClient(
        project=project,
        build_id=build_id,
        user=launchpad_user,
        architectures=architectures,
    )
    lp.login()

    if status:
        _print_status(lp)
    elif recover:
        # Recover from interrupted build.
        if not lp.has_outstanding_build():
            echo.info("No build found.")
            return

        echo.info("Recovering build...")
        _monitor_build(lp)
    elif lp.has_outstanding_build():
        # There was a previous build that hasn't finished.
        # Recover from interrupted build.
        echo.info("Found previously started build.")
        _print_status(lp)

        if not echo.confirm("Do you wish to recover this build?", default=True):
            _clean_build(lp)
            _start_build(
                lp=lp,
                project=project,
                build_id=build_id,
                package_all_sources=package_all_sources,
            )
        _monitor_build(lp)
    else:
        _start_build(
            lp=lp,
            project=project,
            build_id=build_id,
            package_all_sources=package_all_sources,
        )
        _monitor_build(lp)


def _clean_build(lp: LaunchpadClient):
    echo.info("Cleaning existing builds and artifacts...")
    lp.cleanup()
    echo.info("Done.")


def _print_status(lp: LaunchpadClient):
    if lp.has_outstanding_build():
        build_status = lp.get_build_status()
        for arch, status in build_status.items():
            echo.info(f"Build status for arch {arch}: {status}")
    else:
        echo.info("No build found.")


def _start_build(
    *, lp: LaunchpadClient, project: Project, build_id: str, package_all_sources: bool
) -> None:
    # Pull/update sources for project.
    worktree_dir = BaseDirectory.save_data_path("snapcraft", "remote-build", build_id)
    wt = WorkTree(worktree_dir, project, package_all_sources=package_all_sources)
    repo_dir = wt.prepare_repository()
    lp.push_source_tree(repo_dir)

    # Start building.
    lp.start_build()
    echo.info("If interrupted, resume with: 'snapcraft remote-build --recover'")


def _monitor_build(lp: LaunchpadClient) -> None:
    target_list = humanize_list(lp.architectures, "and", "{}")
    echo.info(
        f"Building snap package for {target_list}. This may take some time to finish."
    )

    lp.monitor_build()

    echo.info("Build complete.")
    lp.cleanup()


def _check_supported_architectures(archs: List[str]) -> None:
    unsupported_archs = []
    for item in archs:
        if item not in _SUPPORTED_ARCHS:
            unsupported_archs.append(item)
    if unsupported_archs:
        raise errors.UnsupportedArchitectureError(archs=unsupported_archs)


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


def _determine_architectures(project: Project, user_specified_arch: str):
    # If build architectures not set in snapcraft.yaml, let the user override
    # Launchpad defaults using --build-on.
    project_architectures = _get_project_architectures(project)
    if project_architectures and user_specified_arch:
        raise click.BadOptionUsage(
            "Cannot use --build-on, architecture list is already set in snapcraft.yaml."
        )

    if project_architectures:
        archs = project_architectures
    elif user_specified_arch:
        archs = user_specified_arch.split(",")
    else:
        # Default to typical snapcraft behavior (build for host).
        archs = [project.deb_arch]

    # Sanity check for build architectures
    _check_supported_architectures(archs)

    return archs
