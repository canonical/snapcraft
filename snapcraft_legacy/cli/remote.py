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

import time
from typing import List

import click
from xdg import BaseDirectory

from snapcraft_legacy.formatting_utils import humanize_list
from snapcraft_legacy.internal.remote_build import LaunchpadClient, WorkTree, errors
from snapcraft_legacy.project import Project

from . import echo
from ._options import PromptOption, get_project

_SUPPORTED_ARCHS = ["amd64", "arm64", "armhf", "i386", "ppc64el", "riscv64", "s390x"]


@click.group()
def remotecli():
    """Remote build commands"""


@remotecli.command("remote-build")
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
    "--build-for",
    metavar="<arch-list>",
    type=str,
    nargs=1,
    required=False,
    help="Set architectures to build for.",
)
@click.option(
    "--build-id",
    metavar="<build-id>",
    type=str,
    nargs=1,
    required=False,
    help="Build ID to use. If not provided, build ID will be calculated from working tree.",
)
@click.option(
    "--launchpad-accept-public-upload",
    is_flag=True,
    help="Acknowledge that uploaded code will be publicly available.",
    cls=PromptOption,
)
@click.option(
    "--launchpad-timeout",
    metavar="<seconds>",
    type=int,
    nargs=1,
    required=False,
    help=(
        "Time to wait for Launchpad to build before exiting ('0' disables timeout)."
        "Note that the build will continue on Launchpad and can be resumed later."
    ),
    default=0,
)
@click.option(
    "--package-all-sources",
    is_flag=True,
    help="Package all sources to send to remote builder, not just local sources.",
)
def remote_build(
    recover: bool,
    status: bool,
    build_on: str,
    build_for: str,
    build_id: str,
    launchpad_accept_public_upload: bool,
    launchpad_timeout: int,
    package_all_sources: bool,
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
        snapcraft remote-build
        snapcraft remote-build --build-on=amd64
        snapcraft remote-build --build-on=amd64,arm64,armhf,i386,ppc64el,riscv64,s390x
        snapcraft remote-build --recover
        snapcraft remote-build --recover --build-id snapcraft-my-snap-b98a6bd3
        snapcraft remote-build --status
        snapcraft remote-build --status --build-id snapcraft-my-snap-b98a6bd3
    """
    project = get_project()

    if build_for:
        build_on = build_for

    try:
        project._get_build_base()
    except RuntimeError:
        raise errors.BaseRequiredError()

    if not build_id:
        # If the option wasn't provided, use the project directory hash
        # to create one unique to the status of the working tree, allowing
        # us to distinguish between multiple builds for this project
        # (and others).
        project_hash = project._get_project_directory_hash()
        build_id = f"snapcraft-{project.info.name}-{project_hash}"

    echo.info(f"Using build ID {build_id}")
    architectures = _determine_architectures(project, build_on)

    # Calculate timeout timestamp, if specified.
    if launchpad_timeout > 0:
        deadline = int(time.time()) + launchpad_timeout
    else:
        deadline = 0

    lp = LaunchpadClient(
        project=project,
        build_id=build_id,
        architectures=architectures,
        deadline=deadline,
    )

    if status:
        _print_status(lp)
        return

    has_outstanding_build = lp.has_outstanding_build()
    if recover and not has_outstanding_build:
        echo.info("No build task(s) found.")
        return
    elif has_outstanding_build:
        echo.info("Found previously started build.")
        _print_status(lp)

        # If recovery specified, monitor build and exit.
        if recover or echo.confirm("Do you wish to recover this build?", default=True):
            _monitor_build(lp)
            return

        # Otherwise clean running build before we start a new one.
        _clean_build(lp)

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
        echo.info("No build task(s) found.")


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
    echo.info(
        f"If interrupted, resume with: 'snapcraft remote-build --recover --build-id {build_id}'"
    )


def _monitor_build(lp: LaunchpadClient) -> None:
    target_list = humanize_list(lp.architectures, "and", "{}")
    echo.info(
        f"Building snap package for {target_list}. This may take some time to finish."
    )

    try:
        lp.monitor_build()
    finally:
        echo.info("Build task(s) complete.")
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
            "--build-on",
            "Cannot use --build-on, architecture list is already set in snapcraft.yaml.",
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
