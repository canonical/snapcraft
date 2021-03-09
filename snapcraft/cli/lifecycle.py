# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2019 Canonical Ltd
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

import itertools
import logging
import os
import pathlib
import subprocess
import time
import typing
from typing import List, Optional, Union

import click
import progressbar

from snapcraft import file_utils
from snapcraft.internal import (
    deprecations,
    errors,
    indicators,
    lifecycle,
    project_loader,
    steps,
)
from snapcraft.project._sanity_checks import conduct_project_sanity_check

from . import echo, providers
from ._command import SnapcraftProjectCommand
from ._errors import TRACEBACK_HOST, TRACEBACK_MANAGED
from ._options import add_provider_options, get_project

logger = logging.getLogger(__name__)


if typing.TYPE_CHECKING:
    from snapcraft.internal.project import Project  # noqa: F401


# TODO: when snap is a real step we can simplify the arguments here.
def _execute(  # noqa: C901
    ctx,
    step: steps.Step,
    parts: str,
    pack_project: bool = False,
    output: Optional[str] = None,
    setup_prime_try: bool = False,
    target_arch: Optional[str] = None,
    **kwargs,
) -> "Project":
    provider_options = ctx.obj["provider_options"]
    provider_options.update_from_click_command_args(target_arch=target_arch, **kwargs)
    provider_options.apply_host_environment()

    # Cleanup any previous errors.
    _clean_provider_error()

    # Temporary fix to ignore target_arch.
    if target_arch is not None and provider_options.provider in ["multipass", "lxd"]:
        echo.warning(
            "Ignoring '--target-arch' flag.  This flag requires --destructive-mode and is unsupported with Multipass and LXD build providers."
        )
        target_arch = None

    is_managed_host = provider_options.provider == "managed-host"
    project = get_project(
        is_managed_host=is_managed_host, target_arch=target_arch, **kwargs
    )
    conduct_project_sanity_check(project, target_arch=target_arch, **kwargs)

    project_dir = pathlib.Path(project._project_dir)
    if project_dir.name in ["build-aux", "snap"]:
        echo.warning(
            f"Snapcraft is running in directory {project_dir.name!r}.  If this is the snap assets directory, please run snapcraft from {project_dir.parent}."
        )

    if provider_options.provider in ["host", "managed-host"]:
        project_config = project_loader.load_config(project)
        lifecycle.execute(step, project_config, parts)
        if pack_project:
            _pack(
                project.prime_dir,
                compression=project._snap_meta.compression,
                output=output,
            )
    else:
        manager = providers.setup_instance_manager(
            base=project._get_build_base(),
            project_dir=project_dir,
            project_name=project.info.name,
            provider_options=provider_options,
        )
        try:
            if provider_options.shell:
                # shell means we want to do everything right up to the previous
                # step and then go into a shell instead of the requested step.
                # the "snap" target is a special snowflake that has not made its
                # way to be a proper step.
                previous_step = None
                if pack_project:
                    previous_step = steps.PRIME
                elif step > steps.PULL:
                    previous_step = step.previous_step()
                # steps.PULL is the first step, so we would directly shell into it.
                if previous_step:
                    manager.execute_step(step=previous_step.name)
            elif pack_project:
                manager.pack_project(output=output)
            elif setup_prime_try:
                manager.mount_prime(prime_dir=project._prime_dir)
                manager.execute_step(step=step.name)
            else:
                manager.execute_step(step=step.name)
        except Exception:
            _retrieve_provider_error(manager.instance)
            if project.debug:
                manager.shell()
            else:
                echo.warning(
                    "Run the same command again with --debug to shell into the environment "
                    "if you wish to introspect this failure."
                )
                raise
        else:
            if provider_options.shell or provider_options.shell_after:
                manager.shell()
    return project


def _run_pack(snap_command: List[Union[str, pathlib.Path]]) -> str:
    ret = None
    stdout = ""
    stderr = ""
    with subprocess.Popen(
        snap_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    ) as proc:
        if indicators.is_dumb_terminal():
            echo.info("Snapping...")
            ret = proc.wait()
        else:
            message = f"\033[0;32mSnapping \033[0m"
            progress_indicator = progressbar.ProgressBar(
                widgets=[message, progressbar.AnimatedMarker()],
                # From progressbar.ProgressBar.update(...).
                maxval=progressbar.UnknownLength,
            )
            progress_indicator.start()
            for counter in itertools.count():
                progress_indicator.update(counter)
                time.sleep(0.2)
                ret = proc.poll()
                if ret is not None:
                    break
            progress_indicator.finish()

        if proc.stdout is not None:
            stdout = proc.stdout.read().decode()
        if proc.stderr is not None:
            stderr = proc.stderr.read().decode()
        logger.debug(f"stdout: {stdout} | stderr: {stderr}")

    if ret != 0:
        raise RuntimeError(
            f"Failed to create snap, snap command failed:\nstdout:\n{stdout}\nstderr:\n{stderr}"
        )

    try:
        snap_filename = stdout.split(":")[1].strip()
    except IndexError:
        logger.debug("Failed to parse snap pack outpout: {stdout}")
        snap_filename = stdout

    return snap_filename


def _pack(
    directory: str, *, compression: Optional[str] = None, output: Optional[str]
) -> None:
    """Pack a snap.

    :param directory: directory to snap
    :param compression: compression type to use, None for defaults
    :param output: Output may either be:
        (1) a directory path to output snaps to
        (2) an explicit file path to output snap to
        (3) unpsecified/None to output to current (project) directory
    """
    output_file = None
    output_dir = None

    if output:
        output_path = pathlib.Path(output)
        output_parent = output_path.parent
        if output_path.is_dir():
            output_dir = str(output_path)
        elif output_parent and output_parent != pathlib.Path("."):
            output_dir = str(output_parent)
            output_file = output_path.name
        else:
            output_file = output

    snap_path = file_utils.get_host_tool_path(command_name="snap", package_name="snapd")

    command: List[Union[str, pathlib.Path]] = [snap_path, "pack"]
    # When None, just use snap pack's default settings.
    if compression is not None:
        command.extend(["--compression", compression])

    if output_file is not None:
        command.extend(["--filename", output_file])

    command.append(directory)

    if output_dir is not None:
        command.append(output_dir)

    logger.debug(f"Running pack command: {command}")
    snap_filename = _run_pack(command)
    echo.info(f"Snapped {snap_filename}")


def _clean_provider_error() -> None:
    if os.path.isfile(TRACEBACK_HOST):
        try:
            os.remove(TRACEBACK_HOST)
        except Exception as e:
            logger.debug("can't remove error file: {}", str(e))


def _retrieve_provider_error(instance) -> None:
    try:
        instance.pull_file(TRACEBACK_MANAGED, TRACEBACK_HOST)
    except Exception as e:
        logger.debug("can't retrieve error file: {}", str(e))

    instance.execute_run(["rm", "-f", TRACEBACK_MANAGED])


@click.group()
def lifecyclecli():
    """Never gets invoked with current structure. :-("""


@lifecyclecli.command()
def init():
    """Initialize a snapcraft project."""
    snapcraft_yaml_path = lifecycle.init()
    echo.info("Created {}.".format(snapcraft_yaml_path))
    echo.wrapped(
        "Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more "
        "information about the snapcraft.yaml format."
    )


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@click.pass_context
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def pull(ctx, parts, **kwargs):
    """Download or retrieve artifacts defined for a part.

    \b
    Examples:
        snapcraft pull
        snapcraft pull my-part1 my-part2

    """
    _execute(
        ctx, steps.PULL, parts, **kwargs,
    )


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@click.pass_context
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def build(ctx, parts, **kwargs):
    """Build artifacts defined for a part.

    \b
    Examples:
        snapcraft build
        snapcraft build my-part1 my-part2

    """
    _execute(
        ctx, steps.BUILD, parts, **kwargs,
    )


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@click.pass_context
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def stage(ctx, parts, **kwargs):
    """Stage the part's built artifacts into the common staging area.

    \b
    Examples:
        snapcraft stage
        snapcraft stage my-part1 my-part2

    """
    _execute(ctx, steps.STAGE, parts, **kwargs)


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@click.pass_context
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def prime(ctx, parts, **kwargs):
    """Final copy and preparation for the snap.

    \b
    Examples:
        snapcraft prime
        snapcraft prime my-part1 my-part2

    """
    _execute(ctx, steps.PRIME, parts, **kwargs)


@lifecyclecli.command("try")
@click.pass_context
@add_provider_options()
def try_command(ctx, **kwargs):
    """Try a snap on the host, priming if necessary.

    This feature only works on snap enabled systems.

    \b
    Examples:
        snapcraft try

    """
    project = _execute(ctx, steps.PRIME, [], setup_prime_try=True, **kwargs)

    # project.prime_dir here points to the on-host prime directory.
    echo.info("You can now run `snap try {}`.".format(project.prime_dir))


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@click.pass_context
@add_provider_options()
@click.argument("directory", required=False)
@click.option("--output", "-o", help="path to the resulting snap.")
def snap(ctx, directory, output, **kwargs):
    """Create a snap.

    \b
    Examples:
        snapcraft snap
        snapcraft snap --output renamed-snap.snap

    If you want to snap a directory, you should use the pack command
    instead.
    """
    if directory:
        deprecations.handle_deprecation_notice("dn6")
        _pack(directory, output=output)
    else:
        _execute(
            ctx, steps.PRIME, parts=tuple(), pack_project=True, output=output, **kwargs,
        )


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@click.argument("directory")
@click.option("--output", "-o", help="path to the resulting snap.")
def pack(directory, output, **kwargs):
    """Create a snap from a directory holding a valid snap.

    The layout of <directory> should contain a valid meta/snap.yaml in
    order to be a valid snap.

    \b
    Examples:
        snapcraft pack my-snap-directory
        snapcraft pack my-snap-directory --output renamed-snap.snap

    """
    _pack(directory, output=output)


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@click.pass_context
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
@click.option("--unprime", is_flag=True, required=False, hidden=True)
@click.option("--step", "-s", required=False, hidden=True)
def clean(ctx, parts, unprime, step, target_arch, **kwargs):
    """Remove a part's assets.

    \b
    Examples:
        snapcraft clean
        snapcraft clean my-part
    """
    provider_options = ctx.obj["provider_options"]

    # Temporary fix to ignore target_arch for clean.
    if target_arch is not None and provider_options.provider in ["multipass", "lxd"]:
        target_arch = None

    provider_options.update_from_click_command_args(target_arch=target_arch, **kwargs)
    provider_options.apply_host_environment()

    is_managed_host = provider_options.provider == "managed-host"

    # This option is only valid in legacy.
    if step:
        option = "--step" if "--step" in ctx.obj["argv"] else "-s"
        raise click.BadOptionUsage(option, "no such option: {}".format(option))

    if unprime and not is_managed_host:
        raise click.BadOptionUsage("--unprime", "no such option: --unprime")

    try:
        project = get_project(is_managed_host=is_managed_host)
    except errors.ProjectNotFoundError:
        # Fresh environment, nothing to clean.
        return

    if provider_options.provider in ["host", "managed-host"]:
        step = steps.PRIME if unprime else None
        lifecycle.clean(project, parts, step)
    else:
        manager = providers.setup_existing_instance_manager(
            project_name=project.info.name, provider_options=provider_options
        )

        if parts:
            manager.clean_parts(parts=parts)
        else:
            manager.clean()


if __name__ == "__main__":
    lifecyclecli.main()
