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
import typing
import os
import shutil
import subprocess
import sys
import time
from typing import List, Optional

import click
import progressbar

from . import echo
from ._command import SnapcraftProjectCommand
from ._options import (
    add_provider_options,
    apply_host_provider_flags,
    get_build_provider,
    get_build_provider_flags,
    get_project,
)
from snapcraft.internal import (
    build_providers,
    deprecations,
    errors,
    indicators,
    lifecycle,
    project_loader,
    steps,
)
from snapcraft.project._sanity_checks import conduct_project_sanity_check
from ._errors import TRACEBACK_MANAGED, TRACEBACK_HOST


logger = logging.getLogger(__name__)


if typing.TYPE_CHECKING:
    from snapcraft.internal.project import Project  # noqa: F401


# TODO: when snap is a real step we can simplify the arguments here.
def _execute(  # noqa: C901
    step: steps.Step,
    parts: str,
    pack_project: bool = False,
    output: Optional[str] = None,
    shell: bool = False,
    shell_after: bool = False,
    setup_prime_try: bool = False,
    **kwargs,
) -> "Project":
    # Cleanup any previous errors.
    _clean_provider_error()

    build_provider = get_build_provider(**kwargs)
    build_provider_flags = get_build_provider_flags(build_provider, **kwargs)
    apply_host_provider_flags(build_provider_flags)

    is_managed_host = build_provider == "managed-host"

    # Temporary fix to ignore target_arch.
    if kwargs.get("target_arch") is not None and build_provider in ["multipass", "lxd"]:
        echo.warning(
            "Ignoring '--target-arch' flag.  This flag requires --destructive-mode and is unsupported with Multipass and LXD build providers."
        )
        kwargs.pop("target_arch")

    project = get_project(is_managed_host=is_managed_host, **kwargs)
    conduct_project_sanity_check(project, **kwargs)

    if build_provider in ["host", "managed-host"]:
        project_config = project_loader.load_config(project)
        lifecycle.execute(step, project_config, parts)
        if pack_project:
            _pack(
                project.prime_dir,
                compression=project._snap_meta.compression,
                output=output,
            )
    else:
        build_provider_class = build_providers.get_provider_for(build_provider)
        try:
            build_provider_class.ensure_provider()
        except build_providers.errors.ProviderNotFound as provider_error:
            if provider_error.prompt_installable:
                if echo.is_tty_connected() and echo.confirm(
                    "Support for {!r} needs to be set up. "
                    "Would you like to do it now?".format(provider_error.provider)
                ):
                    build_provider_class.setup_provider(echoer=echo)
                else:
                    raise provider_error
            else:
                raise provider_error

        with build_provider_class(
            project=project, echoer=echo, build_provider_flags=build_provider_flags
        ) as instance:
            instance.mount_project()
            try:
                if shell:
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
                        instance.execute_step(previous_step)
                elif pack_project:
                    instance.pack_project(output=output)
                elif setup_prime_try:
                    instance.expose_prime()
                    instance.execute_step(step)
                else:
                    instance.execute_step(step)
            except Exception:
                _retrieve_provider_error(instance)
                if project.debug:
                    instance.shell()
                else:
                    echo.warning(
                        "Run the same command again with --debug to shell into the environment "
                        "if you wish to introspect this failure."
                    )
                    raise
            else:
                if shell or shell_after:
                    instance.shell()
    return project


def _run_pack(snap_command: List[str]) -> str:
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
    snap_path = shutil.which("snap")
    if snap_path is None:
        raise errors.HostToolNotFoundError(command_name="snap", package_name="snapd")

    command = [snap_path, "pack"]
    # When None, just use snap pack's default settings.
    if compression is not None:
        if compression != "xz":
            echo.warning(
                f"EXPERIMENTAL: Setting the squash FS compression to {compression!r}."
            )
        command.extend(["--compression", compression])
    if output is not None:
        command.extend(["--filename", output])
    command.append(directory)

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
        instance.pull_file(TRACEBACK_MANAGED, TRACEBACK_HOST, delete=True)
    except Exception as e:
        logger.debug("can't retrieve error file: {}", str(e))


@click.group()
@add_provider_options()
@click.pass_context
def lifecyclecli(ctx, **kwargs):
    pass


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
    _execute(steps.PULL, parts, **kwargs)


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def build(parts, **kwargs):
    """Build artifacts defined for a part.

    \b
    Examples:
        snapcraft build
        snapcraft build my-part1 my-part2

    """
    _execute(steps.BUILD, parts, **kwargs)


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def stage(parts, **kwargs):
    """Stage the part's built artifacts into the common staging area.

    \b
    Examples:
        snapcraft stage
        snapcraft stage my-part1 my-part2

    """
    _execute(steps.STAGE, parts, **kwargs)


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@add_provider_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def prime(parts, **kwargs):
    """Final copy and preparation for the snap.

    \b
    Examples:
        snapcraft prime
        snapcraft prime my-part1 my-part2

    """
    _execute(steps.PRIME, parts, **kwargs)


@lifecyclecli.command("try")
@add_provider_options()
def try_command(**kwargs):
    """Try a snap on the host, priming if necessary.

    This feature only works on snap enabled systems.

    \b
    Examples:
        snapcraft try

    """
    project = _execute(steps.PRIME, [], setup_prime_try=True, **kwargs)
    # project.prime_dir here points to the on-host prime directory.
    echo.info("You can now run `snap try {}`.".format(project.prime_dir))


@lifecyclecli.command(cls=SnapcraftProjectCommand)
@add_provider_options()
@click.argument("directory", required=False)
@click.option("--output", "-o", help="path to the resulting snap.")
def snap(directory, output, **kwargs):
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
        _execute(steps.PRIME, parts=tuple(), pack_project=True, output=output, **kwargs)


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
def clean(ctx, parts, unprime, step, **kwargs):
    """Remove a part's assets.

    \b
    Examples:
        snapcraft clean
        snapcraft clean my-part
    """
    # This option is only valid in legacy.
    if step:
        option = "--step" if "--step" in ctx.obj["argv"] else "-s"
        raise click.BadOptionUsage(option, "no such option: {}".format(option))

    build_provider = get_build_provider(**kwargs)
    build_provider_flags = get_build_provider_flags(build_provider, **kwargs)
    apply_host_provider_flags(build_provider_flags)

    is_managed_host = build_provider == "managed-host"

    # Temporary fix to ignore target_arch, silently for clean.
    if "target_arch" in kwargs and build_provider in ["multipass", "lxd"]:
        kwargs.pop("target_arch")

    try:
        project = get_project(is_managed_host=is_managed_host)
    except errors.ProjectNotFoundError:
        # Fresh environment, nothing to clean.
        return

    if unprime and not is_managed_host:
        raise click.BadOptionUsage("--unprime", "no such option: --unprime")

    if build_provider in ["host", "managed-host"]:
        step = steps.PRIME if unprime else None
        lifecycle.clean(project, parts, step)
    else:
        build_provider_class = build_providers.get_provider_for(build_provider)
        if parts:
            with build_provider_class(
                project=project, echoer=echo, build_provider_flags=build_provider_flags
            ) as instance:
                instance.clean(part_names=parts)
        else:
            build_provider_class(project=project, echoer=echo).clean_project()
            # Clear the prime directory on the host, unless on Windows.
            if sys.platform != "win32":
                lifecycle.clean(project, parts, steps.PRIME)


if __name__ == "__main__":
    lifecyclecli.main()
