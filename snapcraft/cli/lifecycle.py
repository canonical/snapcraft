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

import subprocess
import logging
import typing
import sys
import os
from time import sleep

import click

from . import echo
from . import env
from ._options import add_build_options, get_project
from snapcraft.internal import (
    errors,
    build_providers,
    deprecations,
    lifecycle,
    project_loader,
    steps,
    repo,
)
from snapcraft.project._sanity_checks import (
    conduct_project_sanity_check,
    conduct_build_environment_sanity_check,
)
from snapcraft.project.errors import MultipassMissingInstallableError
from ._errors import TRACEBACK_MANAGED, TRACEBACK_HOST


logger = logging.getLogger(__name__)


if typing.TYPE_CHECKING:
    from snapcraft.internal.project import Project  # noqa: F401


def _install_multipass():
    if sys.platform == "linux":
        repo.snaps.install_snaps(["multipass/latest/beta"])
    elif sys.platform == "darwin":
        try:
            subprocess.check_call(["brew", "cask", "install", "multipass"])
        except subprocess.CalledProcessError:
            raise errors.SnapcraftEnvironmentError(
                "Failed to install multipass using homebrew.\n"
                "Verify your homebrew installation and try again.\n"
                "Alternatively, manually install multipass by running 'brew cask install multipass'."
            )

    # wait for multipassd to be available
    click.echo("Waiting for multipass...")
    retry_count = 20
    while retry_count:
        try:
            output = subprocess.check_output(["multipass", "version"]).decode()
        except subprocess.CalledProcessError:
            output = ""
        # if multipassd is in the version information, it means the service is up
        # and we can carry on
        if "multipassd" in output:
            break
        retry_count -= 1
        sleep(1)
    # No need to worry about getting to this point by exhausting our retry count,
    # the rest of the stack will handle the error appropriately.


# TODO: when snap is a real step we can simplify the arguments here.
def _execute(  # noqa: C901
    step: steps.Step,
    parts: str,
    pack_project: bool = False,
    output: str = None,
    shell: bool = False,
    shell_after: bool = False,
    destructive_mode: bool = False,
    **kwargs
) -> "Project":
    _clean_provider_error()
    provider = "host" if destructive_mode else None
    build_environment = env.BuilderEnvironmentConfig(force_provider=provider)
    try:
        conduct_build_environment_sanity_check(build_environment.provider)
    except MultipassMissingInstallableError as e:
        click.echo(
            "You need multipass installed to build snaps "
            "(https://github.com/CanonicalLtd/multipass)."
        )
        if click.confirm("Would you like to install it now?"):
            _install_multipass()
        else:
            raise errors.SnapcraftEnvironmentError(
                "multipass is required to continue."
            ) from e

    project = get_project(is_managed_host=build_environment.is_managed_host, **kwargs)

    echo.wrapped(
        "Using {!r}: Project assets will be "
        "searched for from the {!r} directory.".format(
            project.info.snapcraft_yaml_file_path,
            os.path.relpath(project._get_snapcraft_assets_dir(), project._project_dir),
        )
    )

    conduct_project_sanity_check(project)

    if build_environment.is_managed_host or build_environment.is_host:
        project_config = project_loader.load_config(project)
        lifecycle.execute(step, project_config, parts)
        if pack_project:
            _pack(project.prime_dir, output=output)
    else:
        build_provider_class = build_providers.get_provider_for(
            build_environment.provider
        )
        echo.info("Launching a VM.")
        with build_provider_class(project=project, echoer=echo) as instance:
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


def _pack(directory: str, *, output: str) -> None:
    snap_name = lifecycle.pack(directory, output)
    echo.info("Snapped {}".format(snap_name))


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
@add_build_options()
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


@lifecyclecli.command()
@click.pass_context
@add_build_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def pull(ctx, parts, **kwargs):
    """Download or retrieve artifacts defined for a part.

    \b
    Examples:
        snapcraft pull
        snapcraft pull my-part1 my-part2

    """
    _execute(steps.PULL, parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def build(parts, **kwargs):
    """Build artifacts defined for a part.

    \b
    Examples:
        snapcraft build
        snapcraft build my-part1 my-part2

    """
    _execute(steps.BUILD, parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def stage(parts, **kwargs):
    """Stage the part's built artifacts into the common staging area.

    \b
    Examples:
        snapcraft stage
        snapcraft stage my-part1 my-part2

    """
    _execute(steps.STAGE, parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def prime(parts, **kwargs):
    """Final copy and preparation for the snap.

    \b
    Examples:
        snapcraft prime
        snapcraft prime my-part1 my-part2

    """
    _execute(steps.PRIME, parts, **kwargs)


@lifecyclecli.command()
@add_build_options()
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
        _execute(steps.PRIME, parts=[], pack_project=True, output=output, **kwargs)


@lifecyclecli.command()
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


@lifecyclecli.command()
@click.argument("parts", nargs=-1, metavar="<part>...", required=False)
def clean(parts):
    """Remove a part's assets.

    \b
    Examples:
        snapcraft clean
        snapcraft clean my-part
    """
    build_environment = env.BuilderEnvironmentConfig()
    project = get_project(is_managed_host=build_environment.is_managed_host)

    if build_environment.is_managed_host or build_environment.is_host:
        lifecycle.clean(project, parts)
    else:
        build_provider_class = build_providers.get_provider_for(
            build_environment.provider
        )
        build_provider = build_provider_class(project=project, echoer=echo)
        if parts:
            echo.info("Launching a VM.")
            with build_provider_class(project=project, echoer=echo) as instance:
                instance.clean(part_names=parts)
        else:
            build_provider.clean_project()


if __name__ == "__main__":
    lifecyclecli.main()
