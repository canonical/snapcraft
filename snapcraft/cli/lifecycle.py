# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
import sys
import typing

import click

from . import echo
from . import env
from ._options import add_build_options, get_project
from snapcraft.internal import (
    errors,
    build_providers,
    deprecations,
    lifecycle,
    lxd,
    project_loader,
    steps,
)
from snapcraft.project.errors import YamlValidationError

if typing.TYPE_CHECKING:
    from snapcraft.internal.project import Project  # noqa: F401


# TODO: when snap is a real step we can simplify the arguments here.
# fmt: off
def _execute(  # noqa: C901
    step: steps.Step,
    parts: str,
    pack_project: bool = False,
    output: str = None,
    shell: bool = False,
    shell_after: bool = False,
    **kwargs
) -> "Project":
    # fmt: on
    build_environment = env.BuilderEnvironmentConfig()
    project = get_project(is_managed_host=build_environment.is_managed_host, **kwargs)

    if project.info.base is not None and not (
        build_environment.is_host or build_environment.is_managed_host
    ):
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
                if project.debug:
                    instance.shell()
                else:
                    echo.warning("Run the same command again with --debug to shell into the environment "
                                 "if you wish to introspect this failure.")
                    raise
            else:
                if shell or shell_after:
                    instance.shell()
    elif build_environment.is_managed_host or build_environment.is_host:
        project_config = project_loader.load_config(project)
        lifecycle.execute(step, project_config, parts)
        if pack_project:
            _pack(project.prime_dir, output=output)
    else:
        # containerbuild takes a snapcraft command name, not a step
        lifecycle.containerbuild(command=step.name, project=project, args=parts)
        if pack_project:
            _pack(project.prime_dir, output=output)
    return project


def _pack(directory: str, *, output: str) -> None:
    snap_name = lifecycle.pack(directory, output)
    echo.info("Snapped {}".format(snap_name))


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
    echo.info("Edit the file to your liking or run `snapcraft` to get started")


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
@click.option(
    "--step",
    "-s",
    "step_name",
    type=click.Choice(["pull", "build", "stage", "prime", "strip"]),
    help="only clean the specified step and those that depend on it.",
)
def clean(parts, step_name):
    """Remove content - cleans downloads, builds or install artifacts.

    \b
    Examples:
        snapcraft clean
        snapcraft clean my-part --step build
    """
    build_environment = env.BuilderEnvironmentConfig()
    try:
        project = get_project(
            is_managed_host=build_environment.is_managed_host
        )
    except YamlValidationError:
        # We need to be able to clean invalid projects too.
        project = get_project(
            is_managed_host=build_environment.is_managed_host,
            skip_snapcraft_yaml=True
        )

    step = None
    if step_name:
        if step_name == "strip":
            echo.warning(
                "DEPRECATED: Use `prime` instead of `strip` as the step to clean"
            )
            step_name = "prime"
        step = steps.get_step_by_name(step_name)

    if build_environment.is_lxd:
        lxd.Project(project=project, output=None, source=os.path.curdir).clean(
            parts, step
        )
    elif build_environment.is_host:
        lifecycle.clean(project, parts, step)
    else:
        # TODO support for steps.
        if parts or step_name:
            raise errors.SnapcraftEnvironmentError(
                "Build providers are still not feature complete, specifying parts or a step name "
                "is not yet supported.")
        build_provider_class = build_providers.get_provider_for(
            build_environment.provider
        )
        build_provider_class(project=project, echoer=echo).clean_project()


@lifecyclecli.command()
@add_build_options()
@click.option(
    "--remote",
    metavar="<remote>",
    help="Use a specific lxd remote instead of a local container.",
)
def cleanbuild(remote, **kwargs):
    """Create a snap using a clean environment managed by a build provider.

    \b
    Examples:
        snapcraft cleanbuild

    The cleanbuild command requires a properly setup lxd environment that
    can connect to external networks. Refer to the "Ubuntu Desktop and
    Ubuntu Server" section on
    https://linuxcontainers.org/lxd/getting-started-cli
    to get started.

    If using a remote, a prior setup is required which is described on:
    https://linuxcontainers.org/lxd/getting-started-cli/#multiple-hosts
    """
    # cleanbuild is a special snow flake, while all the other commands
    # would work with the host as the build_provider it makes little
    # sense in this scenario.
    if sys.platform == "darwin":
        default_provider = "multipass"
    else:
        default_provider = "lxd"

    build_environment = env.BuilderEnvironmentConfig(
        default=default_provider, additional_providers=["multipass"]
    )
    project = get_project(
        is_managed=build_environment.is_managed_host, **kwargs
    )

    snap_filename = lifecycle.cleanbuild(
        project=project, echoer=echo, remote=remote, build_environment=build_environment
    )
    echo.info("Retrieved {!r}".format(snap_filename))


if __name__ == "__main__":
    lifecyclecli.main()
