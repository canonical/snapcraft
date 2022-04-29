# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Create snap.yaml metadata file."""

from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Union, cast

import yaml
from pydantic_yaml import YamlModel

from snapcraft.projects import Project
from snapcraft.utils import get_ld_library_paths, process_version


class Socket(YamlModel):
    """snap.yaml app socket entry."""

    listen_stream: Union[int, str]
    socket_mode: Optional[int]

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


class SnapApp(YamlModel):
    """Snap.yaml app entry.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.

    TODO: implement desktop (CRAFT-804)
    TODO: implement extensions (CRAFT-805)
    TODO: implement passthrough (CRAFT-854)
    """

    command: str
    autostart: Optional[str]
    common_id: Optional[str]
    bus_name: Optional[str]
    completer: Optional[str]
    stop_command: Optional[str]
    post_stop_command: Optional[str]
    start_timeout: Optional[str]
    stop_timeout: Optional[str]
    watchdog_timeout: Optional[str]
    reload_command: Optional[str]
    restart_delay: Optional[str]
    timer: Optional[str]
    daemon: Optional[str]
    after: Optional[List[str]]
    before: Optional[List[str]]
    refresh_mode: Optional[str]
    stop_mode: Optional[str]
    restart_condition: Optional[str]
    install_mode: Optional[str]
    plugs: Optional[List[str]]
    aliases: Optional[List[str]]
    environment: Optional[Dict[str, Any]]
    command_chain: Optional[List[str]]
    sockets: Optional[Dict[str, Socket]]

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


class SnapMetadata(YamlModel):
    """The snap.yaml model.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.

    TODO: implement adopt-info (CRAFT-803)
    """

    name: str
    title: Optional[str]
    version: str
    summary: str
    description: str
    license: Optional[str]
    type: Optional[str]
    architectures: List[str]
    base: Optional[str]
    build_base: Optional[str]
    assumes: Optional[List[str]]
    epoch: Optional[str]
    apps: Optional[Dict[str, SnapApp]]
    confinement: str
    grade: str
    environment: Optional[Dict[str, Any]]
    plugs: Optional[Dict[str, Any]]
    slots: Optional[Dict[str, Any]]
    hooks: Optional[Dict[str, Any]]
    layout: Optional[Dict[str, Dict[str, str]]]
    system_usernames: Optional[Dict[str, Any]]

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


def write(project: Project, prime_dir: Path, *, target_arch: str, arch_triplet: str):
    """Create a snap.yaml file.

    :param project: Snapcraft project.
    :param prime_dir: The directory containing the content to be snapped.
    :param target_arch: Target architecture the snap project is built to.
    :param arch_triplet: Architecture triplet of the platform.
    """
    meta_dir = prime_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    assumes: Set[str] = set()

    snap_apps: Dict[str, SnapApp] = {}
    if project.apps:
        for name, app in project.apps.items():

            app_sockets: Dict[str, Socket] = {}
            if app.sockets:
                for socket_name, socket in app.sockets.items():
                    app_sockets[socket_name] = Socket(
                        listen_stream=socket.listen_stream,
                        socket_mode=socket.socket_mode,
                    )

            if app.command_chain:
                assumes.add("command-chain")

            snap_apps[name] = SnapApp(
                command=app.command,
                autostart=app.autostart,
                common_id=app.common_id,
                bus_name=app.bus_name,
                completer=app.completer,
                stop_command=app.stop_command,
                post_stop_command=app.post_stop_command,
                start_timeout=app.start_timeout,
                stop_timeout=app.stop_timeout,
                watchdog_timeout=app.watchdog_timeout,
                reload_command=app.reload_command,
                restart_delay=app.restart_delay,
                timer=app.timer,
                daemon=app.daemon,
                after=app.after if app.after else None,
                before=app.before if app.before else None,
                refresh_mode=app.refresh_mode,
                stop_mode=app.stop_mode,
                restart_condition=app.restart_condition,
                install_mode=app.install_mode,
                plugs=app.plugs,
                aliases=app.aliases,
                environment=app.environment,
                command_chain=app.command_chain if app.command_chain else None,
                sockets=app_sockets if app_sockets else None,
            )

    if project.hooks and any(h for h in project.hooks.values() if h.command_chain):
        assumes.add("command-chain")

    environment = _populate_environment(project.environment, prime_dir, arch_triplet)
    version = process_version(project.version)

    snap_metadata = SnapMetadata(
        name=project.name,
        title=project.title,
        version=version,
        summary=project.summary,
        description=project.description,  # type: ignore
        license=project.license,
        type=project.type,
        architectures=[target_arch],
        base=cast(str, project.base),
        assumes=list(assumes) if assumes else None,
        epoch=project.epoch,
        apps=snap_apps or None,
        confinement=project.confinement,
        grade=project.grade or "stable",
        environment=environment,
        plugs=project.plugs,
        slots=project.slots,
        hooks=project.hooks,
        layout=project.layout,
        system_usernames=project.system_usernames,
    )

    yaml.add_representer(str, _repr_str, Dumper=yaml.SafeDumper)
    yaml_data = snap_metadata.yaml(
        by_alias=True,
        exclude_none=True,
        allow_unicode=True,
        sort_keys=False,
        width=1000,
    )

    snap_yaml = meta_dir / "snap.yaml"
    snap_yaml.write_text(yaml_data)


def _repr_str(dumper, data):
    """Multi-line string representer for the YAML dumper."""
    if "\n" in data:
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)


def _populate_environment(
    environment: Optional[Dict[str, Optional[str]]], prime_dir: Path, arch_triplet: str
):
    """Populate default app environmental variables.

    Three cases for LD_LIBRARY_PATH and PATH variables:
        - If LD_LIBRARY_PATH or PATH are defined, keep user-defined values.
        - If LD_LIBRARY_PATH or PATH are not defined, set to default values.
        - If LD_LIBRARY_PATH or PATH are null, do not use default values.
    """
    if environment is None:
        return {
            "LD_LIBRARY_PATH": get_ld_library_paths(prime_dir, arch_triplet),
            "PATH": "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH",
        }

    try:
        if not environment["LD_LIBRARY_PATH"]:
            environment.pop("LD_LIBRARY_PATH")
    except KeyError:
        environment["LD_LIBRARY_PATH"] = get_ld_library_paths(prime_dir, arch_triplet)

    try:
        if not environment["PATH"]:
            environment.pop("PATH")
    except KeyError:
        environment["PATH"] = "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH"

    if len(environment):
        return environment

    # if the environment only contained a null LD_LIBRARY_PATH and a null PATH, return None
    return None
