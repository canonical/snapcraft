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

import glob
import os
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Union, cast

import yaml
from pydantic_yaml import YamlModel

from snapcraft.projects import Project

_ARCH_TO_TRIPLET: Dict[str, str] = {
    "arm64": "aarch64-linux-gnu",
    "armhf": "arm-linux-gnueabihf",
    "i386": "i386-linux-gnu",
    "powerpc": "powerpc-linux-gnu",
    "ppc64el": "powerpc64le-linux-gnu",
    "riscv64": "riscv64-linux-gnu",
    "s390x": "s390x-linux-gnu",
    "amd64": "x86_64-linux-gnu",
}


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


def write(project: Project, prime_dir: Path, *, arch: str):
    """Create a snap.yaml file."""
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

    snap_metadata = SnapMetadata(
        name=project.name,
        title=project.title,
        version=project.version,
        summary=project.summary,
        description=project.description,  # type: ignore
        license=project.license,
        type=project.type,
        architectures=[arch],
        base=cast(str, project.base),
        assumes=list(assumes) if assumes else None,
        epoch=project.epoch,
        apps=snap_apps or None,
        confinement=project.confinement,
        grade=project.grade or "stable",
        environment=_populate_environment(project.environment, prime_dir, arch),
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


def _populate_environment(environment, prime_dir, arch):
    """Populate default app environmental variables.

    Three cases for LD_LIBRARY_PATH and PATH variables:
        - If LD_LIBRARY_PATH or PATH are defined, keep user-defined values.
        - If LD_LIBRARY_PATH or PATH are not defined, set to default values.
        - If LD_LIBRARY_PATH or PATH are null, do not use default values.
    """
    if environment is None:
        return {
            "LD_LIBRARY_PATH": _get_ld_library_path(prime_dir, arch),
            "PATH": "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH",
        }

    try:
        if not environment["LD_LIBRARY_PATH"]:
            environment.pop("LD_LIBRARY_PATH")
    except KeyError:
        environment["LD_LIBRARY_PATH"] = _get_ld_library_path(prime_dir, arch)

    try:
        if not environment["PATH"]:
            environment.pop("PATH")
    except KeyError:
        environment["PATH"] = "$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH"

    if len(environment):
        return environment

    # if the environment only contained a null LD_LIBRARY_PATH and a null PATH, return None
    return None


def _get_ld_library_path(prime_dir, arch) -> str:
    """Get LD_LIBRARY_PATH variable."""
    paths = ["${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"]
    # Add the default LD_LIBRARY_PATH
    paths += _get_common_ld_library_paths(prime_dir, arch)
    # Add more specific LD_LIBRARY_PATH from staged packages if necessary
    paths += _get_configured_ld_library_paths(prime_dir)

    ld_library_path = ":".join(paths)

    return re.sub(str(prime_dir), "$SNAP", ld_library_path)


def _get_common_ld_library_paths(prime_dir, arch) -> List[str]:
    """Return common library paths for a snap.

    If existing_only is set the paths returned must exist for
    the root that was set.
    """
    triplet = _ARCH_TO_TRIPLET[arch]
    paths = [
        os.path.join(prime_dir, "lib"),
        os.path.join(prime_dir, "usr", "lib"),
        os.path.join(prime_dir, "lib", triplet),
        os.path.join(prime_dir, "usr", "lib", triplet),
    ]

    return [p for p in paths if os.path.exists(p)]


def _get_configured_ld_library_paths(prime_dir: str) -> List[str]:
    """Determine additional library paths needed for the linker loader.

    This is a workaround until full library searching is implemented which
    works by searching for ld.so.conf in specific hard coded locations
    within root.

    :param prime_dir str: the directory to search for specific ld.so.conf
                          entries.
    :returns: a list of strings of library paths where relevant libraries
              can be found within prime_dir.
    """
    # If more ld.so.conf files need to be supported, add them here.
    ld_config_globs = {f"{prime_dir}/usr/lib/*/mesa*/ld.so.conf"}

    ld_library_paths = []
    for this_glob in ld_config_globs:
        for ld_conf_file in glob.glob(this_glob):
            ld_library_paths.extend(_extract_ld_library_paths(ld_conf_file))

    return [prime_dir + path for path in ld_library_paths]


def _extract_ld_library_paths(ld_conf_file: str) -> List[str]:
    # From the ldconfig manpage, paths can be colon-, space-, tab-, newline-,
    # or comma-separated.
    path_delimiters = re.compile(r"[:\s,]")
    comments = re.compile(r"#.*$")

    paths = []
    with open(ld_conf_file, "r", encoding="utf-8") as ld_config:
        for line in ld_config:
            # Remove comments from line
            line = comments.sub("", line).strip()

            if line:
                paths.extend(path_delimiters.split(line))

    return paths
