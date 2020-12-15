# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2020 Canonical Ltd
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

from typing import Dict, List

from snapcraft import formatting_utils
from snapcraft.internal import common, elf


def runtime_env(root: str, arch_triplet: str) -> List[str]:
    """Set the environment variables required for running binaries."""
    env = []

    env.append(
        'PATH="'
        + ":".join(["{0}/usr/sbin", "{0}/usr/bin", "{0}/sbin", "{0}/bin"]).format(root)
        + '${PATH:+:$PATH}"'
    )

    # Add the default LD_LIBRARY_PATH
    paths = common.get_library_paths(root, arch_triplet)
    # Add more specific LD_LIBRARY_PATH from staged packages if necessary
    paths += elf.determine_ld_library_path(root)

    if paths:
        env.append(
            formatting_utils.format_path_variable(
                "LD_LIBRARY_PATH", paths, prepend="", separator=":"
            )
        )

    return env


def build_env(root: str, snap_name: str, arch_triplet: str) -> List[str]:
    """Set the environment variables required for building.

    This is required for the current parts installdir due to stage-packages
    and also to setup the stagedir.
    """
    env = []

    paths = common.get_include_paths(root, arch_triplet)
    if paths:
        for envvar in ["CPPFLAGS", "CFLAGS", "CXXFLAGS"]:
            env.append(
                formatting_utils.format_path_variable(
                    envvar, paths, prepend="-isystem", separator=" "
                )
            )

    paths = common.get_library_paths(root, arch_triplet)
    if paths:
        env.append(
            formatting_utils.format_path_variable(
                "LDFLAGS", paths, prepend="-L", separator=" "
            )
        )

    paths = common.get_pkg_config_paths(root, arch_triplet)
    if paths:
        env.append(
            formatting_utils.format_path_variable(
                "PKG_CONFIG_PATH", paths, prepend="", separator=":"
            )
        )

    return env


def build_env_for_stage(stagedir: str, snap_name: str, arch_triplet: str) -> List[str]:
    env = build_env(stagedir, snap_name, arch_triplet)
    env.append('PERL5LIB="{0}/usr/share/perl5/"'.format(stagedir))

    return env


def environment_to_replacements(environment: Dict[str, str]) -> Dict[str, str]:
    replacements = {}  # type: Dict[str, str]
    for variable, value in environment.items():
        # Support both $VAR and ${VAR} syntax
        replacements["${}".format(variable)] = value
        replacements["${{{}}}".format(variable)] = value

    return replacements
