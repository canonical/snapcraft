# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
from snapcraft import formatting_utils, project
from snapcraft.internal import common, elf, pluginhandler

from typing import Dict, List


def env_for_classic(base: str, arch_triplet: str) -> List[str]:
    """Set the required environment variables for a classic confined build."""
    env = []

    core_path = common.get_core_path(base)
    paths = common.get_library_paths(core_path, arch_triplet, existing_only=False)
    env.append(
        formatting_utils.format_path_variable(
            "LD_LIBRARY_PATH", paths, prepend="", separator=":"
        )
    )

    return env


def runtime_env(root: str, arch_triplet: str) -> List[str]:
    """Set the environment variables required for running binaries."""
    env = []

    env.append(
        'PATH="'
        + ":".join(
            ["{0}/usr/sbin", "{0}/usr/bin", "{0}/sbin", "{0}/bin", "$PATH"]
        ).format(root)
        + '"'
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
                    envvar, paths, prepend="-I", separator=" "
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


def snapcraft_global_environment(project: project.Project) -> Dict[str, str]:
    if project.info.name:
        name = project.info.name
    else:
        name = ""

    if project.info.version:
        version = project.info.version
    else:
        version = ""

    if project.info.grade:
        grade = project.info.grade
    else:
        grade = ""

    return {
        "SNAPCRAFT_ARCH_TRIPLET": project.arch_triplet,
        "SNAPCRAFT_PARALLEL_BUILD_COUNT": project.parallel_build_count,
        "SNAPCRAFT_PROJECT_NAME": name,
        "SNAPCRAFT_PROJECT_VERSION": version,
        "SNAPCRAFT_PROJECT_GRADE": grade,
        "SNAPCRAFT_STAGE": project.stage_dir,
        "SNAPCRAFT_PRIME": project.prime_dir,
        "SNAPCRAFT_EXTENSIONS_DIR": common.get_extensionsdir(),
    }


def snapcraft_part_environment(part: pluginhandler.PluginHandler) -> Dict[str, str]:
    return {
        "SNAPCRAFT_PART_SRC": part.plugin.sourcedir,
        "SNAPCRAFT_PART_BUILD": part.plugin.builddir,
        "SNAPCRAFT_PART_INSTALL": part.plugin.installdir,
    }


def environment_to_replacements(environment: Dict[str, str]) -> Dict[str, str]:
    replacements = {}  # type: Dict[str, str]
    for variable, value in environment.items():
        # Support both $VAR and ${VAR} syntax
        replacements["${}".format(variable)] = value
        replacements["${{{}}}".format(variable)] = value

    return replacements
