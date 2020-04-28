# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from typing import Dict, TYPE_CHECKING

from snapcraft import formatting_utils
from snapcraft.internal import common

if TYPE_CHECKING:
    from snapcraft.project import Project
    from . import PluginHandler


def get_snapcraft_global_environment(project: "Project") -> Dict[str, str]:
    # The triple check is mostly for unit tests support until we completely
    # get rid of project.info.
    if project._snap_meta.name:
        name = project._snap_meta.name
    elif project.info is not None and project.info.name:
        name = project.info.name
    else:
        name = ""

    if project._snap_meta.version:
        version = project._snap_meta.version
    elif project.info is not None and project.info.version:
        version = project.info.version
    else:
        version = ""

    if project._snap_meta.grade:
        grade = project._snap_meta.grade
    elif project.info is not None and project.info.grade:
        grade = project.info.grade
    else:
        grade = ""

    return {
        "SNAPCRAFT_ARCH_TRIPLET": project.arch_triplet,
        "SNAPCRAFT_BUILD_BASE": project._get_build_base(),
        "SNAPCRAFT_PARALLEL_BUILD_COUNT": str(project.parallel_build_count),
        "SNAPCRAFT_PROJECT_NAME": name,
        "SNAPCRAFT_PROJECT_VERSION": version,
        "SNAPCRAFT_PROJECT_DIR": project._project_dir,
        "SNAPCRAFT_PROJECT_GRADE": grade,
        "SNAPCRAFT_STAGE": project.stage_dir,
        "SNAPCRAFT_PRIME": project.prime_dir,
        "SNAPCRAFT_EXTENSIONS_DIR": common.get_extensionsdir(),
    }


def get_snapcraft_part_environment(part: "PluginHandler") -> Dict[str, str]:
    return {
        "SNAPCRAFT_PART_SRC": part.part_source_dir,
        "SNAPCRAFT_PART_BUILD": part.part_build_dir,
        "SNAPCRAFT_PART_INSTALL": part.part_install_dir,
    }


def get_snapcraft_build_environment(part: "PluginHandler") -> Dict[str, str]:
    """Return Snapcraft provided build environment."""
    part_environment = get_snapcraft_global_environment(part._project)
    part_environment.update(get_snapcraft_part_environment(part))

    paths = [part.part_install_dir, part._project.stage_dir]

    bin_paths = list()
    # Add "/" to paths mostly to support snapcraft development in venv.
    for path in paths + ["/"]:
        bin_paths.extend(common.get_bin_paths(root=path, existing_only=True))

    if bin_paths:
        bin_paths.append("$PATH")
        part_environment["PATH"] = formatting_utils.combine_paths(
            paths=bin_paths, prepend="", separator=":"
        )

    include_paths = list()
    for path in paths:
        include_paths.extend(common.get_include_paths(path, part._project.arch_triplet))

    if include_paths:
        for envvar in ["CPPFLAGS", "CFLAGS", "CXXFLAGS"]:
            part_environment[envvar] = formatting_utils.combine_paths(
                paths=include_paths, prepend="-isystem", separator=" "
            )

    library_paths = list()
    for path in paths:
        library_paths.extend(common.get_library_paths(path, part._project.arch_triplet))

    if library_paths:
        part_environment["LDFLAGS"] = formatting_utils.combine_paths(
            paths=library_paths, prepend="-L", separator=" "
        )

    pkg_config_paths = common.get_pkg_config_paths(path, part._project.arch_triplet)
    if pkg_config_paths:
        part_environment["PKG_CONFIG_PATH"] = formatting_utils.combine_paths(
            pkg_config_paths, prepend="", separator=":"
        )

    return part_environment
