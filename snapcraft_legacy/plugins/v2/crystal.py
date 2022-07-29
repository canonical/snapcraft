# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Manas.Tech
# License granted by Canonical Limited
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

"""The Crystal plugin can be used for Crystal projects using `shards`.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - crystal-channel:
      (string, default: latest/stable)
      The Snap Store channel to install Crystal from.

    - crystal-build-options
      (list of strings, default: '[]')
      These options are passed to `shards build`.
"""
import os
import shlex
import shutil
import sys
from typing import Any, Dict, List, Set

import click

from snapcraft_legacy import file_utils
from snapcraft_legacy.internal import common, elf, errors

from snapcraft_legacy.plugins.v2 import PluginV2

_CRYSTAL_CHANNEL = "latest/stable"


class CrystalPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "crystal-channel": {"type": "string", "default": _CRYSTAL_CHANNEL},
                "crystal-build-options": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
            "required": ["source"],
        }

    def get_build_snaps(self) -> Set[str]:
        return {f"crystal/{self.options.crystal_channel}"}

    def get_build_packages(self) -> Set[str]:
        # See https://github.com/crystal-lang/distribution-scripts/blob/8bc01e26291dc518390129e15df8f757d687871c/docker/ubuntu.Dockerfile#L9
        return {
            "git",
            "make",
            "gcc",
            "pkg-config",
            "libssl-dev",
            "libxml2-dev",
            "libyaml-dev",
            "libgmp-dev",
            "libpcre3-dev",
            "libevent-dev",
            "libz-dev",
        }

    def get_build_environment(self) -> Dict[str, str]:
        return dict()

    def get_build_commands(self) -> List[str]:
        if self.options.crystal_build_options:
            build_options = " ".join(
                [shlex.quote(option) for option in self.options.crystal_build_options]
            )
        else:
            build_options = ""

        env = dict(LANG="C.UTF-8", LC_ALL="C.UTF-8")
        env_flags = [f"{key}={value}" for key, value in env.items()]

        return [
            f"shards build --without-development {build_options}",
            'cp -r ./bin "${SNAPCRAFT_PART_INSTALL}"/bin',
            " ".join(
                [
                    "env",
                    "-i",
                    *env_flags,
                    sys.executable,
                    "-I",
                    os.path.abspath(__file__),
                    "stage-runtime-dependencies",
                    "--part-src",
                    '"${SNAPCRAFT_PART_SRC}"',
                    "--part-install",
                    '"${SNAPCRAFT_PART_INSTALL}"',
                    "--part-build",
                    '"${SNAPCRAFT_PART_BUILD}"',
                    "--arch-triplet",
                    '"${SNAPCRAFT_ARCH_TRIPLET}"',
                    "--content-dirs",
                    '"${SNAPCRAFT_CONTENT_DIRS}"',
                ]
            ),
        ]


@click.group()
def plugin_cli():
    pass


@plugin_cli.command()
@click.option("--part-src", envvar="SNAPCRAFT_PART_SRC", required=True)
@click.option("--part-install", envvar="SNAPCRAFT_PART_INSTALL", required=True)
@click.option("--part-build", envvar="SNAPCRAFT_PART_BUILD", required=True)
@click.option("--arch-triplet", envvar="SNAPCRAFT_ARCH_TRIPLET", required=True)
@click.option("--content-dirs", envvar="SNAPCRAFT_CONTENT_DIRS", required=True)
def stage_runtime_dependencies(
    part_src: str,
    part_install: str,
    part_build: str,
    arch_triplet: str,
    content_dirs: str,
):
    build_path = os.path.join(part_build, "bin")
    install_path = os.path.join(part_install, "bin")

    if not os.path.exists(build_path):
        raise errors.SnapcraftEnvironmentError(
            "No binaries were built. Ensure the shards.yaml contains valid targets."
        )

    bin_paths = (os.path.join(build_path, b) for b in os.listdir(build_path))
    elf_files = (elf.ElfFile(path=b) for b in bin_paths if elf.ElfFile.is_elf(b))
    os.makedirs(install_path, exist_ok=True)

    # convert colon-delimited paths into a set
    if content_dirs == "":
        content_dirs_set = set()
    else:
        content_dirs_set = set(content_dirs.split(":"))

    for elf_file in elf_files:
        shutil.copy2(
            elf_file.path,
            os.path.join(install_path, os.path.basename(elf_file.path)),
        )

        elf_dependencies_path = elf_file.load_dependencies(
            root_path=part_install,
            core_base_path=common.get_installed_snap_path("core20"),
            arch_triplet=arch_triplet,
            content_dirs=content_dirs_set,
        )

        for elf_dependency_path in elf_dependencies_path:
            lib_install_path = os.path.join(part_install, elf_dependency_path[1:])
            os.makedirs(os.path.dirname(lib_install_path), exist_ok=True)
            if not os.path.exists(lib_install_path):
                file_utils.link_or_copy(
                    elf_dependency_path, lib_install_path, follow_symlinks=True
                )


if __name__ == "__main__":
    plugin_cli()
