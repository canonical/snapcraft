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

"""This flutter plugin is useful for building flutter based parts.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - flutter-revision
      (string)
      Which Flutter revision to use for the build. This must be a valid
      revision from the flutter repository.
    - flutter-target
      (string, default: lib/main.dart)
      The main entry-point file of the application

This plugin works best with the flutter related extensions.
"""

import logging
from pathlib import Path
import subprocess
from typing import Any, Dict, List

from snapcraft import file_utils
from snapcraft.plugins.v1 import PluginV1

logger = logging.getLogger(__name__)


class FlutterPlugin(PluginV1):
    @classmethod
    def schema(cls) -> Dict[str, Any]:
        schema = super().schema()
        schema["properties"]["flutter-target"] = {
            "type": "string",
            "default": "lib/main.dart",
        }
        schema["properties"]["flutter-revision"] = {"type": "string", "default": None}
        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls) -> List[str]:
        return ["flutter-revision"]

    @classmethod
    def get_build_properties(cls) -> List[str]:
        return ["flutter-target"]

    def __init__(self, name, options, project) -> None:
        super().__init__(name, options, project)

        logger.warning(
            "The flutter plugin is currently in beta, its API may break. Use at your "
            "own risk."
        )

    def pull(self) -> None:
        super().pull()

        work_path = Path(self.sourcedir)
        if self.options.source_subdir:
            work_path /= self.options.source_subdir

        # Let these errors go through to get them on Sentry.
        if self.options.flutter_revision:
            subprocess.run(
                f"yes | flutter version {self.options.flutter_revision}",
                shell=True,
                check=True,
                cwd=work_path,
            )

    def build(self) -> None:
        super().build()

        self.run(["flutter", "pub", "get"])

        self.run(
            [
                "flutter",
                "build",
                "linux",
                "--release",
                "-v",
                "-t",
                self.options.flutter_target,
            ]
        )

        # Flutter only supports arm64 and amd64
        if Path(self.builddir, "build/linux/x64/release/bundle").exists():
            bundle_dir_path = Path(self.builddir, "build/linux/x64/release/bundle")
        elif Path(self.builddir, "build/linux/arm64/release/bundle").exists():
            bundle_dir_path = Path(self.builddir, "build/linux/arm64/release/bundle")
        else:
            bundle_dir_path = Path(self.builddir, "build/linux/release/bundle")

        install_bin_dir_path = Path(self.installdir) / "bin"
        install_bin_dir_path.mkdir(exist_ok=True)
        # Now move everything over to the plugin's installdir
        file_utils.link_or_copy_tree(
            bundle_dir_path.as_posix(), install_bin_dir_path.as_posix()
        )
