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

    - flutter-channel
      (string)
      Which Flutter channel to use for the build
    - flutter-revision
      (string)
      Which Flutter revision to use for the build. This must be a valid
      revision from the flutter repository.
    - flutter-target
      (string, default: lib/main.dart)
      The main entry-point file of the application
"""

import logging
import pathlib
import subprocess
from typing import Any, Dict, List

from snapcraft import file_utils
from snapcraft.internal import errors
from snapcraft.plugins.v1 import PluginV1

logger = logging.getLogger(__name__)


class FlutterPlugin(PluginV1):
    @classmethod
    def schema(cls) -> Dict[str, Any]:
        schema = super().schema()
        schema["properties"]["flutter-channel"] = {
            "type": "string",
            "enum": ["dev", "master"],
        }
        schema["properties"]["flutter-target"] = {
            "type": "string",
            "default": "lib/main.dart",
        }
        schema["properties"]["flutter-revision"] = {
            "type": "string",
            "default": None,
        }
        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls) -> List[str]:
        return ["flutter-channel", "flutter-revision"]

    @classmethod
    def get_build_properties(cls) -> List[str]:
        return ["flutter-target"]

    def __init__(self, name, options, project) -> None:
        super().__init__(name, options, project)

        if project._get_build_base() not in ("core", "core18"):
            raise errors.PluginBaseError(
                part_name=self.name, base=project._get_build_base()
            )

        self.build_snaps.extend(["flutter/latest/stable"])

        logger.warning(
            "The flutter plugin is currently in beta, its API may break. Use at your "
            "own risk."
        )

    def pull(self) -> None:
        super().pull()

        # Let these errors go through to get them on Sentry.
        subprocess.run(["flutter", "channel", self.options.flutter_channel], check=True)
        subprocess.run(["flutter", "config", "--enable-linux-desktop"], check=True)
        subprocess.run(["flutter", "upgrade"], check=True)
        subprocess.run(["flutter", "doctor"], check=True)
        if self.options.flutter_revision:
            subprocess.run(
                f"yes | flutter version {self.options.flutter_revision}",
                shell=True,
                check=True,
            )
        subprocess.run(["flutter", "pub", "get"], check=True)

    def build(self) -> None:
        super().build()

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

        bundle_dir_path = pathlib.Path(self.builddir) / "build/linux/release/bundle"
        install_bin_dir_path = pathlib.Path(self.installdir) / "bin"
        install_bin_dir_path.mkdir(exist_ok=True)
        # Now move everything over to the plugin's installdir
        file_utils.link_or_copy_tree(
            bundle_dir_path.as_posix(), install_bin_dir_path.as_posix()
        )
