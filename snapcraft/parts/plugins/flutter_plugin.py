# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023-2024 Canonical Ltd.
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

"""The flutter plugin."""

from typing import Literal, cast

from craft_parts import infos, plugins
from overrides import overrides

FLUTTER_REPO = "https://github.com/flutter/flutter.git"
"""The repository where the flutter SDK resides."""


class FlutterPluginProperties(plugins.PluginProperties, frozen=True):
    """The part properties used by the flutter plugin."""

    plugin: Literal["flutter"] = "flutter"

    source: str  # type: ignore[reportGeneralTypeIssues]
    flutter_channel: Literal["stable", "master", "beta"] = "stable"
    flutter_target: str = "lib/main.dart"


class FlutterPlugin(plugins.Plugin):
    """A plugin for flutter projects.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:
        - flutter-channel
          (enum: [stable, master, beta], default: stable)
          The default flutter channel to use for the build.
        - flutter-target
          (str, default: lib/main.dart)
          The flutter target to build.
    """

    properties_class = FlutterPluginProperties

    def __init__(
        self, *, properties: plugins.PluginProperties, part_info: infos.PartInfo
    ) -> None:
        super().__init__(properties=properties, part_info=part_info)

        self.flutter_dir = part_info.part_build_dir / "flutter-distro"

    @overrides
    def get_build_snaps(self) -> set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> set[str]:
        return {
            "clang",
            "curl",
            "git",
            "cmake",
            "ninja-build",
            "unzip",
        }

    @overrides
    def get_build_environment(self) -> dict[str, str]:
        return {
            "PATH": f"{self.flutter_dir / 'bin'}:${{PATH}}",
        }

    def _get_setup_flutter(self, options: FlutterPluginProperties) -> list[str]:
        # TODO move to pull
        return [
            # TODO detect changes to plugin properties
            f"git clone --depth 1 -b {options.flutter_channel} {FLUTTER_REPO} {self.flutter_dir}",
            # Workaround for flutter#163308
            # Deletion of this file avoids an if statement that causes the following commands to
            # fail
            f"rm {self.flutter_dir}/engine/src/.gn",
            "flutter precache --linux",
            "flutter pub get",
        ]

    @overrides
    def get_build_commands(self) -> list[str]:
        options = cast(FlutterPluginProperties, self._options)

        flutter_install_cmd: list[str] = []

        if not self.flutter_dir.exists():
            flutter_install_cmd = self._get_setup_flutter(options)

        flutter_build_cmd = [
            f"flutter build linux --release --verbose --target {options.flutter_target}",
            "cp -r build/linux/*/release/bundle/* $CRAFT_PART_INSTALL/",
        ]
        return flutter_install_cmd + flutter_build_cmd
