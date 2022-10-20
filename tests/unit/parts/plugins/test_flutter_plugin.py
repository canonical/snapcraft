# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import pytest
from craft_parts import Part, PartInfo, ProjectInfo

from snapcraft.parts.plugins import FlutterPlugin


@pytest.fixture(autouse=True)
def part_info(new_dir):
    yield PartInfo(
        project_info=ProjectInfo(
            application_name="test", project_name="test-snap", cache_dir=new_dir
        ),
        part=Part("my-part", {}),
    )


def test_get_build_snaps(part_info):
    properties = FlutterPlugin.properties_class.unmarshal({"source": "."})
    plugin = FlutterPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_snaps() == set()


def test_get_build_packages(part_info):
    properties = FlutterPlugin.properties_class.unmarshal({"source": "."})
    plugin = FlutterPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_packages() == {
        "clang",
        "git",
        "cmake",
        "ninja-build",
        "unzip",
    }


def test_get_build_environment(part_info):
    properties = FlutterPlugin.properties_class.unmarshal({"source": "."})
    plugin = FlutterPlugin(properties=properties, part_info=part_info)

    flutter_bin = plugin._part_info.part_build_dir / "flutter-distro" / "bin"
    assert plugin.get_build_environment() == {"PATH": f"{flutter_bin}:${{PATH}}"}


def test_get_build_commands(part_info):
    properties = FlutterPlugin.properties_class.unmarshal({"source": "."})
    plugin = FlutterPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        "git clone --depth 1 -b stable https://github.com/flutter/flutter.git "
        f"{plugin.flutter_dir}",
        "flutter precache --linux",
        "flutter pub get",
        "flutter build linux --release --verbose --target lib/main.dart",
        "mkdir -p $CRAFT_PART_INSTALL/bin/",
        "cp -r build/linux/*/release/bundle/* $CRAFT_PART_INSTALL/",
    ]


def test_get_build_commands_alternative_target(part_info):
    properties = FlutterPlugin.properties_class.unmarshal(
        {"flutter-target": "lib/not-main.dart", "source": "."}
    )
    plugin = FlutterPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        f"git clone --depth 1 -b stable https://github.com/flutter/flutter.git "
        f"{plugin.flutter_dir}",
        "flutter precache --linux",
        "flutter pub get",
        "flutter build linux --release --verbose --target lib/not-main.dart",
        "mkdir -p $CRAFT_PART_INSTALL/bin/",
        "cp -r build/linux/*/release/bundle/* $CRAFT_PART_INSTALL/",
    ]


@pytest.mark.parametrize("value", ["stable", "master", "beta"])
def test_get_build_commands_different_channels(part_info, value):
    properties = FlutterPlugin.properties_class.unmarshal(
        {"flutter-channel": value, "source": "."}
    )
    plugin = FlutterPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        f"git clone --depth 1 -b {value} https://github.com/flutter/flutter.git "
        f"{plugin.flutter_dir}",
        "flutter precache --linux",
        "flutter pub get",
        "flutter build linux --release --verbose --target lib/main.dart",
        "mkdir -p $CRAFT_PART_INSTALL/bin/",
        "cp -r build/linux/*/release/bundle/* $CRAFT_PART_INSTALL/",
    ]


def test_get_build_commands_flutter_bin_exists(part_info):
    properties = FlutterPlugin.properties_class.unmarshal({"source": "."})
    plugin = FlutterPlugin(properties=properties, part_info=part_info)
    plugin.flutter_dir.mkdir(parents=True)

    assert plugin.get_build_commands() == [
        "flutter build linux --release --verbose --target lib/main.dart",
        "mkdir -p $CRAFT_PART_INSTALL/bin/",
        "cp -r build/linux/*/release/bundle/* $CRAFT_PART_INSTALL/",
    ]
