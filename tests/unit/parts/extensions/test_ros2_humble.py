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

import pytest

import snapcraft.extensions.registry as reg
from snapcraft import errors
from snapcraft.extensions.extension import get_extensions_data_dir
from snapcraft.extensions.ros2_humble import ROS2HumbleExtension

_EXTENSION_NAME = "ros2-humble"


@pytest.fixture
def setup_method_fixture():
    def _setup_method_fixture(yaml_data=None, arch=None, target_arch=None):
        if yaml_data is None:
            yaml_data = {}
        if arch is None:
            arch = "amd64"
        if target_arch is None:
            target_arch = "amd64"

        return ROS2HumbleExtension(
            yaml_data=yaml_data, arch=arch, target_arch=target_arch
        )

    yield _setup_method_fixture


class TestExtensionROS2HumbleExtension:
    """ROS 2 Humble extension tests."""

    def test_is_registered(self):
        assert _EXTENSION_NAME in reg.get_extension_names()

        try:
            reg.get_extension_class(_EXTENSION_NAME)
        except errors.ExtensionError as exc:
            raise AssertionError(f"Couldn't get extension '{_EXTENSION_NAME}': {exc}")

    def test_ros_version(self, setup_method_fixture):
        extension = setup_method_fixture()
        assert extension.ROS_VERSION == "2"

    def test_get_supported_bases(self, setup_method_fixture):
        extension = setup_method_fixture()
        assert extension.get_supported_bases() == ("core22",)

    def test_get_supported_confinement(self, setup_method_fixture):
        extension = setup_method_fixture()
        assert extension.get_supported_confinement() == ("strict", "devmode")

    def test_get_root_snippet(self, setup_method_fixture):
        extension = setup_method_fixture()
        assert extension.get_root_snippet() == {
            "package-repositories": [
                {
                    "type": "apt",
                    "url": "http://repo.ros2.org/ubuntu/main",
                    "components": ["main"],
                    "formats": ["deb"],
                    "key-id": "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                    "key-server": "keyserver.ubuntu.com",
                    "suites": ["jammy"],
                }
            ],
            "lint": {
                "ignore": [
                    {
                        "unused-library": [
                            "opt/ros/*",
                            "lib/*/libcrypt.so*",
                            "lib/*/libexpat.so*",
                            "lib/*/libtirpc.so*",
                            "lib/*/libz.so*",
                            "usr/lib/*libatomic.so*",
                            "usr/lib/*libconsole_bridge.so*",
                            "usr/lib/*libfmt.so*",
                            "usr/lib/*libicui18n.so*",
                            "usr/lib/*libicuio.so*",
                            "usr/lib/*libicutest.so*",
                            "usr/lib/*libicutu.so*",
                            "usr/lib/*libpython3.10.so*",
                            "usr/lib/*libspdlog.so*",
                            "usr/lib/*libtinyxml2.so*",
                        ]
                    }
                ]
            },
        }

    def test_get_app_snippet(self, setup_method_fixture):
        python_paths = [
            "$SNAP/opt/ros/humble/lib/python3.10/site-packages",
            "$SNAP/usr/lib/python3/dist-packages",
            "${PYTHONPATH}",
        ]
        extension = setup_method_fixture()
        assert extension.get_app_snippet() == {
            "command-chain": ["snap/command-chain/ros2-launch"],
            "environment": {
                "ROS_VERSION": "2",
                "ROS_DISTRO": "humble",
                "PYTHONPATH": ":".join(python_paths),
                "ROS_HOME": "$SNAP_USER_DATA/ros",
            },
        }

    def test_get_part_snippet(self, setup_method_fixture):
        extension = setup_method_fixture()
        assert extension.get_part_snippet() == {
            "build-environment": [{"ROS_VERSION": "2"}, {"ROS_DISTRO": "humble"}]
        }

    def test_get_parts_snippet(self, setup_method_fixture):
        extension = setup_method_fixture()
        assert extension.get_parts_snippet() == {
            "ros2-humble/ros2-launch": {
                "source": f"{get_extensions_data_dir()}/ros2",
                "plugin": "nil",
                "override-build": "install -D -m 0755 launch ${CRAFT_PART_INSTALL}/snap/command-chain/ros2-launch",
                "build-packages": [
                    "ros-humble-ros-environment",
                    "ros-humble-ros-workspace",
                    "ros-humble-ament-index-cpp",
                    "ros-humble-ament-index-python",
                ],
            }
        }
