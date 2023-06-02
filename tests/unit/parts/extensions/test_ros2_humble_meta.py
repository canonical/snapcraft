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
from snapcraft.extensions.ros2_humble_desktop import ROS2HumbleDesktopExtension
from snapcraft.extensions.ros2_humble_ros_base import ROS2HumbleRosBaseExtension
from snapcraft.extensions.ros2_humble_ros_core import ROS2HumbleRosCoreExtension


def setup_method_fixture(extension, yaml_data=None, arch=None, target_arch=None):
    if yaml_data is None:
        yaml_data = {}
    if arch is None:
        arch = "amd64"
    if target_arch is None:
        target_arch = "amd64"

    return extension(yaml_data=yaml_data, arch=arch, target_arch=target_arch)


class TestExtensionROS2HumbleMetaExtensions:
    """ROS 2 Humble meta extensions tests."""

    fixture_variables = "extension_name,extension_class,meta,meta_dev"
    fixture_values = [
        (
            "ros2-humble-desktop",
            ROS2HumbleDesktopExtension,
            "ros-humble-desktop",
            "ros-humble-desktop-dev",
        ),
        (
            "ros2-humble-ros-base",
            ROS2HumbleRosBaseExtension,
            "ros-humble-ros-base",
            "ros-humble-ros-base-dev",
        ),
        (
            "ros2-humble-ros-core",
            ROS2HumbleRosCoreExtension,
            "ros-humble-ros-core",
            "ros-humble-ros-core-dev",
        ),
    ]

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_is_registered(self, extension_name, extension_class, meta, meta_dev):
        assert extension_name in reg.get_extension_names()

        try:
            reg.get_extension_class(extension_name)
        except errors.ExtensionError as exc:
            raise AssertionError(f"Couldn't get extension '{extension_name}': {exc}")

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_experimental(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
        assert extension.is_experimental(None)

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_ros_version(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
        assert extension.ROS_VERSION == "2"

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_supported_bases(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
        assert extension.get_supported_bases() == ("core22",)

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_supported_confinement(
        self, extension_name, extension_class, meta, meta_dev
    ):
        extension = setup_method_fixture(extension_class)
        assert extension.get_supported_confinement() == ("strict", "devmode")

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_root_snippet(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
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
            "plugs": {
                "ros-humble": {
                    "content": "ros-humble",
                    "default-provider": meta,
                    "interface": "content",
                    "target": "$SNAP/opt/ros/underlay_ws",
                }
            },
        }

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_app_snippet(self, extension_name, extension_class, meta, meta_dev):
        python_paths = [
            "$SNAP/opt/ros/humble/lib/python3.10/site-packages",
            "$SNAP/usr/lib/python3/dist-packages",
            "${PYTHONPATH}",
            "$SNAP/opt/ros/underlay_ws/opt/ros/humble/lib/python3.10/site-packages",
            "$SNAP/opt/ros/underlay_ws/usr/lib/python3/dist-packages",
        ]
        extension = setup_method_fixture(extension_class)
        assert extension.get_app_snippet() == {
            "command-chain": ["snap/command-chain/ros2-launch"],
            "environment": {
                "ROS_VERSION": "2",
                "ROS_DISTRO": "humble",
                "PYTHONPATH": ":".join(python_paths),
                "ROS_HOME": "$SNAP_USER_DATA/ros",
                "LD_LIBRARY_PATH": "$SNAP/opt/ros/underlay_ws/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:$LD_LIBRARY_PATH",
                "PATH": "$SNAP/opt/ros/underlay_ws/usr/bin:$PATH",
            },
        }

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_part_snippet(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
        assert extension.get_part_snippet() == {
            "build-environment": [{"ROS_VERSION": "2"}, {"ROS_DISTRO": "humble"}],
            "ros-build-snaps": [meta_dev],
            "colcon-cmake-args": [
                f'-DCMAKE_SYSTEM_PREFIX_PATH="/snap/{meta_dev}/current/usr"'
            ],
        }

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_parts_snippet(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
        assert extension.get_parts_snippet() == {
            f"{extension_name}/ros2-launch": {
                "source": f"{get_extensions_data_dir()}/ros2",
                "plugin": "make",
                "build-packages": [
                    "ros-humble-ros-environment",
                    "ros-humble-ros-workspace",
                    "ros-humble-ament-index-cpp",
                    "ros-humble-ament-index-python",
                    "libpython3.10-dev",
                ],
                "stage-packages": [
                    "ros-humble-ros-environment",
                    "ros-humble-ros-workspace",
                    "ros-humble-ament-index-cpp",
                    "ros-humble-ament-index-python",
                ],
            }
        }
