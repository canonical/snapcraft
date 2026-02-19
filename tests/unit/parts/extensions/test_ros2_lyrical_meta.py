# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2026 Canonical Ltd
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

from snapcraft import errors
from snapcraft.extensions import registry
from snapcraft.extensions.extension import get_extensions_data_dir
from snapcraft.extensions.ros2_lyrical_desktop import ROS2LyricalDesktopExtension
from snapcraft.extensions.ros2_lyrical_ros_base import ROS2LyricalRosBaseExtension
from snapcraft.extensions.ros2_lyrical_ros_core import ROS2LyricalRosCoreExtension


def setup_method_fixture(extension, yaml_data=None, arch=None, target_arch=None):
    return extension(yaml_data=yaml_data, arch=arch, target_arch=target_arch)


class TestExtensionROS2LyricalMetaExtensions:
    """ROS 2 Lyrical meta extensions tests."""

    fixture_variables = "extension_name,extension_class,meta,meta_dev"
    fixture_values = [
        (
            "ros2-lyrical-desktop",
            ROS2LyricalDesktopExtension,
            "ros-lyrical-desktop",
            "ros-lyrical-desktop-dev",
        ),
        (
            "ros2-lyrical-ros-base",
            ROS2LyricalRosBaseExtension,
            "ros-lyrical-ros-base",
            "ros-lyrical-ros-base-dev",
        ),
        (
            "ros2-lyrical-ros-core",
            ROS2LyricalRosCoreExtension,
            "ros-lyrical-ros-core",
            "ros-lyrical-ros-core-dev",
        ),
    ]

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_is_registered(self, extension_name, extension_class, meta, meta_dev):
        assert extension_name in registry.get_extension_names()

        try:
            registry.get_extension_class(extension_name)
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
        assert extension.get_supported_bases() == ("core26",)

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
                    "url": "http://packages.ros.org/ros2/ubuntu",
                    "components": ["main"],
                    "formats": ["deb"],
                    "key-id": "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                    "key-server": "keyserver.ubuntu.com",
                    "suites": ["resolute"],
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
                            "usr/lib/*libpython3.14.so*",
                            "usr/lib/*libspdlog.so*",
                            "usr/lib/*libtinyxml2.so*",
                        ]
                    }
                ]
            },
            "plugs": {
                meta: {
                    "content": meta,
                    "default-provider": meta,
                    "interface": "content",
                    "target": "$SNAP/opt/ros/underlay_ws",
                }
            },
        }

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_app_snippet(self, extension_name, extension_class, meta, meta_dev):
        python_paths = [
            "$SNAP/opt/ros/lyrical/lib/python3.14/site-packages",
            "$SNAP/usr/lib/python3/dist-packages",
            "${PYTHONPATH}",
            "$SNAP/opt/ros/underlay_ws/opt/ros/lyrical/lib/python3.14/site-packages",
            "$SNAP/opt/ros/underlay_ws/usr/lib/python3/dist-packages",
        ]
        extension = setup_method_fixture(extension_class)
        assert extension.get_app_snippet(app_name="test-app") == {
            "command-chain": ["snap/command-chain/ros2-launch"],
            "environment": {
                "ROS_VERSION": "2",
                "ROS_DISTRO": "lyrical",
                "PYTHONPATH": ":".join(python_paths),
                "ROS_HOME": "$SNAP_USER_DATA/ros",
            },
        }

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_part_snippet(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
        assert extension.get_part_snippet(plugin_name="colcon") == {
            "build-environment": [{"ROS_VERSION": "2"}, {"ROS_DISTRO": "lyrical"}],
            "colcon-ros-build-snaps": [meta_dev],
            "colcon-cmake-args": [
                f'-DCMAKE_SYSTEM_PREFIX_PATH="/snap/{meta_dev}/current/usr"'
            ],
        }

        assert extension.get_part_snippet(plugin_name="cmake") == {
            "build-environment": [{"ROS_VERSION": "2"}, {"ROS_DISTRO": "lyrical"}],
        }

    @pytest.mark.parametrize(fixture_variables, fixture_values)
    def test_get_parts_snippet(self, extension_name, extension_class, meta, meta_dev):
        extension = setup_method_fixture(extension_class)
        assert extension.get_parts_snippet() == {
            f"{extension_name}/ros2-launch": {
                "source": f"{get_extensions_data_dir()}/ros2",
                "plugin": "make",
                "build-packages": [
                    "ros-lyrical-ros-environment",
                    "ros-lyrical-ros-workspace",
                    "ros-lyrical-ament-index-cpp",
                    "ros-lyrical-ament-index-python",
                    "libpython3.14-dev",
                ],
                "stage-packages": [
                    "ros-lyrical-ros-environment",
                    "ros-lyrical-ros-workspace",
                    "ros-lyrical-ament-index-cpp",
                    "ros-lyrical-ament-index-python",
                ],
            }
        }
