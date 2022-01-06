# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

from snapcraft_legacy.internal.project_loader._extensions.ros1_noetic import (
    ExtensionImpl as Ros1NoeticExtension,
)


@pytest.fixture(params=[Ros1NoeticExtension])
def extension_class(request):
    return request.param


def test_extension(extension_class):
    ros1_extension = extension_class(
        extension_name="ros1-noetic", yaml_data=dict(base="core20")
    )

    assert ros1_extension.root_snippet == {
        "package-repositories": [
            {
                "components": ["main"],
                "formats": ["deb"],
                "key-id": "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                "key-server": "keyserver.ubuntu.com",
                "suites": ["focal"],
                "type": "apt",
                "url": "http://packages.ros.org/ros/ubuntu",
            }
        ]
    }

    assert ros1_extension.app_snippet == {
        "command-chain": ["snap/command-chain/ros1-launch"],
        "environment": {
            "PYTHONPATH": "$SNAP/opt/ros/noetic/lib/python3.8/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}",
            "ROS_VERSION": "1",
            "ROS_DISTRO": "noetic",
        },
    }

    assert ros1_extension.part_snippet == {
        "build-environment": [{"ROS_VERSION": "1"}, {"ROS_DISTRO": "noetic"}]
    }

    assert ros1_extension.parts == {
        "ros1-noetic-extension": {
            "build-packages": ["ros-noetic-catkin"],
            "override-build": "install -D -m 0755 launch "
            "${SNAPCRAFT_PART_INSTALL}/snap/command-chain/ros1-launch",
            "plugin": "nil",
            "source": "$SNAPCRAFT_EXTENSIONS_DIR/ros1",
        }
    }


def test_supported_bases(extension_class):
    assert extension_class.get_supported_bases() == ("core20",)


def test_supported_confinement(extension_class):
    extension_class.get_supported_confinement() == ("strict", "devmode")
