# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Extension registry."""

from typing import TYPE_CHECKING

from snapcraft import errors

from .dotnet8 import Dotnet8Extension
from .dotnet9 import Dotnet9Extension
from .dotnet10 import Dotnet10Extension
from .env_injector import EnvInjector
from .gnome import GNOME
from .kde_neon import KDENeon
from .kde_neon_6 import KDENeon6
from .kde_neon_qt6 import KDENeonQt6
from .ros2_humble import ROS2HumbleExtension
from .ros2_humble_desktop import ROS2HumbleDesktopExtension
from .ros2_humble_ros_base import ROS2HumbleRosBaseExtension
from .ros2_humble_ros_core import ROS2HumbleRosCoreExtension
from .ros2_jazzy import ROS2JazzyExtension
from .ros2_jazzy_desktop import ROS2JazzyDesktopExtension
from .ros2_jazzy_ros_base import ROS2JazzyRosBaseExtension
from .ros2_jazzy_ros_core import ROS2JazzyRosCoreExtension
from .ros2_lyrical import ROS2LyricalExtension
from .ros2_lyrical_desktop import ROS2LyricalDesktopExtension
from .ros2_lyrical_ros_base import ROS2LyricalRosBaseExtension
from .ros2_lyrical_ros_core import ROS2LyricalRosCoreExtension

if TYPE_CHECKING:
    from .extension import Extension

    ExtensionType = type[Extension]

_EXTENSIONS: dict[str, "ExtensionType"] = {
    "dotnet8": Dotnet8Extension,
    "dotnet9": Dotnet9Extension,
    "dotnet10": Dotnet10Extension,
    "env-injector": EnvInjector,
    "gnome": GNOME,
    "ros2-humble": ROS2HumbleExtension,
    "ros2-humble-ros-core": ROS2HumbleRosCoreExtension,
    "ros2-humble-ros-base": ROS2HumbleRosBaseExtension,
    "ros2-humble-desktop": ROS2HumbleDesktopExtension,
    "ros2-jazzy": ROS2JazzyExtension,
    "ros2-jazzy-ros-core": ROS2JazzyRosCoreExtension,
    "ros2-jazzy-ros-base": ROS2JazzyRosBaseExtension,
    "ros2-jazzy-desktop": ROS2JazzyDesktopExtension,
    "ros2-lyrical": ROS2LyricalExtension,
    "ros2-lyrical-ros-core": ROS2LyricalRosCoreExtension,
    "ros2-lyrical-ros-base": ROS2LyricalRosBaseExtension,
    "ros2-lyrical-desktop": ROS2LyricalDesktopExtension,
    "kde-neon": KDENeon,
    "kde-neon-6": KDENeon6,
    "kde-neon-qt6": KDENeonQt6,
}


def get_extension_names() -> list[str]:
    """Obtain a extension class given the name.

    :param name: The extension name.
    :return: The list of available extensions.
    :raises ExtensionError: If the extension name is invalid.
    """
    return list(_EXTENSIONS.keys())


def get_extension_class(extension_name: str) -> "ExtensionType":
    """Obtain a extension class given the name.

    :param name: The extension name.
    :return: The extension class.
    :raises ExtensionError: If the extension name is invalid.
    """
    try:
        return _EXTENSIONS[extension_name]
    except KeyError as key_error:
        raise errors.ExtensionError(
            f"Extension {extension_name!r} does not exist"
        ) from key_error


def register(extension_name: str, extension_class: "ExtensionType") -> None:
    """Register extension.

    :param extension_name: the name to register.
    :param extension_class: the Extension implementation.
    """
    _EXTENSIONS[extension_name] = extension_class


def unregister(extension_name: str) -> None:
    """Unregister extension_name.

    :raises KeyError: if extension_name is not registered.
    """
    del _EXTENSIONS[extension_name]
