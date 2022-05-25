# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2018-2022 Canonical Ltd.
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

from typing import TYPE_CHECKING, Dict, List, Type

from snapcraft import errors

from .gnome import GNOME

if TYPE_CHECKING:
    from .extension import Extension

    ExtensionType = Type[Extension]


_EXTENSIONS: Dict[str, "ExtensionType"] = {"gnome": GNOME}


def get_extension_names() -> List[str]:
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
