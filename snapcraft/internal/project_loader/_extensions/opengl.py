# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2020 Canonical Ltd
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

# Import types and tell flake8 to ignore the "unused" List.

import logging
from typing import Any, Dict, Tuple

from ._extension import Extension


logger = logging.getLogger(__name__)


class ExtensionImpl(Extension):
    """This extension eases creation of snaps that integrate with OpenGL.

    For easier desktop integration, it also configures each application
    entry with these additional plugs if strictly confined (or devmode):

    \b
    - opengl (https://snapcraft.io/docs/opengl-interface)
    """

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core18",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode", "classic")

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        logger.warning(
            "*EXPERIMENTAL*: The OpenGL extension is experimental and likely to change."
        )

        if yaml_data.get("confinement") == "classic":
            # Use environment variables for classic snaps.
            self.root_snippet = {
                "environment": {
                    "__EGL_VENDOR_LIBRARY_DIRS": "$SNAP/etc/glvnd/egl_vendor.d:$SNAP/usr/share/glvnd/egl_vendor.d",
                    "LIBGL_DRIVERS_PATH": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/dri",
                }
            }

            # Mesa must use no-patchelf, otherwise driver will crash.
            # TODO: develop/upstream patch to fix mesa.
            self.parts = {
                "mesa": {
                    "plugin": "nil",
                    "build-attributes": ["no-patchelf"],
                    "prime": [
                        "-lib/udev",
                        "-usr/doc",
                        "-usr/doc-base",
                        "-usr/share/applications",
                        "-usr/share/apport",
                        "-usr/share/bug",
                        "-usr/share/doc",
                        "-usr/share/doc-base",
                        "-usr/share/icons",
                        "-usr/share/libdrm",
                        "-usr/share/libwacom",
                        "-usr/share/lintian",
                        "-usr/share/man",
                        "-usr/share/pkgconfig",
                    ],
                    "stage-packages": ["libgl1-mesa-dri", "libglx-mesa0"],
                }
            }
        else:
            # Use plugs and layouts for strict/devmode.

            self.app_snippet = {"plugs": ["opengl"]}

            self.root_snippet = {
                "layout": {
                    "/etc/glvnd": {"bind": "$SNAP/etc/glvnd"},
                    "/usr/lib/${SNAPCRAFT_ARCH_TRIPLET}/dri": {
                        "bind": "$SNAP/usr/lib/${SNAPCRAFT_ARCH_TRIPLET}/dri"
                    },
                    "/usr/share/glvnd": {"bind": "$SNAP/etc/glvnd"},
                }
            }
