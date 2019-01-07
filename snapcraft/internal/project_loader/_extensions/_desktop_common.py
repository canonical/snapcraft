# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from typing import Any, Dict

from ._extension import Extension


class DesktopCommonExtension(Extension):
    """The Common Desktop extension.
    This extension is to be subclassed by the gnome and kde extensions.
    It includes common code that each desktop extension would otherwise
    need to duplicate.
    """

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new DesktopCommonExtension.
        :param dict yaml_data: Loaded snapcraft.yaml data.
        """

        super().__init__(yaml_data)

        self.ld_library_paths = [
            "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/lapack",
            "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/blas",
            "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pulseaudio",
            "$LD_LIBRARY_PATH",
            "/var/lib/snapd/lib/gl",
            # Testability support
            "$SNAP/testability",
            "$SNAP/testability/$SNAPCRAFT_ARCH_TRIPLET",
            "$SNAP/testability/$SNAPCRAFT_ARCH_TRIPLET/mesa",
        ]

        self.path = ["$PATH"]

        self.plugs = {
            "gtk-3-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes:gtk-3-themes",
            },
            "icon-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
                "default-provider": "gtk-common-themes:icon-themes",
            },
            "sound-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
                "default-provider": "gtk-common-themes:sound-themes",
            },
        }

        self.xdg_cache_home = "$SNAP_USER_COMMON/.cache"
        self.xdg_config_home = "$SNAP_USER_DATA/.config"
        self.xdg_data_home = "$SNAP_USER_DATA/.local/share"
        self.xdg_config_dirs = ["$SNAP/etc/xdg", "$XDG_CONFIG_DIRS"]
        self.xdg_data_dirs = [
            # Workaround for GLib < 2.53.2 not searching for schemas in $XDG_DATA_HOME:
            #   https://bugzilla.gnome.org/show_bug.cgi?id=741335
            self.xdg_data_home,
            "$SNAP_USER_DATA",
            "$SNAP/data-dir",
            "$SNAP/share",
            "$SNAP/usr/share",
            "$XDG_DATA_DIRS",
        ]

        self.environment = {
            "FONTCONFIG_FILE": "$SNAP/gnome-platform/etc/fonts/fonts.conf",
            "FONTCONFIG_PATH": "$SNAP/gnome-platform/etc/fonts",
            "GST_PLUGIN_PATH": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gstreamer-1.0",
            "GST_PLUGIN_SCANNER": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner",
            "GST_PLUGIN_SYSTEM_PATH": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gstreamer-1.0",
            "IBUS_CONFIG_PATH": "{}/ibus".format(self.xdg_config_home),
            "LD_LIBRARY_PATH": ":".join(self.ld_library_paths),
            "LIBGL_DRIVERS_PATH": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/dri",
            "PATH": ":".join(self.path),
            "XCURSOR_PATH": "$SNAP/data-dir/icons",
            "XDG_CACHE_HOME": self.xdg_cache_home,
            "XDG_CONFIG_DIRS": ":".join(self.xdg_config_dirs),
            "XDG_CONFIG_HOME": self.xdg_config_home,
            "XDG_DATA_DIRS": ":".join(self.xdg_data_dirs),
            "XDG_DATA_HOME": self.xdg_data_home,
        }

        self.root_snippet = {
            "assumes": ["command-chain"],
            "environment": self.environment,
            "plugs": self.plugs,
        }

        self.command_chain = [
            "snap/command-chain/desktop-init",
            "snap/command-chain/desktop-mark-and-exec",
        ]

        self.app_snippet = {"command-chain": self.command_chain}

        self.parts = {
            "desktop-common-extension": {
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop-common",
                "organize": {"desktop-*": "snap/command-chain/"},
            }
        }
