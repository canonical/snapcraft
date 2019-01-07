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

from snapcraft.internal import errors
from ._desktop_common import DesktopCommonExtension


class GnomeExtensionClassicConfinementError(errors.SnapcraftError):
    fmt = "The gnome extension doesn't support classic confinement."


class GnomeExtension(DesktopCommonExtension):
    """The Gnome extension.
    This extension is to be used by applications that require GTK+.
    Examples might include productivity applications or utilities.
    Note that this extension does not support classically-confined snaps at this time.
    """

    supported_bases = ("core16", "core18")

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new GnomeExtension.
        Note that this extension does not support classic snaps.
        :param dict yaml_data: Loaded snapcraft.yaml data.
        """

        super().__init__(yaml_data)

        if yaml_data.get("confinement") == "classic":
            raise GnomeExtensionClassicConfinementError()

        platform_snap = "gnome-3-26-1604"  # default
        base = yaml_data.get("base")
        after_dependencies = []
        dependency_part = {}
        if base is not None:
            if base == "core16":
                platform_snap = "gnome-3-26-1604"
                after_dependencies = ["gnome-extension-platform-dependencies"]
                dependency_part = {
                    "gnome-extension-platform-dependencies": {
                        "plugin": None,
                        "override-pull": """
                            add-apt-repository ppa:
                            apt-get update
                            apt-get upgrade -yqq
                            """,
                    }
                }
            elif base == "core18":
                platform_snap = "gnome-3-28-1804"

        gi_typelib_paths = [
            "$SNAP/usr/lib/gjs/girepository-1.0",
            "$SNAP/usr/lib/girepository-1.0",
            "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/girepository-1.0",
            "$SNAP/gnome-platform/usr/lib/girepository-1.0",
            "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/girepository-1.0",
            "$GI_TYPELIB_PATH",
        ]

        ld_library_paths = [
            "$SNAP/gnome-platform/lib/$SNAPCRAFT_ARCH_TRIPLET",
            "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET",
            "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/mesa",
            "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/mesa-egl",
            "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/dri",
            "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/libunity",
            "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pulseaudio",
        ] + self.ld_library_paths

        path = self.path + ["$SNAP/gnome-platform/usr/bin"]

        xdg_config_dirs = ["$SNAP/gnome-platform/etc/xdg"] + self.xdg_config_dirs
        xdg_data_dirs = ["$SNAP/gnome-platform/usr/share"] + self.xdg_data_dirs

        self.root_snippet = {
            **self.root_snippet,
            "plugs": {
                **self.plugs,
                platform_snap: {
                    "interface": "content",
                    "target": "$SNAP/gnome-platform",
                    "default-provider": "{snap}:{slot}".format(
                        snap=platform_snap, slot=platform_snap
                    ),
                },
            },
            "environment": {
                **self.environment,
                "LD_LIBRARY_PATH": ":".join(ld_library_paths),
                "GDK_PIXBUF_MODULE_FILE": "{}/gdk-pixbuf-loaders.cache".format(
                    self.xdg_cache_home
                ),
                "GDK_PIXBUF_MODULEDIR": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-2.0/2.10.0/loaders",
                "GI_TYPELIB_PATH": ":".join(gi_typelib_paths),
                "GIO_MODULE_DIR": "{}/gio-modules".format(self.xdg_cache_home),
                "GS_SCHEMA_DIR": "{}/glib-2.0/schemas".format(self.xdg_data_home),
                "GST_PLUGIN_SCANNER": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner",
                "GST_PLUGIN_SYSTEM_PATH": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gstreamer-1.0",
                "GTK_EXE_PREFIX": "$SNAP/gnome-platform/usr",
                "GTK_IM_MODULE_DIR": "{}/immodules".format(self.xdg_cache_home),
                "GTK_IM_MODULE_FILE": "{}/immodules/immodules.cache".format(
                    self.xdg_cache_home
                ),
                "LOCPATH": "$SNAP/gnome-platform/usr/lib/locale",
                "PATH": ":".join(path),
                "XCURSOR_PATH": "$SNAP/data-dir/icons:$SNAP/gnome-platform/usr/share/icons",
                "XDG_CONFIG_DIRS": ":".join(xdg_config_dirs),
                "XDG_DATA_DIRS": ":".join(xdg_data_dirs),
                "XKB_CONFIG_ROOT": "$SNAP/gnome-platform/usr/share/X11/xkb",
                "XLOCALEDIR": "$SNAP/gnome-platform/usr/share/X11/locale",
            },
        }

        command_chain = self.app_snippet["command-chain"]
        exec_command = command_chain.pop()

        command_chain = command_chain + [
            "snap/command-chain/desktop-gnome-specific",
            exec_command,
        ]

        self.app_snippet = {
            **self.app_snippet,
            "command-chain": command_chain,
            "adapter": "full",
        }

        self.parts = {
            **self.parts,
            **dependency_part,
            "gnome-extension": {
                "after": after_dependencies,
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/gnome",
                "source-type": "local",
                "organize": {"desktop-*": "snap/command-chain/"},
                "build-packages": ["build-essential", "libgtk-3-dev"],
            },
        }
