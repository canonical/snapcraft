# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft.internal.project_loader._extensions.gnome_3_28 import ExtensionImpl

from .. import ProjectLoaderBaseTest


class ExtensionTest(ProjectLoaderBaseTest):
    def test_extension(self):
        gnome_extension = ExtensionImpl(
            extension_name="gnome-3.28", yaml_data=dict(base="core18")
        )

        self.expectThat(
            gnome_extension.root_snippet,
            Equals(
                {
                    "assumes": ["snapd2.43"],
                    "plugs": {
                        "desktop": {"mount-host-font-cache": False},
                        "gtk-3-themes": {
                            "interface": "content",
                            "target": "$SNAP/data-dir/themes",
                            "default-provider": "gtk-common-themes",
                        },
                        "icon-themes": {
                            "interface": "content",
                            "target": "$SNAP/data-dir/icons",
                            "default-provider": "gtk-common-themes",
                        },
                        "sound-themes": {
                            "interface": "content",
                            "target": "$SNAP/data-dir/sounds",
                            "default-provider": "gtk-common-themes",
                        },
                        "gnome-3-28-1804": {
                            "interface": "content",
                            "target": "$SNAP/gnome-platform",
                            "default-provider": "gnome-3-28-1804",
                        },
                    },
                    "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform"},
                    "hooks": {
                        "configure": {
                            "plugs": ["desktop"],
                            "command-chain": [
                                "snap/command-chain/hooks-configure-desktop"
                            ],
                        }
                    },
                    "layout": {
                        "/usr/bin/gjs": {"symlink": "$SNAP/gnome-platform/usr/bin/gjs"},
                        "/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0": {
                            "bind": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0"
                        },
                        "/usr/share/xml/iso-codes": {
                            "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                        },
                    },
                }
            ),
        )
        self.expectThat(
            gnome_extension.app_snippet,
            Equals(
                {
                    "command-chain": ["snap/command-chain/desktop-launch"],
                    "plugs": [
                        "desktop",
                        "desktop-legacy",
                        "gsettings",
                        "opengl",
                        "wayland",
                        "x11",
                    ],
                }
            ),
        )
        self.expectThat(gnome_extension.part_snippet, Equals(dict()))
        self.expectThat(
            gnome_extension.parts,
            Equals(
                {
                    "gnome-3-28-extension": {
                        "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                        "source-subdir": "gnome",
                        "plugin": "make",
                        "make-parameters": [
                            "WITH_BINDTEXTDOMAIN=1",
                            "PLATFORM_PLUG=gnome-3-28-1804",
                        ],
                        "build-packages": ["gcc", "libgtk-3-dev"],
                    }
                }
            ),
        )

    def test_supported_bases(self):
        self.assertThat(ExtensionImpl.get_supported_bases(), Equals(("core18",)))

    def test_supported_confinement(self):
        self.assertThat(
            ExtensionImpl.get_supported_confinement(), Equals(("strict", "devmode"))
        )
