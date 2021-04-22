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

from snapcraft.internal.project_loader._extensions.gnome_3_34 import ExtensionImpl
from tests.unit.commands import CommandBaseTestCase

from .. import ProjectLoaderBaseTest


class ExtensionTest(ProjectLoaderBaseTest, CommandBaseTestCase):
    def test_extension(self):
        gnome_extension = ExtensionImpl(
            extension_name="gnome-3.34", yaml_data=dict(base="core18")
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
                        "gnome-3-34-1804": {
                            "interface": "content",
                            "target": "$SNAP/gnome-platform",
                            "default-provider": "gnome-3-34-1804",
                        },
                    },
                    "environment": {
                        "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
                        "GTK_USE_PORTAL": "1",
                    },
                    "hooks": {
                        "configure": {
                            "plugs": ["desktop"],
                            "command-chain": [
                                "snap/command-chain/hooks-configure-desktop"
                            ],
                        }
                    },
                    "layout": {
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
        self.expectThat(
            gnome_extension.part_snippet,
            Equals(
                {
                    "build-environment": [
                        {"PATH": "/snap/gnome-3-34-1804-sdk/current/usr/bin:$PATH"},
                        {
                            "XDG_DATA_DIRS": "$SNAPCRAFT_STAGE/usr/share:/snap/gnome-3-34-1804-sdk/current/usr/share:/usr/share:$XDG_DATA_DIRS"
                        },
                        {
                            "LD_LIBRARY_PATH": "/snap/gnome-3-34-1804-sdk/current/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib:/snap/gnome-3-34-1804-sdk/current/usr/lib/vala-current:$LD_LIBRARY_PATH"
                        },
                        {
                            "PKG_CONFIG_PATH": "/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/lib/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/share/pkgconfig:$PKG_CONFIG_PATH"
                        },
                        {
                            "GETTEXTDATADIRS": "/snap/gnome-3-34-1804-sdk/current/usr/share/gettext-current:$GETTEXTDATADIRS"
                        },
                        {
                            "GDK_PIXBUF_MODULE_FILE": "/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache"
                        },
                        {
                            "PYTHONPATH": "/snap/gnome-3-34-1804-sdk/current/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
                        },
                    ]
                }
            ),
        )
        self.expectThat(
            gnome_extension.parts,
            Equals(
                {
                    "gnome-3-34-extension": {
                        "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                        "source-subdir": "gnome",
                        "plugin": "make",
                        "make-parameters": [
                            "WITH_BINDTEXTDOMAIN=1",
                            "PLATFORM_PLUG=gnome-3-34-1804",
                        ],
                        "build-snaps": ["gnome-3-34-1804-sdk/latest/stable"],
                        "build-packages": ["gcc"],
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
