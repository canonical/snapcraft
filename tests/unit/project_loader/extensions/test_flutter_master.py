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

import textwrap

from testtools.matchers import Equals

from snapcraft.internal.project_loader._extensions.flutter_master import ExtensionImpl

from .. import ProjectLoaderBaseTest


class ExtensionTest(ProjectLoaderBaseTest):
    def test_extension(self):
        flutter_extension = ExtensionImpl(
            extension_name="flutter-master", yaml_data=dict(base="core18")
        )

        self.expectThat(
            flutter_extension.root_snippet,
            Equals(
                {
                    "plugs": {
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
                    "layout": {
                        "/usr/share/xml/iso-codes": {
                            "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                        },
                    },
                }
            ),
        )
        self.expectThat(
            flutter_extension.app_snippet,
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
        self.expectThat(flutter_extension.part_snippet, Equals(dict()))
        self.expectThat(
            flutter_extension.parts,
            Equals(
                {
                    "gnome-3-28-extension": {
                        "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                        "source-subdir": "gnome",
                        "plugin": "make",
                        "build-packages": ["gcc", "libgtk-3-dev"],
                    },
                    "flutter-extension": {
                        "plugin": "nil",
                        "override-pull": textwrap.dedent(
                            """\
                            flutter channel master
                            flutter config --enable-linux-desktop
                            flutter upgrade
                            flutter doctor
                            """
                        ),
                        "build-snaps": ["flutter/latest/stable"],
                    },
                }
            ),
        )

    def test_supported_bases(self):
        self.assertThat(ExtensionImpl.get_supported_bases(), Equals(("core18",)))

    def test_supported_confinement(self):
        self.assertThat(
            ExtensionImpl.get_supported_confinement(), Equals(("strict", "devmode"))
        )
