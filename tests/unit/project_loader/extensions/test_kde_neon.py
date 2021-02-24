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

from snapcraft.internal.project_loader._extensions.kde_neon import ExtensionImpl

from .. import ProjectLoaderBaseTest


class ExtensionTest(ProjectLoaderBaseTest):
    def test_extension(self):
        kde_neon_extension = ExtensionImpl(
            extension_name="kde-neon", yaml_data=dict(base="core18")
        )

        self.expectThat(
            kde_neon_extension.root_snippet,
            Equals(
                {
                    "assumes": ["snapd2.43"],
                    "plugs": {
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
                        "kde-frameworks-5-plug": {
                            "content": "kde-frameworks-5-core18-all",
                            "interface": "content",
                            "target": "$SNAP/kf5",
                            "default-provider": "kde-frameworks-5-core18",
                        },
                    },
                    "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf5"},
                    "hooks": {
                        "configure": {
                            "plugs": ["desktop"],
                            "command-chain": [
                                "snap/command-chain/hooks-configure-desktop"
                            ],
                        }
                    },
                }
            ),
        )
        self.expectThat(
            kde_neon_extension.app_snippet,
            Equals(
                {
                    "command-chain": ["snap/command-chain/desktop-launch"],
                    "plugs": ["desktop", "desktop-legacy", "wayland", "x11"],
                }
            ),
        )
        self.expectThat(kde_neon_extension.part_snippet, Equals(dict()))
        self.expectThat(
            kde_neon_extension.parts,
            Equals(
                {
                    "kde-neon-extension": {
                        "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                        "source-subdir": "kde-neon",
                        "plugin": "make",
                        "make-parameters": ["PLATFORM_PLUG=kde-frameworks-5-plug"],
                        "build-packages": ["g++"],
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
