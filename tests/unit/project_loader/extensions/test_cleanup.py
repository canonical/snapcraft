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

from snapcraft.internal.project_loader._extensions.z_cleanup import ExtensionImpl

from .. import ProjectLoaderBaseTest


_CLEAN_SCRIPT = """
set -eux
for snap in core18; do
    cd "/snap/$snap/current" && find . -type f,l -exec rm -f "$SNAPCRAFT_PRIME/{}" \\;
done
"""

_CLEAN_SCRIPT_2 = """
set -eux
for snap in core18 gnome-3-28-1804 gtk-common-themes; do
    cd "/snap/$snap/current" && find . -type f,l -exec rm -f "$SNAPCRAFT_PRIME/{}" \\;
done
"""


class ExtensionTest(ProjectLoaderBaseTest):
    def test_extension_nopart(self):
        cleanup_extension = ExtensionImpl(
            extension_name="cleanup", yaml_data={"base": "core18"}
        )

        self.expectThat(
            cleanup_extension.parts,
            Equals(
                {
                    "cleanup": {
                        "after": [],
                        "plugin": "nil",
                        "build-snaps": ["core18"],
                        "override-prime": _CLEAN_SCRIPT,
                    }
                }
            ),
        )

    def test_extension(self):
        cleanup_extension = ExtensionImpl(
            extension_name="cleanup",
            yaml_data={
                "base": "core18",
                "parts": {"my-part": {}, "my-second-part": {}},
                "plugs": {
                    "gnome-3-28-1804": {
                        "default-provider": "gnome-3-28-1804",
                        "interface": "content",
                        "target": "$SNAP/gnome-platform",
                    },
                    "gtk-3-themes": {
                        "default-provider": "gtk-common-themes",
                        "interface": "content",
                        "target": "$SNAP/data-dir/themes",
                    },
                    "icon-themes": {
                        "default-provider": "gtk-common-themes",
                        "interface": "content",
                        "target": "$SNAP/data-dir/icons",
                    },
                },
            },
        )

        self.expectThat(
            cleanup_extension.parts,
            Equals(
                {
                    "cleanup": {
                        "after": ["my-part", "my-second-part"],
                        "plugin": "nil",
                        "build-snaps": [
                            "core18",
                            "gnome-3-28-1804",
                            "gtk-common-themes",
                        ],
                        "override-prime": _CLEAN_SCRIPT_2,
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
