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

from testtools.matchers import Equals

from snapcraft.internal.project_loader._extensions.opengl import ExtensionImpl

from .. import ProjectLoaderBaseTest


class ExtensionTest(ProjectLoaderBaseTest):
    def test_supported_bases(self):
        self.assertThat(ExtensionImpl.get_supported_bases(), Equals(("core18",)))

    def test_supported_confinement(self):
        self.assertThat(
            ExtensionImpl.get_supported_confinement(),
            Equals(("strict", "devmode", "classic")),
        )

    def test_extension_strict(self):
        opengl_extension = ExtensionImpl(
            extension_name="opengl", yaml_data=dict(base="core18", confinement="strict")
        )

        self.assertThat(
            opengl_extension.root_snippet,
            Equals(
                {
                    "layout": {
                        "/etc/glvnd": {"bind": "$SNAP/etc/glvnd"},
                        "/usr/lib/${SNAPCRAFT_ARCH_TRIPLET}/dri": {
                            "bind": "$SNAP/usr/lib/${SNAPCRAFT_ARCH_TRIPLET}/dri"
                        },
                        "/usr/share/glvnd": {"bind": "$SNAP/etc/glvnd"},
                    }
                }
            ),
        )
        self.assertThat(opengl_extension.app_snippet, Equals({"plugs": ["opengl"]}))
        self.assertThat(opengl_extension.part_snippet, Equals(dict()))
        self.assertThat(opengl_extension.parts, Equals(dict()))

    def test_extension_classic(self):
        opengl_extension = ExtensionImpl(
            extension_name="opengl",
            yaml_data=dict(base="core18", confinement="classic"),
        )

        self.assertThat(
            opengl_extension.root_snippet,
            Equals(
                {
                    "environment": {
                        "__EGL_VENDOR_LIBRARY_DIRS": "$SNAP/etc/glvnd/egl_vendor.d:$SNAP/usr/share/glvnd/egl_vendor.d",
                        "LIBGL_DRIVERS_PATH": "$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/dri",
                    }
                }
            ),
        )
        self.assertThat(opengl_extension.app_snippet, Equals(dict()))
        self.assertThat(opengl_extension.part_snippet, Equals(dict()))
        self.assertThat(
            opengl_extension.parts,
            Equals(
                {
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
            ),
        )
