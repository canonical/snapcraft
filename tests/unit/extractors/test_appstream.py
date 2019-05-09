# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2019 Canonical Ltd
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

import os
import textwrap

from snapcraft.extractors import appstream, ExtractedMetadata

import testscenarios
from testtools.matchers import Equals

from snapcraft.extractors import _errors
from tests import unit, skip


def _create_desktop_file(desktop_file_path, icon: str = None) -> None:
    dir_name = os.path.dirname(desktop_file_path)
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)
    f = open(desktop_file_path, "w")
    print("[Desktop Entry]", file=f)
    if icon:
        print("Icon={}".format(icon), file=f)
    f.close()


class AppstreamTestCase(unit.TestCase):

    scenarios = testscenarios.multiply_scenarios(
        [
            (
                "summary",
                {
                    "key": "summary",
                    "attributes": {},
                    "param_name": "summary",
                    "value": "test-summary",
                    "expect": "test-summary",
                },
            ),
            (
                "description",
                {
                    "key": "description",
                    "attributes": {},
                    "param_name": "description",
                    "value": "test-description",
                    "expect": "test-description",
                },
            ),
            (
                "local icon with non-existing file",
                {
                    "key": "icon",
                    "attributes": {"type": "local"},
                    "param_name": "icon",
                    "value": "/test/path",
                    "expect": None,
                },
            ),
            (
                "local icon with existing file",
                {
                    "key": "icon",
                    "attributes": {"type": "local"},
                    "param_name": "icon",
                    "value": "/icon.png",
                    "expect": "/icon.png",
                },
            ),
            (
                "common id",
                {
                    "key": "id",
                    "attributes": {},
                    "param_name": "common_id",
                    "value": "test-id",
                    "expect": "test-id",
                },
            ),
        ],
        [
            ("metainfo", {"file_extension": "metainfo.xml"}),
            ("appdata", {"file_extension": "appdata.xml"}),
        ],
    )

    def test_appstream(self):
        file_name = "foo.{}".format(self.file_extension)
        attributes = " ".join(
            '{attribute_name}="{attribute_value}"'.format(
                attribute_name=attribute, attribute_value=self.attributes[attribute]
            )
            for attribute in self.attributes
        )
        with open(file_name, "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <{key} {attributes}>{value}</{key}>
                </component>""".format(
                        key=self.key, value=self.value, attributes=attributes
                    )
                )
            )

        open("icon.png", "w").close()
        kwargs = {self.param_name: self.expect}
        expected = ExtractedMetadata(**kwargs)

        self.assertThat(appstream.extract(file_name, workdir="."), Equals(expected))


# See LP #1814898 for a description of possible fallbacks
class AppstreamIconsTestCase(unit.TestCase):
    def _create_appstream_file(self, icon: str = None, icon_type: str = "local"):
        with open("foo.appdata.xml", "w") as f:
            if icon:
                f.write(
                    textwrap.dedent(
                        """\
                    <?xml version="1.0" encoding="UTF-8"?>
                    <component>
                      <launchable type="desktop-id">my.app.desktop</launchable>
                      <icon type="{}">{}</icon>
                    </component>""".format(
                            icon_type, icon
                        )
                    )
                )
            else:
                f.write(
                    textwrap.dedent(
                        """\
                    <?xml version="1.0" encoding="UTF-8"?>
                    <component>
                      <launchable type="desktop-id">my.app.desktop</launchable>
                    </component>"""
                    )
                )

    def _create_icon_file(self, theme: str, size: str, filename: str) -> None:
        dir_name = os.path.join("usr", "share", "icons", theme, size, "apps")
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        open(os.path.join(dir_name, filename), "w").close()

    def _expect_icon(self, icon):
        expected = ExtractedMetadata(icon=icon)
        actual = appstream.extract("foo.appdata.xml", workdir=".")
        self.assertThat(actual.get_icon(), Equals(expected.get_icon()))

    def test_appstream_stock_icon_exists_png(self):
        self._create_appstream_file(icon="icon", icon_type="stock")
        self._create_icon_file("hicolor", "48x48", "icon.png")
        self._create_icon_file("hicolor", "64x64", "icon.png")
        self._expect_icon("usr/share/icons/hicolor/64x64/apps/icon.png")

    def test_appstream_stock_icon_not_exist(self):
        self._create_appstream_file(icon="missing", icon_type="stock")
        self._expect_icon(None)

    def test_appstream_no_icon_no_fallback(self):
        self._create_appstream_file()
        self._expect_icon(None)

    def test_appstream_local_icon_exists(self):
        self._create_appstream_file(icon="/icon.png")
        open("icon.png", "w").close()
        self._expect_icon("/icon.png")

    def test_appstream_local_icon_not_exist_no_fallback(self):
        self._create_appstream_file(icon="/missing.png")
        self._expect_icon(None)

    def test_appstream_local_icon_not_absolute_no_fallback(self):
        self._create_appstream_file(icon="foo")
        self._expect_icon(None)

    def test_appstream_no_icon_desktop_fallback_no_icon(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop")
        self._expect_icon(None)

    def test_appstream_no_icon_desktop_fallback_icon_not_exist(self):
        self._create_appstream_file()
        _create_desktop_file(
            "usr/share/applications/my.app.desktop", icon="/missing.png"
        )
        self._expect_icon("/missing.png")

    def test_appstream_no_icon_desktop_fallback_icon_exists(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="/icon.png")
        open("icon.png", "w").close()
        self._expect_icon("/icon.png")

    def test_appstream_no_icon_desktop_fallback_icon_exists_with_snap_variable_prefix(
        self
    ):
        self._create_appstream_file()
        _create_desktop_file(
            "usr/share/applications/my.app.desktop", icon="${{SNAP}}/icon.png"
        )  # icon: '${SNAP}/icon.png'.
        open("icon.png", "w").close()
        self._expect_icon("/icon.png")

    def test_appstream_no_icon_theme_fallback_png(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svg")
        self._create_icon_file("hicolor", "48x48", "icon.png")
        self._create_icon_file("hicolor", "64x64", "icon.png")
        self._expect_icon("usr/share/icons/hicolor/64x64/apps/icon.png")

    def test_appstream_no_icon_theme_fallback_xpm(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svg")
        self._create_icon_file("hicolor", "48x48", "icon.png")
        self._create_icon_file("hicolor", "64x64", "icon.xpm")
        self._expect_icon("usr/share/icons/hicolor/64x64/apps/icon.xpm")

    def test_appstream_no_icon_theme_fallback_svg(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svg")
        self._expect_icon("usr/share/icons/hicolor/scalable/apps/icon.svg")

    def test_appstream_no_icon_theme_fallback_svgz(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svgz")
        self._expect_icon("usr/share/icons/hicolor/scalable/apps/icon.svgz")


class AppstreamTest(unit.TestCase):
    @skip.skip_unless_codename(
        "xenial", "this test relies on libxslt from xenial to work"
    )
    def test_appstream_with_ul(self):
        file_name = "snapcraft.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="utf-8"?>
            <component type="desktop">
              <id>io.snapcraft.snapcraft</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0</project_license>
              <name>snapcraft</name>
              <name xml:lang="es">snapcraft</name>
              <summary>Create snaps</summary>
              <summary xml:lang="es">Crea snaps</summary>
              <description>
                <p>Command Line Utility to create snaps.</p>
                <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
                <p>Features:</p>
                <p xml:lang="es">Funciones:</p>
                <ul>
                  <li>Build snaps.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ul>
              </description>
              <provides>
                <binary>snapcraft</binary>
              </provides>
            </component>
        """
        )

        with open(file_name, "w") as f:
            print(content, file=f)

        metadata = appstream.extract(file_name, workdir=".")

        self.assertThat(metadata.get_summary(), Equals("Create snaps"))
        self.assertThat(
            metadata.get_description(),
            Equals(
                textwrap.dedent(
                    """\
            Command Line Utility to create snaps.

            Features:

            - Build snaps.
            - Publish snaps to the store."""
                )
            ),
        )

    @skip.skip_unless_codename(
        "xenial", "this test relies on libxslt from xenial to work"
    )
    def test_appstream_with_ol(self):
        file_name = "snapcraft.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="utf-8"?>
            <component type="desktop">
              <id>io.snapcraft.snapcraft</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0</project_license>
              <name>snapcraft</name>
              <name xml:lang="es">snapcraft</name>
              <summary>Create snaps</summary>
              <summary xml:lang="es">Crea snaps</summary>
              <description>
                <p>Command Line Utility to create snaps.</p>
                <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
                <p>Features:</p>
                <p xml:lang="es">Funciones:</p>
                <ol>
                  <li>Build snaps.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ol>
              </description>
              <provides>
                <binary>snapcraft</binary>
              </provides>
            </component>
        """
        )

        with open(file_name, "w") as f:
            print(content, file=f)

        metadata = appstream.extract(file_name, workdir=".")

        self.assertThat(metadata.get_summary(), Equals("Create snaps"))
        self.assertThat(
            metadata.get_description(),
            Equals(
                textwrap.dedent(
                    """\
            Command Line Utility to create snaps.

            Features:

            1. Build snaps.
            2. Publish snaps to the store."""
                )
            ),
        )


class AppstreamUnhandledFileTestCase(unit.TestCase):
    def test_unhandled_file_test_case(self):
        raised = self.assertRaises(
            _errors.UnhandledFileError, appstream.extract, "unhandled-file", workdir="."
        )

        self.assertThat(raised.path, Equals("unhandled-file"))
        self.assertThat(raised.extractor_name, Equals("appstream"))


class AppstreamLaunchableTestCase(unit.TestCase):

    scenarios = (
        (
            "usr/share",
            {
                "desktop_file_path": "usr/share/applications/com.example.test/app.desktop",
                "workdir": ".",
            },
        ),
        (
            "usr/share in installdir",
            {
                "desktop_file_path": "usr/share/applications/com.example.test/app.desktop",
                "workdir": "install",
            },
        ),
        (
            "usr/local/share",
            {
                "desktop_file_path": "usr/local/share/applications/com.example.test/app.desktop",
                "workdir": ".",
            },
        ),
    )

    def test_appstream_with_launchable(self):
        os.makedirs(self.workdir, exist_ok=True)
        appstream_file = os.path.join(self.workdir, "foo.metainfo.xml")
        with open(appstream_file, "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <launchable type="desktop-id">
                    com.example.test-app.desktop
                  </launchable>
                </component>"""
                )
            )

        desktop_file_path = os.path.join(self.workdir, self.desktop_file_path)
        _create_desktop_file(desktop_file_path)

        extracted = appstream.extract("foo.metainfo.xml", workdir=self.workdir)

        self.assertThat(
            extracted.get_desktop_file_paths(), Equals([self.desktop_file_path])
        )


class AppstreamLegacyDesktopTest(unit.TestCase):

    scenarios = (
        (
            "usr/share",
            {
                "desktop_file_path": "usr/share/applications/com.example.test/app.desktop"
            },
        ),
        (
            "usr/local/share",
            {
                "desktop_file_path": "usr/local/share/applications/com.example.test/app.desktop"
            },
        ),
    )

    def test_appstream_with_launchable(self):
        with open("foo.metainfo.xml", "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component type="desktop">
                  <id>com.example.test-app.desktop</id>
                </component>"""
                )
            )

        _create_desktop_file(self.desktop_file_path)

        extracted = appstream.extract("foo.metainfo.xml", workdir=".")

        self.assertThat(
            extracted.get_desktop_file_paths(), Equals([self.desktop_file_path])
        )


class AppstreamMultipleLaunchableTestCase(unit.TestCase):
    def test_appstream_with_multiple_launchables(self):
        with open("foo.metainfo.xml", "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <launchable type="desktop-id">
                    com.example.test-app1.desktop
                  </launchable>
                  <launchable type="test-wrong-type">
                    dummy
                  </launchable>
                  <launchable type="desktop-id">
                    com.example.test-app2.desktop
                  </launchable>
                  <launchable type="desktop-id">
                    unexisting
                  </launchable>
                </component>"""
                )
            )

        _create_desktop_file(
            "usr/local/share/applications/com.example.test/app1.desktop"
        )
        _create_desktop_file(
            "usr/local/share/applications/com.example.test/app2.desktop"
        )

        expected = [
            "usr/local/share/applications/com.example.test/app1.desktop",
            "usr/local/share/applications/com.example.test/app2.desktop",
        ]
        extracted = appstream.extract("foo.metainfo.xml", workdir=".")

        self.assertThat(extracted.get_desktop_file_paths(), Equals(expected))
