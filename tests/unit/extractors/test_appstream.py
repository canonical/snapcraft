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
from tests import unit


class AppstreamTestCase(unit.TestCase):

    scenarios = testscenarios.multiply_scenarios(
        [
            (
                "summary",
                {
                    "key": "summary",
                    "attributes": {},
                    "value": "test-summary",
                    "param_name": "summary",
                },
            ),
            (
                "description",
                {
                    "key": "description",
                    "attributes": {},
                    "value": "test-description",
                    "param_name": "description",
                },
            ),
            (
                "local icon",
                {
                    "key": "icon",
                    "attributes": {"type": "local"},
                    "param_name": "icon",
                    "value": "/test/path",
                },
            ),
            (
                "common id",
                {
                    "key": "id",
                    "attributes": {},
                    "param_name": "common_id",
                    "value": "test-id",
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

        kwargs = {self.param_name: self.value}
        expected = ExtractedMetadata(**kwargs)

        self.assertThat(appstream.extract(file_name, workdir="."), Equals(expected))


class AppstreamTest(unit.TestCase):
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
        os.makedirs(os.path.dirname(desktop_file_path))
        open(desktop_file_path, "w").close()

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

        os.makedirs(os.path.dirname(self.desktop_file_path))
        open(self.desktop_file_path, "w").close()

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

        os.makedirs("usr/local/share/applications/com.example.test/")
        open("usr/local/share/applications/com.example.test/app1.desktop", "w").close()
        open("usr/local/share/applications/com.example.test/app2.desktop", "w").close()

        expected = [
            "usr/local/share/applications/com.example.test/app1.desktop",
            "usr/local/share/applications/com.example.test/app2.desktop",
        ]
        extracted = appstream.extract("foo.metainfo.xml", workdir=".")

        self.assertThat(extracted.get_desktop_file_paths(), Equals(expected))
