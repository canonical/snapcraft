# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

        self.assertThat(appstream.extract(file_name), Equals(expected))


class AppstreamUnhandledFileTestCase(unit.TestCase):
    def test_unhandled_file_test_case(self):
        raised = self.assertRaises(
            _errors.UnhandledFileError, appstream.extract, "unhandled-file"
        )

        self.assertThat(raised.path, Equals("unhandled-file"))
        self.assertThat(raised.extractor_name, Equals("appstream"))


class AppstreamLaunchableTestCase(unit.TestCase):

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
                <component>
                  <launchable type="desktop-id">
                    com.example.test-app.desktop
                  </launchable>
                </component>"""
                )
            )

        os.makedirs(os.path.dirname(self.desktop_file_path))
        open(self.desktop_file_path, "w").close()

        expected = ExtractedMetadata(desktop_file_paths=[self.desktop_file_path])

        self.assertThat(appstream.extract("foo.metainfo.xml"), Equals(expected))


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

        expected = ExtractedMetadata(
            desktop_file_paths=[
                "usr/local/share/applications/com.example.test/app1.desktop",
                "usr/local/share/applications/com.example.test/app2.desktop",
            ]
        )

        self.assertThat(appstream.extract("foo.metainfo.xml"), Equals(expected))
