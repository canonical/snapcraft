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
import shutil
import yaml

from testtools.matchers import Equals, FileExists

import tests
from tests import fixture_setup, integration


class AppstreamMetadataTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()

        with open("test.metainfo.xml", "w") as appstream_file:
            appstream_file.write(
                textwrap.dedent(
                    """<?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <id>com.example.testid</id>
                  <description>test-appstream-description</description>
                  <summary>test-appstream-summary</summary>
                  <icon type="local">assets/test-icon.png</icon>
                  <launchable type="desktop-id">
                    com.example.test-app.desktop
                  </launchable>
                </component>"""
                )
            )

    def test_metadata_extracted_from_appstream(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, summary=None, description=None
        )
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "nil", "parse-info": ["test.metainfo.xml"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")
        with open(os.path.join("prime", "meta", "snap.yaml")) as snap_yaml_file:
            snap_yaml = yaml.load(snap_yaml_file)
        self.assertThat(snap_yaml["description"], Equals("test-appstream-description"))
        self.assertThat(snap_yaml["summary"], Equals("test-appstream-summary"))

    def test_specified_metadata_not_overwritten(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, description=None)
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "dump", "parse-info": ["test.metainfo.xml"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")
        with open(os.path.join("prime", "meta", "snap.yaml")) as snap_yaml_file:
            snap_yaml = yaml.load(snap_yaml_file)
        self.assertThat(snap_yaml["description"], Equals("test-appstream-description"))
        self.assertThat(snap_yaml["summary"], Equals("test-summary"))

    def test_icon_extracted_from_appstream(self):
        os.mkdir("assets")
        shutil.copyfile(
            os.path.join(os.path.dirname(tests.__file__), "data", "icon.png"),
            os.path.join(self.path, "assets", "test-icon.png"),
        )

        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, summary=None, description=None
        )
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "dump", "parse-info": ["test.metainfo.xml"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")

        self.assertThat(os.path.join("prime", "meta", "gui", "icon.png"), FileExists())

    def test_desktop_extracted_from_appstream(self):
        xdg_data_dir = os.path.join("usr", "local", "share", "applications")
        os.makedirs(os.path.join(xdg_data_dir, "com.example.test"))
        shutil.copyfile(
            os.path.join(os.path.dirname(tests.__file__), "data", "test.desktop"),
            os.path.join(self.path, xdg_data_dir, "com.example.test", "app.desktop"),
        )

        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, summary=None, description=None
        )
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "dump", "parse-info": ["test.metainfo.xml"]}
        )
        snapcraft_yaml.update_app(
            "test-app", {"command": "echo", "common-id": "com.example.testid"}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")

        self.assertThat(
            os.path.join("prime", "meta", "gui", "test-app.desktop"), FileExists()
        )
