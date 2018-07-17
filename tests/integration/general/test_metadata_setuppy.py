# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
import yaml
from textwrap import dedent

from testtools.matchers import Equals

from tests import integration, fixture_setup


class SetupPyMetadataTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()

        with open("setup.py", "w") as setup_file:
            print(
                dedent(
                    """\
                from setuptools import setup

                setup(
                    name='hello-world',
                    version='test-setuppy-version',
                    description='test-setuppy-description',
                    author='Canonical LTD',
                    author_email='snapcraft@lists.snapcraft.io',
                    scripts=['hello']
                )
                """
                ),
                file=setup_file,
            )

    def test_metadata_extracted_from_setuppy(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, version=None, description=None
        )
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "nil", "parse-info": ["setup.py"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")
        snap_yaml_path = os.path.join("prime", "meta", "snap.yaml")
        with open(snap_yaml_path) as snap_yaml_file:
            snap_yaml = yaml.load(snap_yaml_file)

        self.assertThat(snap_yaml["version"], Equals("test-setuppy-version"))
        self.assertThat(snap_yaml["description"], Equals("test-setuppy-description"))

    def test_all_metadata_from_yaml(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, version="test-yaml-version", description="test-yaml-description"
        )
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "dump", "parse-info": ["setup.py"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")
        snap_yaml_path = os.path.join("prime", "meta", "snap.yaml")
        with open(snap_yaml_path) as snap_yaml_file:
            snap_yaml = yaml.load(snap_yaml_file)

        self.assertThat(snap_yaml["version"], Equals("test-yaml-version"))
        self.assertThat(snap_yaml["description"], Equals("test-yaml-description"))

    def test_version_metadata_from_yaml_description_collected(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, version="test-yaml-version", description=None
        )
        snapcraft_yaml.data["adopt-info"] = "test-part"
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "dump", "parse-info": ["setup.py"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")
        snap_yaml_path = os.path.join("prime", "meta", "snap.yaml")
        with open(snap_yaml_path) as snap_yaml_file:
            snap_yaml = yaml.load(snap_yaml_file)

        self.assertThat(snap_yaml["version"], Equals("test-yaml-version"))
        self.assertThat(snap_yaml["description"], Equals("test-setuppy-description"))
