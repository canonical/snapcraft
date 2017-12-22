# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import yaml

from testtools.matchers import Equals

from snapcraft.tests import (
    fixture_setup,
    integration
)


class AppstreamMetadataTestCase(integration.TestCase):

    def setUp(self):
        super().setUp()

        with open('test.metainfo.xml', 'w') as appstream_file:
            appstream_file.write(textwrap.dedent(
                """<?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <description>test-appstream-description</description>
                  <summary>test-appstream-summary</summary>
                </component>"""))

    def test_metadata_extracted_from_appstream(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, summary=None, description=None)
        snapcraft_yaml.data['adopt-info'] = 'test-part'
        snapcraft_yaml.update_part(
            'test-part', {
                'plugin': 'dump',
                'parse-info': ['test.metainfo.xml']})
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft('prime')
        with open(
                os.path.join('prime', 'meta', 'snap.yaml')) as snap_yaml_file:
            snap_yaml = yaml.load(snap_yaml_file)
        self.assertThat(
            snap_yaml['description'], Equals('test-appstream-description'))
        self.assertThat(
            snap_yaml['summary'], Equals('test-appstream-summary'))

    def test_specified_metadata_not_overwritten(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, description=None)
        snapcraft_yaml.data['adopt-info'] = 'test-part'
        snapcraft_yaml.update_part(
            'test-part', {
                'plugin': 'dump',
                'parse-info': ['test.metainfo.xml']})
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft('prime')
        with open(
                os.path.join('prime', 'meta', 'snap.yaml')) as snap_yaml_file:
            snap_yaml = yaml.load(snap_yaml_file)
        self.assertThat(
            snap_yaml['description'], Equals('test-appstream-description'))
        self.assertThat(
            snap_yaml['summary'], Equals('test-summary'))
