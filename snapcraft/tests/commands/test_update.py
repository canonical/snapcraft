# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
from collections import OrderedDict

import fixtures
import yaml
from testtools.matchers import Contains, Equals, FileExists
from xdg import BaseDirectory

from snapcraft.tests import TestWithFakeRemoteParts
from . import CommandBaseTestCase


class UpdateCommandTestCase(CommandBaseTestCase, TestWithFakeRemoteParts):

    def setUp(self):
        super().setUp()
        self.parts_dir = os.path.join(BaseDirectory.xdg_data_home, 'snapcraft')
        self.parts_yaml = os.path.join(self.parts_dir, 'parts.yaml')
        self.headers_yaml = os.path.join(self.parts_dir, 'headers.yaml')

    def test_update(self):
        result = self.run_command(['update'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_yaml, FileExists())
        self.assertThat(self.headers_yaml, FileExists())

        expected_parts = OrderedDict()
        expected_parts['curl'] = p = OrderedDict()
        p['plugin'] = 'autotools'
        p['source'] = 'http://curl.org'
        p['description'] = 'test entry for curl'
        p['maintainer'] = 'none'

        expected_parts['part1'] = p = OrderedDict()
        p['plugin'] = 'go'
        p['source'] = 'http://source.tar.gz'
        p['description'] = 'test entry for part1'
        p['maintainer'] = 'none'

        expected_parts['long-described-part'] = p = OrderedDict()
        p['plugin'] = 'go'
        p['source'] = 'http://source.tar.gz'
        p['description'] = 'this is a repetitive description ' * 3
        p['maintainer'] = 'none'

        expected_parts['multiline-part'] = p = OrderedDict()
        p['plugin'] = 'go'
        p['source'] = 'http://source.tar.gz'
        p['description'] = 'this is a multiline description\n' * 3
        p['maintainer'] = 'none'

        expected_headers = {
            'If-Modified-Since': 'Thu, 07 Jul 2016 10:00:20 GMT',
        }

        with open(self.parts_yaml) as parts_file:
            parts = yaml.load(parts_file)
        with open(self.headers_yaml) as headers_file:
            headers = yaml.load(headers_file)

        self.assertThat(parts, Equals(expected_parts))
        self.assertThat(headers, Equals(expected_headers))

    def test_update_with_unchanged_date_does_not_download_again(self):
        result = self.run_command(['update'])
        self.assertThat(result.exit_code, Equals(0))

        result = self.run_command(['update'])
        self.assertThat(result.exit_code, Equals(0))

        self.assertThat(result.output, Contains(
            'The parts cache is already up to date.'))

    def test_update_with_changed_date_downloads_again(self):
        os.makedirs(self.parts_dir)
        with open(self.headers_yaml, 'w') as headers_file:
            yaml.dump(
                {'If-Modified-Since': 'Fri, 01 Jan 2016 12:00:00 GMT'},
                headers_file)

        result = self.run_command(['update'])
        self.assertThat(result.exit_code, Equals(0))

        self.assertThat(result.output, Equals(''))

    def test_update_with_no_content_length_is_supported(self):
        self.useFixture(fixtures.EnvironmentVariable('NO_CONTENT_LENGTH', '1'))
        result = self.run_command(['update'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_yaml, FileExists())
        self.assertThat(self.headers_yaml, FileExists())
