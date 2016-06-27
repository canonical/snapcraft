# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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


import logging
import os
from unittest import mock

import fixtures
import yaml
from xdg import BaseDirectory

from snapcraft import main, tests
from snapcraft.tests import fixture_setup


class UpdateCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeParts())
        patcher = mock.patch(
            'snapcraft.internal.parts.ProgressBar',
            new=tests.SilentProgressBar)
        patcher.start()
        self.addCleanup(patcher.stop)

        self.parts_dir = os.path.join(BaseDirectory.xdg_data_home, 'snapcraft')
        self.parts_yaml = os.path.join(self.parts_dir, 'parts.yaml')
        self.headers_yaml = os.path.join(self.parts_dir, 'headers.yaml')

    def test_update(self):
        main.main(['update'])

        self.assertTrue(os.path.exists(self.parts_yaml))
        self.assertTrue(os.path.exists(self.headers_yaml))

        expected_parts = {
            'curl': {
                'source': 'http://curl.org',
                'plugin': 'autotools',
                'description': 'test entry for curl',
                'maintainer': 'none',
            },
            'part1': {
                'plugin': 'go',
                'source': 'http://source.tar.gz',
                'description': 'test entry for part1',
                'maintainer': 'none',
            },
            'long-described-part': {
                'plugin': 'go',
                'source': 'http://source.tar.gz',
                'description': 'this is a repetitive description ' * 3,
                'maintainer': 'none',
            },
        }
        expected_headers = {
            'If-None-Match': '1111',
        }

        with open(self.parts_yaml) as parts_file:
            parts = yaml.load(parts_file)
        with open(self.headers_yaml) as headers_file:
            headers = yaml.load(headers_file)

        self.assertEqual(parts, expected_parts)
        self.assertEqual(headers, expected_headers)

    def test_update_with_unchanged_etag_does_not_download_again(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        main.main(['update'])
        main.main(['update'])

        self.assertEqual(
            'The parts cache is already up to date.\n',
            fake_logger.output)

    def test_update_with_no_content_length_is_supported(self):
        self.useFixture(fixtures.EnvironmentVariable('NO_CONTENT_LENGTH', '1'))
        main.main(['update'])
