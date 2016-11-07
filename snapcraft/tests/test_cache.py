# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import fixtures

from snapcraft import tests
from snapcraft.internal import cache
from snapcraft.tests import fixture_setup


class SnapCacheTestCase(tests.TestCase):

    yaml_content = """name: cache-test
version: 0.1
summary: test cached snap
description: test cached snap
grade: devel

parts:
  my-part:
    plugin: nil
"""

    def setUp(self):
        super().setUp()
        super().make_snapcraft_yaml(self.yaml_content)
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    def test_rewrite_snap_filename(self):
        revision = 10
        snap_file = 'my-snap-name_0.1_amd64.snap'

        self.assertEqual(
            'my-snap-name_0.1_amd64_10.snap',
            cache.rewrite_snap_filename_with_revision(snap_file, revision))

    def test_snap_cache(self):
        self.useFixture(fixture_setup.FakeTerminal())
        snap_cache = cache.SnapCache()
        snap_file = 'my-snap-name_0.1_amd64.snap'

        # create dummy snap
        open(os.path.join(self.path, snap_file), 'a').close()

        # cache snap
        cached_snap_path = snap_cache.cache(snap_file, 10)

        _, expected_snap = os.path.split(cached_snap_path)

        self.assertEqual('my-snap-name_0.1_amd64_10.snap', expected_snap)
        self.assertTrue(os.path.isfile(cached_snap_path))
