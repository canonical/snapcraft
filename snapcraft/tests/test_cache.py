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
import time

import fixtures

from snapcraft import (
    file_utils,
    tests,
)
from snapcraft.internal import cache
from snapcraft.tests import fixture_setup


yaml_content = """name: cache-test
version: 0.1
summary: test cached snap
description: test cached snap
grade: devel

parts:
  my-part:
    plugin: nil
"""


class SnapCacheTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        super().make_snapcraft_yaml(yaml_content)
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    def test_snap_cache(self):
        self.useFixture(fixture_setup.FakeTerminal())
        snap_cache = cache.SnapCache(project_name='my-snap-name')
        snap_file = 'my-snap-name_0.1_amd64.snap'

        # create dummy snap
        snap_path = os.path.join(self.path, snap_file)
        open(snap_path, 'a').close()

        # cache snap
        cached_snap_path = snap_cache.cache(snap_file)

        expected_snap_path = os.path.join(
            snap_cache.snap_cache_root,
            file_utils.calculate_sha3_384(snap_path)
        )

        self.assertEqual(
            expected_snap_path,
            cached_snap_path
        )
        self.assertTrue(os.path.isfile(cached_snap_path))

    def test_snap_cache_get_latest(self):
        self.useFixture(fixture_setup.FakeTerminal())

        snap_files = [
            'my-snap-name_0.1_amd64.snap',
            'my-snap-name_0.2_amd64.snap'
        ]

        snap_cache = cache.SnapCache(project_name='my-snap-name')

        # create dummy cached snaps
        for snap in snap_files:
            snap_path = os.path.join(self.path, snap)
            with open(snap_path, 'wb') as f:
                time.sleep(0.01)
                f.write(bytes(os.urandom(1024)))

            last_cached = snap_cache.cache(snap_path)

        latest_snap = snap_cache.get()

        self.assertEqual(last_cached, latest_snap)

    def test_snap_cache_get_by_hash(self):
        self.useFixture(fixture_setup.FakeTerminal())

        snap_files = [
            'my-snap-name_0.1_amd64.snap',
            'my-snap-name_0.2_amd64.snap'
        ]

        snap_cache = cache.SnapCache(project_name='my-snap-name')

        # create dummy cached snaps
        for snap in snap_files:
            snap_path = os.path.join(self.path, snap)
            with open(snap_path, 'wb') as f:
                f.write(bytes(os.urandom(1024)))
            snap_cache.cache(snap_path)

        # get hash of snap
        snap_hash = file_utils.calculate_sha3_384(snap_files[1])

        # get snap by hash
        snap = snap_cache.get(snap_hash=snap_hash)

        self.assertEqual(
            os.path.join(snap_cache.snap_cache_root, snap_hash),
            snap
        )


class SnapCachePruneTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        super().make_snapcraft_yaml(yaml_content)
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    def test_prune_snap_cache(self):
        self.useFixture(fixture_setup.FakeTerminal())
        snap_cache = cache.SnapCache(project_name='my-snap-name')

        snap_files = {
            'my-snap-name_0.1_amd64.snap': '',
            'my-snap-name-0.2_amd64.snap': '',
            'my-snap-name-0.3_amd64.snap': ''
        }

        # create dummy cached snaps and calculate hashes
        for snap in snap_files:
            snap_path = os.path.join(self.path, snap)
            with open(snap_path, 'wb') as f:
                time.sleep(0.01)
                f.write(bytes(os.urandom(1024)))
            snap_files[snap] = file_utils.calculate_sha3_384(snap_path)
            snap_cache.cache(snap_path)

        # confirm expected snap cached
        self.assertEqual(3, len(os.listdir(snap_cache.snap_cache_root)))
        for _, snap_hash in snap_files.items():
            self.assertTrue(
                os.path.isfile(os.path.join(snap_cache.snap_cache_root,
                                            snap_hash)))

        # prune
        keep_hash = snap_files['my-snap-name-0.3_amd64.snap']
        pruned_files = snap_cache.prune(keep_hash=keep_hash)

        self.assertEqual(2, len(pruned_files))
        self.assertIn(
            os.path.join(
                snap_cache.snap_cache_root,
                snap_files['my-snap-name_0.1_amd64.snap']
            ),
            pruned_files
        )
        self.assertNotIn(
            os.path.join(snap_cache.snap_cache_root, keep_hash),
            pruned_files
        )
