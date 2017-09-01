# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import glob
import logging
import os
from unittest import mock

import fixtures
from testtools.matchers import Equals

import snapcraft
from snapcraft import file_utils
from snapcraft.internal import cache
from snapcraft.tests import fixture_setup
from snapcraft.tests.commands import CommandBaseTestCase


class SnapCacheBaseTestCase(CommandBaseTestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        patcher = mock.patch('snapcraft.internal.lifecycle.ProgressBar')
        patcher.start()
        self.addCleanup(patcher.stop)

        self.deb_arch = snapcraft.ProjectOptions().deb_arch

        self.snap_path = os.path.join(
            os.path.dirname(snapcraft.tests.__file__), 'data',
            'test-snap.snap')


class SnapCacheTestCase(SnapCacheBaseTestCase):

    def test_snap_cache(self):
        self.useFixture(fixture_setup.FakeTerminal())

        # cache snap
        snap_cache = cache.SnapCache(project_name='cache-test')
        cached_snap_path = snap_cache.cache(snap_filename=self.snap_path)

        expected_snap_path = os.path.join(
            snap_cache.snap_cache_root,
            'amd64',
            file_utils.calculate_sha3_384(self.snap_path)
        )

        self.assertThat(
            cached_snap_path,
            Equals(expected_snap_path))
        self.assertTrue(os.path.isfile(cached_snap_path))

    def test_snap_cache_get_latest(self):
        self.useFixture(fixture_setup.FakeTerminal())

        # Create snaps
        with open(os.path.join(self.path, 'snapcraft.yaml'), 'w') as f:
            f.write("""name: my-snap-name
summary: test cached snap
description: test cached snap
architectures: ['{}']
confinement: devmode
grade: devel
version: '0.1'

parts:
    my-part:
      plugin: nil
""".format(self.deb_arch))
        result = self.run_command(['snap'])
        self.assertThat(result.exit_code, Equals(0))
        snap_file = glob.glob('*0.1*.snap')[0]

        snap_cache = cache.SnapCache(project_name='my-snap-name')
        snap_cache.cache(snap_filename=snap_file)

        with open(os.path.join(self.path, 'snapcraft.yaml'), 'w') as f:
            f.write("""name: my-snap-name
summary: test cached snap
description: test cached snap
architectures: ['{}']
confinement: devmode
grade: devel
version: '0.2'

parts:
    my-part:
      plugin: nil
""".format(self.deb_arch))
        result = self.run_command(['snap'])
        self.assertThat(result.exit_code, Equals(0))
        snap_file_latest = glob.glob('*0.2*.snap')[0]

        snap_cache.cache(snap_filename=snap_file_latest)
        latest_hash = file_utils.calculate_sha3_384(snap_file_latest)

        # get latest
        latest_snap = snap_cache.get(deb_arch=self.deb_arch)

        expected_snap_path = os.path.join(
            snap_cache.snap_cache_root,
            self.deb_arch,
            latest_hash
        )

        self.assertThat(latest_snap, Equals(expected_snap_path))

    def test_snap_cache_get_by_hash(self):
        self.useFixture(fixture_setup.FakeTerminal())

        snap_cache = cache.SnapCache(project_name='my-snap-name')
        snap_cache.cache(snap_filename=self.snap_path)

        # get hash of snap
        snap_hash = file_utils.calculate_sha3_384(self.snap_path)

        # get snap by hash
        snap = snap_cache.get(deb_arch='amd64', snap_hash=snap_hash)

        self.assertThat(
            snap, Equals(
                os.path.join(snap_cache.snap_cache_root, 'amd64', snap_hash))
        )


class SnapCachePruneTestCase(SnapCacheBaseTestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    def test_prune_snap_cache(self):
        self.useFixture(fixture_setup.FakeTerminal())

        # Create snaps
        with open(os.path.join(self.path, 'snapcraft.yaml'), 'w') as f:
            f.write("""name: my-snap-name
summary: test cached snap
description: test cached snap
architectures: ['{}']
confinement: devmode
grade: devel
version: '0.1'

parts:
    my-part:
      plugin: nil
""".format(self.deb_arch))
        result = self.run_command(['snap'])
        self.assertThat(result.exit_code, Equals(0))
        snap_file = glob.glob('*0.1_*.snap')[0]

        snap_cache = cache.SnapCache(project_name='my-snap-name')
        snap_file_path = snap_cache.cache(snap_filename=snap_file)
        _, snap_file_hash = os.path.split(snap_file_path)

        with open(os.path.join(self.path, 'snapcraft.yaml'), 'w') as f:
            f.write("""name: my-snap-name
summary: test cached snap
description: test cached snap
architectures: ['{}']
confinement: devmode
grade: devel
version: '0.2'

parts:
    my-part:
      plugin: nil
""".format(self.deb_arch))
        result = self.run_command(['snap'])
        self.assertThat(result.exit_code, Equals(0))
        snap_file_2 = glob.glob('*0.2*.snap')[0]
        snap_file_2_path = snap_cache.cache(snap_filename=snap_file_2)
        snap_file_2_dir, snap_file_2_hash = os.path.split(snap_file_2_path)

        # confirm expected snap cached
        self.assertThat(len(os.listdir(snap_file_2_dir)), Equals(2))

        # prune
        pruned_files = snap_cache.prune(deb_arch=self.deb_arch,
                                        keep_hash=snap_file_2_hash)

        self.assertThat(len(pruned_files), Equals(1))
        self.assertIn(
            os.path.join(
                snap_cache.snap_cache_root,
                self.deb_arch,
                snap_file_hash
            ),
            pruned_files
        )
        self.assertNotIn(
            os.path.join(snap_cache.snap_cache_root, snap_file_2_hash),
            pruned_files
        )
