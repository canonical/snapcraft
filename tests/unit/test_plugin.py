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

import os
import tempfile
import unittest
from unittest import mock

from snapcraft.plugin import Plugin


class TestPlugin(unittest.TestCase):

    def test_is_dirty(self):
        p = Plugin("mock", "mock-part", {}, load_config=False)
        p.statefile = tempfile.NamedTemporaryFile().name
        self.addCleanup(os.remove, p.statefile)
        p.code = mock.Mock()
        # pull once
        p.pull()
        p.code.pull.assert_called()
        # pull again, not dirty no need to pull
        p.code.pull.reset_mock()
        p.pull()
        self.assertFalse(p.code.pull.called)

    def test_collect_snap_files(self):
        p = Plugin("mock", "mock-part", {}, load_config=False)

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        p.installdir = tmpdir + '/install'
        os.makedirs(tmpdir + '/install/1/1a/1b')
        os.makedirs(tmpdir + '/install/2/2a')
        os.makedirs(tmpdir + '/install/3')
        open(tmpdir + '/install/a', mode='w').close()
        open(tmpdir + '/install/b', mode='w').close()
        open(tmpdir + '/install/1/a', mode='w').close()
        open(tmpdir + '/install/3/a', mode='w').close()

        p.stagedir = tmpdir + '/stage'
        os.makedirs(tmpdir + '/stage/1/1a/1b')
        os.makedirs(tmpdir + '/stage/2/2a')
        os.makedirs(tmpdir + '/stage/2/2b')
        os.makedirs(tmpdir + '/stage/3')
        open(tmpdir + '/stage/a', mode='w').close()
        open(tmpdir + '/stage/b', mode='w').close()
        open(tmpdir + '/stage/c', mode='w').close()
        open(tmpdir + '/stage/1/a', mode='w').close()
        open(tmpdir + '/stage/2/2b/a', mode='w').close()
        open(tmpdir + '/stage/3/a', mode='w').close()

        self.assertEqual(p.collect_snap_files([], []), (set(), set()))

        self.assertEqual(p.collect_snap_files(['*'], []), (
            set(['1', '1/1a', '1/1a/1b', '2', '2/2a', '3']),
            set(['a', 'b', '1/a', '3/a'])))

        self.assertEqual(p.collect_snap_files(['*'], ['1']), (
            set(['2', '2/2a', '3']),
            set(['a', 'b', '3/a'])))

        self.assertEqual(p.collect_snap_files(['a'], ['*']), (set(), set()))

        self.assertEqual(p.collect_snap_files(['*'], ['*/*']), (
            set(['1', '2', '3']),
            set(['a', 'b'])))

        self.assertEqual(p.collect_snap_files(['1', '2'], ['*/a']), (
            set(['1', '1/1a', '1/1a/1b', '2', '2/2a']),
            set()))
