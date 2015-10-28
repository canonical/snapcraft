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

import fixtures

from unittest import mock

from snapcraft import help
from snapcraft import tests


class CommonTestCase(tests.TestCase):

    @mock.patch('snapcraft.cmds.list_plugins')
    def test_topic_and_plugin_not_found_lists_plugins(self, mock_list):
        fake_logger = fixtures.FakeLogger()
        self.useFixture(fake_logger)

        class Args:
            topic = 'does-not-exist'
            devel = False

        help.topic(Args())
        self.assertTrue(mock_list.called)
        self.assertEqual(mock_list.call_count, 1)
