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

import unittest.mock

import snapcraft.wiki

from snapcraft.tests import TestCase


class TestYaml(TestCase):

    def setUp(self):
        super().setUp()

        class Content:

            @property
            def text(self):
                return '''{{{part1:
  plugin: go
}}}'''

        patcher = unittest.mock.patch('requests.get')
        self.mock_requests = patcher.start()
        self.mock_requests.return_value = Content()
        self.addCleanup(patcher.stop)

    def test_get_part(self):
        w = snapcraft.wiki.Wiki()

        self.assertEqual(w.get_part('part1'), {'plugin': 'go'})
        self.assertEqual(w.get_part('part2'), None)

        self.mock_requests.assert_called_once_with(
            'https://wiki.ubuntu.com/Snappy/Parts',
            params={'action': 'raw'})
