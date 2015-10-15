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
                return '''{{{part-in-wiki:
  plugin: go
  source: http://somesource
}}}'''

        patcher = unittest.mock.patch('requests.get')
        self.mock_requests = patcher.start()
        self.mock_requests.return_value = Content()
        self.addCleanup(patcher.stop)

        self.w = snapcraft.wiki.Wiki()

    def tearDown(self):
        self.mock_requests.assert_called_once_with(
            'https://wiki.ubuntu.com/Snappy/Parts',
            params={'action': 'raw'})

    def test_get_part(self):
        self.assertEqual(self.w.get_part('part-in-wiki'), {
            'plugin': 'go', 'source': 'http://somesource'})
        self.assertEqual(self.w.get_part('part-not-in-wiki'), None)

    def test_compose_part_with_properties_from_the_wiki(self):
        properties = self.w.compose(
            'part-in-wiki', {'source': '.', 'another': 'different'})
        expected_properties = {
            'plugin': 'go', 'source': '.', 'another': 'different'}

        self.assertEqual(properties, expected_properties)

    def test_compose_part_with_properties_from_the_wiki_using_source(self):
        properties = self.w.compose(
            'part-in-wiki', {'another': 'different'})
        expected_properties = {
            'plugin': 'go', 'source': 'http://somesource',
            'another': 'different'}

        self.assertEqual(properties, expected_properties)

    def test_compose_part_for_part_not_in_wiki_raises_exception(self):
        with self.assertRaises(KeyError) as raised:
            self.w.compose('part-not-in-wiki',
                           {'source': '.', 'another': 'different'})
        self.assertEqual(raised.exception.args, ('part-not-in-wiki',))
