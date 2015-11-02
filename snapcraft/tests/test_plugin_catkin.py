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
import os
import os.path

from unittest import mock

from snapcraft import tests
from snapcraft.plugins import catkin


class _IOError(IOError):
    errno = os.errno.EACCES


class CatkinTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class props:
            catkin_packages = ['my_package']

        self.properties = props()

    def test_log_warning_when_unable_to_find_a_catkin_package(self):
        fake_logger = fixtures.FakeLogger()
        self.useFixture(fake_logger)

        plugin = catkin.CatkinPlugin('test-part', self.properties)
        plugin._find_package_deps()

        self.assertEqual('Unable to find packages.xml for "my_package"\n',
                         fake_logger.output)

    @mock.patch('snapcraft.plugins.catkin.open', create=True)
    def test_exception_raised_when_package_definition_cannot_be_read(
            self, mock_open):
        mock_open.side_effect = _IOError()
        plugin = catkin.CatkinPlugin('test-part', self.properties)
        with self.assertRaises(IOError) as raised:
            plugin._find_package_deps()

        self.assertEqual(raised.exception.errno, os.errno.EACCES)
        xml_to_open = os.path.join(
            os.path.abspath(os.curdir), 'parts', 'test-part', 'build', 'src',
            'my_package', 'package.xml')
        mock_open.assert_called_once_with(xml_to_open, 'r')
