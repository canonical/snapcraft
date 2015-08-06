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

import fixtures

from snapcraft import tests
from snapcraft.plugins import ubuntu


class UbuntuPluginTestCase(tests.TestCase):

    def test_get_all_dep_packages_with_unrecognized_package(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        test_options = type('obj', (object,), {'packages': False, 'recommends': False})
        plugin = ubuntu.UbuntuPlugin('test_plugin', test_options)

        with self.assertRaises(SystemExit) as raised:
            plugin.get_all_dep_packages(['test_package'])

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        self.assertEqual(
            "Package 'test_package' not recognized\n", fake_logger.output)
