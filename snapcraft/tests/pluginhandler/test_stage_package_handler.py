# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import doctest
from unittest import mock

import snapcraft

from snapcraft.internal.pluginhandler.stage_package_grammar import (
    process_grammar
)

from snapcraft.internal.pluginhandler._stage_package_handler import (
    StagePackageHandler
)

from snapcraft import tests


def load_tests(loader, tests, ignore):
    patcher = mock.patch('snapcraft.repo.Ubuntu')

    def _setup(test):
        patcher.start()

    def _teardown(test):
        patcher.stop()

    tests.addTests(doctest.DocTestSuite(
        snapcraft.internal.pluginhandler._stage_package_handler,
        setUp=_setup, tearDown=_teardown))
    return tests


class StagePackageHandlerTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch(
            'snapcraft.internal.pluginhandler.stage_package_grammar.'
            'process_grammar')
        self.process_grammar_mock = patcher.start()
        self.process_grammar_mock.side_effect = process_grammar
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.get_mock = self.ubuntu_mock.return_value.get
        self.unpack_mock = self.ubuntu_mock.return_value.unpack
        self.is_valid_mock = self.ubuntu_mock.return_value.is_valid

        self.cache_dir = os.path.join(os.getcwd(), 'cache')
        self.unpack_dir = os.path.join(os.getcwd(), 'unpack')

    def test_fetch(self):
        handler = StagePackageHandler(['foo'], self.cache_dir)

        handler.fetch()

        self.process_grammar_mock.assert_called_once_with(
            ['foo'], mock.ANY, mock.ANY)
        self.get_mock.assert_called_once_with({'foo'})
        self.unpack_mock.assert_not_called()

    def test_unpack(self):
        handler = StagePackageHandler(['foo'], self.cache_dir)

        handler.unpack(self.unpack_dir)

        self.process_grammar_mock.assert_called_once_with(
            ['foo'], mock.ANY, mock.ANY)
        self.get_mock.assert_not_called()
        self.unpack_mock.assert_called_with(self.unpack_dir)
