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

from snapcraft.cli._errors import exception_catcher
import snapcraft.internal.errors

import testtools
from unittest import mock

from snapcraft import tests


class TestSnapcraftError(snapcraft.internal.errors.SnapcraftError):

    fmt = '{message}'

    def __init__(self, message):
        super().__init__(message=message)

    @property
    def exit_code(self):
        return 123


class ErrorsTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('sys.exit')
        self.exit_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.cli._errors.echo.error')
        self.error_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_catcher_raises_non_snapcraft_exceptions(self):
        with testtools.ExpectedException(RuntimeError, 'not a SnapcraftError'):
            with exception_catcher(False):
                raise RuntimeError('not a SnapcraftError')

        with testtools.ExpectedException(RuntimeError, 'not a SnapcraftError'):
            with exception_catcher(True):
                raise RuntimeError('not a SnapcraftError')

        self.error_mock.assert_not_called
        self.exit_mock.assert_not_called

    def test_catcher_catches_snapcraft_exceptions_no_debug(self):
        try:
            with exception_catcher(False):
                raise TestSnapcraftError('is a SnapcraftError')
        except Exception:
            self.fail('Exception unexpectedly raised')

        self.error_mock.assert_called_once_with('is a SnapcraftError')
        self.exit_mock.assert_called_once_with(123)

    def test_catcher_raises_snapcraft_exceptions_with_debug(self):
        with testtools.ExpectedException(
                TestSnapcraftError, 'is a SnapcraftError'):
            with exception_catcher(True):
                raise TestSnapcraftError('is a SnapcraftError')

        self.error_mock.assert_not_called
        self.exit_mock.assert_not_called
