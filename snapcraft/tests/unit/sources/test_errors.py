# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
from testtools.matchers import Equals

from snapcraft.internal.sources import errors
from snapcraft.tests import unit


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
        ('SnapcraftSourceUnhandledError', {
            'exception': errors.SnapcraftSourceUnhandledError,
            'kwargs': {'source': 'unknown://source/type'},
            'expected_message': (
                'Failed to pick an appropriate source handler:\n'
                'unknown://source/type does not look like a known format.\n'
                'Check that the URL is correct and consider adding '
                '"source-type".'
                )}),
        ('SnapcraftSourceNotADirectoryError', {
            'exception': errors.SnapcraftSourceNotADirectoryError,
            'kwargs': {'source': 'foo'},
            'expected_message': (
                "Failed to pick an appropriate source handler:\n"
                "'foo' looks like a local filename but there is no directory "
                "with that name that could be used as a source."
                )}),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)),
            Equals(self.expected_message))
