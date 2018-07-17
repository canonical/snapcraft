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

from snapcraft.internal.repo import errors
from tests import unit


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
        (
            "SnapdConnectionError",
            {
                "exception": errors.SnapdConnectionError,
                "kwargs": {"snap_name": "test", "url": "url"},
                "expected_message": (
                    "Failed to get information for snap 'test': "
                    "could not connect to 'url'."
                ),
            },
        ),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)), Equals(self.expected_message)
        )
