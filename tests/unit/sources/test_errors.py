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
from tests import unit


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
        (
            "SnapcraftSourceUnhandledError",
            {
                "exception": errors.SnapcraftSourceUnhandledError,
                "kwargs": {"source": "unknown://source/type"},
                "expected_message": (
                    "Failed to pull source: "
                    "unable to determine source type of 'unknown://source/type'.\n"
                    "Check that the URL is correct or consider specifying "
                    "`source-type` for this part. "
                    "See `snapcraft help sources` for more information."
                ),
            },
        ),
        (
            "SnapcraftSourceInvalidOptionError",
            {
                "exception": errors.SnapcraftSourceInvalidOptionError,
                "kwargs": {"source_type": "test", "option": "foo"},
                "expected_message": (
                    "Failed to pull source: "
                    "'foo' cannot be used with a test source.\n"
                    "See `snapcraft help sources` for more information."
                ),
            },
        ),
        (
            "SnapcraftSourceIncompatibleOptionsError, two options",
            {
                "exception": errors.SnapcraftSourceIncompatibleOptionsError,
                "kwargs": {"source_type": "test", "options": ["foo", "bar"]},
                "expected_message": (
                    "Failed to pull source: "
                    "cannot specify both 'bar' and 'foo' for a test source.\n"
                    "See `snapcraft help sources` for more information."
                ),
            },
        ),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)), Equals(self.expected_message)
        )
