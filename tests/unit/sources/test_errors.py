# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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
            "SnapcraftSourceNotFoundError",
            {
                "exception": errors.SnapcraftSourceNotFoundError,
                "kwargs": {"source": "file-not-found.tar.gz"},
                "expected_message": (
                    "Failed to pull source: 'file-not-found.tar.gz'.\n"
                    "Please ensure the source path is correct and that it is accessible.\n"
                    "See `snapcraft help sources` for more information."
                ),
            },
        ),
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
        (
            "InvalidSnapError",
            {
                "exception": errors.InvalidSnapError,
                "kwargs": {},
                "expected_message": (
                    "The snap file does not contain valid data. "
                    "Ensure the source lists a proper snap file"
                ),
            },
        ),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)), Equals(self.expected_message)
        )


class SnapcraftExceptionTests(unit.TestCase):

    scenarios = (
        (
            "GitCommandError",
            {
                "exception": errors.GitCommandError,
                "kwargs": dict(
                    command=["echo", "hi", "$foo"],
                    output="some error output",
                    exit_code=0,
                ),
                "expected_brief": "Failed to execute git command: echo hi '$foo'",
                "expected_resolution": "Consider checking your git configuration for settings which may cause issues.",
                "expected_details": "Command failed with exit code 0 and output:\nsome error output",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
    )

    def test_snapcraft_exception_handling(self):
        exception = self.exception(**self.kwargs)
        self.assertEquals(self.expected_brief, exception.get_brief())
        self.assertEquals(self.expected_resolution, exception.get_resolution())
        self.assertEquals(self.expected_details, exception.get_details())
        self.assertEquals(self.expected_docs_url, exception.get_docs_url())
        self.assertEquals(self.expected_reportable, exception.get_reportable())
