# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from snapcraft.internal.remote_build import errors
from tests import unit
from testtools.matchers import Equals


class SnapcraftExceptionTests(unit.TestCase):

    scenarios = (
        (
            "LaunchpadGitPushError",
            {
                "exception": errors.LaunchpadGitPushError,
                "kwargs": dict(command="foo", exit_code=4),
                "expected_brief": "Failed to push sources to Launchpad.",
                "expected_resolution": "Verify connectivity to https://git.launchpad.net and retry build.",
                "expected_details": "Command 'foo' failed with exit code 4.",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "LaunchpadHttpsError",
            {
                "exception": errors.LaunchpadHttpsError,
                "kwargs": None,
                "expected_brief": "Failed to connect to Launchpad API service.",
                "expected_resolution": "Verify connectivity to https://api.launchpad.net and retry build.",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
    )

    def test_snapcraft_exception_handling(self):
        if self.kwargs:
            exception = self.exception(**self.kwargs)
        else:
            exception = self.exception()
        self.assertThat(exception.get_brief(), Equals(self.expected_brief))
        self.assertThat(exception.get_resolution(), Equals(self.expected_resolution))
        self.assertThat(exception.get_details(), Equals(self.expected_details))
        self.assertThat(exception.get_docs_url(), Equals(self.expected_docs_url))
        self.assertThat(exception.get_reportable(), Equals(self.expected_reportable))
