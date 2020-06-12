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


class TestSnapcraftException:

    scenarios = (
        (
            "LaunchpadGitPushError",
            {
                "exception_class": errors.LaunchpadGitPushError,
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
                "exception_class": errors.LaunchpadHttpsError,
                "kwargs": dict(),
                "expected_brief": "Failed to connect to Launchpad API service.",
                "expected_resolution": "Verify connectivity to https://api.launchpad.net and retry build.",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
    )

    def test_snapcraft_exception_handling(
        self,
        exception_class,
        expected_brief,
        expected_details,
        expected_docs_url,
        expected_reportable,
        expected_resolution,
        kwargs,
    ):
        exception = exception_class(**kwargs)

        assert expected_brief == exception.get_brief()
        assert expected_resolution == exception.get_resolution()
        assert expected_details == exception.get_details()
        assert expected_docs_url == exception.get_docs_url()
        assert expected_reportable == exception.get_reportable()
