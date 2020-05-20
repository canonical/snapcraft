# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from textwrap import dedent

from testtools.matchers import Equals

from snapcraft.storeapi import errors
from tests import unit


class SnapcraftExceptionTests(unit.TestCase):
    scenarios = (
        (
            "SnapNotFoundError snap_name",
            dict(
                exception=errors.SnapNotFoundError,
                kwargs=dict(snap_name="not-found"),
                expected_brief="Snap 'not-found' was not found.",
                expected_resolution="Ensure you have proper access rights for 'not-found'.",
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_id",
            dict(
                exception=errors.SnapNotFoundError,
                kwargs=dict(snap_id="1234567890"),
                expected_brief="Cannot find snap with snap_id '1234567890'.",
                expected_resolution="Ensure you have proper access rights for '1234567890'.",
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_name, channel and arch",
            dict(
                exception=errors.SnapNotFoundError,
                kwargs=dict(
                    snap_name="not-found", channel="default/stable", arch="foo"
                ),
                expected_brief="Snap 'not-found' for architecture 'foo' was not found on channel 'default/stable'.",
                expected_resolution=dedent(
                    """\
                Ensure you have proper access rights for 'not-found'.
                Also ensure the correct channel and architecture was used."""
                ),
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_name and channel",
            dict(
                exception=errors.SnapNotFoundError,
                kwargs=dict(snap_name="not-found", channel="default/stable"),
                expected_brief="Snap 'not-found' was not found on channel 'default/stable'.",
                expected_resolution=dedent(
                    """\
                Ensure you have proper access rights for 'not-found'.
                Also ensure the correct channel was used."""
                ),
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_name and arch",
            dict(
                exception=errors.SnapNotFoundError,
                kwargs=dict(snap_name="not-found", arch="foo"),
                expected_brief="Snap 'not-found' for architecture 'foo' was not found.",
                expected_resolution=dedent(
                    """\
                Ensure you have proper access rights for 'not-found'.
                Also ensure the correct architecture was used."""
                ),
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
    )

    def test_snapcraft_exception_handling(self):
        exception = self.exception(**self.kwargs)
        self.expectThat(exception.get_brief(), Equals(self.expected_brief))
        self.expectThat(exception.get_resolution(), Equals(self.expected_resolution))
        self.expectThat(exception.get_details(), Equals(self.expected_details))
        self.expectThat(exception.get_docs_url(), Equals(self.expected_docs_url))
        self.expectThat(exception.get_reportable(), Equals(self.expected_reportable))
