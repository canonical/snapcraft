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

from snapcraft.storeapi import errors


class TestSnapcraftException:
    scenarios = (
        (
            "SnapNotFoundError snap_name",
            dict(
                exception_class=errors.SnapNotFoundError,
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
                exception_class=errors.SnapNotFoundError,
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
                exception_class=errors.SnapNotFoundError,
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
                exception_class=errors.SnapNotFoundError,
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
                exception_class=errors.SnapNotFoundError,
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

    def test_snapcraft_exception_handling(
        self,
        exception_class,
        kwargs,
        expected_brief,
        expected_resolution,
        expected_details,
        expected_docs_url,
        expected_reportable,
    ):
        exception = exception_class(**kwargs)
        assert exception.get_brief() == expected_brief
        assert exception.get_resolution() == expected_resolution
        assert exception.get_details() == expected_details
        assert exception.get_docs_url() == expected_docs_url
        assert exception.get_reportable() == expected_reportable
