# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

from snapcraft.remote import errors


def test_git_error():
    """Test GitError."""
    error = errors.GitError("Error details.")

    assert str(error) == "Git operation failed.\nError details."
    assert (
        repr(error)
        == "GitError(brief='Git operation failed.', details='Error details.')"
    )
    assert error.brief == "Git operation failed."
    assert error.details == "Error details."


def test_remote_build_timeout_error():
    """Test RemoteBuildTimeoutError."""
    error = errors.RemoteBuildTimeoutError()

    assert str(error) == "Remote build timed out."
    assert (
        repr(error)
        == "RemoteBuildTimeoutError(brief='Remote build timed out.', details=None)"
    )
    assert error.brief == "Remote build timed out."


def test_launchpad_https_error():
    """Test LaunchpadHttpsError."""
    error = errors.LaunchpadHttpsError()

    assert str(error) == (
        "Failed to connect to Launchpad API service.\n"
        "Verify connectivity to https://api.launchpad.net and retry build."
    )
    assert repr(error) == (
        "LaunchpadHttpsError(brief='Failed to connect to Launchpad API service.', "
        "details='Verify connectivity to https://api.launchpad.net and retry build.')"
    )

    assert error.brief == "Failed to connect to Launchpad API service."
    assert error.details == (
        "Verify connectivity to https://api.launchpad.net and retry build."
    )
