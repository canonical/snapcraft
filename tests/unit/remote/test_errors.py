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
    error = errors.RemoteBuildTimeoutError(
        recovery_command="craftapp remote-build --recover --build-id test-id"
    )

    assert str(error) == (
        "Remote build command timed out.\nBuild may still be running on Launchpad and "
        "can be recovered with 'craftapp remote-build --recover --build-id test-id'."
    )
    assert repr(error) == (
        "RemoteBuildTimeoutError(brief='Remote build command timed out.', "
        'details="Build may still be running on Launchpad and can be recovered with '
        "'craftapp remote-build --recover --build-id test-id'.\")"
    )
    assert error.brief == "Remote build command timed out."
    assert error.details == (
        "Build may still be running on Launchpad and can be recovered with "
        "'craftapp remote-build --recover --build-id test-id'."
    )


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


def test_unsupported_architecture_error():
    """Test UnsupportedArchitectureError."""
    error = errors.UnsupportedArchitectureError(architectures=["amd64", "arm64"])

    assert str(error) == (
        "Architecture not supported by the remote builder.\nThe following "
        "architectures are not supported by the remote builder: ['amd64', 'arm64'].\n"
        "Please remove them from the architecture list and try again."
    )
    assert repr(error) == (
        "UnsupportedArchitectureError(brief='Architecture not supported by the remote "
        "builder.', details=\"The following architectures are not supported by the "
        "remote builder: ['amd64', 'arm64'].\\nPlease remove them from the "
        'architecture list and try again.")'
    )

    assert error.brief == "Architecture not supported by the remote builder."
    assert error.details == (
        "The following architectures are not supported by the remote builder: "
        "['amd64', 'arm64'].\nPlease remove them from the architecture list and "
        "try again."
    )


def test_accept_public_upload_error():
    """Test AcceptPublicUploadError."""
    error = errors.AcceptPublicUploadError()

    assert str(error) == (
        "Cannot upload data to build servers.\nRemote build needs explicit "
        "acknowledgement that data sent to build servers is public.\n"
        "In non-interactive runs, please use the option "
        "`--launchpad-accept-public-upload`."
    )
    assert repr(error) == (
        "AcceptPublicUploadError(brief='Cannot upload data to build servers.', "
        "details='Remote build needs explicit acknowledgement that data sent to build "
        "servers is public.\\n"
        "In non-interactive runs, please use the option "
        "`--launchpad-accept-public-upload`.')"
    )

    assert error.brief == "Cannot upload data to build servers."
    assert error.details == (
        "Remote build needs explicit acknowledgement that data sent to build servers "
        "is public.\n"
        "In non-interactive runs, please use the option "
        "`--launchpad-accept-public-upload`."
    )
