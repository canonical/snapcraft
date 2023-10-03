# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019, 2023 Canonical Ltd
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

from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional
from unittest.mock import ANY, MagicMock, Mock, call, patch

import pytest

from snapcraft.remote import LaunchpadClient, errors


class FakeLaunchpadObject:
    """Mimic behavior of many launchpad objects."""

    def __init__(self):
        pass

    def __setitem__(self, key, value):
        self.__setattr__(key, value)

    def __getitem__(self, key):
        return self.__getattribute__(key)


class BuildImpl(FakeLaunchpadObject):
    """Fake build implementation."""

    def __init__(self, fake_arch="i386"):
        self._fake_arch = fake_arch
        self.getFileUrls_mock = Mock(
            return_value=[f"url_for/snap_file_{self._fake_arch}.snap"]
        )

    def getFileUrls(self, *args, **kw):
        return self.getFileUrls_mock(*args, **kw)


class SnapBuildEntryImpl(FakeLaunchpadObject):
    """Fake snap build entry."""

    def __init__(
        self,
        arch_tag="",
        buildstate="",
        self_link="",
        build_log_url: Optional[str] = "",
    ):
        self.arch_tag = arch_tag
        self.buildstate = buildstate
        self.self_link = self_link
        self.build_log_url = build_log_url


class SnapBuildsImpl(FakeLaunchpadObject):
    """Fake snap builds."""

    def __init__(self):
        self.entries = [
            SnapBuildEntryImpl(
                arch_tag="i386",
                buildstate="Successfully built",
                self_link="http://build_self_link_1",
                build_log_url="url_for/build_log_file_1",
            ),
            SnapBuildEntryImpl(
                arch_tag="amd64",
                buildstate="Failed to build",
                self_link="http://build_self_link_2",
                build_log_url="url_for/build_log_file_2",
            ),
            SnapBuildEntryImpl(
                arch_tag="arm64",
                buildstate="Failed to build",
                self_link="http://build_self_link_2",
                build_log_url=None,
            ),
        ]


class SnapBuildReqImpl(FakeLaunchpadObject):
    """Fake snap build requests."""

    def __init__(
        self,
        status="Completed",
        error_message="",
        self_link="http://request_self_link/1234",
        builds_collection_link="http://builds_collection_link",
    ):
        self.status = status
        self.error_message = error_message
        self.self_link = self_link
        self.builds_collection_link = builds_collection_link
        self.builds = SnapBuildsImpl()

    def lp_refresh(self):
        pass


class SnapImpl(FakeLaunchpadObject):
    """Fake snap."""

    def __init__(self, builds_collection_link="http://builds_collection_link"):
        self._req = SnapBuildReqImpl()
        self.lp_delete_mock = Mock()
        self.requestBuilds_mock = Mock(return_value=self._req)
        self.builds_collection_link = builds_collection_link

    def lp_delete(self, *args, **kw):
        return self.lp_delete_mock(*args, **kw)

    def requestBuilds(self, *args, **kw):
        return self.requestBuilds_mock(*args, **kw)


class SnapsImpl(FakeLaunchpadObject):
    """Fake snaps."""

    def __init__(self):
        self._snap = SnapImpl()
        self.getByName_mock = Mock(return_value=self._snap)
        self.new_mock = Mock(return_value=self._snap)

    def getByName(self, *args, **kw):
        return self.getByName_mock(*args, **kw)

    def new(self, *args, **kw):
        return self.new_mock(*args, **kw)


class GitImpl(FakeLaunchpadObject):
    """Fake git."""

    def __init__(self):
        self.issueAccessToken_mock = Mock(return_value="access-token")
        self.lp_delete_mock = Mock()

    def issueAccessToken(self, *args, **kw):
        return self.issueAccessToken_mock(*args, **kw)

    def lp_delete(self, *args, **kw):
        return self.lp_delete_mock(*args, **kw)


class GitRepositoriesImpl(FakeLaunchpadObject):
    """Fake git repositories."""

    def __init__(self):
        self._git = GitImpl()
        self.new_mock = Mock(return_value=self._git)
        self.getByPath_mock = Mock(return_value=self._git)

    def getByPath(self, *args, **kw):
        return self.getByPath_mock(*args, **kw)

    def new(self, *args, **kw):
        return self.new_mock(*args, **kw)


class DistImpl(FakeLaunchpadObject):
    """Fake distributions."""

    def __init__(self):
        self.main_archive = "main_archive"


class MeImpl(FakeLaunchpadObject):
    """Fake 'me' object."""

    def __init__(self):
        self.name = "user"


class LaunchpadImpl(FakeLaunchpadObject):
    """Fake implementation of the Launchpad object."""

    def __init__(self):
        self._login_mock = Mock()
        self._load_mock = Mock()
        self._rbi = SnapBuildReqImpl()

        self.git_repositories = GitRepositoriesImpl()
        self.snaps = SnapsImpl()
        self.people = {"user": "/~user"}
        self.distributions = {"ubuntu": DistImpl()}
        self.rbi = self._rbi
        self.me = MeImpl()

    def load(self, url: str, *args, **kw):
        self._load_mock(url, *args, **kw)
        if "/+build-request/" in url:
            return self._rbi
        if "http://build_self_link_1" in url:
            return BuildImpl(fake_arch="i386")
        if "http://build_self_link_2" in url:
            return BuildImpl(fake_arch="amd64")
        return self._rbi.builds


@pytest.fixture()
def mock_git_repo(mocker):
    """Returns a mocked GitRepo."""
    return mocker.patch("snapcraft.remote.launchpad.GitRepo")


@pytest.fixture()
def mock_login_with(mocker):
    """Mock for launchpadlib's `login_with()`."""
    lp = LaunchpadImpl()
    return mocker.patch("launchpadlib.launchpad.Launchpad.login_with", return_value=lp)


@pytest.fixture()
def launchpad_client(mock_login_with):
    """Returns a LaunchpadClient object."""
    return LaunchpadClient(
        app_name="test-app",
        build_id="id",
        project_name="test-project",
        architectures=[],
    )


def test_login(mock_login_with):
    lpc = LaunchpadClient(
        app_name="test-app",
        build_id="id",
        project_name="test-project",
        architectures=[],
    )

    assert lpc.user == "user"

    assert mock_login_with.called_with(
        "test-app remote-build",
        "production",
        ANY,
        credentials_file=ANY,
        version="devel",
    )


@pytest.mark.parametrize("error", [ConnectionRefusedError, TimeoutError])
def test_login_connection_issues(error, mock_login_with):
    mock_login_with.side_effect = error

    with pytest.raises(errors.LaunchpadHttpsError):
        LaunchpadClient(
            app_name="test-app",
            build_id="id",
            project_name="test-project",
            architectures=[],
        )

    mock_login_with.assert_called()


def test_load_connection_refused(launchpad_client, mock_login_with):
    """ConnectionRefusedError should surface."""
    launchpad_client._lp._load_mock.side_effect = ConnectionRefusedError

    with pytest.raises(ConnectionRefusedError):
        launchpad_client._lp_load_url("foo")

    mock_login_with.assert_called()


def test_load_connection_reset_once(launchpad_client, mock_login_with):
    """Load URL should work OK after single connection reset."""
    launchpad_client._lp._load_mock.side_effect = [ConnectionResetError, None]
    launchpad_client._lp_load_url(url="foo")

    mock_login_with.assert_called()


def test_load_connection_reset_twice(launchpad_client, mock_login_with):
    """Load URL should fail with two connection resets."""
    launchpad_client._lp._load_mock.side_effect = [
        ConnectionResetError,
        ConnectionResetError,
    ]

    with pytest.raises(ConnectionResetError):
        launchpad_client._lp_load_url("foo")

    mock_login_with.assert_called()


def test_create_snap(launchpad_client):
    launchpad_client._create_snap()
    launchpad_client._lp.snaps.new_mock.assert_called_with(
        auto_build=False,
        auto_build_archive="/ubuntu/+archive/primary",
        auto_build_pocket="Updates",
        git_path="main",
        git_repository_url="https://user@git.launchpad.net/~user/+git/id/",
        name="id",
        owner="/~user",
    )


def test_create_snap_with_archs(launchpad_client):
    launchpad_client.architectures = ["arch1", "arch2"]
    launchpad_client._create_snap()
    launchpad_client._lp.snaps.new_mock.assert_called_with(
        auto_build=False,
        auto_build_archive="/ubuntu/+archive/primary",
        auto_build_pocket="Updates",
        git_path="main",
        git_repository_url="https://user@git.launchpad.net/~user/+git/id/",
        name="id",
        owner="/~user",
        processors=["/+processors/arch1", "/+processors/arch2"],
    )


def test_delete_snap(launchpad_client):
    launchpad_client._delete_snap()
    launchpad_client._lp.snaps.getByName_mock.assert_called_with(
        name="id", owner="/~user"
    )


def test_start_build(launchpad_client):
    launchpad_client.start_build()


def test_start_build_error(mocker, launchpad_client):
    mocker.patch(
        "tests.unit.remote.test_launchpad.SnapImpl.requestBuilds",
        return_value=SnapBuildReqImpl(
            status="Failed", error_message="snapcraft.yaml not found..."
        ),
    )
    with pytest.raises(errors.RemoteBuildError) as raised:
        launchpad_client.start_build()

    assert str(raised.value) == "snapcraft.yaml not found..."


def test_start_build_error_timeout(mocker, launchpad_client):
    mocker.patch(
        "tests.unit.remote.test_launchpad.SnapImpl.requestBuilds",
        return_value=SnapBuildReqImpl(status="Pending", error_message=""),
    )
    mocker.patch("time.time", return_value=500)
    launchpad_client._deadline = 499

    with pytest.raises(errors.RemoteBuildTimeoutError) as raised:
        launchpad_client.start_build()

    assert str(raised.value) == "Remote build timed out."


def test_issue_build_request_defaults(launchpad_client):
    fake_snap = MagicMock()

    launchpad_client._issue_build_request(fake_snap)

    assert fake_snap.mock_calls == [
        call.requestBuilds(
            archive="main_archive",
            pocket="Updates",
        )
    ]


@patch("snapcraft.remote.LaunchpadClient._download_file")
def test_monitor_build(mock_download_file, new_dir, launchpad_client):
    Path("test-project_i386.txt", encoding="utf-8").touch()
    Path("test-project_i386.1.txt", encoding="utf-8").touch()

    launchpad_client.start_build()
    launchpad_client.monitor_build(interval=0)

    assert mock_download_file.mock_calls == [
        call(url="url_for/snap_file_i386.snap", dst="snap_file_i386.snap"),
        call(
            url="url_for/build_log_file_1", dst="test-project_i386.2.txt", gunzip=True
        ),
        call(url="url_for/snap_file_amd64.snap", dst="snap_file_amd64.snap"),
        call(url="url_for/build_log_file_2", dst="test-project_amd64.txt", gunzip=True),
        call(url="url_for/snap_file_amd64.snap", dst="snap_file_amd64.snap"),
    ]


@patch("snapcraft.remote.LaunchpadClient._download_file")
@patch("logging.Logger.error")
def test_monitor_build_error(mock_log, mock_download_file, mocker, launchpad_client):
    mocker.patch(
        "tests.unit.remote.test_launchpad.BuildImpl.getFileUrls", return_value=[]
    )
    launchpad_client.start_build()
    launchpad_client.monitor_build(interval=0)

    assert mock_download_file.mock_calls == [
        call(url="url_for/build_log_file_1", dst="test-project_i386.txt", gunzip=True),
        call(url="url_for/build_log_file_2", dst="test-project_amd64.txt", gunzip=True),
    ]

    assert mock_log.mock_calls == [
        call("Snap file not available for arch %r.", "i386"),
        call("Snap file not available for arch %r.", "amd64"),
        call("Build failed for arch %r.", "amd64"),
        call("Snap file not available for arch %r.", "arm64"),
        call("Build failed for arch %r.", "arm64"),
    ]


def test_monitor_build_error_timeout(mocker, launchpad_client):
    mocker.patch("snapcraft.remote.LaunchpadClient._download_file")
    mocker.patch("time.time", return_value=500)
    launchpad_client._deadline = 499
    launchpad_client.start_build()

    with pytest.raises(errors.RemoteBuildTimeoutError) as raised:
        launchpad_client.monitor_build(interval=0)

    assert str(raised.value) == "Remote build timed out."


def test_get_build_status(launchpad_client):
    launchpad_client.start_build()
    build_status = launchpad_client.get_build_status()

    assert build_status == {
        "amd64": "Failed to build",
        "arm64": "Failed to build",
        "i386": "Successfully built",
    }


def test_git_repository_create_clean(mock_git_repo, launchpad_client):
    mock_git_repo.return_value.is_clean.return_value = True
    launchpad_client._gitify_repository(Path())

    assert mock_git_repo.mock_calls == [
        call(Path()),
        call().is_clean(),
    ]


def test_git_repository_create_dirty(mock_git_repo, launchpad_client):
    mock_git_repo.return_value.is_clean.return_value = False

    launchpad_client._gitify_repository(Path())

    assert mock_git_repo.mock_calls == [
        call(Path()),
        call().is_clean(),
        call().add_all(),
        call().commit(),
    ]


def test_push_source_tree(new_dir, mock_git_repo, launchpad_client):
    now = datetime.now(timezone.utc)

    with patch("snapcraft.remote.launchpad.datetime") as mock_datetime:
        mock_datetime.now = lambda tz: now
        launchpad_client.push_source_tree(Path())

    launchpad_client._lp.git_repositories._git.issueAccessToken_mock.assert_called_once_with(
        description="test-app remote-build for id",
        scopes=["repository:push"],
        date_expires=(now + timedelta(minutes=1)).isoformat(),
    )

    mock_git_repo.assert_has_calls(
        [
            call.push_url(
                "https://user:access-token@git.launchpad.net/~user/+git/id/",
                "main",
                "HEAD",
                "access-token",
            )
        ]
    )


def test_push_source_tree_error(new_dir, mock_git_repo, launchpad_client):
    mock_git_repo.push_url.side_effect = errors.GitError("test error")

    with pytest.raises(errors.GitError):
        launchpad_client.push_source_tree(Path())
