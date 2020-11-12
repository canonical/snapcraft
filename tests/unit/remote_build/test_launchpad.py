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

import textwrap
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals

import snapcraft
from snapcraft.internal.remote_build import LaunchpadClient, errors
from snapcraft.internal.sources._git import Git
from snapcraft.internal.sources.errors import SnapcraftPullError
from tests import unit

from . import TestDir


class FakeLaunchpadObject:
    """Mimic behavior of many launchpad objects."""

    def __init__(self):
        pass

    def __setitem__(self, key, value):
        self.__setattr__(key, value)

    def __getitem__(self, key):
        return self.__getattribute__(key)


class BuildImpl(FakeLaunchpadObject):
    def __init__(self, fake_arch="i386"):
        self._fake_arch = fake_arch
        self.getFileUrls_mock = mock.Mock(
            return_value=[f"url_for/snap_file_{self._fake_arch}.snap"]
        )

    def getFileUrls(self, *args, **kw):
        return self.getFileUrls_mock(*args, **kw)


class SnapBuildEntryImpl(FakeLaunchpadObject):
    def __init__(self, arch_tag="", buildstate="", self_link="", build_log_url=""):
        self.arch_tag = arch_tag
        self.buildstate = buildstate
        self.self_link = self_link
        self.build_log_url = build_log_url


class SnapBuildsImpl(FakeLaunchpadObject):
    def __init__(
        self,
        entries=[
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
        ],
    ):
        self.entries = entries


class SnapBuildReqImpl(FakeLaunchpadObject):
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
    def __init__(self, builds_collection_link="http://builds_collection_link"):
        self._req = SnapBuildReqImpl()
        self.lp_delete_mock = mock.Mock()
        self.requestBuilds_mock = mock.Mock(return_value=self._req)
        self.builds_collection_link = builds_collection_link

    def lp_delete(self, *args, **kw):
        return self.lp_delete_mock(*args, **kw)

    def requestBuilds(self, *args, **kw):
        return self.requestBuilds_mock(*args, **kw)


class SnapsImpl(FakeLaunchpadObject):
    def __init__(self):
        self._snap = SnapImpl()
        self.getByName_mock = mock.Mock(return_value=self._snap)
        self.new_mock = mock.Mock(return_value=self._snap)

    def getByName(self, *args, **kw):
        return self.getByName_mock(*args, **kw)

    def new(self, *args, **kw):
        return self.new_mock(*args, **kw)


class GitImpl(FakeLaunchpadObject):
    def __init__(self):
        self.issueAccessToken_mock = mock.Mock(return_value="access-token")
        self.lp_delete_mock = mock.Mock()

    def issueAccessToken(self, *args, **kw):
        return self.issueAccessToken_mock(*args, **kw)

    def lp_delete(self, *args, **kw):
        return self.lp_delete_mock(*args, **kw)


class GitRepositoriesImpl(FakeLaunchpadObject):
    def __init__(self):
        self._git = GitImpl()
        self.new_mock = mock.Mock(return_value=self._git)
        self.getByPath_mock = mock.Mock(return_value=self._git)

    def getByPath(self, *args, **kw):
        return self.getByPath_mock(*args, **kw)

    def new(self, *args, **kw):
        return self.new_mock(*args, **kw)


class DistImpl(FakeLaunchpadObject):
    def __init__(self):
        self.main_archive = "main_archive"


class MeImpl(FakeLaunchpadObject):
    def __init__(self):
        self.name = "user"


class LaunchpadImpl(FakeLaunchpadObject):
    def __init__(self):
        self._login_mock = mock.Mock()
        self._load_mock = mock.Mock()
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
        elif "http://build_self_link_1" in url:
            return BuildImpl(fake_arch="i386")
        elif "http://build_self_link_2" in url:
            return BuildImpl(fake_arch="amd64")
        else:
            return self._rbi.builds


class LaunchpadTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self._project = self._make_snapcraft_project()
        self.lp = LaunchpadImpl()
        self.fake_login_with = fixtures.MockPatch(
            "launchpadlib.launchpad.Launchpad.login_with", return_value=self.lp
        )
        self.useFixture(self.fake_login_with)

        self.mock_git_class = mock.Mock(spec=Git)
        self.mock_git_class.check_command_installed.return_value = True

        self.lpc = LaunchpadClient(
            project=self._project,
            build_id="id",
            architectures=[],
            git_class=self.mock_git_class,
            running_snapcraft_version="100.1234",
        )

    def test_login(self):
        self.assertThat(self.lpc.user, Equals("user"))
        self.fake_login_with.mock.assert_called_with(
            "snapcraft remote-build {}".format(snapcraft.__version__),
            "production",
            mock.ANY,
            credentials_file=mock.ANY,
            version="devel",
        )

    def test_login_connection_issues(self):
        self.fake_login_with.mock.side_effect = ConnectionRefusedError()
        self.assertRaises(errors.LaunchpadHttpsError, self.lpc.login)

        self.fake_login_with.mock.side_effect = TimeoutError()
        self.assertRaises(errors.LaunchpadHttpsError, self.lpc.login)

    def test_load_connection_refused(self):
        # ConnectionRefusedError should surface.
        self.fake_login_with.mock.reset_mock()
        self.lpc._lp._load_mock.side_effect = ConnectionRefusedError
        self.assertRaises(ConnectionRefusedError, self.lpc._lp_load_url, "foo")
        self.fake_login_with.mock.assert_not_called()

    def test_load_connection_reset_once(self):
        # Load URL should work OK after single connection reset.
        self.fake_login_with.mock.reset_mock()
        self.lpc._lp._load_mock.side_effect = [ConnectionResetError, None]
        self.lpc._lp_load_url(url="foo")
        self.fake_login_with.mock.assert_called()

    def test_load_connection_reset_twice(self):
        # Load URL should fail with two connection resets.
        self.fake_login_with.mock.reset_mock()
        self.lpc._lp._load_mock.side_effect = [
            ConnectionResetError,
            ConnectionResetError,
        ]
        self.assertRaises(ConnectionResetError, self.lpc._lp_load_url, "foo")
        self.fake_login_with.mock.assert_called()

    def test_create_snap(self):
        self.lpc._create_snap()
        self.lpc._lp.snaps.new_mock.assert_called_with(
            auto_build=False,
            auto_build_archive="/ubuntu/+archive/primary",
            auto_build_pocket="Updates",
            git_path="master",
            git_repository_url="https://user@git.launchpad.net/~user/+git/id/",
            name="id",
            owner="/~user",
        )

    def test_create_snap_with_archs(self):
        self.lpc.architectures = ["arch1", "arch2"]
        self.lpc._create_snap()
        self.lpc._lp.snaps.new_mock.assert_called_with(
            auto_build=False,
            auto_build_archive="/ubuntu/+archive/primary",
            auto_build_pocket="Updates",
            git_path="master",
            git_repository_url="https://user@git.launchpad.net/~user/+git/id/",
            name="id",
            owner="/~user",
            processors=["/+processors/arch1", "/+processors/arch2"],
        )

    def test_delete_snap(self):
        self.lpc._delete_snap()
        self.lpc._lp.snaps.getByName_mock.assert_called_with(name="id", owner="/~user")

    def test_start_build(self):
        self.lpc.start_build()

    @mock.patch(
        "tests.unit.remote_build.test_launchpad.SnapImpl.requestBuilds",
        return_value=SnapBuildReqImpl(
            status="Failed", error_message="snapcraft.yaml not found..."
        ),
    )
    def test_start_build_error(self, mock_rb):
        raised = self.assertRaises(errors.RemoteBuilderError, self.lpc.start_build)
        self.assertThat(str(raised), Contains("snapcraft.yaml not found..."))

    @mock.patch(
        "tests.unit.remote_build.test_launchpad.SnapImpl.requestBuilds",
        return_value=SnapBuildReqImpl(status="Pending", error_message=""),
    )
    @mock.patch("time.time", return_value=500)
    def test_start_build_error_timeout(self, mock_time, mock_rb):
        self.lpc.deadline = 499
        raised = self.assertRaises(errors.RemoteBuildTimeoutError, self.lpc.start_build)
        self.assertThat(
            str(raised), Equals("Remote build exceeded configured timeout.")
        )

    def test_issue_build_request_defaults(self):
        fake_snap = mock.MagicMock()

        self.lpc._issue_build_request(fake_snap)

        self.assertThat(
            fake_snap.mock_calls,
            Equals(
                [
                    mock.call.requestBuilds(
                        archive="main_archive",
                        channels={"core18": "stable", "snapcraft": "stable"},
                        pocket="Updates",
                    )
                ]
            ),
        )

    @mock.patch("snapcraft.internal.remote_build.LaunchpadClient._download_file")
    def test_monitor_build(self, mock_download_file):
        open("test_i386.txt", "w").close()
        open("test_i386.1.txt", "w").close()

        self.lpc.start_build()
        self.lpc.monitor_build(interval=0)
        self.assertThat(
            mock_download_file.mock_calls,
            Equals(
                [
                    mock.call(
                        dst="snap_file_i386.snap", url="url_for/snap_file_i386.snap"
                    ),
                    mock.call(
                        dst="test_i386.2.txt",
                        gunzip=True,
                        url="url_for/build_log_file_1",
                    ),
                    mock.call(
                        dst="snap_file_amd64.snap", url="url_for/snap_file_amd64.snap"
                    ),
                    mock.call(
                        dst="test_amd64.txt",
                        gunzip=True,
                        url="url_for/build_log_file_2",
                    ),
                    mock.call(
                        dst="snap_file_amd64.snap", url="url_for/snap_file_amd64.snap"
                    ),
                ]
            ),
        )

    @mock.patch("snapcraft.internal.remote_build.LaunchpadClient._download_file")
    @mock.patch(
        "tests.unit.remote_build.test_launchpad.BuildImpl.getFileUrls", return_value=[]
    )
    @mock.patch("logging.Logger.error")
    def test_monitor_build_error(self, mock_log, mock_urls, mock_download_file):
        self.lpc.start_build()
        self.lpc.monitor_build(interval=0)
        mock_download_file.assert_has_calls(
            [
                mock.call(
                    url="url_for/build_log_file_2", gunzip=True, dst="test_amd64.txt"
                )
            ]
        )
        self.assertThat(
            mock_log.mock_calls,
            Equals(
                [
                    mock.call("Snap file not available for arch 'i386'."),
                    mock.call("Snap file not available for arch 'amd64'."),
                    mock.call("Build failed for arch 'amd64'."),
                    mock.call("Snap file not available for arch 'arm64'."),
                    mock.call("Build failed for arch 'arm64'."),
                ]
            ),
        )

    @mock.patch("snapcraft.internal.remote_build.LaunchpadClient._download_file")
    @mock.patch("time.time", return_value=500)
    def test_monitor_build_error_timeout(self, mock_time, mock_rb):
        self.lpc.deadline = 499
        self.lpc.start_build()
        raised = self.assertRaises(
            errors.RemoteBuildTimeoutError, self.lpc.monitor_build, interval=0
        )
        self.assertThat(
            str(raised), Equals("Remote build exceeded configured timeout.")
        )

    def test_get_build_status(self):
        self.lpc.start_build()
        build_status = self.lpc.get_build_status()
        self.assertThat(
            build_status,
            Equals(
                {
                    "amd64": "Failed to build",
                    "arm64": "Failed to build",
                    "i386": "Successfully built",
                }
            ),
        )

    def _make_snapcraft_project(self):
        yaml = textwrap.dedent(
            """\
            name: test
            base: core18
            version: "1.0"
            summary: test
            description: test
            confinement: strict
            grade: stable
            """
        )
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(yaml)
        project = snapcraft.project.Project(
            snapcraft_yaml_file_path=snapcraft_yaml_file_path
        )
        return project

    def test_git_repository_creation(self):
        source_testdir = self.useFixture(TestDir())
        source_testdir.create_file("foo")
        repo_dir = source_testdir.path

        self.lpc._gitify_repository(repo_dir)

        assert self.mock_git_class.mock_calls == [
            mock.call.check_command_installed(),
            mock.call(repo_dir, repo_dir, silent=True),
            mock.call().init(),
            mock.call().add("foo"),
            mock.call().commit("committed by snapcraft version: 100.1234"),
        ]

    def test_push_source_tree(self):
        source_testdir = self.useFixture(TestDir())
        repo_dir = source_testdir.path

        self.lpc.push_source_tree(repo_dir)

        self.mock_git_class.assert_has_calls == [
            mock.call(
                "https://user:access-token@git.launchpad.net/~user/+git/id/",
                "HEAD:master",
                force=True,
            )
        ]

    def test_push_source_tree_error(self):
        self.mock_git_class.return_value.push.side_effect = (
            SnapcraftPullError(
                command="git push HEAD:master https://user:access-token@url",
                exit_code=128,
            ),
        )
        source_testdir = self.useFixture(TestDir())
        repo_dir = source_testdir.path

        self.assertRaises(
            errors.LaunchpadGitPushError, self.lpc.push_source_tree, repo_dir
        )
