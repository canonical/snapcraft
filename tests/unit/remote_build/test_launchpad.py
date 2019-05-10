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

import snapcraft
import textwrap

from snapcraft.internal.remote_build import LaunchpadClient, errors
from testtools.matchers import Equals, Contains
from tests import unit
from typing import Any, Dict
from unittest import mock


class DictAttr:
    def __init__(self, data: Dict[str, Any]):
        self._data = data

    def __getattr__(self, attr):
        return self._data[attr]


class BuildImpl:
    def __init__(self):
        self.getFileUrls_mock = mock.Mock()

    def getFileUrls(self, *args, **kw):
        self.getFileUrls_mock(*args, **kw)
        return ["url_for/snap_file_i386.snap"]


class SnapImpl:
    def __init__(self):
        self.requestBuilds_mock = mock.Mock()
        self.lp_delete_mock = mock.Mock()

    def requestBuilds(self, *args, **kw):
        self.requestBuilds_mock(*args, **kw)
        request = {
            "self_link": "http://request_self_link/1234",
            "builds_collection_link": "http://builds_collection_link",
        }
        return DictAttr(request)

    def lp_delete(self):
        self.lp_delete_mock()


class SnapsImpl:
    def __init__(self):
        self.new = mock.Mock()
        self.getByName_mock = mock.Mock()
        self.snap = SnapImpl()

    def getByName(self, *args, **kw):
        self.getByName_mock(*args, **kw)
        return self.snap


class DistImpl:
    def __init__(self):
        self.main_archive = "main_archive"


class LaunchpadImpl:
    def __init__(self):
        self.login_mock = mock.Mock()
        self.load_mock = mock.Mock()
        self.snaps = SnapsImpl()
        self.people = {"user": "/~user"}
        self.distributions = {"ubuntu": DistImpl()}

    def load(self, url: str, *args, **kw):
        self.load_mock(url, *args, **kw)
        if "/+build-request/" in url:
            request = {
                "self_link": "http://request_self_link/1234",
                "builds_collection_link": "http://builds_collection_link",
            }
            return DictAttr(request)
        elif "http://build_self_link_1" in url:
            return BuildImpl()
        else:
            builds = {
                "entries": [
                    {
                        "arch_tag": "i386",
                        "buildstate": "Successfully built",
                        "self_link": "http://build_self_link_1",
                        "build_log_url": "url_for/build_log_file_1",
                    },
                    {
                        "arch_tag": "amd64",
                        "buildstate": "Failed to build",
                        "self_link": "http://build_self_link_2",
                        "build_log_url": "url_for/build_log_file_2",
                    },
                ]
            }
            return DictAttr(builds)


class LaunchpadTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self._project = self._make_snapcraft_project()

    @mock.patch("launchpadlib.launchpad.Launchpad.login_with")
    def test_login(self, mock_login):
        lpc = LaunchpadClient(self._project, "id")
        lpc.login("user")
        self.assertThat(lpc.user, Equals("user"))
        mock_login.assert_called_with(
            "snapcraft remote-build {}".format(snapcraft.__version__),
            "production",
            mock.ANY,
            credentials_file=mock.ANY,
            version="devel",
        )

    @mock.patch("launchpadlib.launchpad.Launchpad")
    def test_create_snap(self, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        lpc.create_snap("git+ssh://repo", "branch", ["arch1", "arch2"])
        lpc._lp.snaps.new.assert_called_with(
            auto_build=False,
            auto_build_archive="/ubuntu/+archive/primary",
            auto_build_pocket="Updates",
            git_path="branch",
            git_repository_url="https://repo",
            name="id",
            owner="/~user",
            processors=["/+processors/arch1", "/+processors/arch2"],
        )

    @mock.patch("launchpadlib.launchpad.Launchpad")
    def test_delete_snap(self, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        lpc.delete_snap()
        lpc._lp.snaps.getByName_mock.assert_called_with(name="id", owner="/~user")

    @mock.patch("launchpadlib.launchpad.Launchpad")
    def test_start_build(self, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        num = lpc.start_build()
        lpc._lp.load_mock.assert_called_once_with("http://builds_collection_link")
        self.assertThat(num, Equals("1234"))

    @mock.patch("launchpadlib.launchpad.Launchpad")
    @mock.patch(
        "tests.unit.remote_build.test_launchpad.LaunchpadImpl.load",
        return_value=DictAttr({"entries": []}),
    )
    def test_start_build_error(self, mock_load, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        raised = self.assertRaises(
            errors.RemoteBuilderNotReadyError, lpc.start_build, timeout=0, attempts=1
        )
        self.assertThat(str(raised), Contains("is not ready"))

    @mock.patch("launchpadlib.launchpad.Launchpad")
    def test_recover_build(self, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        lpc.recover_build(1234)
        self.assertThat(lpc._lp.load_mock.call_count, Equals(2))

    @mock.patch("launchpadlib.launchpad.Launchpad")
    @mock.patch("snapcraft.internal.remote_build.LaunchpadClient._download_file")
    def test_monitor_build(self, mock_download, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        lpc.start_build()
        lpc.monitor_build(interval=0)
        mock_download.assert_has_calls(
            [
                mock.call("url_for/snap_file_i386.snap", "snap_file_i386.snap"),
                mock.call("url_for/build_log_file_2", "buildlog_amd64.txt.gz"),
            ]
        )

    @mock.patch("launchpadlib.launchpad.Launchpad")
    @mock.patch("snapcraft.internal.remote_build.LaunchpadClient._download_file")
    @mock.patch(
        "tests.unit.remote_build.test_launchpad.BuildImpl.getFileUrls", return_value=[]
    )
    @mock.patch("logging.Logger.error")
    def test_monitor_build_error(self, mock_log, mock_urls, mock_download, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        lpc.start_build()
        lpc.monitor_build(interval=0)
        mock_download.assert_has_calls(
            [mock.call("url_for/build_log_file_2", "buildlog_amd64.txt.gz")]
        )
        mock_log.assert_called_with(
            "Build failed for arch amd64. Log file is 'buildlog_amd64.txt.gz'."
        )

    @mock.patch("launchpadlib.launchpad.Launchpad")
    def test_get_build_status(self, mock_lp):
        lpc = LaunchpadClient(self._project, "id")
        lpc.user = "user"
        lpc._lp = LaunchpadImpl()
        lpc.start_build()
        build_status = lpc.get_build_status()
        self.assertThat(
            build_status,
            Equals([("i386", "Successfully built"), ("amd64", "Failed to build")]),
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
