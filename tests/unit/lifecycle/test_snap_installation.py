# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
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

import logging
from unittest import mock

import fixtures
from testtools import TestCase
from testtools.matchers import Contains

from snapcraft.internal.lifecycle._runner import _install_build_snaps


class TestSnapInstall(TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_install(self, mock_install_build_snaps):
        _install_build_snaps({"foo/latest/stable", "bar/default/edge"}, set())

        mock_install_build_snaps.assert_called_once_with(
            {"foo/latest/stable", "bar/default/edge"}
        )

    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_install_with_content_snap(self, mock_install_build_snaps):
        _install_build_snaps({"foo/latest/stable"}, {"content1/latest/stable"})

        mock_install_build_snaps.assert_has_calls(
            [mock.call({"foo/latest/stable"}), mock.call(["content1/latest/stable"])]
        )

    @mock.patch("snapcraft.internal.common.is_process_container", return_value=True)
    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_install_on_docker(self, mock_install_build_snaps, mock_docker_instance):
        _install_build_snaps({"foo/latest/stable", "bar/default/edge"}, set())

        mock_install_build_snaps.assert_not_called()
        self.assertThat(
            self.fake_logger.output,
            Contains(
                "The following snaps are required but not installed as snapcraft "
                "is running inside docker or podman container: "
            ),
        )

    @mock.patch("snapcraft.internal.common.is_process_container", return_value=True)
    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_install_with_content_snap_on_docker(
        self, mock_install_build_snaps, mock_docker_instance
    ):
        _install_build_snaps({"foo/latest/stable"}, {"content1/latest/stable"})

        mock_install_build_snaps.assert_not_called()
        self.assertThat(
            self.fake_logger.output,
            Contains(
                "The following snaps are required but not installed as snapcraft "
                "is running inside docker or podman container: "
            ),
        )
