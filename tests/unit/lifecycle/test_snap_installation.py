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

from testtools.matchers import Contains

from snapcraft.internal import lifecycle, steps
from . import LifecycleTestBase


class LifecycleSnapTest(LifecycleTestBase):
    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_part_with_build_snaps(self, mock_install_build_snaps):
        project = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                    build-snaps: [snap1, snap2]
                """
            )
        )

        lifecycle.execute(steps.PULL, project)

        mock_install_build_snaps.assert_called_once_with({"snap1", "snap2"})

    @mock.patch("snapcraft.internal.common.is_docker_instance", return_value=True)
    @mock.patch("snapcraft.repo.snaps.install_snaps")
    def test_part_with_build_snaps_on_docker(
        self, mock_install_build_snaps, mock_docker_instance
    ):
        project = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                parts:
                  part1:
                    plugin: nil
                    build-snaps: [snap1, snap2]
                """
            )
        )

        lifecycle.execute(steps.PULL, project)

        mock_install_build_snaps.assert_not_called()

        self.assertThat(
            self.fake_logger.output,
            Contains(
                "The following snaps are required but not installed as snapcraft "
                "is running inside docker: "
            ),
        )
