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

import fixtures
from testtools import TestCase
from testtools.matchers import Equals

from snapcraft.internal import lifecycle, project_loader, states, steps
from snapcraft.project import Project
from snapcraft.storeapi.errors import SnapNotFoundError
from snapcraft.storeapi.info import SnapInfo
from tests import fixture_setup


class TestGlobalState(TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixtures.FakeLogger(level=logging.ERROR))

        temp_cwd = fixture_setup.TempCWD()
        self.useFixture(temp_cwd)

        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            temp_cwd.path, base="core18", parts={"test-part": {"plugin": "nil"}}
        )
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )

        self.global_state_filepath = project._get_global_state_file_path()

        self.project_config = project_loader.load_config(project)

        self.useFixture(
            fixtures.MockPatchObject(
                self.project_config, "get_build_snaps", return_value={"core18"}
            )
        )

        self.useFixture(
            fixtures.MockPatch("snapcraft.internal.lifecycle._runner._Executor.run")
        )

        self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo.snaps.install_snaps")
        )

        # Avoid unnecessary calls to info.
        channel_map = []
        for arch in ("amd64", "i386", "s390x", "arm64", "armhf", "ppc64el"):
            channel_map.append(
                {
                    "channel": {
                        "architecture": arch,
                        "name": "stable",
                        "released-at": "2019-10-15T13:54:06.800280+00:00",
                        "risk": "stable",
                        "track": "latest",
                    },
                    "confinement": "strict",
                    "download": {
                        "deltas": [],
                        "sha3-384": "64d232d6bfa65be14d7f8d84e952d4e372e12021e2c3dbaf70cf2af5e78bf51c4baf9c9107dd6db815064636b781bda6",
                        "size": 57151488,
                        "url": "https://api.snapcraft.io/api/v1/snaps/download/CSO04Jhav2yK0uz97cr0ipQRyqg0qQL6_1223.snap",
                    },
                    "revision": 1223,
                }
            )
        info = {
            "channel-map": channel_map,
            "default-track": None,
            "name": "core18",
            "snap": {
                "name": "core18",
                "publisher": {
                    "display-name": "Canonical",
                    "id": "canonical",
                    "username": "canonical",
                    "validation": "verified",
                },
                "snap-id": "CSO04Jhav2yK0uz97cr0ipQRyqg0qQL6",
            },
            "snap-id": "CSO04Jhav2yK0uz97cr0ipQRyqg0qQL6",
        }
        self.fake_storeapi_get_info = fixtures.MockPatch(
            "snapcraft.storeapi._snap_api.SnapAPI.get_info",
            return_value=SnapInfo(info),
        )
        self.useFixture(self.fake_storeapi_get_info)

    def test_stable_grade_for_stable_base(self):
        lifecycle.execute(steps.PULL, self.project_config)

        global_state = states.GlobalState.load(filepath=self.global_state_filepath)
        self.assertThat(global_state.get_required_grade(), Equals("stable"))
        self.fake_storeapi_get_info.mock.assert_called_once_with("core18")

    def test_stable_grade_for_non_stable_base(self):
        self.fake_storeapi_get_info.mock.side_effect = SnapNotFoundError(
            snap_name="core18"
        )
        lifecycle.execute(steps.PULL, self.project_config)

        global_state = states.GlobalState.load(filepath=self.global_state_filepath)
        self.assertThat(global_state.get_required_grade(), Equals("devel"))
        self.fake_storeapi_get_info.mock.assert_called_once_with("core18")

    def test_grade_not_queried_for_if_already_set(self):
        # Set the grade
        global_state = states.GlobalState()
        global_state.set_required_grade("devel")
        global_state.save(filepath=self.global_state_filepath)

        lifecycle.execute(steps.PULL, self.project_config)

        self.fake_storeapi_get_info.mock.assert_not_called()
