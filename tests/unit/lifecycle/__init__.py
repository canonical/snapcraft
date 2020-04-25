# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import fixtures
import logging
import textwrap

import snapcraft
from snapcraft.internal import project_loader
from tests import unit


class LifecycleTestBase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.project_options = snapcraft.ProjectOptions()

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
            "snapcraft.storeapi._snap_index_client.SnapIndexClient.get_info",
            return_value=snapcraft.storeapi.info.SnapInfo(info),
        )

        self.useFixture(self.fake_storeapi_get_info)
        self.useFixture(fixtures.MockPatch("apt.Cache"))

    def make_snapcraft_project(self, parts, snap_type=""):
        yaml = textwrap.dedent(
            """\
            name: test
            base: core18
            version: "1.0"
            summary: test
            description: test
            confinement: strict
            grade: stable
            {type}

            {parts}
            """
        )

        self.snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            yaml.format(parts=parts, type=snap_type)
        )
        project = snapcraft.project.Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml_file_path
        )
        return project_loader.load_config(project)
