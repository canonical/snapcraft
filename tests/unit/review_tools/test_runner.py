# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019 Canonical Ltd
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

import json
import pathlib
import subprocess

import fixtures

from snapcraft.internal import review_tools
from tests import unit


class RunTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.review_tools_path = "/snap/bin/review-tools.snap-review"
        self.fake_check_output = fixtures.MockPatch(
            "subprocess.check_output", return_value=b""
        )
        self.useFixture(self.fake_check_output)

        self.user_common_path = pathlib.Path(self.path) / "common"
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.review_tools._runner._get_review_tools_user_common",
                return_value=self.user_common_path,
            )
        )

        self.fake_snap_path = pathlib.Path("fake.snap")
        self.fake_snap_path.touch()

    def assert_fake_check_output_called(self):
        self.fake_check_output.mock.assert_called_once_with(
            [
                self.review_tools_path,
                self.user_common_path / self.fake_snap_path,
                "--json",
                "--allow-classic",
            ],
            env={"SNAP_ENFORCE_RESQUASHFS": "0"},
            stderr=subprocess.STDOUT,
        )

    def test_review_good(self):
        review_tools.run(snap_filename="fake.snap")

        self.assert_fake_check_output_called()

    def test_review_warnings(self):
        self.fake_check_output.mock.side_effect = subprocess.CalledProcessError(
            cmd=[self.review_tools_path, "fake.snap", "--json"],
            returncode=2,
            output=json.dumps(
                {
                    "snap.v2_functional": {"error": {}, "warn": {}},
                    "snap.v2_security": {
                        "warn": {
                            "security-snap-v2:security_issue": {
                                "text": "(NEEDS REVIEW) security message."
                            }
                        },
                        "error": {},
                    },
                    "snap.v2_lint": {"error": {}, "warn": {}},
                }
            ).encode(),
        )

        self.assertRaises(
            review_tools.errors.ReviewError, review_tools.run, snap_filename="fake.snap"
        )
        self.assert_fake_check_output_called()

    def test_review_errors(self):
        self.fake_check_output.mock.side_effect = subprocess.CalledProcessError(
            cmd=[self.review_tools_path, "fake.snap", "--json", "--allow-classic"],
            returncode=3,
            output=json.dumps(
                {
                    "snap.v2_functional": {"error": {}, "warn": {}},
                    "snap.v2_security": {
                        "error": {
                            "security-snap-v2:security_issue": {
                                "text": "(NEEDS REVIEW) security message."
                            }
                        },
                        "warn": {},
                    },
                    "snap.v2_lint": {"error": {}, "warn": {}},
                }
            ).encode(),
        )

        self.assertRaises(
            review_tools.errors.ReviewError, review_tools.run, snap_filename="fake.snap"
        )
        self.assert_fake_check_output_called()

    def test_review_unkown_error_bubbles_up(self):
        self.fake_check_output.mock.side_effect = subprocess.CalledProcessError(
            cmd=[self.review_tools_path, "fake.snap", "--json", "--allow-classic"],
            returncode=4,
            output=b"unknown",
        )

        self.assertRaises(
            subprocess.CalledProcessError, review_tools.run, snap_filename="fake.snap"
        )
        self.assert_fake_check_output_called()

    def test_review_tools_missing(self):
        self.fake_check_output.mock.side_effect = FileNotFoundError(
            "review-tools.snap-review"
        )

        self.assertRaises(
            review_tools.errors.ReviewToolMissing,
            review_tools.run,
            snap_filename="fake.snap",
        )
        self.assert_fake_check_output_called()
