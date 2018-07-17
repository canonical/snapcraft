# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

import subprocess

import testscenarios

from tests import integration


class ChecksumAlgorithmsTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        (project_dir, {"project_dir": project_dir})
        for project_dir in [
            "checksum-algorithms",
            "deb-with-checksum",
            "rpm-with-checksum",
        ]
    ]

    def test_checksum_algorithms(self):
        self.run_snapcraft("pull", self.project_dir)


class InvalidChecksumsTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        (part, {"part": part})
        for part in [
            "checksum-md5",
            "checksum-sha1",
            "checksum-sha224",
            "checksum-sha256",
            "checksum-sha384",
            "checksum-sha512",
            "checksum-sha3-284",
            "checksum-sha3-256",
            "checksum-sha3-512",
        ]
    ]

    def test_checksum_invalid(self):
        project_dir = "checksum-algorithms-invalid"
        self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft,
            ["pull", self.part],
            project_dir,
        )
