# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import os
import subprocess
import textwrap
import yaml

from testtools.matchers import Equals, Contains

from tests import integration


class SnapcraftctlSetVersionTestCase(integration.TestCase):
    def test_set_version(self):
        self.construct_yaml(
            version=None,
            adopt_info="my-part",
            parts=textwrap.dedent(
                """\
                my-part:
                  plugin: nil
                  override-pull: snapcraftctl set-version override-version
                """
            ),
        )

        self.run_snapcraft("prime")

        with open(os.path.join("prime", "meta", "snap.yaml")) as f:
            y = yaml.load(f)

        self.assertThat(y["version"], Equals("override-version"))

    def test_set_version_no_overwrite(self):
        self.construct_yaml(
            version="test-version",
            adopt_info="my-part",
            parts=textwrap.dedent(
                """\
                my-part:
                  plugin: nil
                  override-pull: snapcraftctl set-version override-version
                """
            ),
        )

        self.run_snapcraft("prime")

        with open(os.path.join("prime", "meta", "snap.yaml")) as f:
            y = yaml.load(f)

        self.assertThat(y["version"], Equals("test-version"))

    def test_set_version_twice_errors(self):
        self.construct_yaml(
            version=None,
            adopt_info="my-part",
            parts=textwrap.dedent(
                """\
                my-part:
                  plugin: nil
                  override-pull: snapcraftctl set-version override-version
                  override-prime: snapcraftctl set-version no-this-version
                """
            ),
        )

        raised = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, "prime"
        )
        self.assertThat(
            raised.output,
            Contains("Unable to set version: it was already set in the 'pull' step"),
        )
        self.assertThat(raised.output, Contains("Failed to run 'override-prime'"))
