# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import snapcraft.internal.errors
from testtools.matchers import Equals, FileContains

from . import CommandBaseTestCase


class InitCommandTestCase(CommandBaseTestCase):
    def test_init_must_write_snapcraft_yaml(self):
        expected_yaml = """name: my-snap-name # you probably want to 'snapcraft register <name>'
version: '0.1' # just for humans, typically '1.2+git' or '1.3.2'
summary: Single-line elevator pitch for your amazing snap # 79 char long summary
description: |
  This is my-snap's description. You have a paragraph or two to tell the
  most important story about your snap. Keep it under 100 words though,
  we live in tweetspace and your description wants to look good in the snap
  store.

grade: devel # must be 'stable' to release into candidate/stable channels
confinement: devmode # use 'strict' once you have the right plugs and slots

parts:
  my-part:
    # See 'snapcraft plugins'
    plugin: nil\n"""  # noqa, lines too long

        result = self.run_command(["init"])

        self.assertThat(
            result.output,
            Equals(
                "Created snap/snapcraft.yaml.\nEdit the file to your liking or "
                "run `snapcraft` to get started\n"
            ),
        )

        # Verify the generated yaml
        self.assertThat(
            os.path.join("snap", "snapcraft.yaml"), FileContains(expected_yaml)
        )


class InitCommandExistingProjectTestCase(CommandBaseTestCase):

    scenarios = [
        (
            "snap/snapcraft.yaml",
            {
                "yaml_path": os.path.join("snap", "snapcraft.yaml"),
                "message": "snap/snapcraft.yaml already exists!\n",
            },
        ),
        # FIXME: Both snapcraft.yaml and .snapcraft.yaml are deprecated.
        (
            "snapcraft.yaml",
            {
                "yaml_path": "snapcraft.yaml",
                "message": "snapcraft.yaml already exists!\n",
            },
        ),
        (
            ".snapcraft.yaml",
            {
                "yaml_path": ".snapcraft.yaml",
                "message": ".snapcraft.yaml already exists!\n",
            },
        ),
    ]

    def test_init_with_existing_yaml(self):
        """Test that init bails if project yaml already exists"""
        dirname = os.path.dirname(self.yaml_path)
        if dirname:
            os.mkdir(dirname)

        open(self.yaml_path, "w").close()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command,
            ["init"],
        )

        self.assertThat(
            str(raised), Equals("{} already exists!".format(self.yaml_path))
        )
