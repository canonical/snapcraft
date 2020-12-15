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
from textwrap import dedent

from testtools.matchers import Contains, Equals, FileContains

import snapcraft.internal.errors

from . import CommandBaseTestCase


class InitCommandTestCase(CommandBaseTestCase):
    def test_init_must_write_snapcraft_yaml(self):
        expected_yaml = dedent(
            """\
            name: my-snap-name # you probably want to 'snapcraft register <name>'
            base: core18 # the base snap is the execution environment for this snap
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
                plugin: nil\n"""
        )

        result = self.run_command(["init"])

        self.assertThat(
            result.output,
            Contains(
                "Created snap/snapcraft.yaml.\n"
                "Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more "
                "information about the snapcraft.yaml format."
            ),
        )

        # Verify the generated yaml
        self.assertThat(
            os.path.join("snap", "snapcraft.yaml"), FileContains(expected_yaml)
        )


class InitCommandExistingProjectTestCase(CommandBaseTestCase):
    def test_init_with_existing_yaml(self):
        """Test that init bails if project yaml already exists"""
        yaml_path = os.path.join("snap", "snapcraft.yaml")

        dirname = os.path.dirname(yaml_path)
        if dirname:
            os.mkdir(dirname)

        open(yaml_path, "w").close()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command,
            ["init"],
        )

        self.assertThat(str(raised), Equals("{} already exists!".format(yaml_path)))
