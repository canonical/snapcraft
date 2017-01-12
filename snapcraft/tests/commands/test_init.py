# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

from snapcraft.main import main
from snapcraft import tests


class InitCommandTestCase(tests.TestCase):

    def test_init_with_existing_snapcraft_yaml_must_fail(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('snapcraft.yaml', 'w').close()

        raised = self.assertRaises(
            SystemExit,
            main, ['init'])

        self.assertEqual(1, raised.code)
        self.assertEqual(
            fake_logger.output,
            'snapcraft.yaml already exists!\n')

    def test_init_with_existing_dot_snapcraft_yaml_must_fail(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        open('.snapcraft.yaml', 'w').close()

        raised = self.assertRaises(
            SystemExit,
            main, ['init'])

        self.assertEqual(1, raised.code)
        self.assertEqual(
            fake_logger.output,
            '.snapcraft.yaml already exists!\n')

    def test_init_must_write_snapcraft_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

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
    plugin: nil"""  # noqa, lines too long

        main(['init'])

        self.assertEqual(
            'Created snapcraft.yaml.\nEdit the file to your '
            'liking or run `snapcraft` to get started\n',
            fake_logger.output)

        # Verify the generated yaml
        with open('snapcraft.yaml', 'r') as f:
            self.assertEqual(f.read(), expected_yaml)
