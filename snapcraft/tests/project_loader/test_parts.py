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

import logging
import os
import unittest
import unittest.mock
from textwrap import dedent

import fixtures
from testtools.matchers import Contains, Equals

import snapcraft
from snapcraft.internal import (
    deprecations,
    dirs,
    project_loader
)
from snapcraft import tests


class TestParts(tests.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch(
            'snapcraft.internal.project_loader.get_snapcraft_yaml')
        self.mock_get_yaml = patcher.start()
        self.mock_get_yaml.return_value = os.path.join(
            'snap', 'snapcraft.yaml')
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch(
            'snapcraft.internal.project_loader._parts_config.PartsConfig'
            '.load_part')
        self.mock_plugin_loader = patcher.start()
        self.addCleanup(patcher.stop)

        self.part_schema = project_loader.Validator().part_schema
        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    def test_get_parts_none(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        config = project_loader.load_config(None)
        self.assertThat(config.parts.get_part('not-a-part'), Equals(None))

    def test_slash_warning(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  part/1:
    plugin: go
    stage-packages: [fswebcam]
""")
        project_loader.load_config()

        self.assertThat(fake_logger.output, Contains(
            'DEPRECATED: Found a "/" in the name of the {!r} part'.format(
                'part/1')))

    def test_snap_deprecation(self):
        """Test that using the 'snap' keyword results in a warning."""

        fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
    snap: [foo]
""")
        project_loader.load_config()

        self.assertThat(fake_logger.output,
                        Contains(deprecations._deprecation_message('dn1')))


class PartsWithDummyRepoTestCase(tests.TestCase):

    @unittest.mock.patch(
        'snapcraft.internal.project_loader._parts_config.repo.Repo',
        wraps=snapcraft.internal.repo._base.DummyRepo)
    def test_load_with_dummy_repo(self, os_release_mock):
        self.make_snapcraft_yaml(dedent("""\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict

            parts:
              part1:
                source: https://github.com/snapcore/snapcraft.git
                plugin: python
                stage-packages: [fswebcam]
                snap: [foo]
        """))

        # Ensure the dummy repo returns an empty set for required
        # build tools.
        project_loader.load_config()
