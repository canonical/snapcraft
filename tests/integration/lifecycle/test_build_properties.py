# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from testtools.matchers import Contains, FileContains, FileExists

from tests import integration


class BuildPropertiesTestCase(integration.TestCase):
    def test_build(self):
        self.assert_expected_build_state("local-plugin-build-properties")

    def test_build_legacy_build_properties(self):
        self.assert_expected_build_state("local-plugin-legacy-build-properties")

    def assert_expected_build_state(self, project_dir):
        self.run_snapcraft("build", project_dir)

        state_file = os.path.join(self.parts_dir, "x-local-plugin", "state", "build")
        self.assertThat(state_file, FileExists())
        # Verify that the correct schema dependencies made it into the state.
        # and that the contents of the dependencies made it in as well.
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                properties:
                  after: []
                  build: ''
                  build-attributes: []
                  build-packages: []
                  disable-parallel: false
                  foo: bar
                  install: ''
                  organize: {}
                  override-build: snapcraftctl build
                  prepare: ''
                  stage-packages: []
                schema_properties:
                - foo
                - stage-packages
              """
                    )
                )
            ),
        )
