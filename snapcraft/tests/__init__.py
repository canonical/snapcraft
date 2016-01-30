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
import os
import tempfile

import fixtures
import testscenarios

from snapcraft import common
from snapcraft.tests import fixture_setup


class MockOptions:

    def __init__(self, source=None, source_type=None, source_branch=None,
                 source_tag=None, source_subdir=None):
        self.source = source
        self.source_type = source_type
        self.source_branch = source_branch
        self.source_tag = source_tag
        self.source_subdir = source_subdir


class TestCase(testscenarios.WithScenarios, fixtures.TestWithFixtures):

    def setUp(self):
        super().setUp()
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path
        # Some tests will directly or indirectly change the plugindir, which
        # is a module variable. Make sure that it is returned to the original
        # value when a test ends.
        self.addCleanup(common.set_plugindir, common.get_plugindir())
        self.addCleanup(common.set_schemadir, common.get_schemadir())
        self.addCleanup(common.set_schemadir, common.get_schemadir())
        self.addCleanup(common.reset_env)
        common.set_schemadir(os.path.join(__file__,
                             '..', '..', '..', 'schema'))
        self.useFixture(fixtures.FakeLogger(level=logging.ERROR))

    def make_snapcraft_yaml(self, content, encoding='utf-8'):
        tempdir_obj = tempfile.TemporaryDirectory()
        self.addCleanup(tempdir_obj.cleanup)
        os.chdir(tempdir_obj.name)
        with open('snapcraft.yaml', 'w', encoding=encoding) as fp:
            fp.write(content)
