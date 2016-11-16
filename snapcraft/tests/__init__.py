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
from unittest import mock

import fixtures
import progressbar
import testscenarios

from snapcraft.internal import common
from snapcraft.tests import fixture_setup


class ContainsList(list):

        def __eq__(self, other):
            return all([i[0] in i[1] for i in zip(self, other)])


class MockOptions:

    def __init__(self, source=None, source_type=None, source_branch=None,
                 source_tag=None, source_subdir=None, source_depth=None,
                 source_commit=None, disable_parallel=False):
        self.source = source
        self.source_type = source_type
        self.source_depth = source_depth
        self.source_branch = source_branch
        self.source_commit = source_commit
        self.source_tag = source_tag
        self.source_subdir = source_subdir
        self.disable_parallel = disable_parallel


class TestCase(testscenarios.WithScenarios, fixtures.TestWithFixtures):

    def setUp(self):
        super().setUp()
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path
        self.useFixture(fixture_setup.TempXDG(self.path))
        self.fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)
        # Some tests will directly or indirectly change the plugindir, which
        # is a module variable. Make sure that it is returned to the original
        # value when a test ends.
        self.addCleanup(common.set_plugindir, common.get_plugindir())
        self.addCleanup(common.set_schemadir, common.get_schemadir())
        self.addCleanup(common.set_librariesdir, common.get_librariesdir())
        self.addCleanup(common.set_tourdir, common.get_tourdir())
        self.addCleanup(common.reset_env)
        common.set_schemadir(os.path.join(__file__,
                             '..', '..', '..', 'schema'))
        self.useFixture(fixtures.FakeLogger(level=logging.ERROR))

        patcher = mock.patch('multiprocessing.cpu_count')
        self.cpu_count = patcher.start()
        self.cpu_count.return_value = 2
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            'snapcraft.internal.indicators.ProgressBar',
            new=SilentProgressBar)
        patcher.start()
        self.addCleanup(patcher.stop)

        # These are what we expect by default
        self.snap_dir = os.path.join(os.getcwd(), 'prime')
        self.stage_dir = os.path.join(os.getcwd(), 'stage')
        self.parts_dir = os.path.join(os.getcwd(), 'parts')
        self.local_plugins_dir = os.path.join(self.parts_dir, 'plugins')

    def make_snapcraft_yaml(self, content, encoding='utf-8'):
        with open('snapcraft.yaml', 'w', encoding=encoding) as fp:
            fp.write(content)

    def verify_state(self, part_name, state_dir, expected_step):
        self.assertTrue(os.path.isdir(state_dir),
                        'Expected state directory for {}'.format(part_name))

        # Expect every step up to and including the specified one to be run
        index = common.COMMAND_ORDER.index(expected_step)
        for step in common.COMMAND_ORDER[:index+1]:
            self.assertTrue(os.path.exists(os.path.join(state_dir, step)),
                            'Expected {!r} to be run for {}'.format(
                                step, part_name))


class TestWithFakeRemoteParts(TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeParts())


class SilentProgressBar(progressbar.ProgressBar):
    """A progress bar causing no spurious output during tests."""

    def start(self):
        pass

    def update(self, value=None):
        pass

    def finish(self):
        pass
