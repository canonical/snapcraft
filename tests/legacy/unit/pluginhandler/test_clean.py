# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
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
import pathlib
from unittest.mock import MagicMock, call, patch

import pytest
from testtools.matchers import Equals

from snapcraft_legacy import file_utils
from snapcraft_legacy.internal import errors, pluginhandler, steps
from tests.legacy.unit import TestCase, load_part


class CleanTestCase(TestCase):
    @patch.object(pluginhandler.PluginHandler, "is_clean")
    @patch("os.rmdir")
    @patch("os.listdir")
    def test_clean_part_that_exists(self, mock_listdir, mock_rmdir, mock_is_clean):
        mock_listdir.return_value = False
        mock_is_clean.return_value = True

        part_name = "test_part"
        partdir = os.path.join(self.parts_dir, part_name)
        os.makedirs(partdir)

        p = self.load_part(part_name)
        p.clean()

        mock_listdir.assert_called_once_with(partdir)
        mock_rmdir.assert_called_once_with(partdir)

    @patch("os.rmdir")
    @patch("os.listdir")
    @patch("os.path.exists")
    def test_clean_part_already_clean(self, mock_exists, mock_listdir, mock_rmdir):
        mock_exists.return_value = False

        part_name = "test_part"
        p = self.load_part(part_name)
        p.clean()

        partdir = os.path.join(self.parts_dir, part_name)
        mock_exists.assert_has_calls([call(partdir)])
        self.assertFalse(mock_listdir.called)
        self.assertFalse(mock_rmdir.called)

    @patch.object(pluginhandler.PluginHandler, "is_clean")
    @patch("os.rmdir")
    @patch("os.listdir")
    def test_clean_part_remaining_parts(self, mock_listdir, mock_rmdir, mock_is_clean):
        mock_listdir.return_value = True
        mock_is_clean.return_value = True

        part_name = "test_part"
        partdir = os.path.join(self.parts_dir, part_name)
        os.makedirs(partdir)

        p = self.load_part(part_name)
        p.clean()

        mock_listdir.assert_called_once_with(partdir)
        self.assertFalse(mock_rmdir.called)

    def test_clean_old_prime_state(self):
        handler = self.load_part("test-part")
        handler.makedirs()

        open(os.path.join(self.prime_dir, "1"), "w").close()

        handler.mark_done(steps.PRIME, None)

        self.assertTrue(os.path.exists(handler.part_dir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.part_dir))

    def test_clean_prime_old_prime_state(self):
        handler = self.load_part("test-part")
        handler.makedirs()

        primed_file = os.path.join(self.prime_dir, "1")
        open(primed_file, "w").close()

        handler.mark_done(steps.PRIME, None)

        raised = self.assertRaises(
            errors.MissingStateCleanError, handler.clean, step=steps.PRIME
        )

        self.assertThat(raised.step, Equals(steps.PRIME))
        self.assertTrue(os.path.isfile(primed_file))

    def test_clean_old_stage_state(self):
        handler = self.load_part("part1")
        handler.makedirs()

        open(os.path.join(self.stage_dir, "1"), "w").close()

        handler.mark_done(steps.STAGE, None)

        self.assertTrue(os.path.exists(handler.part_dir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.part_dir))

    def test_clean_stage_old_stage_state(self):
        handler = self.load_part("test-part")
        handler.makedirs()

        staged_file = os.path.join(self.stage_dir, "1")
        open(staged_file, "w").close()

        handler.mark_done(steps.STAGE, None)

        raised = self.assertRaises(
            errors.MissingStateCleanError, handler.clean, step=steps.STAGE
        )

        self.assertThat(raised.step, Equals(steps.STAGE))
        self.assertTrue(os.path.isfile(staged_file))


class TestCleanStage:
    scenarios = [
        ("all", {"fileset": ["*"]}),
        ("no1", {"fileset": ["-1"]}),
        ("onlya", {"fileset": ["a"]}),
        ("onlybase", {"fileset": ["*", "-*/*"]}),
        ("only1a", {"fileset": ["1/a"]}),
        ("nostara", {"fileset": ["-*/a"]}),
    ]

    def test_clean_stage(self, tmp_work_path, fileset):
        handler = load_part("test_part", part_properties={"stage": fileset})
        handler.makedirs()

        installdir = pathlib.Path(handler.part_install_dir)

        (installdir / "1/1a/1b").mkdir(parents=True)
        (installdir / "2/2a").mkdir(parents=True)
        (installdir / "3").mkdir(parents=True)

        (installdir / "a").touch()
        (installdir / "b").touch()
        (installdir / "1/a").touch()
        (installdir / "3/a").touch()

        handler.mark_done(steps.BUILD)

        # Stage the installed files
        handler.stage()

        assert os.listdir(handler._project.stage_dir) != []

        handler.clean_stage({})

        assert os.listdir(handler._project.stage_dir) == []


class PerStepCleanTestCase(TestCase):
    def setUp(self):
        super().setUp()

        self.manager_mock = MagicMock()

        patcher = patch.object(pluginhandler.PluginHandler, "clean_pull")
        self.manager_mock.attach_mock(patcher.start(), "clean_pull")
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, "clean_build")
        self.manager_mock.attach_mock(patcher.start(), "clean_build")
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, "clean_stage")
        self.manager_mock.attach_mock(patcher.start(), "clean_stage")
        self.addCleanup(patcher.stop)

        patcher = patch.object(pluginhandler.PluginHandler, "clean_prime")
        self.manager_mock.attach_mock(patcher.start(), "clean_prime")
        self.addCleanup(patcher.stop)

        self.handler = self.load_part("test_part")

    def test_clean_pull_order(self):
        self.handler.clean(step=steps.PULL)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(4))
        self.manager_mock.assert_has_calls(
            [
                call.clean_prime({}),
                call.clean_stage({}),
                call.clean_build(),
                call.clean_pull(),
            ]
        )

    def test_clean_build_order(self):
        self.handler.clean(step=steps.BUILD)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(3))
        self.manager_mock.assert_has_calls(
            [call.clean_prime({}), call.clean_stage({}), call.clean_build()]
        )

    def test_clean_stage_order(self):
        self.handler.clean(step=steps.STAGE)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(2))
        self.manager_mock.assert_has_calls([call.clean_prime({}), call.clean_stage({})])

    def test_clean_prime_order(self):
        self.handler.clean(step=steps.PRIME)

        # Verify the step cleaning order
        self.assertThat(len(self.manager_mock.mock_calls), Equals(1))
        self.manager_mock.assert_has_calls([call.clean_prime({})])