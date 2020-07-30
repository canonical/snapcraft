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

from testtools.matchers import Equals

from snapcraft import file_utils
from snapcraft.internal import errors, pluginhandler, steps
from tests.unit import TestCase, load_part


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

    def test_clean_prime_multiple_independent_parts(self):
        # Create part1 and get it through the "build" step.
        handler1 = self.load_part("part1")
        handler1.makedirs()

        bindir = os.path.join(handler1.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()

        handler1.mark_done(steps.BUILD)

        # Now create part2 and get it through the "build" step.
        handler2 = self.load_part("part2")
        handler2.makedirs()

        bindir = os.path.join(handler2.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "2"), "w").close()

        handler2.mark_done(steps.BUILD)

        # Now stage both parts
        handler1.stage()
        handler2.stage()

        # And prime both parts
        handler1.prime()
        handler2.prime()

        # Verify that part1's file has been primeped
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "1")))

        # Verify that part2's file has been primeped
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "2")))

        # Now clean the prime step for part1
        handler1.clean_prime({})

        # Verify that part1's file is no longer primeped
        self.assertFalse(
            os.path.exists(os.path.join(self.prime_dir, "bin", "1")),
            "Expected part1's primeped files to be cleaned",
        )

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(self.prime_dir, "bin", "2")),
            "Expected part2's primeped files to be untouched",
        )

    def test_clean_prime_after_fileset_change(self):
        # Create part1 and get it through the "build" step.
        handler = self.load_part("part1")
        handler.makedirs()

        bindir = os.path.join(handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        handler.mark_done(steps.BUILD)
        handler.stage()
        handler.prime()

        # Verify that both files have been primeped
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "1")))
        self.assertTrue(os.path.exists(os.path.join(self.prime_dir, "bin", "2")))

        # Now update the `snap` fileset to only snap one of these files
        handler.plugin.options.snap = ["bin/1"]

        # Now clean the prime step for part1
        handler.clean_prime({})

        # Verify that part1's file is no longer primeped
        self.assertFalse(
            os.path.exists(os.path.join(self.prime_dir, "bin", "1")),
            "Expected bin/1 to be cleaned",
        )
        self.assertFalse(
            os.path.exists(os.path.join(self.prime_dir, "bin", "2")),
            "Expected bin/2 to be cleaned as well, even though the filesets "
            "changed since it was primeped.",
        )

    def test_clean_old_prime_state(self):
        handler = self.load_part("test-part")
        handler.makedirs()

        open(os.path.join(self.prime_dir, "1"), "w").close()

        handler.mark_done(steps.PRIME, None)

        self.assertTrue(os.path.exists(handler.plugin.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.plugin.partdir))

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

    def test_clean_stage_multiple_independent_parts(self):
        # Create part1 and get it through the "build" step.
        handler1 = self.load_part("part1")
        handler1.makedirs()

        bindir = os.path.join(handler1.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()

        handler1.mark_done(steps.BUILD)

        # Now create part2 and get it through the "build" step.
        handler2 = self.load_part("part2")
        handler2.makedirs()

        bindir = os.path.join(handler2.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "2"), "w").close()

        handler2.mark_done(steps.BUILD)

        # Now stage both parts
        handler1.stage()
        handler2.stage()

        # Verify that part1's file has been staged
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "1")))

        # Verify that part2's file has been staged
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "2")))

        # Now clean the stage step for part1
        handler1.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, "bin", "1")),
            "Expected part1's staged files to be cleaned",
        )

        # Verify that part2's file is still there
        self.assertTrue(
            os.path.exists(os.path.join(self.stage_dir, "bin", "2")),
            "Expected part2's staged files to be untouched",
        )

    def test_clean_stage_after_fileset_change(self):
        # Create part1 and get it through the "build" step.
        handler = self.load_part("part1")
        handler.makedirs()

        bindir = os.path.join(handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        handler.mark_done(steps.BUILD)
        handler.stage()

        # Verify that both files have been staged
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "1")))
        self.assertTrue(os.path.exists(os.path.join(self.stage_dir, "bin", "2")))

        # Now update the `stage` fileset to only snap one of these files
        handler.plugin.options.stage = ["bin/1"]

        # Now clean the prime step for part1
        handler.clean_stage({})

        # Verify that part1's file is no longer staged
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, "bin", "1")),
            "Expected bin/1 to be cleaned",
        )
        self.assertFalse(
            os.path.exists(os.path.join(self.stage_dir, "bin", "2")),
            "Expected bin/2 to be cleaned as well, even though the filesets "
            "changed since it was staged.",
        )

    def test_clean_old_stage_state(self):
        handler = self.load_part("part1")
        handler.makedirs()

        open(os.path.join(self.stage_dir, "1"), "w").close()

        handler.mark_done(steps.STAGE, None)

        self.assertTrue(os.path.exists(handler.plugin.partdir))

        handler.clean()

        self.assertFalse(os.path.exists(handler.plugin.partdir))

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


class TestCleanPrime:

    scenarios = [
        ("all", {"fileset": ["*"]}),
        ("no1", {"fileset": ["-1"]}),
        ("onlya", {"fileset": ["a"]}),
        ("onlybase", {"fileset": ["*", "-*/*"]}),
        ("only1a", {"fileset": ["1/a"]}),
        ("nostara", {"fileset": ["-*/a"]}),
    ]

    def test_clean_prime(self, monkeypatch, tmp_work_path, fileset):
        monkeypatch.setattr(file_utils, "get_snap_tool_path", lambda x: x)

        handler = load_part("test_part", part_properties={"prime": fileset})
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

        # Now prime them
        handler.prime()

        assert os.listdir(handler._project.prime_dir) != []

        handler.clean_prime({})

        assert os.listdir(handler._project.prime_dir) == []


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


class CleanPullTestCase(TestCase):
    def test_clean_pull_directory(self):
        handler = self.load_part("test-part")

        handler.pull()
        source_file = os.path.join(handler.part_source_dir, "source")
        open(source_file, "w").close()

        handler.clean_pull()

        # The source directory should now be gone
        self.assertFalse(os.path.exists(handler.part_source_dir))

    def test_clean_pull_symlink(self):
        real_source_directory = os.path.join(os.getcwd(), "src")
        os.mkdir(real_source_directory)

        handler = self.load_part("test-part", part_properties={"source": "src"})

        handler.pull()
        os.rmdir(handler.part_source_dir)
        os.symlink(real_source_directory, handler.part_source_dir)

        handler.clean_pull()

        # The source symlink should now be gone, but the real source should
        # still be there.
        self.assertFalse(os.path.exists(handler.part_source_dir))
        self.assertTrue(os.path.isdir(real_source_directory))


def test_clean_build(tmp_work_path):
    handler = load_part("test-part")

    handler.build()

    source_file = pathlib.Path(handler.part_source_dir) / "source"
    build_basedir = pathlib.Path(handler.plugin.build_basedir)
    source_file.touch()
    (build_basedir / "built").touch()
    (build_basedir / "built").touch()

    handler.clean_build()

    # Make sure the source file hasn't been touched
    assert source_file.exists()

    # Make sure the build directory is gone
    assert not build_basedir.exists()

    # Make sure the install directory is gone
    assert not pathlib.Path(handler.plugin.installdir).exists()
