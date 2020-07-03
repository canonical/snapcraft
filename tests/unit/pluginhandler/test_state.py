# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) Canonical Ltd
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
from collections import OrderedDict
from unittest.mock import call, patch

import fixtures
from testtools.matchers import Contains, Equals

from snapcraft import extractors, plugins
from snapcraft.internal import elf, errors, states, steps
from tests import fixture_setup, unit


class StateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.get_pull_properties_mock = self.useFixture(
            fixtures.MockPatch(
                "snapcraft.plugins.v1.PluginV1.get_pull_properties", return_value=[]
            )
        ).mock

        self.get_build_properties_mock = self.useFixture(
            fixtures.MockPatch(
                "snapcraft.plugins.v1.PluginV1.get_build_properties", return_value=[]
            )
        ).mock

        self.handler = self.load_part("test_part")
        self.handler.makedirs()

        self.get_elf_files_mock = self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.elf.get_elf_files", return_value=frozenset()
            )
        ).mock

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.xattrs.read_origin_stage_package", return_value=None
            )
        )


class PullStateTestCase(StateBaseTestCase):
    def test_pull_build_packages_without_grammar_properties(self):
        self.handler = self.load_part(
            "test_part", part_properties={"build-packages": ["package1"]}
        )
        self.handler.mark_pull_done()
        state = self.handler.get_pull_state()

        self.assertTrue(type(state) is states.PullState)
        self.assertThat(state.assets.get("build-packages"), Equals({"package1"}))

    def test_pull_build_packages_with_grammar_properties(self):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "build-packages": [
                    {"on amd64": ["package1"]},
                    {"on i386": ["package2"]},
                    "package3",
                ],
                "build-snaps": [
                    {"on amd64": ["snap1"]},
                    {"on i386": ["snap2"]},
                    "snap3",
                ],
            },
        )
        self.handler.mark_pull_done()
        state = self.handler.get_pull_state()

        self.assertTrue(type(state) is states.PullState)
        self.assertThat(
            state.assets.get("build-packages"), Equals(set(["package1", "package3"]))
        )
        self.assertThat(
            state.assets.get("build-snaps"), Equals(set(["snap1", "snap3"]))
        )


class StateTestCase(StateBaseTestCase):
    @patch("snapcraft.internal.repo.Repo")
    def test_pull_state(self, repo_mock):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        repo_mock.get_installed_build_packages.return_value = []

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(11))
        for expected in [
            "source",
            "source-branch",
            "source-commit",
            "source-depth",
            "source-subdir",
            "source-tag",
            "source-type",
            "plugin",
            "stage-packages",
            "parse-info",
            "override-pull",
        ]:
            self.assertTrue(expected in state.properties)
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    @patch("snapcraft.internal.repo.Repo")
    def test_pull_state_with_extracted_metadata(self, repo_mock):
        self.handler = self.load_part(
            "test_part",
            part_properties={"source": ".", "parse-info": ["metadata-file"]},
        )

        # Create metadata file
        open("metadata-file", "w").close()

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(
                common_id="test_common_id",
                summary="test summary",
                description="test description",
                icon="/test/path",
                desktop_file_paths=[
                    "usr/share/applications/com.example.test/app.desktop"
                ],
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        repo_mock.get_installed_build_packages.return_value = []

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(11))
        for expected in [
            "source",
            "source-branch",
            "source-commit",
            "source-depth",
            "source-subdir",
            "source-tag",
            "source-type",
            "plugin",
            "stage-packages",
            "parse-info",
            "override-pull",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(state.project_options, Contains("deb_arch"))

        self.assertTrue(type(state.extracted_metadata) is OrderedDict)
        for expected in ("metadata", "files"):
            self.assertThat(state.extracted_metadata, Contains(expected))
        metadata = state.extracted_metadata["metadata"]
        self.assertThat(metadata.get_common_id(), Equals("test_common_id"))
        self.assertThat(metadata.get_summary(), Equals("test summary"))
        self.assertThat(metadata.get_description(), Equals("test description"))
        self.assertThat(metadata.get_icon(), Equals("/test/path"))
        self.assertThat(
            metadata.get_desktop_file_paths(),
            Equals(["usr/share/applications/com.example.test/app.desktop"]),
        )
        files = state.extracted_metadata["files"]
        self.assertThat(
            files, Equals([os.path.join(self.handler.part_source_dir, "metadata-file")])
        )

    @patch("snapcraft.internal.repo.Repo")
    def test_pull_state_with_scriptlet_metadata(self, repo_mock):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-pull": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        repo_mock.get_installed_build_packages.return_value = []

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(11))
        for expected in [
            "source",
            "source-branch",
            "source-commit",
            "source-depth",
            "source-subdir",
            "source-tag",
            "source-type",
            "plugin",
            "stage-packages",
            "parse-info",
            "override-pull",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-pull"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    def test_pull_state_with_properties(self):
        self.get_pull_properties_mock.return_value = ["foo"]
        self.handler.plugin.options.foo = "bar"
        self.handler._part_properties["foo"] = "bar"

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))
        state = self.handler.get_pull_state()

        self.assertTrue(state, "Expected pull to save state YAML")
        self.assertTrue(type(state) is states.PullState)
        self.assertTrue(type(state.properties) is OrderedDict)
        if isinstance(self.handler.plugin, plugins.v1.PluginV1):
            self.assertTrue("foo" in state.properties)
            self.assertThat(state.properties["foo"], Equals("bar"))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    @patch.object(plugins.v1.nil.NilPlugin, "clean_pull")
    def test_clean_pull_state(self, mock_clean_pull):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()

        self.handler.clean_pull()

        # Verify that the plugin had clean_pull() called
        if isinstance(self.handler.plugin, plugins.v1.PluginV1):
            mock_clean_pull.assert_called_once_with()

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

    def test_build_state(self):
        if not isinstance(self.handler.plugin, plugins.v1.PluginV1):
            self.skipTest("Test designed for PluginV1")

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(6))
        for expected in [
            "after",
            "build-attributes",
            "build-packages",
            "disable-parallel",
            "organize",
            "override-build",
        ]:
            self.assertTrue(expected in state.properties)
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    def test_build_state_with_extracted_metadata(self):
        self.handler = self.load_part(
            "test_part", part_properties={"parse-info": ["metadata-file"]}
        )

        # Create metadata file
        open(os.path.join(self.handler.part_source_dir, "metadata-file"), "w").close()

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(
                common_id="test_common_id",
                summary="test summary",
                description="test description",
                icon="/test/path",
                desktop_file_paths=[
                    "usr/share/applications/com.example.test/app.desktop"
                ],
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(6))
        for expected in [
            "after",
            "build-attributes",
            "build-packages",
            "disable-parallel",
            "organize",
            "override-build",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(state.project_options, Contains("deb_arch"))

        self.assertTrue(type(state.extracted_metadata) is OrderedDict)
        for expected in ("metadata", "files"):
            self.assertThat(state.extracted_metadata, Contains(expected))
        metadata = state.extracted_metadata["metadata"]
        self.assertThat(metadata.get_common_id(), Equals("test_common_id"))
        self.assertThat(metadata.get_summary(), Equals("test summary"))
        self.assertThat(metadata.get_description(), Equals("test description"))
        self.assertThat(metadata.get_icon(), Equals("/test/path"))
        self.assertThat(
            metadata.get_desktop_file_paths(),
            Equals(["usr/share/applications/com.example.test/app.desktop"]),
        )
        files = state.extracted_metadata["files"]
        self.assertThat(
            files, Equals([os.path.join(self.handler.part_build_dir, "metadata-file")])
        )

    def test_build_state_with_scriptlet_metadata(self):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-build": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()
        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(6))
        for expected in [
            "after",
            "build-attributes",
            "build-packages",
            "disable-parallel",
            "organize",
            "override-build",
        ]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-build"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    def test_build_state_with_properties(self):
        self.get_build_properties_mock.return_value = ["foo"]
        self.handler.plugin.options.foo = "bar"
        self.handler._part_properties = {"foo": "bar"}

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.build()

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        state = self.handler.get_build_state()

        self.assertTrue(state, "Expected build to save state YAML")
        self.assertTrue(type(state) is states.BuildState)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertTrue("foo" in state.properties)
        self.assertThat(state.properties["foo"], Equals("bar"))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertTrue("deb_arch" in state.project_options)

    @patch.object(plugins.v1.nil.NilPlugin, "clean_build")
    def test_clean_build_state(self, mock_clean_build):
        if not isinstance(self.handler.plugin, plugins.v1.PluginV1):
            self.skipTest("Test designed for PluginV1")

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)

        self.handler.mark_done(steps.PULL)
        self.handler.build()

        self.handler.clean_build()

        # Verify that the plugin had clean_build() called for v1.
        if isinstance(self.handler.plugin, plugins.v1.PluginV1):
            mock_clean_build.assert_called_once_with()

        self.assertThat(self.handler.latest_step(), Equals(steps.PULL))
        self.assertThat(self.handler.next_step(), Equals(steps.BUILD))

    def test_stage_state(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        state = self.handler.get_stage_state()

        self.assertTrue(state, "Expected stage to save state YAML")
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(2))
        self.assertTrue("bin/1" in state.files)
        self.assertTrue("bin/2" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertTrue("stage" in state.properties)
        self.assertThat(state.properties["stage"], Equals(["*"]))
        self.assertTrue("filesets" in state.properties)
        self.assertThat(state.properties["filesets"], Equals({}))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    def test_stage_state_with_scriptlet_metadata(self):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-stage": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()
        self.handler.build()
        self.handler.stage()

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        state = self.handler.get_stage_state()

        self.assertTrue(state, "Expected stage to save state YAML")
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(3))
        for expected in ["stage", "filesets", "override-stage"]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-stage"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    def test_stage_state_with_stage_keyword(self):
        self.handler.plugin.options.stage = ["bin/1"]
        self.handler._part_properties["stage"] = ["bin/1"]

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        state = self.handler.get_stage_state()

        self.assertTrue(state, "Expected stage to save state YAML")
        self.assertTrue(type(state) is states.StageState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(1))
        self.assertTrue("bin/1" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertTrue("stage" in state.properties)
        self.assertThat(state.properties["stage"], Equals(["bin/1"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))

    def test_clean_stage_state(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.stage_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)

        self.handler.mark_done(
            steps.STAGE, states.StageState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_stage({})

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        self.assertFalse(os.path.exists(bindir))

    def test_clean_stage_state_multiple_parts(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        bindir = os.path.join(self.stage_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()
        open(os.path.join(bindir, "3"), "w").close()

        self.handler.mark_done(steps.BUILD)

        self.handler.mark_done(
            steps.STAGE, states.StageState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_stage({})

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertFalse(os.path.exists(os.path.join(bindir, "2")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "3")),
            "Expected 'bin/3' to remain as it wasn't staged by this part",
        )

    def test_clean_stage_state_common_files(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        bindir = os.path.join(self.stage_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)

        self.handler.mark_done(
            steps.STAGE, states.StageState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_stage({"other_part": states.StageState({"bin/2"}, {"bin"})})

        self.assertThat(self.handler.latest_step(), Equals(steps.BUILD))
        self.assertThat(self.handler.next_step(), Equals(steps.STAGE))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "2")),
            "Expected 'bin/2' to remain as it's required by other parts",
        )

    def test_clean_stage_old_state(self):
        self.handler.mark_done(steps.STAGE, None)
        raised = self.assertRaises(
            errors.MissingStateCleanError, self.handler.clean_stage, {}
        )

        self.assertThat(raised.step, Equals(steps.STAGE))

    @patch("shutil.copy")
    def test_prime_state(self, mock_copy):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler._project.prime_dir, {"bin/1", "bin/2"}
        )
        self.assertFalse(mock_copy.called)

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(2))
        self.assertTrue("bin/1" in state.files)
        self.assertTrue("bin/2" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertThat(len(state.dependency_paths), Equals(0))
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["*"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    def test_prime_state_with_scriptlet_metadata(self):
        self.handler = self.load_part(
            "test_part",
            part_properties={
                "override-prime": "snapcraftctl set-version override-version"
            },
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.pull()
        self.handler.build()
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        state = self.handler.get_prime_state()

        self.assertTrue(state, "Expected prime to save state YAML")
        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.properties), Equals(2))
        for expected in ["prime", "override-prime"]:
            self.assertThat(state.properties, Contains(expected))
        self.assertThat(
            state.properties["override-prime"],
            Equals("snapcraftctl set-version override-version"),
        )

        metadata = state.scriptlet_metadata
        self.assertThat(metadata.get_version(), Equals("override-version"))

    @patch("shutil.copy")
    def test_prime_state_with_stuff_already_primed(self, mock_copy):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        bindir = os.path.join(self.handler._project.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        # bin/2 shouldn't be in this list as it was already primed by another
        # part.
        self.get_elf_files_mock.assert_called_once_with(
            self.handler._project.prime_dir, {"bin/1"}
        )
        self.assertFalse(mock_copy.called)

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(1))
        self.assertTrue("bin/1" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertThat(len(state.dependency_paths), Equals(0))
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["*"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    @patch("snapcraft.internal.elf.ElfFile._extract_attributes")
    @patch("snapcraft.internal.elf.ElfFile.load_dependencies")
    @patch("snapcraft.internal.pluginhandler._migrate_files")
    def test_prime_state_with_dependencies(
        self, mock_migrate_files, mock_load_dependencies, mock_get_symbols
    ):
        mock_load_dependencies.return_value = {
            "/foo/bar/baz",
            "{}/lib1/installed".format(self.handler.part_install_dir),
            "{}/lib2/staged".format(self.handler._project.stage_dir),
            "{}/lib3/primed".format(self.handler._project.prime_dir),
        }
        self.get_elf_files_mock.return_value = frozenset(
            [
                elf.ElfFile(
                    path=os.path.join(self.handler._project.prime_dir, "bin", "1")
                ),
                elf.ElfFile(
                    path=os.path.join(self.handler._project.prime_dir, "bin", "2")
                ),
            ]
        )
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()

        # Resetting for test clarity
        mock_migrate_files.reset_mock()

        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler._project.prime_dir, {"bin/1", "bin/2"}
        )
        mock_migrate_files.assert_has_calls(
            [
                call(
                    {"bin/1", "bin/2"},
                    {"bin"},
                    self.handler._project.stage_dir,
                    self.handler._project.prime_dir,
                )
            ]
        )

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(2))
        self.assertThat(state.dependency_paths, Equals({"lib3"}))
        self.assertTrue("bin/1" in state.files)
        self.assertTrue("bin/2" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["*"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    @patch("snapcraft.internal.elf.ElfFile._extract_attributes")
    @patch("snapcraft.internal.elf.ElfFile.load_dependencies")
    @patch("snapcraft.internal.pluginhandler._migrate_files")
    def test_prime_state_missing_libraries(
        self, mock_migrate_files, mock_load_dependencies, mock_get_symbols
    ):
        self.handler = self.load_part("test_part")

        self.get_elf_files_mock.return_value = frozenset(
            [
                elf.ElfFile(
                    path=os.path.join(self.handler._project.prime_dir, "bin", "file")
                )
            ]
        )
        # Pretend we found a system dependency, as well as a part and stage
        # dependency.
        mock_load_dependencies.return_value = set(
            [
                "/foo/bar/baz",
                "{}/lib1/installed".format(self.handler.part_install_dir),
                "{}/lib2/staged".format(self.handler._project.stage_dir),
                "{}/lib3/primed".format(self.handler._project.prime_dir),
            ]
        )

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "file"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        mock_migrate_files.reset_mock()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler._project.prime_dir, {"bin/file"}
        )
        # Verify that only the part's files were migrated-- not the system
        # dependency.
        mock_migrate_files.assert_called_once_with(
            {"bin/file"},
            {"bin"},
            self.handler._project.stage_dir,
            self.handler._project.prime_dir,
        )

        state = self.handler.get_prime_state()

        # Verify that only the primed paths were captured.
        # The rest should be considered missing.
        self.assertThat(state.dependency_paths, Equals({"lib3"}))

    @patch("snapcraft.internal.elf.ElfFile._extract_attributes")
    @patch("snapcraft.internal.elf.ElfFile.load_dependencies")
    @patch("snapcraft.internal.pluginhandler._migrate_files")
    def test_prime_state_with_shadowed_dependencies(
        self, mock_migrate_files, mock_load_dependencies, mock_get_symbols
    ):
        self.get_elf_files_mock.return_value = frozenset([elf.ElfFile(path="bin/1")])
        mock_load_dependencies.return_value = {
            f"{self.handler._project.prime_dir}/foo/bar/baz"
        }

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        foobardir = os.path.join(self.handler.part_install_dir, "foo", "bar")
        os.makedirs(bindir)
        os.makedirs(foobardir)

        # Make a "binary" as well as a "library" at the same path as the one on
        # the system
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(foobardir, "baz"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        mock_migrate_files.reset_mock()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler._project.prime_dir, {"bin/1", "foo/bar/baz"}
        )
        mock_migrate_files.assert_called_once_with(
            {"bin/1", "foo/bar/baz"},
            {"bin", "foo", "foo/bar"},
            self.handler._project.stage_dir,
            self.handler._project.prime_dir,
        )

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertThat(state.dependency_paths, Equals({"foo/bar"}))

    @patch("shutil.copy")
    def test_prime_state_with_prime_keyword(self, mock_copy):
        self.handler = self.load_part("test_part", part_properties={"prime": ["bin/1"]})

        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        bindir = os.path.join(self.handler.part_install_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.BUILD)
        self.handler.stage()
        self.handler.prime()

        self.assertThat(self.handler.latest_step(), Equals(steps.PRIME))
        self.assertRaises(errors.NoNextStepError, self.handler.next_step)
        self.get_elf_files_mock.assert_called_once_with(
            self.handler._project.prime_dir, {"bin/1"}
        )
        self.assertFalse(mock_copy.called)

        state = self.handler.get_prime_state()

        self.assertTrue(type(state) is states.PrimeState)
        self.assertTrue(type(state.files) is set)
        self.assertTrue(type(state.directories) is set)
        self.assertTrue(type(state.dependency_paths) is set)
        self.assertTrue(type(state.properties) is OrderedDict)
        self.assertThat(len(state.files), Equals(1))
        self.assertTrue("bin/1" in state.files)
        self.assertThat(len(state.directories), Equals(1))
        self.assertTrue("bin" in state.directories)
        self.assertThat(len(state.dependency_paths), Equals(0))
        self.assertTrue("prime" in state.properties)
        self.assertThat(state.properties["prime"], Equals(["bin/1"]))
        self.assertTrue(type(state.project_options) is OrderedDict)
        self.assertThat(len(state.project_options), Equals(0))

    def test_clean_prime_state(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.STAGE)

        self.handler.mark_done(
            steps.PRIME, states.PrimeState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_prime({})

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        self.assertFalse(os.path.exists(bindir))

    def test_clean_prime_state_inconsistent_files(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))

        self.handler.mark_done(steps.STAGE)

        self.handler.mark_done(
            steps.PRIME, states.PrimeState({"bin/1", "bin/2"}, {"bin", "not-there"})
        )

        # Only create a subset of the files and directories that were marked as primed
        bindir = os.path.join(self.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()

        self.handler.clean_prime({})

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        self.assertFalse(os.path.exists(bindir))

    def test_clean_prime_state_multiple_parts(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()
        open(os.path.join(bindir, "3"), "w").close()

        self.handler.mark_done(steps.STAGE)

        self.handler.mark_done(
            steps.PRIME, states.PrimeState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_prime({})

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertFalse(os.path.exists(os.path.join(bindir, "2")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "3")),
            "Expected 'bin/3' to remain as it wasn't primed by this part",
        )

    def test_clean_prime_state_common_files(self):
        self.assertRaises(errors.NoLatestStepError, self.handler.latest_step)
        self.assertThat(self.handler.next_step(), Equals(steps.PULL))
        bindir = os.path.join(self.prime_dir, "bin")
        os.makedirs(bindir)
        open(os.path.join(bindir, "1"), "w").close()
        open(os.path.join(bindir, "2"), "w").close()

        self.handler.mark_done(steps.STAGE)

        self.handler.mark_done(
            steps.PRIME, states.PrimeState({"bin/1", "bin/2"}, {"bin"})
        )

        self.handler.clean_prime({"other_part": states.PrimeState({"bin/2"}, {"bin"})})

        self.assertThat(self.handler.latest_step(), Equals(steps.STAGE))
        self.assertThat(self.handler.next_step(), Equals(steps.PRIME))
        self.assertFalse(os.path.exists(os.path.join(bindir, "1")))
        self.assertTrue(
            os.path.exists(os.path.join(bindir, "2")),
            "Expected 'bin/2' to remain as it's required by other parts",
        )

    def test_clean_prime_old_state(self):
        self.handler.mark_done(steps.PRIME, None)
        raised = self.assertRaises(
            errors.MissingStateCleanError, self.handler.clean_prime, {}
        )

        self.assertThat(raised.step, Equals(steps.PRIME))


class TestStateFileMigration:
    scenarios = [(step.name, dict(step=step)) for step in steps.STEPS]

    def test_state_file_migration(self, tmp_work_path, step):
        part_name = "foo"

        part_dir = tmp_work_path / "parts" / part_name
        part_dir.mkdir(parents=True)
        with (part_dir / "state").open("w") as state_file:
            print(step.name, file=state_file, end="")

        handler = unit.load_part(part_name)

        assert handler.latest_step() == step
