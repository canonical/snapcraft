# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
from textwrap import dedent
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, HasLength

from . import LoadPartBaseTest, ProjectLoaderBaseTest
from snapcraft.project import Project
from snapcraft.internal import errors, deprecations, project_loader, remote_parts
from tests import fixture_setup


class TestParts(ProjectLoaderBaseTest):
    def make_snapcraft_project(self, parts):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part("part1", dict(plugin="nil"))
        for part_name, part in parts:
            snapcraft_yaml.update_part(part_name, part)
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        return project_loader.load_config(project)

    def test_get_parts_none(self):
        project_config = self.make_snapcraft_project([("part1", dict(plugin="nil"))])
        self.assertThat(project_config.parts.get_part("not-a-part"), Equals(None))

    def test_slash_warning(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(fake_logger)

        self.make_snapcraft_project([("part/1", dict(plugin="nil"))])
        self.assertThat(
            fake_logger.output,
            Contains(
                'DEPRECATED: Found a "/" in the name of the {!r} part'.format("part/1")
            ),
        )

    def test_snap_deprecation(self):
        """Test that using the 'snap' keyword results in a warning."""

        fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(fake_logger)
        self.make_snapcraft_project([("part1", dict(plugin="nil", snap=["foo"]))])

        self.assertThat(
            fake_logger.output, Contains(deprecations._deprecation_message("dn1"))
        )


class PartOrderTestCase(ProjectLoaderBaseTest):

    scenarios = [
        (
            "part1 then part2",
            {
                "contents": dedent(
                    """\
                name: test
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """
                ),
                "expected_order": ["part1", "part2"],
            },
        ),
        (
            "part2 then part1",
            {
                "contents": dedent(
                    """\
                name: test
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part2:
                    plugin: nil
                  part1:
                    plugin: nil
                """
                ),
                "expected_order": ["part1", "part2"],
            },
        ),
        (
            "single after",
            {
                "contents": dedent(
                    """\
                name: test
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part2:
                    plugin: nil
                    after: [part3]
                  part1:
                    plugin: nil
                  part3:
                    plugin: nil
                """
                ),
                "expected_order": ["part1", "part3", "part2"],
            },
        ),
        (
            "multiple after",
            {
                "contents": dedent(
                    """\
                name: test
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part2:
                    plugin: nil
                    after: [part3]
                  part1:
                    plugin: nil
                    after: [part3]
                  part3:
                    plugin: nil
                """
                ),
                "expected_order": ["part3", "part1", "part2"],
            },
        ),
    ]

    def test_part_order_consistency(self):
        """Test that parts are always processed in the same order."""
        project_config = self.make_snapcraft_project(self.contents)
        self.assertThat(project_config.all_parts, HasLength(len(self.expected_order)))

        for part, expected_name in zip(project_config.all_parts, self.expected_order):
            self.expectThat(part.name, Equals(expected_name))


class PluginLoadTest(LoadPartBaseTest):
    def test_plugin_loading(self):
        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
                stage-packages: [fswebcam]
            """
            )
        )

        self.mock_load_part.assert_called_with(
            "part1",
            "nil",
            {"stage-packages": ["fswebcam"], "plugin": "nil", "stage": [], "prime": []},
        )


class RemotePartTest(LoadPartBaseTest):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeParts())
        remote_parts.update()

    def test_composes_with_remote_parts(self):
        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
        """
            )
        )

        self.mock_load_part.assert_called_with(
            "part1",
            "go",
            {
                "source": "http://source.tar.gz",
                "plugin": "go",
                "stage": [],
                "prime": [],
            },
        )

    def test_composes_with_modified_remote_parts(self):
        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                stage-packages: [fswebcam]
        """
            )
        )

        self.mock_load_part.assert_called_with(
            "part1",
            "go",
            {
                "source": "http://source.tar.gz",
                "stage-packages": ["fswebcam"],
                "plugin": "go",
                "stage": [],
                "prime": [],
            },
        )

    def test_composes_with_remote_subpart(self):
        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                stage-packages: [fswebcam]
        """
            )
        )

        self.mock_load_part.assert_called_with(
            "part1",
            "go",
            {
                "source": "http://source.tar.gz",
                "stage-packages": ["fswebcam"],
                "plugin": "go",
                "stage": [],
                "prime": [],
            },
        )

    def test_chaining_remotes_not_locally_declared(self):
        """Test to verify we can load non locally declared chained remotes."""
        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                after: [curl]
              curl:
                after: [long-described-part]
              part2:
                plugin: nil
        """
            )
        )

    def test_composes_with_a_non_existent_remote_part(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              non-existing-part:
                stage-packages: [fswebcam]
        """
        )

        raised = self.assertRaises(
            errors.SnapcraftPartMissingError,
            self.make_snapcraft_project,
            snapcraft_yaml,
        )
        self.assertThat(raised.part_name, Equals("non-existing-part"))

    def test_after_is_an_undefined_part(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
                after: [non-existing-part]
        """
        )

        raised = self.assertRaises(
            errors.SnapcraftPartMissingError,
            self.make_snapcraft_project,
            snapcraft_yaml,
        )
        self.assertThat(raised.part_name, Equals("non-existing-part"))

    def test_uses_remote_part_from_after(self):
        def load_effect(*args, **kwargs):
            mock_part = mock.Mock()
            mock_part.code.build_packages = []
            mock_part.deps = []
            mock_part.name = args[0]

            return mock_part

        self.mock_load_part.side_effect = load_effect

        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                after:
                  - curl
                plugin: go
                stage-packages: [fswebcam]
            """
            )
        )

        call1 = mock.call(
            "curl",
            "autotools",
            {
                "plugin": "autotools",
                "stage": [],
                "prime": [],
                "source": "http://curl.org",
            },
        )
        call2 = mock.call(
            "part1",
            "go",
            {"plugin": "go", "stage": [], "prime": [], "stage-packages": ["fswebcam"]},
        )

        self.mock_load_part.assert_has_calls([call1, call2], any_order=True)
