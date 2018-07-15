# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017, 2018 Canonical Ltd
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

import doctest
from unittest import mock

from testtools.matchers import Equals
from testscenarios import multiply_scenarios

from snapcraft import project
from snapcraft.internal.project_loader.grammar_processing import (
    PartGrammarProcessor,
    _part_grammar_processor as processor,
)
from tests import unit
from tests.fixture_setup import FakeSnapd


def load_tests(loader, tests, ignore):
    if project.Project().deb_arch == "amd64":
        tests.addTests(doctest.DocTestSuite(processor))
    return tests


class PartGrammarSourceTestCase(unit.TestCase):

    source_scenarios = [
        (
            "empty",
            {
                "properties": {"source": ""},
                "expected_amd64": "",
                "expected_i386": "",
                "expected_armhf": "",
            },
        ),
        (
            "plain string",
            {
                "properties": {"source": "foo"},
                "expected_amd64": "foo",
                "expected_i386": "foo",
                "expected_armhf": "foo",
            },
        ),
        (
            "on amd64",
            {
                "properties": {"source": [{"on amd64": "foo"}]},
                "expected_amd64": "foo",
                "expected_i386": "",
                "expected_armhf": "foo",  # 'on' cares about host arch, not target
            },
        ),
        (
            "on i386",
            {
                "properties": {"source": [{"on i386": "foo"}]},
                "expected_amd64": "",
                "expected_i386": "foo",
                "expected_armhf": "",
            },
        ),
        (
            "on armhf",
            {
                "properties": {"source": [{"on armhf": "foo"}]},
                "expected_amd64": "",
                "expected_i386": "",
                "expected_armhf": "",
            },
        ),
        (
            "on amd64 with else",
            {
                "properties": {"source": [{"on amd64": "foo"}, {"else": "bar"}]},
                "expected_amd64": "foo",
                "expected_i386": "bar",
                "expected_armhf": "foo",  # 'on' cares about host arch, not target
            },
        ),
        (
            "on i386 with else",
            {
                "properties": {"source": [{"on i386": "foo"}, {"else": "bar"}]},
                "expected_amd64": "bar",
                "expected_i386": "foo",
                "expected_armhf": "bar",
            },
        ),
        # 'on' only cares about the host arch, not target
        (
            "on armhf with else",
            {
                "properties": {"source": [{"on armhf": "foo"}, {"else": "bar"}]},
                "expected_amd64": "bar",
                "expected_i386": "bar",
                "expected_armhf": "bar",
            },
        ),
        (
            "try",
            {
                "properties": {"source": [{"try": "foo"}]},
                "expected_amd64": "foo",
                "expected_i386": "foo",
                "expected_armhf": "foo",
            },
        ),
        (
            "to amd64",
            {
                "properties": {"source": [{"to amd64": "foo"}]},
                "expected_amd64": "foo",
                "expected_i386": "",
                "expected_armhf": "",
            },
        ),
        (
            "to i386",
            {
                "properties": {"source": [{"to i386": "foo"}]},
                "expected_amd64": "",
                "expected_i386": "foo",
                "expected_armhf": "",
            },
        ),
        (
            "to armhf",
            {
                "properties": {"source": [{"to armhf": "foo"}]},
                "expected_amd64": "",
                "expected_i386": "",
                "expected_armhf": "foo",
            },
        ),
        (
            "to amd64 with else",
            {
                "properties": {"source": [{"to amd64": "foo"}, {"else": "bar"}]},
                "expected_amd64": "foo",
                "expected_i386": "bar",
                "expected_armhf": "bar",
            },
        ),
        (
            "to i386 with else",
            {
                "properties": {"source": [{"to i386": "foo"}, {"else": "bar"}]},
                "expected_amd64": "bar",
                "expected_i386": "foo",
                "expected_armhf": "bar",
            },
        ),
        (
            "to armhf with else",
            {
                "properties": {"source": [{"to armhf": "foo"}, {"else": "bar"}]},
                "expected_amd64": "bar",
                "expected_i386": "bar",
                "expected_armhf": "foo",
            },
        ),
        (
            "on amd64 to armhf",
            {
                "properties": {"source": [{"on amd64 to armhf": "foo"}]},
                "expected_amd64": "",
                "expected_i386": "",
                "expected_armhf": "foo",
            },
        ),
        (
            "on amd64 to armhf with else",
            {
                "properties": {
                    "source": [{"on amd64 to armhf": "foo"}, {"else": "bar"}]
                },
                "expected_amd64": "bar",
                "expected_i386": "bar",
                "expected_armhf": "foo",
            },
        ),
    ]

    arch_scenarios = [
        ("amd64", {"host_arch": "x86_64", "target_arch": "amd64"}),
        ("i386", {"host_arch": "i686", "target_arch": "i386"}),
        ("amd64 to armhf", {"host_arch": "x86_64", "target_arch": "armhf"}),
    ]

    scenarios = multiply_scenarios(source_scenarios, arch_scenarios)

    @mock.patch("platform.architecture")
    @mock.patch("platform.machine")
    def test_source(self, platform_machine_mock, platform_architecture_mock):
        platform_machine_mock.return_value = self.host_arch
        platform_architecture_mock.return_value = ("64bit", "ELF")

        repo = mock.Mock()
        plugin = mock.Mock()
        plugin.properties = self.properties.copy()
        expected = getattr(self, "expected_{}".format(self.target_arch))
        self.assertThat(
            PartGrammarProcessor(
                plugin=plugin,
                properties=plugin.properties,
                project=project.Project(target_deb_arch=self.target_arch),
                repo=repo,
            ).get_source(),
            Equals(expected),
        )
        # Verify that the original properties haven't changed
        self.assertThat(plugin.properties, Equals(self.properties))


class PartGrammarBuildSnapsTestCase(unit.TestCase):

    source_scenarios = [
        (
            "empty",
            {
                "build_snaps": "",
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": set(),
            },
        ),
        (
            "single item",
            {
                "build_snaps": ["foo"],
                "expected_amd64": {"foo"},
                "expected_i386": {"foo"},
                "expected_armhf": {"foo"},
            },
        ),
        (
            "on amd64",
            {
                "build_snaps": [{"on amd64": ["foo"]}],
                "expected_amd64": {"foo"},
                "expected_i386": set(),
                "expected_armhf": {"foo"},  # 'on' cares about host, not target
            },
        ),
        (
            "try",
            {
                "build_snaps": [{"try": ["hello"]}],
                "expected_amd64": {"hello"},
                "expected_i386": {"hello"},
                "expected_armhf": {"hello"},
            },
        ),
        (
            "try optional",
            {
                "build_snaps": [{"try": ["-invalid-"]}],
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": set(),
            },
        ),
        (
            "to armhf",
            {
                "build_snaps": [{"to armhf": ["foo"]}],
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": {"foo"},
            },
        ),
        (
            "on amd64 to armhf",
            {
                "build_snaps": [{"on amd64 to armhf": "foo"}],
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": {"foo"},
            },
        ),
    ]

    arch_scenarios = [
        ("amd64", {"host_arch": "x86_64", "target_arch": "amd64"}),
        ("i386", {"host_arch": "i686", "target_arch": "i386"}),
        ("amd64 to armhf", {"host_arch": "x86_64", "target_arch": "armhf"}),
    ]

    scenarios = multiply_scenarios(source_scenarios, arch_scenarios)

    def setUp(self):
        super().setUp()

        fake_snapd = FakeSnapd()
        fake_snapd.find_result = [
            {"hello": {"channels": {"latest/stable": {"confinement": "devmode"}}}}
        ]
        self.useFixture(fake_snapd)

    @mock.patch("platform.architecture")
    @mock.patch("platform.machine")
    def test_build_snaps(self, platform_machine_mock, platform_architecture_mock):
        platform_machine_mock.return_value = self.host_arch
        platform_architecture_mock.return_value = ("64bit", "ELF")

        repo = mock.Mock()
        plugin = mock.Mock()
        plugin.build_snaps = self.build_snaps
        expected = getattr(self, "expected_{}".format(self.target_arch))
        self.assertThat(
            PartGrammarProcessor(
                plugin=plugin,
                properties={},
                project=project.Project(target_deb_arch=self.target_arch),
                repo=repo,
            ).get_build_snaps(),
            Equals(expected),
        )


class PartGrammarBuildAndStagePackagesTestCase(unit.TestCase):

    source_scenarios = [
        (
            "empty",
            {
                "packages": "",
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": set(),
            },
        ),
        (
            "single item",
            {
                "packages": ["foo"],
                "expected_amd64": {"foo"},
                "expected_i386": {"foo"},
                "expected_armhf": {"foo"},
            },
        ),
        (
            "on amd64",
            {
                "packages": [{"on amd64": ["foo"]}],
                "expected_amd64": {"foo"},
                "expected_i386": set(),
                "expected_armhf": {"foo"},  # 'on' cares about host, not target
            },
        ),
        (
            "try",
            {
                "packages": [{"try": ["foo"]}],
                "expected_amd64": {"foo"},
                "expected_i386": {"foo"},
                "expected_armhf": {"foo"},
            },
        ),
        (
            "to armhf",
            {
                "packages": [{"to armhf": ["foo"]}],
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": {"foo:armhf"},
            },
        ),
        (
            "on amd64 to armhf",
            {
                "packages": [{"on amd64 to armhf": "foo"}],
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": {"foo:armhf"},
            },
        ),
    ]

    arch_scenarios = [
        ("amd64", {"host_arch": "x86_64", "target_arch": "amd64"}),
        ("i386", {"host_arch": "i686", "target_arch": "i386"}),
        ("amd64 to armhf", {"host_arch": "x86_64", "target_arch": "armhf"}),
    ]

    scenarios = multiply_scenarios(source_scenarios, arch_scenarios)

    @mock.patch("platform.architecture")
    @mock.patch("platform.machine")
    def test_packages(self, platform_machine_mock, platform_architecture_mock):
        platform_machine_mock.return_value = self.host_arch
        platform_architecture_mock.return_value = ("64bit", "ELF")

        repo = mock.Mock()
        plugin = mock.Mock()
        plugin.build_packages = self.packages
        plugin.stage_packages = self.packages
        expected = getattr(self, "expected_{}".format(self.target_arch))
        processor = PartGrammarProcessor(
            plugin=plugin,
            properties={},
            project=project.Project(target_deb_arch=self.target_arch),
            repo=repo,
        )
        self.assertThat(processor.get_build_packages(), Equals(expected))
        self.assertThat(processor.get_stage_packages(), Equals(expected))
