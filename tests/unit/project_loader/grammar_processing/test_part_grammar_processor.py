# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017, 2018-2019 Canonical Ltd
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
import platform
from unittest import mock

from testscenarios import multiply_scenarios

from snapcraft import project
from snapcraft.internal import repo as snapcraft_repo
from snapcraft.internal.project_loader.grammar_processing import PartGrammarProcessor
from snapcraft.internal.project_loader.grammar_processing import (
    _part_grammar_processor as processor,
)


def load_tests(loader, tests, ignore):
    if project.Project().deb_arch == "amd64":
        tests.addTests(doctest.DocTestSuite(processor))
    return tests


class TestPartGrammarSource:

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

    def test(
        self,
        monkeypatch,
        host_arch,
        target_arch,
        properties,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        repo = mock.Mock()
        plugin = mock.Mock()
        plugin.properties = properties.copy()

        expected_arch = dict(
            expected_amd64=expected_amd64,
            expected_i386=expected_i386,
            expected_armhf=expected_armhf,
        )
        assert (
            PartGrammarProcessor(
                plugin=plugin,
                properties=plugin.properties,
                project=project.Project(target_deb_arch=target_arch),
                repo=repo,
            ).get_source()
            == expected_arch[f"expected_{target_arch}"]
        )

        # Verify that the original properties haven't changed
        assert plugin.properties == properties


class TestPartGrammarBuildAndStageSnaps:

    source_scenarios = [
        (
            "empty",
            {
                "snaps": "",
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": set(),
            },
        ),
        (
            "single item",
            {
                "snaps": ["foo"],
                "expected_amd64": {"foo"},
                "expected_i386": {"foo"},
                "expected_armhf": {"foo"},
            },
        ),
        (
            "on amd64",
            {
                "snaps": [{"on amd64": ["foo"]}],
                "expected_amd64": {"foo"},
                "expected_i386": set(),
                "expected_armhf": {"foo"},  # 'on' cares about host, not target
            },
        ),
        (
            "try",
            {
                "snaps": [{"try": ["hello"]}],
                "expected_amd64": {"hello"},
                "expected_i386": {"hello"},
                "expected_armhf": {"hello"},
            },
        ),
        (
            "try optional",
            {
                "snaps": [{"try": ["-invalid-"]}],
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": set(),
            },
        ),
        (
            "to armhf",
            {
                "snaps": [{"to armhf": ["foo"]}],
                "expected_amd64": set(),
                "expected_i386": set(),
                "expected_armhf": {"foo"},
            },
        ),
        (
            "on amd64 to armhf",
            {
                "snaps": [{"on amd64 to armhf": "foo"}],
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

    def test_snaps(
        self,
        monkeypatch,
        host_arch,
        target_arch,
        snaps,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))
        monkeypatch.setattr(
            snapcraft_repo.snaps.SnapPackage,
            "is_valid_snap",
            lambda x: "invalid" not in x,
        )

        repo = mock.Mock()

        class Plugin:
            build_snaps = snaps
            stage_snaps = snaps

        plugin = Plugin()
        processor = PartGrammarProcessor(
            plugin=plugin,
            properties={
                "build-snaps": {"plugin-preferred"},
                "stage-snaps": "plugin-preferred",
            },
            project=project.Project(target_deb_arch=target_arch),
            repo=repo,
        )

        expected_arch = dict(
            expected_amd64=expected_amd64,
            expected_i386=expected_i386,
            expected_armhf=expected_armhf,
        )
        assert processor.get_build_snaps() == expected_arch[f"expected_{target_arch}"]
        assert processor.get_stage_snaps() == expected_arch[f"expected_{target_arch}"]

    def test_snaps_no_plugin_attribute(
        self,
        monkeypatch,
        host_arch,
        target_arch,
        snaps,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))
        monkeypatch.setattr(
            snapcraft_repo.snaps.SnapPackage,
            "is_valid_snap",
            lambda x: "invalid" not in x,
        )

        repo = mock.Mock()

        class Plugin:
            pass

        plugin = Plugin()
        processor = PartGrammarProcessor(
            plugin=plugin,
            properties={"build-snaps": snaps, "stage-snaps": snaps},
            project=project.Project(target_deb_arch=target_arch),
            repo=repo,
        )

        expected_arch = dict(
            expected_amd64=expected_amd64,
            expected_i386=expected_i386,
            expected_armhf=expected_armhf,
        )
        assert processor.get_build_snaps() == expected_arch[f"expected_{target_arch}"]
        assert processor.get_stage_snaps() == expected_arch[f"expected_{target_arch}"]


class TestPartGrammarStagePackages:

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

    def test_packages(
        self,
        monkeypatch,
        host_arch,
        target_arch,
        packages,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        repo = mock.Mock()

        class Plugin:
            build_packages = packages
            stage_packages = packages

        plugin = Plugin()
        processor = PartGrammarProcessor(
            plugin=plugin,
            properties={"stage-packages": "plugin-preferred"},
            project=project.Project(target_deb_arch=target_arch),
            repo=repo,
        )

        expected_arch = dict(
            expected_amd64=expected_amd64,
            expected_i386=expected_i386,
            expected_armhf=expected_armhf,
        )
        assert (
            processor.get_stage_packages() == expected_arch[f"expected_{target_arch}"]
        )

    def test_packages_plugin_no_attr(
        self,
        monkeypatch,
        host_arch,
        target_arch,
        packages,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        repo = mock.Mock()

        class Plugin:
            pass

        plugin = Plugin()
        processor = PartGrammarProcessor(
            plugin=plugin,
            properties={"stage-packages": packages},
            project=project.Project(target_deb_arch=target_arch),
            repo=repo,
        )

        expected_arch = dict(
            expected_amd64=expected_amd64,
            expected_i386=expected_i386,
            expected_armhf=expected_armhf,
        )
        assert (
            processor.get_stage_packages() == expected_arch[f"expected_{target_arch}"]
        )


class TestPartGrammarBuildPackages:

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
                "expected_armhf": {"foo"},
            },
        ),
        (
            "on amd64 to armhf",
            {
                "packages": [{"on amd64 to armhf": "foo"}],
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

    def test_packages(
        self,
        monkeypatch,
        host_arch,
        target_arch,
        packages,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        repo = mock.Mock()

        class Plugin:
            build_packages = packages
            stage_packages = packages

        plugin = Plugin()
        processor = PartGrammarProcessor(
            plugin=plugin,
            properties={"build-packages": {"plugin-preferred"}},
            project=project.Project(target_deb_arch=target_arch),
            repo=repo,
        )

        expected_arch = dict(
            expected_amd64=expected_amd64,
            expected_i386=expected_i386,
            expected_armhf=expected_armhf,
        )
        assert (
            processor.get_build_packages() == expected_arch[f"expected_{target_arch}"]
        )

    def test_packages_plugin_no_attr(
        self,
        monkeypatch,
        host_arch,
        target_arch,
        packages,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        repo = mock.Mock()

        class Plugin:
            pass

        plugin = Plugin()
        processor = PartGrammarProcessor(
            plugin=plugin,
            properties={"build-packages": packages},
            project=project.Project(target_deb_arch=target_arch),
            repo=repo,
        )

        expected_arch = dict(
            expected_amd64=expected_amd64,
            expected_i386=expected_i386,
            expected_armhf=expected_armhf,
        )
        assert (
            processor.get_build_packages() == expected_arch[f"expected_{target_arch}"]
        )
