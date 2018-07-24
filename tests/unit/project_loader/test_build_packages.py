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

import unittest
import unittest.mock
from textwrap import dedent

from testtools.matchers import Contains, Equals

from . import ProjectLoaderBaseTest


class VCSBuildPackagesTest(ProjectLoaderBaseTest):

    scenarios = [
        (
            "git",
            dict(
                source="git://github.com/ubuntu-core/snapcraft.git",
                expected_package="git",
            ),
        ),
        ("bzr", dict(source="lp:ubuntu-push", expected_package="bzr")),
        (
            "tar",
            dict(
                source=(
                    "https://github.com/ubuntu-core/snapcraft/archive/2.0.1.tar.gz"
                ),
                expected_package=None,
            ),
        ),
    ]

    def test_config_adds_vcs_packages_to_build_packages(self):
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
                source: {0}
                plugin: nil
            """
        ).format(self.source)

        project_config = self.make_snapcraft_project(snapcraft_yaml)

        if self.expected_package:
            self.assertThat(project_config.build_tools, Contains(self.expected_package))


class VCSBuildPackagesFromTypeTest(ProjectLoaderBaseTest):

    scenarios = [
        ("git", dict(type_="git", package="git")),
        ("hg", dict(type_="hg", package="mercurial")),
        ("mercurial", dict(type_="mercurial", package="mercurial")),
        ("bzr", dict(type_="bzr", package="bzr")),
        ("tar", dict(type_="tar", package=None)),
        ("svn", dict(type_="svn", package="subversion")),
        ("subversion", dict(type_="subversion", package="subversion")),
    ]

    def test_config_adds_vcs_packages_to_build_packages_from_types(self):
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
                source: http://something/somewhere
                source-type: {0}
                plugin: autotools
        """
        ).format(self.type_)

        project_config = self.make_snapcraft_project(snapcraft_yaml)

        if self.package:
            self.assertThat(project_config.build_tools, Contains(self.package))


class XCompileTest(ProjectLoaderBaseTest):
    def setUp(self):
        super().setUp()

        self.snapcraft_yaml = dedent(
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
        """
        )

    def test_config_adds_extra_build_tools_when_cross_compiling(self):
        project_kwargs = dict(target_deb_arch="armhf")
        with unittest.mock.patch(
            "platform.machine"
        ) as machine_mock, unittest.mock.patch("platform.architecture") as arch_mock:
            arch_mock.return_value = ("64bit", "ELF")
            machine_mock.return_value = "x86_64"
            project_config = self.make_snapcraft_project(
                self.snapcraft_yaml, project_kwargs
            )

        self.assertThat(
            project_config.parts.build_tools, Contains("gcc-arm-linux-gnueabihf")
        ),
        self.assertThat(
            project_config.parts.build_tools, Contains("libc6-dev-armhf-cross")
        ),

    def test_config_has_no_extra_build_tools_when_not_cross_compiling(self):
        project_config = self.make_snapcraft_project(self.snapcraft_yaml)

        self.assertThat(project_config.parts.build_tools, Equals(set()))
