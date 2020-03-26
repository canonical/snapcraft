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
import subprocess
from collections import namedtuple
from textwrap import dedent

import testscenarios
from testtools.matchers import Contains, FileContains, FileExists

from tests import fixture_setup, integration


class AssetTrackingTestCase(integration.TestCase):
    def test_pull(self):
        self.copy_project_to_cwd("asset-tracking")
        stage_version = self.set_stage_package_version(
            "snapcraft.yaml", part="asset-tracking", package="hello"
        )
        build_version = self.set_build_package_version(
            "snapcraft.yaml", part="asset-tracking", package="hello"
        )

        self.run_snapcraft("pull")

        state_file = os.path.join(self.parts_dir, "asset-tracking", "state", "pull")
        self.assertThat(state_file, FileExists())
        # Verify that the correct version of 'hello' is installed
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                assets:
                  build-packages:
                  - hello={}
                  build-snaps: []
                  source-details: null
                  stage-packages:
                  - hello={}
                """.format(
                            build_version, stage_version
                        )
                    )
                )
            ),
        )

    def test_pull_global_build_packages_are_excluded(self):
        """
        Ensure global build-packages are not included in each part's
        build-packages data.
        """
        self.copy_project_to_cwd("build-package-global")
        self.set_build_package_version(
            os.path.join("snap", "snapcraft.yaml"), part=None, package="haskell-doc"
        )
        self.run_snapcraft("pull")

        state_file = os.path.join(self.parts_dir, "empty-part", "state", "pull")
        self.assertThat(state_file, FileExists())
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                assets:
                  build-packages: []
                  build-snaps: []
                  source-details: null
                  stage-packages: []
                """
                    )
                )
            ),
        )

    def test_pull_build_package_with_any_architecture(self):
        self.copy_project_to_cwd("build-package")
        self.set_build_package_architecture(
            os.path.join("snap", "snapcraft.yaml"),
            part="hello",
            package="hello",
            architecture="any",
        )
        self.run_snapcraft("pull")

        state_file = os.path.join(self.parts_dir, "hello", "state", "pull")
        self.assertThat(state_file, FileExists())
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                assets:
                  build-packages:
                  - hello:any
                  build-snaps: []
                  source-details: null
                  stage-packages: []
                """
                    )
                )
            ),
        )

    def test_pull_with_virtual_build_package(self):
        virtual_package = "fortunes-off"
        self.addCleanup(subprocess.call, ["sudo", "apt-get", "remove", virtual_package])
        self.run_snapcraft("pull", "build-virtual-package")

        state_file = os.path.join("snap", ".snapcraft", "state")
        self.assertThat(state_file, FileExists())
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                  - {}={}
                """.format(
                            virtual_package,
                            integration.get_package_version(
                                virtual_package, self.distro_series, self.deb_arch
                            ),
                        )
                    )
                )
            ),
        )


TestDetail = namedtuple("TestDetail", ["field", "value"])


class GitAssetTrackingTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        (
            "plain",
            dict(
                part_name="git-part",
                source_branch="''",
                source_commit=True,
                source_tag="''",
            ),
        ),
        (
            "branch",
            dict(
                part_name="git-part-branch",
                source_branch="test-branch",
                source_commit=False,
                source_tag="''",
            ),
        ),
        (
            "tag",
            dict(
                part_name="git-part-tag",
                source_branch="''",
                source_commit=False,
                source_tag="feature-tag",
            ),
        ),
    ]

    def test_pull_git(self):
        repo_fixture = fixture_setup.GitRepo()
        self.useFixture(repo_fixture)
        project_dir = "asset-tracking-git"

        source_commit = repo_fixture.commit if self.source_commit else "''"

        self.run_snapcraft(["pull", self.part_name], project_dir)

        state_file = os.path.join(self.parts_dir, self.part_name, "state", "pull")
        self.assertThat(state_file, FileExists())
        # fall back to the commit if no other source option is provided
        # snapcraft.source.Git doesn't allow both a tag and a commit
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                assets:
                  build-packages: []
                  build-snaps: []
                  source-details:
                    source: git-source
                    source-branch: {source_branch}
                    source-checksum: ''
                    source-commit: {source_commit}
                    source-tag: {source_tag}
                  stage-packages: []
                """.format(
                            source_branch=self.source_branch,
                            source_commit=source_commit,
                            source_tag=self.source_tag,
                        )
                    )
                )
            ),
        )


class BazaarAssetTrackingTestCase(testscenarios.WithScenarios, integration.TestCase):
    scenarios = [
        ("plain", dict(part_name="bzr-part", source_commit=True, source_tag="''")),
        (
            "tag",
            dict(
                part_name="bzr-part-tag", source_commit=False, source_tag="feature-tag"
            ),
        ),
    ]

    def test_pull_bzr(self):
        repo_fixture = fixture_setup.BzrRepo("bzr-source")
        self.useFixture(repo_fixture)
        project_dir = "asset-tracking-bzr"

        source_commit = repo_fixture.commit if self.source_commit else ""

        self.run_snapcraft(["pull", self.part_name], project_dir)

        state_file = os.path.join(self.parts_dir, self.part_name, "state", "pull")
        self.assertThat(state_file, FileExists())
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                assets:
                  build-packages: []
                  build-snaps: []
                  source-details:
                    source: bzr-source
                    source-branch: null
                    source-commit: '{source_commit}'
                    source-tag: {source_tag}
                  stage-packages: []
                """.format(
                            source_commit=source_commit, source_tag=self.source_tag
                        )
                    )
                )
            ),
        )


class MercurialAssetTrackingTestCase(testscenarios.WithScenarios, integration.TestCase):
    scenarios = [
        ("plain", dict(part_name="hg-part", source_commit=True, source_tag="''")),
        (
            "tag",
            dict(
                part_name="hg-part-tag", source_commit=False, source_tag="feature-tag"
            ),
        ),
    ]

    def test_pull_hg(self):
        repo_fixture = fixture_setup.HgRepo("hg-source")
        self.useFixture(repo_fixture)
        project_dir = "asset-tracking-hg"

        source_commit = repo_fixture.commit if self.source_commit else "''"

        self.run_snapcraft(["pull", self.part_name], project_dir)

        state_file = os.path.join(self.parts_dir, self.part_name, "state", "pull")
        self.assertThat(state_file, FileExists())
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                assets:
                  build-packages: []
                  build-snaps: []
                  source-details:
                    source: hg-source
                    source-branch: ''
                    source-commit: {source_commit}
                    source-tag: {source_tag}
                  stage-packages: []
                """.format(
                            source_commit=source_commit, source_tag=self.source_tag
                        )
                    )
                )
            ),
        )


class SubversionAssetTrackingTestCase(integration.TestCase):
    def test_pull_svn(self):
        repo_fixture = fixture_setup.SvnRepo("svn-source")
        self.useFixture(repo_fixture)
        project_dir = "asset-tracking-svn"
        part_name = "svn-part"

        self.run_snapcraft(["pull", part_name], project_dir)

        state_file = os.path.join(self.parts_dir, part_name, "state", "pull")
        self.assertThat(state_file, FileExists())
        self.assertThat(state_file, FileExists())
        self.assertThat(
            state_file,
            FileContains(
                matcher=Contains(
                    dedent(
                        """\
                assets:
                  build-packages: []
                  build-snaps: []
                  source-details:
                    source: svn-source
                    source-branch: null
                    source-commit: '{source_commit}'
                    source-tag: null
                  stage-packages: []
                """.format(
                            source_commit=repo_fixture.commit
                        )
                    )
                )
            ),
        )
