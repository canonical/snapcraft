# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

import filecmp
import os
import shutil
import subprocess
import sys
import tempfile
import yaml

import apt
import fixtures
import testscenarios
from testtools.matchers import Contains, Equals

from tests.integration import repo
from tests import integration, fixture_setup


class AssetRecordingBaseTestCase(integration.TestCase):
    """Test that the prime step records an annotated manifest.yaml

    The annotated file will be in prime/snap/manifest.yaml.

    """

    def setUp(self):
        super().setUp()
        # The combination of snapd, lxd and armhf does not currently work.
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")

        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_INFO", "1"))

        try:
            subprocess.check_output(["snap", "list", "review-tools"])
        except subprocess.CalledProcessError:
            subprocess.check_call(["sudo", "snap", "install", "review-tools", "--edge"])

    def assert_review_passes(self, snap_file: str) -> None:
        # review-tools do not really have access to tmp, let's assume it can look
        # in its own snap directory and that that does not change as we cannot
        # query what the data store is for a given snap.
        review_tools_common_dir = os.path.expanduser(
            os.path.join("~", "snap", "review-tools", "common")
        )
        os.makedirs(review_tools_common_dir, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=review_tools_common_dir) as temp_snap_file:
            shutil.copyfile(snap_file, temp_snap_file.name)
            try:
                subprocess.check_output(
                    ["review-tools.snap-review", temp_snap_file.name]
                )
            except subprocess.CalledProcessError as call_error:
                self.fail(
                    "{!r} does not pass the review:\n{}".format(
                        snap_file, call_error.stdout.decode()
                    )
                )


class SnapcraftYamlRecordingTestCase(AssetRecordingBaseTestCase):
    def test_prime_records_snapcraft_yaml_copy(self):
        self.run_snapcraft("prime", project_dir="basic")
        source_snapcraft_yaml_path = os.path.join("snap", "snapcraft.yaml")
        recorded_snapcraft_yaml_path = os.path.join(
            self.prime_dir, "snap", "snapcraft.yaml"
        )
        self.assertTrue(
            filecmp.cmp(
                source_snapcraft_yaml_path, recorded_snapcraft_yaml_path, shallow=False
            )
        )


class ManifestRecordingTestCase(AssetRecordingBaseTestCase):
    def test_prime_records_uname(self):
        self.run_snapcraft(["snap", "--output", "basic.snap"], project_dir="basic")

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        expected_uname = (
            subprocess.check_output(["uname", "-srvmpio"])
            .decode(sys.getfilesystemencoding())
            .strip()
        )
        self.assertThat(
            recorded_yaml["parts"]["dummy-part"]["uname"], Equals(expected_uname)
        )
        self.assert_review_passes("basic.snap")

    def test_prime_records_installed_packages(self):
        self.run_snapcraft(["snap", "--output", "basic.snap"], project_dir="basic")

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        with apt.Cache() as apt_cache:
            expected_package = "python3={}".format(
                apt_cache["python3"].installed.version
            )
        self.assertThat(
            recorded_yaml["parts"]["dummy-part"]["installed-packages"],
            Contains(expected_package),
        )
        self.assert_review_passes("basic.snap")

    def test_prime_records_installed_snaps(self):
        subprocess.check_call(["sudo", "snap", "install", "core"])
        self.run_snapcraft(["snap", "--output", "basic.snap"], project_dir="basic")

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        expected_package = "core={}".format(
            repo.get_local_snap_info("core")["revision"]
        )
        self.assertThat(
            recorded_yaml["parts"]["dummy-part"]["installed-snaps"],
            Contains(expected_package),
        )
        self.assert_review_passes("basic.snap")

    def test_prime_with_architectures(self):
        """Test the recorded manifest for a basic snap

        This snap doesn't have stage or build packages and is declared that it
        works on all architectures.
        """
        self.run_snapcraft(["snap", "--output", "basic.snap"], project_dir="basic")

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(recorded_yaml["architectures"], Equals(["all"]))
        self.assert_review_passes("basic.snap")

    def test_prime_without_architectures_records_current_arch(self):
        """Test the recorded manifest for a basic snap

        This snap doesn't have stage or build packages and it is not declared
        that it works on all architectures, which makes it specific to the
        current architecture.
        """
        self.run_snapcraft(
            ["snap", "--output", "basic-without-arch.snap"],
            project_dir="basic-without-arch",
        )

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(recorded_yaml["architectures"], Equals([self.deb_arch]))
        self.assert_review_passes("basic-without-arch.snap")

    def test_prime_records_build_snaps(self):
        self.useFixture(fixture_setup.WithoutSnapInstalled("hello"))
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "nil", "build-snaps": ["hello"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")

        expected_revision = repo.get_local_snap_info("hello")["revision"]
        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(
            recorded_yaml["build-snaps"], Equals(["hello={}".format(expected_revision)])
        )


class ManifestRecordingBuildPackagesTestCase(
    testscenarios.WithScenarios, AssetRecordingBaseTestCase
):

    scenarios = [
        (snap, {"snap": snap})
        for snap in [
            "build-package-global",
            "build-packages-missing-dependency",
            "build-packages-without-dependencies",
        ]
    ]

    def test_prime_with_build_packages(self):
        """Test the recorded manifest for a snap with build packages

        This snap declares one global build package that has undeclared
        dependencies.

        """
        expected_packages = ["haskell-doc", "haskell98-tutorial", "haskell98-report"]
        self.addCleanup(
            subprocess.call, ["sudo", "apt", "remove", "-y"] + expected_packages
        )

        self.run_snapcraft("prime", self.snap)

        expected_packages_with_version = [
            "{}={}".format(
                package,
                integration.get_package_version(
                    package, self.distro_series, self.deb_arch
                ),
            )
            for package in expected_packages
        ]

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(
            recorded_yaml["build-packages"], Equals(expected_packages_with_version)
        )


class ManifestRecordingStagePackagesTestCase(AssetRecordingBaseTestCase):
    def test_prime_records_packages_version(self):
        """Test the recorded manifest for a snap with packages

        This snap declares all the packages that it requires, there are
        no additional dependencies. The packages specify their version.
        """
        expected_packages = ["haskell-doc", "haskell98-tutorial", "haskell98-report"]
        self.copy_project_to_cwd("stage-packages-without-dependencies")
        part_name = "part-with-stage-packages"
        for package in expected_packages:
            self.set_stage_package_version(
                os.path.join("snap", "snapcraft.yaml"), part_name, package
            )

        self.run_snapcraft("prime")

        with open(os.path.join("snap", "snapcraft.yaml")) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(
            recorded_yaml["parts"][part_name]["stage-packages"],
            Equals(source_yaml["parts"][part_name]["stage-packages"]),
        )

    def test_prime_without_packages_version(self):
        """Test the recorded manifest for a snap with packages

        This snap declares all the packages that it requires, there are
        no additional dependencies. The packages don't specify their
        version.
        """
        self.run_snapcraft("prime", project_dir="stage-packages-without-dependencies")

        with open(os.path.join("snap", "snapcraft.yaml")) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        part_name = "part-with-stage-packages"
        expected_packages = [
            "{}={}".format(
                package,
                integration.get_package_version(
                    package, self.distro_series, self.deb_arch
                ),
            )
            for package in source_yaml["parts"][part_name]["stage-packages"]
        ]

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(
            recorded_yaml["parts"][part_name]["stage-packages"],
            Equals(expected_packages),
        )

    def test_prime_with_packages_missing_dependency(self):
        """Test the recorded manifest for a snap with packages

        This snap declares one package that has undeclared dependencies.
        """
        self.copy_project_to_cwd("stage-packages-missing-dependency")
        part_name = "part-with-stage-packages"
        self.set_stage_package_version(
            os.path.join("snap", "snapcraft.yaml"), part_name, package="haskell-doc"
        )
        self.run_snapcraft("prime")

        expected_packages = [
            "{}={}".format(
                package,
                integration.get_package_version(
                    package, self.distro_series, self.deb_arch
                ),
            )
            for package in ["haskell-doc", "haskell98-tutorial", "haskell98-report"]
        ]

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(
            recorded_yaml["parts"][part_name]["stage-packages"],
            Equals(expected_packages),
        )


class ManifestRecordingBzrSourceTestCase(
    integration.BzrSourceBaseTestCase, AssetRecordingBaseTestCase
):
    def test_prime_with_bzr_source(self):
        self.copy_project_to_cwd("bzr-head")

        self.init_source_control()
        self.commit('"test-commit"', unchanged=True)

        self.run_snapcraft(["snap", "--output", "bzr.snap"])

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        commit = self.get_revno()
        self.assertThat(recorded_yaml["parts"]["bzr"]["source-commit"], Equals(commit))
        self.assert_review_passes("bzr.snap")


class ManifestRecordingGitSourceTestCase(
    integration.GitSourceBaseTestCase, AssetRecordingBaseTestCase
):
    def test_prime_with_git_source(self):
        self.copy_project_to_cwd("git-head")

        self.init_source_control()
        self.commit('"test-commit"', allow_empty=True)

        self.run_snapcraft(["snap", "--output", "git.snap"])

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        commit = self.get_revno()
        self.assertThat(recorded_yaml["parts"]["git"]["source-commit"], Equals(commit))
        self.assert_review_passes("git.snap")


class ManifestRecordingHgSourceTestCase(
    integration.HgSourceBaseTestCase, AssetRecordingBaseTestCase
):
    def test_prime_with_hg_source(self):
        self.copy_project_to_cwd("hg-head")

        self.init_source_control()
        open("1", "w").close()
        self.commit("1", "1")

        self.run_snapcraft(["snap", "--output", "hg.snap"])

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        commit = self.get_id()
        self.assertThat(
            recorded_yaml["parts"]["mercurial"]["source-commit"], Equals(commit)
        )
        self.assert_review_passes("hg.snap")


class ManifestRecordingSubversionSourceTestCase(
    integration.SubversionSourceBaseTestCase, AssetRecordingBaseTestCase
):
    def test_prime_with_subversion_source(self):
        self.copy_project_to_cwd("svn-pull")

        self.init_source_control()
        self.checkout("file:///{}".format(os.path.join(self.path, "repo")), "local")

        open(os.path.join("local", "file"), "w").close()
        self.add("file", cwd="local")
        self.commit("test", cwd="local/")
        self.update(cwd="local/")
        subprocess.check_call(["rm", "-rf", "local/"], stdout=subprocess.DEVNULL)

        self.run_snapcraft(["snap", "--output", "svn.snap"])

        recorded_yaml_path = os.path.join(self.prime_dir, "snap", "manifest.yaml")
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertThat(recorded_yaml["parts"]["svn"]["source-commit"], Equals("1"))
        self.assert_review_passes("svn.snap")
