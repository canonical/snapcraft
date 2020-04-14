# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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

import apt
import os
import subprocess
import textwrap
from pathlib import Path
from subprocess import CalledProcessError
from unittest import mock
from unittest.mock import call, patch, MagicMock, PropertyMock

from testtools.matchers import Contains, Equals
import fixtures
from typing import Dict, List, Optional

from snapcraft.internal import repo
from snapcraft.internal.repo import errors
from tests import unit
from . import RepoBaseTestCase


class FakeAptDependency:
    def __init__(self, *, target_versions: List[MagicMock]) -> None:
        self.target_versions = target_versions


class FakeAptVersion:
    def __init__(
        self,
        *,
        dependencies: List[MagicMock],
        package: MagicMock,
        priority: str,
        version: str,
    ) -> None:
        self.dependencies = dependencies
        self.package = package
        self.priority = priority
        self.version = version

    def fetch_binary(self, path: str) -> str:
        return os.path.join(path, self.package.name + ".deb")


class FakeAptPackage:
    def __init__(
        self,
        *,
        name: str,
        candidate: Optional[MagicMock] = None,
        installed: Optional[MagicMock] = None,
    ) -> None:
        self.name = name
        self.candidate = candidate
        self.installed = installed


class FakeAptCache:
    def __init__(self) -> None:
        self.dependencies: Dict[str, MagicMock] = dict()
        self.packages: Dict[str, MagicMock] = dict()
        self.versions: Dict[str, MagicMock] = dict()

    def __getitem__(self, key) -> MagicMock:
        return self.packages[key]

    def __contains__(self, key) -> bool:
        return key in self.packages

    def add_fake_package(
        self,
        *,
        name: str,
        priority: str = "normal",
        version: str = "1.0",
        installed: bool = False,
        dependency_names: List[str],
    ) -> MagicMock:
        package = MagicMock(wraps=FakeAptPackage(name=name))
        package_name_mock = PropertyMock(return_value=name)
        type(package).name = package_name_mock

        self.packages[name] = package

        package_dependencies = [
            self.dependencies.get(name) for name in dependency_names
        ]

        version = MagicMock(
            wraps=FakeAptVersion(
                version=version,
                priority=priority,
                package=package,
                dependencies=package_dependencies,
            )
        )
        self.versions[name] = version

        # Wire up coupled package properties.
        package.candidate = version
        if installed:
            package.installed = version
        else:
            package.installed = None

        # Create matching dependency, even if unused.
        dependency = MagicMock(wraps=FakeAptDependency(target_versions=[version]))
        self.dependencies[name] = dependency

        return package

    def is_virtual_package(self, package_name):
        return package_name.startswith("virtual-")

    def get_providing_packages(self, package_name):
        resolved_package = self.packages[package_name.replace("virtual-", "")]
        return [resolved_package]

    def close(self):
        pass


class UbuntuTestCase(RepoBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_apt_cache = FakeAptCache()
        self.fake_apt_cache.add_fake_package(name="fake-package", dependency_names=[])

        self.fake_apt_cache_mock = MagicMock(wraps=self.fake_apt_cache)

        self.useFixture(
            fixtures.MockPatch("apt.Cache", return_value=self.fake_apt_cache)
        )

        self.fake_run = self.useFixture(
            fixtures.MockPatch("subprocess.check_call")
        ).mock

        # Override the cache directory to temp path.
        repo.Ubuntu._cache_dir = self.path

        # Ensure the cache is reset.
        repo.Ubuntu._cache = None

    def test_install_stage_package(self):
        repo.Ubuntu.install_stage_packages(
            package_names=["fake-package"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache["fake-package"].mock_calls,
            Contains(call.candidate.fetch_binary(self.path)),
        )

    def test_install_virtual_stage_package(self):
        repo.Ubuntu.install_stage_packages(
            package_names=["virtual-fake-package"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache["fake-package"].mock_calls,
            Contains(call.candidate.fetch_binary(self.path)),
        )

    def test_get_package_fetch_error(self):
        self.fake_apt_cache.packages[
            "fake-package"
        ].candidate.fetch_binary.side_effect = apt.package.FetchError("foo")

        raised = self.assertRaises(
            errors.PackageFetchError,
            repo.Ubuntu.install_stage_packages,
            package_names=["fake-package"],
            install_dir=self.path,
        )
        self.assertThat(str(raised), Equals("Package fetch error: foo"))

    def test_get_multiarch_package(self):
        self.fake_apt_cache.add_fake_package(
            name="fake-package:arch", dependency_names=[]
        )

        repo.Ubuntu.install_stage_packages(
            package_names=["fake-package:arch"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache.packages["fake-package:arch"].mock_calls,
            Contains(call.candidate.fetch_binary(self.path)),
        )

    @mock.patch(
        "snapcraft.internal.os_release.OsRelease.version_codename", return_value="testy"
    )
    def test_sources_formatting(self, mock_version_codename):
        sources_list = textwrap.dedent(
            """
            deb http://archive.ubuntu.com/ubuntu ${release} main restricted
            deb http://archive.ubuntu.com/ubuntu ${release}-updates main restricted
            """
        )

        sources_list = repo._deb._format_sources_list(sources_list)

        expected_sources_list = textwrap.dedent(
            """
            deb http://archive.ubuntu.com/ubuntu testy main restricted
            deb http://archive.ubuntu.com/ubuntu testy-updates main restricted
            """
        )
        self.assertThat(sources_list, Equals(expected_sources_list))


class BuildPackagesTestCase(UbuntuTestCase):
    def setUp(self):
        super().setUp()

        self.fake_apt_cache.add_fake_package(
            name="package-installed",
            priority="normal",
            version="0.1",
            installed=True,
            dependency_names=[],
        )

        self.fake_apt_cache.add_fake_package(
            name="package-not-installed",
            priority="normal",
            version="0.1",
            installed=False,
            dependency_names=[],
        )

        self.fake_apt_cache.add_fake_package(
            name="versioned-package",
            priority="normal",
            version="0.2",
            installed=False,
            dependency_names=[],
        )

        self.test_packages = [
            "package-installed",
            "package-not-installed",
            "versioned-package=0.2",
        ]

    def get_installable_packages(self, packages):
        return ["package-not-installed", "versioned-package=0.2"]

    @patch("os.environ")
    def install_test_packages(self, test_pkgs, mock_env):
        mock_env.copy.return_value = {}
        repo.Ubuntu.install_build_packages(test_pkgs)

    @patch("snapcraft.repo._deb.is_dumb_terminal")
    @patch("subprocess.check_call")
    def test_install_build_package(self, mock_check_call, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = False
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        mock_check_call.assert_has_calls(
            [
                call(
                    "sudo --preserve-env apt-get --no-install-recommends -y "
                    "-o Dpkg::Progress-Fancy=1 install".split()
                    + sorted(set(installable)),
                    env={
                        "DEBIAN_FRONTEND": "noninteractive",
                        "DEBCONF_NONINTERACTIVE_SEEN": "true",
                        "DEBIAN_PRIORITY": "critical",
                    },
                )
            ]
        )

    @patch("snapcraft.repo._deb.is_dumb_terminal")
    @patch("subprocess.check_call")
    def test_install_virtual_build_package(
        self, mock_check_call, mock_is_dumb_terminal
    ):
        mock_is_dumb_terminal.return_value = False
        self.install_test_packages(["virtual-package-not-installed"])

        mock_check_call.assert_has_calls(
            [
                call(
                    "sudo --preserve-env apt-get --no-install-recommends -y "
                    "-o Dpkg::Progress-Fancy=1 install package-not-installed".split(),
                    env={
                        "DEBIAN_FRONTEND": "noninteractive",
                        "DEBCONF_NONINTERACTIVE_SEEN": "true",
                        "DEBIAN_PRIORITY": "critical",
                    },
                )
            ]
        )

    @patch("snapcraft.repo._deb.is_dumb_terminal")
    @patch("subprocess.check_call")
    def test_install_buid_package_in_dumb_terminal(
        self, mock_check_call, mock_is_dumb_terminal
    ):
        mock_is_dumb_terminal.return_value = True
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        mock_check_call.assert_has_calls(
            [
                call(
                    "sudo --preserve-env apt-get --no-install-recommends -y install".split()
                    + sorted(set(installable)),
                    env={
                        "DEBIAN_FRONTEND": "noninteractive",
                        "DEBCONF_NONINTERACTIVE_SEEN": "true",
                        "DEBIAN_PRIORITY": "critical",
                    },
                )
            ]
        )

    @patch("subprocess.check_call")
    def test_install_buid_package_marks_auto_installed(self, mock_check_call):
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        mock_check_call.assert_has_calls(
            [
                call(
                    "sudo apt-mark auto".split() + sorted(set(installable)),
                    env={
                        "DEBIAN_FRONTEND": "noninteractive",
                        "DEBCONF_NONINTERACTIVE_SEEN": "true",
                        "DEBIAN_PRIORITY": "critical",
                    },
                )
            ]
        )

    @patch("subprocess.check_call")
    def test_mark_installed_auto_error_is_not_fatal(self, mock_check_call):
        error = CalledProcessError(101, "bad-cmd")
        mock_check_call.side_effect = lambda c, env: error if "apt-mark" in c else None
        self.install_test_packages(["package-not-installed"])

    def test_invalid_package_requested(self):
        self.assertRaises(
            errors.BuildPackageNotFoundError,
            repo.Ubuntu.install_build_packages,
            ["package-does-not-exist"],
        )

    @patch("subprocess.check_call")
    def test_broken_package_apt_install(self, mock_check_call):
        mock_check_call.side_effect = CalledProcessError(100, "apt-get")
        raised = self.assertRaises(
            errors.BuildPackagesNotInstalledError,
            repo.Ubuntu.install_build_packages,
            ["package-not-installed"],
        )
        self.assertThat(raised.packages, Equals("package-not-installed"))

    @patch("subprocess.check_call")
    def test_refresh_buid_packages(self, mock_check_call):
        repo.Ubuntu.refresh_build_packages()

        mock_check_call.assert_called_once_with(
            ["sudo", "--preserve-env", "apt-get", "update"]
        )

    @patch(
        "subprocess.check_call",
        side_effect=CalledProcessError(
            returncode=1, cmd=["sudo", "--preserve-env", "apt-get", "update"]
        ),
    )
    def test_refresh_buid_packages_fails(self, mock_check_call):
        self.assertRaises(
            errors.CacheUpdateFailedError, repo.Ubuntu.refresh_build_packages
        )

        mock_check_call.assert_called_once_with(
            ["sudo", "--preserve-env", "apt-get", "update"]
        )


class PackageForFileTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        def fake_dpkg_query(*args, **kwargs):
            # dpkg-query -S file_path
            if args[0][2] == "/bin/bash":
                return "bash: /bin/bash\n".encode()
            elif args[0][2] == "/bin/sh":
                return (
                    "diversion by dash from: /bin/sh\n"
                    "diversion by dash to: /bin/sh.distrib\n"
                    "dash: /bin/sh\n"
                ).encode()
            else:
                raise CalledProcessError(
                    1,
                    "dpkg-query: no path found matching pattern {}".format(args[0][2]),
                )

        self.useFixture(
            fixtures.MockPatch("subprocess.check_output", side_effect=fake_dpkg_query)
        )

    def test_get_package_for_file(self):
        self.assertThat(repo.Ubuntu.get_package_for_file("/bin/bash"), Equals("bash"))

    def test_get_package_for_file_with_no_leading_slash(self):
        self.assertThat(repo.Ubuntu.get_package_for_file("bin/bash"), Equals("bash"))

    def test_get_package_for_file_with_diversions(self):
        self.assertThat(repo.Ubuntu.get_package_for_file("/bin/sh"), Equals("dash"))

    def test_get_package_for_file_not_found(self):
        self.assertRaises(
            repo.errors.FileProviderNotFound,
            repo.Ubuntu.get_package_for_file,
            "/bin/not-found",
        )


class TestUbuntuInstallRepo(unit.TestCase):
    @mock.patch("snapcraft.internal.repo._deb.Ubuntu.refresh_build_packages")
    def test_install(self, mock_refresh):
        snapcraft_list = Path(self.path, "snapcraft.list")

        with mock.patch(
            "snapcraft.internal.repo._deb.Ubuntu._SNAPCRAFT_INSTALLED_SOURCES_LIST",
            new=str(snapcraft_list),
        ):
            test_source = "deb http://source"
            repo.Ubuntu.install_source(test_source)

            self.assertThat(snapcraft_list.exists(), Equals(True))
            self.assertThat(snapcraft_list.owner(), Equals("root"))
            self.assertThat(snapcraft_list.group(), Equals("root"))
            self.assertThat(snapcraft_list.stat().st_mode & 0o777, Equals(0o644))

            installed_sources = snapcraft_list.read_text().splitlines()

            self.assertThat(test_source in installed_sources, Equals(True))

            test_source2 = "deb http://source2"
            repo.Ubuntu.install_source(test_source2)

            installed_sources = snapcraft_list.read_text().splitlines()
            expected_sources = sorted([test_source, test_source2])

            self.assertThat(installed_sources, Equals(expected_sources))

    @mock.patch("subprocess.run")
    def test_install_gpg(self, mock_run):
        repo.Ubuntu.install_gpg_key("FAKEKEY")

        self.assertThat(
            mock_run.mock_calls,
            Equals(
                [
                    call(
                        [
                            "sudo",
                            "apt-key",
                            "--keyring",
                            repo.Ubuntu._SNAPCRAFT_INSTALLED_GPG_KEYRING,
                            "add",
                            "-",
                        ],
                        check=True,
                        input=b"FAKEKEY",
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                    )
                ]
            ),
        )

    @mock.patch("subprocess.run")
    def test_apt_key_failure(self, mock_run):
        mock_run.side_effect = CalledProcessError(
            cmd=["foo"], returncode=1, output="some error"
        )

        self.assertRaises(
            errors.AptGPGKeyInstallError, repo.Ubuntu.install_gpg_key, "FAKEKEY"
        )
