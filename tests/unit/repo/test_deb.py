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
from typing import Dict, List, Optional
from unittest import mock
from unittest.mock import call, patch, MagicMock, PropertyMock

import testtools
from testtools.matchers import Contains, Equals
import fixtures

from snapcraft.internal import repo
from snapcraft.internal.repo import errors
from tests import unit
from . import RepoBaseTestCase


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
        architecture: str,
        candidate: Optional[MagicMock] = None,
        installed: Optional[MagicMock] = None,
    ) -> None:
        self.name = name
        self.candidate = candidate
        self.installed = installed
        self._architecture = architecture

    def architecture(self):
        return self._architecture


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
        architecture: Optional[str] = None,
        dependency_names: List[str],
    ) -> MagicMock:
        if architecture is None:
            architecture = repo._deb._get_host_arch()

        package = MagicMock(wraps=FakeAptPackage(name=name, architecture=architecture))
        type(package).name = PropertyMock(return_value=name)
        package.architecture.return_value = architecture

        self.packages[name] = package
        self.packages[f"{name}:{architecture}"] = package

        package_dependencies = [
            self.dependencies.get(name) for name in dependency_names
        ]

        apt_version = MagicMock(
            wraps=FakeAptVersion(
                version=version,
                priority=priority,
                package=package,
                dependencies=package_dependencies,
            )
        )

        type(apt_version).version = PropertyMock(return_value=version)
        type(apt_version).priority = PropertyMock(return_value=priority)
        type(apt_version).package = PropertyMock(return_value=package)
        type(apt_version).dependencies = PropertyMock(return_value=package_dependencies)

        self.versions[name] = apt_version

        # Wire up coupled package properties.
        type(package).candidate = PropertyMock(return_value=apt_version)
        if installed:
            type(package).installed = PropertyMock(return_value=apt_version)
        else:
            type(package).installed = PropertyMock(return_value=None)

        # Create matching dependency, even if unused.
        # Dependency will require a fake apt.package.BaseDependency.
        base_dependency = MagicMock(spec=["name"])
        type(base_dependency).name = PropertyMock(return_value=name)

        dependency = MagicMock(spec=["target_versions"])
        type(dependency).target_versions = PropertyMock(return_value=[apt_version])

        self.dependencies[name] = dependency

        return package

    def is_virtual_package(self, package_name):
        return package_name.startswith("virtual-")

    def get_providing_packages(self, package_name):
        package_name = package_name.replace("virtual-", "")
        resolved_package = self.packages[package_name]
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
        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["fake-package"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache["fake-package"].candidate.mock_calls,
            Contains(call.fetch_binary(self.path)),
        )

        self.assertThat(installed_packages, Equals(["fake-package=1.0"]))

    def test_install_virtual_stage_package(self):
        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["virtual-fake-package"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache["fake-package"].candidate.mock_calls,
            Contains(call.fetch_binary(self.path)),
        )

        self.assertThat(installed_packages, Equals(["fake-package=1.0"]))

    def test_install_stage_package_with_deps(self):
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
            version="0.2",
            installed=False,
            dependency_names=[],
        )

        self.fake_apt_cache.add_fake_package(
            name="package-essential",
            priority="essential",
            version="0.3",
            installed=False,
            dependency_names=[],
        )

        self.fake_apt_cache.add_fake_package(
            name="package-with-deps",
            priority="normal",
            version="0.4",
            installed=False,
            dependency_names=["package-installed", "package-not-installed"],
        )

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["package-with-deps"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache["package-not-installed"].candidate.mock_calls,
            Contains(call.fetch_binary(self.path)),
        )

        self.assertThat(
            self.fake_apt_cache["package-installed"].candidate.mock_calls,
            Contains(call.fetch_binary(self.path)),
        )

        self.assertThat(
            self.fake_apt_cache["package-with-deps"].candidate.mock_calls,
            Contains(call.fetch_binary(self.path)),
        )

        self.assertThat(
            self.fake_apt_cache["package-essential"].candidate.mock_calls, Equals([])
        )

        self.assertThat(
            installed_packages,
            Equals(
                [
                    "package-with-deps=0.4",
                    "package-installed=0.1",
                    "package-not-installed=0.2",
                ]
            ),
        )

    def test_install_virtual_package_with_implicit_arch(self):
        self.fake_apt_cache.add_fake_package(
            name="package-dep",
            priority="normal",
            version="0.1",
            installed=True,
            dependency_names=[],
            architecture="i737",
        )

        self.fake_apt_cache.add_fake_package(
            name="package-i737",
            priority="normal",
            version="0.1",
            installed=True,
            dependency_names=["package-dep"],
            architecture="i737",
        )

        self.fake_apt_cache.add_fake_package(
            name="virtual-package-i737",
            priority="normal",
            version="0.2",
            installed=False,
            dependency_names=[],
        )

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["virtual-package-i737"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache["package-i737:i737"].candidate.mock_calls,
            Contains(call.fetch_binary(self.path)),
        )

        self.assertThat(
            self.fake_apt_cache["virtual-package-i737"].candidate.mock_calls, Equals([])
        )

        self.assertThat(
            installed_packages, Equals(["package-i737=0.1", "package-dep=0.1"])
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

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["fake-package:arch"], install_dir=self.path
        )

        self.assertThat(
            self.fake_apt_cache.packages["fake-package:arch"].candidate.mock_calls,
            Contains(call.fetch_binary(self.path)),
        )

        self.assertThat(installed_packages, Equals(["fake-package:arch=1.0"]))

    @mock.patch(
        "snapcraft.internal.os_release.OsRelease.version_codename", return_value="testy"
    )
    def test_sources_formatting(self, mock_version_codename):
        sources_list = textwrap.dedent(
            """
            deb http://archive.ubuntu.com/ubuntu $SNAPCRAFT_APT_RELEASE main restricted
            deb http://archive.ubuntu.com/ubuntu $SNAPCRAFT_APT_RELEASE-updates main restricted
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

        self.useFixture(fixtures.MockPatch("os.environ.copy", return_value={}))

    @patch("snapcraft.repo._deb.is_dumb_terminal")
    @patch("subprocess.check_call")
    def test_install_build_package(self, mock_check_call, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = False
        repo.Ubuntu.install_build_packages(
            ["package-installed", "package-not-installed", "versioned-package=0.2"]
        )

        self.assertThat(
            mock_check_call.mock_calls,
            Equals(
                [
                    call(
                        [
                            "sudo",
                            "--preserve-env",
                            "apt-get",
                            "--no-install-recommends",
                            "-y",
                            "-o",
                            "Dpkg::Progress-Fancy=1",
                            "install",
                            "package-not-installed",
                            "versioned-package=0.2",
                        ],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                    call(
                        [
                            "sudo",
                            "apt-mark",
                            "auto",
                            "package-not-installed",
                            "versioned-package=0.2",
                        ],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                ]
            ),
        )

    @patch("snapcraft.repo._deb.is_dumb_terminal")
    @patch("subprocess.check_call")
    def test_install_virtual_build_package(
        self, mock_check_call, mock_is_dumb_terminal
    ):
        mock_is_dumb_terminal.return_value = False
        repo.Ubuntu.install_build_packages(["virtual-package-not-installed"])

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
        repo.Ubuntu.install_build_packages(
            ["package-installed", "package-not-installed", "versioned-package=0.2"]
        )

        self.assertThat(
            mock_check_call.mock_calls,
            Equals(
                [
                    call(
                        [
                            "sudo",
                            "--preserve-env",
                            "apt-get",
                            "--no-install-recommends",
                            "-y",
                            "install",
                            "package-not-installed",
                            "versioned-package=0.2",
                        ],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                    call(
                        [
                            "sudo",
                            "apt-mark",
                            "auto",
                            "package-not-installed",
                            "versioned-package=0.2",
                        ],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                ]
            ),
        )

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
    @mock.patch("subprocess.run")
    def test_install_gpg(self, mock_run):
        repo.Ubuntu.install_gpg_key(key_id="FAKE_KEYID", key="FAKEKEY")

        env = os.environ.copy()
        env["LANG"] = "C.UTF-8"

        mock_run.assert_has_calls(
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
                    env=env,
                )
            ]
        )

    @mock.patch("subprocess.run")
    def test_install_gpg_key_id(self, mock_run):
        repo.Ubuntu.install_gpg_key_id(key_id="FAKE_KEYID", keys_path=Path(self.path))

        env = os.environ.copy()
        env["LANG"] = "C.UTF-8"

        mock_run.assert_has_calls(
            [
                call(
                    [
                        "sudo",
                        "apt-key",
                        "--keyring",
                        "/etc/apt/trusted.gpg.d/snapcraft.gpg",
                        "adv",
                        "--keyserver",
                        "keyserver.ubuntu.com",
                        "--recv-keys",
                        "FAKE_KEYID",
                    ],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    env=env,
                )
            ]
        )

    @mock.patch("subprocess.run")
    def test_install_gpg_key_id_error(self, mock_run):
        mock_run.side_effect = CalledProcessError(
            cmd=["foo"], returncode=1, output=b"some error"
        )

        raised = self.assertRaises(
            errors.AptGPGKeyInstallError,
            repo.Ubuntu.install_gpg_key_id,
            key_id="FAKE_KEYID",
            keys_path=Path(self.path),
        )

        self.assertThat(raised._output, Equals("some error"))

    @mock.patch("snapcraft.internal.repo._deb._sudo_write_file")
    def test_install_sources(self, mock_write):
        new_sources = repo.Ubuntu.install_sources(
            architectures=["amd64", "arm64"],
            components=["test-component"],
            deb_types=["deb", "deb-src"],
            name="test-name",
            suites=["test-suite1", "test-suite2"],
            url="http://test.url/ubuntu",
        )

        self.assertThat(
            mock_write.mock_calls,
            Equals(
                [
                    call(
                        content=b"Types: deb deb-src\nURIs: http://test.url/ubuntu\nSuites: test-suite1 test-suite2\nComponents: test-component\nArchitectures: amd64 arm64\n",
                        dst_path=Path(
                            "/etc/apt/sources.list.d/snapcraft-test-name.sources"
                        ),
                    )
                ]
            ),
        )

        self.assertThat(new_sources, Equals(True))

    @mock.patch("subprocess.run")
    @mock.patch("snapcraft.internal.repo._deb.Launchpad")
    @mock.patch("snapcraft.internal.repo._deb.Ubuntu.install_sources")
    def test_install_ppa(self, mock_install_sources, mock_launchpad, mock_run):
        mock_launchpad.login_anonymously.return_value.load.return_value.signing_key_fingerprint = (
            "FAKE-SIGNING-KEY"
        )
        repo.Ubuntu.install_ppa(keys_path=Path(self.path), ppa="test/ppa")

        env = os.environ.copy()
        env["LANG"] = "C.UTF-8"

        mock_run.assert_has_calls(
            [
                call(
                    [
                        "sudo",
                        "apt-key",
                        "--keyring",
                        "/etc/apt/trusted.gpg.d/snapcraft.gpg",
                        "adv",
                        "--keyserver",
                        "keyserver.ubuntu.com",
                        "--recv-keys",
                        "FAKE-SIGNING-KEY",
                    ],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    env=env,
                )
            ]
        )

        self.assertThat(
            mock_install_sources.mock_calls,
            Equals(
                [
                    call(
                        components=["main"],
                        deb_types=["deb"],
                        name="ppa-test_ppa",
                        suites=["$SNAPCRAFT_APT_RELEASE"],
                        url="http://ppa.launchpad.net/test/ppa/ubuntu",
                    )
                ]
            ),
        )

    def test_install_ppa_invalid(self):
        raised = self.assertRaises(
            errors.AptPPAInstallError,
            repo.Ubuntu.install_ppa,
            keys_path=Path(self.path),
            ppa="testppa",
        )

        self.assertThat(raised._ppa, Equals("testppa"))

    @mock.patch("subprocess.run")
    def test_apt_key_failure(self, mock_run):
        mock_run.side_effect = CalledProcessError(
            cmd=["foo"], returncode=1, output=b"some error"
        )

        raised = self.assertRaises(
            errors.AptGPGKeyInstallError,
            repo.Ubuntu.install_gpg_key,
            key_id="FAKE_KEYID",
            key="FAKEKEY",
        )

        self.assertThat(raised._output, Equals("some error"))


class TestGetPackagesInBase(testtools.TestCase):
    def test_hardcoded_bases(self):
        for base in ("core", "core16", "core18"):
            self.expectThat(
                repo._deb.get_packages_in_base(base=base),
                Equals(repo._deb._DEFAULT_FILTERED_STAGE_PACKAGES),
            )

    @mock.patch.object(repo._deb, "_get_dpkg_list_path")
    def test_package_list_from_dpkg_list(self, mock_dpkg_list_path):
        temp_dir = self.useFixture(fixtures.TempDir())
        dpkg_list_path = Path(f"{temp_dir.path}/dpkg.list")
        mock_dpkg_list_path.return_value = dpkg_list_path
        with dpkg_list_path.open("w") as dpkg_list_file:
            print(
                textwrap.dedent(
                    """\
            Desired=Unknown/Install/Remove/Purge/Hold
            | Status=Not/Inst/Conf-files/Unpacked/halF-conf/Half-inst/trig-aWait/Trig-pend
            |/ Err?=(none)/Reinst-required (Status,Err: uppercase=bad)
            ||/ Name                          Version                    Architecture Description
            +++-=============================-==========================-============-===========
            ii  adduser                       3.118ubuntu1               all          add and rem
            ii  apparmor                      2.13.3-7ubuntu2            amd64        user-space
            ii  apt                           2.0.1                      amd64        commandline
            ii  base-files                    11ubuntu4                  amd64        Debian base
            ii  base-passwd                   3.5.47                     amd64        Debian base
            ii  zlib1g:amd64                  1:1.2.11.dfsg-2ubuntu1     amd64        compression
            """
                ),
                file=dpkg_list_file,
            )

        self.expectThat(
            repo._deb.get_packages_in_base(base="core20"),
            Equals(
                [
                    "adduser",
                    "apparmor",
                    "apt",
                    "base-files",
                    "base-passwd",
                    "zlib1g:amd64",
                ]
            ),
        )

    @mock.patch.object(repo._deb, "_get_dpkg_list_path")
    def test_package_empty_list_from_missing_dpkg_list(self, mock_dpkg_list_path):
        temp_dir = self.useFixture(fixtures.TempDir())
        dpkg_list_path = Path(f"{temp_dir.path}/dpkg.list")
        mock_dpkg_list_path.return_value = dpkg_list_path

        self.expectThat(repo._deb.get_packages_in_base(base="core22"), Equals(list()))
