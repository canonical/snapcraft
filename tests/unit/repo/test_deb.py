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
from unittest.mock import ANY, DEFAULT, call, patch, MagicMock

from testtools.matchers import Contains, Equals, FileExists, Not
import fixtures

from snapcraft.internal import repo
from snapcraft.internal.repo import errors
from tests import fixture_setup, unit
from . import RepoBaseTestCase


class UbuntuTestCase(RepoBaseTestCase):
    def setUp(self):
        super().setUp()
        patcher = patch("snapcraft.repo._deb.apt.Cache")
        self.mock_cache = patcher.start()
        self.addCleanup(patcher.stop)

        def _fetch_binary(download_dir, **kwargs):
            path = os.path.join(download_dir, "fake-package.deb")
            open(path, "w").close()
            return path

        self.mock_package = MagicMock()
        self.mock_package.candidate.fetch_binary.side_effect = _fetch_binary
        self.mock_cache.return_value.get_changes.return_value = [self.mock_package]

    @patch("snapcraft.internal.repo._deb.apt.apt_pkg")
    def test_cache_update_failed(self, mock_apt_pkg):
        self.mock_cache().is_virtual_package.return_value = False
        self.mock_cache().update.side_effect = apt.cache.FetchFailedException()
        ubuntu = repo.Ubuntu(self.tempdir)
        self.assertRaises(errors.CacheUpdateFailedError, ubuntu.get, ["fake-package"])

    @patch("shutil.rmtree")
    @patch("snapcraft.internal.repo._deb.apt.apt_pkg")
    def test_cache_hashsum_mismatch(self, mock_apt_pkg, mock_rmtree):
        self.mock_cache().is_virtual_package.return_value = False
        self.mock_cache().update.side_effect = [
            apt.cache.FetchFailedException(
                "E:Failed to fetch copy:foo Hash Sum mismatch"
            ),
            DEFAULT,
        ]
        ubuntu = repo.Ubuntu(self.tempdir)
        ubuntu.get(["fake-package"])

    def test_get_pkg_name_parts_name_only(self):
        name, version = repo.get_pkg_name_parts("hello")
        self.assertThat(name, Equals("hello"))
        self.assertThat(version, Equals(None))

    def test_get_pkg_name_parts_all(self):
        name, version = repo.get_pkg_name_parts("hello:i386=2.10-1")
        self.assertThat(name, Equals("hello:i386"))
        self.assertThat(version, Equals("2.10-1"))

    def test_get_pkg_name_parts_no_arch(self):
        name, version = repo.get_pkg_name_parts("hello=2.10-1")
        self.assertThat(name, Equals("hello"))
        self.assertThat(version, Equals("2.10-1"))

    @patch("snapcraft.internal.repo._deb.apt.apt_pkg")
    def test_get_package(self, mock_apt_pkg):
        self.mock_cache().is_virtual_package.return_value = False

        fake_trusted_parts_path = os.path.join(self.path, "fake-trusted-parts")
        os.mkdir(fake_trusted_parts_path)
        open(os.path.join(fake_trusted_parts_path, "trusted-part.gpg"), "w").close()

        def _fake_find_file(key: str):
            if key == "Dir::Etc::TrustedParts":
                return fake_trusted_parts_path
            else:
                return DEFAULT

        mock_apt_pkg.config.find_file.side_effect = _fake_find_file

        ubuntu = repo.Ubuntu(self.tempdir)
        ubuntu.get(["fake-package"])

        mock_apt_pkg.assert_has_calls(
            [
                call.config.set("Apt::Install-Recommends", "False"),
                call.config.set("Acquire::AllowInsecureRepositories", "False"),
                call.config.find_file("Dir::Etc::Trusted"),
                call.config.set("Dir::Etc::Trusted", ANY),
                call.config.find_file("Dir::Etc::TrustedParts"),
                call.config.set("Dir::Etc::TrustedParts", ANY),
                call.config.clear("APT::Update::Post-Invoke-Success"),
            ]
        )

        self.mock_cache.assert_has_calls(
            [
                call(memonly=True, rootdir=ANY),
                call().update(fetch_progress=ANY, sources_list=ANY),
                call().open(),
            ]
        )

        # __getitem__ is tricky
        self.assertThat(
            self.mock_cache.return_value.__getitem__.call_args_list,
            Contains(call("fake-package")),
        )

        # Verify that the package was actually fetched and copied into the
        # requested location.
        self.assertThat(
            os.path.join(self.tempdir, "download", "fake-package.deb"), FileExists()
        )

        # Verify that TrustedParts were properly setup
        trusted_parts_dir = os.path.join(
            ubuntu._cache.base_dir,
            os.path.join(self.path, "fake-trusted-parts").lstrip("/"),
        )
        self.assertThat(os.listdir(trusted_parts_dir), Equals(["trusted-part.gpg"]))

    @patch("snapcraft.internal.repo._deb.apt.apt_pkg")
    def test_get_package_fetch_error(self, mock_apt_pkg):
        self.mock_package.candidate.fetch_binary.side_effect = apt.package.FetchError(
            "foo"
        )
        self.mock_cache().is_virtual_package.return_value = False
        ubuntu = repo.Ubuntu(self.tempdir)
        raised = self.assertRaises(
            errors.PackageFetchError, ubuntu.get, ["fake-package"]
        )
        self.assertThat(str(raised), Equals("Package fetch error: foo"))

    @patch("snapcraft.internal.repo._deb.apt.apt_pkg")
    def test_get_package_trusted_parts_already_imported(self, mock_apt_pkg):
        self.mock_cache().is_virtual_package.return_value = False

        def _fake_find_file(key: str):
            if key == "Dir::Etc::TrustedParts":
                return os.path.join(ubuntu._cache.base_dir, "trusted")
            else:
                return DEFAULT

        mock_apt_pkg.config.find_file.side_effect = _fake_find_file

        ubuntu = repo.Ubuntu(self.tempdir)
        ubuntu.get(["fake-package"])

        mock_apt_pkg.assert_has_calls(
            [
                call.config.set("Apt::Install-Recommends", "False"),
                call.config.set("Acquire::AllowInsecureRepositories", "False"),
                call.config.find_file("Dir::Etc::Trusted"),
                call.config.set("Dir::Etc::Trusted", ANY),
                call.config.find_file("Dir::Etc::TrustedParts"),
                call.config.clear("APT::Update::Post-Invoke-Success"),
            ]
        )

        self.mock_cache.assert_has_calls(
            [
                call(memonly=True, rootdir=ANY),
                call().update(fetch_progress=ANY, sources_list=ANY),
                call().open(),
            ]
        )

        # __getitem__ is tricky
        self.assertThat(
            self.mock_cache.return_value.__getitem__.call_args_list,
            Contains(call("fake-package")),
        )

        # Verify that the package was actually fetched and copied into the
        # requested location.
        self.assertThat(
            os.path.join(self.tempdir, "download", "fake-package.deb"), FileExists()
        )

    @patch("snapcraft.internal.repo._deb.apt.apt_pkg")
    def test_get_multiarch_package(self, mock_apt_pkg):
        self.mock_cache().is_virtual_package.return_value = False

        fake_trusted_parts_path = os.path.join(self.path, "fake-trusted-parts")
        os.mkdir(fake_trusted_parts_path)
        open(os.path.join(fake_trusted_parts_path, "trusted-part.gpg"), "w").close()

        def _fake_find_file(key: str):
            if key == "Dir::Etc::TrustedParts":
                return fake_trusted_parts_path
            else:
                return DEFAULT

        mock_apt_pkg.config.find_file.side_effect = _fake_find_file

        ubuntu = repo.Ubuntu(self.tempdir)
        ubuntu.get(["fake-package:arch"])

        mock_apt_pkg.assert_has_calls(
            [
                call.config.set("Apt::Install-Recommends", "False"),
                call.config.set("Acquire::AllowInsecureRepositories", "False"),
                call.config.find_file("Dir::Etc::Trusted"),
                call.config.set("Dir::Etc::Trusted", ANY),
                call.config.find_file("Dir::Etc::TrustedParts"),
                call.config.set("Dir::Etc::TrustedParts", ANY),
                call.config.clear("APT::Update::Post-Invoke-Success"),
            ]
        )
        self.mock_cache.assert_has_calls(
            [
                call(memonly=True, rootdir=ANY),
                call().update(fetch_progress=ANY, sources_list=ANY),
                call().open(),
            ]
        )

        # __getitem__ is tricky
        self.assertThat(
            self.mock_cache.return_value.__getitem__.call_args_list,
            Contains(call("fake-package:arch")),
        )

        # Verify that the package was actually fetched and copied into the
        # requested location.
        self.assertThat(
            os.path.join(self.tempdir, "download", "fake-package.deb"), FileExists()
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


class UbuntuTestCaseWithFakeAptCache(RepoBaseTestCase):
    def setUp(self):
        super().setUp()
        self.fake_apt_cache = fixture_setup.FakeAptCache()
        self.useFixture(self.fake_apt_cache)

    def test_get_installed_packages(self):
        for name, version, installed in (
            ("test-installed-package", "test-installed-package-version", True),
            ("test-not-installed-package", "dummy", False),
        ):
            self.fake_apt_cache.add_package(
                fixture_setup.FakeAptCachePackage(name, version, installed=installed)
            )

        self.assertThat(
            repo.Repo.get_installed_packages(),
            Equals(["test-installed-package=test-installed-package-version"]),
        )


class AutokeepTestCase(RepoBaseTestCase):
    def test_autokeep(self):
        self.fake_apt_cache = fixture_setup.FakeAptCache()
        self.useFixture(self.fake_apt_cache)
        self.test_packages = (
            "main-package",
            "dependency",
            "sub-dependency",
            "conflicting-dependency",
        )
        self.fake_apt_cache.add_packages(self.test_packages)
        self.fake_apt_cache.cache["main-package"].dependencies = [
            [
                fixture_setup.FakeAptBaseDependency(
                    "dependency", [self.fake_apt_cache.cache["dependency"]]
                ),
                fixture_setup.FakeAptBaseDependency(
                    "conflicting-dependency",
                    [self.fake_apt_cache.cache["conflicting-dependency"]],
                ),
            ]
        ]
        self.fake_apt_cache.cache["dependency"].dependencies = [
            [
                fixture_setup.FakeAptBaseDependency(
                    "sub-dependency", [self.fake_apt_cache.cache["sub-dependency"]]
                )
            ]
        ]
        self.fake_apt_cache.cache["conflicting-dependency"].conflicts = [
            self.fake_apt_cache.cache["dependency"]
        ]

        ubuntu = repo.Ubuntu(self.tempdir)
        ubuntu.get(["main-package", "conflicting-dependency"])

        # Verify that the package was actually fetched and copied into the
        # requested location.
        self.assertThat(
            os.path.join(self.tempdir, "download", "main-package.deb"), FileExists()
        )
        self.assertThat(
            os.path.join(self.tempdir, "download", "conflicting-dependency.deb"),
            FileExists(),
        )
        self.assertThat(
            os.path.join(self.tempdir, "download", "dependency.deb"),
            Not(FileExists()),
            "Dependency should not have been fetched",
        )
        self.assertThat(
            os.path.join(self.tempdir, "download", "sub-dependency.deb"),
            Not(FileExists()),
            "Sub-dependency should not have been fetched",
        )


class BuildPackagesTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.fake_apt_cache = fixture_setup.FakeAptCache()
        self.useFixture(self.fake_apt_cache)
        self.test_packages = (
            "package-not-installed",
            "package-installed",
            "another-uninstalled",
            "another-installed",
            "repeated-package",
            "repeated-package",
            "versioned-package=0.2",
            "versioned-package",
        )
        self.fake_apt_cache.add_packages(self.test_packages)
        self.fake_apt_cache.cache["package-installed"].installed = True
        self.fake_apt_cache.cache["another-installed"].installed = True
        self.fake_apt_cache.cache["versioned-package"].version = "0.1"

    def get_installable_packages(self, packages):
        return [
            "package-not-installed",
            "another-uninstalled",
            "repeated-package",
            "versioned-package=0.2",
        ]

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
    def test_broken_package_requested(self, mock_check_call):
        self.fake_apt_cache.add_packages(("package-not-installable",))
        self.fake_apt_cache.cache["package-not-installable"].dependencies = [
            [fixture_setup.FakeAptBaseDependency("broken-dependency", [])]
        ]
        self.assertRaises(
            errors.PackageBrokenError,
            repo.Ubuntu.install_build_packages,
            ["package-not-installable"],
        )

    @patch("subprocess.check_call")
    def test_broken_package_apt_install(self, mock_check_call):
        mock_check_call.side_effect = CalledProcessError(100, "apt-get")
        self.fake_apt_cache.add_packages(("package-not-installable",))
        raised = self.assertRaises(
            errors.BuildPackagesNotInstalledError,
            repo.Ubuntu.install_build_packages,
            ["package-not-installable"],
        )
        self.assertThat(raised.packages, Equals("package-not-installable"))

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
