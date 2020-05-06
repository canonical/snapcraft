# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

import contextlib
import os
import subprocess
import textwrap
from pathlib import Path
from subprocess import CalledProcessError
from unittest import mock
from unittest.mock import call

import testtools
from testtools.matchers import Equals
import fixtures

from snapcraft.internal import repo
from snapcraft.internal.repo import errors
from tests import unit


class BaseDebTestSetup(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_apt_cache = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo._deb.AptCache")
        ).mock

        self.fake_run = self.useFixture(
            fixtures.MockPatch("subprocess.check_call")
        ).mock

        self.stage_cache_path = Path(self.path, "stage-cache")
        self.debs_path = Path(self.path, "debs")
        self.debs_path.mkdir(parents=True, exist_ok=False)

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.repo._deb._DEB_CACHE_DIR", new=self.debs_path
            )
        )
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.repo._deb._STAGE_CACHE_DIR",
                new=self.stage_cache_path,
            )
        )
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.repo._deb._STAGE_PCACHE",
                new=Path(self.stage_cache_path, "pcache"),
            )
        )

        @contextlib.contextmanager
        def fake_tempdir(*, suffix: str, **kwargs):
            temp_dir = Path(self.path, suffix)
            temp_dir.mkdir(exist_ok=True, parents=True)
            yield str(temp_dir)

        self.fake_tmp_mock = self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.repo._deb.tempfile.TemporaryDirectory",
                new=fake_tempdir,
            )
        ).mock

    def test_install_stage_packages(self):
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", Path(self.path))
        ]

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["fake-package"], install_dir=self.path, base="core"
        )

        self.fake_apt_cache.assert_has_calls(
            [
                call(stage_cache=self.stage_cache_path),
                call().__enter__(),
                call().__enter__().update(),
                call().__enter__().mark_packages({"fake-package"}),
                call()
                .__enter__()
                .unmark_packages(
                    required_names={"fake-package"},
                    filtered_names=set(repo._deb._DEFAULT_FILTERED_STAGE_PACKAGES),
                ),
                call().__enter__().fetch_archives(self.debs_path),
            ]
        )

        self.assertThat(installed_packages, Equals(["fake-package=1.0"]))

    def test_install_virtual_stage_package(self):
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", Path(self.path))
        ]

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["virtual-fake-package"], install_dir=self.path, base="core"
        )

        self.assertThat(installed_packages, Equals(["fake-package=1.0"]))

    def test_install_stage_package_with_deps(self):
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", Path(self.path)),
            ("fake-package-dep", "2.0", Path(self.path)),
        ]

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["fake-package"], install_dir=self.path, base="core"
        )

        self.assertThat(
            installed_packages,
            Equals(sorted(["fake-package=1.0", "fake-package-dep=2.0"])),
        )

    def test_get_package_fetch_error(self):
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.side_effect = errors.PackageFetchError(
            "foo"
        )

        raised = self.assertRaises(
            errors.PackageFetchError,
            repo.Ubuntu.install_stage_packages,
            package_names=["fake-package"],
            install_dir=self.path,
            base="core",
        )

        self.assertThat(str(raised), Equals("Package fetch error: foo"))

    def test_install_stage_package_cached(self):
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", Path(self.path))
        ]

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["fake-package"], install_dir=self.path, base="core"
        )

        self.assertThat(installed_packages, Equals(["fake-package=1.0"]))
        self.fake_apt_cache.assert_called()
        self.fake_apt_cache.reset_mock()

        installed_packages = repo.Ubuntu.install_stage_packages(
            package_names=["fake-package"], install_dir=self.path, base="core"
        )

        self.fake_apt_cache.assert_not_called()
        self.assertThat(installed_packages, Equals(["fake-package=1.0"]))


class TestSourcesFormatting(unit.TestCase):
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


class BuildPackagesTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_apt_cache = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo._deb.AptCache")
        ).mock

        self.fake_run = self.useFixture(
            fixtures.MockPatch("subprocess.check_call")
        ).mock

        def get_installed_version(package_name):
            return "1.0" if "installed" in package_name else None

        self.fake_apt_cache.return_value.__enter__.return_value.get_installed_version.side_effect = (
            get_installed_version
        )

        self.useFixture(fixtures.MockPatch("os.environ.copy", return_value={}))

        self.fake_is_dumb_terminal = self.useFixture(
            fixtures.MockPatch(
                "snapcraft.repo._deb.is_dumb_terminal", return_value=True
            )
        ).mock

    def test_install_build_package(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_marked_packages.return_value = [
            ("package", "1.0"),
            ("package-installed", "1.0"),
            ("versioned-package", "2.0"),
            ("dependency-package", "1.0"),
        ]

        build_packages = repo.Ubuntu.install_build_packages(
            ["package-installed", "package", "versioned-package=2.0"]
        )

        self.assertThat(
            build_packages,
            Equals(
                [
                    "dependency-package=1.0",
                    "package-installed=1.0",
                    "package=1.0",
                    "versioned-package=2.0",
                ]
            ),
        )
        self.assertThat(
            self.fake_run.mock_calls,
            Equals(
                [
                    call(["sudo", "--preserve-env", "apt-get", "update"]),
                    call(
                        [
                            "sudo",
                            "--preserve-env",
                            "apt-get",
                            "--no-install-recommends",
                            "-y",
                            "install",
                            "dependency-package",
                            "package",
                            "package-installed",
                            "versioned-package",
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
                            "dependency-package",
                            "package",
                            "package-installed",
                            "versioned-package",
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

    def test_already_installed_no_specified_version(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_marked_packages.return_value = [
            ("package-installed", "1.0")
        ]

        build_packages = repo.Ubuntu.install_build_packages(["package-installed"])

        self.assertThat(build_packages, Equals(["package-installed=1.0"]))
        self.assertThat(self.fake_run.mock_calls, Equals([]))

    def test_already_installed_with_specified_version(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_marked_packages.return_value = [
            ("package-installed", "1.0")
        ]

        build_packages = repo.Ubuntu.install_build_packages(["package-installed=1.0"])

        self.assertThat(build_packages, Equals(["package-installed=1.0"]))
        self.assertThat(self.fake_run.mock_calls, Equals([]))

    def test_already_installed_with_different_version(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_marked_packages.return_value = [
            ("package-installed", "3.0")
        ]

        build_packages = repo.Ubuntu.install_build_packages(["package-installed=3.0"])

        self.assertThat(build_packages, Equals(["package-installed=3.0"]))
        self.assertThat(
            self.fake_run.mock_calls,
            Equals(
                [
                    call(["sudo", "--preserve-env", "apt-get", "update"]),
                    call(
                        [
                            "sudo",
                            "--preserve-env",
                            "apt-get",
                            "--no-install-recommends",
                            "-y",
                            "install",
                            "package-installed",
                        ],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                    call(
                        ["sudo", "apt-mark", "auto", "package-installed"],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                ]
            ),
        )

    def test_install_virtual_build_package(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_marked_packages.return_value = [
            ("package", "1.0")
        ]

        build_packages = repo.Ubuntu.install_build_packages(["virtual-package"])

        self.assertThat(build_packages, Equals(["package=1.0"]))
        self.assertThat(
            self.fake_run.mock_calls,
            Equals(
                [
                    call(["sudo", "--preserve-env", "apt-get", "update"]),
                    call(
                        [
                            "sudo",
                            "--preserve-env",
                            "apt-get",
                            "--no-install-recommends",
                            "-y",
                            "install",
                            "package",
                        ],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                    call(
                        ["sudo", "apt-mark", "auto", "package"],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                ]
            ),
        )

    def test_smart_terminal(self):
        self.fake_is_dumb_terminal.return_value = False
        self.fake_apt_cache.return_value.__enter__.return_value.get_marked_packages.return_value = [
            ("package", "1.0")
        ]

        repo.Ubuntu.install_build_packages(["package"])

        self.assertThat(
            self.fake_run.mock_calls,
            Equals(
                [
                    call(["sudo", "--preserve-env", "apt-get", "update"]),
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
                            "package",
                        ],
                        env={
                            "DEBIAN_FRONTEND": "noninteractive",
                            "DEBCONF_NONINTERACTIVE_SEEN": "true",
                            "DEBIAN_PRIORITY": "critical",
                        },
                    ),
                    call(
                        ["sudo", "apt-mark", "auto", "package"],
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
        self.fake_apt_cache.return_value.__enter__.return_value.mark_packages.side_effect = errors.PackageNotFoundError(
            "package-invalid"
        )

        self.assertRaises(
            errors.BuildPackageNotFoundError,
            repo.Ubuntu.install_build_packages,
            ["package-invalid"],
        )

    def test_broken_package_apt_install(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_marked_packages.return_value = [
            ("package", "1.0")
        ]
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.repo._deb.Ubuntu.refresh_build_packages"
            )
        )

        self.fake_run.side_effect = CalledProcessError(100, "apt-get")
        raised = self.assertRaises(
            errors.BuildPackagesNotInstalledError,
            repo.Ubuntu.install_build_packages,
            ["package"],
        )
        self.assertThat(raised.packages, Equals("package"))

    def test_refresh_buid_packages(self):
        repo.Ubuntu.refresh_build_packages()

        self.fake_run.assert_called_once_with(
            ["sudo", "--preserve-env", "apt-get", "update"]
        )

    def test_refresh_buid_packages_fails(self):
        self.fake_run.side_effect = CalledProcessError(
            returncode=1, cmd=["sudo", "--preserve-env", "apt-get", "update"]
        )
        self.assertRaises(
            errors.CacheUpdateFailedError, repo.Ubuntu.refresh_build_packages
        )

        self.assertThat(
            self.fake_run.mock_calls,
            Equals([call(["sudo", "--preserve-env", "apt-get", "update"])]),
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
