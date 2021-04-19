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
import textwrap
from pathlib import Path
from subprocess import CalledProcessError
from unittest import mock
from unittest.mock import call

import fixtures
import pytest
import testtools
from testtools.matchers import Equals

from snapcraft.internal import repo
from snapcraft.internal.repo import errors
from snapcraft.internal.repo.deb_package import DebPackage
from tests import unit


@pytest.fixture(autouse=True)
def mock_env_copy():
    with mock.patch("os.environ.copy", return_value=dict()) as m:
        yield m


class TestPackages(unit.TestCase):
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

        repo._deb._DEB_CACHE_DIR = self.debs_path
        repo._deb._STAGE_CACHE_DIR = self.stage_cache_path

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

        self.stage_packages_path = Path(self.path)

    @mock.patch(
        "snapcraft.internal.repo._deb._DEFAULT_FILTERED_STAGE_PACKAGES",
        {"filtered-pkg-1", "filtered-pkg-2"},
    )
    def test_fetch_stage_packages(self):
        fake_package = self.debs_path / "fake-package_1.0_all.deb"
        fake_package.touch()
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", fake_package)
        ]

        fetched_packages = repo.Ubuntu.fetch_stage_packages(
            package_names=["fake-package"],
            stage_packages_path=self.stage_packages_path,
            base="core",
            target_arch="amd64",
        )

        self.fake_apt_cache.assert_has_calls(
            [
                call(stage_cache=self.stage_cache_path, stage_cache_arch="amd64"),
                call().__enter__(),
                call().__enter__().update(),
                call().__enter__().mark_packages({"fake-package"}),
                call()
                .__enter__()
                .unmark_packages({"filtered-pkg-1", "filtered-pkg-2"}),
                call().__enter__().fetch_archives(self.debs_path),
            ]
        )

        self.assertThat(fetched_packages, Equals(["fake-package=1.0"]))

    @mock.patch(
        "snapcraft.internal.repo._deb._DEFAULT_FILTERED_STAGE_PACKAGES",
        {"filtered-pkg-1", "filtered-pkg-2", "filtered-pkg-3:amd64", "filtered-pkg-4"},
    )
    def test_fetch_stage_package_filtered_arch_version(self):
        fake_package = self.debs_path / "fake-package_1.0_all.deb"
        fake_package.touch()
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", fake_package)
        ]

        package_names = [
            "filtered-pkg-1=0.0",
            "filtered-pkg-2:amd64",
            "filtered-pkg-3",
            "non-filtered",
        ]
        fetched_packages = repo.Ubuntu.fetch_stage_packages(
            package_names=package_names,
            stage_packages_path=self.stage_packages_path,
            base="core",
            target_arch="amd64",
        )

        self.fake_apt_cache.assert_has_calls(
            [
                call(stage_cache=self.stage_cache_path, stage_cache_arch="amd64"),
                call().__enter__(),
                call().__enter__().update(),
                call().__enter__().mark_packages(set(package_names)),
                call().__enter__().unmark_packages({"filtered-pkg-4"}),
                call().__enter__().fetch_archives(self.debs_path),
            ]
        )

        self.assertThat(fetched_packages, Equals(["fake-package=1.0"]))

    def test_fetch_virtual_stage_package(self):
        fake_package = self.debs_path / "fake-package_1.0_all.deb"
        fake_package.touch()
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", fake_package)
        ]

        fetched_packages = repo.Ubuntu.fetch_stage_packages(
            package_names=["virtual-fake-package"],
            stage_packages_path=self.stage_packages_path,
            base="core",
            target_arch="amd64",
        )

        self.assertThat(fetched_packages, Equals(["fake-package=1.0"]))

    def test_fetch_stage_package_with_deps(self):
        fake_package = self.debs_path / "fake-package_1.0_all.deb"
        fake_package.touch()
        fake_package_dep = self.debs_path / "fake-package-dep_1.0_all.deb"
        fake_package_dep.touch()
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.return_value = [
            ("fake-package", "1.0", fake_package),
            ("fake-package-dep", "2.0", fake_package_dep),
        ]

        fetched_packages = repo.Ubuntu.fetch_stage_packages(
            package_names=["fake-package"],
            stage_packages_path=self.stage_packages_path,
            base="core",
            target_arch="amd64",
        )

        self.assertThat(
            fetched_packages,
            Equals(sorted(["fake-package=1.0", "fake-package-dep=2.0"])),
        )

    def test_get_package_fetch_error(self):
        self.fake_apt_cache.return_value.__enter__.return_value.fetch_archives.side_effect = errors.PackageFetchError(
            "foo"
        )

        raised = self.assertRaises(
            errors.PackageFetchError,
            repo.Ubuntu.fetch_stage_packages,
            package_names=["fake-package"],
            stage_packages_path=Path(self.path),
            base="core",
            target_arch="amd64",
        )
        self.assertThat(str(raised), Equals("Package fetch error: foo"))


class BuildPackagesTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_apt_cache = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo._deb.AptCache")
        ).mock

        self.fake_run = self.useFixture(
            fixtures.MockPatch("subprocess.check_call")
        ).mock

        def get_installed_version(package_name, resolve_virtual_packages=False):
            return "1.0" if "installed" in package_name else None

        self.fake_apt_cache.return_value.__enter__.return_value.get_installed_version.side_effect = (
            get_installed_version
        )

        self.fake_is_dumb_terminal = self.useFixture(
            fixtures.MockPatch(
                "snapcraft.repo._deb.is_dumb_terminal", return_value=True
            )
        ).mock

    def test_install_build_package(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_packages_marked_for_installation.return_value = [
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
                    "package=1.0",
                    "package-installed=1.0",
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
                            "--allow-downgrades",
                            "install",
                            "dependency-package=1.0",
                            "package=1.0",
                            "package-installed=1.0",
                            "versioned-package=2.0",
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
        self.fake_apt_cache.return_value.__enter__.return_value.get_packages_marked_for_installation.return_value = [
            ("package-installed", "1.0")
        ]

        build_packages = repo.Ubuntu.install_build_packages(["package-installed"])

        self.assertThat(build_packages, Equals(["package-installed=1.0"]))
        self.assertThat(self.fake_run.mock_calls, Equals([]))

    def test_already_installed_with_specified_version(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_packages_marked_for_installation.return_value = [
            ("package-installed", "1.0")
        ]

        build_packages = repo.Ubuntu.install_build_packages(["package-installed=1.0"])

        self.assertThat(build_packages, Equals(["package-installed=1.0"]))
        self.assertThat(self.fake_run.mock_calls, Equals([]))

    def test_already_installed_with_different_version(self):
        self.fake_apt_cache.return_value.__enter__.return_value.get_packages_marked_for_installation.return_value = [
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
                            "--allow-downgrades",
                            "install",
                            "package-installed=3.0",
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
        self.fake_apt_cache.return_value.__enter__.return_value.get_packages_marked_for_installation.return_value = [
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
                            "--allow-downgrades",
                            "install",
                            "package=1.0",
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
        self.fake_apt_cache.return_value.__enter__.return_value.get_packages_marked_for_installation.return_value = [
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
                            "--allow-downgrades",
                            "-o",
                            "Dpkg::Progress-Fancy=1",
                            "install",
                            "package=1.0",
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
        self.fake_apt_cache.return_value.__enter__.return_value.get_packages_marked_for_installation.return_value = [
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
            ["package=1.0"],
        )
        self.assertThat(raised.packages, Equals("package=1.0"))

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


class TestGetPackagesInBase(testtools.TestCase):
    def test_hardcoded_bases(self):
        for base in ("core", "core16", "core18"):
            packages = [
                DebPackage.from_unparsed(p)
                for p in repo._deb._DEFAULT_FILTERED_STAGE_PACKAGES
            ]
            assert repo._deb.get_packages_in_base(base=base) == packages

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
                    DebPackage(name="adduser"),
                    DebPackage(name="apparmor"),
                    DebPackage(name="apt"),
                    DebPackage(name="base-files"),
                    DebPackage(name="base-passwd"),
                    DebPackage(name="zlib1g", arch="amd64"),
                ]
            ),
        )

    @mock.patch.object(repo._deb, "_get_dpkg_list_path")
    def test_package_empty_list_from_missing_dpkg_list(self, mock_dpkg_list_path):
        temp_dir = self.useFixture(fixtures.TempDir())
        dpkg_list_path = Path(f"{temp_dir.path}/dpkg.list")
        mock_dpkg_list_path.return_value = dpkg_list_path

        self.expectThat(repo._deb.get_packages_in_base(base="core22"), Equals(list()))


@mock.patch.object(repo._deb, "get_packages_in_base")
def test_get_filtered_stage_package_restricts_core20_ignore_filter(
    mock_get_packages_in_base,
):
    mock_get_packages_in_base.return_value = [
        DebPackage(name="foo"),
        DebPackage(name="foo2"),
        DebPackage(name="python3-attr"),
        DebPackage(name="python3-blinker"),
        DebPackage(name="python3-certifi"),
        DebPackage(name="python3-cffi-backend"),
        DebPackage(name="python3-chardet"),
        DebPackage(name="python3-configobj"),
        DebPackage(name="python3-cryptography"),
        DebPackage(name="python3-idna"),
        DebPackage(name="python3-importlib-metadata"),
        DebPackage(name="python3-jinja2"),
        DebPackage(name="python3-json-pointer"),
        DebPackage(name="python3-jsonpatch"),
        DebPackage(name="python3-jsonschema"),
        DebPackage(name="python3-jwt"),
        DebPackage(name="python3-lib2to3"),
        DebPackage(name="python3-markupsafe"),
        DebPackage(name="python3-more-itertools"),
        DebPackage(name="python3-netifaces"),
        DebPackage(name="python3-oauthlib"),
        DebPackage(name="python3-pyrsistent"),
        DebPackage(name="python3-pyudev"),
        DebPackage(name="python3-requests"),
        DebPackage(name="python3-requests-unixsocket"),
        DebPackage(name="python3-serial"),
        DebPackage(name="python3-six"),
        DebPackage(name="python3-urllib3"),
        DebPackage(name="python3-urwid"),
        DebPackage(name="python3-yaml"),
        DebPackage(name="python3-zipp"),
    ]

    filtered_names = repo._deb._get_filtered_stage_package_names(
        base="core20", package_list=[]
    )

    assert filtered_names == {"foo", "foo2"}


@mock.patch.object(repo._deb, "get_packages_in_base")
def test_get_filtered_stage_package_empty_ignore_filter(mock_get_packages_in_base):
    mock_get_packages_in_base.return_value = [
        DebPackage(name="some-base-pkg"),
        DebPackage(name="some-other-base-pkg"),
    ]

    filtered_names = repo._deb._get_filtered_stage_package_names(
        base="core00", package_list=[]
    )

    assert filtered_names == {"some-base-pkg", "some-other-base-pkg"}
