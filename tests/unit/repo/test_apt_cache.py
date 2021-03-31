# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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
import unittest
from pathlib import Path
from unittest import mock
from unittest.mock import call

import fixtures
from testtools.matchers import Equals

from snapcraft.internal.repo.apt_cache import AptCache
from tests import unit


class TestAptStageCache(unit.TestCase):
    # This are expensive tests, but is much more valuable than using mocks.
    # When adding tests, consider adding it to test_stage_packages(), or
    # create mocks.
    @unittest.skip("hangs on google spread test with 'Error in function start'")
    def test_stage_packages(self):
        fetch_dir_path = Path(self.path, "debs")
        fetch_dir_path.mkdir(exist_ok=True, parents=True)
        stage_cache = Path(self.path, "cache")
        stage_cache.mkdir(exist_ok=True, parents=True)

        with AptCache(stage_cache=stage_cache) as apt_cache:
            apt_cache.update()

            package_names = {"pciutils"}
            filtered_names = {"base-files", "libc6", "libkmod2", "libudev1", "zlib1g"}

            apt_cache.mark_packages(package_names)
            apt_cache.unmark_packages(unmark_names=filtered_names)

            marked_packages = apt_cache.get_packages_marked_for_installation()
            self.assertThat(
                sorted([name for name, _ in marked_packages]),
                Equals(["libpci3", "pciutils"]),
            )

            names = []
            for pkg_name, pkg_version, dl_path in apt_cache.fetch_archives(
                fetch_dir_path
            ):
                names.append(pkg_name)
                self.assertThat(dl_path.exists(), Equals(True))
                self.assertThat(dl_path.parent, Equals(fetch_dir_path))
                self.assertThat(isinstance(pkg_version, str), Equals(True))

            self.assertThat(sorted(names), Equals(["libpci3", "pciutils"]))


class TestMockedApt(unit.TestCase):
    def test_stage_cache(self):
        stage_cache = Path(self.path, "cache")
        stage_cache.mkdir(exist_ok=True, parents=True)
        self.fake_apt = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo.apt_cache.apt")
        ).mock

        with AptCache(stage_cache=stage_cache) as apt_cache:
            apt_cache.update()

        self.assertThat(
            self.fake_apt.mock_calls,
            Equals(
                [
                    call.apt_pkg.config.set("Apt::Install-Recommends", "False"),
                    call.apt_pkg.config.set(
                        "Acquire::AllowInsecureRepositories", "False"
                    ),
                    call.apt_pkg.config.set(
                        "Dir::Etc::Trusted", "/etc/apt/trusted.gpg"
                    ),
                    call.apt_pkg.config.set(
                        "Dir::Etc::TrustedParts", "/etc/apt/trusted.gpg.d/"
                    ),
                    call.apt_pkg.config.clear("APT::Update::Post-Invoke-Success"),
                    call.progress.text.AcquireProgress(),
                    call.Cache(memonly=True, rootdir=str(stage_cache)),
                    call.Cache().update(fetch_progress=mock.ANY, sources_list=None),
                    call.Cache().close(),
                    call.Cache(memonly=True, rootdir=str(stage_cache)),
                    call.Cache().close(),
                ]
            ),
        )

    def test_stage_cache_in_snap(self):
        self.fake_apt = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo.apt_cache.apt")
        ).mock

        stage_cache = Path(self.path, "cache")
        stage_cache.mkdir(exist_ok=True, parents=True)

        snap = Path(self.path, "snap")
        snap.mkdir(exist_ok=True, parents=True)

        self.useFixture(
            fixtures.MockPatch("snapcraft.internal.common.is_snap", return_value=True)
        )
        self.useFixture(fixtures.EnvironmentVariable("SNAP", str(snap)))

        with AptCache(stage_cache=stage_cache) as apt_cache:
            apt_cache.update()

        self.assertThat(
            self.fake_apt.mock_calls,
            Equals(
                [
                    call.apt_pkg.config.set("Apt::Install-Recommends", "False"),
                    call.apt_pkg.config.set(
                        "Acquire::AllowInsecureRepositories", "False"
                    ),
                    call.apt_pkg.config.set("Dir", str(Path(snap, "usr/lib/apt"))),
                    call.apt_pkg.config.set(
                        "Dir::Bin::methods",
                        str(Path(snap, "usr/lib/apt/methods")) + os.sep,
                    ),
                    call.apt_pkg.config.set(
                        "Dir::Bin::solvers::",
                        str(Path(snap, "usr/lib/apt/solvers")) + os.sep,
                    ),
                    call.apt_pkg.config.set(
                        "Dir::Bin::apt-key", str(Path(snap, "usr/bin/apt-key"))
                    ),
                    call.apt_pkg.config.set(
                        "Apt::Key::gpgvcommand", str(Path(snap, "usr/bin/gpgv"))
                    ),
                    call.apt_pkg.config.set(
                        "Dir::Etc::Trusted", "/etc/apt/trusted.gpg"
                    ),
                    call.apt_pkg.config.set(
                        "Dir::Etc::TrustedParts", "/etc/apt/trusted.gpg.d/"
                    ),
                    call.apt_pkg.config.clear("APT::Update::Post-Invoke-Success"),
                    call.progress.text.AcquireProgress(),
                    call.Cache(memonly=True, rootdir=str(stage_cache)),
                    call.Cache().update(fetch_progress=mock.ANY, sources_list=None),
                    call.Cache().close(),
                    call.Cache(memonly=True, rootdir=str(stage_cache)),
                    call.Cache().close(),
                ]
            ),
        )

    def test_host_cache_setup(self):
        self.fake_apt = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo.apt_cache.apt")
        ).mock

        with AptCache() as _:
            pass

        self.assertThat(
            self.fake_apt.mock_calls, Equals([call.Cache(), call.Cache().close()])
        )


class TestAptReadonlyHostCache(unit.TestCase):
    def test_host_is_package_valid(self):
        with AptCache() as apt_cache:
            self.assertThat(apt_cache.is_package_valid("apt"), Equals(True))
            self.assertThat(
                apt_cache.is_package_valid("fake-news-bears"), Equals(False)
            )

    def test_host_get_installed_packages(self):
        with AptCache() as apt_cache:
            installed_packages = apt_cache.get_installed_packages()
            self.assertThat(isinstance(installed_packages, dict), Equals(True))
            self.assertThat("apt" in installed_packages, Equals(True))
            self.assertThat("fake-news-bears" in installed_packages, Equals(False))

    def test_host_get_installed_version(self):
        with AptCache() as apt_cache:
            self.assertThat(
                isinstance(apt_cache.get_installed_version("apt"), str), Equals(True)
            )
            self.assertThat(
                apt_cache.get_installed_version("fake-news-bears"), Equals(None)
            )
