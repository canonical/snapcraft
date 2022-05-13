# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021-2022 Canonical Ltd.
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


from textwrap import dedent
from unittest.mock import call

import pytest

from snapcraft.repo import apt_ppa, apt_sources_manager, errors
from snapcraft.repo.package_repository import (
    PackageRepositoryApt,
    PackageRepositoryAptPPA,
)


@pytest.fixture(autouse=True)
def mock_apt_ppa_get_signing_key(mocker):
    yield mocker.patch(
        "snapcraft.repo.apt_ppa.get_launchpad_ppa_key_id",
        spec=apt_ppa.get_launchpad_ppa_key_id,
        return_value="FAKE-PPA-SIGNING-KEY",
    )


@pytest.fixture(autouse=True)
def mock_environ_copy(mocker):
    yield mocker.patch("os.environ.copy")


@pytest.fixture(autouse=True)
def mock_host_arch(mocker):
    m = mocker.patch("snapcraft.utils.get_host_architecture")
    m.return_value = "FAKE-HOST-ARCH"

    yield m


@pytest.fixture(autouse=True)
def mock_run(mocker):
    yield mocker.patch("subprocess.run")


@pytest.fixture(autouse=True)
def mock_version_codename(mocker):
    yield mocker.patch(
        "snapcraft.os_release.OsRelease.version_codename",
        return_value="FAKE-CODENAME",
    )


@pytest.fixture
def apt_sources_mgr(tmp_path):
    sources_list_d = tmp_path / "sources.list.d"
    sources_list_d.mkdir(parents=True)

    yield apt_sources_manager.AptSourcesManager(
        sources_list_d=sources_list_d,
    )


@pytest.mark.parametrize(
    "package_repo,name,content",
    [
        (
            PackageRepositoryApt(
                architectures=["amd64", "arm64"],
                components=["test-component"],
                formats=["deb", "deb-src"],
                key_id="A" * 40,
                suites=["test-suite1", "test-suite2"],
                url="http://test.url/ubuntu",
            ),
            "snapcraft-http_test_url_ubuntu.sources",
            dedent(
                """\
                Types: deb deb-src
                URIs: http://test.url/ubuntu
                Suites: test-suite1 test-suite2
                Components: test-component
                Architectures: amd64 arm64
                """
            ).encode(),
        ),
        (
            PackageRepositoryApt(
                architectures=["amd64", "arm64"],
                components=["test-component"],
                key_id="A" * 40,
                name="NO-FORMAT",
                suites=["test-suite1", "test-suite2"],
                url="http://test.url/ubuntu",
            ),
            "snapcraft-NO-FORMAT.sources",
            dedent(
                """\
                Types: deb
                URIs: http://test.url/ubuntu
                Suites: test-suite1 test-suite2
                Components: test-component
                Architectures: amd64 arm64
                """
            ).encode(),
        ),
        (
            PackageRepositoryApt(
                key_id="A" * 40,
                name="WITH-PATH",
                path="some-path",
                url="http://test.url/ubuntu",
            ),
            "snapcraft-WITH-PATH.sources",
            dedent(
                """\
                Types: deb
                URIs: http://test.url/ubuntu
                Suites: some-path/
                Architectures: FAKE-HOST-ARCH
                """
            ).encode(),
        ),
        (
            PackageRepositoryApt(
                key_id="A" * 40,
                name="IMPLIED-PATH",
                url="http://test.url/ubuntu",
            ),
            "snapcraft-IMPLIED-PATH.sources",
            dedent(
                """\
                Types: deb
                URIs: http://test.url/ubuntu
                Suites: /
                Architectures: FAKE-HOST-ARCH
                """
            ).encode(),
        ),
        (
            PackageRepositoryAptPPA(ppa="test/ppa"),
            "snapcraft-ppa-test_ppa.sources",
            dedent(
                """\
                Types: deb
                URIs: http://ppa.launchpad.net/test/ppa/ubuntu
                Suites: FAKE-CODENAME
                Components: main
                Architectures: FAKE-HOST-ARCH
                """
            ).encode(),
        ),
    ],
)
def test_install(package_repo, name, content, apt_sources_mgr, mocker):
    run_mock = mocker.patch("subprocess.run")
    sources_path = apt_sources_mgr._sources_list_d / name

    changed = apt_sources_mgr.install_package_repository_sources(
        package_repo=package_repo
    )

    assert changed is True
    assert sources_path.read_bytes() == content

    if isinstance(package_repo, PackageRepositoryApt) and package_repo.architectures:
        assert run_mock.mock_calls == [
            call(["dpkg", "--add-architecture", "amd64"], check=True),
            call(["dpkg", "--add-architecture", "arm64"], check=True),
        ]
    else:
        assert run_mock.mock_calls == []

    run_mock.reset_mock()

    # Verify a second-run does not incur any changes.
    changed = apt_sources_mgr.install_package_repository_sources(
        package_repo=package_repo
    )

    assert changed is False
    assert sources_path.read_bytes() == content
    assert run_mock.mock_calls == []


def test_install_ppa_invalid(apt_sources_mgr):
    repo = PackageRepositoryAptPPA(ppa="ppa-missing-slash")

    with pytest.raises(errors.AptPPAInstallError) as raised:
        apt_sources_mgr.install_package_repository_sources(package_repo=repo)

    assert str(raised.value) == (
        "Failed to install PPA 'ppa-missing-slash': invalid PPA format"
    )
