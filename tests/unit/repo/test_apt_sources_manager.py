# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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


import pathlib
import subprocess
from textwrap import dedent
from unittest import mock
from unittest.mock import call

import pytest

from snapcraft.internal.meta.package_repository import (
    PackageRepositoryApt,
    PackageRepositoryAptPpa,
)
from snapcraft.internal.repo import apt_ppa, apt_sources_manager, errors


@pytest.fixture(autouse=True)
def mock_apt_ppa_get_signing_key():
    with mock.patch(
        "snapcraft.internal.repo.apt_ppa.get_launchpad_ppa_key_id",
        spec=apt_ppa.get_launchpad_ppa_key_id,
        return_value="FAKE-PPA-SIGNING-KEY",
    ) as m:
        yield m


@pytest.fixture(autouse=True)
def mock_environ_copy():
    with mock.patch("os.environ.copy") as m:
        yield m


@pytest.fixture(autouse=True)
def mock_host_arch():
    with mock.patch("snapcraft.internal.repo.apt_sources_manager.ProjectOptions") as m:
        m.return_value.deb_arch = "FAKE-HOST-ARCH"
        yield m


@pytest.fixture(autouse=True)
def mock_run():
    with mock.patch("subprocess.run") as m:
        yield m


@pytest.fixture()
def mock_sudo_write():
    def write_file(*, dst_path: pathlib.Path, content: bytes) -> None:
        dst_path.write_bytes(content)

    with mock.patch(
        "snapcraft.internal.repo.apt_sources_manager._sudo_write_file"
    ) as m:
        m.side_effect = write_file
        yield m


@pytest.fixture(autouse=True)
def mock_version_codename():
    with mock.patch(
        "snapcraft.internal.os_release.OsRelease.version_codename",
        return_value="FAKE-CODENAME",
    ) as m:
        yield m


@pytest.fixture
def apt_sources_mgr(tmp_path):
    sources_list_d = tmp_path / "sources.list.d"
    sources_list_d.mkdir(parents=True)

    yield apt_sources_manager.AptSourcesManager(sources_list_d=sources_list_d,)


@mock.patch("tempfile.NamedTemporaryFile")
@mock.patch("os.unlink")
def test_sudo_write_file(mock_unlink, mock_tempfile, mock_run, tmp_path):
    mock_tempfile.return_value.__enter__.return_value.name = "/tmp/foobar"

    apt_sources_manager._sudo_write_file(dst_path="/foo/bar", content=b"some-content")

    assert mock_tempfile.mock_calls == [
        call(delete=False),
        call().__enter__(),
        call().__enter__().write(b"some-content"),
        call().__enter__().flush(),
        call().__exit__(None, None, None),
    ]
    assert mock_run.mock_calls == [
        call(
            [
                "sudo",
                "install",
                "--owner=root",
                "--group=root",
                "--mode=0644",
                "/tmp/foobar",
                "/foo/bar",
            ],
            check=True,
        )
    ]
    assert mock_unlink.mock_calls == [call("/tmp/foobar")]


def test_sudo_write_file_fails(mock_run):
    mock_run.side_effect = subprocess.CalledProcessError(
        cmd=["sudo"], returncode=1, output=b"some error"
    )

    with pytest.raises(RuntimeError) as error:
        apt_sources_manager._sudo_write_file(
            dst_path="/foo/bar", content=b"some-content"
        )

    assert (
        str(error.value).startswith(
            "Failed to install repository config with: ['sudo', 'install'"
        )
        is True
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
                key_id="A" * 40, name="IMPLIED-PATH", url="http://test.url/ubuntu",
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
            PackageRepositoryAptPpa(ppa="test/ppa"),
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
def test_install(package_repo, name, content, apt_sources_mgr, mock_sudo_write):
    sources_path = apt_sources_mgr._sources_list_d / name

    changed = apt_sources_mgr.install_package_repository_sources(
        package_repo=package_repo
    )

    assert changed is True
    assert sources_path.read_bytes() == content
    assert mock_sudo_write.mock_calls == [call(content=content, dst_path=sources_path,)]

    # Verify a second-run does not incur any changes.
    mock_sudo_write.reset_mock()

    changed = apt_sources_mgr.install_package_repository_sources(
        package_repo=package_repo
    )

    assert changed is False
    assert sources_path.read_bytes() == content
    assert mock_sudo_write.mock_calls == []


def test_install_ppa_invalid(apt_sources_mgr):
    repo = PackageRepositoryAptPpa(ppa="ppa-missing-slash")

    with pytest.raises(errors.AptPPAInstallError) as exc_info:
        apt_sources_mgr.install_package_repository_sources(package_repo=repo)

    assert exc_info.value._ppa == "ppa-missing-slash"
