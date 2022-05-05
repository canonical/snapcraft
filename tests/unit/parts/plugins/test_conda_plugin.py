# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os

import pytest
from craft_parts import Part, PartInfo, ProjectInfo
from pydantic import ValidationError

from snapcraft import errors
from snapcraft.parts.plugins import CondaPlugin
from snapcraft.parts.plugins.conda_plugin import _get_miniconda_source


@pytest.fixture(autouse=True)
def part_info(new_dir):
    yield PartInfo(
        project_info=ProjectInfo(
            application_name="test", project_name="test-snap", cache_dir=new_dir
        ),
        part=Part("my-part", {}),
    )


@pytest.fixture
def fake_platform(monkeypatch):
    if os.getenv("SNAP_ARCH"):
        monkeypatch.delenv("SNAP_ARCH")
    monkeypatch.setattr("platform.machine", lambda: "x86_64")
    monkeypatch.setattr("platform.architecture", lambda: ("64bit", "ELF"))


@pytest.mark.usefixtures("fake_platform")
class TestPluginCondaPlugin:
    """Conda plugin tests."""

    def test_get_build_snaps(self, part_info):
        properties = CondaPlugin.properties_class.unmarshal({})
        plugin = CondaPlugin(properties=properties, part_info=part_info)
        assert plugin.get_build_snaps() == set()

    def test_get_build_packages(self, part_info):
        properties = CondaPlugin.properties_class.unmarshal({})
        plugin = CondaPlugin(properties=properties, part_info=part_info)
        assert plugin.get_build_packages() == set()

    def test_get_build_environment(self, part_info):
        properties = CondaPlugin.properties_class.unmarshal({})
        plugin = CondaPlugin(properties=properties, part_info=part_info)

        assert plugin.get_build_environment() == {
            "PATH": "${HOME}/miniconda/bin:${PATH}"
        }

    def test_get_build_commands(self, part_info):
        properties = CondaPlugin.properties_class.unmarshal({})
        plugin = CondaPlugin(properties=properties, part_info=part_info)

        assert plugin.get_build_commands() == [
            'if ! [ -e "${HOME}/miniconda.sh" ]; then\n'
            "    curl --proto '=https' --tlsv1.2 -sSf "
            "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh "
            "> ${HOME}/miniconda.sh\n"
            "    chmod 755 ${HOME}/miniconda.sh\n"
            "fi",
            "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
            (
                "CONDA_TARGET_PREFIX_OVERRIDE=/snap/test-snap/current conda create --prefix "
                f"{plugin._part_info.part_install_dir!s} "
                "--yes"
            ),
        ]

    def test_get_build_commands_conda_packages(self, part_info):
        properties = CondaPlugin.properties_class.unmarshal(
            {"conda-packages": ["test-package-1", "test-package-2"]}
        )
        plugin = CondaPlugin(properties=properties, part_info=part_info)

        assert plugin.get_build_commands() == [
            'if ! [ -e "${HOME}/miniconda.sh" ]; then\n'
            "    curl --proto '=https' --tlsv1.2 -sSf "
            "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh "
            "> ${HOME}/miniconda.sh\n"
            "    chmod 755 ${HOME}/miniconda.sh\n"
            "fi",
            "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
            (
                "CONDA_TARGET_PREFIX_OVERRIDE=/snap/test-snap/current conda create --prefix "
                f"{plugin._part_info.part_install_dir!s} "
                "--yes "
                "test-package-1 test-package-2"
            ),
        ]

    @pytest.mark.parametrize("value", [None, []])
    def test_get_build_commands_conda_packages_empty(self, part_info, value):
        properties = CondaPlugin.properties_class.unmarshal({"conda-packages": value})
        plugin = CondaPlugin(properties=properties, part_info=part_info)

        assert plugin.get_build_commands() == [
            'if ! [ -e "${HOME}/miniconda.sh" ]; then\n'
            "    curl --proto '=https' --tlsv1.2 -sSf "
            "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh "
            "> ${HOME}/miniconda.sh\n"
            "    chmod 755 ${HOME}/miniconda.sh\n"
            "fi",
            "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
            (
                "CONDA_TARGET_PREFIX_OVERRIDE=/snap/test-snap/current conda create --prefix "
                f"{plugin._part_info.part_install_dir!s} "
                "--yes"
            ),
        ]

    @pytest.mark.parametrize(
        "conda_packages",
        ["i am a string", {"i am": "a dictionary"}],
    )
    def test_get_build_commands_conda_packages_invalid(self, conda_packages):
        with pytest.raises(ValidationError):
            CondaPlugin.properties_class.unmarshal({"conda-packages": conda_packages})

    def test_get_build_commands_conda_python_version(self, part_info):
        properties = CondaPlugin.properties_class.unmarshal(
            {"conda-python-version": "3.9"}
        )
        plugin = CondaPlugin(properties=properties, part_info=part_info)

        assert plugin.get_build_commands() == [
            'if ! [ -e "${HOME}/miniconda.sh" ]; then\n'
            "    curl --proto '=https' --tlsv1.2 -sSf "
            "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh "
            "> ${HOME}/miniconda.sh\n"
            "    chmod 755 ${HOME}/miniconda.sh\n"
            "fi",
            "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
            (
                "CONDA_TARGET_PREFIX_OVERRIDE=/snap/test-snap/current conda create --prefix "
                f"{plugin._part_info.part_install_dir!s} "
                "--yes python=3.9"
            ),
        ]

    @pytest.mark.parametrize(
        "conda_python_version",
        [{"i am": "a dictionary"}, ["i am", "a list"]],
    )
    def test_get_build_commands_conda_python_version_invalid(
        self, conda_python_version
    ):
        with pytest.raises(ValidationError):
            CondaPlugin.properties_class.unmarshal(
                {"conda-python-version": conda_python_version}
            )

    @pytest.mark.parametrize(
        "conda_install_prefix",
        [{"i am": "a dictionary"}, ["i am", "a list"]],
    )
    def test_get_build_commands_conda_install_prefix_invalid(
        self, conda_install_prefix
    ):
        with pytest.raises(ValidationError):
            CondaPlugin.properties_class.unmarshal(
                {"conda-install-prefix": conda_install_prefix}
            )


@pytest.mark.parametrize(
    "snap_arch, url_arch",
    [
        ("i386", "x86"),
        ("amd64", "x86_64"),
        ("armhf", "armv7l"),
        ("ppc64el", "ppc64le"),
    ],
)
def test_get_miniconda(monkeypatch, snap_arch, url_arch):
    monkeypatch.setenv("SNAP_ARCH", snap_arch)

    assert _get_miniconda_source("latest") == (
        f"https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-{url_arch}.sh"
    )


@pytest.mark.parametrize(
    "snap_arch",
    ("s390x", "other", "not-supported", "new-arch"),
)
def test_get_miniconda_unsupported_arch(
    monkeypatch,
    snap_arch,
):
    monkeypatch.setenv("SNAP_ARCH", snap_arch)

    with pytest.raises(errors.SnapcraftError) as raised:
        _get_miniconda_source("latest")

    assert str(raised.value) == (
        f"Architecture not supported for conda plugin: {snap_arch!r}"
    )
