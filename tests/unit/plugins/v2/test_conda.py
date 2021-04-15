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

import platform

import pytest

from snapcraft.plugins.v2.conda import (
    CondaPlugin,
    ArchitectureMissing,
    _get_miniconda_source,
)


def test_schema():
    assert CondaPlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "conda-packages": {
                "type": "array",
                "minItems": 1,
                "uniqueItems": True,
                "items": {"type": "string"},
                "default": [],
            },
            "conda-python-version": {"type": "string", "default": ""},
            "conda-miniconda-version": {"type": "string", "default": "latest"},
        },
    }


def test_get_build_packages():
    plugin = CondaPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_packages() == {"curl"}


def test_get_build_environment():
    plugin = CondaPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_environment() == {"PATH": "${HOME}/miniconda/bin:${PATH}"}


def test_get_build_commands():
    class Options:
        conda_packages = ["python"]
        conda_python_version = ""
        conda_miniconda_version = "latest"

    plugin = CondaPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'if ! [ -e "${HOME}/miniconda.sh" ]; then\n    curl --proto \'=https\' --tlsv1.2 -sSf https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh > ${HOME}/miniconda.sh\n    chmod 755 ${HOME}/miniconda.sh\n    export PATH="${HOME}/miniconda/bin:${PATH}"\nfi\n',
        "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
        "CONDA_TARGET_PREFIX_OVERRIDE=/snap/${SNAPCRAFT_PROJECT_NAME}/current conda create --prefix $SNAPCRAFT_PART_INSTALL --yes python",
    ]


def test_get_build_commands_with_python_version_parameters():
    class Options:
        conda_packages = ["python"]
        conda_python_version = "3.7"
        conda_miniconda_version = "latest"

    plugin = CondaPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        'if ! [ -e "${HOME}/miniconda.sh" ]; then\n    curl --proto \'=https\' --tlsv1.2 -sSf https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh > ${HOME}/miniconda.sh\n    chmod 755 ${HOME}/miniconda.sh\n    export PATH="${HOME}/miniconda/bin:${PATH}"\nfi\n',
        "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
        "CONDA_TARGET_PREFIX_OVERRIDE=/snap/${SNAPCRAFT_PROJECT_NAME}/current conda create --prefix $SNAPCRAFT_PART_INSTALL --yes python=3.7 python",
    ]


@pytest.mark.parametrize(
    "arch",
    [("amd64", "x86_64"), ("ppc64el", "ppc64le"), ("armhf", "armv7l"), ("i386", "x86")],
)
def test_get_miniconda_source_with_snap_arch(monkeypatch, arch):
    monkeypatch.setenv("SNAP_ARCH", arch[0])

    assert (
        _get_miniconda_source(version="1.9")
        == f"https://repo.anaconda.com/miniconda/Miniconda3-1.9-Linux-{arch[1]}.sh"
    )


def test_get_miniconda_source_unsupported_snap_arch(monkeypatch):
    # raising False as this may only be set by a running
    # environment triggering the tests.
    monkeypatch.setenv("SNAP_ARCH", "arm64")

    with pytest.raises(ArchitectureMissing):
        _get_miniconda_source(version="1.9")


def test_get_miniconda_source_unsupported_dev_env(monkeypatch):
    monkeypatch.setattr(platform, "machine", lambda: "foo")
    monkeypatch.delenv("SNAP_ARCH", raising=False)

    with pytest.raises(KeyError):
        _get_miniconda_source(version="1.9")


def test_architecture_missing_exception():
    exception = ArchitectureMissing("amd64")

    assert (
        exception.get_brief()
        == "Architecture 'amd64' is not supported with the 'conda' plugin."
    )
    assert (
        exception.get_resolution()
        == "Ensure running the build on a supported architecture for this plugin."
    )
    assert exception.get_details() is None
    assert exception.get_exit_code() == 2
    assert exception.get_docs_url() is None
