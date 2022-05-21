# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2022 Canonical Ltd.
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
import textwrap
from pathlib import Path
from typing import Any, Dict

import pytest

from snapcraft import errors
from snapcraft.parts.setup_assets import _validate_command_chain, setup_assets
from snapcraft.projects import Project


@pytest.fixture
def desktop_file():
    def _write_file(filename: str):
        Path(filename).write_text(
            textwrap.dedent(
                """\
                [Desktop Entry]
                Name=appstream-desktop
                Exec=appstream
                Type=Application
                Icon=/usr/share/icons/my-icon.svg"""
            )
        )

    yield _write_file


@pytest.fixture
def yaml_data():
    def _yaml_data(extra_data: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "name": "test-project",
            "base": "core22",
            "confinement": "strict",
            "parts": {},
            **extra_data,
        }

    yield _yaml_data


@pytest.fixture
def gadget_yaml_file(new_dir):
    Path("gadget.yaml").write_text(
        textwrap.dedent(
            """\
            gadget-key: gadget-value
            """
        )
    )


@pytest.fixture
def kernel_yaml_file(new_dir):
    Path("kernel.yaml").write_text(
        textwrap.dedent(
            """\
            kernel-key: kernel-value
            """
        )
    )


def test_gadget(yaml_data, gadget_yaml_file, new_dir):
    project = Project.unmarshal(
        yaml_data(
            {
                "type": "gadget",
                "version": "1.0",
                "summary": "summary",
                "description": "description",
            }
        )
    )

    setup_assets(
        project,
        assets_dir=Path("snap"),
        project_dir=Path.cwd(),
        prime_dir=Path("prime"),
    )

    # gadget file should be in meta/
    gadget_path = Path("prime/meta/gadget.yaml")
    assert gadget_path.is_file()


def test_gadget_missing(yaml_data, new_dir):
    project = Project.unmarshal(
        yaml_data(
            {
                "type": "gadget",
                "version": "1.0",
                "summary": "summary",
                "description": "description",
            }
        )
    )

    with pytest.raises(errors.SnapcraftError) as raised:
        setup_assets(
            project,
            assets_dir=Path("snap"),
            project_dir=Path.cwd(),
            prime_dir=Path("prime"),
        )

    assert str(raised.value) == "gadget.yaml is required for gadget snaps"


def test_kernel(yaml_data, kernel_yaml_file, new_dir):
    project = Project.unmarshal(
        {
            "name": "custom-kernel",
            "type": "kernel",
            "confinement": "strict",
            "version": "1.0",
            "summary": "summary",
            "description": "description",
            "parts": {},
        }
    )

    setup_assets(
        project,
        assets_dir=Path("snap"),
        project_dir=Path.cwd(),
        prime_dir=Path("prime"),
    )

    # kernel file should be in meta/
    kernel_path = Path("prime/meta/kernel.yaml")
    assert kernel_path.is_file()


def test_kernel_missing(yaml_data, new_dir):
    project = Project.unmarshal(
        {
            "name": "custom-kernel",
            "type": "kernel",
            "confinement": "strict",
            "version": "1.0",
            "summary": "summary",
            "description": "description",
            "parts": {},
        }
    )

    setup_assets(
        project,
        assets_dir=Path("snap"),
        project_dir=Path.cwd(),
        prime_dir=Path("prime"),
    )

    # kernel file should not be in meta/
    kernel_path = Path("prime/meta/kernel.yaml")
    assert not kernel_path.is_file()


class TestSetupAssets:
    """Check copied assets and desktop entries."""

    @pytest.fixture(autouse=True)
    def setup_method_fixture(self, new_dir):
        # create prime tree
        Path("prime").mkdir()
        Path("prime/test.sh").touch()
        Path("prime/test.sh").chmod(0o755)

        # create assets dir
        Path("snap").mkdir()

    def test_setup_assets_happy(self, desktop_file, yaml_data, new_dir):
        desktop_file("prime/test.desktop")
        Path("prime/usr/share/icons").mkdir(parents=True)
        Path("prime/usr/share/icons/my-icon.svg").touch()

        # define project
        project = Project.unmarshal(
            yaml_data(
                {
                    "adopt-info": "part",
                    "apps": {
                        "app1": {
                            "command": "test.sh",
                            "common-id": "my-test",
                            "desktop": "test.desktop",
                        },
                    },
                },
            )
        )

        setup_assets(
            project,
            assets_dir=Path("snap"),
            project_dir=Path.cwd(),
            prime_dir=Path("prime"),
        )

        # desktop file should be in meta/gui and named after app
        desktop_path = Path("prime/meta/gui/app1.desktop")
        assert desktop_path.is_file()

        # desktop file content should make icon relative to ${SNAP}
        content = desktop_path.read_text()
        assert content == textwrap.dedent(
            """\
            [Desktop Entry]
            Name=appstream-desktop
            Exec=test-project.app1
            Type=Application
            Icon=${SNAP}/usr/share/icons/my-icon.svg

            """
        )

    def test_setup_assets_icon_in_assets_dir(self, desktop_file, yaml_data, new_dir):
        desktop_file("prime/test.desktop")
        Path("snap/gui").mkdir(parents=True)
        Path("snap/gui/icon.svg").touch()

        # define project
        project = Project.unmarshal(
            yaml_data(
                {
                    "adopt-info": "part",
                    "apps": {
                        "app1": {
                            "command": "test.sh",
                            "common-id": "my-test",
                            "desktop": "test.desktop",
                        },
                    },
                },
            )
        )

        setup_assets(
            project,
            assets_dir=Path("snap"),
            project_dir=Path.cwd(),
            prime_dir=Path("prime"),
        )

        # desktop file should be in meta/gui and named after app
        desktop_path = Path("prime/meta/gui/app1.desktop")
        assert desktop_path.is_file()

        # desktop file content should make icon relative to ${SNAP}
        content = desktop_path.read_text()
        assert content == textwrap.dedent(
            """\
            [Desktop Entry]
            Name=appstream-desktop
            Exec=test-project.app1
            Type=Application
            Icon=${SNAP}/snap/gui/icon.svg

            """
        )

        # icon file exists
        Path("prime/snap/gui/icon.svg").is_file()

    def test_setup_assets_no_apps(self, desktop_file, yaml_data, new_dir):
        desktop_file("prime/test.desktop")
        Path("prime/usr/share/icons").mkdir(parents=True)
        Path("prime/usr/share/icons/icon.svg").touch()
        Path("snap/gui").mkdir()

        # define project
        project = Project.unmarshal(yaml_data({"adopt-info": "part"}))

        # setting up assets does not crash
        setup_assets(
            project,
            assets_dir=Path("snap"),
            project_dir=Path.cwd(),
            prime_dir=Path("prime"),
        )

        assert os.listdir("prime/meta/gui") == []

    def test_setup_assets_remote_icon(self, desktop_file, yaml_data, new_dir):
        # create primed tree (no icon)
        desktop_file("prime/test.desktop")

        # define project
        # pylint: disable=line-too-long
        project = Project.unmarshal(
            yaml_data(
                {
                    "adopt-info": "part",
                    "icon": "https://dashboard.snapcraft.io/site_media/appmedia/2018/04/Snapcraft-logo-bird.png",
                    "apps": {
                        "app1": {
                            "command": "test.sh",
                            "common-id": "my-test",
                            "desktop": "test.desktop",
                        },
                    },
                },
            )
        )
        # pylint: enable=line-too-long

        setup_assets(
            project,
            assets_dir=Path("snap"),
            project_dir=Path.cwd(),
            prime_dir=Path("prime"),
        )

        # desktop file should be in meta/gui and named after app
        desktop_path = Path("prime/meta/gui/app1.desktop")
        assert desktop_path.is_file()

        # desktop file content should make icon relative to ${SNAP}
        content = desktop_path.read_text()
        assert content == textwrap.dedent(
            """\
            [Desktop Entry]
            Name=appstream-desktop
            Exec=test-project.app1
            Type=Application
            Icon=${SNAP}/meta/gui/icon.png

            """
        )

        # icon was downloaded
        icon_path = Path("prime/meta/gui/icon.png")
        assert icon_path.is_file()
        assert icon_path.stat().st_size > 0


class TestCommandChain:
    """Command chain items are valid."""

    def test_setup_assets_app_command_chain_error(self, yaml_data, new_dir):
        project = Project.unmarshal(
            yaml_data(
                {
                    "adopt-info": "part1",
                    "apps": {
                        "app1": {
                            "command": "test.sh",
                            "command-chain": ["does-not-exist"]
                        },
                    },
                },
            )
        )

        with pytest.raises(errors.SnapcraftError) as raised:
            setup_assets(project, assets_dir=Path("snap"),
                         project_dir=Path.cwd(), prime_dir=new_dir)

        assert str(raised.value) == (
           "Failed to generate snap metadata: The command-chain item 'does-not-exist' "
           "defined in app 'app1' does not exist or is not executable."
        )

    def test_setup_assets_hook_command_chain_error(self, yaml_data, new_dir):
        # define project
        project = Project.unmarshal(
            yaml_data(
                {
                    "adopt-info": "part1",
                    "hooks": {
                        "hook1": {
                            "command-chain": ["does-not-exist"]
                        },
                    },
                },
            )
        )

        with pytest.raises(errors.SnapcraftError) as raised:
            setup_assets(project, assets_dir=Path("snap"),
                         project_dir=Path.cwd(), prime_dir=new_dir)

        assert str(raised.value) == (
           "Failed to generate snap metadata: The command-chain item 'does-not-exist' "
           "defined in hook 'hook1' does not exist or is not executable."
        )


    def test_command_chain_path_not_found(self, new_dir):

        with pytest.raises(errors.SnapcraftError) as raised:
            _validate_command_chain(
                ["file-not-found"], name="foo", prime_dir=new_dir
            )

        assert str(raised.value) == (
            "Failed to generate snap metadata: The command-chain item 'file-not-found' "
            "defined in foo does not exist or is not executable."
        )

    def test_command_chain_path_not_executable(self, new_dir):
        Path("file-executable").touch()
        Path("file-executable").chmod(0o755)

        Path("file-not-executable").touch()

        with pytest.raises(errors.SnapcraftError) as raised:
            _validate_command_chain(
                ["file-executable", "file-not-executable"], name="foo", prime_dir=new_dir,
            )

        assert str(raised.value) == (
            "Failed to generate snap metadata: The command-chain item 'file-not-executable' "
            "defined in foo does not exist or is not executable."
        )
