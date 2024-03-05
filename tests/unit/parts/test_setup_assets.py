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
import shutil
import textwrap
from pathlib import Path
from typing import Any, Dict
from unittest.mock import call

import pytest

from snapcraft import errors, models
from snapcraft.parts import setup_assets as parts_setup_assets
from snapcraft.parts.setup_assets import (
    _create_hook_wrappers,
    _ensure_hook,
    _ensure_hook_executable,
    _validate_command_chain,
    _write_hook_wrapper,
    setup_assets,
)


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
    project = models.Project.unmarshal(
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
    project = models.Project.unmarshal(
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
    project = models.Project.unmarshal(
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
    project = models.Project.unmarshal(
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


@pytest.mark.parametrize(
    "source,destination,copied",
    [
        (Path("foo"), Path("bar"), True),
        (Path("foo"), Path("foo"), False),
        (Path("foo"), Path("dest/foo"), True),
    ],
)
def test_copy_files(source, destination, copied, new_dir, mocker):
    copy_mock = mocker.patch("shutil.copy")
    Path("foo").touch()
    Path("dest").mkdir()
    Path("dest/foo").write_text("dest")

    parts_setup_assets._copy_file(source, destination)
    if copied:
        assert copy_mock.mock_calls == [call(source, destination)]
    else:
        assert copy_mock.mock_calls == []


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
        project = models.Project.unmarshal(
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

    def test_hooks_default_handler(self, default_project, new_dir):
        project_dir = new_dir
        assets_dir = project_dir / "snap"
        prime_dir = project_dir / "prime"

        # Create a configure hook provided by the project
        project_configure_hook = assets_dir / "hooks" / "configure"
        project_configure_hook.parent.mkdir(parents=True)
        project_configure_hook.write_text("project_configure")

        # Create an install hook built through the project
        built_install_hook = prime_dir / "snap" / "hooks" / "install"
        built_install_hook.parent.mkdir(parents=True)
        built_install_hook.write_text("built_install")

        setup_assets(
            default_project,
            assets_dir=assets_dir,
            project_dir=project_dir,
            prime_dir=prime_dir,
        )

        hook_meta_dir = prime_dir / "meta" / "hooks"

        # Assert the project configure hook has its wrapper
        # to the copied over hook and is executable.
        meta_configure_hook = hook_meta_dir / "configure"
        assert meta_configure_hook.exists()
        assert meta_configure_hook.read_text() == textwrap.dedent(
            """\
            #!/bin/sh
            exec "$SNAP/snap/hooks/configure" "$@"
        """
        )
        assert (
            prime_dir / "snap" / "hooks" / "configure"
        ).read_text() == "project_configure"
        assert oct(meta_configure_hook.stat().st_mode)[-3:] == "755"

        # Assert the project install hook has its wrapper
        # And is executable
        meta_install_hook = hook_meta_dir / "install"
        assert meta_install_hook.exists()
        assert meta_install_hook.read_text() == textwrap.dedent(
            """\
            #!/bin/sh
            exec "$SNAP/snap/hooks/install" "$@"
        """
        )
        assert built_install_hook.read_text() == "built_install"
        assert oct(meta_install_hook.stat().st_mode)[-3:] == "755"

    def test_hooks_with_handler(self, default_project, new_dir):
        project_dir = new_dir
        assets_dir = project_dir / "snap"
        prime_dir = project_dir / "prime"

        # Create a configure hook provided by the project
        project_configure_hook = assets_dir / "hooks" / "configure"
        project_configure_hook.parent.mkdir(parents=True)
        project_configure_hook.write_text("project_configure")

        def handler(assets_dir, prime_dir) -> None:
            hooks_meta_dir = prime_dir / "meta" / "hooks"
            hooks_meta_dir.mkdir(parents=True)
            for hook in (assets_dir / "hooks").iterdir():
                shutil.copy(hook, hooks_meta_dir / hook.name)

        setup_assets(
            default_project,
            assets_dir=assets_dir,
            project_dir=project_dir,
            prime_dir=prime_dir,
            meta_directory_handler=handler,
        )

        hook_meta_dir = prime_dir / "meta" / "hooks"

        # Assert the project configure hook was copied
        # with no wrapper, is executable and that nothing
        # was put in prime_dir/snap/hooks
        meta_configure_hook = hook_meta_dir / "configure"
        assert meta_configure_hook.exists()
        assert meta_configure_hook.read_text() == "project_configure"
        assert not (prime_dir / "snap" / "hooks" / "configure").exists()
        assert oct(meta_configure_hook.stat().st_mode)[-3:] == "755"

    def test_setup_assets_icon_in_assets_dir(self, desktop_file, yaml_data, new_dir):
        desktop_file("prime/test.desktop")
        Path("snap/gui").mkdir(parents=True)
        Path("snap/gui/icon.svg").touch()

        # define project
        project = models.Project.unmarshal(
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
        project = models.Project.unmarshal(yaml_data({"adopt-info": "part"}))

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
        project = models.Project.unmarshal(
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
        project = models.Project.unmarshal(
            yaml_data(
                {
                    "adopt-info": "part1",
                    "apps": {
                        "app1": {
                            "command": "test.sh",
                            "command-chain": ["does-not-exist"],
                        },
                    },
                },
            )
        )

        with pytest.raises(errors.SnapcraftError) as raised:
            setup_assets(
                project,
                assets_dir=Path("snap"),
                project_dir=Path.cwd(),
                prime_dir=new_dir,
            )

        assert str(raised.value) == (
            "Failed to generate snap metadata: The command-chain item 'does-not-exist' "
            "defined in app 'app1' does not exist or is not executable."
        )

    def test_setup_assets_hook_command_chain_error(self, yaml_data, new_dir):
        # define project
        project = models.Project.unmarshal(
            yaml_data(
                {
                    "adopt-info": "part1",
                    "hooks": {
                        "hook1": {"command-chain": ["does-not-exist"]},
                    },
                },
            )
        )

        with pytest.raises(errors.SnapcraftError) as raised:
            setup_assets(
                project,
                assets_dir=Path("snap"),
                project_dir=Path.cwd(),
                prime_dir=new_dir,
            )

        assert str(raised.value) == (
            "Failed to generate snap metadata: The command-chain item 'does-not-exist' "
            "defined in hook 'hook1' does not exist or is not executable."
        )

    def test_command_chain_path_not_found(self, new_dir):
        with pytest.raises(errors.SnapcraftError) as raised:
            _validate_command_chain(["file-not-found"], name="foo", prime_dir=new_dir)

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
                ["file-executable", "file-not-executable"],
                name="foo",
                prime_dir=new_dir,
            )

        assert str(raised.value) == (
            "Failed to generate snap metadata: The command-chain item 'file-not-executable' "
            "defined in foo does not exist or is not executable."
        )


def test_ensure_hook(new_dir):
    """Verify creation of executable placeholder hooks."""
    hook_path: Path = new_dir / "configure"
    _ensure_hook(hook_path)

    assert hook_path.exists()
    assert hook_path.read_text() == "#!/bin/true\n"


def test_ensure_hook_does_not_overwrite(new_dir):
    """Verify existing hooks are not overwritten with placeholder hooks."""
    hook_path: Path = new_dir / "configure"
    hook_path.write_text("#!/bin/python3\n")
    hook_path.chmod(0o700)

    _ensure_hook(hook_path)

    assert hook_path.exists()
    assert hook_path.read_text() == "#!/bin/python3\n"
    assert oct(hook_path.stat().st_mode)[-3:] == "700"


def test_ensure_hook_executable(new_dir):
    """Verify _ensure_hook_executable makes a file executable."""
    # create a non-executable file
    hook_path: Path = new_dir / "configure"
    hook_path.write_text("#!/bin/true\n")
    hook_path.chmod(0o644)

    _ensure_hook_executable(hook_path)

    assert hook_path.exists()
    assert oct(hook_path.stat().st_mode)[-3:] == "755"


def test_create_hook_wrappers(new_dir):
    """Verify hook wrappers are created for generated hooks."""
    # create a directory for the hooks
    hooks_snap_dir: Path = new_dir / "snap" / "hooks"
    hooks_snap_dir.mkdir(parents=True)

    # create 2 generated hooks
    for hook_name in ["configure", "install"]:
        hook: Path = hooks_snap_dir / hook_name
        hook.write_text("#!/bin/true\n")
        hook.chmod(0o644)

    _create_hook_wrappers(new_dir)

    # verify prime/meta/hooks directory was created
    hooks_meta_dir = new_dir / "meta" / "hooks"
    assert hooks_meta_dir.exists()

    # verify wrappers were generated for each hook
    for hook_name in ["configure", "install"]:
        hook_wrapper = hooks_meta_dir / hook_name
        assert hook_wrapper.exists()


def test_write_hook_wrapper(new_dir):
    """Verify hook wrappers are written correctly."""
    # create a directory for the hooks
    hooks_dir: Path = new_dir / "hooks"
    hooks_dir.mkdir()

    # create a generated hook
    hook_name = "configure"
    hook: Path = hooks_dir / hook_name
    hook.write_text("#!/bin/true\n")
    hook.chmod(0o644)

    # create a directory for the hook wrappers
    hook_wrapper_dir = new_dir / "hook-wrappers"
    hook_wrapper_dir.mkdir()
    hook_wrapper = hook_wrapper_dir / hook_name

    _write_hook_wrapper(hook_name, hook_wrapper)

    # verify content of hook wrapper
    assert hook_wrapper.exists()
    assert (
        hook_wrapper.read_text()
        == f'#!/bin/sh\nexec "$SNAP/snap/hooks/{hook_name}" "$@"\n'
    )
