# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Tests for setup_assets with components."""

import shutil
import textwrap
from pathlib import Path

import pytest

from snapcraft.parts.setup_assets import setup_assets


@pytest.fixture
def extra_project_params(extra_project_params):
    from craft_application.models import SummaryStr, VersionStr

    extra_project_params["components"] = {
        "firstcomponent": {
            "type": "test",
            "summary": SummaryStr("first component"),
            "description": "lorem ipsum",
            "version": VersionStr("1.0"),
            "hooks": {
                "configure": {
                    "environment": {
                        "test-variable-1": "test",
                        "test-variable-2": "test",
                    },
                    "plugs": ["home", "network"],
                    "passthrough": {"somefield": ["some", "value"]},
                },
                "install": {},
            },
        },
    }

    return extra_project_params


def test_hooks_default_handler(default_project, new_dir):
    project_dir = new_dir
    assets_dir = project_dir / "snap"
    component_prime_dir = project_dir / "partitions/component/firstcomponent/prime"

    # Create an configure hook provided by the project
    project_configure_hook = assets_dir / "component/firstcomponent/hooks/configure"
    project_configure_hook.parent.mkdir(parents=True)
    project_configure_hook.write_text("project_configure")

    # Create a install hook built through the project
    built_install_hook = component_prime_dir / "snap/hooks/install"
    built_install_hook.parent.mkdir(parents=True)
    built_install_hook.write_text("built_install")

    setup_assets(
        default_project,
        assets_dir=assets_dir,
        project_dir=project_dir,
        prime_dirs={
            None: Path("prime"),
            "firstcomponent": component_prime_dir,
        },
    )

    hook_meta_dir = component_prime_dir / "meta" / "hooks"

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
        component_prime_dir / "snap" / "hooks" / "configure"
    ).read_text() == "project_configure"
    assert oct(meta_configure_hook.stat().st_mode)[-3:] == "755"

    # Assert the project configure hook has its wrapper
    # And is executable
    meta_configure_hook = hook_meta_dir / "configure"
    assert meta_configure_hook.exists()
    assert meta_configure_hook.read_text() == textwrap.dedent(
        """\
        #!/bin/sh
        exec "$SNAP/snap/hooks/configure" "$@"
    """
    )
    assert built_install_hook.read_text() == "built_install"
    assert oct(meta_configure_hook.stat().st_mode)[-3:] == "755"


def test_hooks_with_handler(default_project, new_dir):
    project_dir = new_dir
    assets_dir = project_dir / "snap"
    component_prime_dir = project_dir / "partitions/component/firstcomponent/prime"

    # Create a configure hook provided by the project
    project_configure_hook = (
        assets_dir / "component" / "firstcomponent" / "hooks" / "configure"
    )
    project_configure_hook.parent.mkdir(parents=True)
    project_configure_hook.write_text("project_configure")

    def handler(assets_dir, prime_dir) -> None:
        hooks_meta_dir = prime_dir / "meta" / "hooks"
        hooks_meta_dir.mkdir(parents=True)
        if (assets_dir / "hooks").is_dir():
            for hook in (assets_dir / "hooks").iterdir():
                shutil.copy(hook, hooks_meta_dir / hook.name)

    setup_assets(
        default_project,
        assets_dir=assets_dir,
        project_dir=project_dir,
        prime_dirs={
            None: Path("prime"),
            "firstcomponent": component_prime_dir,
        },
        meta_directory_handler=handler,
    )

    hook_meta_dir = component_prime_dir / "meta" / "hooks"

    # Assert the project configure hook was copied
    # with no wrapper, is executable and that nothing
    # was put in prime_dir/snap/hooks
    meta_configure_hook = hook_meta_dir / "configure"
    assert meta_configure_hook.exists()
    assert meta_configure_hook.read_text() == "project_configure"
    assert not (component_prime_dir / "snap" / "hooks" / "configure").exists()
    assert oct(meta_configure_hook.stat().st_mode)[-3:] == "755"
