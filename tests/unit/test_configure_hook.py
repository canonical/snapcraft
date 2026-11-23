# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Unit tests for the snapcraft snap configure hook."""

from __future__ import annotations

import importlib.machinery
import importlib.util
from pathlib import Path

import pytest

_HOOK_PATH = Path(__file__).parents[2] / "snap" / "hooks" / "configure"


@pytest.fixture
def configure_hook():
    loader = importlib.machinery.SourceFileLoader(
        "snapcraft_configure_hook", str(_HOOK_PATH)
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


@pytest.fixture
def mock_snap_config(mocker, configure_hook):
    return mocker.patch.object(configure_hook.snaphelpers, "SnapConfigOptions")


def test_configure_hook_uses_snap_config(configure_hook, mock_snap_config, mocker):
    """Verify the hook delegates provider validation to the shared model."""
    data = {"provider": "lxd"}
    mock_snap_config.return_value.as_dict.return_value = data
    unmarshal = mocker.patch.object(configure_hook.SnapConfig, "unmarshal")

    configure_hook.validate_snap_config()

    unmarshal.assert_called_once_with(data)


@pytest.mark.parametrize("provider", ["lxd", "multipass", "LXD", "MultiPass"])
def test_validate_snap_config_valid(configure_hook, mock_snap_config, provider):
    mock_snap_config.return_value.as_dict.return_value = {"provider": provider}

    configure_hook.validate_snap_config()

    mock_snap_config.assert_called_once_with(keys=["provider"])
    mock_snap_config.return_value.fetch.assert_called_once_with()


def test_validate_snap_config_missing_provider(configure_hook, mock_snap_config):
    mock_snap_config.return_value.as_dict.return_value = {}

    configure_hook.validate_snap_config()


@pytest.mark.parametrize("provider", ["invalid", "", 1, ["lxd"], True])
def test_validate_snap_config_invalid(
    configure_hook, mock_snap_config, provider, capsys
):
    mock_snap_config.return_value.as_dict.return_value = {"provider": provider}

    with pytest.raises(SystemExit, match="1"):
        configure_hook.validate_snap_config()

    _, err = capsys.readouterr()
    assert err.startswith("Could not configure snapcraft:")
    assert "Input should be 'lxd' or 'multipass'" in err
