# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Unit tests for SnapConfig class."""

import ast
import os
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from snaphelpers import SnapCtlError

from snapcraft.snap_config import (
    SnapConfig,
    get_snap_config,
    is_snapcraft_running_from_snap,
)

_SNAP_CONFIG_PATH = Path(__file__).parents[2] / "snapcraft" / "snap_config.py"


@pytest.fixture
def mock_config():
    with patch(
        "snapcraft.snap_config.SnapConfigOptions", autospec=True
    ) as mock_snap_config:
        yield mock_snap_config


@pytest.fixture()
def mock_is_running_from_snap(mocker):
    yield mocker.patch(
        "snapcraft.snap_config.is_snapcraft_running_from_snap", return_value=True
    )


def test_module_imports():
    """Verify snap_config does not import craft-application, parts, or providers."""
    tree = ast.parse(_SNAP_CONFIG_PATH.read_text(encoding="utf-8"))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(
                alias.name.split(".", maxsplit=1)[0] for alias in node.names
            )
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".", maxsplit=1)[0])

    assert not {"craft_application", "craft_parts", "craft_providers"}.intersection(
        imported
    )


def test_module_imports_in_fresh_process():
    """Verify loading the model and hook avoids heavy indirect imports."""
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    process = subprocess.run(
        [
            sys.executable,
            "-c",
            "import runpy, sys; "
            "import snapcraft.snap_config; "
            "runpy.run_path('snap/hooks/configure'); "
            "heavy = {'craft_application', 'craft_parts', 'craft_providers'}"
            ".intersection(sys.modules); "
            "assert not heavy, heavy",
        ],
        cwd=_SNAP_CONFIG_PATH.parents[1],
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert process.returncode == 0, process.stderr


@pytest.mark.parametrize("provider", ["lxd", "multipass", "LXD", "MultiPass"])
def test_unmarshal(provider):
    """Verify unmarshalling works as expected."""
    config = SnapConfig.unmarshal({"provider": provider})

    assert config.provider == provider.lower()


@pytest.mark.parametrize("data", [{}, {"provider": None}])
def test_unmarshal_default_provider(data):
    """Verify an empty provider stays unset."""
    assert SnapConfig.unmarshal(data).provider is None


@pytest.mark.parametrize("data", [None, [], "lxd"])
def test_unmarshal_not_a_dict(data):
    """Verify non-dictionary data raises TypeError."""
    with pytest.raises(TypeError, match="Project data is not a dictionary"):
        SnapConfig.unmarshal(data)


@pytest.mark.parametrize("provider", ["multipass", "MultiPass"])
def test_validate_assignment(provider):
    """Verify assigning a provider normalizes its case."""
    config = SnapConfig(provider="lxd")
    config.provider = provider
    assert config.provider == provider.lower()


@pytest.mark.parametrize("provider", ["invalid-value", 1, ["lxd"]])
def test_validate_assignment_invalid(provider):
    """Verify an invalid provider assignment raises an error."""
    config = SnapConfig(provider="lxd")
    with pytest.raises(ValueError, match="Input should be 'lxd' or 'multipass'"):
        config.provider = provider


@pytest.mark.parametrize(
    ("snap_name", "snap_path", "expected"),
    [
        (None, None, False),
        ("snapcraft", None, False),
        (None, "/snap/snapcraft/current", False),
        ("other", "/snap/other/current", False),
        ("snapcraft", "/snap/snapcraft/current", True),
        ("snapcraft", "", True),
    ],
)
def test_is_snapcraft_running_from_snap(monkeypatch, snap_name, snap_path, expected):
    """Verify snap detection from SNAP_NAME and SNAP."""
    for key, value in (("SNAP_NAME", snap_name), ("SNAP", snap_path)):
        monkeypatch.delenv(key, raising=False)
        if value is not None:
            monkeypatch.setenv(key, value)

    assert is_snapcraft_running_from_snap() is expected


def test_unmarshal_invalid_provider_error():
    """Verify unmarshalling with an invalid provider raises an error."""
    error = "provider\n  Input should be 'lxd' or 'multipass'"
    with pytest.raises(ValueError, match=error):
        SnapConfig.unmarshal({"provider": "invalid-value"})


def test_unmarshal_extra_data_error():
    """Verify unmarshalling with extra data raises an error."""
    error = "test\n  Extra inputs are not permitted"
    with pytest.raises(ValueError, match=error):
        SnapConfig.unmarshal({"provider": "lxd", "test": "test"})


@pytest.mark.parametrize("provider", ["lxd", "multipass"])
def test_get_snap_config(mock_config, mock_is_running_from_snap, provider):
    """Verify getting a valid snap config."""

    def fake_as_dict():
        return {"provider": provider}

    mock_config.return_value.as_dict.side_effect = fake_as_dict
    config = get_snap_config()

    assert config == SnapConfig(provider=provider)


def test_get_snap_config_empty(mock_config, mock_is_running_from_snap):
    """Verify getting an empty config returns a default SnapConfig."""

    def fake_as_dict():
        return {}

    mock_config.return_value.as_dict.side_effect = fake_as_dict
    config = get_snap_config()

    assert config == SnapConfig()


def test_get_snap_config_not_from_snap(mock_is_running_from_snap):
    """Verify None is returned when snapcraft is not running from a snap."""
    mock_is_running_from_snap.return_value = False

    assert get_snap_config() is None


@pytest.mark.parametrize("error", [AttributeError, SnapCtlError(process=MagicMock())])
def test_get_snap_config_handle_init_error(
    error, mock_config, mock_is_running_from_snap
):
    """An error when initializing the snap config object should return None."""
    mock_config.side_effect = error

    assert get_snap_config() is None


@pytest.mark.parametrize("error", [AttributeError, SnapCtlError(process=MagicMock())])
def test_get_snap_config_handle_fetch_error(
    error, mock_config, mock_is_running_from_snap
):
    """An error when fetching the snap config should return None."""
    mock_config.return_value.fetch.side_effect = error

    assert get_snap_config() is None
