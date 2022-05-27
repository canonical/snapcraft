# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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


import pytest

from snapcraft import errors, extensions
from snapcraft.extensions import extension


@pytest.mark.usefixtures("fake_extension")
def test_apply_extension():
    yaml_data = {
        "name": "fake-snap",
        "summary": "fake summary",
        "description": "fake description",
        "base": "core22",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "plugs": ["my-fake-plug"],
                "extensions": ["fake-extension"],
            }
        },
        "parts": {"fake-part": {"source": ".", "plugin": "dump"}},
    }

    assert extensions.apply_extensions(
        yaml_data, arch="amd64", target_arch="amd64"
    ) == {
        "name": "fake-snap",
        "summary": "fake summary",
        "description": "fake description",
        "base": "core22",
        "grade": "fake-grade",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "plugs": ["fake-plug", "my-fake-plug"],
            }
        },
        "parts": {
            "fake-part": {
                "source": ".",
                "plugin": "dump",
                "after": ["fake-extension/fake-part"],
            },
            "fake-extension/fake-part": {"plugin": "nil"},
        },
    }


@pytest.mark.usefixtures("fake_extension")
@pytest.mark.usefixtures("fake_extension_extra")
def test_apply_multiple_extensions():
    yaml_data = {
        "name": "fake-snap",
        "summary": "fake summary",
        "description": "fake description",
        "base": "core22",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "plugs": ["my-fake-plug"],
                "extensions": ["fake-extension", "fake-extension-extra"],
            }
        },
        "parts": {"fake-part": {"source": ".", "plugin": "dump"}},
    }

    assert extensions.apply_extensions(
        yaml_data, arch="amd64", target_arch="amd64"
    ) == {
        "name": "fake-snap",
        "summary": "fake summary",
        "description": "fake description",
        "base": "core22",
        "grade": "fake-grade",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "plugs": ["fake-plug", "fake-plug-extra", "my-fake-plug"],
            }
        },
        "parts": {
            "fake-part": {
                "source": ".",
                "plugin": "dump",
                "after": ["fake-extension-extra/fake-part", "fake-extension/fake-part"],
            },
            "fake-extension/fake-part": {
                "plugin": "nil",
                "after": ["fake-extension-extra/fake-part"],
            },
            "fake-extension-extra/fake-part": {"plugin": "nil"},
        },
    }


@pytest.mark.usefixtures("fake_extension")
def test_apply_extension_wrong_base():
    yaml_data = {
        "base": "core20",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "extensions": ["fake-extension"],
            }
        },
    }

    with pytest.raises(errors.ExtensionError) as raised:
        extensions.apply_extensions(yaml_data, arch="amd64", target_arch="amd64")

    assert (
        str(raised.value)
        == "Extension 'fake-extension' does not support base: 'core20'"
    )


@pytest.mark.usefixtures("fake_extension")
def test_apply_extension_wrong_confinement():
    yaml_data = {
        "base": "core22",
        "confinement": "classic",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "extensions": ["fake-extension"],
            }
        },
    }

    with pytest.raises(errors.ExtensionError) as raised:
        extensions.apply_extensions(yaml_data, arch="amd64", target_arch="amd64")

    assert (
        str(raised.value)
        == "Extension 'fake-extension' does not support confinement 'classic'"
    )


@pytest.mark.usefixtures("fake_extension_invalid_parts")
def test_apply_extension_invalid_parts():
    # This is a Snapcraft developer error.
    yaml_data = {
        "base": "core22",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "extensions": ["fake-extension-invalid-parts"],
            }
        },
    }

    with pytest.raises(ValueError) as raised:
        extensions.apply_extensions(yaml_data, arch="amd64", target_arch="amd64")

    assert str(raised.value) == (
        "Extension has invalid part names: ['fake-part', 'fake-part-2']. "
        "Format is <extension-name>/<part-name>"
    )


@pytest.mark.usefixtures("fake_extension_experimental")
def test_apply_extension_experimental():
    yaml_data = {
        "base": "core22",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "extensions": ["fake-extension-experimental"],
            }
        },
    }

    with pytest.raises(errors.ExtensionError) as raised:
        extensions.apply_extensions(yaml_data, arch="amd64", target_arch="amd64")

    assert (
        str(raised.value) == "Extension is experimental: 'fake-extension-experimental'"
    )
    assert raised.value.docs_url == "https://snapcraft.io/docs/supported-extensions"


@pytest.mark.usefixtures("fake_extension_experimental")
def test_apply_extension_experimental_with_environment(emitter, monkeypatch):
    monkeypatch.setenv("SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS", "1")

    yaml_data = {
        "base": "core22",
        "apps": {
            "fake-command": {
                "command": "bin/fake-command",
                "extensions": ["fake-extension-experimental"],
            }
        },
        "parts": {
            "fake-part": {
                "source": ".",
                "plugin": "dump",
                "after": ["fake-extension-extra/fake-part", "fake-extension/fake-part"],
            },
        },
    }

    # Should not raise.
    extensions.apply_extensions(yaml_data, arch="amd64", target_arch="amd64")

    emitter.assert_message(
        "*EXPERIMENTAL* extension 'fake-extension-experimental' enabled",
        intermediate=True,
    )


def test_prepend_path():
    assert (
        extension.prepend_to_env("TEST_ENV", ["/usr/bin", "/usr/local/bin"])
        == "/usr/bin:/usr/local/bin${TEST_ENV:+:$TEST_ENV}"
    )


def test_append_path():
    assert (
        extension.append_to_env("TEST_ENV", ["/usr/bin", "/usr/local/bin"])
        == "${TEST_ENV:+$TEST_ENV:}/usr/bin:/usr/local/bin"
    )


def test_prepend_path_with_separator():
    assert (
        extension.prepend_to_env(
            "TEST_ENV", ["/usr/bin", "/usr/local/bin"], separator=";"
        )
        == "/usr/bin;/usr/local/bin${TEST_ENV:+;$TEST_ENV}"
    )


def test_append_path_with_separator():
    assert (
        extension.append_to_env(
            "TEST_ENV", ["/usr/bin", "/usr/local/bin"], separator=";"
        )
        == "${TEST_ENV:+$TEST_ENV;}/usr/bin;/usr/local/bin"
    )
