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

import pytest

from snapcraft.extensions import env_injector

############
# Fixtures #
############


@pytest.fixture
def envinjector_extension():
    return env_injector.EnvInjector(
        yaml_data={"base": "core24", "parts": {}}, arch="amd64", target_arch="amd64"
    )


#########################
# EnvInjector Extension #
#########################


def test_get_supported_bases(envinjector_extension):
    assert envinjector_extension.get_supported_bases() == ("core24",)


def test_get_supported_confinement(envinjector_extension):
    assert envinjector_extension.get_supported_confinement() == (
        "strict",
        "devmode",
        "classic",
    )


def test_is_experimental():
    assert env_injector.EnvInjector.is_experimental(base="core24") is True


def test_get_root_snippet(envinjector_extension):
    assert envinjector_extension.get_root_snippet() == {}


def test_get_app_snippet(envinjector_extension):
    assert envinjector_extension.get_app_snippet(app_name="test") == {
        "command-chain": ["bin/command-chain/env-exporter"],
        "environment": {
            "env_alias": "test",
        },
    }


def test_get_toolchain_amd64(envinjector_extension):
    assert envinjector_extension.get_toolchain("amd64") == "x86_64-unknown-linux-gnu"


def test_get_toolchain_arm64(envinjector_extension):
    assert envinjector_extension.get_toolchain("arm64") == "aarch64-unknown-linux-gnu"


class TestGetPartSnippet:
    """Tests for EnvInjector.get_part_snippet when using the default sdk snap name."""

    def test_get_part_snippet(self, envinjector_extension):
        self.assert_get_part_snippet(envinjector_extension)

    @staticmethod
    def assert_get_part_snippet(envinjector_extension):
        assert envinjector_extension.get_part_snippet(plugin_name="nil") == {}

    @pytest.mark.parametrize(
        "unsupported_arch", ["armhf", "riscv64", "ppc64el", "s390x"]
    )
    def test_get_parts_snippet(self, envinjector_extension, unsupported_arch):
        toolchain = "x86_64-unknown-linux-gnu"
        assert envinjector_extension.get_parts_snippet() == {
            "env-injector/env-injector": {
                "source": "https://github.com/canonical/snappy-env.git",
                "source-tag": "v1.0.0-beta",
                "plugin": "nil",
                "build-snaps": ["rustup"],
                "override-build": f"""
                rustup default stable
                rustup target add {toolchain}

                cargo build --target {toolchain} --release
                mkdir -p $SNAPCRAFT_PART_INSTALL/bin/command-chain
                
                cp target/{toolchain}/release/env-exporter $SNAPCRAFT_PART_INSTALL/bin/command-chain
                """,
            }
        }

        envinjector_extension.arch = unsupported_arch
        with pytest.raises(
            ValueError, match="Unsupported architecture for env-injector extension"
        ):
            envinjector_extension.get_parts_snippet()
