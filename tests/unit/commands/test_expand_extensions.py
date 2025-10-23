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

from argparse import Namespace
from dataclasses import dataclass
from pathlib import Path
from textwrap import dedent

import pytest
from craft_platforms import DebianArchitecture

from snapcraft import commands, const


@dataclass
class CoreData:
    """Dataclass containing base info for a given core."""

    base: str
    build_base: str
    grade: str


@pytest.fixture(params=const.CURRENT_BASES - {"core22", "devel"})
def valid_core_data(request) -> CoreData:
    """Fixture that provides valid base, build-base and grade values for each base."""
    # special handling for the in-development base
    if request.param == "core26":
        return CoreData(base="core26", build_base="devel", grade="devel")

    return CoreData(base=request.param, build_base=request.param, grade="stable")


@pytest.mark.usefixtures("fake_extension")
def test_expand_extensions_simple_core22(new_dir, emitter, fake_app_config):
    """Expand an extension for a simple snapcraft.yaml file."""
    with Path("snapcraft.yaml").open("w") as yaml_file:
        print(
            dedent(
                """\
            name: test-name
            version: "0.1"
            summary: testing extensions
            description: expand a fake extension
            base: core22
            confinement: strict
            grade: stable

            apps:
                app1:
                    command: app1
                    command-chain: [fake-command]
                    extensions: [fake-extension]

            parts:
                part1:
                    plugin: nil
            """
            ),
            file=yaml_file,
        )

    cmd = commands.ExpandExtensionsCommand(fake_app_config)
    cmd.run(Namespace())
    emitter.assert_message(
        dedent(
            f"""\
        name: test-name
        version: '0.1'
        summary: testing extensions
        description: expand a fake extension
        base: core22
        parts:
          part1:
            plugin: nil
            after:
            - fake-extension/fake-part
          fake-extension/fake-part:
            plugin: nil
        confinement: strict
        grade: stable
        architectures:
        - build-on:
          - {DebianArchitecture.from_host()}
          build-for:
          - {DebianArchitecture.from_host()}
        apps:
          app1:
            command: app1
            plugs:
            - fake-plug
            command-chain:
            - fake-command
        """
        )
    )


@pytest.mark.usefixtures("fake_extension")
def test_expand_extensions_simple(new_dir, emitter, valid_core_data, fake_app_config):
    """Expand an extension for a simple snapcraft.yaml file."""
    with Path("snapcraft.yaml").open("w") as yaml_file:
        print(
            dedent(
                f"""\
            name: test-name
            version: "0.1"
            summary: testing extensions
            description: expand a fake extension
            base: {valid_core_data.base}
            build-base: {valid_core_data.build_base}
            confinement: strict
            grade: {valid_core_data.grade}

            apps:
                app1:
                    command: app1
                    command-chain: [fake-command]
                    extensions: [fake-extension]

            parts:
                part1:
                    plugin: nil
            """
            ),
            file=yaml_file,
        )

    cmd = commands.ExpandExtensionsCommand(fake_app_config)
    cmd.run(Namespace())
    emitter.assert_message(
        dedent(
            f"""\
        name: test-name
        version: '0.1'
        summary: testing extensions
        description: expand a fake extension
        base: {valid_core_data.base}
        build-base: {valid_core_data.build_base}
        parts:
          part1:
            plugin: nil
            after:
            - fake-extension/fake-part
          fake-extension/fake-part:
            plugin: nil
        confinement: strict
        grade: {valid_core_data.grade}
        apps:
          app1:
            command: app1
            plugs:
            - fake-plug
            command-chain:
            - fake-command
        """
        )
    )


@pytest.mark.usefixtures("fake_extension")
def test_expand_extensions_complex_core22(new_dir, emitter, mocker, fake_app_config):
    """Expand an extension for a complex snapcraft.yaml file.

    This includes parse-info, architectures, and advanced grammar.
    """
    # mock for advanced grammar parsing (i.e. `on amd64:`)
    mocker.patch(
        "craft_platforms.DebianArchitecture.from_host",
        return_value="amd64",
    )
    with Path("snapcraft.yaml").open("w") as yaml_file:
        print(
            dedent(
                """\
                name: test-name
                version: "0.1"
                summary: testing extensions
                description: expand a fake extension
                base: core22
                confinement: strict
                grade: stable
                architectures: [amd64, arm64, armhf]

                apps:
                  app1:
                    command: app1
                    command-chain: [fake-command]
                    extensions: [fake-extension]

                parts:
                  nil:
                    plugin: nil
                    parse-info:
                      - usr/share/metainfo/app1.appdata.xml
                    stage-packages:
                      - mesa-opencl-icd
                      - ocl-icd-libopencl1
                      - on amd64:
                        - intel-opencl-icd
            """
            ),
            file=yaml_file,
        )

    cmd = commands.ExpandExtensionsCommand(fake_app_config)
    cmd.run(Namespace())
    emitter.assert_message(
        dedent(
            f"""\
            name: test-name
            version: '0.1'
            summary: testing extensions
            description: expand a fake extension
            base: core22
            parts:
              nil:
                plugin: nil
                stage-packages:
                - mesa-opencl-icd
                - ocl-icd-libopencl1
                - intel-opencl-icd
                after:
                - fake-extension/fake-part
              fake-extension/fake-part:
                plugin: nil
            confinement: strict
            grade: stable
            architectures:
            - build-on:
              - {DebianArchitecture.from_host()}
              build-for:
              - {DebianArchitecture.from_host()}
            apps:
              app1:
                command: app1
                plugs:
                - fake-plug
                command-chain:
                - fake-command
        """
        )
    )


@pytest.mark.usefixtures("fake_extension")
def test_expand_extensions_complex(
    new_dir, emitter, mocker, valid_core_data, fake_app_config
):
    """Expand an extension for a complex snapcraft.yaml file.

    This includes parse-info, architectures, and advanced grammar.
    """
    # mock for advanced grammar parsing (i.e. `on amd64:`)
    mocker.patch(
        "craft_platforms.DebianArchitecture.from_host",
        return_value="amd64",
    )
    with Path("snapcraft.yaml").open("w") as yaml_file:
        print(
            dedent(
                f"""\
                name: test-name
                version: "0.1"
                summary: testing extensions
                description: expand a fake extension
                base: {valid_core_data.base}
                build-base: {valid_core_data.build_base}
                confinement: strict
                grade: {valid_core_data.grade}
                platforms: [amd64, arm64, armhf]

                apps:
                  app1:
                    command: app1
                    command-chain: [fake-command]
                    extensions: [fake-extension]

                parts:
                  nil:
                    plugin: nil
                    parse-info:
                      - usr/share/metainfo/app1.appdata.xml
                    stage-packages:
                      - mesa-opencl-icd
                      - ocl-icd-libopencl1
                      - on amd64:
                        - intel-opencl-icd
            """
            ),
            file=yaml_file,
        )

    cmd = commands.ExpandExtensionsCommand(fake_app_config)
    cmd.run(Namespace())
    emitter.assert_message(
        dedent(
            f"""\
            name: test-name
            version: '0.1'
            summary: testing extensions
            description: expand a fake extension
            base: {valid_core_data.base}
            build-base: {valid_core_data.build_base}
            parts:
              nil:
                plugin: nil
                stage-packages:
                - mesa-opencl-icd
                - ocl-icd-libopencl1
                - intel-opencl-icd
                after:
                - fake-extension/fake-part
              fake-extension/fake-part:
                plugin: nil
            confinement: strict
            grade: {valid_core_data.grade}
            apps:
              app1:
                command: app1
                plugs:
                - fake-plug
                command-chain:
                - fake-command
        """
        )
    )
