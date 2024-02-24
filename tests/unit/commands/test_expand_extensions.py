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

from argparse import Namespace
from dataclasses import dataclass
from pathlib import Path
from textwrap import dedent

import pytest

from snapcraft import commands


@dataclass
class CoreData:
    """Dataclass containing base info for a given core."""

    base: str
    build_base: str
    grade: str
    command_class: type


VALID_CORE_DATA = {
    "core22": CoreData(
        "core22", "core22", "stable", commands.core22.ExpandExtensionsCommand
    ),
    "core24": CoreData("core24", "devel", "devel", commands.ExpandExtensions),
}


@pytest.fixture(params=VALID_CORE_DATA.keys())
def valid_core_data(request) -> CoreData:
    """Fixture that provides valid base, build-base and grade values for each
    coreXX base."""
    return VALID_CORE_DATA[request.param]


@pytest.mark.usefixtures("fake_extension")
def test_expand_extensions_simple(new_dir, emitter, valid_core_data):
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

    cmd = valid_core_data.command_class(None)
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
        confinement: strict
        grade: {valid_core_data.grade}
        apps:
            app1:
                command: app1
                command-chain:
                - fake-command
                plugs:
                - fake-plug
        parts:
            part1:
                plugin: nil
                after:
                - fake-extension/fake-part
            fake-extension/fake-part:
                plugin: nil
        """
        )
    )


@pytest.mark.usefixtures("fake_extension")
def test_expand_extensions_complex(new_dir, emitter, mocker, valid_core_data):
    """Expand an extension for a complex snapcraft.yaml file.

    This includes parse-info, architectures, and advanced grammar.
    """
    # mock for advanced grammar parsing (i.e. `on amd64:`)
    mocker.patch(
        "snapcraft.commands.core22.extensions.get_host_architecture",
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

    cmd = valid_core_data.command_class(None)
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
            confinement: strict
            grade: {valid_core_data.grade}
            apps:
                app1:
                    command: app1
                    command-chain:
                    - fake-command
                    plugs:
                    - fake-plug
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
        """
        )
    )
