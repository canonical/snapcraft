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
from pathlib import Path
from textwrap import dedent

import pytest

from snapcraft.commands import ExpandExtensionsCommand


@pytest.mark.usefixtures("fake_extension")
def test_command(new_dir, emitter):
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

    cmd = ExpandExtensionsCommand(None)
    cmd.run(Namespace())
    emitter.assert_message(
        dedent(
            """\
        name: test-name
        version: '0.1'
        summary: testing extensions
        description: expand a fake extension
        base: core22
        confinement: strict
        grade: stable
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
