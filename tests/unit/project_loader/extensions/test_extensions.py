# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import inspect
import textwrap

from testscenarios import multiply_scenarios

from snapcraft.internal.project_loader import supported_extension_names

from .. import ProjectLoaderBaseTest


class ExtensionCombinationTest(ProjectLoaderBaseTest):
    scenarios = multiply_scenarios(
        *(
            [
                [
                    (
                        "{} extension".format(extension_name),
                        {"extension_{}".format(extension_name): extension_name},
                    ),
                    (
                        "no {} extension".format(extension_name),
                        {"extension_{}".format(extension_name): None},
                    ),
                ]
                for extension_name in supported_extension_names()
            ]
            or [[]]  # This guard can be removed once at least one extension is added
        )
    )

    def test_extensions_all_combinations_validate(self):
        # Determine extension list given scenarios
        extension_names = list()
        for member_pair in inspect.getmembers(self):
            name = member_pair[0]
            if name.startswith("extension_"):
                value = getattr(self, name)
                if value:
                    extension_names.append(value)

        # This shouldn't have any validation issues
        self.make_snapcraft_project(
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: test-command
                        extensions: {extensions}

                parts:
                    part1:
                        plugin: nil
                """
            ).format(extensions=extension_names)
        )
