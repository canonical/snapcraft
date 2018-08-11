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

import json
import os
import re
import textwrap

from testtools.matchers import Contains, Equals, MatchesRegex
import snapcraft.internal.errors

from . import CommandBaseTestCase


class InspectInvalidOptionsCommandTest(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: my-snap-name
                version: '0.1'
                summary: summary
                description: description

                grade: devel
                confinement: devmode

                parts:
                    part1:
                        plugin: nil
                """
            )
        )

    def test_inspect_invalid_options(self):
        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command,
            ["inspect", "--latest-step", "--provides", "foo"],
        )

        self.assertThat(
            str(raised),
            Equals("Only one of --provides or --latest-step may be supplied"),
        )


class InspectProvidesCommandTest(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: my-snap-name
                version: '0.1'
                summary: summary
                description: description

                grade: devel
                confinement: devmode

                parts:
                    part1:
                        plugin: dump
                        override-pull: |
                            mkdir dir
                            touch dir/file1

                    part2:
                        plugin: dump
                        override-pull: |
                            mkdir dir
                            touch dir/file2

                    part3:
                        plugin: nil
                        override-stage: |
                            touch file3
                """
            )
        )

        self.run_command(["prime"])

    def test_inspect_provides(self):
        for part_number in ("1", "2"):
            part = "part{}".format(part_number)
            file_name = "file{}".format(part_number)
            result = self.run_command(
                ["inspect", "--provides", os.path.join("stage", "dir", file_name)]
            )
            self.expectThat(
                result.output,
                Equals(
                    "This path was provided by the following part:\n{}\n".format(part)
                ),
            )

            result = self.run_command(
                [
                    "inspect",
                    "--provides",
                    os.path.join(self.prime_dir, "dir", file_name),
                ]
            )
            self.expectThat(
                result.output,
                Equals(
                    "This path was provided by the following part:\n{}\n".format(part)
                ),
            )

        result = self.run_command(
            ["inspect", "--provides", os.path.join("prime", "dir")]
        )

        self.expectThat(
            result.output,
            MatchesRegex(".*provided by the following parts.*part1.*part2", re.DOTALL),
        )

    def test_inspect_provides_json(self):
        for part_number in ("1", "2"):
            part = "part{}".format(part_number)
            file_name = "file{}".format(part_number)
            file_path = os.path.join("stage", "dir", file_name)
            result = self.run_command(["inspect", "--json", "--provides", file_path])
            self.expectThat(
                json.loads(result.output), Equals({"path": file_path, "parts": [part]})
            )


class InspectLatestStepTest(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: my-snap-name
                version: '0.1'
                summary: summary
                description: description

                grade: devel
                confinement: devmode

                parts:
                    part1:
                        plugin: nil
                    part2:
                        plugin: nil
                """
            )
        )

    def test_inspect_latest_step(self):
        self.run_command(["pull"])
        self.run_command(["build", "part1"])

        result = self.run_command(["inspect", "--latest-step"])
        self.expectThat(
            result.output,
            Contains(
                "The latest step that was run is the 'build' step of the 'part1' part"
            ),
        )
        part1_build_dir = os.path.abspath(os.path.join("parts", "part1", "build"))
        self.expectThat(
            result.output,
            Contains(
                "The working directory for this step is {!r}".format(part1_build_dir)
            ),
        )

    def test_inspect_latest_step_json(self):
        self.run_command(["pull"])
        self.run_command(["build", "part1"])

        result = json.loads(
            self.run_command(["inspect", "--json", "--latest-step"]).output
        )
        part1_build_dir = os.path.abspath(os.path.join("parts", "part1", "build"))
        self.expectThat(result["part"], Equals("part1"))
        self.expectThat(result["step"], Equals("build"))
        self.expectThat(result["directory"], Equals(part1_build_dir))


class InspectLifecycleStatusTest(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: my-snap-name
                version: '0.1'
                summary: summary
                description: description

                grade: devel
                confinement: devmode

                parts:
                    part1:
                        plugin: nil
                        source: src1/
                    part2:
                        plugin: nil
                        source: src2/
                        after: [part1]
                """
            )
        )

        os.mkdir("src1")
        os.mkdir("src2")

    def test_inspect_lifecycle_status(self):
        self.run_command(["pull"])

        # Not even going to bother testing the table output: it's too hard to keep in
        # sync. JSON should be the same data.
        result = json.loads(self.run_command(["inspect", "--json"]).output)
        self.expectThat(
            result,
            Equals(
                {
                    "part1": {
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": None,
                    },
                    "part2": {
                        "pull": "complete",
                        "build": None,
                        "stage": None,
                        "prime": None,
                    },
                }
            ),
        )

        self.run_command(["prime"])
        result = json.loads(self.run_command(["inspect", "--json"]).output)
        self.expectThat(
            result,
            Equals(
                {
                    "part1": {
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                    "part2": {
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                }
            ),
        )

        open(os.path.join("src2", "file"), "w").close()
        result = json.loads(self.run_command(["inspect", "--json"]).output)
        self.expectThat(
            result,
            Equals(
                {
                    "part1": {
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                    "part2": {
                        "pull": "outdated (source changed)",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                }
            ),
        )

        self.run_command(["clean", "part1", "--step=prime"])
        result = json.loads(self.run_command(["inspect", "--json"]).output)
        self.expectThat(
            result,
            Equals(
                {
                    "part1": {
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": None,
                    },
                    "part2": {
                        "pull": "outdated (source changed)",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "dirty ('part1' changed)",
                    },
                }
            ),
        )
