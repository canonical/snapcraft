# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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

import os
import shutil
from textwrap import dedent

from testtools.matchers import Contains, Equals, DirExists, FileExists, Not

import snapcraft
from snapcraft.internal import errors, steps
from snapcraft.project import errors as project_errors
from . import CommandBaseTestCase


class CleanCommandBaseTestCase(CommandBaseTestCase):

    yaml_template = dedent(
        """\
        name: clean-test
        base: core18
        version: "1.0"
        summary: test clean
        description: if the clean is successful the state file will be updated
        confinement: strict
        grade: stable

        parts:
        {parts}"""
    )

    yaml_part = """  clean{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1, create=True):
        parts = "\n".join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))

        parts = []
        for i in range(n):
            part_name = "clean{}".format(i)

            properties = {"plugin": "nil"}
            project_options = snapcraft.ProjectOptions()

            handler = self.load_part(
                part_name=part_name,
                plugin_name="nil",
                part_properties=properties,
                project_options=project_options,
            )

            parts.append({"part_dir": handler.plugin.partdir})

            if create:
                handler.makedirs()
                open(os.path.join(handler.plugin.installdir, part_name), "w").close()

                handler.mark_done(steps.PULL)
                handler.mark_done(steps.BUILD)

                handler.stage()
                handler.prime()

        return parts


class CleanCommandTestCase(CleanCommandBaseTestCase):
    def test_part_to_remove_not_defined_exits_with_error(self):
        self.make_snapcraft_yaml(n=3)

        raised = self.assertRaises(
            errors.SnapcraftEnvironmentError, self.run_command, ["clean", "no-clean"]
        )

        self.assertThat(
            str(raised),
            Equals("The part named 'no-clean' is not defined in 'snap/snapcraft.yaml'"),
        )

    def test_clean_all(self):
        self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, Not(DirExists()))
        self.assertThat(self.stage_dir, Not(DirExists()))
        self.assertThat(self.prime_dir, Not(DirExists()))

    def test_unprime_fails(self):
        self.make_snapcraft_yaml()

        result = self.run_command(["clean", "--unprime"])

        self.assertThat(result.exit_code, Equals(2))

    def test_destructive_mode(self):
        self.make_snapcraft_yaml()

        result = self.run_command(["clean", "--destructive-mode"])

        self.assertThat(result.exit_code, Equals(0))


class CleanCommandPartsTestCase(CleanCommandBaseTestCase):
    def test_local_plugin_not_removed(self):
        self.make_snapcraft_yaml(n=3)

        local_plugin = os.path.join(self.local_plugins_dir, "foo.py")
        os.makedirs(os.path.dirname(local_plugin))
        open(local_plugin, "w").close()

        result = self.run_command(["clean"])

        self.assertThat(result.exit_code, Equals(0))

        self.assertThat(self.parts_dir, Not(DirExists()))
        self.assertThat(self.stage_dir, Not(DirExists()))
        self.assertThat(self.prime_dir, Not(DirExists()))
        self.assertThat(local_plugin, FileExists())

    def test_clean_all_when_all_parts_specified(self):
        self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean", "clean0", "clean1", "clean2"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, Not(DirExists()))
        self.assertThat(self.stage_dir, Not(DirExists()))
        self.assertThat(self.prime_dir, Not(DirExists()))

    def test_partial_clean(self):
        parts = self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean", "clean0", "clean2"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(self.stage_dir, DirExists())
        self.assertThat(self.prime_dir, DirExists())

        for i in [0, 2]:
            self.assertThat(parts[i]["part_dir"], Not(DirExists()))

        # Now clean it the rest of the way
        result = self.run_command(["clean", "clean1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, Not(DirExists()))
        self.assertThat(self.stage_dir, Not(DirExists()))
        self.assertThat(self.prime_dir, Not(DirExists()))

    def test_everything_is_clean(self):
        """Don't crash if everything is already clean."""
        self.make_snapcraft_yaml(n=3, create=False)

        result = self.run_command(["clean"])

        self.assertThat(result.exit_code, Equals(0))


class CleanCommandReverseDependenciesTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            dedent(
                """\
            name: clean-test
            base: core18
            version: "1.0"
            summary: test clean
            description: test clean
            confinement: strict
            grade: stable

            parts:
                main:
                    plugin: nil

                dependent:
                    plugin: nil
                    after: [main]

                nested-dependent:
                    plugin: nil
                    after: [dependent]
        """
            )
        )

        self.part_dirs = {}
        for part in ["main", "dependent", "nested-dependent"]:
            self.part_dirs[part] = os.path.join(self.parts_dir, part)

        result = self.run_command(["pull"])
        self.assertThat(result.exit_code, Equals(0))

    def assert_clean(self, parts):
        for part in parts:
            self.assertThat(
                os.path.join(self.part_dirs[part], "state"),
                Not(DirExists()),
                "{!r} is not clean!".format(part),
            )

    def assert_not_clean(self, parts):
        for part in parts:
            self.assertThat(
                os.path.join(self.part_dirs[part], "state"),
                DirExists(),
                "{!r} is clean!".format(part),
            )

    def test_clean_dependent_parts(self):
        result = self.run_command(["clean", "dependent", "nested-dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assert_clean(["dependent", "nested-dependent"])
        self.assert_not_clean(["main"])

    def test_clean_part_with_clean_dependent(self):
        result = self.run_command(["clean", "nested-dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assert_clean(["nested-dependent"])

        # Not specifying nested-dependent here should be okay since it's
        # already clean.
        result = self.run_command(["clean", "dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assert_clean(["dependent", "nested-dependent"])

    def test_clean_part_unspecified_uncleaned_dependent_notifies(self):
        # Not specifying nested-dependent here should result in clean notifying
        # that its dependents are now out-of-date
        result = self.run_command(["clean", "dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                "Cleaned 'dependent', which makes the following part out of date: "
                "'nested-dependent'"
            ),
        )
        self.assert_clean(["dependent"])
        self.assert_not_clean(["nested-dependent"])

    def test_clean_nested_dependent_parts(self):
        result = self.run_command(["clean", "main", "dependent", "nested-dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assert_clean(["main", "dependent", "nested-dependent"])

    def test_clean_part_with_clean_dependent_uncleaned_nested_dependent(self):
        shutil.rmtree(self.part_dirs["dependent"])
        self.assert_clean(["dependent"])

        # Not specifying dependent here should be okay since it's already
        # clean.
        result = self.run_command(["clean", "main", "nested-dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assert_clean(["main", "dependent", "nested-dependent"])

    def test_clean_part_with_clean_nested_dependent(self):
        shutil.rmtree(self.part_dirs["nested-dependent"])
        self.assert_clean(["nested-dependent"])

        # Not specifying nested-dependent here should be okay since it's
        # already clean.
        result = self.run_command(["clean", "main", "dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assert_clean(["main", "dependent", "nested-dependent"])

    def test_clean_part_unspecified_uncleaned_dependent_nested_notifies(self):
        # Not specifying dependent here should result in clean notifying that
        # its dependents are now dirty. It should NOT clean them, though.
        result = self.run_command(["clean", "main"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                "Cleaned 'main', which makes the following parts out of date: "
                "'dependent' and 'nested-dependent'"
            ),
        )
        self.assert_clean(["main"])
        self.assert_not_clean(["dependent", "nested-dependent"])

    def test_clean_part_unspecified_uncleaned_nested_dependent_notifies(self):
        # Not specifying the nested-dependent here should result in clean
        # notifying that it's now dirty. It should NOT clean it, though.
        result = self.run_command(["clean", "main", "dependent"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                "Cleaned 'dependent', which makes the following part out of date: "
                "'nested-dependent'"
            ),
        )
        self.assert_clean(["main", "dependent"])
        self.assert_not_clean(["nested-dependent"])


class CleanCommandSnapcraftYamlTest(CommandBaseTestCase):
    def test_clean_with_only_name_and_base_defined(self):
        self.make_snapcraft_yaml(
            dedent(
                """\
            name: clean-test
            base: core
        """
            )
        )

        result = self.run_command(["clean"])
        self.assertThat(result.exit_code, Equals(0))

    def test_clean_with_missing_required_name_defined(self):
        self.make_snapcraft_yaml(
            dedent(
                """\
            base: core16
        """
            )
        )
        self.assertRaises(
            project_errors.YamlValidationError, self.run_command, ["clean"]
        )

    def test_clean_with_no_snapcraft_yaml(self):
        self.assertRaises(
            project_errors.MissingSnapcraftYamlError, self.run_command, ["clean"]
        )
