# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import fixtures
from unittest.mock import call, patch, ANY

from testtools.matchers import Contains, Equals, DirExists, FileExists, Not

import snapcraft
from snapcraft.internal import errors, steps
from tests import fixture_setup
from . import CommandBaseTestCase


class CleanCommandBaseTestCase(CommandBaseTestCase):

    yaml_template = """name: clean-test
version: 1.0
summary: test clean
description: if the clean is successful the state file will be updated
icon: icon.png
confinement: strict
grade: stable

parts:
{parts}"""

    yaml_part = """  clean{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1, create=True):
        parts = "\n".join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))
        open("icon.png", "w").close()

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


class ContainerizedCleanCommandTestCase(CleanCommandBaseTestCase):

    scenarios = [("local", dict(snapcraft_container_builds="1", remote="local"))]

    def test_clean_containerized_noop(self):
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_CONTAINER_BUILDS", self.snapcraft_container_builds
            )
        )
        self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean"])

        self.assertThat(result.exit_code, Equals(0))
        # clean should be a noop if no container exists yet/ anymore
        fake_lxd.check_call_mock.assert_not_called()

    @patch("snapcraft.internal.lifecycle.clean")
    def test_clean_containerized_exists_stopped(self, mock_lifecycle_clean):
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        # Container was created before, and isn't running
        fake_lxd.name = "{}:snapcraft-clean-test".format(self.remote)
        fake_lxd.status = "Stopped"
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_CONTAINER_BUILDS", self.snapcraft_container_builds
            )
        )
        self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean"])

        self.assertThat(result.exit_code, Equals(0))
        # clean with no parts should delete the container
        fake_lxd.check_call_mock.assert_has_calls(
            [call(["lxc", "delete", "-f", fake_lxd.name])]
        )
        # no other commands should be run in the container
        self.assertThat(fake_lxd.check_call_mock.call_count, Equals(1))
        # clean should be called normally, outside of the container
        mock_lifecycle_clean.assert_has_calls([call(ANY, (), steps.PULL)])

    @patch("snapcraft.internal.lifecycle.clean")
    def test_clean_containerized_pull_retains_container(self, mock_lifecycle_clean):
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        # Container was created before, and isn't running
        fake_lxd.name = "{}:snapcraft-clean-test".format(self.remote)
        fake_lxd.status = "Stopped"
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_CONTAINER_BUILDS", self.snapcraft_container_builds
            )
        )
        self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean", "-s", "pull"])

        self.assertThat(result.exit_code, Equals(0))
        # clean pull should NOT delete the container
        fake_lxd.check_call_mock.assert_not_called()
        # clean should be called normally, outside of the container
        mock_lifecycle_clean.assert_has_calls([call(ANY, (), steps.PULL)])

    def test_clean_containerized_with_part(self):
        fake_lxd = fixture_setup.FakeLXD()
        fake_lxd.name = "local:snapcraft-clean-test"
        fake_lxd.status = "Stopped"
        self.useFixture(fake_lxd)
        # Container should not be initialized at all
        fake_lxd.check_output_mock.side_effect = FileNotFoundError("lxc")
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_CONTAINER_BUILDS", self.snapcraft_container_builds
            )
        )
        self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean", "clean1"])

        self.assertThat(result.exit_code, Equals(0))
        # clean with parts should NOT delete the container
        fake_lxd.check_call_mock.assert_not_called()


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

    def test_cleaning_with_strip_does_prime_and_warns(self):
        self.make_snapcraft_yaml(n=3)

        result = self.run_command(["clean", "--step=strip"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains("DEPRECATED: Use `prime` instead of `strip` as the step to clean"),
        )
        self.assertThat(self.prime_dir, Not(DirExists()))


class CleanCommandReverseDependenciesTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            """name: clean-test
version: 1.0
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
    after: [dependent]"""
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
