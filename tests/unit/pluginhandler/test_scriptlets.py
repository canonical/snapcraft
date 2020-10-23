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

import functools
import os
import subprocess
import textwrap
from unittest import mock

import fixtures
import testtools
from testscenarios.scenarios import multiply_scenarios
from testtools.matchers import Equals

from snapcraft import yaml_utils
from snapcraft.internal import errors
from tests import unit
from tests.unit.commands import CommandBaseTestCase


class ScriptletCommandsTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: my-snap-name
                base: core18
                summary: summary
                description: description

                adopt-info: my-part
                confinement: devmode

                parts:
                    my-part:
                        plugin: dump
                        source: src
                        override-pull: |
                            snapcraftctl pull
                            snapcraftctl set-grade devel
                            version="$(cat version.txt)"
                            snapcraftctl set-version "$version"
                """
            )
        )

        os.mkdir("src")
        open(os.path.join("src", "version.txt"), "w").write("v1.0")

        fake_install_build_packages = fixtures.MockPatch(
            "snapcraft.internal.lifecycle._runner._install_build_packages",
            return_value=list(),
        )
        self.useFixture(fake_install_build_packages)

        fake_install_build_snaps = fixtures.MockPatch(
            "snapcraft.internal.lifecycle._runner._install_build_snaps",
            return_value=list(),
        )
        self.useFixture(fake_install_build_snaps)

    def test_scriptlet_after_repull(self):
        self.run_command(["prime"])

        with open(os.path.join("prime", "meta", "snap.yaml")) as f:
            y = yaml_utils.load(f)

        self.assertThat(y["grade"], Equals("devel"))
        self.assertThat(y["version"], Equals("v1.0"))

        # modifying source file (src/version.txt) will trigger re-pull
        open(os.path.join("src", "version.txt"), "w").write("v2.0")
        self.run_command(["prime"])

        with open(os.path.join("prime", "meta", "snap.yaml")) as f:
            z = yaml_utils.load(f)

        self.assertThat(z["grade"], Equals("devel"))
        self.assertThat(z["version"], Equals("v2.0"))


class TestScriptletSetter:

    scenarios = [
        (
            "set-version",
            {"setter": "set-version", "getter": "get_version", "value": "v1"},
        ),
        ("set-grade", {"setter": "set-grade", "getter": "get_grade", "value": "devel"}),
    ]

    def test_set_in_pull(self, tmp_work_path, setter, getter, value):
        handler = unit.load_part(
            "test_part",
            part_properties={
                "override-pull": "snapcraftctl {} {}".format(setter, value)
            },
        )

        handler.pull()

        pull_metadata = handler.get_pull_state().scriptlet_metadata

        assert getattr(pull_metadata, getter)() == value

    def test_set_in_build(self, tmp_work_path, setter, getter, value):
        handler = unit.load_part(
            "test_part",
            part_properties={
                "override-build": "snapcraftctl {} {}".format(setter, value)
            },
        )

        handler.pull()
        handler.build()

        pull_metadata = handler.get_pull_state().scriptlet_metadata
        build_metadata = handler.get_build_state().scriptlet_metadata

        assert getattr(pull_metadata, getter)() is None
        assert getattr(build_metadata, getter)() == value

    def test_set_in_stage(self, tmp_work_path, setter, getter, value):
        handler = unit.load_part(
            "test_part",
            part_properties={
                "override-stage": "snapcraftctl {} {}".format(setter, value)
            },
        )

        handler.pull()
        handler.build()
        handler.stage()

        pull_metadata = handler.get_pull_state().scriptlet_metadata
        build_metadata = handler.get_build_state().scriptlet_metadata
        stage_metadata = handler.get_stage_state().scriptlet_metadata

        assert getattr(pull_metadata, getter)() is None
        assert getattr(build_metadata, getter)() is None
        assert getattr(stage_metadata, getter)() == value

    def test_set_in_prime(self, tmp_work_path, setter, getter, value):
        handler = unit.load_part(
            "test_part",
            part_properties={
                "override-prime": "snapcraftctl {} {}".format(setter, value)
            },
        )

        handler.pull()
        handler.build()
        handler.stage()
        handler.prime()

        pull_metadata = handler.get_pull_state().scriptlet_metadata
        build_metadata = handler.get_build_state().scriptlet_metadata
        stage_metadata = handler.get_stage_state().scriptlet_metadata
        prime_metadata = handler.get_prime_state().scriptlet_metadata

        assert getattr(pull_metadata, getter)() is None
        assert getattr(build_metadata, getter)() is None
        assert getattr(stage_metadata, getter)() is None
        assert getattr(prime_metadata, getter)() == value


class TestScriptletMultipleSettersError:

    scriptlet_scenarios = [
        (
            "override-pull/build",
            {
                "override_pull": "snapcraftctl {setter} {value}",
                "override_build": "snapcraftctl {setter} {value}",
                "override_stage": None,
                "override_prime": None,
            },
        ),
        (
            "override-pull/stage",
            {
                "override_pull": "snapcraftctl {setter} {value}",
                "override_build": None,
                "override_stage": "snapcraftctl {setter} {value}",
                "override_prime": None,
            },
        ),
        (
            "override-pull/prime",
            {
                "override_pull": "snapcraftctl {setter} {value}",
                "override_build": None,
                "override_stage": None,
                "override_prime": "snapcraftctl {setter} {value}",
            },
        ),
        (
            "override-build/stage",
            {
                "override_pull": None,
                "override_build": "snapcraftctl {setter} {value}",
                "override_stage": "snapcraftctl {setter} {value}",
                "override_prime": None,
            },
        ),
        (
            "override-build/prime",
            {
                "override_pull": None,
                "override_build": "snapcraftctl {setter} {value}",
                "override_stage": None,
                "override_prime": "snapcraftctl {setter} {value}",
            },
        ),
        (
            "override-stage/prime",
            {
                "override_pull": None,
                "override_build": None,
                "override_stage": "snapcraftctl {setter} {value}",
                "override_prime": "snapcraftctl {setter} {value}",
            },
        ),
    ]

    setter_scenarios = [
        ("set-version", {"setter": "set-version", "value": "v1"}),
        ("set-grade", {"setter": "set-grade", "value": "stable"}),
    ]

    scenarios = multiply_scenarios(setter_scenarios, scriptlet_scenarios)

    def test_set_multiple_times(
        self,
        tmp_work_path,
        setter,
        value,
        override_pull,
        override_build,
        override_stage,
        override_prime,
    ):
        part_properties = {}
        if override_pull is not None:
            part_properties["override-pull"] = override_pull.format(
                setter=setter, value=value
            )
        if override_build is not None:
            part_properties["override-build"] = override_build.format(
                setter=setter, value=value
            )
        if override_stage is not None:
            part_properties["override-stage"] = override_stage.format(
                setter=setter, value=value
            )
        if override_prime is not None:
            part_properties["override-prime"] = override_prime.format(
                setter=setter, value=value
            )

        # A few of these test cases result in only one of these scriptlets
        # being set. In that case, we actually want to double them up (i.e.
        # call set-version twice in the same scriptlet), which should still be
        # an error.
        if len(part_properties) == 1:
            for key, value in part_properties.items():
                part_properties[key] += "\n{}".format(value)

        handler = unit.load_part("test_part", part_properties=part_properties)

        with testtools.ExpectedException(errors.ScriptletDuplicateFieldError):
            silent_popen = functools.partial(
                subprocess.Popen, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

            with mock.patch("subprocess.Popen", wraps=silent_popen):
                handler.pull()
                handler.build()
                handler.stage()
                handler.prime()
