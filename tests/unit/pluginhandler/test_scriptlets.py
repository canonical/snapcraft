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

import contextlib
import functools
import os
import subprocess

import testtools
from testtools.matchers import Equals, FileExists
from testscenarios.scenarios import multiply_scenarios
from unittest import mock

from snapcraft.internal import errors

from tests import unit


class ScriptletSetterTestCase(unit.TestCase):

    scenarios = [
        ("set-version", {"setter": "set-version", "getter": "get_version"}),
        ("set-grade", {"setter": "set-grade", "getter": "get_grade"}),
    ]

    def test_set_in_pull(self):
        handler = self.load_part(
            "test_part",
            part_properties={
                "override-pull": "snapcraftctl {} test-value".format(self.setter)
            },
        )

        handler.pull()
        metadata = handler.get_pull_state().scriptlet_metadata
        self.assertThat(getattr(metadata, self.getter)(), Equals("test-value"))

    def test_set_in_build(self):
        handler = self.load_part(
            "test_part",
            part_properties={
                "override-build": "snapcraftctl {} test-value".format(self.setter)
            },
        )

        handler.pull()
        handler.build()
        metadata = handler.get_build_state().scriptlet_metadata
        self.assertThat(getattr(metadata, self.getter)(), Equals("test-value"))
        self.assertFalse(handler.get_pull_state().scriptlet_metadata)

    def test_set_in_stage(self):
        handler = self.load_part(
            "test_part",
            part_properties={
                "override-stage": "snapcraftctl {} test-value".format(self.setter)
            },
        )

        handler.pull()
        handler.build()
        handler.stage()
        metadata = handler.get_stage_state().scriptlet_metadata
        self.assertThat(getattr(metadata, self.getter)(), Equals("test-value"))
        self.assertFalse(handler.get_pull_state().scriptlet_metadata)
        self.assertFalse(handler.get_build_state().scriptlet_metadata)

    def test_set_in_prime(self):
        handler = self.load_part(
            "test_part",
            part_properties={
                "override-prime": "snapcraftctl {} test-value".format(self.setter)
            },
        )

        handler.pull()
        handler.build()
        handler.stage()
        handler.prime()
        metadata = handler.get_prime_state().scriptlet_metadata
        self.assertThat(getattr(metadata, self.getter)(), Equals("test-value"))
        self.assertFalse(handler.get_pull_state().scriptlet_metadata)
        self.assertFalse(handler.get_build_state().scriptlet_metadata)
        self.assertFalse(handler.get_stage_state().scriptlet_metadata)


class ScriptletMultipleSettersErrorTestCase(unit.TestCase):

    scriptlet_scenarios = [
        ("override-pull", {"override_pull": "snapcraftctl {setter} 1"}),
        ("override-build", {"override_build": "snapcraftctl {setter} 2"}),
        ("override-stage", {"override_stage": "snapcraftctl {setter} 3"}),
        ("override-prime", {"override_prime": "snapcraftctl {setter} 4"}),
    ]

    multiple_setters_scenarios = multiply_scenarios(
        scriptlet_scenarios, scriptlet_scenarios
    )

    setter_scenarios = [
        ("set-version", {"setter": "set-version"}),
        ("set-grade", {"setter": "set-grade"}),
    ]

    scenarios = multiply_scenarios(setter_scenarios, multiple_setters_scenarios)

    def test_set_multiple_times(self):
        part_properties = {}
        with contextlib.suppress(AttributeError):
            part_properties["override-pull"] = self.override_pull.format(
                setter=self.setter
            )
        with contextlib.suppress(AttributeError):
            part_properties["override-build"] = self.override_build.format(
                setter=self.setter
            )
        with contextlib.suppress(AttributeError):
            part_properties["override-stage"] = self.override_stage.format(
                setter=self.setter
            )
        with contextlib.suppress(AttributeError):
            part_properties["override-prime"] = self.override_prime.format(
                setter=self.setter
            )

        # A few of these test cases result in only one of these scriptlets
        # being set. In that case, we actually want to double them up (i.e.
        # call set-version twice in the same scriptlet), which should still be
        # an error.
        if len(part_properties) == 1:
            for key, value in part_properties.items():
                part_properties[key] += "\n{}".format(value)

        handler = self.load_part("test_part", part_properties=part_properties)

        with testtools.ExpectedException(errors.ScriptletRunError):
            silent_popen = functools.partial(
                subprocess.Popen, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

            with mock.patch("subprocess.Popen", wraps=silent_popen):
                handler.pull()
                handler.build()
                handler.stage()
                handler.prime()


# These are deprecated
class OldScripletTestCase(unit.TestCase):
    def test_run_prepare_scriptlet(self):
        handler = self.load_part(
            "test-part", part_properties={"prepare": "touch prepare"}
        )

        handler.build()

        before_build_file_path = os.path.join(handler.plugin.build_basedir, "prepare")
        self.assertThat(before_build_file_path, FileExists())

    def test_run_install_scriptlet(self):
        handler = self.load_part(
            "test-part", part_properties={"install": "touch install"}
        )

        handler.build()

        after_build_file_path = os.path.join(handler.plugin.build_basedir, "install")
        self.assertThat(after_build_file_path, FileExists())
