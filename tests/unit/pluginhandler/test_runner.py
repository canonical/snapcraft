# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2020 Canonical Ltd
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
from textwrap import dedent
from unittest import mock

from testtools.matchers import Contains, FileContains, FileExists

from snapcraft.internal import errors
from snapcraft.internal.pluginhandler import _runner
from tests import fixture_setup, unit


def _fake_pull():
    open(os.path.join("sourcedir", "fake-pull"), "w").close()


def _fake_build():
    open(os.path.join("builddir", "fake-build"), "w").close()


def _fake_stage():
    open(os.path.join("stagedir", "fake-stage"), "w").close()


def _fake_prime():
    open(os.path.join("primedir", "fake-prime"), "w").close()


class RunnerTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.partdir = os.path.abspath("partdir")
        os.mkdir(self.partdir)

    def test_pull(self):
        os.mkdir("sourcedir")

        runner = _runner.Runner(
            part_properties={"override-pull": "touch pull"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.pull()

        self.assertThat(os.path.join("sourcedir", "pull"), FileExists())

    def test_builtin_function_from_pull(self):
        os.mkdir("sourcedir")

        runner = _runner.Runner(
            part_properties={"override-pull": "snapcraftctl pull"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"pull": _fake_pull},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.pull()

        self.assertThat(os.path.join("sourcedir", "fake-pull"), FileExists())

    def test_snapcraft_utils_in_path_if_snap(self):
        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"override-build": "echo $PATH > path"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.build()

        expected_path_segment = "/snap/snapcraft/current/bin/scriptlet-bin"

        self.assertThat(os.path.join("builddir", "path"), FileExists())
        self.assertThat(
            os.path.join("builddir", "path"),
            FileContains(matcher=Contains(expected_path_segment)),
        )

    def test_build(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"override-build": "touch build"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.build()

        self.assertThat(os.path.join("builddir", "build"), FileExists())

    def test_builtin_function_from_build(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"override-build": "snapcraftctl build"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"build": _fake_build},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.build()

        self.assertThat(os.path.join("builddir", "fake-build"), FileExists())

    def test_stage(self):
        os.mkdir("stagedir")

        runner = _runner.Runner(
            part_properties={"override-stage": "touch stage"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.stage()

        self.assertThat(os.path.join("stagedir", "stage"), FileExists())

    def test_builtin_function_from_stage(self):
        os.mkdir("stagedir")

        runner = _runner.Runner(
            part_properties={"override-stage": "snapcraftctl stage"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"stage": _fake_stage},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.stage()

        self.assertThat(os.path.join("stagedir", "fake-stage"), FileExists())

    def test_prime(self):
        os.mkdir("primedir")

        runner = _runner.Runner(
            part_properties={"override-prime": "touch prime"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.prime()

        self.assertThat(os.path.join("primedir", "prime"), FileExists())

    def test_builtin_function_from_prime(self):
        os.mkdir("primedir")

        runner = _runner.Runner(
            part_properties={"override-prime": "snapcraftctl prime"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"prime": _fake_prime},
            env_generator=lambda step: "export FOO=BAR",
        )

        runner.prime()

        self.assertThat(os.path.join("primedir", "fake-prime"), FileExists())


class RunnerFailureTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.partdir = os.path.abspath("partdir")
        os.mkdir(self.partdir)

    def test_failure_on_last_script_command_results_in_failure(self):
        os.mkdir("builddir")

        script = dedent(
            """\
            touch success
            false  # this should trigger an error
        """
        )

        runner = _runner.Runner(
            part_properties={"override-build": script},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        self.assertRaises(errors.ScriptletRunError, runner.build)

    def test_failure_to_execute_mid_script_results_in_failure(self):
        os.mkdir("builddir")

        script = dedent(
            """\
            false  # this should trigger an error
            touch success
        """
        )

        runner = _runner.Runner(
            part_properties={"override-build": script},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        self.assertRaises(errors.ScriptletRunError, runner.build)

    def test_snapcraftctl_no_alias_if_not_snap(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"override-build": "alias snapcraftctl 2> /dev/null"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
            env_generator=lambda step: "export FOO=BAR",
        )

        self.assertRaises(errors.ScriptletRunError, runner.build)

    def test_snapcraftctl_errors_on_exception(self):
        os.mkdir("primedir")

        class _TestException(errors.ScriptletBaseError):
            fmt = "I'm an error"

        def _raise():
            raise _TestException()

        runner = _runner.Runner(
            part_properties={"override-prime": "snapcraftctl prime"},
            partdir=self.partdir,
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"prime": _raise},
            env_generator=lambda step: "export FOO=BAR",
        )

        silent_popen = functools.partial(
            subprocess.Popen, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

        with mock.patch("subprocess.Popen", wraps=silent_popen):
            self.assertRaises(_TestException, runner.prime)
