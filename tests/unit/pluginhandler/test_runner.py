# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import logging
import os
import subprocess
from textwrap import dedent

import fixtures
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
    def test_pull(self):
        os.mkdir("sourcedir")

        runner = _runner.Runner(
            part_properties={"override-pull": "touch pull"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        runner.pull()

        self.assertThat(os.path.join("sourcedir", "pull"), FileExists())

    def test_builtin_function_from_pull(self):
        os.mkdir("sourcedir")

        runner = _runner.Runner(
            part_properties={"override-pull": "snapcraftctl pull"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"pull": _fake_pull},
        )

        runner.pull()

        self.assertThat(os.path.join("sourcedir", "fake-pull"), FileExists())

    def test_prepare(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"prepare": "touch prepare"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        runner.prepare()

        self.assertThat(os.path.join("builddir", "prepare"), FileExists())

    def test_builtin_function_from_prepare(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"prepare": "snapcraftctl build"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"build": _fake_build},
        )

        runner.prepare()

        self.assertThat(os.path.join("builddir", "fake-build"), FileExists())

    def test_snapcraftctl_alias_if_snap(self):
        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"prepare": "alias snapcraftctl > definition"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        with mock.patch("os.path.exists", return_value=True):
            runner.prepare()

        expected_snapcrafctl = "/snap/snapcraft/current/bin/snapcraftctl"

        self.assertThat(os.path.join("builddir", "definition"), FileExists())
        self.assertThat(
            os.path.join("builddir", "definition"),
            FileContains("snapcraftctl={!r}\n".format(expected_snapcrafctl)),
        )

    def test_old_build(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"build": "touch build"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        runner.build()

        self.assertThat(os.path.join("builddir", "build"), FileExists())

    def test_build(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"override-build": "touch build"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        runner.build()

        self.assertThat(os.path.join("builddir", "build"), FileExists())

    def test_builtin_function_from_build(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"override-build": "snapcraftctl build"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"build": _fake_build},
        )

        runner.build()

        self.assertThat(os.path.join("builddir", "fake-build"), FileExists())

    def test_install(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"install": "touch install"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        runner.install()

        self.assertThat(os.path.join("builddir", "install"), FileExists())

    def test_builtin_function_from_install(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"install": "snapcraftctl build"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"build": _fake_build},
        )

        runner.install()

        self.assertThat(os.path.join("builddir", "fake-build"), FileExists())

    def test_stage(self):
        os.mkdir("stagedir")

        runner = _runner.Runner(
            part_properties={"override-stage": "touch stage"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        runner.stage()

        self.assertThat(os.path.join("stagedir", "stage"), FileExists())

    def test_builtin_function_from_stage(self):
        os.mkdir("stagedir")

        runner = _runner.Runner(
            part_properties={"override-stage": "snapcraftctl stage"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"stage": _fake_stage},
        )

        runner.stage()

        self.assertThat(os.path.join("stagedir", "fake-stage"), FileExists())

    def test_prime(self):
        os.mkdir("primedir")

        runner = _runner.Runner(
            part_properties={"override-prime": "touch prime"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        runner.prime()

        self.assertThat(os.path.join("primedir", "prime"), FileExists())

    def test_builtin_function_from_prime(self):
        os.mkdir("primedir")

        runner = _runner.Runner(
            part_properties={"override-prime": "snapcraftctl prime"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"prime": _fake_prime},
        )

        runner.prime()

        self.assertThat(os.path.join("primedir", "fake-prime"), FileExists())


class RunnerFailureTestCase(unit.TestCase):
    def test_failure_on_last_script_command_results_in_failure(self):
        os.mkdir("builddir")

        script = dedent(
            """\
            touch success
            false  # this should trigger an error
        """
        )

        runner = _runner.Runner(
            part_properties={"build": script},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
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
            part_properties={"build": script},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        self.assertRaises(errors.ScriptletRunError, runner.build)

    def test_snapcraftctl_no_alias_if_not_snap(self):
        os.mkdir("builddir")

        runner = _runner.Runner(
            part_properties={"build": "alias snapcraftctl 2> /dev/null"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
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
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={"prime": _raise},
        )

        silent_popen = functools.partial(
            subprocess.Popen, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

        with mock.patch("subprocess.Popen", wraps=silent_popen):
            self.assertRaises(errors.ScriptletRunError, runner.prime)


class RunnerDeprecationTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        os.mkdir("builddir")

    def test_prepare_deprecation(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        _runner.Runner(
            part_properties={"prepare": "foo"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        self.assertThat(
            self.fake_logger.output,
            Contains(
                "DEPRECATED: The 'prepare' keyword has been replaced by "
                "'override-build'"
            ),
        )

    def test_build_deprecation(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        _runner.Runner(
            part_properties={"build": "foo"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        self.assertThat(
            self.fake_logger.output,
            Contains(
                "DEPRECATED: The 'build' keyword has been replaced by "
                "'override-build'"
            ),
        )

    def test_install_deprecation(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        _runner.Runner(
            part_properties={"install": "foo"},
            sourcedir="sourcedir",
            builddir="builddir",
            stagedir="stagedir",
            primedir="primedir",
            builtin_functions={},
        )

        self.assertThat(
            self.fake_logger.output,
            Contains(
                "DEPRECATED: The 'install' keyword has been replaced by "
                "'override-build'"
            ),
        )
