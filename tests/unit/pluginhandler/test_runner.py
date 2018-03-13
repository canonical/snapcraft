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

import os
from textwrap import dedent

from testtools.matchers import FileExists

from snapcraft.internal import errors
from snapcraft.internal.pluginhandler import _runner
from tests import unit


def _fake_build():
    open(os.path.join('builddir', 'fake-build'), 'w').close()


class RunnerTestCase(unit.TestCase):

    def test_prepare(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'prepare': 'touch prepare'},
                builddir='builddir',
                builtin_functions={})

        runner.prepare()

        self.assertThat(os.path.join('builddir', 'prepare'), FileExists())

    def test_builtin_function_from_prepare(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'prepare': 'snapcraftctl build'},
                builddir='builddir',
                builtin_functions={'build': _fake_build})

        runner.prepare()

        self.assertThat(os.path.join('builddir', 'fake-build'), FileExists())

    def test_build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'build': 'touch build'},
                builddir='builddir',
                builtin_functions={})

        runner.build()

        self.assertThat(os.path.join('builddir', 'build'), FileExists())

    def test_builtin_function_from_build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'build': 'snapcraftctl build'},
                builddir='builddir',
                builtin_functions={'build': _fake_build})

        runner.build()

        self.assertThat(os.path.join('builddir', 'fake-build'), FileExists())

    def test_install(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'install': 'touch install'},
                builddir='builddir',
                builtin_functions={})

        runner.install()

        self.assertThat(os.path.join('builddir', 'install'), FileExists())

    def test_builtin_function_from_install(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'install': 'snapcraftctl build'},
                builddir='builddir',
                builtin_functions={'build': _fake_build})

        runner.install()

        self.assertThat(os.path.join('builddir', 'fake-build'), FileExists())


class RunnerFailureTestCase(unit.TestCase):

    def test_failure_on_last_script_command_results_in_failure(self):
        os.mkdir('builddir')

        script = dedent("""\
            touch success
            false  # this should trigger an error
        """)

        runner = _runner.Runner(
                part_properties={'build': script},
                builddir='builddir',
                builtin_functions={})

        self.assertRaises(errors.ScriptletRunError, runner.build)

    def test_failure_to_execute_mid_script_results_in_failure(self):
        os.mkdir('builddir')

        script = dedent("""\
            false  # this should trigger an error
            touch success
        """)

        runner = _runner.Runner(
                part_properties={'build': script},
                builddir='builddir',
                builtin_functions={})

        self.assertRaises(errors.ScriptletRunError, runner.build)
