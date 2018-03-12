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
import fixtures
import logging
from subprocess import CalledProcessError
from textwrap import dedent

from testtools.matchers import Contains, FileExists, Not

from snapcraft.internal.pluginhandler import _runner
from tests import unit


def _default_pre_build():
    open(os.path.join('builddir', 'default-pre-build'), 'w').close()


def _default_build():
    open(os.path.join('builddir', 'default-build'), 'w').close()


def _default_post_build():
    open(os.path.join('builddir', 'default-post-build'), 'w').close()


class RunnerTestCase(unit.TestCase):

    def test_pre_build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'pre-build': 'touch pre-build'},
                default_pre_build=_default_pre_build,
                builddir='builddir')

        runner.pre_build()

        self.assertThat(os.path.join('builddir', 'pre-build'), FileExists())
        self.assertThat(
                os.path.join('builddir', 'default-pre-build'),
                Not(FileExists()))

    def test_default_pre_build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={},
                default_pre_build=_default_pre_build,
                builddir='builddir')

        runner.pre_build()

        self.assertThat(
                os.path.join('builddir', 'default-pre-build'),
                FileExists())

    def test_build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'build': 'touch build'},
                default_build=_default_build,
                builddir='builddir')

        runner.build()

        self.assertThat(os.path.join('builddir', 'build'), FileExists())
        self.assertThat(
                os.path.join('builddir', 'default-build'),
                Not(FileExists()))

    def testdefault__build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={},
                default_build=_default_build,
                builddir='builddir')

        runner.build()

        self.assertThat(
                os.path.join('builddir', 'default-build'),
                FileExists())

    def test_post_build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={'post-build': 'touch post-build'},
                default_post_build=_default_post_build,
                builddir='builddir')

        runner.post_build()

        self.assertThat(os.path.join('builddir', 'post-build'), FileExists())
        self.assertThat(
                os.path.join('builddir', 'default-post-build'),
                Not(FileExists()))

    def test_default_post_build(self):
        os.mkdir('builddir')

        runner = _runner.Runner(
                part_properties={},
                default_post_build=_default_post_build,
                builddir='builddir')

        runner.post_build()

        self.assertThat(
                os.path.join('builddir', 'default-post-build'),
                FileExists())


class RunnerFailureTestCase(unit.TestCase):

    def test_failure_on_last_script_command_results_in_failure(self):
        os.mkdir('builddir')

        script = dedent("""\
            touch success
            false  # this should trigger an error
        """)

        runner = _runner.Runner(
                part_properties={'build': script},
                builddir='builddir')

        self.assertRaises(CalledProcessError, runner.build)

    def test_failure_to_execute_mid_script_results_in_failure(self):
        os.mkdir('builddir')

        script = dedent("""\
            false  # this should trigger an error
            touch success
        """)

        runner = _runner.Runner(
                part_properties={'build': script},
                builddir='builddir')

        self.assertRaises(CalledProcessError, runner.build)


class RunnerDeprecationTestCase(unit.TestCase):

    def test_prepare_deprecation(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        _runner.Runner(part_properties={'prepare': 'foo'}, builddir='bar')

        self.assertThat(self.fake_logger.output, Contains(
            "DEPRECATED: The 'prepare' keyword has been replaced by "
            "'pre-build'"))

    def test_install_deprecation(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        _runner.Runner(part_properties={'install': 'foo'}, builddir='bar')

        self.assertThat(self.fake_logger.output, Contains(
            "DEPRECATED: The 'install' keyword has been replaced by "
            "'post-build'"))
