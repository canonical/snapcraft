# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017 Canonical Ltd
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

from testtools.matchers import Contains, DirExists, Equals

from . import CommandBaseTestCase


class TourCommandTestCase(CommandBaseTestCase):

    def test_scaffold(self):
        result = self.run_command(['tour'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(dedent("""\
            Copying examples tour to ./snapcraft-tour/
            Snapcraft tour initialized in './snapcraft-tour/'
            Instructions are in the README, or http://snapcraft.io/create/#tour"""))) # noqa
        self.assertThat('snapcraft-tour', DirExists())

    def test_scaffold_to_path(self):
        result = self.run_command(['tour', 'my-tour'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(dedent("""\
            Copying examples tour to my-tour
            Snapcraft tour initialized in 'my-tour'
            Instructions are in the README, or http://snapcraft.io/create/#tour"""))) # noqa
        self.assertThat('my-tour', DirExists())

    def test_scaffold_twice_with_defaults(self):
        result = self.run_command(['tour'])
        self.assertThat(result.exit_code, Equals(0))

        result = self.run_command(['tour'])
        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(
            "./snapcraft-tour/' already exists, please specify a destination "
            "directory."))

    def test_scaffold_to_existing_directory(self):
        os.mkdir('existing-directory')

        result = self.run_command(['tour', 'existing-directory'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(dedent("""\
            Copying examples tour to existing-directory
            Snapcraft tour initialized in 'existing-directory'
            Instructions are in the README, or http://snapcraft.io/create/#tour"""))) # noqa

    def test_scaffold_to_file_fails_gracefully(self):
        open('target-file', 'w').close()

        result = self.run_command(['tour', 'target-file'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(dedent("""\
            Copying examples tour to target-file
            {!r} is a file, cannot be used as a destination""".format(
                os.path.abspath('target-file')))))
