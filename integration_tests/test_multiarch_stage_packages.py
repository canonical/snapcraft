# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import subprocess

from testtools.matchers import Contains

import integration_tests


class MultiarchTestCase(integration_tests.TestCase):

    def test_bad_multiarch_stage_packages(self):
        project_dir = 'bad-arch'
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            'pull', project_dir)

        self.assertThat(
            exception.output, Contains(
                "Error downloading stage packages for part 'my-part': The "
                "Ubuntu package 'hello:fake-arch' was not found.\n"
                'You may need to add support for this architecture with '
                "'dpkg --add-architecture fake-arch'"))
