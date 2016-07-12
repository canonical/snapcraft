# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import subprocess

from testtools import content

import integration_tests


class TestParser(integration_tests.TestCase):
    """Test bin/snapcraft-parser"""

    def test_parser_basic(self):
        """Test snapcraft-parser basic usage"""

        command = [self.snapcraft_parser_command, '-d', '--index',
                   'https://wiki.ubuntu.com/snapcraft/parts?action=raw',
                   '--output', 'parts.yaml']
        try:
            subprocess.check_output(
                command, stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise

        self.assertTrue(os.path.exists('parts.yaml'))
