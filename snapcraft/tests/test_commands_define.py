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

from snapcraft import main, tests
from snapcraft.internal import parts


class DefineCommandTestCase(tests.TestWithFakeRemoteParts):

    def test_defining_a_part_that_exists(self):
        main.main(['define', 'curl'])

        expected_output = """Maintainer: 'none'
Description: test entry for curl

curl:
  plugin: autotools
  source: http://curl.org
"""
        self.assertEqual(self.fake_terminal.getvalue(), expected_output)

    def test_defining_a_part_that_doesnt_exist_helps_out(self):
        with self.assertRaises(RuntimeError) as raised:
            parts.define('curler')

        self.assertEqual(
            str(raised.exception),
            'Cannot find the part name {!r} in the cache. Please consider '
            'going to https://wiki.ubuntu.com/snapcraft/parts to add it.')

    def test_defining_a_part_with_multiline_description(self):
        main.main(['define', 'multiline-part'])

        expected_output = """Maintainer: 'none'
Description: this is a multiline description
this is a multiline description
this is a multiline description


multiline-part:
  plugin: go
  source: http://source.tar.gz
"""
        self.assertEqual(self.fake_terminal.getvalue(), expected_output)
