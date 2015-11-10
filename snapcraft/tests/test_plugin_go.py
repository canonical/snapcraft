# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

from snapcraft.plugins import go
from snapcraft import tests


class GoPluginTestCase(tests.TestCase):

    def test_environment(self):
        class Options:
            source = 'http://github.com/testplug'

        plugin = go.GoPlugin('test', Options())
        self.assertEqual(plugin.env('myroot'), [
            'GOPATH=myroot/go',
            'CGO_LDFLAGS=$CGO_LDFLAGS"-Lmyroot/lib -Lmyroot/usr/lib '
            '-Lmyroot/lib/x86_64-linux-gnu '
            '-Lmyroot/usr/lib/x86_64-linux-gnu $LDFLAGS"'])
