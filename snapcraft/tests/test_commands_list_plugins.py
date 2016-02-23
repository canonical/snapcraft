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

import io
from unittest import mock

from snapcraft import tests
from snapcraft.commands import list_plugins


class ListPluginsCommandTestCase(tests.TestCase):

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_list_plugins(self, mock_stdout):
        expected_list = '''ant
autotools
catkin
cmake
copy
go
jdk
make
maven
nil
nodejs
python2
python3
qml
scons
tar-content
'''
        list_plugins.main()
        self.assertEqual(mock_stdout.getvalue(), expected_list)
