# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

import fixtures

import integration_tests


class ListPluginsTestCase(integration_tests.TestCase):

    def test_list_plugins(self):
        # Alignment issues are complicated and it is hard for people
        # to maintain so let's make it easy and enjoyable.
        self.useFixture(fixtures.EnvironmentVariable('COLUMNS', '15'))
        output = self.run_snapcraft('list-plugins')
        expected = """ant        
autotools  
catkin     
cmake      
copy       
go         
gulp       
jdk        
kbuild     
kernel     
make       
maven      
nil        
nodejs     
python2    
python3    
qmake      
scons      
tar-content
"""
        self.assertEqual(expected, output)
