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

import os.path
import shutil
from unittest.mock import Mock, patch

from snapcraft.plugins.script import ScriptPlugin
from snapcraft.tests import TestCase


class TestScriptPlugin(TestCase):

    def setUp(self):
        self.mock_options = Mock()
        self.mock_options.script = ''
        self.mock_options.destination_dir = ''
        # setup the expected target dir in our tempdir
        self.dst_prefix = 'parts/script/install/'
        self.builddir = 'parts/script/install/'
        # Clean paths
        if os.path.exists(self.builddir):
            shutil.rmtree(self.builddir)
        if os.path.exists(self.dst_prefix):
            shutil.rmtree(self.dst_prefix)
        # create the test script file
        self.script_name = 'test_script.sh'
        with open(self.script_name, 'w') as f:
            f.write(
                '# assert the plugin gives us a destination dir argument\n'
                'if [ $# -lt 1 ]; then exit 1; fi\n'
                '# simply create a result file in the destination dir\n'
                'echo "" > $1/result\n'
            )
        # let's not pollute stdout
        patcher = patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

    def tearDown(self):
        try:
            os.remove(self.script_name)
        except OSError:
            pass
        if os.path.exists('parts'):
            shutil.rmtree('parts')

    def test_script_plugin_default_destination(self):
        self.mock_options.script = 'test_script.sh'
        p = ScriptPlugin('script', self.mock_options)
        p.build()
        self.assertTrue(os.path.exists(
            os.path.join(self.dst_prefix, 'result')
        ))

    def test_script_plugin_specific_destination(self):
        self.mock_options.script = 'test_script.sh'
        self.mock_options.destination_dir = 'specific/subdirectory'
        p = ScriptPlugin('script', self.mock_options)
        p.build()
        self.assertTrue(os.path.exists(
            os.path.join(self.dst_prefix, 'specific', 'subdirectory', 'result')
        ))
