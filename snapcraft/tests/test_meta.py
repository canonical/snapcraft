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

import os

from snapcraft import (
    common,
    meta,
    tests
)


class WrapExeTestCase(tests.TestCase):

    def test_wrap_exe_must_write_wrapper(self):
        snapdir = common.get_snapdir()
        os.mkdir(snapdir)
        relative_exe_path = 'test_relexepath'
        relative_wrapper_path = meta._wrap_exe(relative_exe_path)
        wrapper_path = os.path.join(snapdir, relative_wrapper_path)

        expected = ('#!/bin/sh\n'
                    '\n'
                    'exec "$SNAP_APP_PATH/test_relexepath" $*\n')
        with open(wrapper_path, 'r') as wrapper_file:
            wrapper_contents = wrapper_file.read()

        self.assertEqual(expected, wrapper_contents)
