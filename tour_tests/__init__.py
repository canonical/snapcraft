# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

import inspect
import os
import sys

topdir = os.path.abspath(os.path.join(__file__, '..', '..'))
sys.path = [topdir] + sys.path

import demos_tests


class TourTestCase(demos_tests.ExampleTestCase):

    def __init__(self, *args, **kwargs):
        relative_path = os.path.relpath(
            os.path.dirname(inspect.getfile(self.__class__)),
            os.path.dirname(__file__))
        self.src_dir = os.path.join("tour", relative_path)
        super().__init__(*args, **kwargs)

