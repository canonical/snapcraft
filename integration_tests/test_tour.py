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

import integration_tests


class TourTestCase(integration_tests.TestCase):

    def test_tour_no_arg(self):
        output = self.run_snapcraft('tour')
        self.assertTrue(os.path.isdir('snapcraft-tour'),
                        'Tour directory was created')
        self.assertIn('Snapcraft tour initialized in ./snapcraft-tour/',
                      output)

    def test_tour_with_relative_dir(self):
        dest_dir = 'foo'
        output = self.run_snapcraft(['tour', dest_dir])
        self.assertTrue(os.path.isdir(dest_dir),
                        'Tour directory was created')
        self.assertIn('Snapcraft tour initialized in {}'.format(dest_dir),
                      output)

    def test_tour_with_absolute_dir(self):
        dest_dir = os.path.abspath('foo')
        output = self.run_snapcraft(['tour', dest_dir])
        self.assertTrue(os.path.isdir(dest_dir),
                        'Tour directory was created')
        self.assertIn('Snapcraft tour initialized in {}'.format(dest_dir),
                      output)
