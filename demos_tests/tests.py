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

import testscenarios

import demos_tests


class TestSnapcraftExamples(
        testscenarios.WithScenarios, demos_tests.ExampleTestCase):

    scenarios = [
        ('downloader-with-wiki-parts', {
            'snap_content_dir': 'downloader-with-wiki-parts',
            'name': 'downloader',
            'version': '1.0',
            }),
        ('godd', {
            'snap_content_dir': 'godd',
            'name': 'godd',
            'version': '1.0',
            }),
        ('git', {
            'snap_content_dir': 'git',
            'name': 'git',
            'version': '2.8.0',
            }),
        ('py2-project', {
            'snap_content_dir': 'py2-project',
            'name': 'spongeshaker',
            'version': '0',
            }),
        ('py3-project', {
            'snap_content_dir': 'py3-project',
            'name': 'spongeshaker',
            'version': '0',
            }),
    ]

    def test_demo(self):
        # Build snap will raise an exception in case of error.
        self.build_snap(self.snap_content_dir)
        # Install snap will raise an exception in case of error.
        self.install_snap(self.snap_content_dir, self.name, self.version)
