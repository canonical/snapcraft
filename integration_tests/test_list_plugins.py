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

import pkgutil

from snapcraft import plugins

import integration_tests

from testtools.matchers import (
    Equals,
    StartsWith
)


class ListPluginsTestCase(integration_tests.TestCase):

    def test_list_plugins(self):
        output = self.run_snapcraft('list-plugins')
        version, plugins_list = output.split('\n', 1)
        self.assertThat(version, StartsWith("Starting snapcraft "))
        plugins_list = set(plugins_list.split())
        expected = set()
        for _, modname, _ in pkgutil.iter_modules(plugins.__path__):
            if not modname.startswith('_'):
                expected.add(modname.replace('_', '-'))
        self.assertThat(plugins_list, Equals(expected))
