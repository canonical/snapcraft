# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
from textwrap import dedent

from testtools.matchers import Equals

from snapcraft.internal import (
    common,
    errors
)
from snapcraft import tests


class CommonTestCase(tests.TestCase):

    def test_set_plugindir(self):
        plugindir = os.path.join(self.path, 'testplugin')
        common.set_plugindir(plugindir)
        self.assertThat(plugindir, Equals(common.get_plugindir()))

    def test_isurl(self):
        self.assertTrue(common.isurl('git://'))
        self.assertTrue(common.isurl('bzr://'))
        self.assertFalse(common.isurl('./'))
        self.assertFalse(common.isurl('/foo'))
        self.assertFalse(common.isurl('/fo:o'))


class CommonMigratedTestCase(tests.TestCase):

    def test_parallel_build_count_migration_message(self):
        raised = self.assertRaises(
            errors.PluginOutdatedError,
            common.get_parallel_build_count)

        self.assertThat(
            str(raised),
            Equals("This plugin is outdated: use 'parallel_build_count'"))

    def test_deb_arch_migration_message(self):
        raised = self.assertRaises(
            errors.PluginOutdatedError,
            common.get_arch)

        self.assertThat(
            str(raised),
            Equals("This plugin is outdated: use 'project.deb_arch'"))

    def test_arch_triplet_migration_message(self):
        raised = self.assertRaises(
            errors.PluginOutdatedError,
            common.get_arch_triplet)

        self.assertThat(
            str(raised),
            Equals("This plugin is outdated: use 'project.arch_triplet'"))


class FormatInColumnsTestCase(tests.TestCase):

    elements_list = ['ant', 'autotools', 'catkin', 'cmake', 'copy', 'go',
                     'jdk', 'kbuild', 'kernel', 'make', 'maven', 'nil',
                     'nodejs', 'python2', 'python3', 'scons', 'tar-content']

    def test_format_output_in_columns_default(self):
        """Format output on 2 lines, with default max-width and space sep"""
        expected = ['ant        catkin  copy  jdk     kernel  maven  '
                    'nodejs   python3  tar-content',
                    'autotools  cmake   go    kbuild  make    nil    '
                    'python2  scons  ']
        self.assertThat(
            common.format_output_in_columns(self.elements_list),
            Equals(expected))

    def test_format_output_in_columns_narrow(self):
        """Format output on 3 lines, with narrow max-width and space sep"""
        expected = ['ant        cmake  jdk     make   nodejs   scons      ',
                    'autotools  copy   kbuild  maven  python2  tar-content',
                    'catkin     go     kernel  nil    python3']
        self.assertThat(
            common.format_output_in_columns(self.elements_list,
                                            max_width=60),
            Equals(expected))

    def test_format_output_in_columns_large(self):
        """Format output on one big line, with default space sep"""
        expected = ['ant  autotools  catkin  cmake  copy  go  jdk  kbuild  '
                    'kernel  make  maven  nil  nodejs  python2  python3  '
                    'scons  tar-content']
        self.assertThat(
            common.format_output_in_columns(self.elements_list,
                                            max_width=990),
            Equals(expected))

    def test_format_output_in_columns_one_space(self):
        """Format output with one space sep"""
        expected = ['ant       cmake jdk    make  nodejs  scons      ',
                    'autotools copy  kbuild maven python2 tar-content',
                    'catkin    go    kernel nil   python3']
        self.assertThat(
            common.format_output_in_columns(self.elements_list,
                                            max_width=60,
                                            num_col_spaces=1),
            Equals(expected))


class OSReleaseTestCase(tests.TestCase):

    def test_os_release_with_blank_lines(self):
        with open('os-release', 'w') as release_file:
            print(dedent("""\
                NAME="Arch Linux"

                PRETTY_NAME="Arch Linux"
                ID=arch
                ID_LIKE=archlinux
                ANSI_COLOR="0;36"
                HOME_URL="https://www.archlinux.org/"
                SUPPORT_URL="https://bbs.archlinux.org/"
                BUG_REPORT_URL="https://bugs.archlinux.org/"

            """), file=release_file)

        release_info = common.get_os_release_info(os_release_file='os-release')

        expected_release_info = dict(
            NAME='Arch Linux',
            PRETTY_NAME='Arch Linux',
            ID='arch',
            ID_LIKE='archlinux',
            ANSI_COLOR='0;36',
            HOME_URL='https://www.archlinux.org/',
            SUPPORT_URL='https://bbs.archlinux.org/',
            BUG_REPORT_URL='https://bugs.archlinux.org/',
        )

        self.assertThat(release_info, Equals(expected_release_info))
