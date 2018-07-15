# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft import formatting_utils
from tests import unit


class HumanizeListTestCases(unit.TestCase):
    def test_no_items(self):
        items = []
        output = formatting_utils.humanize_list(items, "and")
        self.assertThat(output, Equals(""))

    def test_one_item(self):
        items = ["foo"]
        output = formatting_utils.humanize_list(items, "and")
        self.assertThat(output, Equals("'foo'"))

    def test_two_items(self):
        items = ["foo", "bar"]
        output = formatting_utils.humanize_list(items, "and")
        self.assertThat(
            output,
            Equals("'bar' and 'foo'"),
            "Expected 'bar' before 'foo' due to sorting",
        )

    def test_three_items(self):
        items = ["foo", "bar", "baz"]
        output = formatting_utils.humanize_list(items, "and")
        self.assertThat(output, Equals("'bar', 'baz', and 'foo'"))

    def test_four_items(self):
        items = ["foo", "bar", "baz", "qux"]
        output = formatting_utils.humanize_list(items, "and")
        self.assertThat(output, Equals("'bar', 'baz', 'foo', and 'qux'"))

    def test_another_conjunction(self):
        items = ["foo", "bar", "baz", "qux"]
        output = formatting_utils.humanize_list(items, "or")
        self.assertThat(output, Equals("'bar', 'baz', 'foo', or 'qux'"))


class FormatPathVariableTestCases(unit.TestCase):
    def test_no_paths(self):
        self.assertRaises(
            ValueError, formatting_utils.format_path_variable, "PATH", None, "/usr", ":"
        )

    def test_one_path(self):
        paths = ["/bin"]
        output = formatting_utils.format_path_variable("PATH", paths, "/usr", ":")
        self.assertThat(output, Equals('PATH="$PATH:/usr/bin"'))

    def test_two_paths(self):
        paths = ["/bin", "/sbin"]
        output = formatting_utils.format_path_variable("PATH", paths, "/usr", ":")
        self.assertThat(output, Equals('PATH="$PATH:/usr/bin:/usr/sbin"'))

    def test_two_paths_other_paremeters(self):
        paths = ["/usr/bin", "/usr/sbin"]
        output = formatting_utils.format_path_variable("PATH", paths, "", ",")
        self.assertThat(output, Equals('PATH="$PATH,/usr/bin,/usr/sbin"'))
