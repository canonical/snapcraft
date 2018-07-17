# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

from snapcraft.project._project_options import _find_machine
from tests import unit


class OptionsTestCase(unit.TestCase):

    scenarios = [
        ("x86_64", dict(machine="x86_64", expected_machine="x86_64")),
        ("amd64", dict(machine="amd64", expected_machine="x86_64")),
        ("i386", dict(machine="i386", expected_machine="i686")),
        ("i686", dict(machine="i686", expected_machine="i686")),
        ("armhf", dict(machine="armhf", expected_machine="armv7l")),
        ("arm", dict(machine="arm", expected_machine="armv7l")),
        ("aarch64", dict(machine="aarch64", expected_machine="aarch64")),
        ("arm64", dict(machine="arm64", expected_machine="aarch64")),
        ("ppc64el", dict(machine="ppc64el", expected_machine="ppc64le")),
        ("ppc", dict(machine="powerpc", expected_machine="ppc")),
        ("s390x", dict(machine="s390x", expected_machine="s390x")),
    ]

    def test_find_machine(self):
        machine = _find_machine(self.machine)
        self.assertThat(machine, Equals(self.expected_machine))
