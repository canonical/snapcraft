# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from textwrap import dedent

from testtools.matchers import Equals, FileContains

from snapcraft.internal.states import GlobalState
from tests import unit


_scenarios = [
    (
        "build-snaps and build-packages",
        dict(build_packages=["pkg1", "pkg2"], build_snaps=["snap1", "snap2"]),
    ),
    (
        "build-snaps and no build-packages",
        dict(build_packages=[], build_snaps=["snap1", "snap2"]),
    ),
    (
        "no build-snaps and build-packages",
        dict(build_packages=["pkg1", "pkg2"], build_snaps=[]),
    ),
]


class _GlobalStateSaveTest(unit.TestCase):

    scenarios = _scenarios

    def test_save(self):
        global_state = GlobalState(
            build_packages=self.build_packages, build_snaps=self.build_snaps
        )
        global_state.save(filepath="state")

        build_packages_str = ", ".join(self.build_packages)
        build_snaps_str = ", ".join(self.build_snaps)

        self.assertThat(
            "state",
            FileContains(
                dedent(
                    """\
                    !GlobalState
                    assets:
                      build-packages: [{}]
                      build-snaps: [{}]
                    """
                ).format(build_packages_str, build_snaps_str)
            ),
        )


class _GlobalStateLoadTest(unit.TestCase):

    scenarios = _scenarios

    def test_load(self):
        with open("state", "w") as state_file:
            print(
                dedent(
                    """\
                !GlobalState
                assets:
                  build-packages: {}
                  build-snaps: {}
                """
                ).format(self.build_packages, self.build_snaps),
                file=state_file,
            )

        global_state = GlobalState.load(filepath="state")

        self.assertThat(global_state.get_build_packages(), Equals(self.build_packages))
        self.assertThat(global_state.get_build_snaps(), Equals(self.build_snaps))


class _GlobalStateLoadMissingKeysTest(unit.TestCase):

    scenarios = _scenarios

    def test_load(self):
        with open("state", "w") as state_file:
            print("!GlobalState", file=state_file)
            print("assets: ", file=state_file)
            if self.build_packages:
                print(
                    "  build-packages: {}".format(self.build_packages), file=state_file
                )
            if self.build_snaps:
                print("  build-snaps: {}".format(self.build_snaps), file=state_file)

        global_state = GlobalState.load(filepath="state")

        self.assertThat(global_state.get_build_packages(), Equals(self.build_packages))
        self.assertThat(global_state.get_build_snaps(), Equals(self.build_snaps))
