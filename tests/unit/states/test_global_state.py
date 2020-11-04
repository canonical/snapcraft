# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

from snapcraft.internal.states import GlobalState

_scenarios = [
    (
        "build-snaps, build-packages and required-grade",
        dict(
            build_packages=["pkg1", "pkg2"],
            build_snaps=["snap1", "snap2"],
            required_grade="stable",
        ),
    ),
    (
        "build-snaps and no build-packages nor required-grade",
        dict(build_packages=[], build_snaps=["snap1", "snap2"], required_grade=None),
    ),
    (
        "no build-snaps, build-packages and no required-grade",
        dict(build_packages=["pkg1", "pkg2"], build_snaps=[], required_grade=None),
    ),
    (
        "no build-snaps nor build-packages and required-grade",
        dict(build_packages=[], build_snaps=[], required_grade="stable"),
    ),
]


class TestGlobalState:

    scenarios = _scenarios

    def test_save(self, tmp_work_path, build_packages, build_snaps, required_grade):
        global_state = GlobalState()
        global_state.append_build_packages(build_packages)
        global_state.append_build_snaps(build_snaps)
        global_state.set_required_grade(required_grade)

        global_state.save(filepath="state")

        prepend = "  - "

        if build_packages:
            build_packages = "\n" + "\n".join(
                ["{}{}".format(prepend, p) for p in build_packages]
            )
        else:
            build_packages = " []"

        if build_snaps:
            build_snaps = "\n" + "\n".join(
                ["{}{}".format(prepend, p) for p in build_snaps]
            )
        else:
            build_snaps = " []"

        if required_grade:
            required_grade = required_grade
        else:
            required_grade = "null"

        with open("state") as state_file:
            state_file_contents = state_file.read()
            assert (
                state_file_contents
                == dedent(
                    """\
                    !GlobalState
                    assets:
                      build-packages:{}
                      build-snaps:{}
                      required-grade: {}
                    """
                ).format(build_packages, build_snaps, required_grade)
            )

    def test_load(self, tmp_work_path, build_packages, build_snaps, required_grade):
        if required_grade:
            required_grade = required_grade
        else:
            required_grade = "null"

        with open("state", "w") as state_file:
            print(
                dedent(
                    """\
                !GlobalState
                assets:
                  build-packages: {}
                  build-snaps: {}
                  required-grade: {}
                """
                ).format(build_packages, build_snaps, required_grade),
                file=state_file,
            )

        global_state = GlobalState.load(filepath="state")

        assert global_state.get_build_packages() == build_packages
        assert global_state.get_build_snaps() == build_snaps

    def test_save_load_and_append(
        self, tmp_work_path, build_packages, build_snaps, required_grade
    ):
        global_state = GlobalState()
        global_state.append_build_packages(build_packages)
        global_state.append_build_snaps(build_snaps)
        global_state.save(filepath="state")

        assert global_state.get_build_packages() == build_packages
        assert global_state.get_build_snaps() == build_snaps

        new_packages = ["new-pkg1", "new-pkg2"]
        new_snaps = ["new-snap1", "new-snap2"]
        global_state = GlobalState.load(filepath="state")
        global_state.append_build_packages(new_packages)
        global_state.append_build_snaps(new_snaps)

        assert global_state.get_build_packages() == build_packages + new_packages

        assert global_state.get_build_snaps() == build_snaps + new_snaps

    def test_append_duplicate(
        self, tmp_work_path, build_packages, build_snaps, required_grade
    ):
        global_state = GlobalState()
        global_state.append_build_packages(build_packages)
        global_state.append_build_snaps(build_snaps)

        assert global_state.get_build_packages() == build_packages
        assert global_state.get_build_snaps() == build_snaps

        global_state.append_build_packages(build_packages)
        global_state.append_build_snaps(build_snaps)

        assert global_state.get_build_packages() == build_packages
        assert global_state.get_build_snaps() == build_snaps

    def test_load_with_missing(
        self, tmp_work_path, build_packages, build_snaps, required_grade
    ):
        with open("state", "w") as state_file:
            print("!GlobalState", file=state_file)
            print("assets: ", file=state_file)
            if build_packages:
                print("  build-packages: {}".format(build_packages), file=state_file)
            if build_snaps:
                print("  build-snaps: {}".format(build_snaps), file=state_file)
            if required_grade:
                print("  required-grade: {}".format(required_grade), file=state_file)

        global_state = GlobalState.load(filepath="state")

        assert global_state.get_build_packages() == build_packages
        assert global_state.get_build_snaps() == build_snaps
        assert global_state.get_required_grade() == required_grade
