# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import contextlib
from textwrap import dedent

from testtools.matchers import FileContains, FileExists

from snapcraft.internal.meta.desktop import DesktopFile
from snapcraft.internal.meta import errors
from tests import unit


class DesktopExecTest(unit.TestCase):
    scenarios = (
        ("snap name != app name", dict(app_name="bar", expected_exec="foo.bar")),
        ("snap name == app name", dict(app_name="foo", expected_exec="foo")),
    )

    def setUp(self):
        super().setUp()
        self.snap_name = "foo"
        self.desktop_file_path = "app.desktop"

        with open(self.desktop_file_path, "w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Exec=in-snap-exe", file=desktop_file)

        self.expected_desktop_file = "{}.desktop".format(self.app_name)

    def test_generate_desktop_file(self):
        d = DesktopFile(
            snap_name=self.snap_name,
            app_name=self.app_name,
            filename=self.desktop_file_path,
            prime_dir=self.path,
        )
        d.write(gui_dir=".")

        self.assertThat(self.expected_desktop_file, FileExists())
        self.assertThat(
            self.expected_desktop_file,
            FileContains(
                dedent(
                    """\
            [Desktop Entry]
            Exec={} %U

        """
                ).format(self.expected_exec)
            ),
        )


class DesktopIconTest(unit.TestCase):
    scenarios = (
        (
            "icon_path preferred",
            dict(
                icon="other.png", icon_path="foo.png", expected_icon="${SNAP}/foo.png"
            ),
        ),
        (
            "icon_path with / preferred",
            dict(icon="/foo.png", icon_path="foo.png", expected_icon="${SNAP}/foo.png"),
        ),
        (
            "icon path with ${SNAP}",
            dict(icon="${SNAP}/foo.png", expected_icon="${SNAP}/foo.png"),
        ),
        ("icon name", dict(icon="foo", expected_icon="foo")),
    )

    def setUp(self):
        super().setUp()
        self.snap_name = self.app_name = "foo"
        self.desktop_file_path = "other.desktop"

        with contextlib.suppress(AttributeError):
            open(self.icon_path, "w").close()

        with open(self.desktop_file_path, "w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Exec=in-snap-exe", file=desktop_file)
            print("Icon={}".format(self.icon), file=desktop_file)

        self.expected_desktop_file = "{}.desktop".format(self.app_name)

    def test_generate_desktop_file(self):
        d = DesktopFile(
            snap_name=self.snap_name,
            app_name=self.app_name,
            filename=self.desktop_file_path,
            prime_dir=self.path,
        )
        try:
            d.write(icon_path=self.icon_path, gui_dir=".")
        except AttributeError:
            d.write(gui_dir=".")

        self.assertThat(self.expected_desktop_file, FileExists())
        self.assertThat(
            self.expected_desktop_file,
            FileContains(
                dedent(
                    """\
            [Desktop Entry]
            Exec=foo %U
            Icon={}

        """.format(
                        self.expected_icon
                    )
                )
            ),
        )


class DesktopFileErrorTest(unit.TestCase):
    def test_not_found(self):
        self.assertRaises(
            errors.InvalidDesktopFileError,
            DesktopFile,
            snap_name="foo",
            app_name="foo",
            filename="desktop-file-not-found",
            prime_dir=self.path,
        )

    def test_no_desktop_section(self):
        with open("foo.desktop", "w") as desktop_file:
            print("[Random Entry]", file=desktop_file)
            print("Exec=foo", file=desktop_file)
            print("Icon=foo", file=desktop_file)

        d = DesktopFile(
            snap_name="foo", app_name="foo", filename="foo.desktop", prime_dir=self.path
        )

        self.assertRaises(errors.InvalidDesktopFileError, d.write, gui_dir=self.path)

    def test_missing_exec_entry(self):
        with open("foo.desktop", "w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Icon=foo", file=desktop_file)

        d = DesktopFile(
            snap_name="foo", app_name="foo", filename="foo.desktop", prime_dir=self.path
        )

        self.assertRaises(errors.InvalidDesktopFileError, d.write, gui_dir=self.path)
