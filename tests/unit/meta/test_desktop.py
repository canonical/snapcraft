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

from textwrap import dedent

import pytest

from snapcraft.internal.meta import errors
from snapcraft.internal.meta.desktop import DesktopFile


class TestDesktopExec:
    scenarios = (
        (
            "snap name != app name",
            dict(app_name="bar", app_args="", expected_exec="foo.bar"),
        ),
        (
            "snap name != app name",
            dict(app_name="bar", app_args="--arg", expected_exec="foo.bar --arg"),
        ),
        (
            "snap name == app name",
            dict(app_name="foo", app_args="", expected_exec="foo"),
        ),
        (
            "snap name == app name",
            dict(app_name="foo", app_args="--arg", expected_exec="foo --arg"),
        ),
        (
            "snap name == app name",
            dict(app_name="foo", app_args="--arg %U", expected_exec="foo --arg %U"),
        ),
        (
            "snap name == app name",
            dict(app_name="foo", app_args="%U", expected_exec="foo %U"),
        ),
    )

    def test_generate_desktop_file(
        self, tmp_work_path, app_name, app_args, expected_exec
    ):
        snap_name = "foo"

        desktop_file_path = tmp_work_path / "app.desktop"
        with desktop_file_path.open("w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print(
                "Exec={}".format(" ".join(["in-snap-exe", app_args])), file=desktop_file
            )

        d = DesktopFile(
            snap_name=snap_name,
            app_name=app_name,
            filename=desktop_file_path,
            prime_dir=tmp_work_path.as_posix(),
        )
        d.write(gui_dir=".")

        expected_desktop_file = tmp_work_path / f"{app_name}.desktop"
        assert expected_desktop_file.exists()
        with expected_desktop_file.open() as desktop_file:
            assert (
                desktop_file.read()
                == dedent(
                    """\
            [Desktop Entry]
            Exec={}

        """
                ).format(expected_exec)
            )


class TestDesktopIcon:
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
            dict(
                icon="${SNAP}/foo.png", icon_path=None, expected_icon="${SNAP}/foo.png"
            ),
        ),
        ("icon name", dict(icon="foo", icon_path=None, expected_icon="foo")),
    )

    def test_generate_desktop_file(self, tmp_work_path, icon, icon_path, expected_icon):
        snap_name = app_name = "foo"

        desktop_file_path = tmp_work_path / "app.desktop"
        with desktop_file_path.open("w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Exec=in-snap-exe", file=desktop_file)
            print("Icon={}".format(icon), file=desktop_file)

        if icon_path is not None:
            (tmp_work_path / icon_path).touch()

        d = DesktopFile(
            snap_name=snap_name,
            app_name=app_name,
            filename=desktop_file_path,
            prime_dir=tmp_work_path.as_posix(),
        )
        d.write(gui_dir=".")

        if icon_path is not None:
            d.write(icon_path=icon_path, gui_dir=".")
        else:
            d.write(gui_dir=".")

        expected_desktop_file = tmp_work_path / f"{app_name}.desktop"
        assert expected_desktop_file.exists()
        with expected_desktop_file.open() as desktop_file:
            assert (
                desktop_file.read()
                == dedent(
                    """\
            [Desktop Entry]
            Exec=foo
            Icon={}

        """
                ).format(expected_icon)
            )

    def test_generate_desktop_file_multisection(
        self, tmp_work_path, icon, icon_path, expected_icon
    ):
        snap_name = app_name = "foo"

        desktop_file_path = tmp_work_path / "app.desktop"
        with desktop_file_path.open("w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Exec=in-snap-exe", file=desktop_file)
            print("Icon={}".format(icon), file=desktop_file)
            print("[Desktop Entry Two]", file=desktop_file)
            print("Exec=in-snap-exe2", file=desktop_file)
            print("Icon={}".format(icon), file=desktop_file)

        if icon_path is not None:
            (tmp_work_path / icon_path).touch()

        d = DesktopFile(
            snap_name=snap_name,
            app_name=app_name,
            filename=desktop_file_path,
            prime_dir=tmp_work_path.as_posix(),
        )

        if icon_path is not None:
            d.write(icon_path=icon_path, gui_dir=".")
        else:
            d.write(gui_dir=".")

        expected_desktop_file = tmp_work_path / f"{app_name}.desktop"
        assert expected_desktop_file.exists()
        with expected_desktop_file.open() as desktop_file:
            assert (
                desktop_file.read()
                == dedent(
                    """\
            [Desktop Entry]
            Exec=foo
            Icon={0}

            [Desktop Entry Two]
            Exec=foo
            Icon={0}

        """
                ).format(expected_icon)
            )


def test_not_found(tmp_path):
    with pytest.raises(errors.InvalidDesktopFileError):
        DesktopFile(
            snap_name="foo",
            app_name="foo",
            filename="desktop-file-not-found",
            prime_dir=tmp_path.as_posix(),
        )


def test_no_desktop_section(tmp_work_path):
    with open("foo.desktop", "w") as desktop_file:
        print("[Random Entry]", file=desktop_file)
        print("Exec=foo", file=desktop_file)
        print("Icon=foo", file=desktop_file)

    d = DesktopFile(
        snap_name="foo",
        app_name="foo",
        filename="foo.desktop",
        prime_dir=tmp_work_path.as_posix(),
    )

    with pytest.raises(errors.InvalidDesktopFileError):
        d.write(gui_dir=tmp_work_path.as_posix())


def test_missing_exec_entry(tmp_work_path):
    with open("foo.desktop", "w") as desktop_file:
        print("[Desktop Entry]", file=desktop_file)
        print("Icon=foo", file=desktop_file)

    d = DesktopFile(
        snap_name="foo",
        app_name="foo",
        filename="foo.desktop",
        prime_dir=tmp_work_path.as_posix(),
    )

    with pytest.raises(errors.InvalidDesktopFileError):
        d.write(gui_dir=tmp_work_path.as_posix())
