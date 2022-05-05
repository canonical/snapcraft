# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019-2022 Canonical Ltd.
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

from pathlib import Path
from textwrap import dedent

import pytest

from snapcraft import errors
from snapcraft.parts.desktop_file import DesktopFile


class TestDesktopExec:
    """Exec entry rewriting."""

    @pytest.mark.parametrize(
        "app_name,app_args,expected_exec",
        [
            # snap name == app name
            ("foo", "", "foo"),
            ("foo", "--arg", "foo --arg"),
            ("foo", "--arg %U", "foo --arg %U"),
            ("foo", "%U", "foo %U"),

            # snap name != app name
            ("bar", "", "foo.bar"),
            ("bar", "--arg", "foo.bar --arg"),
        ]
    )
    def test_generate_desktop_file(
        self, new_dir, app_name, app_args, expected_exec
    ):
        snap_name = "foo"

        desktop_file_path = new_dir / "app.desktop"
        with desktop_file_path.open("w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print(
                f"Exec={' '.join(['in-snap-exe', app_args])}", file=desktop_file
            )

        d = DesktopFile(
            snap_name=snap_name,
            app_name=app_name,
            filename=desktop_file_path,
            prime_dir=new_dir.as_posix(),
        )
        d.write(gui_dir=Path())

        expected_desktop_file = new_dir / f"{app_name}.desktop"
        assert expected_desktop_file.exists()
        with expected_desktop_file.open() as desktop_file:
            assert desktop_file.read() == dedent(
                f"""\
                [Desktop Entry]
                Exec={expected_exec}

                """
            )


class TestDesktopIcon:
    """Icon entry rewriting."""

    @pytest.mark.parametrize(
        "icon,icon_path,expected_icon",
        [
            # icon_path preferred
            ("other.png", "foo.png", "${SNAP}/foo.png"),

            # icon_path with / preferred
            ("/foo.png", "foo.png", "${SNAP}/foo.png"),

            # icon path with ${SNAP}
            ("${SNAP}/foo.png", None, "${SNAP}/foo.png"),

            # icon name
            ("foo", None, "foo"),
        ]
    )
    def test_generate_desktop_file(self, new_dir, icon, icon_path, expected_icon):
        snap_name = app_name = "foo"

        desktop_file_path = new_dir / "app.desktop"
        with desktop_file_path.open("w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Exec=in-snap-exe", file=desktop_file)
            print(f"Icon={icon}", file=desktop_file)

        if icon_path is not None:
            (new_dir / icon_path).touch()

        d = DesktopFile(
            snap_name=snap_name,
            app_name=app_name,
            filename=desktop_file_path,
            prime_dir=new_dir,
        )
        d.write(gui_dir=Path())

        if icon_path is not None:
            d.write(icon_path=icon_path, gui_dir=Path())
        else:
            d.write(gui_dir=Path())

        expected_desktop_file = new_dir / f"{app_name}.desktop"
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

    @pytest.mark.parametrize(
        "icon,icon_path,expected_icon",
        [
            # icon_path preferred
            ("other.png", "foo.png", "${SNAP}/foo.png"),

            # icon_path with / preferred
            ("/foo.png", "foo.png", "${SNAP}/foo.png"),

            # icon path with ${SNAP}
            ("${SNAP}/foo.png", None, "${SNAP}/foo.png"),

            # icon name
            ("foo", None, "foo"),
        ]
    )
    def test_generate_desktop_file_multisection(
        self, new_dir, icon, icon_path, expected_icon
    ):
        snap_name = app_name = "foo"

        desktop_file_path = new_dir / "app.desktop"
        with desktop_file_path.open("w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Exec=in-snap-exe", file=desktop_file)
            print(f"Icon={icon}", file=desktop_file)
            print("[Desktop Entry Two]", file=desktop_file)
            print("Exec=in-snap-exe2", file=desktop_file)
            print(f"Icon={icon}", file=desktop_file)

        if icon_path is not None:
            (new_dir / icon_path).touch()

        d = DesktopFile(
            snap_name=snap_name,
            app_name=app_name,
            filename=desktop_file_path,
            prime_dir=new_dir,
        )

        if icon_path is not None:
            d.write(icon_path=icon_path, gui_dir=Path())
        else:
            d.write(gui_dir=Path())

        expected_desktop_file = new_dir / f"{app_name}.desktop"
        assert expected_desktop_file.exists()
        with expected_desktop_file.open() as desktop_file:
            assert desktop_file.read() == dedent(
                f"""\
                [Desktop Entry]
                Exec=foo
                Icon={expected_icon}

                [Desktop Entry Two]
                Exec=foo
                Icon={expected_icon}

                """
            )


def test_not_found(new_dir):
    with pytest.raises(errors.DesktopFileError):
        DesktopFile(
            snap_name="foo",
            app_name="foo",
            filename="desktop-file-not-found",
            prime_dir=new_dir,
        )


def test_no_desktop_section(new_dir):
    with open("foo.desktop", "w") as desktop_file:
        print("[Random Entry]", file=desktop_file)
        print("Exec=foo", file=desktop_file)
        print("Icon=foo", file=desktop_file)

    d = DesktopFile(
        snap_name="foo",
        app_name="foo",
        filename="foo.desktop",
        prime_dir=new_dir,
    )

    with pytest.raises(errors.DesktopFileError):
        d.write(gui_dir=new_dir)


def test_missing_exec_entry(new_dir):
    with open("foo.desktop", "w") as desktop_file:
        print("[Desktop Entry]", file=desktop_file)
        print("Icon=foo", file=desktop_file)

    d = DesktopFile(
        snap_name="foo",
        app_name="foo",
        filename="foo.desktop",
        prime_dir=new_dir,
    )

    with pytest.raises(errors.DesktopFileError):
        d.write(gui_dir=new_dir)
