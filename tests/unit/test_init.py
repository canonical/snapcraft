# This file is part of starcraft.
#
# Copyright 2023 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License version 3, as published
# by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
# SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Basic Starcraft package demo unit tests."""
from unittest import mock

import starcraft


def test_version():
    assert starcraft.__version__ is not None


def test_hello(mocker):
    mocker.patch("builtins.print")

    starcraft.hello()

    print.assert_called_once_with("Hello *craft team!")


def test_hello_people(mocker):
    mocker.patch("builtins.print")

    starcraft.hello(["people"])

    print.assert_has_calls(
        [
            mock.call("Hello *craft team!"),
            mock.call("Hello people!"),
        ]
    )
