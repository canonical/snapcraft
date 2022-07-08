# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import pytest

from snapcraft import errors, extensions


@pytest.mark.usefixtures("fake_extension")
@pytest.mark.usefixtures("fake_extension_extra")
@pytest.mark.usefixtures("fake_extension_experimental")
def test_get_extension_names():
    assert extensions.get_extension_names() == [
        "gnome",
        "fake-extension-experimental",
        "fake-extension-extra",
        "fake-extension",
    ]


def test_get_extension_class(fake_extension):
    assert extensions.get_extension_class("fake-extension") == fake_extension


def test_get_extesion_class_not_found():
    # This is a developer error.
    with pytest.raises(errors.ExtensionError) as raised:
        extensions.get_extension_class("fake-extension-not-found")

    assert str(raised.value) == "Extension 'fake-extension-not-found' does not exist"
