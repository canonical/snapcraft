# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Tests for the Snapcraft provider service."""


def test_provider(provider_service, monkeypatch):
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "foo")
    monkeypatch.setenv("SNAPCRAFT_IMAGE_INFO", "bar")
    monkeypatch.setenv("SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS", "baz")
    provider_service.setup()
    assert provider_service.environment["SNAPCRAFT_BUILD_INFO"] == "foo"
    assert provider_service.environment["SNAPCRAFT_IMAGE_INFO"] == "bar"
    assert (
        provider_service.environment["SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS"]
        == "baz"
    )
