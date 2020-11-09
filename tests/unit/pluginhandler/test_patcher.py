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

from unittest import mock

import pytest

from snapcraft import file_utils
from snapcraft.internal import errors
from snapcraft.internal.pluginhandler import PartPatcher
from tests.unit import load_part


@pytest.fixture
def mock_elf_patcher():
    """Return a mock for snapcraft.internal.elf.Patcher."""
    patcher = mock.patch("snapcraft.internal.elf.Patcher", autospec=True)
    yield patcher.start()
    patcher.stop()


@pytest.fixture
def mock_partpatcher():
    """Return a mock for snapcraft.internal.pluginhandler.PartPatcher."""
    patcher = mock.patch("snapcraft.internal.pluginhandler.PartPatcher", autospec=True)
    yield patcher.start()
    patcher.stop()


class TestStaticBasePatching:
    scenarios = (
        ("strict", dict(confinement="strict")),
        ("classic", dict(confinement="classic")),
    )

    def test_static_base_with_libc6_stage_packaged(self, mock_elf_patcher, confinement):
        # The "bare" base is a static base.
        handler = load_part(
            "test-part",
            snap_type="app",
            base="bare",
            build_base="core",
            confinement=confinement,
        )

        patcher = PartPatcher(
            elf_files=frozenset(["foo"]),
            project=handler._project,
            snap_base_path="/snap/test-snap/current",
            stage_packages=["libc6"],
        )

        patcher.patch()

        mock_elf_patcher.assert_called_once_with(
            dynamic_linker="/snap/test-snap/current/lib/x86_64-linux-gnu/ld-2.27.so",
            preferred_patchelf_path=None,
            root_path=handler._project.prime_dir,
        )

    def test_static_base_without_libc6_stage_packaged(
        self, mock_elf_patcher, confinement
    ):
        # The "bare" base is a static base, empty, so there is no linker loader to look for.
        handler = load_part(
            "test-part",
            snap_type="app",
            base="bare",
            build_base="core",
            confinement=confinement,
        )

        patcher = PartPatcher(
            elf_files=frozenset(["foo"]),
            project=handler._project,
            snap_base_path="/snap/test-snap/current",
            stage_packages=[],
        )

        patcher.patch()

        mock_elf_patcher.assert_not_called()

    def test_no_base(self, mock_elf_patcher, confinement):
        handler = load_part(
            "test-part",
            snap_type="app",
            base=None,
            build_base="core",
            confinement=confinement,
        )

        patcher = PartPatcher(
            elf_files=frozenset(["foo"]),
            project=handler._project,
            snap_base_path="/snap/test-snap/current",
            stage_packages=[],
        )

        patcher.patch()

        mock_elf_patcher.assert_not_called()

    def test_conflicting_patchelf_build_attributes(
        self, monkeypatch, mock_partpatcher, confinement
    ):
        monkeypatch.setattr(file_utils, "get_snap_tool_path", lambda x: x)

        part_properties = {
            "source-subdir": "src",
            "build-attributes": ["no-patchelf", "enable-patchelf"],
        }

        handler = load_part(
            "test-part",
            snap_name="test-snap",
            part_properties=part_properties,
            confinement=confinement,
        )

        with pytest.raises(errors.BuildAttributePatchelfConflictError):
            handler.prime()


class TestPrimeTypeExcludesPatching:

    scenarios = (
        (
            "kernel",
            dict(
                snap_type="kernel",
                snap_name="test-snap",
                confinement=None,
                build_attributes=None,
            ),
        ),
        (
            "gadget",
            dict(
                snap_type="gadget",
                snap_name="test-snap",
                confinement=None,
                build_attributes=None,
            ),
        ),
        (
            "base",
            dict(
                snap_type="base",
                snap_name="core18",
                confinement=None,
                build_attributes=None,
            ),
        ),
        (
            "os",
            dict(
                snap_type="os",
                snap_name="test-snap",
                confinement=None,
                build_attributes=None,
            ),
        ),
        (
            "strict app",
            dict(
                snap_type="app",
                snap_name="test-snap",
                confinement="strict",
                build_attributes=None,
            ),
        ),
        (
            "classic but disabled",
            dict(
                snap_type="app",
                snap_name="test-snap",
                confinement="classic",
                build_attributes=["no-patchelf"],
            ),
        ),
    )

    def test_no_patcher_called(
        self,
        monkeypatch,
        mock_partpatcher,
        snap_type,
        snap_name,
        confinement,
        build_attributes,
    ):
        monkeypatch.setattr(file_utils, "get_snap_tool_path", lambda x: x)

        part_properties = {"source-subdir": "src"}
        if build_attributes:
            part_properties["build-attributes"] = build_attributes

        handler = load_part(
            "test-part",
            snap_name=snap_name,
            part_properties=part_properties,
            snap_type=snap_type,
            confinement=confinement,
        )

        handler.prime()

        mock_partpatcher.assert_not_called()


class TestPrimeTypeIncludesPatching:

    scenarios = (
        (
            "classic",
            dict(confinement="classic", build_attributes=None, stage_packages=None),
        ),
        (
            "strict but enabled",
            dict(
                confinement="strict",
                build_attributes=["enable-patchelf"],
                stage_packages=None,
            ),
        ),
        (
            "libc staged",
            dict(confinement="strict", build_attributes=None, stage_packages=["libc6"]),
        ),
    )

    def test_patcher_called(
        self,
        monkeypatch,
        mock_partpatcher,
        confinement,
        build_attributes,
        stage_packages,
    ):
        monkeypatch.setattr(file_utils, "get_snap_tool_path", lambda x: x)

        part_properties = {"source-subdir": "src"}
        if build_attributes:
            part_properties["build-attributes"] = build_attributes
        if stage_packages:
            part_properties["stage-packages"] = stage_packages
        else:
            stage_packages = list()

        handler = load_part(
            "test-part",
            snap_type="app",
            part_properties=part_properties,
            confinement=confinement,
        )

        handler.prime()

        mock_partpatcher.assert_called_with(
            elf_files=frozenset(),
            project=mock.ANY,
            snap_base_path="/snap/fake-name/current",
            stage_packages=stage_packages,
        )
