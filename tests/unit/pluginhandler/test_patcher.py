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

import fixtures

from snapcraft.internal.pluginhandler import PartPatcher
from tests import unit


class StaticBasePatchingTest(unit.TestCase):
    scenarios = (
        ("strict", dict(confinement="strict")),
        ("classic", dict(confinement="classic")),
    )

    def setUp(self):
        super().setUp()

        self.fake_patchelf = fixtures.MockPatch("snapcraft.internal.elf.Patcher")
        self.useFixture(self.fake_patchelf)

    def test_static_base_with_libc6_stage_packaged(self):
        # The "bare" base is a static base.
        handler = self.load_part(
            "test-part",
            snap_type="app",
            base="bare",
            build_base="core",
            confinement=self.confinement,
        )

        patcher = PartPatcher(
            elf_files=frozenset(["foo"]),
            project=handler._project,
            snap_base_path="/snap/test-snap/current",
            stage_packages=["libc6"],
        )

        patcher.patch()

        self.fake_patchelf.mock.assert_called_once_with(
            dynamic_linker="/snap/test-snap/current/lib/x86_64-linux-gnu/ld-2.27.so",
            preferred_patchelf_path=None,
            root_path=handler._project.prime_dir,
        )

    def test_static_base_without_libc6_stage_packaged(self):
        # The "bare" base is a static base, empty, so there is no linker loader to look for.
        handler = self.load_part(
            "test-part",
            snap_type="app",
            base="bare",
            build_base="core",
            confinement=self.confinement,
        )

        patcher = PartPatcher(
            elf_files=frozenset(["foo"]),
            project=handler._project,
            snap_base_path="/snap/test-snap/current",
            stage_packages=[],
        )

        patcher.patch()

        self.fake_patchelf.mock.assert_not_called()

    def test_no_base(self):
        handler = self.load_part(
            "test-part",
            snap_type="app",
            base=None,
            build_base="core",
            confinement=self.confinement,
        )

        patcher = PartPatcher(
            elf_files=frozenset(["foo"]),
            project=handler._project,
            snap_base_path="/snap/test-snap/current",
            stage_packages=[],
        )

        patcher.patch()

        self.fake_patchelf.mock.assert_not_called()


class PrimeTypeExcludesPatchingTestCase(unit.TestCase):

    scenarios = (
        ("kernel", dict(snap_type="kernel", snap_name="test-snap")),
        ("gadget", dict(snap_type="gadget", snap_name="test-snap")),
        ("base", dict(snap_type="base", snap_name="core18")),
        ("os", dict(snap_type="os", snap_name="test-snap")),
    )

    def test_no_patcher_called(self):
        handler = self.load_part(
            "test-part",
            snap_name=self.snap_name,
            part_properties={"source-subdir": "src"},
            snap_type=self.snap_type,
        )

        patcher_class = "snapcraft.internal.pluginhandler.PartPatcher"
        with mock.patch(patcher_class) as patcher_mock:
            handler.prime()
            patcher_mock.assert_not_called()


class PrimeTypeAppDoesPatchingTestCase(unit.TestCase):
    def test_patcher_called(self):
        handler = self.load_part(
            "test-part",
            part_properties={"source-subdir": "src"},
            snap_type="app",
            base="core18",
        )

        patcher_class = "snapcraft.internal.pluginhandler.PartPatcher"
        with mock.patch(patcher_class) as patcher_mock:
            handler.prime()
            patcher_mock.assert_called_with(
                elf_files=frozenset(),
                project=mock.ANY,
                snap_base_path="/snap/fake-name/current",
                stage_packages=[],
            )
