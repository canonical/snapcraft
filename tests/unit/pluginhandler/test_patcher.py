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
from unittest import mock

from tests import unit


class PrimeTypeExcludesPatchingTestCase(unit.TestCase):

    scenarios = (
        ("kernel", dict(snap_type="kernel")),
        ("gadget", dict(snap_type="gadget")),
        ("base", dict(snap_type="base")),
        ("os", dict(snap_type="os")),
    )

    def test_no_patcher_called(self):
        handler = self.load_part(
            "test-part",
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
            "test-part", part_properties={"source-subdir": "src"}, snap_type="app"
        )

        patcher_class = "snapcraft.internal.pluginhandler.PartPatcher"
        with mock.patch(patcher_class) as patcher_mock:
            handler.prime()
            patcher_mock.assert_called_with(
                confinement="strict",
                core_base="core",
                elf_files=frozenset(),
                plugin=mock.ANY,
                primedir=self.prime_dir,
                project=mock.ANY,
                snap_base_path="/snap/fake-name/current",
                stage_packages=[],
                stagedir=self.stage_dir,
            )
