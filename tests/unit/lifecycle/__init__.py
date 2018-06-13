
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import fixtures
import logging
import textwrap

import snapcraft
from tests import unit


class BaseLifecycleTestCase(unit.TestCase):

    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.project_options = snapcraft.ProjectOptions()

    def make_snapcraft_yaml(self, parts, snap_type=''):
        yaml = textwrap.dedent("""\
            name: test
            version: 0
            summary: test
            description: test
            confinement: strict
            grade: stable
            {type}

            {parts}
            """)

        super().make_snapcraft_yaml(yaml.format(parts=parts, type=snap_type))
