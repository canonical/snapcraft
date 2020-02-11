# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from testtools.matchers import Equals, Is, IsInstance

from snapcraft.storeapi.v2 import progressive
from tests import unit


class ProgressiveTest(unit.TestCase):
    def test_progressive(self):
        payload = {"key": "my-key", "paused": False, "percentage": 83.3}

        p = progressive.Progressive(payload=payload)

        self.expectThat(repr(p), Equals("<Progressive: 'my-key'>"))
        self.expectThat(p.key, Equals(payload["key"]))
        self.expectThat(p.paused, Equals(payload["paused"]))
        self.expectThat(p.percentage, Equals(payload["percentage"]))

    def test_none(self):
        payload = {"key": None, "paused": None, "percentage": None}

        p = progressive.Progressive(payload=payload)

        self.expectThat(repr(p), Equals("<Progressive: None>"))
        self.expectThat(p.key, Equals(payload["key"]))
        self.expectThat(p.paused, Equals(payload["paused"]))
        self.expectThat(p.percentage, Equals(payload["percentage"]))
