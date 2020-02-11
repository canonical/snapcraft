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

from snapcraft.storeapi.v2 import revision
from tests import unit


class RevisionTest(unit.TestCase):
    def test_revision(self):
        payload = {"revision": 2, "version": "2.0", "architectures": ["amd64", "arm64"]}

        r = revision.Revision(payload=payload)

        self.expectThat(
            repr(r),
            Equals(
                "<Revision: 2 for version '2.0' and architectures ['amd64', 'arm64']>"
            ),
        )
        self.expectThat(r.revision, Equals(payload["revision"]))
        self.expectThat(r.version, Equals(payload["version"]))
        self.expectThat(r.architectures, Equals(payload["architectures"]))
