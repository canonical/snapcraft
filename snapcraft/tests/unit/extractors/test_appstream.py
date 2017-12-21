# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import textwrap

from snapcraft.extractors import appstream, ExtractedMetadata

from testtools.matchers import Equals

from snapcraft.tests import unit


class AppstreamTestCase(unit.TestCase):

    scenarios = [
        ('summary', {
            'key': 'summary',
        }),
        ('description', {
            'key': 'description',
        }),
    ]

    def test_appstream(self):
        with open('foo.metainfo.xml', 'w') as f:
            f.write(textwrap.dedent("""\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <{key}>test-{key}</{key}>
                </component>""".format(key=self.key)))

        kwargs = {self.key: 'test-{}'.format(self.key)}
        expected = ExtractedMetadata(**kwargs)

        self.assertThat(
            appstream.extract('foo.metainfo.xml'), Equals(expected))
