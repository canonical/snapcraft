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

import testscenarios
from testtools.matchers import Equals

from snapcraft.extractors import _errors
from snapcraft.tests import unit


class AppstreamTestCase(unit.TestCase):

    scenarios = testscenarios.multiply_scenarios(
        [('summary', {'key': 'summary'}),
         ('description', {'key': 'description'})],
        [('metainfo', {'file_extension': 'metainfo.xml'}),
         ('appdata', {'file_extension': 'appdata.xml'})]
    )

    def test_appstream(self):
        file_name = 'foo.{}'.format(self.file_extension)
        with open(file_name, 'w') as f:
            f.write(textwrap.dedent("""\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <{key}>test-{key}</{key}>
                </component>""".format(key=self.key)))

        kwargs = {self.key: 'test-{}'.format(self.key)}
        expected = ExtractedMetadata(**kwargs)

        self.assertThat(
            appstream.extract(file_name), Equals(expected))


class AppstreamUnhandledFileTestCase(unit.TestCase):

    def test_unhandled_file_test_case(self):
        raised = self.assertRaises(
            _errors.UnhandledFileError, appstream.extract,
            'unhandled-file')

        self.assertThat(raised.path, Equals('unhandled-file'))
        self.assertThat(raised.extractor_name, Equals('appstream'))
