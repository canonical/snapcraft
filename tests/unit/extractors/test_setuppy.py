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

from textwrap import dedent

from snapcraft.extractors import setuppy, ExtractedMetadata

from testtools.matchers import Equals

from snapcraft.extractors import _errors
from tests import unit


class SetupPyTestCase(unit.TestCase):

    scenarios = [
        ('description', dict(params=dict(
            version=None,
            description='test-description'))),
        ('version', dict(params=dict(
            version='test-version',
            description=None))),
        ('key and version', dict(params=dict(
            description='test-description',
            version='test-version')))
    ]

    def setUp(self):
        super().setUp()

        params = ['    {}="{}",'.format(k, v)
                  for k, v in self.params.items() if v]

        with open('setup.py', 'w') as setup_file:
            print(dedent("""\
                import setuptools

                setuptools.setup(
                    name='hello-world',
                {}
                    author='Canonical LTD',
                    author_email='snapcraft@lists.snapcraft.io',
                    scripts=['hello']
                )
            """).format('\n'.join(params)), file=setup_file)

    def test_info_extraction(self):
        expected = ExtractedMetadata(**self.params)
        actual = setuppy.extract('setup.py')
        self.assertThat(str(actual), Equals(str(expected)))
        self.assertThat(actual, Equals(expected))


class SetupPyUnhandledFileTestCase(unit.TestCase):

    def test_unhandled_file_test_case(self):
        raised = self.assertRaises(
            _errors.UnhandledFileError, setuppy.extract,
            'unhandled-file')

        self.assertThat(raised.path, Equals('unhandled-file'))
        self.assertThat(raised.extractor_name, Equals('setup.py'))
