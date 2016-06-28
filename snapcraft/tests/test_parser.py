# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import os
from unittest import mock

import requests
import yaml

from snapcraft.internal.parser import (
    _get_namespaced_partname,
    PART_NAMESPACE_SEP,
    PARTS_FILE,
    main,
)
from snapcraft.tests import TestCase, fixture_setup

TEST_OUTPUT_PATH = os.path.join(os.getcwd(), 'test_output.wiki')


def _create_example_output(output):
    with open(TEST_OUTPUT_PATH, 'w') as fp:
        fp.write(output)


def _get_part_list(path=PARTS_FILE):
    with open(path) as fp:
        return yaml.load(fp)


def _get_part(name, path=PARTS_FILE):
    part_list = _get_part_list(path)
    return part_list.get(name)


def _get_part_list_count(path=PARTS_FILE):
    return len(_get_part_list(path))


class TestParser(TestCase):
    def tearDown(self):
        try:
            os.remove(PARTS_FILE)
            os.remove(TEST_OUTPUT_PATH)
        except FileNotFoundError:
            pass

    def test_namespace(self):
        partname = 'part'
        subpart = 'subpart'

        result = _get_namespaced_partname(partname, subpart)

        self.assertEqual('{p}{s}{sp}'.format(p=partname,
                                             s=PART_NAMESPACE_SEP,
                                             sp=subpart),
                         result)

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_main_nested_parts_valid(self, mock_get, mock_get_origin_data):
        """Ensure that we fail if there are dependent parts that
        are not included in the wiki's 'parts' section."""

        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                    'after': ['part1'],
                },
                'part1': {
                    'source': 'lp:somethingelse1',
                    'plugin': 'copy',
                    'files': ['subfile1'],
                    'after': ['part2'],
                },
                'part2': {
                    'source': 'lp:somethingelse2',
                    'plugin': 'copy',
                    'files': ['subfile2'],
                },
            }
        }
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
parts: [part1, part2]
""")
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(3, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_main_nested_parts_invalid(self, mock_get, mock_get_origin_data):
        """Ensure that we fail if there are dependent parts that
        are not included in the wiki's 'parts' section."""

        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                    'after': ['part1'],
                },
                'part1': {
                    'source': 'lp:somethingelse1',
                    'plugin': 'copy',
                    'files': ['subfile1'],
                    'after': ['part2'],
                },
                'part2': {
                    'source': 'lp:somethingelse2',
                    'plugin': 'copy',
                    'files': ['subfile2'],
                },
            }
        }
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
parts: [part1]
""")
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_wiki_code_tags(self, mock_get, mock_get_origin_data):
        _create_example_output("""
{{{
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
}}}
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_main_valid(self, mock_get, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_main_invalid(self,
                          mock_get,
                          mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
parts: [part1]
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                    'after': ['part1'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_single_part_origin(self,
                                mock_get,
                                mock_get_origin_data):
        """Test a wiki entry with a single origin part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_multiple_part_origin(self,
                                  mock_get,
                                  mock_get_origin_data):
        """Test a wiki entry with multiple origin parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
parts: ['subpart']
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                    'after': ['subpart'],
                },
                'subpart': {
                    'source': 'lp:somethingelse',
                    'plugin': 'copy',
                    'files': ['subfile2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        self.assertEqual(2, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_output_parameter(self,
                              mock_get,
                              mock_get_origin_data):
        """Test a wiki entry with multiple origin parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }

        filename = 'parts.yaml'

        main(['--debug', '--index', TEST_OUTPUT_PATH, '--output', filename])

        self.assertEqual(1, _get_part_list_count(filename))
        self.assertTrue(os.path.exists(filename))

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_source_with_local_part_origin(self,
                                           mock_get,
                                           mock_get_origin_data):
        """Test a wiki entry with a source with a local part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': '.',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())
        part = _get_part('main')
        self.assertNotEqual('.', part['source'])
        self.assertEqual(5, len(part.keys()))

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_source_with_local_subdir_part_origin(self,
                                                  mock_get,
                                                  mock_get_origin_data):
        """Test a wiki entry with a source with a local part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'local',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())
        part = _get_part('main')
        self.assertNotEqual('local', part['source'])
        self.assertEqual('local', part['source-subdir'])
        self.assertEqual(6, len(part.keys()))

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_source_with_local_source_subdir_part_origin(
            self, mock_get, mock_get_origin_data):
        """Test a wiki entry with a source with a local source-subdir part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': '.',
                    'source-subdir': 'local',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())
        part = _get_part('main')
        self.assertNotEqual('.', part['source'])
        self.assertEqual('local', part['source-subdir'])
        self.assertEqual(6, len(part.keys()))

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_n_documents(
            self, mock_get, mock_get_origin_data):
        """Test 2 wiki entries."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main2
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:project',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
                'main2': {
                    'source': 'lp:project',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        self.assertEqual(2, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_maintaner_is_included(
            self, mock_get, mock_get_origin_data):
        """Test maintainer is included in parsed parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main
project-part: main
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main2
project-part: main2
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:project',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
                'main2': {
                    'source': 'lp:project',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        part = _get_part('main')
        self.assertEqual('John Doe <john.doe@example.com>', part['maintainer'])

        part = _get_part('main2')
        self.assertEqual('Jim Doe <jim.doe@example.com>', part['maintainer'])

        self.assertEqual(2, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_description_is_included(
            self, mock_get, mock_get_origin_data):
        """Test description is included in parsed parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main
project-part: main
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main2
project-part: main2
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:project',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
                'main2': {
                    'source': 'lp:project',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        part = _get_part('main')
        self.assertEqual('example main', part['description'])

        part = _get_part('main2')
        self.assertEqual('example main2', part['description'])

        self.assertEqual(2, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_origin_part_without_source(self, mock_get, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())
        self.assertEqual(4, len(_get_part('main').keys()))

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_missing_fields(self, mock_get, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_partial_processing_for_malformed_yaml(self,
                                                   mock_get,
                                                   mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description:
  example

  Usage
    blahblahblah
project-part: 'main'
---
maintainer: John Doeson <john.doeson@example.com>
origin: lp:snapcraft-parser-example
description:
  example

  Usage:
    blahblahblah
project-part: 'main2'
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
                'main2': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_wiki_interactions_with_fake(self,
                                         mock_get,
                                         mock_get_origin_data):

        fixture = fixture_setup.FakePartsWiki()
        self.useFixture(fixture)

        mock_get_origin_data.return_value = {
            'parts': {
                'curl': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', fixture.fake_parts_wiki_fixture.url])
        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft.internal.sources.get')
    def test_wiki_with_fake_origin(self, mock_get):

        fixture = fixture_setup.FakePartsWikiOrigin()
        self.useFixture(fixture)
        origin_url = fixture.fake_parts_wiki_origin_fixture.url

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: {origin_url}
description: example
project-part: 'somepart'
""".format(origin_url=origin_url))

        # TODO: update this once we start encoding the origin_dir
        origin_dir = os.path.join('/tmp', 'somepart')
        os.makedirs(origin_dir, exist_ok=True)

        # Create a fake snapcraft.yaml for _get_origin_data() to parse
        with open(os.path.join(origin_dir, 'snapcraft.yaml'),
                  'w') as fp:
            text = requests.get(origin_url).text
            fp.write(text)

        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())
        part = _get_part('somepart')
        self.assertTrue(part is not None)

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_duplicate_entries(self, mock_get, mock_get_origin_data):
        """Test duplicate parts are ignored."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main
project-part: main
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main duplicate
project-part: main
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main': {
                    'source': 'lp:project',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])

        part = _get_part('main')
        self.assertEqual('example main', part['description'])

        self.assertEqual(1, _get_part_list_count())
