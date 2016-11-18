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

import logging
import os
from unittest import mock

import requests
import fixtures
import yaml
from collections import OrderedDict

import snapcraft                           # noqa, initialize yaml
from snapcraft.internal.errors import MissingCommandError
from snapcraft.internal import parser
from snapcraft.internal.parser import (
    _get_origin_data,
    _encode_origin,
    BadSnapcraftYAMLError,
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


class TestParserBaseDir(TestCase):
    def test__get_base_dir(self):
        self.assertEqual('/tmp', parser._get_base_dir())


class TestParser(TestCase):
    def tearDown(self):
        try:
            os.remove(PARTS_FILE)
            os.remove(TEST_OUTPUT_PATH)
        except FileNotFoundError:
            pass

    def setUp(self):
        super().setUp()
        tempdir = fixtures.TempDir()
        self.useFixture(tempdir)
        self.tempdir_path = tempdir.path
        patcher = mock.patch('snapcraft.internal.parser._get_base_dir')
        base_dir = patcher.start()
        base_dir.return_value = tempdir.path
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.internal.sources.get')
        self.mock_get = patcher.start()
        self.addCleanup(patcher.stop)

    def test_ordereddict_yaml(self):
        from collections import OrderedDict
        data = OrderedDict()

        data['name'] = 'test'
        data['description'] = 'description'

        output = yaml.dump(data)

        self.assertTrue(isinstance(yaml.load(output), OrderedDict))

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_main_nested_parts_valid(self, mock_get_origin_data):
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
parts: [main, part1, part2]
""")
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(3, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_main_nested_parts_invalid(self, mock_get_origin_data):
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
parts: [main, part1]
""")
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_wiki_code_tags(self, mock_get_origin_data):
        _create_example_output("""
{{{
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
    def test_main_valid_variable_substition(self, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
""")
        mock_get_origin_data.return_value = {
            'name': 'something',
            'version': '0.1',
            'parts': {
                'main': {
                    'source': 'lp:$SNAPCRAFT_PROJECT_NAME'
                              '/r$SNAPCRAFT_PROJECT_VERSION',

                    'plugin': 'copy',
                    'files': ['$SNAPCRAFT_PROJECT_NAME/file1', 'file2'],
                },
            }
        }
        retval = main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())
        part = _get_part('main')
        self.assertEqual(part['source'], 'lp:something/r0.1')
        self.assertIn('something/file1', part['files'])
        self.assertEqual(0, retval)

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_main_valid(self, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
        retval = main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())
        self.assertEqual(0, retval)

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_main_slash_warning(self, mock_get_origin_data):
        fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(fake_logger)

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main/a]
""")
        mock_get_origin_data.return_value = {
            'parts': {
                'main/a': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())

        m = 'DEPRECATED: Found a "/" in the name of the {!r} part'.format(
            'main/a')
        self.assertTrue(m in fake_logger.output,
                        'Missing slash deprecation warning in output')

    def test_main_valid_with_default_index(self):
        main(['--debug'])
        self.assertEqual(0, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_main_invalid(self, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main, part1]
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
    def test_single_part_origin(self, mock_get_origin_data):
        """Test a wiki entry with a single origin part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
    def test_multiple_part_origin(self, mock_get_origin_data):
        """Test a wiki entry with multiple origin parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: ['main', 'subpart']
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
    def test_output_parameter(self, mock_get_origin_data):
        """Test a wiki entry with multiple origin parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
    def test_source_with_local_part_origin(self, mock_get_origin_data):
        """Test a wiki entry with a source with a local part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
    def test_source_with_local_subdir_part_origin(self, mock_get_origin_data):
        """Test a wiki entry with a source with a local part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
    def test_source_with_local_source_subdir_part_origin(
            self, mock_get_origin_data):
        """Test a wiki entry with a source with a local source-subdir part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
    def test_n_documents(self, mock_get_origin_data):
        """Test 2 wiki entries."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main2]
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
    def test_maintaner_is_included(self, mock_get_origin_data):
        """Test maintainer is included in parsed parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main
parts: [main]
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main2
parts: [main2]
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
    def test_description_is_included(self, mock_get_origin_data):
        """Test description is included in parsed parts."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main
parts: [main]
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main2
parts: [main2]
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
    def test_origin_part_without_source(self, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
parts: [main]
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
    def test_missing_fields(self, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
parts: [main]
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

        retval = main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())
        self.assertEqual(2, retval)

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_partial_processing_for_malformed_yaml(self, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description:
  example

  Usage
    blahblahblah
parts: [main]
---
maintainer: John Doeson <john.doeson@example.com>
origin: lp:snapcraft-parser-example
description:
  example

  Usage:
    blahblahblah
parts: [main2]
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
    def test_descriptions_get_block_quotes(self, mock_get_origin_data):
        output = """
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: |
  example

  Usage:
    blahblahblah
parts: [main]
"""
        _create_example_output(output)
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
        with open('snap-parts.yaml') as fp:
            data = fp.read()
            self.assertNotIn('description: "', data)
            self.assertIn('description: |', data)
        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_wiki_interactions_with_fake(self, mock_get_origin_data):

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

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_wiki_interactions_with_fake_with_slashes(
            self, mock_get_origin_data):

        fixture = fixture_setup.FakePartsWikiWithSlashes()
        self.useFixture(fixture)

        mock_get_origin_data.return_value = {
            'parts': {
                'curl/a': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
                'curl-a': {
                    'source': 'lp:something',
                    'plugin': 'copy',
                    'files': ['file1', 'file2'],
                },
            }
        }
        main(['--debug', '--index',
              fixture.fake_parts_wiki_with_slashes_fixture.url])
        self.assertEqual(2, _get_part_list_count())

        part1 = _get_part('curl/a')
        self.assertTrue(part1)

        part2 = _get_part('curl-a')
        self.assertTrue(part2)

    def test_missing_snapcraft_yaml(self):

        fixture = fixture_setup.FakePartsWikiOrigin()
        self.useFixture(fixture)
        origin_url = fixture.fake_parts_wiki_origin_fixture.url

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: {origin_url}
description: example
parts: [somepart]
""".format(origin_url=origin_url))

        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

        self.assertTrue(
            'Invalid wiki entry'
            in fake_logger.output, 'Missing invalid wiki entry info in output')

    def test_missing_snapcraft_yaml_without_debug(self):

        fixture = fixture_setup.FakePartsWikiOrigin()
        self.useFixture(fixture)
        origin_url = fixture.fake_parts_wiki_origin_fixture.url

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: {origin_url}
description: example
parts: [somepart]
""".format(origin_url=origin_url))

        main(['--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

    def test_wiki_with_fake_origin_with_bad_snapcraft_yaml(self):

        fixture = fixture_setup.FakePartsWikiOrigin()
        self.useFixture(fixture)
        origin_url = fixture.fake_parts_wiki_origin_fixture.url

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: {origin_url}
description: example
parts: [somepart]
""".format(origin_url=origin_url))

        origin_dir = os.path.join(parser._get_base_dir(),
                                  _encode_origin(origin_url))
        os.makedirs(origin_dir, exist_ok=True)

        # Create a fake snapcraft.yaml for _get_origin_data() to parse
        with open(os.path.join(origin_dir, 'snapcraft.yaml'),
                  'w') as fp:
            fp.write("bad yaml is : bad :yaml:::")

        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

        self.assertTrue(
            'Invalid wiki entry'
            in fake_logger.output, 'Missing invalid wiki entry info in output')

    def test_wiki_with_fake_origin(self):

        fixture = fixture_setup.FakePartsWikiOrigin()
        self.useFixture(fixture)
        origin_url = fixture.fake_parts_wiki_origin_fixture.url

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: {origin_url}
description: example
parts: [somepart]
""".format(origin_url=origin_url))

        origin_dir = os.path.join(parser._get_base_dir(),
                                  _encode_origin(origin_url))
        os.makedirs(origin_dir, exist_ok=True)

        # Create a fake snapcraft.yaml for _get_origin_data() to parse
        with open(os.path.join(origin_dir, 'snapcraft.yaml'),
                  'w') as fp:
            text = requests.get(origin_url).text
            fp.write(text)

        main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())
        part = _get_part('somepart')
        self.assertTrue(part)

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_carriage_returns(self, mock_get_origin_data):
        """Test carriage returns in the wiki."""

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        _create_example_output("""\r
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main\r
parts: [main]\r
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main duplicate
parts: [main]
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

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_duplicate_entries(self, mock_get_origin_data):
        """Test duplicate parts are ignored."""

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main
parts: [main]
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main duplicate
parts: [main]
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

        self.assertTrue(
            'Duplicate part found in the wiki: main'
            in fake_logger.output, 'Missing duplicate part info in output')

    @mock.patch('snapcraft.internal.parser._get_origin_data')
    def test_parsed_output_matches_wiki_order(self, mock_get_origin_data):
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main
parts: [main]
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main2
parts: [main2]
---
maintainer: Jim Doe <jim.doe@example.com>
origin: lp:snapcraft-parser-example
description: example main2
parts: [app1]
""")
        parts = OrderedDict()

        parts_main = OrderedDict()
        parts_main['description'] = 'example main'
        parts_main['files'] = ['file1', 'file2']
        parts_main['maintainer'] = 'John Doe <john.doe@example.com>'
        parts_main['plugin'] = 'copy'
        parts_main['source'] = 'lp:project'
        parts['main'] = parts_main

        parts_main2 = OrderedDict()
        parts_main2['description'] = 'example main2'
        parts_main2['files'] = ['file1', 'file2']
        parts_main2['maintainer'] = 'Jim Doe <jim.doe@example.com>'
        parts_main2['plugin'] = 'copy'
        parts_main2['source'] = 'lp:project'
        parts['main2'] = parts_main2

        parts_app1 = OrderedDict()
        parts_app1['description'] = 'example main2'
        parts_app1['files'] = ['file1', 'file2']
        parts_app1['maintainer'] = 'Jim Doe <jim.doe@example.com>'
        parts_app1['plugin'] = 'copy'
        parts_app1['source'] = 'lp:project'
        parts['app1'] = parts_app1

        mock_get_origin_data.return_value = {
            'parts': parts,
        }
        main(['--index', TEST_OUTPUT_PATH])
        self.assertEqual(3, _get_part_list_count())

        self.assertEqual(parts,
                         _get_part_list())

    def test__get_origin_data_both(self):
        with open(os.path.join(self.tempdir_path,
                  '.snapcraft.yaml'), 'w') as fp:
            fp.write("")
        with open(os.path.join(self.tempdir_path,
                  'snapcraft.yaml'), 'w') as fp:
            fp.write("")

        self.assertRaises(BadSnapcraftYAMLError, _get_origin_data,
                          self.tempdir_path)

    def test__get_origin_data_hidden_only(self):
        with open(os.path.join(self.tempdir_path,
                  '.snapcraft.yaml'), 'w') as fp:
            fp.write("")

        _get_origin_data(self.tempdir_path)

    def test__get_origin_data_normal_only(self):
        with open(os.path.join(self.tempdir_path,
                  'snapcraft.yaml'), 'w') as fp:
            fp.write("")

        _get_origin_data(self.tempdir_path)

    def test__encode_origin_git(self):
        origin = 'git@github.com:testuser/testproject.git'
        origin_dir = _encode_origin(origin)

        self.assertEqual('gitgithub.comtestusertestproject.git', origin_dir)

    def test__encode_origin_lp(self):
        origin = 'lp:~testuser/testproject/testbranch'
        origin_dir = _encode_origin(origin)

        self.assertEqual('lptestusertestprojecttestbranch', origin_dir)


class MissingAssetsTestCase(TestCase):

    @mock.patch('snapcraft.internal.sources.Local.pull')
    @mock.patch('snapcraft.internal.sources._get_source_type_from_uri')
    def test_filenotfound_for_non_repos(self, mock_type, mock_pull):
        mock_pull.side_effect = FileNotFoundError()
        mock_type.return_value = None
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:not-a-real-snapcraft-parser-example
description: example main
parts: [main]
""")
        with self.assertRaises(FileNotFoundError):
            main(['--debug', '--index', TEST_OUTPUT_PATH])

    @mock.patch('snapcraft.internal.sources.Bazaar.__init__')
    def test_missing_packages(self, mock_init):
        mock_init.side_effect = MissingCommandError('bzr')
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com>
origin: lp:not-a-real-snapcraft-parser-example
description: example main
parts: [main]
---
maintainer: John Doe <john.doe@example.com>
origin: lp:not-a-real-snapcraft-parser-example
description: example main
parts: [main2]
""")
        retval = main(['--debug', '--index', TEST_OUTPUT_PATH])
        self.assertEqual(2, retval)

        self.assertTrue(
            'One or more required commands are missing, please install'
            in fake_logger.output, 'No missing package info in output')
