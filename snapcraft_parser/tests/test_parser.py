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
import yaml

from snapcraft_parser.main import (
    _get_namespaced_partname,
    PART_NAMESPACE_SEP,
    PARTS_FILE,
    main,
)

from snapcraft.tests import TestCase


TEST_OUTPUT_PATH=os.path.join(os.getcwd(), "test_output.wiki")

def _create_example_output(output):
    with open(TEST_OUTPUT_PATH, "w") as fp:
        fp.write(output)

def _get_part_list(path=PARTS_FILE):
    input = ""
    with open(path, "r") as fp:
        input = fp.read()

    return yaml.load(input)

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
        partname = "part"
        subpart = "subpart"

        result = _get_namespaced_partname(partname, subpart)
        logging.warn("JOE: result: %s", result)

        self.assertEqual("{p}{s}{sp}".format(p=partname,
                                             s=PART_NAMESPACE_SEP,
                                             sp=subpart),
                         result)

    @mock.patch('snapcraft_parser.main._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_main_nested_parts_valid(self, mock_get, mock_get_origin_data):
        """ Ensure that we fail if there are dependent parts that
        are not included in the wiki's "parts" section."""

        mock_get_origin_data.return_value = {
            "parts": {
                "main": {
                    "source": "lp:something",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                    "after": ["part1"],
                },
                "part1": {
                    "source": "lp:somethingelse1",
                    "plugin": "copy",
                    "files": ["subfile1"],
                    "after": ["part2"],
                },
                "part2": {
                    "source": "lp:somethingelse2",
                    "plugin": "copy",
                    "files": ["subfile2"],
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
        main(["--debug", "--index", TEST_OUTPUT_PATH])
        self.assertEqual(3, _get_part_list_count())

    @mock.patch('snapcraft_parser.main._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_main_nested_parts_invalid(self, mock_get, mock_get_origin_data):
        """ Ensure that we fail if there are dependent parts that
        are not included in the wiki's "parts" section."""

        mock_get_origin_data.return_value = {
            "parts": {
                "main": {
                    "source": "lp:something",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                    "after": ["part1"],
                },
                "part1": {
                    "source": "lp:somethingelse1",
                    "plugin": "copy",
                    "files": ["subfile1"],
                    "after": ["part2"],
                },
                "part2": {
                    "source": "lp:somethingelse2",
                    "plugin": "copy",
                    "files": ["subfile2"],
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
        main(["--debug", "--index", TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

    @mock.patch('snapcraft_parser.main._get_origin_data')
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
            "parts": {
                "main": {
                    "source": "lp:something",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])
        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft_parser.main._get_origin_data')
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
            "parts": {
                "main": {
                    "source": "lp:something",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                    "after": ["part1"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])
        self.assertEqual(0, _get_part_list_count())

    @mock.patch('snapcraft_parser.main._get_origin_data')
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
            "parts": {
                "main": {
                    "source": "lp:something",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())

    @mock.patch('snapcraft_parser.main._get_origin_data')
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
            "parts": {
                "main": {
                    "source": "lp:something",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                    "after": ["subpart"],
                },
                "subpart": {
                    "source": "lp:somethingelse",
                    "plugin": "copy",
                    "files": ["subfile2"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])

        self.assertEqual(2, _get_part_list_count())

    @mock.patch('snapcraft_parser.main._get_origin_data')
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
parts: []
""")
        mock_get_origin_data.return_value = {
            "parts": {
                "main": {
                    "source": "lp:something",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
            }
        }

        filename = "parts.yaml"

        main(["--debug", "--index", TEST_OUTPUT_PATH, "--output", filename])

        self.assertEqual(1, _get_part_list_count(filename))
        self.assertTrue(os.path.exists(filename))

    @mock.patch('snapcraft_parser.main._get_origin_data')
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
            "parts": {
                "main": {
                    "source": ".",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())
        part = _get_part("main")
        self.assertNotEqual(".", part["source"])
        self.assertEqual(3, len(part.keys()))

    @mock.patch('snapcraft_parser.main._get_origin_data')
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
            "parts": {
                "main": {
                    "source": "local",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())
        part = _get_part("main")
        self.assertNotEqual("local", part["source"])
        self.assertEqual("local", part["source-subdir"])
        self.assertEqual(4, len(part.keys()))

    @mock.patch('snapcraft_parser.main._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_source_with_local_source_subdir_part_origin(self,
                                                  mock_get,
                                                  mock_get_origin_data):
        """Test a wiki entry with a source with a local source-subdir part."""
        _create_example_output("""
---
maintainer: John Doe <john.doe@example.com
origin: lp:snapcraft-parser-example
description: example
project-part: main
""")
        mock_get_origin_data.return_value = {
            "parts": {
                "main": {
                    "source": ".",
                    "source-subdir": "local",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])

        self.assertEqual(1, _get_part_list_count())
        part = _get_part("main")
        self.assertNotEqual(".", part["source"])
        self.assertEqual("local", part["source-subdir"])
        self.assertEqual(4, len(part.keys()))

    @mock.patch('snapcraft_parser.main._get_origin_data')
    @mock.patch('snapcraft.internal.sources.get')
    def test_source_with_local_source_subdir_part_origin(self,
                                                  mock_get,
                                                  mock_get_origin_data):
        """Test a wiki entry with a source with a local source-subdir part."""
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
            "parts": {
                "main": {
                    "source": "lp:project",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
                "main2": {
                    "source": "lp:project",
                    "plugin": "copy",
                    "files": ["file1", "file2"],
                },
            }
        }
        main(["--debug", "--index", TEST_OUTPUT_PATH])

        self.assertEqual(2, _get_part_list_count())
