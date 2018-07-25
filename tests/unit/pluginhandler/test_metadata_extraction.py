# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
from testtools.matchers import Contains, Equals

from snapcraft.internal import errors
from snapcraft import extractors
from snapcraft.internal.pluginhandler import extract_metadata
from tests import fixture_setup, unit


class MetadataExtractionTestCase(unit.TestCase):
    def test_handled_file(self):
        open("test-metadata-file", "w").close()

        def _fake_extractor(file_path):
            return extractors.ExtractedMetadata(
                summary="test summary", description="test description"
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        metadata = extract_metadata("test-part", "test-metadata-file")
        self.assertThat(metadata.get_summary(), Equals("test summary"))
        self.assertThat(metadata.get_description(), Equals("test description"))

    def test_unhandled_file(self):
        open("unhandled-file", "w").close()

        def _fake_extractor(file_path):
            raise extractors.UnhandledFileError(file_path, "fake")

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        raised = self.assertRaises(
            errors.UnhandledMetadataFileTypeError,
            extract_metadata,
            "test-part",
            "unhandled-file",
        )

        self.assertThat(raised.path, Equals("unhandled-file"))

    def test_extractor_missing_extract_function(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(fake_logger)

        open("unhandled-file", "w").close()

        def _fake_extractor(file_path):
            raise extractors.UnhandledFileError(file_path, "fake")

        self.useFixture(
            fixture_setup.FakeMetadataExtractor("fake", _fake_extractor, "not_extract")
        )

        raised = self.assertRaises(
            errors.UnhandledMetadataFileTypeError,
            extract_metadata,
            "test-part",
            "unhandled-file",
        )

        self.assertThat(raised.path, Equals("unhandled-file"))
        self.assertThat(
            fake_logger.output,
            Contains("'fake' doesn't include the 'extract' function"),
        )

    def test_extractor_returning_invalid_things(self):
        open("unhandled-file", "w").close()

        def _fake_extractor(file_path):
            return "I'm not metadata!"

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        raised = self.assertRaises(
            errors.InvalidExtractorValueError,
            extract_metadata,
            "test-part",
            "unhandled-file",
        )

        self.assertThat(raised.path, Equals("unhandled-file"))
        self.assertThat(raised.extractor_name, Equals("fake"))

    def test_missing_file(self):
        raised = self.assertRaises(
            errors.MissingMetadataFileError,
            extract_metadata,
            "test-part",
            "non-existent",
        )

        self.assertThat(raised.path, Equals("non-existent"))
