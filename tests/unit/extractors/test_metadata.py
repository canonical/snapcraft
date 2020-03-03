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

from testtools.matchers import Equals, Not

from snapcraft.extractors._metadata import ExtractedMetadata
from tests import unit


class ExtractedMetadataTestCase(unit.TestCase):
    def test_init(self):
        metadata = ExtractedMetadata(summary="summary")

        self.assertThat(metadata.get_summary(), Equals("summary"))
        self.assertThat(metadata.get_description(), Equals(None))

    def test_update_merge(self):
        metadata = ExtractedMetadata(summary="summary")
        metadata2 = ExtractedMetadata(description="description")
        metadata.update(metadata2)

        self.assertThat(metadata.get_summary(), Equals("summary"))
        self.assertThat(metadata.get_description(), Equals("description"))

    def test_update_overwrite(self):
        metadata = ExtractedMetadata(summary="summary", description="description")
        metadata2 = ExtractedMetadata(description="new description")
        metadata.update(metadata2)

        self.assertThat(metadata.get_summary(), Equals("summary"))
        self.assertThat(metadata.get_description(), Equals("new description"))

    def test_overlap(self):
        metadata = ExtractedMetadata(summary="summary", description="description")
        metadata2 = ExtractedMetadata(description="new description")

        self.assertThat(metadata.overlap(metadata2), Equals({"description"}))

    def test_eq(self):
        metadata1 = ExtractedMetadata(summary="summary")
        metadata2 = ExtractedMetadata(summary="summary")
        self.assertThat(metadata1, Equals(metadata2))

    def test_empty_eq(self):
        self.assertThat(ExtractedMetadata(), Equals(ExtractedMetadata()))

    def test_not_eq(self):
        metadata1 = ExtractedMetadata(summary="summary")
        metadata2 = ExtractedMetadata(description="description")
        self.assertThat(metadata1, Not(Equals(metadata2)))

    def test_len(self):
        metadata = ExtractedMetadata(version="version")
        self.assertThat(len(metadata), Equals(1))

        metadata = ExtractedMetadata(summary="summary", version="version")
        self.assertThat(len(metadata), Equals(2))

    def test_to_dict_partial(self):
        metadata = ExtractedMetadata(summary="summary")
        self.assertThat(metadata.to_dict(), Equals({"summary": "summary"}))

    def test_to_dict_complete(self):
        metadata = ExtractedMetadata(summary="summary", description="description")
        self.assertThat(
            metadata.to_dict(),
            Equals({"summary": "summary", "description": "description"}),
        )

    def test_to_dict_is_a_copy(self):
        metadata = ExtractedMetadata(summary="summary")
        metadata_dict = metadata.to_dict()
        metadata_dict["summary"] = "edited summary"

        # Ensure the metadata cannot be edited with its dict
        self.assertThat(metadata.get_summary(), Equals("summary"))


class TestExtractedMetadataGetters:

    scenarios = [
        ("common_id", {"prop": "common_id", "value": "test-value"}),
        ("summary", {"prop": "summary", "value": "test-value"}),
        ("description", {"prop": "description", "value": "test-value"}),
        ("version", {"prop": "version", "value": "test-value"}),
        ("grade", {"prop": "grade", "value": "test-value"}),
        ("icon", {"prop": "icon", "value": "test-value"}),
        ("desktop_file_paths", {"prop": "desktop_file_paths", "value": ["test-value"]}),
    ]

    properties = (
        "common_id",
        "summary",
        "description",
        "version",
        "grade",
        "icon",
        "desktop_file_paths",
    )

    def test_getters(self, tmp_work_path, prop, value):
        metadata = ExtractedMetadata(**{prop: value})
        for p in self.properties:
            gotten = getattr(metadata, "get_{}".format(p))()
            if p == prop:
                assert gotten == value
            else:
                assert gotten is None
