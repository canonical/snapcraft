# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit


class TestUri(unit.TestCase):

    scenarios = [
        ("tar.gz", dict(result="tar", source="https://golang.tar.gz")),
        ("tar.gz", dict(result="tar", source="https://golang.tar.xz")),
        ("tar.bz2", dict(result="tar", source="https://golang.tar.bz2")),
        ("tgz", dict(result="tar", source="https://golang.tgz")),
        ("tar", dict(result="tar", source="https://golang.tar")),
        ("git:", dict(result="git", source="git://github.com:snapcore/snapcraft.git")),
        ("git@", dict(result="git", source="git@github.com:snapcore/snapcraft.git")),
        (
            ".git",
            dict(result="git", source="https://github.com:snapcore/snapcraft.git"),
        ),
        ("lp:", dict(result="bzr", source="lp:snapcraft_test_source")),
        ("bzr:", dict(result="bzr", source="bzr:dummy_source")),
        ("svn:", dict(result="subversion", source="svn://sylpheed.jp/sylpheed/trunk")),
    ]

    def test_get_source_type_from_uri(self):
        self.assertThat(
            sources._get_source_type_from_uri(self.source), Equals(self.result)
        )


class SourceWithBranchTestCase(unit.TestCase):

    scenarios = [
        (
            "bzr with source branch",
            {
                "source_type": "bzr",
                "source_branch": "test_branch",
                "source_tag": None,
                "source_commit": None,
                "error": "source-branch",
            },
        ),
        (
            "tar with source branch",
            {
                "source_type": "tar",
                "source_branch": "test_branch",
                "source_tag": None,
                "source_commit": None,
                "error": "source-branch",
            },
        ),
        (
            "tar with source tag",
            {
                "source_type": "tar",
                "source_branch": None,
                "source_tag": "test_tag",
                "source_commit": None,
                "error": "source-tag",
            },
        ),
        (
            "tar with source commit",
            {
                "source_type": "tar",
                "source_branch": None,
                "source_tag": None,
                "source_commit": "commit",
                "error": "source-commit",
            },
        ),
        (
            "deb with source branch",
            {
                "source_type": "deb",
                "source_branch": "test_branch",
                "source_tag": None,
                "source_commit": None,
                "error": "source-branch",
            },
        ),
        (
            "deb with source tag",
            {
                "source_type": "deb",
                "source_branch": None,
                "source_tag": "test_tag",
                "source_commit": None,
                "error": "source-tag",
            },
        ),
        (
            "deb with source commit",
            {
                "source_type": "deb",
                "source_branch": None,
                "source_tag": None,
                "source_commit": "commit",
                "error": "source-commit",
            },
        ),
    ]

    def test_get_source_with_branch_must_raise_error(self):
        handler = sources.get_source_handler(
            "https://source.com", source_type=self.source_type
        )
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceInvalidOptionError,
            handler,
            "https://source.com",
            source_dir=".",
            source_branch=self.source_branch,
            source_tag=self.source_tag,
            source_commit=self.source_commit,
        )

        self.assertThat(raised.source_type, Equals(self.source_type))
        self.assertThat(raised.option, Equals(self.error))


class SourceWithBranchAndTagTestCase(unit.TestCase):

    scenarios = [
        (
            "git with source branch and tag",
            {"source_type": "git", "source_branch": "test_branch", "source_tag": "tag"},
        ),
        (
            "hg with source branch and tag",
            {
                "source_type": "mercurial",
                "source_branch": "test_branch",
                "source_tag": "tag",
            },
        ),
    ]

    def test_get_source_with_branch_and_tag_must_raise_error(self):
        handler = sources.get_source_handler(
            "https://source.com", source_type=self.source_type
        )
        raised = self.assertRaises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError,
            handler,
            "https://source.com",
            source_dir=".",
            source_branch=self.source_branch,
            source_tag=self.source_tag,
        )

        self.assertThat(raised.source_type, Equals(self.source_type))
        self.assertThat(raised.options, Equals(["source-tag", "source-branch"]))


class GetSourceTestClass(unit.TestCase):
    def test_get(self):
        open("file", "w").close()

        class Options:
            source = "."

        sources.get("src", "useless-arg", Options())

        self.assertTrue(os.path.isdir("src"))
        self.assertTrue(os.path.isfile(os.path.join("src", "file")))
