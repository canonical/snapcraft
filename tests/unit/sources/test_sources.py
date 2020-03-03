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

import pytest

from snapcraft.internal import sources


class TestUri:

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

    def test(self, source, result):
        assert sources._get_source_type_from_uri(source) == result


class TestSourceWithBranchErrors:

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

    def test(self, source_type, source_branch, source_tag, source_commit, error):
        handler = sources.get_source_handler(
            "https://source.com", source_type=source_type
        )

        with pytest.raises(sources.errors.SnapcraftSourceInvalidOptionError) as err:
            handler(
                "https://source.com",
                source_dir=".",
                source_branch=source_branch,
                source_tag=source_tag,
                source_commit=source_commit,
            )

        assert err.value.source_type == source_type
        assert err.value.option == error


class TestSourceWithBranchAndTagErrors:

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

    def test(self, source_type, source_branch, source_tag):
        handler = sources.get_source_handler(
            "https://source.com", source_type=source_type
        )

        with pytest.raises(
            sources.errors.SnapcraftSourceIncompatibleOptionsError
        ) as error:
            handler(
                "https://source.com",
                source_dir=".",
                source_branch=source_branch,
                source_tag=source_tag,
            )

        assert error.value.source_type == source_type
        assert error.value.options == ["source-tag", "source-branch"]


def test_get(tmp_work_path):
    file_path = tmp_work_path / "file"
    file_path.touch()

    class Options:
        source = "."

    sources.get("src", "useless-arg", Options())

    src_dir = tmp_work_path / "src"
    assert src_dir.is_dir()
    assert (src_dir / "file").is_file()
