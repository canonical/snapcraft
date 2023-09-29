# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Remote-build utility tests."""

import re
from pathlib import Path

import pytest

from snapcraft.remote import get_build_id


@pytest.mark.usefixtures("new_dir")
def test_get_build_id():
    """Get the build id."""
    Path("test").write_text("Hello, World!", encoding="utf-8")

    build_id = get_build_id("test-app", "test-project", Path())

    assert re.match("test-app-test-project-[0-9a-f]{32}", build_id)


@pytest.mark.usefixtures("new_dir")
def test_get_build_id_empty_dir():
    """An empty directory should still produce a valid build id."""
    build_id = get_build_id("test-app", "test-project", Path())

    assert re.match("test-app-test-project-[0-9a-f]{32}", build_id)


@pytest.mark.usefixtures("new_dir")
def test_get_build_id_is_reproducible():
    """The build id should be the same when there are no changes to the directory."""
    Path("test").write_text("Hello, World!", encoding="utf-8")

    build_id_1 = get_build_id("test-app", "test-project", Path())
    build_id_2 = get_build_id("test-app", "test-project", Path())

    assert build_id_1 == build_id_2


@pytest.mark.usefixtures("new_dir")
def test_get_build_id_computed_is_unique_file_modified():
    """The build id should change when a file is modified."""
    Path("test1").write_text("Hello, World!", encoding="utf-8")
    build_id_1 = get_build_id("test-app", "test-project", Path())

    # adding a new file should change the build-id
    Path("test2").write_text("Hello, World!", encoding="utf-8")
    build_id_2 = get_build_id("test-app", "test-project", Path())

    assert build_id_1 != build_id_2


@pytest.mark.usefixtures("new_dir")
def test_get_build_id_computed_is_unique_file_contents_modified():
    """The build id should change when the contents of a file is modified."""
    Path("test").write_text("Hello, World!", encoding="utf-8")
    build_id_1 = get_build_id("test-app", "test-project", Path())

    # modifying the contents of a file should change the build-id
    Path("test").write_text("Goodbye, World!", encoding="utf-8")
    build_id_2 = get_build_id("test-app", "test-project", Path())

    assert build_id_1 != build_id_2


@pytest.mark.usefixtures("new_dir")
def test_get_build_id_directory_does_not_exist_error():
    """Raise an error if the directory does not exist."""
    with pytest.raises(FileNotFoundError) as raised:
        get_build_id("test-app", "test-project", Path("does-not-exist"))

    assert str(raised.value) == (
        "Could not compute hash because directory "
        f"{str(Path('does-not-exist').absolute())} does not exist."
    )


@pytest.mark.usefixtures("new_dir")
def test_get_build_id_directory_is_not_a_directory_error():
    """Raise an error if the directory is not a directory."""
    Path("regular-file").touch()

    with pytest.raises(FileNotFoundError) as raised:
        get_build_id("test-app", "test-project", Path("regular-file"))

    assert str(raised.value) == (
        f"Could not compute hash because {str(Path('regular-file').absolute())} "
        "is not a directory."
    )
