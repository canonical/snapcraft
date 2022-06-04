# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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
from pathlib import Path
from typing import Any, Dict

import pytest

from snapcraft.meta import ExtractedMetadata
from snapcraft.parts.update_metadata import update_project_metadata
from snapcraft.projects import App, Project


@pytest.fixture
def appstream_file(new_dir):
    content = textwrap.dedent(
        """
        <?xml version="1.0" encoding="utf-8"?>
        <component type="desktop">
          <id>io.snapcraft.snapcraft</id>
          <metadata_license>CC0-1.0</metadata_license>
          <project_license>GPL-3.0</project_license>
          <name>snapcraft</name>
          <name xml:lang="es">snapcraft</name>
          <summary>Create snaps</summary>
          <summary xml:lang="es">Crea snaps</summary>
          <description>
            <p>Command Line Utility to create snaps.</p>
            <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
            <p>Features:</p>
            <p xml:lang="es">Funciones:</p>
            <ol>
              <li>Build snaps.</li>
              <li xml:lang="es">Construye snaps.</li>
              <li>Publish snaps to the store.</li>
              <li xml:lang="es">Publica snaps en la tienda.</li>
            </ol>
          </description>
          <provides>
            <binary>snapcraft</binary>
          </provides>
        </component>
        """
    )
    yaml_path = Path("appstream.appdata.xml")
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(content)


@pytest.fixture
def project_yaml_data():
    def yaml_data(extra_args: Dict[str, Any]):
        return {
            "name": "name",
            "summary": "summary",
            "description": "description",
            "base": "core22",
            "grade": "stable",
            "confinement": "strict",
            "parts": {},
            **extra_args,
        }

    yield yaml_data


def _project_app(data: Dict[str, Any]) -> App:
    return App(**data)


def test_update_project_metadata(project_yaml_data, appstream_file, new_dir):
    project = Project.unmarshal(project_yaml_data({"adopt-info": "part"}))
    metadata = ExtractedMetadata(
        common_id="common.id",
        title="title",
        summary="summary",
        description="description",
        version="1.2.3",
        icon="assets/icon.png",
        desktop_file_paths=["assets/file.desktop"],
    )
    assets_dir = Path("assets")
    prime_dir = Path("prime")

    # set up project apps
    project.apps = {
        "app1": _project_app({"command": "bin/app1"}),
        "app2": _project_app({"command": "bin/app2", "common_id": "other.id"}),
        "app3": _project_app({"command": "bin/app3", "common_id": "common.id"}),
    }

    prime_dir.mkdir()
    (prime_dir / "assets").mkdir()
    (prime_dir / "assets/icon.png").touch()
    (prime_dir / "assets/file.desktop").touch()

    prj_vars = {"version": "0.1", "grade": "stable"}
    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata],
        assets_dir=assets_dir,
        prime_dir=prime_dir,
    )

    assert project.title == "title"
    assert project.summary == "summary"  # already set in project
    assert project.description == "description"  # already set in project
    assert project.version == "0.1"  # already set in project
    assert project.icon == "assets/icon.png"
    assert project.apps["app3"].desktop == "assets/file.desktop"


@pytest.mark.parametrize(
    "project_entries,expected",
    [
        (
            {
                "version": "1.2.3",
                "summary": "project summary",
                "description": "project description",
                "title": "project title",
                "grade": "stable",
            },
            {
                "version": "1.2.3",
                "summary": "project summary",
                "description": "project description",
                "title": "project title",
                "grade": "stable",
            },
        ),
        (
            {},
            {
                "version": "4.5.6",
                "summary": "metadata summary",
                "description": "metadata description",
                "title": "metadata title",
                "grade": "devel",
            },
        ),
    ],
)
def test_update_project_metadata_fields(
    appstream_file, project_entries, expected, new_dir
):
    yaml_data = {
        "name": "my-project",
        "base": "core22",
        "confinement": "strict",
        "adopt-info": "part",
        "parts": {},
        **project_entries,
    }
    project = Project(**yaml_data)
    metadata = ExtractedMetadata(
        version="4.5.6",
        summary="metadata summary",
        description="metadata description",
        title="metadata title",
        grade="devel",
    )
    prj_vars = {"version": "", "grade": ""}

    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata],
        assets_dir=new_dir,
        prime_dir=new_dir,
    )

    assert project.version == expected["version"]
    assert project.summary == expected["summary"]
    assert project.description == expected["description"]
    assert project.title == expected["title"]
    assert project.grade == expected["grade"]


@pytest.mark.parametrize(
    "project_entries,expected",
    [
        (
            {
                "version": "1.2.3",
                "summary": "project summary",
                "description": "project description",
                "title": "project title",
                "grade": "stable",
            },
            {
                "version": "1.2.3",
                "summary": "project summary",
                "description": "project description",
                "title": "project title",
                "grade": "stable",
            },
        ),
        (
            {},
            {
                "version": "4.5.6",
                "summary": "metadata summary",
                "description": "metadata description",
                "title": "metadata title",
                "grade": "devel",
            },
        ),
    ],
)
def test_update_project_metadata_multiple(
    appstream_file, project_entries, expected, new_dir
):
    yaml_data = {
        "name": "my-project",
        "base": "core22",
        "confinement": "strict",
        "adopt-info": "part",
        "parts": {},
        **project_entries,
    }
    project = Project(**yaml_data)
    metadata1 = ExtractedMetadata(version="4.5.6")
    metadata2 = ExtractedMetadata(
        summary="metadata summary", description="metadata description"
    )
    metadata3 = ExtractedMetadata(
        version="7.8.9", title="metadata title", grade="devel"
    )
    metadata4 = ExtractedMetadata(
        summary="extra summary", description="extra description"
    )
    prj_vars = {"version": "", "grade": ""}

    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata1, metadata2, metadata3, metadata4],
        assets_dir=new_dir,
        prime_dir=new_dir,
    )

    assert project.version == expected["version"]
    assert project.summary == expected["summary"]
    assert project.description == expected["description"]
    assert project.title == expected["title"]
    assert project.grade == expected["grade"]


@pytest.mark.parametrize(
    "project_entries,icon_exists,asset_exists,expected_icon",
    [
        ({"icon": "icon.png"}, True, True, "icon.png"),  # use project icon if defined
        (  # use project icon if defined even if already in assets
            {"icon": "icon.png"},
            True,
            False,
            "icon.png",
        ),
        (  # use metadata icon if not defined in project
            {},
            True,
            False,
            "metadata_icon.png",
        ),
        (
            # use metadata icon even if not downloaded yet
            {},
            False,
            False,
            "metadata_icon.png",
        ),
        ({}, True, True, None),  # don't use metadata if asset icon already exists
    ],
)
def test_update_project_metadata_icon(
    project_yaml_data,
    project_entries,
    icon_exists,
    asset_exists,
    expected_icon,
    new_dir,
):
    yaml_data = project_yaml_data(
        {"version": "1.0", "adopt-info": "part", "parts": {}, **project_entries}
    )
    project = Project(**yaml_data)
    metadata = ExtractedMetadata(icon="metadata_icon.png")

    # create icon file
    if icon_exists:
        Path("metadata_icon.png").touch()

    # create icon file in assets dir
    if asset_exists:
        Path("assets/gui").mkdir(parents=True)
        Path("assets/gui/icon.svg").touch()

    prj_vars = {"version": "", "grade": "stable"}

    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata],
        assets_dir=new_dir / "assets",
        prime_dir=new_dir,
    )

    assert project.icon == expected_icon


@pytest.mark.parametrize(
    "project_entries,desktop_exists,asset_exists,expected_desktop",
    [
        (  # use project desktop file if defined
            {
                "apps": {
                    "foo": {
                        "command": "foo",
                        "common-id": "test.id",
                        "desktop": "project/foo.desktop",
                    },
                },
            },
            True,
            False,
            "project/foo.desktop",
        ),
        (  # use project desktop if no common-id defined
            {
                "apps": {
                    "foo": {
                        "command": "foo",
                        "desktop": "project/foo.desktop",
                    },
                },
            },
            True,
            False,
            "project/foo.desktop",
        ),
        (  # don't read from metadata if common-id not defined
            {
                "apps": {
                    "foo": {
                        "command": "foo",
                    },
                },
            },
            True,
            False,
            None,
        ),
        (  # use metadata if no project definition and metadata icon exists
            {
                "apps": {
                    "foo": {
                        "command": "foo",
                        "common-id": "test.id",
                    },
                },
            },
            True,
            False,
            "metadata/foo.desktop",
        ),
        (  # only use metadata desktop file if it exists
            {
                "apps": {
                    "foo": {
                        "command": "foo",
                        "common-id": "test.id",
                    },
                },
            },
            False,
            False,
            None,
        ),
        (  # existing file has precedence over metadata
            {
                "apps": {
                    "foo": {
                        "command": "foo",
                        "common-id": "test.id",
                    },
                },
            },
            True,
            True,
            None,
        ),
    ],
)
def test_update_project_metadata_desktop(
    project_yaml_data,
    project_entries,
    desktop_exists,
    asset_exists,
    expected_desktop,
    new_dir,
):
    yaml_data = project_yaml_data(
        {"version": "1.0", "adopt-info": "part", "parts": {}, **project_entries}
    )
    project = Project(**yaml_data)
    metadata = ExtractedMetadata(
        common_id="test.id", desktop_file_paths=["metadata/foo.desktop"]
    )

    # create desktop file
    if desktop_exists:
        Path("metadata").mkdir()
        Path("metadata/foo.desktop").touch()

    # create desktop file in assets dir
    if asset_exists:
        Path("assets/gui").mkdir(parents=True)
        Path("assets/gui/foo.desktop").touch()

    prj_vars = {"version": "", "grade": "stable"}

    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata],
        assets_dir=new_dir / "assets",
        prime_dir=new_dir,
    )

    assert project.apps is not None
    assert project.apps["foo"].desktop == expected_desktop


def test_update_project_metadata_desktop_multiple(project_yaml_data, new_dir):
    yaml_data = project_yaml_data(
        {
            "version": "1.0",
            "adopt-info": "part",
            "parts": {},
            "apps": {
                "foo": {
                    "command": "foo",
                    "common-id": "test.id",
                },
            },
        }
    )
    project = Project(**yaml_data)
    metadata = ExtractedMetadata(
        common_id="test.id",
        desktop_file_paths=["metadata/foo.desktop", "metadata/bar.desktop"],
    )

    # create desktop files
    Path("metadata").mkdir()
    Path("metadata/foo.desktop").touch()
    Path("metadata/bar.desktop").touch()

    prj_vars = {"version": "", "grade": "stable"}

    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata],
        assets_dir=new_dir / "assets",
        prime_dir=new_dir,
    )

    assert project.apps is not None
    assert project.apps["foo"].desktop == "metadata/foo.desktop"


def test_update_project_metadata_multiple_apps(project_yaml_data, new_dir):
    yaml_data = project_yaml_data(
        {
            "version": "1.0",
            "adopt-info": "part",
            "parts": {},
            "apps": {
                "foo": {
                    "command": "foo",
                    "common-id": "foo.id",
                },
                "bar": {
                    "command": "bar",
                    "common-id": "bar.id",
                },
            },
        }
    )
    project = Project(**yaml_data)
    metadata1 = ExtractedMetadata(
        common_id="foo.id",
        desktop_file_paths=["metadata/foo.desktop"],
    )
    metadata2 = ExtractedMetadata(
        common_id="bar.id",
        desktop_file_paths=["metadata/bar.desktop"],
    )

    # create desktop files
    Path("metadata").mkdir()
    Path("metadata/foo.desktop").touch()
    Path("metadata/bar.desktop").touch()

    prj_vars = {"version": "", "grade": "stable"}

    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata1, metadata2],
        assets_dir=new_dir / "assets",
        prime_dir=new_dir,
    )

    assert project.apps is not None
    assert project.apps["foo"].desktop == "metadata/foo.desktop"
    assert project.apps["bar"].desktop == "metadata/bar.desktop"


def test_update_project_metadata_desktop_no_apps(project_yaml_data, new_dir):
    yaml_data = project_yaml_data(
        {
            "version": "1.0",
            "adopt-info": "part",
            "parts": {},
        }
    )
    project = Project(**yaml_data)
    metadata = ExtractedMetadata(
        common_id="test.id",
        desktop_file_paths=["metadata/foo.desktop", "metadata/bar.desktop"],
    )

    # create desktop file
    Path("metadata").mkdir()
    Path("metadata/foo.desktop").touch()
    Path("metadata/bar.desktop").touch()

    prj_vars = {"version": "", "grade": "stable"}

    update_project_metadata(
        project,
        project_vars=prj_vars,
        metadata_list=[metadata],
        assets_dir=new_dir / "assets",
        prime_dir=new_dir,
    )

    assert project.apps is None
