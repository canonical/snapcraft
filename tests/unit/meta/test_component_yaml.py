# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

import pytest

from snapcraft.errors import SnapcraftError
from snapcraft.meta import component_yaml
from snapcraft.meta.component_yaml import ComponentMetadata
from snapcraft.projects import Project


@pytest.fixture
def stub_project_data():
    return {
        "name": "mytest",
        "version": "1.29.3",
        "base": "core22",
        "summary": "Single-line elevator pitch for your amazing snap",
        "description": "test-description",
        "confinement": "strict",
        "parts": {
            "part1": {
                "plugin": "nil",
            },
        },
        "apps": {
            "app1": {
                "command": "bin/mytest",
            },
        },
        "components": {
            "component-a": {
                "type": "test",
                "summary": "test summary",
                "description": "test description",
                "version": "1.0",
            },
        },
    }


def test_unmarshal_component():
    """Unmarshal a dictionary containing a component."""
    component_data = {
        "component": "mytest+component-a",
        "type": "test",
        "version": "1.0",
        "summary": "test summary",
        "description": "test description",
    }

    component = ComponentMetadata.unmarshal(component_data)

    assert component.component == "mytest+component-a"
    assert component.type == "test"
    assert component.version == "1.0"
    assert component.summary == "test summary"
    assert component.description == "test description"


def test_write_component_yaml(stub_project_data, new_dir):
    """Write a component.yaml file from a project."""
    project = Project.unmarshal(stub_project_data)
    yaml_file = Path("meta/component.yaml")

    component_yaml.write(
        project, component_name="component-a", component_prime_dir=new_dir
    )

    assert yaml_file.is_file()
    assert yaml_file.read_text() == textwrap.dedent(
        """\
        component: mytest+component-a
        type: test
        version: '1.0'
        summary: test summary
        description: test description
        """
    )


def test_write_component_no_components(stub_project_data, new_dir):
    """Raise an error if no components are defined."""
    stub_project_data.pop("components")
    project = Project.unmarshal(stub_project_data)

    with pytest.raises(SnapcraftError) as raised:
        component_yaml.write(
            project, component_name="component-a", component_prime_dir=new_dir
        )

    assert str(raised.value) == "Project does not contain any components."


def test_write_component_non_existent(stub_project_data, new_dir):
    """Raise an error if the component does not exist."""
    project = Project.unmarshal(stub_project_data)

    with pytest.raises(SnapcraftError) as raised:
        component_yaml.write(project, component_name="bad", component_prime_dir=new_dir)

    assert str(raised.value) == "Component does not exist."
