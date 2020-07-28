#!/usr/bin/env python3

import pytest

from snapcraft.internal import states


class Project:
    def __init__(self):
        self.deb_arch = "amd64"


_DEFAULT = (Project(), ["foo"], {"foo": "bar"})
_VARIANTS = [
    (Project(), [], {"foo": "bar"}),
    (Project(), ["foo"], None),
    (None, ["foo"], {"foo": "bar"}),
]


@pytest.fixture
def pull_state():
    """Return a PullState."""
    project, property_names, part_properties = _DEFAULT

    return states.PullState(property_names, part_properties, project)


@pytest.fixture(params=_VARIANTS)
def pull_state_variant(request):
    """Return variants of pull_state."""
    project, property_names, part_properties = request.param

    return states.PullState(property_names, part_properties, project)


@pytest.fixture
def build_state():
    """Return a BuildState."""
    project, property_names, part_properties = _DEFAULT

    return states.BuildState(property_names, part_properties, project)


@pytest.fixture(params=_VARIANTS)
def build_state_variant(request):
    """Return variants of build_state."""
    project, property_names, part_properties = request.param

    return states.BuildState(property_names, part_properties, project)


_STAGE_DEFAULT = (
    Project(),
    {"foo"},
    {"bar"},
    {
        "filesets": {"qux": "quux"},
        "override-stage": "touch override-stage",
        "stage": ["baz"],
    },
)
_STAGE_VARIANTS = [
    (
        Project(),
        set(),
        {"bar"},
        {
            "filesets": {"qux": "quux"},
            "override-stage": "touch override-stage",
            "stage": ["baz"],
        },
    ),
    (
        Project(),
        {"foo"},
        set(),
        {
            "filesets": {"qux": "quux"},
            "override-stage": "touch override-stage",
            "stage": ["baz"],
        },
    ),
    (Project(), {"foo"}, {"bar"}, None),
]


@pytest.fixture
def stage_state():
    """Return a StageState."""
    project, files, directories, part_properties = _STAGE_DEFAULT

    return states.StageState(files, directories, part_properties, project)


@pytest.fixture(params=_STAGE_VARIANTS)
def stage_state_variant(request):
    """Return variants of stage_state."""
    project, files, directories, part_properties = request.param

    return states.StageState(files, directories, part_properties, project)


_PRIME_DEFAULT = (
    Project(),
    {"foo"},
    {"bar"},
    {"baz"},
    {"override-prime": "touch override-prime", "prime": ["qux"]},
)
_PRIME_VARIANTS = [
    (
        Project(),
        set(),
        {"bar"},
        {"baz"},
        {"override-prime": "touch override-prime", "prime": ["qux"]},
    ),
    (
        Project(),
        {"foo"},
        {"bar"},
        set(),
        {"override-prime": "touch override-prime", "prime": ["qux"]},
    ),
    (Project(), {"foo"}, {"bar"}, {"baz"}, None),
]


@pytest.fixture
def prime_state():
    """Return a PrimeState."""
    project, files, directories, dependency_paths, part_properties = _PRIME_DEFAULT

    return states.PrimeState(
        files, directories, dependency_paths, part_properties, project
    )


@pytest.fixture(params=_PRIME_VARIANTS)
def prime_state_variant(request):
    """Return variants of prime_state."""
    project, files, directories, dependency_paths, part_properties = request.param

    return states.PrimeState(
        files, directories, dependency_paths, part_properties, project
    )
