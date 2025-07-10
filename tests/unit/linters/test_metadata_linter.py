from pathlib import Path

import pytest

from snapcraft import linters, models
from snapcraft.linters.base import LinterIssue, LinterResult
from snapcraft.meta import snap_yaml


@pytest.fixture
def snap_yaml_data():
    def _snap_yaml_data(**kwargs):
        return {
            "name": "mytest",
            "adopt-info": "mytest",
            "base": "core22",
            "version": "1.29.3",
            "summary": "Single-line elevator pitch for your amazing snap",
            "description": "test-description",
            "confinement": "strict",
            "icon": "icon.png",
            "contact": "Hello@h.com",
            "donation": {
                "https://donate1.com",
            },
            "parts": {},
            "apps": {
                "app1": {
                    "command": "bin/mytest",
                    "common_id": "com.example.mytest",
                },
            },
            **kwargs,
        }

    return _snap_yaml_data


def test_metadata_linter_missing_metadata(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data()
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    assert issues == [
        LinterIssue(
            name="metadata",
            result=LinterResult.WARNING,
            text="Metadata field 'title' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#title",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.WARNING,
            text="Metadata field 'license' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#license",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'issues' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#issues",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'source-code' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#source-code",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'website' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#website",
        ),
    ]


def test_metadata_linter_ignore_all_issues(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data()
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=models.Lint(ignore=["metadata"]))
    assert len(issues) == 0


def test_metadata_linter_ignore_some_issues(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data()
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(
        new_dir, lint=models.Lint(ignore=[{"metadata": ["title"]}])
    )
    assert issues == [
        LinterIssue(
            name="metadata",
            result=LinterResult.IGNORED,
            text="Metadata field 'title' is ignored.",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.WARNING,
            text="Metadata field 'license' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#license",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'issues' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#issues",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'source-code' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#source-code",
        ),
        LinterIssue(
            name="metadata",
            result=LinterResult.INFO,
            text="Metadata field 'website' is missing or empty.",
            url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#website",
        ),
    ]


def test_metadata_linter_ignore_devel_grade(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data(**{"grade": "devel"})
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)
    assert len(issues) == 0


def test_metadata_linter_common_id(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data(**{"apps": {"app1": {"command": "bin/mytest"}}})
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)
    assert issues[2] == LinterIssue(
        name="metadata",
        result=LinterResult.WARNING,
        text="Metadata field 'common-id' for app 'app1' is missing or empty.",
        url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.<app-name>.common-id",
    )


def test_metadata_linter_common_id_with_multiple_app(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data(
        **{
            "apps": {
                "app1": {"command": "bin/mytest", "common_id": "com.example.mytest"},
                "app2": {"command": "bin/mytest"},
            }
        }
    )
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)
    assert issues[2] == LinterIssue(
        name="metadata",
        result=LinterResult.WARNING,
        text="Metadata field 'common-id' for app 'app2' is missing or empty.",
        url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.<app-name>.common-id",
    )


def test_metadata_linter_empty_common_id_in_multiple_app(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data(
        **{
            "apps": {
                "app1": {"command": "bin/mytest"},
                "app2": {"command": "bin/mytest"},
            }
        }
    )
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)
    assert issues[2] == LinterIssue(
        name="metadata",
        result=LinterResult.WARNING,
        text="Metadata field 'common-id' for app 'app1' is missing or empty.",
        url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.<app-name>.common-id",
    )
    assert issues[3] == LinterIssue(
        name="metadata",
        result=LinterResult.WARNING,
        text="Metadata field 'common-id' for app 'app2' is missing or empty.",
        url="https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/#apps.<app-name>.common-id",
    )


def test_metadata_linter_ignore_common_id_when_not_app(new_dir, snap_yaml_data):
    yaml_data = snap_yaml_data(
        **{"apps": {"app2": {"command": "bin/mytest"}}, "type": "gadget"}
    )
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)
    assert len(issues) == 5
