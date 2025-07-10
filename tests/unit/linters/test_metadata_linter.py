from pathlib import Path

from snapcraft import linters, models
from snapcraft.linters.base import LinterIssue, LinterResult
from snapcraft.meta import snap_yaml

yaml_data = {
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
        }
    },
}


def test_metadata_linter_missing_metadata(new_dir):
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


def test_metadata_linter_ignore_all_issues(new_dir):
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=models.Lint(ignore=["metadata"]))
    assert len(issues) == 0


def test_metadata_linter_ignore_some_issues(new_dir):
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(
        new_dir, lint=models.Lint(ignore=[{"metadata": ["title"]}])
    )
    assert len(issues) == 5
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


def test_metadata_linter_ignore_devel_grade(new_dir):
    yaml_data["grade"] = "devel"
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(
        new_dir, lint=models.Lint(ignore=[{"metadata": ["title"]}])
    )
    assert len(issues) == 0
