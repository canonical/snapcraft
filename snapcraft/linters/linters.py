# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

"""Snapcraft linting execution and reporting."""

import enum
import fnmatch
import json
import os
from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Type

from craft_cli import emit

from snapcraft import projects
from snapcraft.meta import snap_yaml

from .base import Linter, LinterIssue, LinterResult
from .classic_linter import ClassicLinter
from .library_linter import LibraryLinter

if TYPE_CHECKING:
    import argparse


LinterType = Type[Linter]


LINTERS: Dict[str, LinterType] = {
    "classic": ClassicLinter,
    "library": LibraryLinter,
}


@enum.unique
class LinterStatus(enum.IntEnum):
    """Status codes for the linter execution."""

    OK = 0
    FATAL = 1
    ERRORS = 2
    WARNINGS = 3


_lint_reports: Dict[LinterResult, str] = {
    LinterResult.OK: "Lint OK",
    LinterResult.WARNING: "Lint warnings",
    LinterResult.ERROR: "Lint errors",
}


def report(
    issues: List[LinterIssue], *, json_output: bool = False, intermediate: bool = False
) -> LinterStatus:
    """Display the linter report in textual or json formats.

    :param issues: The list of issues to display.
    :param json_output: Display issues in json format.
    :param intermediate: Set if the linter output are is not the main
        outcome of the command execution.
    """
    if intermediate:
        display = partial(emit.progress, permanent=True)
    else:
        display = emit.message

    status = LinterStatus.OK

    # split dictionary based on result
    issues_by_result: Dict[LinterResult, List[LinterIssue]] = {}
    for issue in issues:
        status = _update_status(status, issue.result)
        if status == LinterStatus.FATAL:
            break

        if issue.result in _lint_reports:
            issues_by_result.setdefault(issue.result, []).append(issue)

    if json_output:
        display(json.dumps([x.dict(exclude_none=True) for x in issues]))
    else:
        # show issues by result
        for result, header in _lint_reports.items():
            if issues_by_result.get(result):
                display(f"{header}:")
                for issue in issues_by_result[result]:
                    display(f"- {issue!s}")

    return status


def _update_status(status: LinterStatus, result: LinterResult) -> LinterStatus:
    """Compute the consolidated status based on individual linter results."""
    if result == LinterResult.FATAL:
        status = LinterStatus.FATAL
    elif result == LinterResult.ERROR and status != LinterStatus.FATAL:
        status = LinterStatus.ERRORS
    elif result == LinterResult.WARNING and status == LinterStatus.OK:
        status = LinterStatus.WARNINGS

    return status


def lint_command(parsed_args: "argparse.Namespace") -> None:
    """``snapcraft lint`` command handler."""
    # XXX: obtain lint configuration
    run_linters(parsed_args.snap_file, lint=None)


def run_linters(location: Path, *, lint: Optional[projects.Lint]) -> List[LinterIssue]:
    """Run all the defined linters.

    :param location: The root of the snap payload subtree to run linters on.
    :param lint: The linter configuration defined for this project.
    :return: A list of linter issues.
    """
    all_issues: List[LinterIssue] = []
    previous_dir = os.getcwd()
    try:
        os.chdir(location)

        emit.progress("Reading snap metadata...")
        snap_metadata = snap_yaml.read(Path())

        emit.progress("Running linters...")
        for name, linter_class in LINTERS.items():
            if lint and lint.all_ignored(name):
                continue

            categories = linter_class.get_categories()

            if lint and categories and all(lint.all_ignored(c) for c in categories):
                continue

            linter = linter_class(name=name, lint=lint, snap_metadata=snap_metadata)
            emit.progress(f"Running linter: {name}")
            issues = linter.run()
            all_issues += issues
    finally:
        os.chdir(previous_dir)

    _ignore_matching_filenames(all_issues, lint=lint)

    return all_issues


def _ignore_matching_filenames(
    issues: List[LinterIssue], *, lint: Optional[projects.Lint]
) -> None:
    """Mark any remaining filename match as ignored."""
    if lint is None:
        return

    for issue in issues:
        files = lint.ignored_files(issue.name)
        for pattern in files:
            if (
                issue.filename
                and issue.result != LinterResult.IGNORED
                and fnmatch.fnmatch(issue.filename, pattern)
            ):
                emit.verbose(
                    f"Ignore {issue.name!r} linter issue ({issue.filename!r} "
                    f"matches {pattern!r})"
                )
                issue.result = LinterResult.IGNORED
                break
