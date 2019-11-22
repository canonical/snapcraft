# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019 Canonical Ltd
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

from typing import Any, Dict, Optional

from snapcraft.internal.errors import SnapcraftException


class ReviewToolMissing(SnapcraftException):
    def get_brief(self) -> str:
        return "The Review Tools are not installed on this system."

    def get_resolution(self) -> str:
        return "Review Tools can be installed with:\nsnap install review-tools"


class ReviewError(SnapcraftException):
    def __init__(self, review_json: Dict[str, Any]) -> None:
        self._review_json = review_json

    def get_brief(self) -> str:
        return "Review Tools did not fully pass for this snap."

    def get_resolution(self) -> str:
        return (
            "Specific measures might need to be taken on the Snap Store before "
            "this snap can be fully accepted."
        )

    def _get_issues(self, issue_type: str) -> Dict[str, Any]:
        issues: Dict[str, Any] = dict()
        for severity in ("error", "warn"):
            issues.update(self._review_json[issue_type].get(severity))

        return issues

    def get_details(self) -> Optional[str]:
        all_issues = {
            "Linting Issues": self._get_issues("snap.v2_lint"),
            "Functional Issues": self._get_issues("snap.v2_functional"),
            "Security Issues": self._get_issues("snap.v2_security"),
        }

        # Reduce to actual issues.
        existing_issues = {k: v for k, v in all_issues.items() if v is not None}

        # Return early if there are no details.
        if not existing_issues:
            return None

        details = ""
        for issue_title, issues in sorted(existing_issues.items()):
            if not issues:
                continue
            details += f"{issue_title}:\n"
            for issue in issues.values():
                if "link" in issue:
                    details += "- {text} (Refer to {link})\n".format(**issue)
                else:
                    details += "- {text}\n".format(**issue)
            else:
                details += "\n"

        return details.strip()
