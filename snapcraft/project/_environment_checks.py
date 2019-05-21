# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from typing import List, Optional  # noqa: F401
from typing import Dict

from snapcraft import project
from snapcraft.internal import common, errors, os_release

_MSG_NO_BASE_FOR_SNAP = (
    "This snapcraft project does not specify the base keyword, "
    "explicitly setting the base keyword enables the latest snapcraft features."
)
_MSG_BASE_FOR_DEB = (
    "This version of snapcraft only has experimental support for the base keyword."
)
_MSG_UPGRADE = (
    "Upgrade snapraft https://docs.snapcraft.io/t/upgrading-snapcraft/11658 to take "
    "advantage of the latest features in snapcraft."
)
_MSG_HOST_MISMATCH_NO_BASE_TMPL = (
    "This project is best built on '{expected_release_name} {expected_release_id}', "
    "but is building on a '{current_release_name} {current_release_id}' host."
)
_MSG_HOST_MISMATCH_BASE_TMPL = (
    "This project targets the {base!r} base, best built on "
    "'{expected_release_name} {expected_release_id}', but is building on a "
    "'{current_release_name} {current_release_id}' host."
)
_MSG_BASES_INFO = (
    "Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"
)


class EnvironmentChecks:
    def __init__(self, project: project.Project) -> None:
        self._project = project
        self._messages = None  # type: Optional[List[str]]

    def get_messages(self) -> str:
        # Unit tests are not setting the base, so we check for that here.
        if self._project.info is None:
            return ""
        # If the message has been constructed, return it straight away.
        if self._messages is not None:
            return self._get_message_str()

        self._messages = list()
        self._check_base()
        self._check_upgrade()
        self._check_host()

        # Lastly, if there were any messages we add our bases notification
        # entry.
        if self._messages:
            self._messages.append(_MSG_BASES_INFO)

        return self._get_message_str()

    def _get_message_str(self) -> str:
        return "\n".join(self._messages)

    def _check_base(self) -> None:
        base = self._project.info.base
        if base is None and common.is_snap() and not common.is_docker_instance():
            self._messages.append(_MSG_NO_BASE_FOR_SNAP)
        elif base is not None and common.is_deb():
            self._messages.append(_MSG_BASE_FOR_DEB)

    def _check_upgrade(self) -> None:
        # Any setup run from the debian package and not created from docker
        # should get the upgrade message.
        if not common.is_docker_instance() and common.is_deb():
            self._messages.append(_MSG_UPGRADE)
        # Or if run from a docker instance and a base is set.
        elif common.is_docker_instance() and self._project.info.base is not None:
            self._messages.append(_MSG_UPGRADE)

    def _check_host(self) -> None:
        base = self._project.info.base
        # core is not really a base, so it will not be set (legacy).
        expected_release = self._project.get_expected_release_info_for_base(base)
        current_release = self._get_current_release_info()
        if current_release["codename"] != expected_release["codename"]:
            if base:
                self._messages.append(
                    _MSG_HOST_MISMATCH_BASE_TMPL.format(
                        base=base,
                        expected_release_name=expected_release["name"],
                        expected_release_id=expected_release["version_id"],
                        current_release_name=current_release["name"],
                        current_release_id=current_release["version_id"],
                    )
                )
            else:
                self._messages.append(
                    _MSG_HOST_MISMATCH_NO_BASE_TMPL.format(
                        expected_release_name=expected_release["name"],
                        expected_release_id=expected_release["version_id"],
                        current_release_name=current_release["name"],
                        current_release_id=current_release["version_id"],
                    )
                )

    def _get_current_release_info(self) -> Dict[str, Optional[str]]:
        current_release_info = dict()  # type: Dict[str, Optional[str]]
        current_release = os_release.OsRelease()
        try:
            current_release_info["codename"] = current_release.version_codename()
        except errors.OsReleaseCodenameError:
            current_release_info["codename"] = None
        try:
            current_release_info["name"] = current_release.name()
        except errors.OsReleaseNameError:
            current_release_info["name"] = "Unknown Name"
        try:
            current_release_info["version_id"] = current_release.version_id()
        except errors.OsReleaseVersionIdError:
            current_release_info["version_id"] = "Unknown Version"
        return current_release_info
