# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2020 Canonical Ltd
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

import hashlib
import os
from datetime import datetime
from pathlib import Path
from typing import List, Set

from snapcraft_legacy.internal.deprecations import handle_deprecation_notice
from snapcraft_legacy.internal.meta.snap import Snap

from ._project_info import ProjectInfo  # noqa: F401
from ._project_options import ProjectOptions


class Project(ProjectOptions):
    """All details around building a project concerning the build environment
    and the snap being built."""

    def __init__(
        self,
        *,
        target_deb_arch=None,
        debug=False,
        snapcraft_yaml_file_path=None,
        work_dir: str = None,
        is_managed_host: bool = False
    ) -> None:

        project_dir = os.getcwd()
        if is_managed_host:
            work_dir = os.path.expanduser("~")
        else:
            work_dir = project_dir

        super().__init__(target_deb_arch, debug, work_dir=work_dir)

        # This here check is mostly for backwards compatibility with the
        # rest of the code base.
        if snapcraft_yaml_file_path is None:
            self.info: ProjectInfo = None  # type: ignore

        else:
            self.info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)

        self._is_managed_host = is_managed_host
        self._project_dir = project_dir
        self._work_dir = work_dir

        self.local_plugins_dir = self._get_local_plugins_dir()
        self._start_time = datetime.utcnow()

        # XXX: (Re)set by Config because it mangles source data.
        # Ideally everywhere wold converge to operating on snap_meta, and ww
        # would only need to initialize it once (properly).
        self._snap_meta = Snap()

    def _get_build_base(self) -> str:
        """
        Return name for type base or the base otherwise build-base is set
        """

        # In case snap_meta is not yet populated, lookup base in info.
        if self.info:
            if self.info.build_base:
                return self.info.build_base
            if self.info.base:
                return self.info.base

        return self._snap_meta.get_build_base()

    def _get_project_directory_hash(self) -> str:
        # This function uses md5 hashes because they are fast, and because
        # the hashes only need to be unique per project, so clashes are
        # tremendously unlikely and not a big deal even if they happen.
        hashes: List[str] = list()
        for root, dirs, files in os.walk(self._project_dir):
            # Sort contents, we need this to be stable so it's reproducible
            dirs.sort()
            files.sort()

            for filename in files:
                md5_hash = hashlib.md5()
                # Read files in chunks in case they are big
                with open(os.path.join(root, filename), "rb") as f:
                    for block in iter(lambda: f.read(4096), b""):
                        md5_hash.update(block)
                hashes.append(md5_hash.hexdigest())

        # Return final hash of hashes for the directory
        return hashlib.md5("".join(hashes).encode()).hexdigest()

    def _get_content_snaps(self) -> Set[str]:
        """Return the set of content snaps from snap_meta."""
        return set(
            [
                x.provider
                for x in self._snap_meta.get_content_plugs()
                if x.provider is not None
            ]
        )

    def _get_provider_content_dirs(self) -> Set[str]:
        """Return the set installed provider content directories."""
        return self._snap_meta.get_provider_content_directories()

    def _get_snapcraft_assets_dir(self) -> str:
        # Many test cases don't set the yaml file path and assume the default dir
        if not self.info:
            return os.path.join(self._project_dir, "snap")

        if self.info.snapcraft_yaml_file_path.endswith(
            os.path.join("build-aux", "snap", "snapcraft.yaml")
        ):
            return os.path.join(self._project_dir, "build-aux", "snap")
        else:
            return os.path.join(self._project_dir, "snap")

    def _get_keys_path(self) -> Path:
        # Directory containing <KEY_ID>.asc keys for use with
        # package-repositories, relative to 'snap' assets.
        return Path(self._get_snapcraft_assets_dir(), "keys")

    def _get_local_plugins_dir(self) -> str:
        deprecated_plugins_dir = os.path.join(self._parts_dir, "plugins")
        if os.path.exists(deprecated_plugins_dir):
            handle_deprecation_notice("dn2")
            return deprecated_plugins_dir
        else:
            assets_dir = self._get_snapcraft_assets_dir()
            return os.path.join(assets_dir, "plugins")

    def _get_global_state_file_path(self) -> str:
        if self._is_managed_host:
            state_file_path = os.path.join(self._work_dir, "state")
        else:
            state_file_path = os.path.join(self._parts_dir, ".snapcraft_global_state")

        return state_file_path

    def _get_stage_packages_target_arch(self) -> str:
        """Get architecture for staging packages.

        Prior to core20, staging packages has broken behavior in that it will
        stage native architecture packages by default.

        :return: The appropriate default architecture to stage.
        """
        if self._get_build_base() == "core18":
            return self.deb_arch
        else:
            return self.target_arch

    def _get_start_time(self) -> datetime:
        """Returns the timestamp for when a snapcraft project was loaded."""
        return self._start_time
