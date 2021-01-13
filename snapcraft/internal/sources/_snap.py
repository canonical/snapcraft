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

import os
import shutil
import tempfile

from snapcraft import file_utils, yaml_utils

from . import errors
from ._base import FileBase


class Snap(FileBase):
    """Handles downloading and extractions for a snap source.

    On provision, the meta directory is renamed to meta.<snap-name> and, if present,
    the same applies for the snap directory which shall be renamed to
    snap.<snap-name>.
    """

    def __init__(
        self,
        source: str,
        source_dir: str,
        source_tag: str = None,
        source_commit: str = None,
        source_branch: str = None,
        source_depth: str = None,
        source_checksum: str = None,
    ) -> None:
        super().__init__(
            source,
            source_dir,
            source_tag,
            source_commit,
            source_branch,
            source_depth,
            source_checksum,
            "unsquashfs",
        )
        if source_tag:
            raise errors.SnapcraftSourceInvalidOptionError("snap", "source-tag")
        elif source_commit:
            raise errors.SnapcraftSourceInvalidOptionError("snap", "source-commit")
        elif source_branch:
            raise errors.SnapcraftSourceInvalidOptionError("snap", "source-branch")

    def provision(
        self,
        dst: str,
        clean_target: bool = True,
        keep_snap: bool = False,
        src: str = None,
    ) -> None:
        """
        Provision the snap source to dst.

        :param str dst: the destination directory to provision to.
        :param bool clean_target: clean dst before provisioning if True.
        :param bool keep_snap: keep the snap after provisioning is done if
                               True.
        :param str src: force a new source to use for extraction.
        raises errors.InvalidSnapError: when trying to provision an invalid
                                        snap.
        """
        if src:
            snap_file = src
        else:
            snap_file = os.path.join(self.source_dir, os.path.basename(self.source))
        snap_file = os.path.realpath(snap_file)

        if clean_target:
            tmp_snap = tempfile.NamedTemporaryFile().name
            shutil.move(snap_file, tmp_snap)
            shutil.rmtree(dst)
            os.makedirs(dst)
            shutil.move(tmp_snap, snap_file)

        # unsquashfs [options] filesystem [directories or files to extract]
        # options:
        # -force: if file already exists then overwrite
        # -dest <pathname>: unsquash to <pathname>
        with tempfile.TemporaryDirectory(prefix=os.path.dirname(snap_file)) as temp_dir:
            extract_command = ["unsquashfs", "-force", "-dest", temp_dir, snap_file]
            self._run_output(extract_command)
            snap_name = _get_snap_name(temp_dir)
            # Rename meta and snap dirs from the snap
            rename_paths = (os.path.join(temp_dir, d) for d in ["meta", "snap"])
            rename_paths = (d for d in rename_paths if os.path.exists(d))
            for rename in rename_paths:
                shutil.move(rename, "{}.{}".format(rename, snap_name))
            file_utils.link_or_copy_tree(source_tree=temp_dir, destination_tree=dst)

        if not keep_snap:
            os.remove(snap_file)


def _get_snap_name(snap_dir: str) -> str:
    try:
        with open(os.path.join(snap_dir, "meta", "snap.yaml")) as snap_yaml:
            return yaml_utils.load(snap_yaml)["name"]
    except (FileNotFoundError, KeyError) as snap_error:
        raise errors.InvalidSnapError() from snap_error
