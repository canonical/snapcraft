# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import logging
import os
import time
from subprocess import Popen, PIPE, STDOUT

import yaml
from progressbar import AnimatedMarker, ProgressBar

from snapcraft import file_utils
from snapcraft.internal import common
from snapcraft.internal.indicators import is_dumb_terminal


logger = logging.getLogger(__name__)


def _snap_data_from_dir(directory):
    with open(os.path.join(directory, "meta", "snap.yaml")) as f:
        snap = yaml.safe_load(f)

    return {
        "name": snap["name"],
        "version": snap["version"],
        "arch": snap.get("architectures", []),
        "type": snap.get("type", ""),
    }


def pack(directory, output=None):
    mksquashfs_path = file_utils.get_tool_path("mksquashfs")

    snap = _snap_data_from_dir(directory)
    output_snap_name = output or common.format_snap_name(snap)

    # If a .snap-build exists at this point, when we are about to override
    # the snap blob, it is stale. We rename it so user have a chance to
    # recover accidentally lost assertions.
    snap_build = output_snap_name + "-build"
    if os.path.isfile(snap_build):
        _new = "{}.{}".format(snap_build, int(time.time()))
        logger.warning("Renaming stale build assertion to {}".format(_new))
        os.rename(snap_build, _new)

    _run_mksquashfs(
        mksquashfs_path,
        directory=directory,
        snap_name=snap["name"],
        snap_type=snap["type"],
        output_snap_name=output_snap_name,
    )

    return output_snap_name


def _run_mksquashfs(
    mksquashfs_command, *, directory, snap_name, snap_type, output_snap_name
):
    # These options need to match the review tools:
    # http://bazaar.launchpad.net/~click-reviewers/click-reviewers-tools/trunk/view/head:/clickreviews/common.py#L38
    mksquashfs_args = ["-noappend", "-comp", "xz", "-no-xattrs", "-no-fragments"]
    if snap_type not in ("os", "base"):
        mksquashfs_args.append("-all-root")

    complete_command = [
        mksquashfs_command,
        directory,
        output_snap_name,
    ] + mksquashfs_args

    with Popen(complete_command, stdout=PIPE, stderr=STDOUT) as proc:
        ret = None
        if is_dumb_terminal():
            logger.info("Snapping {!r} ...".format(snap_name))
            ret = proc.wait()
        else:
            message = "\033[0;32m\rSnapping {!r}\033[0;32m ".format(snap_name)
            progress_indicator = ProgressBar(
                widgets=[message, AnimatedMarker()], maxval=7
            )
            progress_indicator.start()

            ret = proc.poll()
            count = 0

            while ret is None:
                if count >= 7:
                    progress_indicator.start()
                    count = 0
                progress_indicator.update(count)
                count += 1
                time.sleep(.2)
                ret = proc.poll()
        print("")
        if ret != 0:
            logger.error(proc.stdout.read().decode("utf-8"))
            raise RuntimeError("Failed to create snap {!r}".format(output_snap_name))

        logger.debug(proc.stdout.read().decode("utf-8"))
