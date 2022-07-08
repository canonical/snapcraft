# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021-2022 Canonical Ltd.
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

"""Build environment provider support for snapcraft."""

import pathlib
import tempfile

from craft_cli import emit
from craft_providers import Executor

from snapcraft.utils import get_managed_environment_log_path


def capture_logs_from_instance(instance: Executor) -> None:
    """Retrieve logs from instance.

    :param instance: Instance to retrieve logs from.

    :returns: String of logs.
    """
    # Get a temporary file path.
    with tempfile.NamedTemporaryFile(delete=False, prefix="snapcraft-") as tmp_file:
        local_log_path = pathlib.Path(tmp_file.name)

    instance_log_path = get_managed_environment_log_path()

    try:
        instance.pull_file(source=instance_log_path, destination=local_log_path)
    except FileNotFoundError:
        emit.debug("No logs found in instance.")
        return

    emit.debug("Logs captured from managed instance:")
    with local_log_path.open("rt", encoding="utf8") as logfile:
        for line in logfile:
            emit.debug(":: " + line.rstrip())
    local_log_path.unlink()
