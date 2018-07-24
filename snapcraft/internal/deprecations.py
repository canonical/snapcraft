# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

"""Handle surfacing deprecation notices.

When a new deprecation has occurred, write a Deprecation Notice for it on the
wiki, assigning it the next ID (DN1, DN2, etc.). Then add that ID along with a
brief message for surfacing to the user into the _DEPRECATION_MESSAGES in this
module.
"""

import logging

_DEPRECATION_MESSAGES = {
    "dn1": "The 'snap' keyword has been replaced by 'prime'.",
    "dn2": "Custom plugins should now be placed in 'snap/plugins'.",
    "dn3": "Assets in 'setup/gui' should now be placed in 'snap/gui'.",
    "dn4": "The 'history' command has been replaced by 'list-revisions'.",
    "dn5": "Aliases are now handled by the store, and shouldn't be declared "
    "in the snap.",
    "dn6": "Use of the 'snap' command with a directory has been deprecated "
    "in favour of the 'pack' command.",
    "dn7": "The 'prepare' keyword has been replaced by 'override-build'",
    "dn8": "The 'build' keyword has been replaced by 'override-build'",
    "dn9": "The 'install' keyword has been replaced by 'override-build'",
}

_DEPRECATION_URL_FMT = "http://snapcraft.io/docs/deprecation-notices/{id}"

logger = logging.getLogger(__name__)


def _deprecation_message(id):
    message = _DEPRECATION_MESSAGES.get(id)
    if not message:
        raise RuntimeError("No deprecation notice with id {!r}".format(id))

    return message


def handle_deprecation_notice(id):
    message = _deprecation_message(id)

    logger.warning(
        "DEPRECATED: {}\nSee {} for more information.".format(
            message, _DEPRECATION_URL_FMT.format(id=id)
        )
    )
