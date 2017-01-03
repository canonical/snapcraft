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
    'dn1': "The 'snap' keyword has been replaced by 'prime'."
}

_DEPRECATION_URL_FMT = (
    'https://github.com/snapcore/snapcraft/wiki/Deprecation-Notices#{id}')

logger = logging.getLogger(__name__)


def _deprecation_message(id):
    message = _DEPRECATION_MESSAGES.get(id)
    if not message:
        raise RuntimeError('No deprecation notice with id {!r}'.format(id))

    return message


def handle_deprecation_notice(id):
    message = _deprecation_message(id)

    logger.warning('DEPRECATED: {}\nSee {} for more information.'.format(
        message, _DEPRECATION_URL_FMT.format(id=id)))
