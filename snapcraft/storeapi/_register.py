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

from snapcraft import config


from .common import (
    get_oauth_session,
    store_raw_api_call,
)


def register_name(name):
    """Register a snap name via the store API.

    If successful, returns the snap ID.
    """
    # FIXME: Switch to macaroons as soon as register-name/ have support for
    # them. -- vila 2016-04-07
    conf = config.Config()
    conf.load()
    session = get_oauth_session(conf)
    data = dict(snap_name=name)
    response = store_raw_api_call('register-name/', method='POST',
                                  data=data,
                                  session=session)
    # for macaroons replace session=session with:
    # headers={'authorization': macaroon_auth}
    if not response.ok:
        # if (response['errors'] == ['Authorization Required']
        #     and (response.headers['WWW-Authenticate']
        #          == "Macaroon needs_refresh=1":
            # Refresh the discharge macaroon and retry
        pass
    return response
