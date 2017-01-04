# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

import json
from textwrap import dedent

_sample_keys = [
    {
        'name': 'default',
        'sha3-384': (
            'vdEeQvRxmZ26npJCFaGnl-VfGz0lU2jZ'
            'ZkWp_s7E-RxVCNtH2_mtjcxq2NkDKkIp'),
    },
    {
        'name': 'another',
        'sha3-384': (
            'JsfToV5hO2eN9l89pYYCKXUioTERrZII'
            'HUgQQd47jW8YNNBskupiIjWYd3KXLY_D'),
    },
]


def get_sample_key(name):
    for key in _sample_keys:
        if key['name'] == name:
            return key
    raise KeyError(name)


def mock_snap_output(command, *args, **kwargs):
    if command == ['snap', 'keys', '--json']:
        return json.dumps(_sample_keys)
    elif command[:2] == ['snap', 'export-key']:
        if not command[2].startswith('--account='):
            raise AssertionError('Unhandled command: {}'.format(command))
        account_id = command[2][len('--account='):]
        name = command[3]
        # This isn't a full account-key-request assertion, but it's enough
        # for testing.
        return dedent('''\
            type: account-key-request
            account-id: {account_id}
            name: {name}
            public-key-sha3-384: {sha3_384}
            ''').format(
                account_id=account_id, name=name,
                sha3_384=get_sample_key(name)['sha3-384'])
    else:
        raise AssertionError('Unhandled command: {}'.format(command))
