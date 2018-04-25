# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
from typing import Type

from snapcraft.internal.build_providers import errors


class InstanceInfo:

    @classmethod
    def from_json(cls: Type['InstanceInfo'], *, instance_name: str,
                  json_info: str) -> 'InstanceInfo':
        """Create an InstanceInfo from json_info retrieved from multipass.

        :param str instance_name: the name of the instance.
        :param str json_info: a json formatted string with the structure
                              that would follow the output of a json formatted
                              multipass info command.
        :returns: an InstanceInfo.
        :rtype: InstanceInfo
        :raises snapcraft.internal.build_providers.ProviderInfoDataKeyError:
            if the instance name cannot be found in the given json or if a
            required key is missing from that data structure for the instance.
        """
        try:
            json_data = json.loads(json_info)
        except json.decoder.JSONDecodeError as decode_error:
            raise errors.ProviderBadDataError(
                provider_name='multipass',
                data=json_info) from decode_error
        try:
            instance_info = json_data['info'][instance_name]
        except KeyError as missing_key:
            raise errors.ProviderInfoDataKeyError(
                provider_name='multipass',
                missing_key=str(missing_key),
                data=json_data) from missing_key
        try:
            return cls(name=instance_name,
                       state=instance_info['state'],
                       image_release=instance_info['image_release'])
        except KeyError as missing_key:
            raise errors.ProviderInfoDataKeyError(
                provider_name='multipass',
                missing_key=str(missing_key),
                data=instance_info) from missing_key

    def __init__(self, *, name: str, state: str, image_release: str) -> None:
        """Initialize an InstanceInfo.

        :param str name: the instance name.
        :param str state: the state of the instance which can be any one of
                          RUNNING, STOPPED, DELETED.
        :param str image_release: the Operating System release string for the
                                  image.
        """
        # We do not check for validity of state given that new states could
        # be introduced.
        self.name = name
        self.state = state
        self.image_release = image_release

    def is_stopped(self):
        return self.state == 'STOPPED'
