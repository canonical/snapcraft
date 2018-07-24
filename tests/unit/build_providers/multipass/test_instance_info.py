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

from textwrap import dedent

from testtools.matchers import Equals

from snapcraft.internal.build_providers import errors
from snapcraft.internal.build_providers._multipass._instance_info import (
    InstanceInfo
)  # noqa: E501
from tests import unit


class InstanceInfoGeneralTest(unit.TestCase):
    def test_initialize(self):
        instance_info = InstanceInfo(
            name="instance-name", state="RUNNING", image_release="16.04 LTS"
        )

        self.assertThat(instance_info.name, Equals("instance-name"))
        self.assertThat(instance_info.state, Equals("RUNNING"))
        self.assertThat(instance_info.image_release, Equals("16.04 LTS"))
        self.assertThat(instance_info.is_stopped(), Equals(False))

    def test_instance_is_stopped(self):
        instance_info = InstanceInfo(
            name="instance-name", state="STOPPED", image_release="16.04 LTS"
        )

        self.assertThat(instance_info.is_stopped(), Equals(True))


class InstanceInfoFromJSONTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.json_string = dedent(
            """\
            {
                "errors": [
                ],
                "info": {
                    "announceable-rodney": {
                        "disks": {
                            "sda1": {
                                "total": "5136297984",
                                "used": "1076072448"
                            }
                        },
                        "image_hash": "bc990872f070249450fc187d668bf65e2f87ffa3d7cf6e2934cb84b62af368e1",
                        "image_release": "16.04 LTS",
                        "ipv4": [
                            "10.74.70.141"
                        ],
                        "load": [
                            0,
                            0,
                            0
                        ],
                        "memory": {
                            "total": 1040318464,
                            "used": 42754048
                        },
                        "mounts": {
                        },
                        "release": "Ubuntu 16.04.4 LTS",
                        "state": "RUNNING"
                    }
                }
            }
        """
        )  # noqa: E501
        self.instance_name = "announceable-rodney"

    def test_new_instance_from_unmarshalled_json(self):
        instance_info = InstanceInfo.from_json(
            instance_name=self.instance_name, json_info=self.json_string
        )

        self.assertThat(instance_info.name, Equals(self.instance_name))
        self.assertThat(instance_info.state, Equals("RUNNING"))
        self.assertThat(instance_info.image_release, Equals("16.04 LTS"))

    def test_new_instance_from_unmarshalled_json_wrong_instance_fails(self):
        raised = self.assertRaises(
            errors.ProviderInfoDataKeyError,
            InstanceInfo.from_json,
            instance_name="wrong-name",
            json_info=self.json_string,
        )
        self.assertThat(raised.provider_name, Equals("multipass"))
        self.assertThat(raised.missing_key, Equals("'wrong-name'"))

    def test_new_instance_from_unmarshalled_json_empty_fails(self):
        raised = self.assertRaises(
            errors.ProviderBadDataError,
            InstanceInfo.from_json,
            instance_name=self.instance_name,
            json_info="bad-json",
        )
        self.assertThat(raised.provider_name, Equals("multipass"))
        self.assertThat(raised.data, Equals("bad-json"))
