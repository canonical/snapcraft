# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import pytest
from craft_parts import Part, PartInfo, ProjectInfo

from snapcraft.parts.plugins import InitrdPlugin


@pytest.fixture(autouse=True)
def part_info(new_dir):
    yield PartInfo(
        project_info=ProjectInfo(
            application_name="test", project_name="test-snap", cache_dir=new_dir
        ),
        part=Part("my-part", {}),
    )


def test_get_build_snaps(part_info):
    properties = InitrdPlugin.properties_class.unmarshal({"source": "."})
    plugin = InitrdPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_snaps() == set()


def test_get_build_packages(part_info):
    properties = InitrdPlugin.properties_class.unmarshal({"source": "."})
    plugin = InitrdPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_packages() == {
        "curl",
        "dracut-core",
        "fakechroot",
        "fakeroot",
    }


def test_get_build_environment(part_info):
    properties = InitrdPlugin.properties_class.unmarshal({"source": "."})
    plugin = InitrdPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_environment() == {}


def test_get_build_commands(part_info):
    properties = InitrdPlugin.properties_class.unmarshal(
        {
            "source": ".",
            "initrd-addons": {
                "usr/bin/foo",
            },
            "initrd-firmware": {
                "foo.bin",
            },
            "initrd-modules": {
                "foo",
            },
            "initrd-build-efi-image": "true",
            "initrd-efi-image-key": "signing.key",
            "initrd-efi-image-cert": "cert.pem",
        }
    )
    plugin = InitrdPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/initrd_build.sh "
        "initrd-modules=foo "
        "initrd-firmware=foo.bin "
        "initrd-addons=usr/bin/foo "
        "initrd-build-efi-image=True "
        "initrd-efi-image-key=signing.key "
        "initrd-efi-image-cert=cert.pem"
    ]
