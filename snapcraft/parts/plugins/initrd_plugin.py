# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
# pylint: disable=line-too-long,too-many-lines,attribute-defined-outside-init
#
# Copyright 2025 Canonical Ltd.
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

"""The initrd plugin for building kernel snaps.

- initrd-addons
  (list of strings; default: none)
  A list of files to include in the initrd, provided as relative paths to
  $CRAFT_STAGE/addons. For example,

      initrd-addons:
          - usr/bin/foo

  will result in "${CRAFT_STAGE}/addons/usr/bin/foo" being placed in the
  initrd as /usr/bin/foo.

- initrd-firmware:
  (list of strings; default: none)
  A list of firmware to include in the initrd, provided as relative paths to
  $CRAFT_STAGE/firmware. For example,

      initrd-firmware:
          - foo/bar.bin

  will result in "${CRAFT_STAGE}/firmware/foo/bar.bin" being placed in the
  initrd as /usr/lib/firmware/foo/bar.bin.

- initrd-modules:
  (list of strings; default: none)
  A list of modules to include in the initrd, provided as a list of module
  names. If the specified module(s) have dependencies, they are also installed.

- initrd-build-efi-image
  (string; default: false)
  Set to true if an EFI or UKI image is preferred over discrete kernel and
  initrd files.

- initrd-efi-image-key
  (string; default: snake oil key (/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key))
  Requires initrd-build-efi-image to be true.
  Key to be used when creating the EFI image, provided as a relative path to
  $CRAFT_STAGE/signing. For example,

      initrd-efi-image-key: signing.key

  will result in "${CRAFT_STAGE}/signing/signing.key" being placed in the
  initrd chroot as /root/signing.key.

- initrd-efi-image-cert
  (string; default: snake oil certificate (/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem))
  Requires initrd-build-efi-image to be true.
  Certificate to be used when creating the EFI image, provided as a relative
  path to $CRAFT_STAGE/signing. For example,

      initrd-efi-image-cert: cert.pem

  will result in "${CRAFT_STAGE}/signing/cert.pem" being placed in the
  initrd chroot as /root/cert.pem.
"""

import os
from typing import Literal, cast

import pydantic
from craft_parts import infos, plugins
from overrides import overrides
from typing_extensions import Self


class InitrdPluginProperties(plugins.PluginProperties, frozen=True):
    """The part properties used by the Initrd plugin."""

    plugin: Literal["initrd"] = "initrd"

    initrd_build_efi_image: bool = False
    initrd_efi_image_key: str = (
        "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key"
    )
    initrd_efi_image_cert: str = (
        "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem"
    )
    initrd_modules: list[str] = []
    initrd_firmware: list[str] = []
    initrd_addons: list[str] = []

    # part properties required by the plugin
    @pydantic.model_validator(mode="after")
    def validate_plugin_options(self) -> Self:
        _signing_key = self.initrd_efi_image_key
        _signing_cert = self.initrd_efi_image_cert

        # Validate that either both key and cert are specified or neither is
        if bool(_signing_key) ^ bool(_signing_cert):
            raise ValueError(
                "If one of initrd-efi-image-key or initrd-efi-image-cert is set, both must be set"
            )

        return self


class InitrdPlugin(plugins.Plugin):
    """Plugin for the initrd snap build."""

    properties_class = InitrdPluginProperties

    def __init__(
        self, *, properties: plugins.PluginProperties, part_info: infos.PartInfo
    ) -> None:
        super().__init__(properties=properties, part_info=part_info)
        self.options = cast(InitrdPluginProperties, self._options)

    @overrides
    def get_build_snaps(self) -> set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> set[str]:
        _host_arch = self._part_info.host_arch
        _target_arch = self._part_info.target_arch

        build_packages = {
            "curl",
            "dracut-core",
            "fakeroot",
        }
        # if running as non-root and cross-building
        # we need libfake{ch}root for the target arch
        if _host_arch != _target_arch and (os.getuid() != 0):
            build_packages |= {
                f"libfakeroot:{_target_arch}",
            }
        return build_packages

    @overrides
    def get_build_environment(self) -> dict[str, str]:
        return {}

    @overrides
    def get_build_commands(self) -> list[str]:
        _base = self._part_info.base
        _arch = self._part_info.target_arch
        build_efi_image = self.options.initrd_build_efi_image

        # It only makes sense to provide a key and cert when building a UKI
        if self.options.initrd_efi_image_key and self.options.initrd_efi_image_cert:
            build_efi_image = True

        if build_efi_image:
            # There are no EFI stubs for s390x or ppc64el
            if _arch in {"s390x", "ppc64el"}:
                raise ValueError("initrd-build-efi-image not allowed for " + _arch)

            # There are no EFI stubs for riscv until 24.04
            if _arch == "riscv64" and _base == "core22":
                raise ValueError("initrd-build-efi-image not allowed for riscv64")

        return [
            " ".join(
                [
                    "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/initrd_build.sh",
                    f"initrd-modules={','.join(self.options.initrd_modules)}",
                    f"initrd-firmware={','.join(self.options.initrd_firmware)}",
                    f"initrd-addons={','.join(self.options.initrd_addons)}",
                    f"initrd-build-efi-image={build_efi_image}",
                    f"initrd-efi-image-key={self.options.initrd_efi_image_key}",
                    f"initrd-efi-image-cert={self.options.initrd_efi_image_cert}",
                ]
            )
        ]
