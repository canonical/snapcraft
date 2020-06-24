# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

# Import types and tell flake8 to ignore the "unused" List.

from typing import Any, Dict, Tuple

from ._extension import Extension

_PLATFORM_SNAP = dict(core18="openvino-sdk-2020-3")


class ExtensionImpl(Extension):
    """This extension assists in creation of snaps using OpenVINO toolkit."""

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core18",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        toolkit_snap = "cjp256-openvino-toolkit-1804"

        self.root_snippet = {
            "plugs": {
                "openvino-1804": {
                    "interface": "content",
                    "target": "$SNAP/openvino-platform",
                    "default-provider": toolkit_snap,
                },
            },
        }

        self.app_snippet = {
            "environment": {
                "INTEL_OPENVINO_DIR": "$SNAP/openvino-platform/openvino",
                "GI_TYPELIB_PATH": "$SNAP/openvino-platform/openvino/data_processing/gstreamer/lib/girepository-1.0",
                "GST_PLUGIN_PATH": "$SNAP/openvino-platform/openvino/data_processing/dl_streamer/lib:$SNAP/openvino-platform/openvino/data_processing/gstreamer/lib/gstreamer-1.0",
                "GST_PLUGIN_SCANNER": "$SNAP/openvino-platform/openvino/data_processing/gstreamer/bin/gstreamer-1.0/gst-plugin-scanner",
                "GST_SAMPLES_DIR": "$SNAP/openvino-platform/openvino/data_processing/dl_streamer/samples",
                "GST_VAAPI_ALL_DRIVERS": "1",
                "HDDL_INSTALL_DIR": "$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/external/hddl",
                "INTEL_CVSDK_DIR": "$SNAP/openvino-platform/openvino",
                "InferenceEngine_DIR": "$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/share",
                "LC_NUMERIC": "C",
                "LD_LIBRARY_PATH": "$SNAP/openvino-platform/openvino/data_processing/dl_streamer/lib:$SNAP/openvino-platform/openvino/data_processing/gstreamer/lib:$SNAP/openvino-platform/openvino/opencv/lib:$SNAP/openvino-platform/openvino/deployment_tools/ngraph/lib:$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/external/hddl/lib:$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/external/gna/lib:$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/external/tbb/lib:$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/lib/intel64:$SNAP/openvino-platform/openvino/deployment_tools/inference_engine/lib/aarch64:$SNAP/openvino-platform/openvino/lib:$SNAP/openvino-platform/lib/$SNAPCRAFT_ARCH_TRIPLET:$SNAP/openvino-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:$SNAP/openvino-platform/usr/lib:$SNAP/openvino-platform/usr/lib/vala-current",
                "LIBRARY_PATH": "$SNAP/openvino-platform/openvino/data_processing/dl_streamer/lib:$SNAP/openvino-platform/openvino/data_processing/gstreamer/lib:",
                "OpenCV_DIR": "$SNAP/openvino-platform/openvino/opencv/cmake",
                "PATH": "$SNAP/openvino-platform/openvino/deployment_tools/model_optimizer:$SNAP/openvino-platform/openvino/data_processing/gstreamer/bin:$SNAP/openvino-platform/openvino/data_processing/gstreamer/bin/gstreamer-1.0:$PATH",
                "PKG_CONFIG_PATH": "$SNAP/openvino-platform/openvino/data_processing/dl_streamer/lib/pkgconfig:$SNAP/openvino-platform/openvino/data_processing/gstreamer/lib/pkgconfig:",
                "PYTHONPATH": "$SNAP/openvino-platform/openvino/opencv/lib/python3.6/dist-packages:$SNAP/openvino-platform/openvino/opencv/lib/python3.6/site-packages:$SNAP/openvino-platform/openvino/python/python3.6:$SNAP/openvino-platform/openvino/python/python3:$SNAP/openvino-platform/openvino/deployment_tools/open_model_zoo/tools/accuracy_checker:$SNAP/openvino-platform/openvino/deployment_tools/model_optimizer:$SNAP/openvino-platform/openvino/data_processing/dl_streamer/python:$SNAP/openvino-platform/openvino/data_processing/gstreamer/lib/python3.6/site-packages:",
                # "ngraph_DIR": "$SNAP/openvino-platform/openvino/deployment_tools/ngraph/cmake",
                "ngraph_DIR": "$SNAP/openvino-platform/openvino/cmake",
            },
            "plugs": [
                "home",
                "hardware-observe",
                "mount-observe",
                "network-bind",
                "network-control",
                "network-observe",
                "raw-usb",
            ],
        }

        self.part_snippet = {
            "build-environment": [
                {"OPENVINO_SNAP": f"/snap/{toolkit_snap}/current"},
                {"PATH": "$OPENVINO_SNAP/usr/bin:$PATH"},
                {
                    "LD_LIBRARY_PATH": "$OPENVINO_SNAP/lib/$SNAPCRAFT_ARCH_TRIPLET:$OPENVINO_SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:$OPENVINO_SNAP/usr/lib:$OPENVINO_SNAP/usr/lib/vala-current:$LD_LIBRARY_PATH"
                },
                {
                    "PKG_CONFIG_PATH": "$OPENVINO_SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:$OPENVINO_SNAP/usr/lib/pkgconfig:$OPENVINO_SNAP/usr/share/pkgconfig:$PKG_CONFIG_PATH"
                },
                {"INTEL_OPENVINO_DIR": "$OPENVINO_SNAP/openvino"},
                {
                    "GI_TYPELIB_PATH": "$INTEL_OPENVINO_DIR/data_processing/gstreamer/lib/girepository-1.0"
                },
                {
                    "GST_PLUGIN_PATH": "$INTEL_OPENVINO_DIR/data_processing/dl_streamer/lib:$INTEL_OPENVINO_DIR/data_processing/gstreamer/lib/gstreamer-1.0"
                },
                {
                    "GST_PLUGIN_SCANNER": "$INTEL_OPENVINO_DIR/data_processing/gstreamer/bin/gstreamer-1.0/gst-plugin-scanner"
                },
                {
                    "GST_SAMPLES_DIR": "$INTEL_OPENVINO_DIR/data_processing/dl_streamer/samples"
                },
                {"GST_VAAPI_ALL_DRIVERS": "1"},
                {
                    "HDDL_INSTALL_DIR": "$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/external/hddl"
                },
                {"INTEL_CVSDK_DIR": "$INTEL_OPENVINO_DIR"},
                {
                    "InferenceEngine_DIR": "$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/share"
                },
                {"LC_NUMERIC": "C"},
                {
                    "LD_LIBRARY_PATH": "$INTEL_OPENVINO_DIR/data_processing/dl_streamer/lib:$INTEL_OPENVINO_DIR/data_processing/gstreamer/lib:$INTEL_OPENVINO_DIR/opencv/lib:$INTEL_OPENVINO_DIR/deployment_tools/ngraph/lib:$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/external/hddl/lib:$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/external/gna/lib:$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/external/mkltiny_lnx/lib:$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/external/tbb/lib:$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/lib/intel64:$INTEL_OPENVINO_DIR/deployment_tools/inference_engine/lib/aarch64:$INTEL_OPENVINO_DIR/lib:"
                },
                {
                    "LIBRARY_PATH": "$INTEL_OPENVINO_DIR/data_processing/dl_streamer/lib:$INTEL_OPENVINO_DIR/data_processing/gstreamer/lib:"
                },
                {"MODELS_PATH": "$INTEL_OPENVINO_DIR/models"},
                {"OpenCV_DIR": "$INTEL_OPENVINO_DIR/opencv"},
                {
                    "PATH": "$INTEL_OPENVINO_DIR/deployment_tools/model_optimizer:$INTEL_OPENVINO_DIR/data_processing/gstreamer/bin:$INTEL_OPENVINO_DIR/data_processing/gstreamer/bin/gstreamer-1.0:$PATH"
                },
                {
                    "PKG_CONFIG_PATH": "$INTEL_OPENVINO_DIR/data_processing/dl_streamer/lib/pkgconfig:$INTEL_OPENVINO_DIR/data_processing/gstreamer/lib/pkgconfig:"
                },
                {
                    "PYTHONPATH": "$INTEL_OPENVINO_DIR/opencv/lib/python3.6/site-packages:$INTEL_OPENVINO_DIR/python/python3.6:$INTEL_OPENVINO_DIR/python/python3:$INTEL_OPENVINO_DIR/deployment_tools/open_model_zoo/tools/accuracy_checker:$INTEL_OPENVINO_DIR/deployment_tools/model_optimizer:$INTEL_OPENVINO_DIR/data_processing/dl_streamer/python:$INTEL_OPENVINO_DIR/data_processing/gstreamer/lib/python3.6/site-packages:"
                },
                # {"ngraph_DIR": "$INTEL_OPENVINO_DIR/deployment_tools/ngraph/cmake"},
                {"ngraph_DIR": "$INTEL_OPENVINO_DIR/cmake"},
            ]
        }

        self.parts = {
            "openvino-2020-3-194": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR",
                # "source": "http://registrationcenter-download.intel.com/akdlm/irc_nas/16670/l_openvino_toolkit_p_2020.3.194.tgz",
                # "override-build": "./install.sh --accept_eula --silent --install_dir $SNAPCRAFT_PART_INSTALL/openvino-toolkit",
                "plugin": "nil",
                "build-snaps": [f"{toolkit_snap}/latest/edge"],
                "build-packages": [
                    "libbison-dev",
                    "libdrm-dev",
                    "libgl1-mesa-dev",
                    "libglib2.0-dev",
                    "libglvnd-core-dev",
                    "libglvnd-dev",
                    "libgudev-1.0-dev",
                    "libmjpegtools-dev",
                    "libpcre3-dev",
                    "libpthread-stubs0-dev",
                    "libusb-1.0-0-dev",
                    "libx11-dev",
                    "libx11-xcb-dev",
                    "libxau-dev",
                    "libxcb-dri2-0-dev",
                    "libxcb-dri3-dev",
                    "libxcb-glx0-dev",
                    "libxcb-present-dev",
                    "libxcb-randr0-dev",
                    "libxcb-render0-dev",
                    "libxcb-shape0-dev",
                    "libxcb-sync-dev",
                    "libxcb-xfixes0-dev",
                    "libxcb1-dev",
                    "libxdamage-dev",
                    "libxdmcp-dev",
                    "libxext-dev",
                    "libxfixes-dev",
                    "libxshmfence-dev",
                    "libxxf86vm-dev",
                    "mesa-common-dev",
                    "x11proto-core-dev",
                    "x11proto-damage-dev",
                    "x11proto-dev",
                    "x11proto-fixes-dev",
                    "x11proto-xext-dev",
                    "x11proto-xf86vidmode-dev",
                    "xtrans-dev",
                    "zlib1g-dev",
                ],
                "override-build": f"""
                    snapcraftctl build
                    mkdir -p $SNAPCRAFT_PART_INSTALL/openvino-platform
                """,
            }
        }
