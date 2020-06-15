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

        self.root_snippet = {}

        self.app_snippet = {
            "environment": {
                "INTEL_OPENVINO_DIR": "$SNAP/openvino",
                "GI_TYPELIB_PATH": "$SNAP/openvino/data_processing/gstreamer/lib/girepository-1.0",
                "GST_PLUGIN_PATH": "$SNAP/openvino/data_processing/dl_streamer/lib:$SNAP/openvino/data_processing/gstreamer/lib/gstreamer-1.0",
                "GST_PLUGIN_SCANNER": "$SNAP/openvino/data_processing/gstreamer/bin/gstreamer-1.0/gst-plugin-scanner",
                "GST_SAMPLES_DIR": "$SNAP/openvino/data_processing/dl_streamer/samples",
                "GST_VAAPI_ALL_DRIVERS": "1",
                "HDDL_INSTALL_DIR": "$SNAP/openvino/deployment_tools/inference_engine/external/hddl",
                "INTEL_CVSDK_DIR": "$SNAP/openvino",
                "InferenceEngine_DIR": "$SNAP/openvino/deployment_tools/inference_engine/share",
                "LC_NUMERIC": "C",
                "LD_LIBRARY_PATH": "$SNAP/openvino/data_processing/dl_streamer/lib:$SNAP/openvino/data_processing/gstreamer/lib:$SNAP/openvino/opencv/lib:$SNAP/openvino/deployment_tools/ngraph/lib:$SNAP/openvino/deployment_tools/inference_engine/external/hddl/lib:$SNAP/openvino/deployment_tools/inference_engine/external/gna/lib:$SNAP/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:$SNAP/openvino/deployment_tools/inference_engine/external/tbb/lib:$SNAP/openvino/deployment_tools/inference_engine/lib/intel64:$SNAP/openvino/deployment_tools/inference_engine/lib/aarch64:$SNAP/openvino/lib",
                "LIBRARY_PATH": "$SNAP/openvino/data_processing/dl_streamer/lib:$SNAP/openvino/data_processing/gstreamer/lib:",
                "MODELS_PATH": "/home/ubuntu/intel/dl_streamer/models",
                "OpenCV_DIR": "$SNAP/openvino/opencv/cmake",
                "PATH": "$SNAP/openvino/deployment_tools/model_optimizer:$SNAP/openvino/data_processing/gstreamer/bin:$SNAP/openvino/data_processing/gstreamer/bin/gstreamer-1.0:$PATH",
                "PKG_CONFIG_PATH": "$SNAP/openvino/data_processing/dl_streamer/lib/pkgconfig:$SNAP/openvino/data_processing/gstreamer/lib/pkgconfig:",
                "PYTHONPATH": "$SNAP/openvino/opencv/lib/python3.6/site-packages:$SNAP/openvino/python/python3.6:$SNAP/openvino/python/python3:$SNAP/openvino/deployment_tools/open_model_zoo/tools/accuracy_checker:$SNAP/openvino/deployment_tools/model_optimizer:$SNAP/openvino/data_processing/dl_streamer/python:$SNAP/openvino/data_processing/gstreamer/lib/python3.6/site-packages:",
                #"ngraph_DIR": "$SNAP/openvino/deployment_tools/ngraph/cmake",
                "ngraph_DIR": "$SNAP/openvino/cmake",
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
                {
                    "INTEL_OPENVINO_DIR": f"/snap/{toolkit_snap}/current/openvino"
                },
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
                {
                    "INTEL_CVSDK_DIR": "$INTEL_OPENVINO_DIR"
                },
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
                #{"ngraph_DIR": "$INTEL_OPENVINO_DIR/deployment_tools/ngraph/cmake"},
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
                "stage-packages": [
                    "bison",
                    "ffmpeg",
                    "flex",
                    "gettext",
                    "gettext-base",
                    "gir1.2-glib-2.0",
                    "gir1.2-gudev-1.0",
                    "gobject-introspection",
                    "gstreamer1.0-plugins-bad",
                    "gstreamer1.0-plugins-base",
                    "gstreamer1.0-plugins-good",
                    "iso-codes",
                    "liba52-0.7.4",
                    "libaa1",
                    "libasound2",
                    "libasound2-data",
                    "libass9",
                    "libasyncns0",
                    "libatk-bridge2.0-0",
                    "libatk1.0-0",
                    "libatspi2.0-0",
                    "libavc1394-0",
                    "libavcodec57",
                    "libavdevice57",
                    "libavfilter6",
                    "libavformat57",
                    "libavresample3",
                    "libavutil55",
                    "libbluetooth3",
                    "libbluray2",
                    "libbs2b0",
                    "libcaca0",
                    "libcap2-bin",
                    "libcdio-cdda2",
                    "libcdio-paranoia2",
                    "libcdio17",
                    "libcdparanoia0",
                    "libchromaprint1",
                    #"libcrystalhd3",
                    "libcurl3-gnutls",
                    "libdc1394-22",
                    "libdca0",
                    "libde265-0",
                    "libdirectfb-1.7-7",
                    "libdrm-amdgpu1",
                    #"libdrm-intel1",
                    "libdrm-nouveau2",
                    "libdrm-radeon1",
                    "libdv4",
                    "libdvdnav4",
                    "libdvdread4",
                    "libegl-mesa0",
                    "libegl1",
                    "libelf1",
                    "libepoxy0",
                    "libfaac0",
                    "libfaad2",
                    "libfdk-aac1",
                    "libfftw3-double3",
                    "libflac8",
                    "libflite1",
                    "libfluidsynth1",
                    "libfribidi0",
                    "libgbm1",
                    "libgirepository-1.0-1",
                    "libgl1",
                    "libgl1-mesa-dri",
                    "libglapi-mesa",
                    "libgles1",
                    "libgles2",
                    "libglib2.0-bin",
                    "libglib2.0-data",
                    "libglib2.0-dev-bin",
                    "libglvnd0",
                    "libglx-mesa0",
                    "libglx0",
                    "libgme0",
                    "libgpm2",
                    "libgraphene-1.0-0",
                    "libgsl23",
                    "libgslcblas0",
                    "libgsm1",
                    "libgssdp-1.0-3",
                    "libgstreamer-gl1.0-0",
                    "libgstreamer-plugins-bad1.0-0",
                    "libgstreamer-plugins-base1.0-0",
                    "libgstreamer-plugins-good1.0-0",
                    "libgstreamer1.0-0",
                    "libgtk2.0-0",
                    "libgtk-3-0",
                    "libgudev-1.0-0",
                    "libgupnp-1.0-4",
                    "libgupnp-igd-1.0-4",
                    "libiec61883-0",
                    "libilmbase12",
                    "libjack-jackd2-0",
                    "libkate1",
                    "liblavfile-2.1-0",
                    "liblavjpeg-2.1-0",
                    "liblavplay-2.1-0",
                    "liblilv-0-0",
                    "libllvm9",
                    "liblrdf0",
                    "libmjpegutils-2.1-0",
                    "libmms0",
                    "libmodplug1",
                    "libmp3lame0",
                    "libmpcdec6",
                    "libmpeg2-4",
                    "libmpeg2encpp-2.1-0",
                    "libmpg123-0",
                    "libmplex2-2.1-0",
                    "libmysofa0",
                    "libneon27",
                    "libnice10",
                    "libnorm1",
                    "libnuma1",
                    "libofa0",
                    "libogg0",
                    "libopenal-data",
                    "libopenal1",
                    "libopencore-amrnb0",
                    "libopencore-amrwb0",
                    "libopenexr22",
                    "libopengl0",
                    "libopenjp2-7",
                    "libopenmpt0",
                    "libopenni2-0",
                    "libopus0",
                    "liborc-0.4-0",
                    "libpciaccess0",
                    "libpcre16-3",
                    "libpcre32-3",
                    "libpcrecpp0v5",
                    "libpgm-5.2-0",
                    "libpostproc54",
                    "libpulse0",
                    "libpython3.6",
                    "libquicktime2",
                    "libraptor2-0",
                    "libraw1394-11",
                    "librubberband2",
                    "libsamplerate0",
                    "libsbc1",
                    "libsdl1.2debian",
                    "libsdl2-2.0-0",
                    "libsensors4",
                    "libserd-0-0",
                    "libshine3",
                    "libshout3",
                    "libslang2",
                    "libsnappy1v5",
                    "libsndfile1",
                    "libsndio6.1",
                    "libsodium23",
                    "libsord-0-0",
                    "libsoundtouch1",
                    "libsoxr0",
                    "libspandsp2",
                    "libspeex1",
                    "libsratom-0-0",
                    "libsrtp2-1",
                    "libssh-4",
                    "libssh-gcrypt-4",
                    "libssh2-1",
                    "libswresample2",
                    "libswscale4",
                    "libtag-extras1",
                    "libtag1v5",
                    "libtag1v5-vanilla",
                    "libtheora0",
                    "libtwolame0",
                    "libusb-1.0-0",
                    "libv4l-0",
                    "libv4lconvert0",
                    "libva-drm2",
                    "libva-x11-2",
                    "libva2",
                    "libvdpau1",
                    "libvisual-0.4-0",
                    "libvo-aacenc0",
                    "libvo-amrwbenc0",
                    "libvorbis0a",
                    "libvorbisenc2",
                    "libvorbisfile3",
                    "libvpx5",
                    "libvulkan1",
                    "libwavpack1",
                    "libwayland-server0",
                    "libwebp6",
                    "libwebpmux3",
                    "libwebrtc-audio-processing1",
                    "libwildmidi-config",
                    "libwildmidi2",
                    "libwrap0",
                    "libx11-xcb1",
                    "libx264-152",
                    "libx265-146",
                    "libxcb-dri2-0",
                    "libxcb-dri3-0",
                    "libxcb-glx0",
                    "libxcb-present0",
                    "libxcb-randr0",
                    "libxcb-shape0",
                    "libxcb-sync1",
                    "libxcb-xfixes0",
                    "libxcb-xkb1",
                    "libxcomposite1",
                    "libxshmfence1",
                    "libxslt1.1",
                    "libxss1",
                    "libxv1",
                    "libxvidcore4",
                    "libxxf86vm1",
                    "libyajl2",
                    "libzbar0",
                    "libzmq5",
                    "libzvbi-common",
                    "libzvbi0",
                    "pkg-config",
                    "python3-distutils",
                    "python3-gi",
                    "python3-lib2to3",
                    "python3-mako",
                    "python3-markupsafe",
                    "x11-common",
                    "xorg-sgml-doctools",
                ],
                "override-build": f"""
                    snapcraftctl build
                    mkdir -p $SNAPCRAFT_PART_INSTALL/openvino
                    cp -rv /snap/{toolkit_snap}/current/openvino/* $SNAPCRAFT_PART_INSTALL/openvino/
                """,
            }
        }
