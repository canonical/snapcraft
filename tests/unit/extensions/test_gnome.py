# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import os
import shutil
import subprocess

import pytest

from snapcraft.extensions import gnome
from snapcraft.extensions.extension import get_extensions_data_dir

############
# Fixtures #
############


@pytest.fixture
def gnome_extension():
    return gnome.GNOME(
        yaml_data={"base": "core22", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def gnome_extension_core24():
    return gnome.GNOME(
        yaml_data={"base": "core24", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def gnome_extension_core26():
    return gnome.GNOME(
        yaml_data={"base": "core26", "parts": {}}, arch="amd64", target_arch="amd64"
    )


@pytest.fixture
def gnome_extension_with_build_snap():
    return gnome.GNOME(
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["gnome-44-2204-sdk"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


@pytest.fixture
def gnome_extension_with_default_build_snap_from_latest_edge():
    return gnome.GNOME(
        yaml_data={
            "base": "core22",
            "parts": {"part1": {"build-snaps": ["gnome-42-2204-sdk/latest/edge"]}},
        },
        arch="amd64",
        target_arch="amd64",
    )


###################
# GNOME Extension #
###################


def test_get_supported_bases():
    assert gnome.GNOME.get_supported_bases() == ("core22", "core24", "core26")


def test_get_supported_confinement():
    assert gnome.GNOME.get_supported_confinement() == ("strict", "devmode")


@pytest.mark.parametrize("base", ["core22", "core24", "core26"])
def test_is_experimental(base):
    assert gnome.GNOME.is_experimental(base=base) is False


def test_get_app_snippet(gnome_extension):
    assert gnome_extension.get_app_snippet(app_name="test-app") == {
        "command-chain": ["snap/command-chain/desktop-launch"],
        "plugs": ["desktop", "desktop-legacy", "gsettings", "opengl", "wayland", "x11"],
    }


def test_get_app_snippet_core24(gnome_extension_core24):
    assert gnome_extension_core24.get_app_snippet(app_name="test-app") == {
        "command-chain": [
            "snap/command-chain/gpu-2404-wrapper",
            "snap/command-chain/desktop-launch",
        ],
        "plugs": ["desktop", "desktop-legacy", "gsettings", "opengl", "wayland", "x11"],
    }


def test_get_app_snippet_core26(gnome_extension_core26):
    assert gnome_extension_core26.get_app_snippet(app_name="test-app") == {
        "command-chain": [
            "snap/command-chain/gpu-2604-wrapper",
            "snap/command-chain/desktop-launch",
        ],
        "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11"],
    }


def test_get_root_snippet(gnome_extension):
    assert gnome_extension.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "environment": {
            "GTK_USE_PORTAL": "1",
            "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
        },
        "hooks": {
            "configure": {
                "command-chain": ["snap/command-chain/hooks-configure-fonts"],
                "plugs": ["desktop"],
            }
        },
        "layout": {
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0"
            },
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1"
            },
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy"
            },
            "/usr/share/xml/iso-codes": {
                "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
            },
            "/usr/share/libdrm": {"bind": "$SNAP/gnome-platform/usr/share/libdrm"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gnome-42-2204": {
                "default-provider": "gnome-42-2204",
                "interface": "content",
                "target": "$SNAP/gnome-platform",
            },
            "gtk-3-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
            },
            "icon-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
            },
            "sound-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
            },
        },
    }


def test_get_root_snippet_with_gpu(gnome_extension_core24):
    snippet = gnome_extension_core24.get_root_snippet()

    assert snippet["plugs"]["gpu-2404"] == {
        "default-provider": "mesa-2404",
        "interface": "content",
        "target": "$SNAP/gpu-2404",
    }

    assert snippet["layout"]["/usr/share/libdrm"] == {
        "bind": "$SNAP/gpu-2404/libdrm",
    }

    assert snippet["layout"]["/usr/share/drirc.d"] == {
        "symlink": "$SNAP/gpu-2404/drirc.d",
    }

    assert snippet["layout"]["/usr/share/X11/XErrorDB"] == {
        "symlink": "$SNAP/gpu-2404/X11/XErrorDB",
    }


def test_get_root_snippet_core24_webkit(gnome_extension_core24):
    snippet = gnome_extension_core24.get_root_snippet()

    assert snippet["layout"][
        "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1"
    ] == {
        "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1",
    }
    assert (
        "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0" not in snippet["layout"]
    )


def test_get_root_snippet_core26_webkit(gnome_extension_core26):
    snippet = gnome_extension_core26.get_root_snippet()

    assert snippet["layout"][
        "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkitgtk-6.0"
    ] == {
        "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkitgtk-6.0",
    }
    assert snippet["layout"][
        "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1"
    ] == {
        "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1",
    }
    assert (
        "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0" not in snippet["layout"]
    )


def test_get_root_snippet_with_glycin(gnome_extension_core26):
    snippet = gnome_extension_core26.get_root_snippet()

    assert snippet["layout"]["/usr/libexec/glycin-loaders"] == {
        "bind": "$SNAP/gnome-platform/usr/libexec/glycin-loaders",
    }


def test_get_root_snippet_with_external_sdk(gnome_extension_with_build_snap):
    assert gnome_extension_with_build_snap.get_root_snippet() == {
        "assumes": ["snapd2.43"],
        "environment": {
            "GTK_USE_PORTAL": "1",
            "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
        },
        "hooks": {
            "configure": {
                "command-chain": ["snap/command-chain/hooks-configure-fonts"],
                "plugs": ["desktop"],
            }
        },
        "layout": {
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0"
            },
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1"
            },
            "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy": {
                "bind": "$SNAP/gnome-platform/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy"
            },
            "/usr/share/xml/iso-codes": {
                "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
            },
            "/usr/share/libdrm": {"bind": "$SNAP/gnome-platform/usr/share/libdrm"},
        },
        "plugs": {
            "desktop": {"mount-host-font-cache": False},
            "gnome-44-2204": {
                "default-provider": "gnome-44-2204",
                "interface": "content",
                "target": "$SNAP/gnome-platform",
            },
            "gtk-3-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
            },
            "icon-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
            },
            "sound-themes": {
                "default-provider": "gtk-common-themes",
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
            },
        },
    }


class TestGetPartSnippet:
    """Tests for GNOME.get_part_snippet when using the default sdk snap name."""

    def test_get_part_snippet(self, gnome_extension):
        self.assert_get_part_snippet(gnome_extension)

    def test_get_part_snippet_latest_edge(
        self, gnome_extension_with_default_build_snap_from_latest_edge
    ):
        self.assert_get_part_snippet(
            gnome_extension_with_default_build_snap_from_latest_edge
        )

    @staticmethod
    def assert_get_part_snippet(gnome_instance):
        assert gnome_instance.get_part_snippet(plugin_name="autotools") == {
            "build-environment": [
                {"SNAPCRAFT_GNOME_SDK": "/snap/gnome-42-2204-sdk/current/"},
                {"PATH": "/snap/gnome-42-2204-sdk/current/usr/bin${PATH:+:$PATH}"},
                {
                    "XDG_DATA_DIRS": (
                        "$CRAFT_STAGE/usr/share:/snap/gnome-42-2204-sdk"
                        "/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                    )
                },
                {
                    "LD_LIBRARY_PATH": ":".join(
                        [
                            "/snap/gnome-42-2204-sdk/current/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                            "/snap/gnome-42-2204-sdk/current/usr/lib",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/vala-current",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/"
                            "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pulseaudio",
                        ]
                    )
                    + "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                },
                {
                    "PKG_CONFIG_PATH": (
                        "/snap/gnome-42-2204-sdk/current/usr/lib/"
                        "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pkgconfig:"
                        "/snap/gnome-42-2204-sdk/current/usr/lib/pkgconfig:"
                        "/snap/gnome-42-2204-sdk/current/usr/share/pkgconfig"
                        "${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
                    )
                },
                {
                    "GETTEXTDATADIRS": (
                        "/snap/gnome-42-2204-sdk/current/usr/share/gettext-current"
                        "${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}"
                    )
                },
                {
                    "GDK_PIXBUF_MODULE_FILE": (
                        "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                        "/gdk-pixbuf-current/loaders.cache"
                    )
                },
                {
                    "ACLOCAL_PATH": (
                        "/snap/gnome-42-2204-sdk/current/usr/share/aclocal"
                        "${ACLOCAL_PATH:+:$ACLOCAL_PATH}"
                    )
                },
                {
                    "PYTHONPATH": ":".join(
                        [
                            "/snap/gnome-42-2204-sdk/current/usr/lib/python3.10",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/python3/dist-packages",
                            "/snap/gnome-42-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                            "/gobject-introspection",
                        ]
                    )
                    + "${PYTHONPATH:+:$PYTHONPATH}"
                },
                {
                    "GI_TYPELIB_PATH": ":".join(
                        [
                            "/snap/gnome-42-2204-sdk/current/usr/lib/girepository-1.0",
                            (
                                "/snap/gnome-42-2204-sdk/current/usr/lib/"
                                "$CRAFT_ARCH_TRIPLET_BUILD_FOR/girepository-1.0"
                            ),
                        ]
                    )
                    + "${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}"
                },
                {
                    "CMAKE_PREFIX_PATH": (
                        "$CRAFT_STAGE/usr:"
                        "/snap/gnome-42-2204-sdk/current/usr"
                        "${CMAKE_PREFIX_PATH:"
                        "+:$CMAKE_PREFIX_PATH}"
                    )
                },
            ]
        }


def test_get_part_snippet_with_external_sdk(gnome_extension_with_build_snap):
    assert gnome_extension_with_build_snap.get_part_snippet(plugin_name="meson") == {
        "build-environment": [
            {"SNAPCRAFT_GNOME_SDK": "/snap/gnome-44-2204-sdk/current/"},
            {"PATH": "/snap/gnome-44-2204-sdk/current/usr/bin${PATH:+:$PATH}"},
            {
                "XDG_DATA_DIRS": (
                    "$CRAFT_STAGE/usr/share:/snap/gnome-44-2204-sdk"
                    "/current/usr/share:/usr/share${XDG_DATA_DIRS:+:$XDG_DATA_DIRS}"
                )
            },
            {
                "LD_LIBRARY_PATH": ":".join(
                    [
                        "/snap/gnome-44-2204-sdk/current/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                        "/snap/gnome-44-2204-sdk/current/usr/lib",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/vala-current",
                        (
                            "/snap/gnome-44-2204-sdk/current/usr/lib/"
                            "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pulseaudio"
                        ),
                    ]
                )
                + "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            },
            {
                "PKG_CONFIG_PATH": (
                    "/snap/gnome-44-2204-sdk/current/usr/lib/"
                    "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pkgconfig:"
                    "/snap/gnome-44-2204-sdk/current/usr/lib/pkgconfig:"
                    "/snap/gnome-44-2204-sdk/current/usr/share/pkgconfig"
                    "${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
                )
            },
            {
                "GETTEXTDATADIRS": (
                    "/snap/gnome-44-2204-sdk/current/usr/share/gettext-current"
                    "${GETTEXTDATADIRS:+:$GETTEXTDATADIRS}"
                )
            },
            {
                "GDK_PIXBUF_MODULE_FILE": (
                    "/snap/gnome-44-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                    "/gdk-pixbuf-current/loaders.cache"
                )
            },
            {
                "ACLOCAL_PATH": (
                    "/snap/gnome-44-2204-sdk/current/usr/share/aclocal"
                    "${ACLOCAL_PATH:+:$ACLOCAL_PATH}"
                )
            },
            {
                "PYTHONPATH": ":".join(
                    [
                        "/snap/gnome-44-2204-sdk/current/usr/lib/python3.10",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/python3/dist-packages",
                        "/snap/gnome-44-2204-sdk/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                        "/gobject-introspection",
                    ]
                )
                + "${PYTHONPATH:+:$PYTHONPATH}"
            },
            {
                "GI_TYPELIB_PATH": ":".join(
                    [
                        "/snap/gnome-44-2204-sdk/current/usr/lib/girepository-1.0",
                        (
                            "/snap/gnome-44-2204-sdk/current/usr/lib/"
                            "$CRAFT_ARCH_TRIPLET_BUILD_FOR/girepository-1.0"
                        ),
                    ]
                )
                + "${GI_TYPELIB_PATH:+:$GI_TYPELIB_PATH}"
            },
            {
                "CMAKE_PREFIX_PATH": (
                    "$CRAFT_STAGE/usr:"
                    "/snap/gnome-44-2204-sdk/current/usr"
                    "${CMAKE_PREFIX_PATH:"
                    "+:$CMAKE_PREFIX_PATH}"
                )
            },
        ]
    }


def test_get_parts_snippet(gnome_extension):
    assert gnome_extension.get_parts_snippet() == {
        "gnome/sdk": {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
            "plugin": "make",
            "build-snaps": ["gnome-42-2204-sdk"],
        }
    }


def test_get_parts_snippet_core24(gnome_extension_core24):
    assert gnome_extension_core24.get_parts_snippet() == {
        "gnome/gpu/wrapper": {
            "source": str(get_extensions_data_dir() / "gpu" / "command-chain"),
            "plugin": "make",
            "make-parameters": ["GPU_INTERFACE=gpu-2404"],
        },
        "gnome/gpu/cleanup": {
            "after": [],
            "source": "https://github.com/canonical/gpu-snap.git",
            "plugin": "nil",
            "override-prime": (
                "craftctl default\n"
                "${CRAFT_PART_SRC}/bin/gpu-2404-cleanup mesa-2404\n"
                "# Workaround for https://bugs.launchpad.net/snapd/+bug/2055273\n"
                'mkdir -p "${CRAFT_PRIME}/gpu-2404"'
            ),
        },
        "gnome/sdk": {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
            "plugin": "make",
            "build-snaps": ["gnome-46-2404-sdk"],
        },
    }


def test_get_parts_snippet_core26(gnome_extension_core26):
    assert gnome_extension_core26.get_parts_snippet() == {
        "gnome/gpu/wrapper": {
            "source": str(get_extensions_data_dir() / "gpu" / "command-chain"),
            "plugin": "make",
            "make-parameters": ["GPU_INTERFACE=gpu-2604"],
        },
        "gnome/gpu/cleanup": {
            "after": [],
            "source": "https://github.com/canonical/gpu-snap.git",
            "plugin": "nil",
            "override-prime": (
                "craftctl default\n${CRAFT_PART_SRC}/bin/gpu-2604-cleanup mesa-2604\n"
            ),
        },
        "gnome/sdk": {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
            "plugin": "make",
            "build-snaps": ["gnome-core26-sdk"],
        },
        "gnome/cleanup": {
            "after": [],
            "plugin": "nil",
            "build-snaps": ["gnome-core26", "gtk-common-themes"],
            "override-prime": CLEANUP_SCRIPTLET,
        },
    }


def test_get_parts_snippet_with_external_sdk(gnome_extension_with_build_snap):
    assert gnome_extension_with_build_snap.get_parts_snippet() == {
        "gnome/sdk": {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
            "plugin": "make",
        }
    }


def test_get_parts_snippet_with_external_sdk_different_channel(
    gnome_extension_with_default_build_snap_from_latest_edge,
):
    assert (
        gnome_extension_with_default_build_snap_from_latest_edge.get_parts_snippet()
        == {
            "gnome/sdk": {
                "source": str(get_extensions_data_dir() / "desktop" / "command-chain"),
                "plugin": "make",
            }
        }
    )


# The override-prime scriptlet injected by the extension for base >= 26,
# mirrored here so the executable behavior tests run exactly what ships.
CLEANUP_SCRIPTLET = (
    "set -eux\n"
    'PRIME_CANON="$(realpath "$CRAFT_PRIME")"\n'
    'for snap in "gnome-core26" "gtk-common-themes"; do\n'
    '    (cd "/snap/$snap/current" && find . -type f,l \\( -name "*.so.*" -o -name "*.so" \\) -print0) | while IFS= read -r -d \'\' rel; do\n'
    '        dir_canon="$(realpath "$CRAFT_PRIME/${rel%/*}")"\n'
    '        case "$dir_canon" in\n'
    '            "$PRIME_CANON"/*) ;;\n'
    '            *) continue ;;\n'
    '        esac\n'
    '        f="$dir_canon/${rel##*/}"\n'
    '        if [ -d "$f" ] && [ ! -L "$f" ]; then continue; fi\n'
    '        # Follow soname symlink chains (libfoo.so ->\n'
    '        # libfoo.so.X -> libfoo.so.X.Y.Z) and remove the real\n'
    '        # versioned file too: once the soname symlinks are gone\n'
    '        # it is unreachable, and providers built from newer\n'
    '        # sources use different file names for the same SONAME.\n'
    '        # The walk stays inside the prime (canonical directory\n'
    '        # containment checked above and same-directory relative\n'
    '        # targets only) and inside the SONAME family (the target\n'
    '        # basename must extend the symlink basename, so\n'
    '        # libfoo.so.1 does not remove a distinct libfoo.so.10).\n'
    '        cur="$f"\n'
    '        hops=0\n'
    '        while [ -L "$cur" ] && [ "$hops" -lt 8 ]; do\n'
    '            t="$(readlink "$cur")"\n'
    '            case "$t" in\n'
    '                */*|"" ) break ;;\n'
    '            esac\n'
    '            b="${cur##*/}"\n'
    '            case "$t" in\n'
    '                "$b".*) ;;\n'
    '                *) break ;;\n'
    '            esac\n'
    '            rm -f "$cur"\n'
    '            cur="${cur%/*}/$t"\n'
    '            hops=$((hops + 1))\n'
    '        done\n'
    '        if [ ! -d "$cur" ] || [ -L "$cur" ]; then rm -f "$cur"; fi\n'
    '    done\n'
    "done\n"
)


@pytest.fixture
def cleanup_scriptlet_harness(tmp_path):
    """Run the injected cleanup scriptlet against a fake provider/prime."""

    def run(provider_files, prime_files, providers=("provider",)):
        """provider_files/prime_files: str = regular file,
        tuple(path, target) = symlink at path pointing to target."""

        def materialize(base, entries):
            for e in entries:
                path, target = (e, None) if isinstance(e, str) else e
                p = base / path
                p.parent.mkdir(parents=True, exist_ok=True)
                if target is not None:
                    os.symlink(target, p)
                else:
                    p.touch()

        # Both trees are laid out under a "current" directory like
        # /snap/<name>/current, as the scriptlet expects.
        materialize(tmp_path / "provider", provider_files)
        materialize(tmp_path / "prime", prime_files)
        for name in providers:
            for e in provider_files:
                path, target = (e, None) if isinstance(e, str) else e
                dst = tmp_path / name / "current" / path
                dst.parent.mkdir(parents=True, exist_ok=True)
                if not dst.exists():
                    if target is not None:
                        os.symlink(target, dst)
                    else:
                        dst.touch()
        script = CLEANUP_SCRIPTLET.replace(
            '"gnome-core26" "gtk-common-themes"',
            '"provider"',
        ).replace('cd "/snap/$snap/current"', 'cd "$SNAP_ROOT/$snap/current"')
        env = dict(os.environ, CRAFT_PRIME=str(tmp_path / "prime"), SNAP_ROOT=str(tmp_path))
        subprocess.run(
            ["bash", "-c", script],
            check=True,
            env=env,
            capture_output=True,
            text=True,
        )
        return tmp_path / "prime"

    return run


def _tree(root):
    return sorted(
        os.path.join(os.path.relpath(dirpath, root), name)
        for dirpath, _, files in os.walk(root)
        for name in files
        if not os.path.islink(os.path.join(dirpath, name)) or os.path.exists(os.path.join(dirpath, name))
    )


def test_cleanup_scriptlet_removes_provider_duplicates_and_dead_reals(
    cleanup_scriptlet_harness, tmp_path
):
    prime = cleanup_scriptlet_harness(
        provider_files=[
            "usr/lib/libcairo.so.2.11806.6",
            ("usr/lib/libcairo.so.2", "libcairo.so.2.11806.6"),
            ("usr/lib/libcairo.so", "libcairo.so.2"),
            "usr/lib/libbar.so.1.1",
            ("usr/lib/libbar.so.1", "libbar.so.1.1"),
        ],
        prime_files=[
            "usr/lib/libcairo.so.2.11804.4",
            ("usr/lib/libcairo.so.2", "libcairo.so.2.11804.4"),
            ("usr/lib/libcairo.so", "libcairo.so.2"),
            "usr/lib/libbar.so.1.1",
            ("usr/lib/libbar.so.1", "libbar.so.1.1"),
            "usr/lib/libbar.so.10",  # distinct SONAME: must survive
            "usr/lib/libunrelated.so.0",
        ],
    )
    remaining = _tree(prime)
    # the whole cairo family is gone (dead real file included)
    assert not [f for f in remaining if "libcairo" in f]
    # strict family: libbar.so.1 removed, the distinct libbar.so.10 survives
    assert "usr/lib/libbar.so.10" in remaining
    assert "usr/lib/libbar.so.1" not in [f for f in remaining]
    assert "usr/lib/libunrelated.so.0" in remaining


def test_cleanup_scriptlet_does_not_escape_prime(cleanup_scriptlet_harness, tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "libfoo.so.2").touch()
    prime = cleanup_scriptlet_harness(
        provider_files=["usr/lib/libfoo.so"],
        prime_files=[("usr/lib/libfoo.so", "../../../outside/libfoo.so.2")],
        providers=("provider",),
    )
    # the escaping symlink is removed, but the OUTSIDE target survives
    assert (outside / "libfoo.so.2").exists()
    assert not (prime / "usr" / "lib" / "libfoo.so").is_symlink()


def test_cleanup_scriptlet_skips_directory_collision(cleanup_scriptlet_harness):
    prime = cleanup_scriptlet_harness(
        provider_files=["usr/lib/libfoo.so"],
        prime_files=["usr/lib/libfoo.so/keepme"],
        providers=("provider",),
    )
    # the directory (and its contents) are left alone
    assert (prime / "usr" / "lib" / "libfoo.so" / "keepme").exists()


def test_cleanup_scriptlet_handles_spaces_and_dangling_links(
    cleanup_scriptlet_harness, tmp_path
):
    prime = cleanup_scriptlet_harness(
        provider_files=[
            "usr/lib/libfoo space.so.2.6",
            ("usr/lib/libfoo space.so.2", "libfoo space.so.2.6"),
        ],
        prime_files=[
            "usr/lib/libfoo space.so.2.4",
            ("usr/lib/libfoo space.so.2", "libfoo space.so.2.4"),
            ("usr/lib/dangling.so.2", "nonexistent.so.2.0"),
        ],
        providers=("provider",),
    )
    remaining = _tree(prime)
    # the dead real file behind the removed soname symlink is removed
    # (that is the point of the fix), the dangling link is removed, and
    # nothing else in the prime is touched
    assert remaining == []
