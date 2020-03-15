# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from testtools.matchers import Equals, Contains

from snapcraft.internal.project_loader._extensions.gnome_3_34 import ExtensionImpl

from .. import ProjectLoaderBaseTest
import textwrap

from tests.unit.commands import CommandBaseTestCase


class ExtensionTest(ProjectLoaderBaseTest, CommandBaseTestCase):
    def test_extension(self):
        gnome_extension = ExtensionImpl(
            extension_name="gnome-3.34", yaml_data=dict(base="core18")
        )

        self.expectThat(
            gnome_extension.root_snippet,
            Equals(
                {
                    "plugs": {
                        "gtk-3-themes": {
                            "interface": "content",
                            "target": "$SNAP/data-dir/themes",
                            "default-provider": "gtk-common-themes",
                        },
                        "icon-themes": {
                            "interface": "content",
                            "target": "$SNAP/data-dir/icons",
                            "default-provider": "gtk-common-themes",
                        },
                        "sound-themes": {
                            "interface": "content",
                            "target": "$SNAP/data-dir/sounds",
                            "default-provider": "gtk-common-themes",
                        },
                        "gnome-3-34-1804": {
                            "interface": "content",
                            "target": "$SNAP/gnome-platform",
                            "default-provider": "gnome-3-34-1804",
                        },
                    },
                    "environment": {
                        "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
                        "GTK_USE_PORTALS": "1",
                    },
                    "layout": {
                        "/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0": {
                            "bind": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0"
                        },
                        "/usr/share/xml/iso-codes": {
                            "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                        },
                    },
                }
            ),
        )
        self.expectThat(
            gnome_extension.app_snippet,
            Equals(
                {
                    "command-chain": ["snap/command-chain/desktop-launch"],
                    "plugs": [
                        "desktop",
                        "desktop-legacy",
                        "gsettings",
                        "wayland",
                        "x11",
                    ],
                }
            ),
        )
        self.expectThat(
            gnome_extension.part_snippet,
            Equals(
                {
                    "build-environment": [
                        {"PATH": "/snap/gnome-3-34-1804-sdk/current/usr/bin:$PATH"},
                        {
                            "XDG_DATA_DIRS": "/snap/gnome-3-34-1804-sdk/current/usr/share:/usr/share:$XDG_DATA_DIRS"
                        },
                        {
                            "LD_LIBRARY_PATH": "/snap/gnome-3-34-1804-sdk/current/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib:/snap/gnome-3-34-1804-sdk/current/usr/lib/vala-current:$LD_LIBRARY_PATH"
                        },
                        {
                            "PKG_CONFIG_PATH": "/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/lib/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/share/pkgconfig:$PKG_CONFIG_PATH"
                        },
                        {
                            "GETTEXTDATADIRS": "/snap/gnome-3-34-1804-sdk/current/usr/share/gettext-current:$GETTEXTDATADIRS"
                        },
                        {
                            "GDK_PIXBUF_MODULE_FILE": "/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache"
                        },
                    ]
                }
            ),
        )
        self.expectThat(
            gnome_extension.parts,
            Equals(
                {
                    "gnome-3-34-extension": {
                        "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                        "source-subdir": "gnome",
                        "plugin": "make",
                        "build-snaps": ["gnome-3-34-1804-sdk/latest/stable"],
                    }
                }
            ),
        )

    def test_supported_bases(self):
        self.assertThat(ExtensionImpl.get_supported_bases(), Equals(("core18",)))

    def test_supported_confinement(self):
        self.assertThat(
            ExtensionImpl.get_supported_confinement(), Equals(("strict", "devmode"))
        )

    def test_no_defined_build_env(self):
        """
        Test that the build-environment defined by the gnome-3-34 extension is
        applied to all parts. In this test case, there is no user-defined
        build-environment section in any part.
        """
        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: test
                version: '1'
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        extensions: [gnome-3-34]

                parts:
                    test-part-1:
                        plugin: nil
                    test-part-2:
                        plugin: nil
                """
            )
        )

        result = self.run_command(["expand-extensions"])

        self.assertThat(
            result.output,
            Contains(
                textwrap.dedent(
                    """\
                        parts:
                          test-part-1:
                            plugin: nil
                            build-environment: &id001
                            - PATH: /snap/gnome-3-34-1804-sdk/current/usr/bin:$PATH
                            - XDG_DATA_DIRS: /snap/gnome-3-34-1804-sdk/current/usr/share:/usr/share:$XDG_DATA_DIRS
                            - LD_LIBRARY_PATH: /snap/gnome-3-34-1804-sdk/current/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib:/snap/gnome-3-34-1804-sdk/current/usr/lib/vala-current:$LD_LIBRARY_PATH
                            - PKG_CONFIG_PATH: /snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/lib/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/share/pkgconfig:$PKG_CONFIG_PATH
                            - GETTEXTDATADIRS: /snap/gnome-3-34-1804-sdk/current/usr/share/gettext-current:$GETTEXTDATADIRS
                            - GDK_PIXBUF_MODULE_FILE: /snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache
                          test-part-2:
                            plugin: nil
                            build-environment: *id001
                          gnome-3-34-extension:
                            build-snaps:
                            - gnome-3-34-1804-sdk/latest/stable
                            plugin: make
                            source: $SNAPCRAFT_EXTENSIONS_DIR/desktop
                            source-subdir: gnome
                    """
                )
            ),
        )

        # Test that the layouts are as set in the extension
        self.assertThat(
            result.output,
            Contains(
                textwrap.dedent(
                    """\
                        layout:
                          /usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0:
                            bind: $SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0
                          /usr/share/xml/iso-codes:
                            bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
                    """
                )
            ),
        )

        # test that the environment is as set in the extension
        self.assertThat(
            result.output,
            Contains(
                textwrap.dedent(
                    """\
                        environment:
                          GTK_USE_PORTALS: \'1\'
                          SNAP_DESKTOP_RUNTIME: $SNAP/gnome-platform
                    """
                )
            ),
        )

        # test that the interface plugs are as set in the extension
        self.assertThat(
            result.output,
            Contains(
                textwrap.dedent(
                    """\
                        plugs:
                          gnome-3-34-1804:
                            default-provider: gnome-3-34-1804
                            interface: content
                            target: $SNAP/gnome-platform
                          gtk-3-themes:
                            default-provider: gtk-common-themes
                            interface: content
                            target: $SNAP/data-dir/themes
                          icon-themes:
                            default-provider: gtk-common-themes
                            interface: content
                            target: $SNAP/data-dir/icons
                          sound-themes:
                            default-provider: gtk-common-themes
                            interface: content
                            target: $SNAP/data-dir/sounds
                    """
                )
            ),
        )

    def test_no_redundant_build_env(self):
        """
        Test that the build-environment defined by the gnome-3-34 extension is
        added to any build-environment variables defined by the user. In this
        test case, there are some user-defined build-environment variables that
        are not defined by the extension. The final build-environment should be
        the total set of variables defined by the user and the extension.
        """
        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: test
                version: '1'
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        extensions: [gnome-3-34]

                parts:
                    test-part-1:
                        plugin: nil
                        build-environment:
                            - TEST_VAR1: dir1
                            - TEST_VAR2: dir2
                    test-part-2:
                        plugin: nil
                        build-environment:
                            - TEST_VAR3: dir3
                            - TEST_VAR4: dir4
                """
            )
        )

        result = self.run_command(["expand-extensions"])

        self.assertThat(
            result.output,
            Contains(
                textwrap.dedent(
                    """\
                        parts:
                          test-part-1:
                            plugin: nil
                            build-environment:
                            - &id001
                              PATH: /snap/gnome-3-34-1804-sdk/current/usr/bin:$PATH
                            - &id002
                              XDG_DATA_DIRS: /snap/gnome-3-34-1804-sdk/current/usr/share:/usr/share:$XDG_DATA_DIRS
                            - &id003
                              LD_LIBRARY_PATH: /snap/gnome-3-34-1804-sdk/current/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib:/snap/gnome-3-34-1804-sdk/current/usr/lib/vala-current:$LD_LIBRARY_PATH
                            - &id004
                              PKG_CONFIG_PATH: /snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/lib/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/share/pkgconfig:$PKG_CONFIG_PATH
                            - &id005
                              GETTEXTDATADIRS: /snap/gnome-3-34-1804-sdk/current/usr/share/gettext-current:$GETTEXTDATADIRS
                            - &id006
                              GDK_PIXBUF_MODULE_FILE: /snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache
                            - TEST_VAR1: dir1
                            - TEST_VAR2: dir2
                          test-part-2:
                            plugin: nil
                            build-environment:
                            - *id001
                            - *id002
                            - *id003
                            - *id004
                            - *id005
                            - *id006
                            - TEST_VAR3: dir3
                            - TEST_VAR4: dir4
                    """
                )
            ),
        )

    def test_some_redundant_build_env(self):
        """
        Test that the user-defined build-environment variables are used in
        replacement of the extension-defined variables, when the variables are
        defined in both places.
        """
        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: test
                version: '1'
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        extensions: [gnome-3-34]

                parts:
                    test-part-1:
                        plugin: nil
                        build-environment:
                            - TEST_VAR1: dir1
                            - TEST_VAR2: dir2
                            - PATH: path1
                            - PATH: path2
                            - PKG_CONFIG_PATH: $PKG_CONFIG_PATH
                    test-part-2:
                        plugin: nil
                        build-environment:
                            - TEST_VAR3: dir3
                            - TEST_VAR4: dir4
                            - GDK_PIXBUF_MODULE_FILE: $GDK_PIXBUF_MODULE_FILE
                            - LD_LIBRARY_PATH: $LD_LIBRARY_PATH
                """
            )
        )

        result = self.run_command(["expand-extensions"])

        self.assertThat(
            result.output,
            Contains(
                textwrap.dedent(
                    """\
                        parts:
                          test-part-1:
                            plugin: nil
                            build-environment:
                            - &id001
                              PATH: /snap/gnome-3-34-1804-sdk/current/usr/bin:$PATH
                            - &id002
                              XDG_DATA_DIRS: /snap/gnome-3-34-1804-sdk/current/usr/share:/usr/share:$XDG_DATA_DIRS
                            - &id003
                              LD_LIBRARY_PATH: /snap/gnome-3-34-1804-sdk/current/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib:/snap/gnome-3-34-1804-sdk/current/usr/lib/vala-current:$LD_LIBRARY_PATH
                            - &id004
                              PKG_CONFIG_PATH: /snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/lib/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/share/pkgconfig:$PKG_CONFIG_PATH
                            - &id005
                              GETTEXTDATADIRS: /snap/gnome-3-34-1804-sdk/current/usr/share/gettext-current:$GETTEXTDATADIRS
                            - &id006
                              GDK_PIXBUF_MODULE_FILE: /snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache
                            - TEST_VAR1: dir1
                            - TEST_VAR2: dir2
                            - PATH: path1
                            - PATH: path2
                            - PKG_CONFIG_PATH: $PKG_CONFIG_PATH
                          test-part-2:
                            plugin: nil
                            build-environment:
                            - *id001
                            - *id002
                            - *id003
                            - *id004
                            - *id005
                            - *id006
                            - TEST_VAR3: dir3
                            - TEST_VAR4: dir4
                            - GDK_PIXBUF_MODULE_FILE: $GDK_PIXBUF_MODULE_FILE
                            - LD_LIBRARY_PATH: $LD_LIBRARY_PATH
                    """
                )
            ),
        )
