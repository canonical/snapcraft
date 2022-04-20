# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2022 Canonical Ltd.
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
import textwrap
from pathlib import Path
from typing import Optional

import pytest

from snapcraft.meta import ExtractedMetadata, appstream


def _create_desktop_file(desktop_file_path, icon: Optional[str] = None) -> None:
    dir_name = os.path.dirname(desktop_file_path)
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)
    with open(desktop_file_path, "w") as f:
        print("[Desktop Entry]", file=f)
        if icon:
            print(f"Icon={icon}", file=f)


@pytest.mark.usefixtures("new_dir")
class TestAppstreamData:
    """Extract metadata and check each extracted field."""

    @pytest.mark.parametrize("file_extension", ["metainfo.xml", "appdata.xml"])
    @pytest.mark.parametrize(
        "key,attributes,param_name,value,expect",
        [
            ("summary", {}, "summary", "test-summary", "test-summary"),
            ("description", {}, "description", "test-description", "test-description"),
            ("icon", {"type": "local"}, "icon", "/test/path", None),
            ("icon", {"type": "local"}, "icon", "/icon.png", "/icon.png"),
            ("id", {}, "common_id", "test-id", "test-id"),
            ("name", {}, "title", "test-title", "test-title"),
        ],
    )
    def test_entries(self, file_extension, key, attributes, param_name, value, expect):
        file_name = f"foo.{file_extension}"
        attrs = " ".join(f'{attr}="{attributes[attr]}"' for attr in attributes)
        Path(file_name).write_text(
            textwrap.dedent(
                f"""\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <{key} {attrs}>{value}</{key}>
                </component>"""
            )
        )

        Path("icon.png").touch()
        kwargs = {param_name: expect}
        expected = ExtractedMetadata(**kwargs)

        assert appstream.extract(file_name, workdir=".") == expected


# See LP #1814898 for a description of possible fallbacks
@pytest.mark.usefixtures("new_dir")
class TestAppstreamIcons:
    """Check extraction of icon-related metadata."""

    def _create_appstream_file(
        self, icon: Optional[str] = None, icon_type: str = "local"
    ):
        with open("foo.appdata.xml", "w") as f:
            if icon:
                f.write(
                    textwrap.dedent(
                        f"""\
                        <?xml version="1.0" encoding="UTF-8"?>
                        <component>
                          <launchable type="desktop-id">my.app.desktop</launchable>
                          <icon type="{icon_type}">{icon}</icon>
                        </component>"""
                    )
                )
            else:
                f.write(
                    textwrap.dedent(
                        """\
                        <?xml version="1.0" encoding="UTF-8"?>
                        <component>
                          <launchable type="desktop-id">my.app.desktop</launchable>
                        </component>"""
                    )
                )

    def _create_index_theme(self, theme: str):
        # TODO: populate index.theme
        dir_name = os.path.join("usr", "share", "icons", theme)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        Path(dir_name, "index.theme").touch()

    def _create_icon_file(self, theme: str, size: str, filename: str) -> None:
        dir_name = os.path.join("usr", "share", "icons", theme, size, "apps")
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        Path(dir_name, filename).touch()

    def _expect_icon(self, icon):
        expected = ExtractedMetadata(icon=icon)
        actual = appstream.extract("foo.appdata.xml", workdir=".")
        assert actual is not None
        assert actual.icon == expected.icon

    def test_appstream_NxN_size_not_int_is_skipped(self):
        self._create_appstream_file(icon="icon", icon_type="stock")
        dir_name = os.path.join("usr", "share", "icons", "hicolor", "NxN")
        os.makedirs(dir_name)
        self._expect_icon(None)

    def test_appstream_index_theme_is_not_confused_for_size(self):
        self._create_appstream_file(icon="icon", icon_type="stock")
        self._create_index_theme("hicolor")
        self._expect_icon(None)

    def test_appstream_stock_icon_exists_png(self):
        self._create_appstream_file(icon="icon", icon_type="stock")
        self._create_icon_file("hicolor", "48x48", "icon.png")
        self._create_icon_file("hicolor", "64x64", "icon.png")
        self._expect_icon("usr/share/icons/hicolor/64x64/apps/icon.png")

    def test_appstream_stock_icon_not_exist(self):
        self._create_appstream_file(icon="missing", icon_type="stock")
        self._expect_icon(None)

    def test_appstream_no_icon_no_fallback(self):
        self._create_appstream_file()
        self._expect_icon(None)

    def test_appstream_local_icon_exists(self):
        self._create_appstream_file(icon="/icon.png")
        Path("icon.png").touch()
        self._expect_icon("/icon.png")

    def test_appstream_local_icon_not_exist_no_fallback(self):
        self._create_appstream_file(icon="/missing.png")
        self._expect_icon(None)

    def test_appstream_local_icon_not_absolute_no_fallback(self):
        self._create_appstream_file(icon="foo")
        self._expect_icon(None)

    def test_appstream_no_icon_desktop_fallback_no_icon(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop")
        self._expect_icon(None)

    def test_appstream_no_icon_desktop_fallback_icon_not_exist(self):
        self._create_appstream_file()
        _create_desktop_file(
            "usr/share/applications/my.app.desktop", icon="/missing.png"
        )
        self._expect_icon("/missing.png")

    def test_appstream_no_icon_desktop_fallback_icon_exists(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="/icon.png")
        Path("icon.png").touch()
        self._expect_icon("/icon.png")

    def test_appstream_no_icon_theme_fallback_png(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svg")
        self._create_icon_file("hicolor", "48x48", "icon.png")
        self._create_icon_file("hicolor", "64x64", "icon.png")
        self._expect_icon("usr/share/icons/hicolor/64x64/apps/icon.png")

    def test_appstream_no_icon_theme_fallback_xpm(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svg")
        self._create_icon_file("hicolor", "48x48", "icon.png")
        self._create_icon_file("hicolor", "64x64", "icon.xpm")
        self._expect_icon("usr/share/icons/hicolor/64x64/apps/icon.xpm")

    def test_appstream_no_icon_theme_fallback_svg(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svg")
        self._expect_icon("usr/share/icons/hicolor/scalable/apps/icon.svg")

    def test_appstream_no_icon_theme_fallback_svgz(self):
        self._create_appstream_file()
        _create_desktop_file("usr/share/applications/my.app.desktop", icon="icon")
        self._create_icon_file("hicolor", "scalable", "icon.svgz")
        self._expect_icon("usr/share/icons/hicolor/scalable/apps/icon.svgz")


@pytest.mark.usefixtures("new_dir")
class TestAppstreamContent:
    """Check variations of the Appstream file content."""

    def test_appstream_with_ul(self):
        file_name = "snapcraft_legacy.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="utf-8"?>
            <component type="desktop">
              <id>io.snapcraft.snapcraft</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0</project_license>
              <name>snapcraft</name>
              <name xml:lang="es">snapcraft</name>
              <summary>Create snaps</summary>
              <summary xml:lang="es">Crea snaps</summary>
              <description>
                <p>Command Line Utility to create snaps.</p>
                <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
                <p>Features:</p>
                <p xml:lang="es">Funciones:</p>
                <ul>
                  <li>Build snaps.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ul>
              </description>
              <provides>
                <binary>snapcraft</binary>
              </provides>
            </component>
        """
        )

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.summary == "Create snaps"
        assert metadata.description == textwrap.dedent(
            """\
            Command Line Utility to create snaps.

            Features:

            - Build snaps.
            - Publish snaps to the store."""
        )

    def test_appstream_with_ol(self):
        file_name = "snapcraft_legacy.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="utf-8"?>
            <component type="desktop">
              <id>io.snapcraft.snapcraft</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0</project_license>
              <name>snapcraft</name>
              <name xml:lang="es">snapcraft</name>
              <summary>Create snaps</summary>
              <summary xml:lang="es">Crea snaps</summary>
              <description>
                <p>Command Line Utility to create snaps.</p>
                <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
                <p>Features:</p>
                <p xml:lang="es">Funciones:</p>
                <ol>
                  <li>Build snaps.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ol>
              </description>
              <provides>
                <binary>snapcraft</binary>
              </provides>
            </component>
        """
        )

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.summary == "Create snaps"
        assert metadata.description == textwrap.dedent(
            """\
            Command Line Utility to create snaps.

            Features:

            1. Build snaps.
            2. Publish snaps to the store."""
        )

    def test_appstream_with_ul_in_p(self):
        file_name = "snapcraft_legacy.appdata.xml"
        # pylint: disable=line-too-long
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="UTF-8"?>
              <component type="desktop">
              <id>com.github.maoschanz.drawing</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0-or-later</project_license>
              <content_rating type="oars-1.1"/>
              <name>Drawing</name>
              <name xml:lang="tr">Çizim</name>
              <name xml:lang="pt_BR">Drawing</name>
              <summary>A drawing application for the GNOME desktop</summary>
              <summary xml:lang="pt_BR">Uma aplicacao de desenho para o ambiente GNOME</summary>
              <summary xml:lang="nl">Een tekenprogramma voor de GNOME-werkomgeving</summary>
              <description>
                <p>"Drawing" is a basic image editor, supporting PNG, JPEG and BMP file types.</p>
                <p xml:lang="pt_BR">"Drawing" e um simples editor de imagens, que suporta arquivos PNG,JPEG e BMP</p>
                <p xml:lang="nl">"Tekenen" is een eenvoudige afbeeldingsbewerker, met ondersteuning voor PNG, JPEG en BMP.</p>
                <p xml:lang="fr">"Dessin" est un éditeur d'images basique, qui supporte les fichiers de type PNG, JPEG ou BMP.</p>
                <p>It allows you to draw or edit pictures with tools such as:
                  <ul>
                    <li>Pencil (with various options)</li>
                    <li xml:lang="pt_BR">Lápis (Com varias opções)</li>
                    <li xml:lang="nl">Potlood (verschillende soorten)</li>
                    <li xml:lang="fr">Crayon (avec diverses options)</li>
                    <li>Selection (cut/copy/paste/drag/…)</li>
                    <li xml:lang="tr">Seçim (kes/kopyala/yapıştır/sürükle /…)</li>
                    <li xml:lang="ru">Выделение (вырезать/копировать/вставить/перетащить/…)</li>
                    <li xml:lang="pt_BR">Seleção (cortar/copiar/colar/arrastar/…)</li>
                    <li xml:lang="nl">Selectie (knippen/kopiëren/plakken/verslepen/...)</li>
                    <li xml:lang="it">Selezione (taglia/copia/incolla/trascina/…)</li>
                    <li xml:lang="he">בחירה (חתיכה/העתקה/הדבקה/גרירה/...)</li>
                    <li xml:lang="fr">Sélection (copier/coller/déplacer/…)</li>
                    <li xml:lang="es">Selección (cortar/copiar/pegar/arrastrar/…)</li>
                    <li xml:lang="de_DE">Auswahl (Ausschneiden/Kopieren/Einfügen/Ziehen/...)</li>
                    <li>Line, Arc (with various options)</li>
                    <li xml:lang="pt_BR">Linha, Arco (com varias opcoes)</li>
                    <li xml:lang="nl">Lijn, Boog (verschillende soorten)</li>
                    <li xml:lang="fr">Trait, Arc (avec diverses options)</li>
                    <li>Shapes (rectangle, circle, polygon, …)</li>
                    <li xml:lang="pt_BR">Formas (retângulo, circulo, polígono, …)</li>
                    <li xml:lang="nl">Vormen (vierkant, cirkel, veelhoek, ...)</li>
                    <li xml:lang="fr">Formes (rectangle, cercle, polygone, …)</li>
                    <li>Text insertion</li>
                    <li xml:lang="pt_BR">Inserção de texto</li>
                    <li xml:lang="nl">Tekst invoeren</li>
                    <li xml:lang="fr">Insertion de texte</li>
                    <li>Resizing, cropping, rotating</li>
                    <li xml:lang="pt_BR">Redimencionar, cortar, rotacionar</li>
                    <li xml:lang="nl">Afmetingen wijzigen, bijsnijden, draaien</li>
                    <li xml:lang="fr">Redimensionnement, rognage, rotation</li>
                  </ul>
                </p>
              </description>
              </component>
        """
        )
        # pylint: enable=line-too-long

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.summary == "A drawing application for the GNOME desktop"
        assert metadata.description == textwrap.dedent(
            """\
            "Drawing" is a basic image editor, supporting PNG, JPEG and BMP file types.

            It allows you to draw or edit pictures with tools such as:

            - Pencil (with various options)
            - Selection (cut/copy/paste/drag/…)
            - Line, Arc (with various options)
            - Shapes (rectangle, circle, polygon, …)
            - Text insertion
            - Resizing, cropping, rotating"""
        )

    def test_appstream_multilang_title(self):
        file_name = "foliate.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="UTF-8"?>
            <component type="desktop">
            <name>Foliate</name>
            <name xml:lang="id_ID">Foliate_id</name>
            <name xml:lang="pt_BR">Foliate_pt</name>
            <name xml:lang="ru_RU">Foliate_ru</name>
            <name xml:lang="nl_NL">Foliate_nl</name>
            <name xml:lang="fr_FR">Foliate_fr</name>
            <name xml:lang="cs_CS">Foliate_cs</name>
            </component>
        """
        )

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.title == "Foliate"

    def test_appstream_release(self):
        file_name = "foliate.appdata.xml"
        # pylint: disable=line-too-long
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="UTF-8"?>
            <component type="desktop">
            <releases>
                <release version="1.5.3" date="2019-07-25">
                <description>
                    <ul>
                    <li>Fixed Flatpak version not being able to open .mobi, .azw, and .azw3 files</li>
                    <li>Improved Wiktionary lookup, now with links and example sentences</li>
                    <li>Improved popover footnote extraction and formatting</li>
                    <li>Added option to export annotations to BibTeX</li>
                    </ul>
                </description>
                </release>
                <release version="1.5.2" date="2019-07-19">
                <description>
                    <ul>
                    <li>Fixed table of contents navigation not working with some books</li>
                    <li>Fixed not being able to zoom images with Kindle books</li>
                    <li>Fixed not being able to open books with .epub3 filename extension</li>
                    <li>Fixed temporary directory not being cleaned after closing</li>
                    </ul>
                </description>
                </release>
                <release version="1.5.1" date="2019-07-17">
                <description>
                    <ul>
                    <li>Fixed F9 shortcut not working</li>
                    <li>Updated translations</li>
                    </ul>
                </description>
                </release>
            </releases>
            </component>
        """
        )
        # pylint: enable=line-too-long

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.version == "1.5.3"

    def test_appstream_em(self):
        file_name = "foliate.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="UTF-8"?>
              <component type="desktop">
              <id>com.github.maoschanz.drawing</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0-or-later</project_license>
              <content_rating type="oars-1.1"/>
              <name>Drawing</name>
              <description>
                <p>Command Line Utility to <em>create snaps</em> quickly.</p>
                <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
                <p>Ordered Features:</p>
                <p xml:lang="es">Funciones:</p>
                <ol>
                  <li><em>Build snaps</em>.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ol>
                <p>Unordered Features:</p>
                <ul>
                  <li><em>Build snaps</em>.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ul>
              </description>
              </component>
        """
        )

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.description == textwrap.dedent(
            """\
            Command Line Utility to _create snaps_ quickly.

            Ordered Features:

            1. _Build snaps_.
            2. Publish snaps to the store.

            Unordered Features:

            - _Build snaps_.
            - Publish snaps to the store."""
        )

    def test_appstream_code_tags_not_swallowed(self):
        file_name = "foliate.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="UTF-8"?>
              <component type="desktop">
              <id>com.github.maoschanz.drawing</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0-or-later</project_license>
              <content_rating type="oars-1.1"/>
              <name>Drawing</name>
              <description>
                <p>Command Line Utility to <code>create snaps</code> quickly.</p>
                <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
                <p>Ordered Features:</p>
                <p xml:lang="es">Funciones:</p>
                <ol>
                  <li><code>Build snaps</code>.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ol>
                <p>Unordered Features:</p>
                <ul>
                  <li><code>Build snaps</code>.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ul>
              </description>
              </component>
        """
        )

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.description == textwrap.dedent(
            """\
            Command Line Utility to create snaps quickly.

            Ordered Features:

            1. Build snaps.
            2. Publish snaps to the store.

            Unordered Features:

            - Build snaps.
            - Publish snaps to the store."""
        )

    def test_appstream_with_comments(self):
        file_name = "foo.appdata.xml"
        content = textwrap.dedent(
            """\
            <?xml version="1.0" encoding="UTF-8"?>
              <component type="desktop">
              <id>com.github.maoschanz.drawing</id>
              <metadata_license>CC0-1.0</metadata_license>
              <project_license>GPL-3.0-or-later</project_license>
              <content_rating type="oars-1.1"/>
              <!-- TRANSLATORS: the application name -->
              <name>Drawing</name>
              <!-- TRANSLATORS: one-line description for the app -->
              <summary>Draw stuff</summary>
              <description>
                <!-- TRANSLATORS: AppData description marketing paragraph -->
                <p>Command Line Utility to create snaps quickly.</p>
                <p xml:lang="es">Aplicativo de línea de comandos para crear snaps.</p>
                <p>Ordered Features:</p>
                <p xml:lang="es">Funciones:</p>
                <ol>
                  <li>Build snaps.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ol>
                <p>Unordered Features:</p>
                <ul>
                  <li>Build snaps.</li>
                  <li xml:lang="es">Construye snaps.</li>
                  <li>Publish snaps to the store.</li>
                  <li xml:lang="es">Publica snaps en la tienda.</li>
                </ul>
              </description>
              </component>
        """
        )

        Path(file_name).write_text(content)

        metadata = appstream.extract(file_name, workdir=".")

        assert metadata is not None
        assert metadata.description == textwrap.dedent(
            """\
            Command Line Utility to create snaps quickly.

            Ordered Features:

            1. Build snaps.
            2. Publish snaps to the store.

            Unordered Features:

            - Build snaps.
            - Publish snaps to the store."""
        )


@pytest.mark.usefixtures("new_dir")
class TestAppstreamUnhandledFile:
    """Unhandled files should return None."""

    def test_unhandled_file_test_case(self):
        assert appstream.extract("unhandled-file", workdir=".") is None


@pytest.mark.usefixtures("new_dir")
class TestAppstreamLaunchable:
    """Desktop file path must be extracted correctly."""

    @pytest.mark.parametrize(
        "desktop_file_path",
        [
            "usr/share/applications/com.example.test/app.desktop",
            "usr/local/share/applications/com.example.test/app.desktop",
        ],
    )
    def test(self, desktop_file_path):
        Path("foo.metainfo.xml").write_text(
            textwrap.dedent(
                """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <launchable type="desktop-id">
                    com.example.test-app.desktop
                  </launchable>
                </component>"""
            )
        )

        _create_desktop_file(desktop_file_path)

        extracted = appstream.extract("foo.metainfo.xml", workdir=".")

        assert extracted is not None
        assert extracted.desktop_file_paths == [desktop_file_path]


@pytest.mark.usefixtures("new_dir")
class TestAppstreamLegacyDesktop:
    """Legacy desktop file path must be extracted correctly."""

    @pytest.mark.parametrize(
        "desktop_file_path",
        [
            "usr/share/applications/com.example.test/app.desktop",
            "usr/local/share/applications/com.example.test/app.desktop",
        ],
    )
    def test_launchable(self, desktop_file_path):
        Path("foo.metainfo.xml").write_text(
            textwrap.dedent(
                """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component type="desktop">
                  <id>com.example.test-app.desktop</id>
                </component>"""
            )
        )

        _create_desktop_file(desktop_file_path)

        extracted = appstream.extract("foo.metainfo.xml", workdir=".")

        assert extracted is not None
        assert extracted.desktop_file_paths == [desktop_file_path]

    @pytest.mark.parametrize(
        "desktop_file_path",
        [
            "usr/share/applications/com.example.test/app.desktop",
            "usr/local/share/applications/com.example.test/app.desktop",
        ],
    )
    def test_appstream_no_desktop_suffix(self, desktop_file_path):
        Path("foo.metainfo.xml").write_text(
            textwrap.dedent(
                """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component type="desktop">
                  <id>com.example.test-app</id>
                </component>"""
            )
        )

        _create_desktop_file(desktop_file_path)

        extracted = appstream.extract("foo.metainfo.xml", workdir=".")

        assert extracted is not None
        assert extracted.desktop_file_paths == [desktop_file_path]


@pytest.mark.usefixtures("new_dir")
class TestAppstreamMultipleLaunchable:
    """Multiple desktop file paths must be extracted correctly."""

    def test_appstream_with_multiple_launchables(self):
        Path("foo.metainfo.xml").write_text(
            textwrap.dedent(
                """\
                <?xml version="1.0" encoding="UTF-8"?>
                <component>
                  <launchable type="desktop-id">
                    com.example.test-app1.desktop
                  </launchable>
                  <launchable type="test-wrong-type">
                    dummy
                  </launchable>
                  <launchable type="desktop-id">
                    com.example.test-app2.desktop
                  </launchable>
                  <launchable type="desktop-id">
                    unexisting
                  </launchable>
                </component>"""
            )
        )

        _create_desktop_file(
            "usr/local/share/applications/com.example.test/app1.desktop"
        )
        _create_desktop_file(
            "usr/local/share/applications/com.example.test/app2.desktop"
        )

        expected = [
            "usr/local/share/applications/com.example.test/app1.desktop",
            "usr/local/share/applications/com.example.test/app2.desktop",
        ]
        extracted = appstream.extract("foo.metainfo.xml", workdir=".")

        assert extracted is not None
        assert extracted.desktop_file_paths == expected
