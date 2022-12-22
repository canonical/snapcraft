# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_data_files, copy_metadata

block_cipher = None

data = [
        ("extensions", os.path.join("share", "snapcraft", "extensions")),
        ("keyrings", os.path.join("share", "snapcraft", "keyrings")),
        ("schema", os.path.join("share", "snapcraft", "schema")),
    ]
data += collect_data_files("launchpadlib")
data += collect_data_files("lazr.restfulclient")
data += collect_data_files("lazr.uri")
data += collect_data_files("wadllib")
data += copy_metadata("launchpadlib")
data += copy_metadata("lazr.restfulclient")
data += copy_metadata("lazr.uri")
data += copy_metadata("wadllib")

a = Analysis(
    ["snapcraft_legacy\\cli\\__main__.py"],
    pathex=[],
    binaries=[],
    datas=data,
    hiddenimports=[
        "cffi",
        "click",
        "configparser",
        "pkg_resources",
         # Workaround PyInstaller & SetupTools, https://github.com/pypa/setuptools/issues/1963
        "pkg_resources.py2_warn",
        "pymacaroons",
        "responses",
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="snapcraft",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    icon="windows/snapcraft.ico",
)
