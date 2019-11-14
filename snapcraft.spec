# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_data_files

block_cipher = None

datas = [
        ("extensions", os.path.join("share", "snapcraft", "extensions")),
        ("keyrings", os.path.join("share", "snapcraft", "keyrings")),
        ("schema", os.path.join("share", "snapcraft", "schema")),
    ]
datas += collect_data_files("lazr.restfulclient")
datas += collect_data_files("lazr.uri")
datas += collect_data_files("wadllib")

a = Analysis(
    ["snapcraft\\cli\\__main__.py"],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=[
        "cffi",
        "click",
        "configparser",
        "pkg_resources",
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
